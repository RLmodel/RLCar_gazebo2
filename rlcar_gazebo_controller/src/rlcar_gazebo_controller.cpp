// Copyright 2023 @RoadBalance
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <math.h>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using Twist = geometry_msgs::msg::Twist;
using Float64 = std_msgs::msg::Float64;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

template <typename T>
inline T round_digit(T num, int d) {
	T t = pow(10, d - 1);
	return round(num * t) / t;
}

class RLCarGazeboController : public rclcpp::Node {
private:
  bool verbose = false;

  double L = 0.276;
  double T = 0.1734;
  double maxsteer_inside = 0.7853;
  double wheel_radius = 0.05;
  double max_wheel_turn_speed = 167;

  double R_Min_interior;
  double R_Min_baselink;

  double steering_radius = 0.0;
  float turning_sign = 0.0;
  float linear_sign = 0.0;
  double omega_turning_speed = 0.0;

  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  
  rclcpp::Publisher<Float64MultiArray>::SharedPtr steering_pub;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr throttling_pub;

  rclcpp::Publisher<Float64>::SharedPtr steering_pub_middle;
  rclcpp::Publisher<Float64>::SharedPtr throttling_pub_middle;

  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub;

  Float64MultiArray steering_msg;
  Float64MultiArray throttling_msg;
  Float64 steering_msg_middle;
  Float64 throttling_msg_middle;

  void cmd_vel_callback(const Twist::SharedPtr data) {
    linear_velocity = data->linear.x;
    angular_velocity = data->angular.z;

    process_cmd_vel_data();
    publish();
  }

  void process_cmd_vel_data(){

    if(verbose){
      RCLCPP_INFO(this->get_logger(), "linear_velocity=%f", linear_velocity);
      RCLCPP_INFO(this->get_logger(), "angular_velocity=%f", angular_velocity);
    }

    if(angular_velocity != 0.0){
      double steering_radius_raw = abs(linear_velocity / angular_velocity);
      steering_radius = std::max(abs(steering_radius_raw), R_Min_baselink);
      turning_sign = -1 * copysign(1, angular_velocity);
      linear_sign = copysign(1, linear_velocity);
      omega_turning_speed = linear_velocity / steering_radius;
    } else {
      steering_radius = -1;
      turning_sign = 0.0;
      linear_sign = 0.0;
      omega_turning_speed = 0.0;
    }
  }

  void publish(){

    double turning_radius_right_rear_wheel = 0.0;
    double turning_radius_left_rear_wheel = 0.0;

    double wheel_turnig_speed_left_rear_wheel;
    double wheel_turnig_speed_right_rear_wheel;
    double wheel_turnig_speed_left_front_wheel;
    double wheel_turnig_speed_right_front_wheel;

    double wheel_turnig_speed_com;
    double alfa_right_front_wheel;
    double alfa_left_front_wheel;

    if(steering_radius >= 0){
      turning_radius_right_rear_wheel = steering_radius + 
        (-1 * turning_sign * linear_sign) * (T / 2.0);
      auto vel_right_rear_wheel = omega_turning_speed * turning_radius_right_rear_wheel;
      wheel_turnig_speed_right_rear_wheel = limit_wheel_speed(vel_right_rear_wheel / wheel_radius);

      turning_radius_left_rear_wheel = steering_radius + 
        (1 * turning_sign * linear_sign) * (T / 2.0);
      auto vel_left_rear_wheel = omega_turning_speed * turning_radius_left_rear_wheel;
      wheel_turnig_speed_left_rear_wheel = limit_wheel_speed(vel_left_rear_wheel / wheel_radius);

      // turning_radius_com = 
      auto vel_com = omega_turning_speed * steering_radius;
      wheel_turnig_speed_com = limit_wheel_speed(vel_com / wheel_radius);
    } else {
      wheel_turnig_speed_right_rear_wheel = limit_wheel_speed(linear_velocity / wheel_radius);
      wheel_turnig_speed_left_rear_wheel = limit_wheel_speed(linear_velocity / wheel_radius); 
      wheel_turnig_speed_com = limit_wheel_speed(linear_velocity / wheel_radius);
    }

    if(steering_radius >= 0 && linear_velocity != 0){
      auto distance_to_turning_point_right_front_wheel = sqrt(
          pow(L, 2) + pow(turning_radius_right_rear_wheel, 2)
        );

      wheel_turnig_speed_right_front_wheel = limit_wheel_speed(
        (omega_turning_speed * distance_to_turning_point_right_front_wheel) / wheel_radius
      );

      alfa_right_front_wheel = asin((omega_turning_speed * L ) / linear_velocity);
      auto distance_to_turning_point_left_front_wheel = sqrt(
          pow(L, 2) + pow(turning_radius_left_rear_wheel, 2)
        );
      
      wheel_turnig_speed_left_front_wheel = limit_wheel_speed(
        (omega_turning_speed * distance_to_turning_point_left_front_wheel) / wheel_radius
      );

      alfa_left_front_wheel = asin((omega_turning_speed * L ) / linear_velocity);
    } else {
      wheel_turnig_speed_right_front_wheel = limit_wheel_speed(linear_velocity / wheel_radius);
      alfa_right_front_wheel = 0.0;

      wheel_turnig_speed_left_front_wheel = limit_wheel_speed(linear_velocity / wheel_radius);
      alfa_left_front_wheel = 0.0;
    }

    auto right_steering = -1 * turning_sign * linear_sign * alfa_right_front_wheel;
    auto left_steering = -1 * turning_sign * linear_sign * alfa_left_front_wheel;
    steering_msg.data.push_back(right_steering);
    steering_msg.data.push_back(left_steering);

    throttling_msg.data.push_back(wheel_turnig_speed_left_rear_wheel);
    throttling_msg.data.push_back(wheel_turnig_speed_right_rear_wheel);
    throttling_msg.data.push_back(wheel_turnig_speed_left_front_wheel);
    throttling_msg.data.push_back(wheel_turnig_speed_right_front_wheel);

    steering_msg_middle.data = (round_digit(right_steering, 5) + round_digit(left_steering, 5)) / 2;
    throttling_msg_middle.data = wheel_turnig_speed_com;

    steering_pub->publish(steering_msg);
    steering_pub_middle->publish(steering_msg_middle);
    throttling_pub->publish(throttling_msg);
    throttling_pub_middle->publish(throttling_msg_middle);

    steering_msg.data.clear();
    throttling_msg.data.clear();
  }

  template <typename T>
  T limit_wheel_speed(T in_speed){
    if (in_speed > max_wheel_turn_speed)
      in_speed = max_wheel_turn_speed;
    else if (in_speed < -1.0 * max_wheel_turn_speed)
      in_speed = -1.0 * max_wheel_turn_speed;
    
    return in_speed;
  }

public:
  RLCarGazeboController() : Node("rlcar_gazebo_controller") {
    R_Min_interior = L / tan(maxsteer_inside);
    R_Min_baselink = R_Min_interior + (T / 2.0);
    RCLCPP_INFO(this->get_logger(), "MINIMUM TURNING RADIUS ACKERMAN : %f", R_Min_baselink);

    cmd_vel_sub = create_subscription<Twist>(
      "cmd_vel", 10,
      std::bind(&RLCarGazeboController::cmd_vel_callback, this, std::placeholders::_1)
    );

    steering_pub = create_publisher<Float64MultiArray>("forward_position_controller/commands", 10);
    throttling_pub = create_publisher<Float64MultiArray>("velocity_controller/commands", 10);
    steering_pub_middle = create_publisher<Float64>("steering_angle_middle", 10);
    throttling_pub_middle = create_publisher<Float64>("throttling_vel_middle", 10);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto rlcar_gazebo_controller = std::make_shared<RLCarGazeboController>();

  rclcpp::spin(rlcar_gazebo_controller);
  rclcpp::shutdown();

  return 0;
}
