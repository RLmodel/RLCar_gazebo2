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

#include <memory>
#include <boost/range/combine.hpp>

#include "rlcar_gazebo_odometry/rlcar_gazebo_odometry.hpp"
#include "rlcar_gazebo_odometry/odometry.hpp"

RLCarOdometry::RLCarOdometry() : Node("ackermann_odometry")
{
  // Setup Parameters
  verbose_ = declare_parameter("verbose", false);
  RCLCPP_INFO(get_logger(), "verbose : %s", verbose_ == true ? "true" : "false");

  auto publish_rate = declare_parameter("publish_rate", 50);
  RCLCPP_INFO(get_logger(), "publish_rate : %d", publish_rate);

  auto interval = std::chrono::duration<double>(1.0 / publish_rate);
  pub_timer_ = this->create_wall_timer(interval, std::bind(&RLCarOdometry::timerCallback, this));

  open_loop_ = declare_parameter("open_loop", false);
  RCLCPP_INFO(get_logger(), "open_loop_ : %s", open_loop_ == true ? "true" : "false");

  has_imu_heading_ = declare_parameter("has_imu_heading", true);
  RCLCPP_INFO(get_logger(), "has_imu_heading_ : %s", has_imu_heading_ == true ? "true" : "false");

  is_gazebo_ = declare_parameter("is_gazebo", false);
  RCLCPP_INFO(get_logger(), "is_gazebo_ : %s", is_gazebo_ == true ? "true" : "false");

  auto wheel_separation_h = declare_parameter("wheel_separation_h", 0.325);
  RCLCPP_INFO(get_logger(), "wheel_separation_h : %f", wheel_separation_h);

  auto wheel_separation_h_multiplier = declare_parameter("wheel_separation_h_multiplier", 1.1);
  RCLCPP_INFO(get_logger(), "wheel_separation_h_multiplier : %f", wheel_separation_h_multiplier);

  auto wheel_radius = declare_parameter("wheel_radius", 0.0508);
  RCLCPP_INFO(get_logger(), "wheel_radius : %f", wheel_radius);

  auto encoder_resolution = declare_parameter("encoder_resolution", 150);
  RCLCPP_INFO(get_logger(), "encoder_resolution : %d", encoder_resolution);

  // imu 쓴다면 사용하지는 않는 값임
  auto wheel_radius_multiplier = declare_parameter("wheel_radius_multiplier", 1.0);
  RCLCPP_INFO(get_logger(), "wheel_radius_multiplier : %f", wheel_radius_multiplier);

  // imu 쓴다면 사용하지는 않는 값임
  steer_pos_multiplier_ = declare_parameter("steer_pos_multiplier", 1.0);
  RCLCPP_INFO(get_logger(), "steer_pos_multiplier : %f", steer_pos_multiplier_);

  auto velocity_rolling_window_size = declare_parameter("velocity_rolling_window_size", 10);
  RCLCPP_INFO(get_logger(), "velocity_rolling_window_size : %f", velocity_rolling_window_size);

  odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  RCLCPP_INFO(get_logger(), "base_frame_id_ : %s", base_frame_id_.c_str());

  odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
  RCLCPP_INFO(get_logger(), "odom_frame_id_ : %s", odom_frame_id_.c_str());

  enable_odom_tf_ = declare_parameter("enable_odom_tf", true);
  RCLCPP_INFO(get_logger(), "enable_odom_tf_ : %s", enable_odom_tf_ == true ? "true" : "false");

  const double ws_h = wheel_separation_h_multiplier * wheel_separation_h;
  const double wr = wheel_radius_multiplier * wheel_radius;

  odometry_.setWheelParams(ws_h, wr, static_cast<uint>(encoder_resolution));

  RCLCPP_INFO(this->get_logger(), "Ackermann Odometry Node created");

  odom_pub_ = this->create_publisher<Odometry>("odom", rclcpp::QoS(1));

  imu_heading_pub_ = this->create_publisher<Float64>("imu_heading", 10);

  joint_state_sub_ = this->create_subscription<JointState>("joint_states", 10,
                                                           std::bind(&RLCarOdometry::jointstateCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<Twist>("cmd_vel", 10,
                                                  std::bind(&RLCarOdometry::cmdvelSubCallback, this, std::placeholders::_1));

  if (is_gazebo_ == true)
    clock_sub_ = this->create_subscription<Clock>(
        "clock", 10,
        [this](const Clock::SharedPtr msg) -> void
        {
          gazebo_clock_.clock = msg->clock;
        });
  else
    encoder_sub_ = this->create_subscription<Int64>(
        "encoder_value", rclcpp::SensorDataQoS(),
        [this](const Int64::SharedPtr msg) -> void
        {
          encoder_pos_ = msg->data;
        });

  if (has_imu_heading_ == true)
    imu_sub_ = this->create_subscription<Imu>(
        "imu/data", 10,
        std::bind(&RLCarOdometry::imuSubCallback, this, std::placeholders::_1));
  else
    steering_angle_sub_ = this->create_subscription<Float64>(
        "steering_angle_middle", 10,
        std::bind(&RLCarOdometry::steeringAngleSubCallback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  this->starting();
}

void RLCarOdometry::starting()
{

  rclcpp::Time time = this->get_clock()->now();

  last_state_publish_time_ = time;

  odometry_.init(time);
}

void RLCarOdometry::jointstateCallback(const JointState::SharedPtr msg)
{
  for (auto i = 0; i < 6; i++)
  {
    if (msg->position[i] == 0)
      break;

    if (strcmp(msg->name[i].c_str(), "lr_wheel_joint") == 0)
      left_rear_wheel_joint = msg->position[i];
    if (strcmp(msg->name[i].c_str(), "rr_wheel_joint") == 0)
      right_rear_wheel_joint = msg->position[i];
  }

  rear_wheel_pos = (left_rear_wheel_joint + right_rear_wheel_joint) / 2;
  if (verbose_)
    RCLCPP_INFO(this->get_logger(), "Rear Wheel Pose : %lf", rear_wheel_pos);
}

void RLCarOdometry::steeringAngleSubCallback(const Float64::SharedPtr msg)
{

  front_hinge_pos = msg->data;

  if (verbose_)
    RCLCPP_INFO(this->get_logger(), "Front Hinge Pose : %f", front_hinge_pos);
}

void RLCarOdometry::cmdvelSubCallback(const Twist::SharedPtr msg)
{
  linear_x = msg->linear.x;
  angular_z = msg->angular.z;
}

void RLCarOdometry::imuSubCallback(const Imu::SharedPtr msg)
{
  tf2::Quaternion q_;

  q_[0] = msg->orientation.x;
  q_[1] = msg->orientation.y;
  q_[2] = msg->orientation.z;
  q_[3] = msg->orientation.w;

  tf2::Matrix3x3 m(q_);

  double roll, pitch;
  m.getRPY(roll, pitch, heading_angle);

  Float64 heading_msg;
  heading_msg.data = heading_angle;

  imu_heading_pub_->publish(heading_msg);

  if (verbose_)
    RCLCPP_INFO(this->get_logger(), "yaw : %f", heading_angle);
}

void RLCarOdometry::odomUpdate(const rclcpp::Time &time)
{
  // COMPUTE AND PUBLISH ODOMETRY
  // TODO : open_loop implement & comparison
  if (open_loop_)
    odometry_.updateOpenLoop(linear_x, angular_z, time);
  else
  {
    if (std::isnan(rear_wheel_pos) || std::isnan(front_hinge_pos))
      return;

    // Estimate linear and angular velocity using joint information
    if (has_imu_heading_)
      if (is_gazebo_)
        odometry_.updateWithHeading(rear_wheel_pos, heading_angle, time);
      else
        odometry_.updateWithHeading(encoder_pos_, heading_angle, time);
    else
    {
      front_hinge_pos *= steer_pos_multiplier_;
      odometry_.update(rear_wheel_pos, front_hinge_pos, time);
    }
  }
}

void RLCarOdometry::publishOdomTopic(const rclcpp::Time &time)
{
  // Publish odometry message
  // Compute and store orientation info
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odometry_.getHeading());

  // Populate odom message and publish
  auto odom = std::make_unique<Odometry>();

  if(is_gazebo_ == true)
    odom->header.stamp = gazebo_clock_.clock;
  else
    odom->header.stamp = time;

  odom->header.frame_id = odom_frame_id_;
  odom->child_frame_id = base_frame_id_;

  odom->pose.pose.position.x = odometry_.getX();
  odom->pose.pose.position.y = odometry_.getY();
  
  odom->pose.pose.orientation.x = q.x();
  odom->pose.pose.orientation.y = q.y();
  odom->pose.pose.orientation.z = q.z();
  odom->pose.pose.orientation.w = q.w();
  
  odom->pose.covariance.fill(0.0);
  // x y z roll pitch yaw
  odom->pose.covariance[0] = 1e-3;
  odom->pose.covariance[7] = 1e-3;
  odom->pose.covariance[14] = 1e6;
  odom->pose.covariance[21] = 1e6;
  odom->pose.covariance[28] = 1e6;
  odom->pose.covariance[35] = 1e-3;

  odom->twist.twist.linear.x = odometry_.getLinear();
  odom->twist.twist.angular.z = odometry_.getAngular();

  odom->twist.covariance.fill(0.0);
  odom->twist.covariance[0] = 1e-3;
  odom->twist.covariance[7] = 1e-3;
  odom->twist.covariance[14] = 1e6;
  odom->twist.covariance[21] = 1e6;
  odom->twist.covariance[28] = 1e6;
  odom->twist.covariance[35] = 1e3;

  if (enable_odom_tf_)
  {
    // publish TF
    geometry_msgs::msg::TransformStamped odom_tf;
   
    if(is_gazebo_ == true)
      odom_tf.header.stamp = gazebo_clock_.clock;
    else
      odom_tf.header.stamp = time;
    
    odom_tf.header.frame_id = odom_frame_id_;
    odom_tf.child_frame_id = base_frame_id_;
    odom_tf.transform.translation.x = odometry_.getX();
    odom_tf.transform.translation.y = odometry_.getY();
    odom_tf.transform.translation.z = 0;
    odom_tf.transform.rotation = odom->pose.pose.orientation;

    tf_broadcaster_->sendTransform(odom_tf);
  }

  odom_pub_->publish(std::move(odom));
}

void RLCarOdometry::timerCallback()
{
  rclcpp::Time cur_time = this->get_clock()->now();

  odomUpdate(cur_time);
  publishOdomTopic(cur_time);
}

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RLCarOdometry>());
  rclcpp::shutdown();

  return 0;
}
