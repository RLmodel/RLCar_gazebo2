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

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>

#include "rlcar_gazebo_odometry/odometry.hpp"

using String = std_msgs::msg::String;
using Float64 = std_msgs::msg::Float64;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using JointState = sensor_msgs::msg::JointState;
using Imu = sensor_msgs::msg::Imu;
using Int64 = std_msgs::msg::Int64;
using Clock = rosgraph_msgs::msg::Clock;

class RLCarOdometry: public rclcpp::Node {
public:
    RLCarOdometry();

    void update(const rclcpp::Time& time, const rclcpp::Duration& period);
    void starting();
    void stopping(const rclcpp::Time& /*time*/);

    void jointstateCallback(const JointState::SharedPtr msg);
    void steeringAngleSubCallback(const Float64::SharedPtr msg);
    void cmdvelSubCallback(const Twist::SharedPtr msg);
    void imuSubCallback(const Imu::SharedPtr msg);

    void odomUpdate(const rclcpp::Time &time);
    void publishOdomTopic(const rclcpp::Time &time);
    void timerCallback();

private:
    ackermann_steering_controller::Odometry odometry_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<Float64>::SharedPtr imu_heading_pub_;

    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<Float64>::SharedPtr steering_angle_sub_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<Int64>::SharedPtr encoder_sub_;
    rclcpp::Subscription<Clock>::SharedPtr clock_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool verbose_;

    /// Odometry related:
    rclcpp::Time last_state_publish_time_;
    Clock gazebo_clock_;

    /// use open loop odom or not
    bool open_loop_;

    /// if imu topic exists, it'll be better to use it during odom calculation 
    /// most ackermann robots has wheel slip
    bool has_imu_heading_;

    bool is_gazebo_;
    
    /// (usefull only has_imu_heading_ is activated) get raw heading angle from imu 
    double heading_angle;

    /// Wheel steer calibration multipliers:
    double steer_pos_multiplier_;

    // open loop variables
    double linear_x, angular_z;
    
    // Mean of two rear wheel pose (radian)
    double rear_wheel_pos;
    double left_rear_wheel_joint, right_rear_wheel_joint;

    // real robot only, rear wheel encoder position
    int64_t encoder_pos_;

    // Front wheel steering pose (radian)
    float front_hinge_pos;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Whether to allow multiple publishers on cmd_vel topic or not:
    bool allow_multiple_cmd_vel_publishers_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Frame to use for odometry and odom tf:
    std::string odom_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Number of wheel joints:
    size_t wheel_joints_size_;

    /// Number of steer joints:
    size_t steer_joints_size_;

    /// Speed limiters:
    // Commands last1_cmd_;
    // Commands last0_cmd_;
};