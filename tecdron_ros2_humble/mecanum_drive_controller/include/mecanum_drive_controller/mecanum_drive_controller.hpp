// Copyright 2020 PAL Robotics S.L.
// Copyright 2022 Husarion
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

// Copied and adapted from diff_drive_controller (https://github.com/ros-controls/ros2_controllers)
// Author: Maciej Stępień

#ifndef MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
#define MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <array>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <thread>

#include "controller_interface/controller_interface.hpp"
#include "mecanum_drive_controller/odometry.hpp"
#include "mecanum_drive_controller/speed_limiter.hpp"
#include "mecanum_drive_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace mecanum_drive_controller
{
class MecanumDriveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  MecanumDriveController();

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

protected:
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  const char* feedback_type() const;
  controller_interface::CallbackReturn configure_wheel(const std::string& wheel_name,
                                                       std::unique_ptr<WheelHandle>& registered_handle);

  std::string front_left_wheel_name_;
  std::string front_right_wheel_name_;
  std::string rear_left_wheel_name_;
  std::string rear_right_wheel_name_;

  std::string filtered_odometry_topic_;

  std::unique_ptr<WheelHandle> registered_front_left_wheel_handle_;
  std::unique_ptr<WheelHandle> registered_front_right_wheel_handle_;
  std::unique_ptr<WheelHandle> registered_rear_left_wheel_handle_;
  std::unique_ptr<WheelHandle> registered_rear_right_wheel_handle_;

  struct WheelParams
  {
    double separation_x = 0.0;  // w.r.t. the midpoint of the wheel width
    double separation_y = 0.0;  // w.r.t. the midpoint of the wheel width
    double radius = 0.0;        // Assumed to be the same for both wheels
    double separation_x_multiplier = 1.0;
    double separation_y_multiplier = 1.0;
    double radius_multiplier = 1.0;
  } wheel_params_;

  struct Controller{
    Eigen::Matrix<float, 5, 5> A;
    Eigen::Matrix<float, 5, 3> B;
    Eigen::Matrix<float, 4, 5> C;
    Eigen::Matrix<float, 4, 3> D;

    Controller() {
        A << -0.3116,  0.1647,  -0.0016,  -0.0005,  -0.0000,
          0.1409, -0.1069,  -0.0089,  -0.0027,  -0.0000,
         -0.0010, -0.0065,  -0.3946,   0.1855,   0.0010,
          0.0001,  0.0006,  -0.0547,   0.0301,   0.0057,
          0.0000,  0.0000,  -0.0001,   0.0012,  -0.0354;

        B << -1.8280,  -2.5304,   1.2475,
          -0.2764,   1.4002,  -0.6393,
           1.2851,   0.6740,  -0.3055,
           1.3989,  -0.0349,  -0.0532,
           1.2142,  -0.1442,  -0.0065;

        C <<  6.1391,  -7.1188,   46.5238,  -38.4334,    6.4692,
          37.0834,  76.6929,   -3.0061,  -44.8515,  134.0147,
        -137.7471, -254.0060,    1.1048, -200.4365,  -29.5463,
         179.6186,  321.1417,   44.1334,  103.1433,  181.7447;

        D << 13.3333,   8.4611,   0.2921,
           9.6942,  -8.6161,   0.2352,
          11.3076,  10.5586,  -3.0698,
          12.4502, -10.6754,   3.5251;

    }
  } K;

  Eigen::Matrix<float, 3, 1> measured_states;
  Eigen::Matrix<float, 3, 1> commands;
  Eigen::Matrix<float, 3, 1> i_error = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> error_k = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> error_k1;
  Eigen::Matrix<float, 5, 1> X_k = Eigen::Matrix<float, 5, 1>::Zero();
  Eigen::Matrix<float, 5, 1> X_k1;
  Eigen::Matrix<float, 4, 1> Y;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
  std::chrono::duration<double> delta_time;

  struct OdometryParams
  {
    bool open_loop = false;
    bool position_feedback = true;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_ =
      nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{ 500 };

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{ nullptr };

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odometry_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<nav_msgs::msg::Odometry>> received_filtered_msg_ptr_{ nullptr };
  

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_x_;
  SpeedLimiter limiter_linear_y_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ = nullptr;

  rclcpp::Time previous_update_timestamp_{ 0 };

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{ 0 };

  bool is_halted = false;
  bool use_stamped_vel_ = true;
  

  bool reset();
  void halt();
};
}  // namespace mecanum_drive_controller
#endif  // MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
