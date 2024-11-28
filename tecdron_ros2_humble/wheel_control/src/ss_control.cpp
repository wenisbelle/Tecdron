#include "geometry_msgs/msg/twist.hpp"
#include "rcl/node_options.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"


#include <Eigen/Dense>
#include <Eigen/Core>
#include <array>

#include <chrono>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;


/************************************
*************** CLASS ***************
*************************************/

class SS_Control : public rclcpp::Node {
public:

  explicit SS_Control(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("SS_Control", node_options)
      {
    
    // Esse é o meu construtor posso inicializar as variaveis aqui

    // Initialize the MutuallyExclusive callback group object
    callback_publisher_wheel_velocities_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_controller_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_subscriber_cmd_vel_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        
    callback_subscriber_odometry_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_cmd_vel;
    options_cmd_vel.callback_group = callback_subscriber_cmd_vel_group_;

    rclcpp::SubscriptionOptions options_odometry;
    options_odometry.callback_group = callback_subscriber_odometry_group_; 

    wheel_publisher_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_forward_controller/commands", 10);

    publisher_timer_ = this->create_wall_timer(
        5ms, std::bind(&SS_Control::wheel_timer_callback, this),
        callback_publisher_wheel_velocities_group_); //

    controller_timer_ = this->create_wall_timer(
        5ms, std::bind(&SS_Control::controller_callback, this),
        callback_controller_group); // 

    subscription_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&SS_Control::subs_cmd_vel_callback, this, std::placeholders::_1),
        options_cmd_vel);

    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "true_odom", 10,
        std::bind(&SS_Control::odometry_callback, this, std::placeholders::_1),
        options_odometry);

    last_time = std::chrono::high_resolution_clock::now();
  }

private:
    // FAZER O CÓDIGO RECEBER OS VALORES DE A, B, C E D. Deixa ele mais fácil de ser alterado. 
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
  };


  void wheel_timer_callback() {
    auto message = std_msgs::msg::Float64MultiArray();
    std::vector<double> wheel_speed = {wheel_speed_left_front, wheel_speed_right_front, wheel_speed_left_back, wheel_speed_right_back};    
    wheel_speed.insert(wheel_speed.end(), wheel_speed.begin(), wheel_speed.end()); // Duplicate the wheel speeds to match the number of joints in the controller
    message.data = wheel_speed;
    wheel_publisher_->publish(message);
  }

  void controller_callback(){
    // Implementing controller
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> delta_time = now - last_time;
    
    error_k1 = commands - measured_states;
    i_error = i_error + (error_k1)*delta_time.count();
  
    X_k1 = K.A*X_k + K.B*i_error;

    Y = K.C*X_k1 + K.D*i_error;

    X_k = X_k1;
    error_k = error_k1;
    last_time = now;

    for (int i = 0; i < Y.rows(); i++)
    {
      if (Y(i) > 100)
      {
        Y(i) = 100;
      }
      else if (Y(i) < -100)
      {
        Y(i) = -100;
      }
    }
    

    wheel_speed_left_front = Y(0);
    wheel_speed_right_front = Y(1);
    wheel_speed_right_back = Y(2);
    wheel_speed_left_back = Y(3);

    std::cout << "error: " << error_k1 << std::endl;
    //std::cout << "Wheel speeds: " << wheel_speed_left_front << " " << wheel_speed_right_front << " " << wheel_speed_right_back << " " << wheel_speed_left_back << std::endl;
    //std::cout << "Matrix" << Y << std::endl;
  }

 
  void subs_cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    commands << msg->linear.x,
                msg->linear.y,
                msg->angular.z;

  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
   measured_states << odom->twist.twist.linear.x,
             odom->twist.twist.linear.y,
             odom->twist.twist.angular.z;
  }
 
  Eigen::Matrix<float, 3, 1> measured_states;
  Eigen::Matrix<float, 3, 1> commands;
  Eigen::Matrix<float, 3, 1> i_error = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> error_k = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> error_k1;
  Eigen::Matrix<float, 5, 1> X_k = Eigen::Matrix<float, 5, 1>::Zero();
  Eigen::Matrix<float, 5, 1> X_k1;
  Eigen::Matrix<float, 4, 1> Y;
  Controller K;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_time;

  // ROS variables
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;
  rclcpp::CallbackGroup::SharedPtr callback_publisher_wheel_velocities_group_;
  rclcpp::CallbackGroup::SharedPtr callback_controller_group;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_cmd_vel_group_;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_odometry_group_;

  double wheel_speed_left_front;
  double wheel_speed_right_front;
  double wheel_speed_left_back;
  double wheel_speed_right_back;
};



/************************************
************** MAIN *****************
*************************************/

int main(int argc, char *argv[]) {
  
  rclcpp::init(argc, argv);

  std::shared_ptr<SS_Control> SS_Robust_Controller =
      std::make_shared<SS_Control>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(SS_Robust_Controller);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}