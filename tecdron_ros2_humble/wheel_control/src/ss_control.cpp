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
        A << -807.7,  222.7,    0,      0,       0,
             190.6, -531,  -13.15,    0,       0,
             0,  -9.577, -900.4,  234.2,     0,
             0,     0,    -69,   -364.2,  4.553,
             0,     0,     0,   0.9319,  -429.4;

        B << -1073,  -1684,  824.5,
             53.94,  897.4, -418.5,
             670.5,   449,  -195.4,
             576.1,  10.26, -30.86,
             502.8, -59.79, -2.678;

        C << 21.93,  -18.87,  144.4,  -100.7,  13.85,
             75.43,  157.8,  -14.99,  -84.24,  278.4,
            -294.9, -514.5,  -39.34, -383.5,  -58.98,
             389.3,  648.5,  171.8,   170.8,   375.7;

        D.setZero(); // Set all elements of D to zero
    }
  };


  void wheel_timer_callback() {
    auto message = std_msgs::msg::Float64MultiArray();
    /*std::vector<double> wheel_speed = {wheel_speed_left_front, wheel_speed_right_front, wheel_speed_left_back, wheel_speed_right_back};    
    wheel_speed.insert(wheel_speed.end(), wheel_speed.begin(), wheel_speed.end()); // Duplicate the wheel speeds to match the number of joints in the controller
    message.data = wheel_speed;
    wheel_publisher_->publish(message);*/
  }

  void controller_callback(){
    // Implementing controller
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> delta_time = now - last_time;
    
    error_k1 = commands - measured_states;
    i_error = i_error + (error_k1 - error_k)*delta_time.count();
  
    auto i_X_k1 = K.A*X_k + K.B*i_error;
    X_k1 = X_k + i_X_k1*delta_time.count();

    Y = K.C*X_k1;

    X_k = X_k1;
    error_k = error_k1;
    last_time = now;

    wheel_speed_left_front = Y(0);
    wheel_speed_right_front = Y(1);
    wheel_speed_right_back = Y(2);
    wheel_speed_left_back = Y(3);

    std::cout << "error: " << error_k1 << std::endl;
    //std::cout << "Wheel speeds: " << wheel_speed_left_front << " " << wheel_speed_right_front << " " << wheel_speed_right_back << " " << wheel_speed_left_back << std::endl;
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