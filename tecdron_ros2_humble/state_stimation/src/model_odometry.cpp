#include "custom_interfaces/msg/wheel_speed.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class OdomModel : public rclcpp::Node {
public:

  OdomModel() : Node("odom_from_model") {
    odom_publisher_ =
      this->create_publisher<nav_msgs::msg::Odometry>("odom_model", 10);

    odom_timer_ = this->create_wall_timer(
        25ms, std::bind(&OdomModel::odom_timer_callback, this));

    subscription_ = this->create_subscription<custom_interfaces::msg::WheelSpeed>(
        "wheel_speed", 10,
        std::bind(&OdomModel::subscription_callback, this, std::placeholders::_1));
    
    last_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "IDEAL ODOMETRY INITIALIZED");
  }

private:
  void odom_timer_callback() {
    auto message = nav_msgs::msg::Odometry();
    
    InverseDynamics();
    yawToQuaternion(yaw);
    Update();

    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation.w = q.w;
    message.pose.pose.orientation.x = q.x;
    message.pose.pose.orientation.y = q.y;
    message.pose.pose.orientation.z = q.z;

    message.twist.twist.linear.x = vx;
    message.twist.twist.linear.y = vy;
    message.twist.twist.linear.z = 0.0;  // Assuming planar motion
    message.twist.twist.angular.x = 0.0;  // No roll
    message.twist.twist.angular.y = 0.0;  // No pitch
    message.twist.twist.angular.z = wz;   // Yaw angular velocity

    message.header.stamp = this->get_clock()->now();  // Use current time
    //message.header.frame_id = "odom";  // The reference frame for pose
    //message.child_frame_id = "base_link";  // The frame for velocity


    odom_publisher_->publish(message);
  }

  void subscription_callback(const custom_interfaces::msg::WheelSpeed::SharedPtr msg) {
    
    this->wheel_speed = msg->wheel_speed;
  }

  void InverseDynamics()
  {
    vx = R * (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / 4;
    vy = R * (-wheel_speed[0] + wheel_speed[1] + wheel_speed[2] - wheel_speed[3]) / 4;
    wz = R * (-wheel_speed[0] + wheel_speed[1] - wheel_speed[2] + wheel_speed[3]) / (4 * (L + W));
  }

  void yawToQuaternion(double yaw) {
    
    q.w = cos(yaw / 2.0);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw / 2.0);
  }

  void Update()
  {
    auto current_time = std::chrono::steady_clock::now();

    std::chrono::duration<double> difftime = current_time - last_time_;
    double dt = difftime.count();

    x = last_x + (vx * cos(last_yaw) - vy * sin(last_yaw))*dt;
    y = last_y + (vx * sin(last_yaw) + vy * cos(last_yaw))*dt;
    yaw = last_yaw + wz*dt;
    last_x = x;
    last_y = y;
    last_yaw = yaw;
    last_time_ = current_time;
  }

  struct Quaternion {
    double w, x, y, z;
  };

  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<custom_interfaces::msg::WheelSpeed>::SharedPtr subscription_;
  std::chrono::steady_clock::time_point last_time_;
  std::vector<double> wheel_speed = {0, 0, 0, 0};
  double vx = 0;
  double vy = 0;
  double wz = 0;
  double x = 0;
  double last_x = 0;
  double y = 0;
  double last_y = 0; 
  double yaw = 0;
  double last_yaw = 0;
  Quaternion q;
  const double R = 0.127;
  const double L = 0.35;
  const double W = 0.20;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomModel>());
  rclcpp::shutdown();
  return 0;
}