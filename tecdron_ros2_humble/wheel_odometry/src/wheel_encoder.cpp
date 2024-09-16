#include "custom_interfaces/msg/wheel_speed.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class WheelEncoder : public rclcpp::Node {
public:

  WheelEncoder() : Node("wheel_encoder") {
    publisher_ =
      this->create_publisher<custom_interfaces::msg::WheelSpeed>("wheel_speed", 10);

    timer_ = this->create_wall_timer(
        50ms, std::bind(&WheelEncoder::timer_callback, this));

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&WheelEncoder::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SIMULATED IDEAL ENCODER INITIALIZED");
  }

private:
  void timer_callback() {
    auto message = custom_interfaces::msg::WheelSpeed();
    
    message.wheel_speed = wheels_speed;

    publisher_->publish(message);
  }

  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    
    for(int i = 0; i < NUMBER_OF_WHEELS; i++){
      wheels_speed[i] = (msg->velocity[2*i]);
      // This is due to the fact that the joint states give also the visual wheels speed, but we need just the collision wheel speed
    }    

  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::WheelSpeed>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::vector<double> wheels_speed = {0, 0, 0, 0};
  const int NUMBER_OF_WHEELS = 4;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelEncoder>());
  rclcpp::shutdown();
  return 0;
}