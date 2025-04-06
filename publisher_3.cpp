#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LowFreqPublisher : public rclcpp::Node {
public:
  LowFreqPublisher() : Node("low_freq_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("mux_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),  // 2 Hz
      std::bind(&LowFreqPublisher::publish_message, this)
    );
  }

private:
  void publish_message() {
    auto message = std_msgs::msg::String();
    message.data = "Low Frequency Message";
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowFreqPublisher>());
  rclcpp::shutdown();
  return 0;
}

?
/*
Documentation
Low Frequency Publisher Node (ROS 2)
====================================

Description
--------------
This ROS 2 C++ node publishes string messages at a low frequency (2 Hz) to a topic named `mux_topic`. It demonstrates the use of a ROS 2 publisher with a slower publishing rate using a wall timer.

Dependencies
---------------
- rclcpp (ROS 2 Client Library for C++)
- std_msgs (Standard message types, specifically `std_msgs/msg/String`)

Code Structure
-----------------
- **Node Name:** low_freq_publisher
- **Topic Published:** mux_topic
- **Message Type:** std_msgs::msg::String
- **Publishing Rate:** 2 Hz (every 500 milliseconds)

How It Works
----------------
1. Initializes a publisher to publish messages to the `mux_topic` topic.
2. Sets up a timer that triggers every 500 milliseconds.
3. The timer callback publishes a string message with the content "Low Frequency Message" and logs it.


*/