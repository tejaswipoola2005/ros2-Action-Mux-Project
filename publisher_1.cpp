#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HighFreqPublisher : public rclcpp::Node {
public:
  HighFreqPublisher() : Node("high_freq_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("mux_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&HighFreqPublisher::publish_message, this)
    );
  }

private:
  void publish_message() {
    auto message = std_msgs::msg::String();
    message.data = "High Frequency Message";
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HighFreqPublisher>());
  rclcpp::shutdown();
  return 0;
}

/* Documentation
High Frequency Publisher Node (ROS 2)
=====================================

Description
--------------
This is a simple ROS 2 C++ node that publishes string messages at a high frequency (10 Hz) to a topic named `mux_topic`. It demonstrates the use of ROS 2 publishers and timers.

Dependencies
---------------
- rclcpp (ROS 2 Client Library for C++)
- std_msgs (Standard message types, specifically `std_msgs/msg/String`)

Code Structure
-----------------
- **Node Name:** high_freq_publisher
- **Topic Published:** mux_topic
- **Message Type:** std_msgs::msg::String
- **Publishing Rate:** 10 Hz (every 100 milliseconds)

How It Works
----------------
1. Initializes a publisher for the topic `mux_topic` with a queue size of 10.
2. Creates a wall timer that triggers every 100 milliseconds.
3. Each timer callback publishes a string message "High Frequency Message" and logs it.

*/