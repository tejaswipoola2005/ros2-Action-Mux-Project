#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MediumFreqPublisher : public rclcpp::Node {
public:
  MediumFreqPublisher() : Node("medium_freq_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("mux_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),  // 5 Hz
      std::bind(&MediumFreqPublisher::publish_message, this)
    );
  }

private:
  void publish_message() {
    auto message = std_msgs::msg::String();
    message.data = "Medium Frequency Message";
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MediumFreqPublisher>());
  rclcpp::shutdown();
  return 0;
}

/*Documentation
Medium Frequency Publisher Node (ROS 2)
=======================================

Description
--------------
This is a ROS 2 C++ node that publishes string messages at a medium frequency (5 Hz) to a topic named `mux_topic`. It showcases how to create a ROS 2 publisher and use a wall timer for periodic message publishing.

 Dependencies
---------------
- rclcpp (ROS 2 Client Library for C++)
- std_msgs (Standard message types, specifically `std_msgs/msg/String`)

Code Structure
-----------------
- **Node Name:** medium_freq_publisher
- **Topic Published:** mux_topic
- **Message Type:** std_msgs::msg::String
- **Publishing Rate:** 5 Hz (every 200 milliseconds)

How It Works
----------------
1. Initializes a publisher to the topic `mux_topic` with a queue size of 10.
2. Sets up a wall timer to call a callback function every 200 milliseconds.
3. In the callback, a string message with the content "Medium Frequency Message" is created, published, and logged.


*/
