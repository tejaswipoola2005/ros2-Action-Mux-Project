#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <typeinfo>

class GenericSubscriber : public rclcpp::Node {
public:
  GenericSubscriber() : Node("generic_subscriber") {
    RCLCPP_INFO(this->get_logger(), "Generic Subscriber Ready.");
    RCLCPP_INFO(this->get_logger(), "Creating subscription on topic: mux_topic");
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "mux_topic", 10,
      std::bind(&GenericSubscriber::topic_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Generic Subscriber Ready.");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "Message type: %s", typeid(*msg).name());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GenericSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
Documentation
Generic Subscriber Node (ROS 2)
===============================

Description
--------------
This ROS 2 C++ node subscribes to the `mux_topic` topic and logs the received messages along with their data type. It demonstrates how to build a generic subscriber that listens to a topic and prints both content and type info using `typeid`.

Dependencies
---------------
- rclcpp (ROS 2 Client Library for C++)
- std_msgs (Standard message types, specifically `std_msgs/msg/String`)
- typeinfo (C++ standard library header for type introspection)

Code Structure
-----------------
- **Node Name:** generic_subscriber
- **Topic Subscribed:** mux_topic
- **Message Type:** std_msgs::msg::String

How It Works
----------------
1. Initializes a subscriber to the `mux_topic` topic with a queue size of 10.
2. Logs a message every time a message is received.
3. Additionally logs the message’s type using C++'s RTTI (`typeid`).

*/