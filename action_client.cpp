#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_mux_interfaces/action/mux_action.hpp"
#include "std_msgs/msg/string.hpp"

class ActionClientNode : public rclcpp::Node {
public:
  using MuxAction = action_mux_interfaces::action::MuxAction;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MuxAction>;

  ActionClientNode(const std::string &node_name, const std::string &id)
  : Node(node_name), client_id_(id) {
    client_ = rclcpp_action::create_client<MuxAction>(this, "mux_action");

    publisher_ = this->create_publisher<std_msgs::msg::String>("mux_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // You can vary this across clients
      std::bind(&ActionClientNode::publish_message, this)
    );
  }

  void publish_message() {
    std_msgs::msg::String msg;
    msg.data = client_id_;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[%s] Publishing: %s", this->get_name(), msg.data.c_str());
  }

private:
  std::string client_id_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<MuxAction>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionClientNode>("mux_client_1", "client_1");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
Documentation
ActionClientNode Documentation (ROS 2)
======================================

📄 Description
--------------
`ActionClientNode` is a ROS 2 C++ node designed to:
1. Periodically **publish messages** on a shared topic (`mux_topic`) identifying itself by a unique `client_id_`.
2. Simultaneously act as an **action client** for a custom action called `MuxAction`, prepared to communicate with an `action_server`.

It is typically used in coordination with the `MuxActionServer` node, which selects the most frequent publisher and initiates a goal based on that publisher.

📦 Dependencies
---------------
- `rclcpp`
- `rclcpp_action`
- `std_msgs/msg/String`
- `action_mux_interfaces/action/MuxAction` (Custom action interface)

🧱 Node Configuration
---------------------
- **Node Name (Example):** `"mux_client_1"`
- **Client ID (Sent in message):** `"client_1"`
- **Topic Published:** `"mux_topic"`
- **Message Type:** `std_msgs::msg::String`
- **Action Used:** `"mux_action"`

🧠 Functional Overview
----------------------
- This node acts as a **simulated client** by publishing a string message (its unique ID) to a topic at regular intervals (e.g., 100 ms).
- The message represents this client's identity to the MuxActionServer, which uses it to determine publishing frequency and potentially trigger an action goal.
- An **action client** is also instantiated, ready to communicate with the action server.


*/