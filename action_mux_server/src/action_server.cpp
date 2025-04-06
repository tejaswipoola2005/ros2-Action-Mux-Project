#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_mux_interfaces/action/mux_action.hpp"
#include "std_msgs/msg/string.hpp"

#include <unordered_map>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

class MuxActionServer : public rclcpp::Node {
public:
  using MuxAction = action_mux_interfaces::action::MuxAction;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MuxAction>;

  MuxActionServer() : Node("mux_action_server") {
    action_server_ = rclcpp_action::create_server<MuxAction>(
      this, "mux_action",
      std::bind(&MuxActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MuxActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MuxActionServer::handle_accepted, this, std::placeholders::_1)
    );

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "mux_topic", 10,
      std::bind(&MuxActionServer::message_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Mux Action Server Ready.");
  }

private:
  rclcpp_action::Server<MuxAction>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::mutex goal_mutex_;

  std::unordered_map<std::string, std::deque<rclcpp::Time>> publisher_timestamps_;
  const size_t window_size_ = 10;

  void message_callback(const std_msgs::msg::String::SharedPtr msg) {
    auto now = this->now();
    std::string publisher_id = msg->data;

    auto &timestamps = publisher_timestamps_[publisher_id];
    timestamps.push_back(now);
    if (timestamps.size() > window_size_)
      timestamps.pop_front();

    std::string highest_freq_publisher = get_highest_frequency_publisher();
    RCLCPP_INFO(this->get_logger(), "Highest frequency: %s", highest_freq_publisher.c_str());

    if (publisher_id == highest_freq_publisher) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (active_goal_) {
        RCLCPP_WARN(this->get_logger(), "Preempting current goal.");
        active_goal_->abort(std::make_shared<MuxAction::Result>());
        active_goal_.reset();
      }

      auto goal_msg = std::make_shared<MuxAction::Goal>();
      goal_msg->message = publisher_id;
      RCLCPP_INFO(this->get_logger(), "Accepted goal from: %s", publisher_id.c_str());

      // Simulate goal execution
      std::thread([this, goal_msg]() {
        rclcpp::sleep_for(1s);
        RCLCPP_INFO(this->get_logger(), "Processed goal from: %s", goal_msg->message.c_str());
      }).detach();
    }
  }

  std::string get_highest_frequency_publisher() {
    std::string max_id;
    double max_freq = 0.0;

    for (const auto &[id, timestamps] : publisher_timestamps_) {
      if (timestamps.size() < 2) continue;

      auto duration = (timestamps.back() - timestamps.front()).seconds();
      double freq = duration > 0 ? (timestamps.size() - 1) / duration : 0.0;

      if (freq > max_freq) {
        max_freq = freq;
        max_id = id;
      }
    }

    return max_id;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MuxAction::Goal> goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_ = goal_handle;
    }

    std::thread([this, goal_handle]() {
      auto feedback = std::make_shared<MuxAction::Feedback>();
      auto result = std::make_shared<MuxAction::Result>();
      for (int i = 0; i < 5; ++i) {
        rclcpp::sleep_for(500ms);
        feedback->progress = (i + 1) * 20.0;
        goal_handle->publish_feedback(feedback);
      }
      result->success = true;
      goal_handle->succeed(result);
    }).detach();
  }
};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MuxActionServer>());
  rclcpp::shutdown();
  return 0;
}

/*
Documentation
MuxActionServer Node Documentation (ROS 2)
==========================================

 Description
--------------
The `MuxActionServer` is a ROS 2 C++ node that subscribes to a shared topic (`mux_topic`) receiving string messages from multiple publishers. It identifies the publisher sending messages at the **highest frequency** and triggers an **action** goal for that publisher. If a new high-frequency publisher is detected, it preempts the previous goal.

 Dependencies
---------------
- `rclcpp`
- `rclcpp_action`
- `std_msgs/msg/String`
- `action_mux_interfaces/action/MuxAction` (Custom action)
- Standard C++ Libraries: `<unordered_map>`, `<chrono>`, `<deque>`, `<mutex>`, `<thread>`

 Code Structure
-----------------
- **Node Name:** `mux_action_server`
- **Subscribed Topic:** `mux_topic`
- **Action Name:** `mux_action`
- **Custom Action Type:** `MuxAction`
- **Functionality:** Goal preemption based on real-time publisher frequency detection

 Functional Summary
---------------------
- Subscribes to the topic `mux_topic` where each message's content indicates a publisher ID.
- Maintains a sliding time window of message timestamps for each publisher using a `std::deque`.
- Calculates message frequency per publisher.
- If a new publisher sends messages more frequently than the current one, the node **preempts the current goal**, aborts it, and starts a new goal.
- Action feedback is periodically published to simulate execution progress.
- Each action simulates processing using detached threads.


*/
