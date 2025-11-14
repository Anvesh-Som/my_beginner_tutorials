// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @file subscriber_member_function.cpp
 * @brief Minimal ROS 2 subscriber that listens on `topic` and prints the
 * received string, demonstrating all logging levels via `_STREAM` macros.
 * 
 * @author Anvesh Som
 * @date 11 Nov 2025
 * @details
 * **Topics**
 * - Subscribes to `std_msgs/msg/String` on `topic`.
 *
 * **Logging**
 * - INFO on every message, DEBUG when count is a multiple of 7,
 *   WARN for long messages, ERROR if payload contains `"error"`,
 *   FATAL if payload equals `"FATAL"` (demo only).
 */
class MinimalSubscriber : public rclcpp::Node {
public:
  /**
   * @brief Construct the node and create the subscription.
   *
   * @note Emits an INFO message on startup.
   */
  MinimalSubscriber() : rclcpp::Node("minimal_subscriber"), received_(0) {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::OnMsg, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscriber started on 'topic'");
  }

private:
  /**
   * @brief Callback invoked on each received `std_msgs::msg::String`.
   *
   * @param msg The received message.
   *
   * @details
   * - Logs INFO for the payload, DEBUG on every 7th message,
   *   WARN if payload length exceeds 200 chars,
   *   ERROR if it contains `"error"`,
   *   FATAL if it equals `"FATAL"` (demo only).
   */

  void OnMsg(const std_msgs::msg::String &msg) {
    ++received_;
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.data << "'");

    if ((received_ % 7) == 0) {
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Received count multiple of 7: " << received_);
    }
    if (msg.data.size() > 200) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Long message (" << msg.data.size() << " chars).");
    }
    if (msg.data.find("error") != std::string::npos) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Detected 'error' token in message.");
    }
    if (msg.data == "FATAL") {
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "Upstream set base to 'FATAL' (demo FATAL level).");
    }

  }
  
  // ----------------------- Members -----------------------

  /**
   * @brief Subscription to `std_msgs::msg::String` on `topic`.
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  /**
   * @brief Count of messages received (used to trigger periodic DEBUG logs).
   */
  size_t received_;
};

/**
 * @brief Node entry point.
 *
 * @param argc Process argc from the shell.
 * @param argv Process argv from the shell.
 * @return `0` on normal shutdown.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
