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

/**
 * @file
 * @brief Minimal ROS 2 publisher that exposes a service to change the base
 * string and publishes at a parameterized rate. Demonstrates all logging levels
 * using the `_STREAM` macros.
 *
 * @details
 * **Parameters**
 * - `base_string` (`std::string`, default: `"UMD Robotics says hi!"`) — Base
 * text prefix.
 * - `publish_frequency_hz` (`double`, default: `2.0`) — Publication frequency
 * in Hz.
 *
 * **Topics**
 * - Publishes `std_msgs/msg/String` on `topic`.
 *
 * **Services**
 * - `/set_base_string` (`beginner_tutorials/srv/SetBaseString`):
 *   - Request: `string base`
 *   - Response: `bool success`, `string message`
 *
 * **Logging**
 * - Uses DEBUG/INFO/WARN/ERROR/FATAL via `_STREAM` macros in timer and service
 * paths.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
/**
 * @file publisher_member_function.cpp
 * @brief Defines a publisher with custom message
 * @author Anvesh Som
 * @date 11 Nov 2025
 * @class TalkerWithService
 * * @details
 * - Declares `base_string` and `publish_frequency_hz` as runtime parameters.
 * - Publishes `std_msgs::msg::String` on `topic` at the configured rate.
 * - `/set_base_string` updates the internal base string with simple validation.
 * - Emits illustrative log lines at all severities.
 */
class MinimalPublisher : public rclcpp::Node {
public:
  /**
   * @brief Construct the node, declare parameters, configure timer, create
   * publisher and service.
   *
   * @note If `publish_frequency_hz <= 0`, the node clamps to 1.0 Hz and emits a
   * WARN message.
   */
  TalkerWithService() : rclcpp::Node("minimal_publisher"), count_(0) {
    // Declare parameters (can be set via launch or CLI).
    base_ = this->declare_parameter<std::string>("base_string",
                                                 "UMD Robotics says hi!");
    const double hz =
    this->declare_parameter<double>("publish_frequency_hz", 2.0);

    // Validate frequency.
    if (hz <= 0.0) {
    RCLCPP_WARN_STREAM(this->get_logger(),
    "publish_frequency_hz <= 0; clamping to 1.0 Hz");
    timer_period_ = 1000ms;
    } else {
    timer_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / hz));
    }

    // Publisher to "topic".
    pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Service to update base string.
    srv_ = this->create_service<SetBaseString>(
    "set_base_string",
    std::bind(&TalkerWithService::OnSetBaseString, this,
    std::placeholders::_1, std::placeholders::_2));

    // Wall timer controls publication cadence.
    timer_ = this->create_wall_timer(
    timer_period_, std::bind(&TalkerWithService::OnTimer, this));

    RCLCPP_INFO_STREAM(this->get_logger(),
    "TalkerWithService started. base_string='"
    << base_ << "', freq="
    << (1000.0 / timer_period_.count()) << " Hz");
  }

private:
  /**
   * @brief Timer callback that publishes a message and emits illustrative logs.
   *
   * @details
   * - Message layout: `<base_> seq=<count_>`.
   * - DEBUG every 10th message, ERROR every 37th (non-fatal), WARN if `base_`
   * is empty.
   */
  void OnTimer() {
    std_msgs::msg::String msg;
    msg.data = base_ + " seq=" + std::to_string(count_);

    // Demonstrate multiple logging levels.
    if ((count_ % 10) == 0) {
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Timer tick; next message count=" << count_);
    }
    if (base_.empty()) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Base string is empty; publishing only sequence number.");
    }

    pub_->publish(msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << msg.data << "'");
    ++count_;

    // Emit an ERROR (non-fatal) every 37 messages to demonstrate level usage.
    if ((count_ % 37) == 0) {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Simulated non-fatal error condition at count=" << count_);
    }
  }

  /**
   * @brief Service handler for `/set_base_string`.
   *
   * @param[in]  req  Request containing the new base string (`req->base`).
   * @param[out] resp Response populated with success flag and a human-readable
   * message.
   *
   * @details
   * - Rejects base strings longer than 256 characters (sets `success=false`,
   * logs ERROR).
   * - On success, updates internal `base_`, returns `success=true`, logs INFO.
   * - If the new base equals `"FATAL"`, emits a FATAL log (demo only).
   */
  void OnSetBaseString(const SetBaseString::Request::SharedPtr req,
                       SetBaseString::Response::SharedPtr resp) {
    const std::string &requested = req->base;
    if (requested.size() > 256) {
      resp->success = false;
      resp->message = "Rejected: base string too long (>256)";
      RCLCPP_ERROR_STREAM(this->get_logger(), resp->message);
      return;
    }
    base_ = requested;
    resp->success = true;
    resp->message = "Base string updated to '" + base_ + "'";
    RCLCPP_INFO_STREAM(this->get_logger(), resp->message);

    // Demonstrate FATAL logging level.
    if (base_ == "FATAL") {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "User set base_string to 'FATAL' (demo of FATAL level).");
    }
  }

  // ----------------------- Members -----------------------

  /**
   * @brief Publisher for `std_msgs::msg::String` on topic `"topic"`.
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  /**
   * @brief Service server for `/set_base_string`.
   */
  rclcpp::Service<SetBaseString>::SharedPtr srv_;

  /**
   * @brief Wall timer controlling publish cadence.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Configured timer period derived from `publish_frequency_hz`.
   */
  std::chrono::milliseconds timer_period_;

  /**
   * @brief Base text prefix for published messages (mutable via
   * service/parameters).
   */
  std::string base_;

  /**
   * @brief Monotonic sequence counter included in the payload.
   */
  size_t count_;
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
  rclcpp::spin(std::make_shared<TalkerWithService>());
  rclcpp::shutdown();
  return 0;
}