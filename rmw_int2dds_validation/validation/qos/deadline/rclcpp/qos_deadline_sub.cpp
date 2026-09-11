// Copyright 2026 Int2DDS Project
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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{

class DeadlineSubscriberCpp : public rclcpp::Node
{
public:
  DeadlineSubscriberCpp(
    const std::string & topic,
    double deadline_sec,
    double timeout_sec,
    int expect_at_least,
    bool expect_none,
    bool require_deadline_event,
    bool require_incompatible_event)
  : Node("qos_deadline_sub_cpp",
      rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false)),
    topic_(topic),
    expect_at_least_(expect_at_least),
    expect_none_(expect_none),
    require_deadline_event_(require_deadline_event),
    require_incompatible_event_(require_incompatible_event)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.deadline(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
        deadline_sec)));

    rclcpp::SubscriptionOptions options;
    if (require_deadline_event) {
      options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineRequestedInfo & info) {
          deadline_events_.emplace_back(info.total_count, info.total_count_change);
          RCLCPP_INFO(
            get_logger(),
            "subscription deadline event: total_count=%d, total_count_change=%d",
            info.total_count,
            info.total_count_change);
        };
    }
    if (require_incompatible_event) {
      options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
          incompatible_events_.push_back(
            std::make_tuple(info.total_count, info.total_count_change, info.last_policy_kind));
          RCLCPP_INFO(
            get_logger(),
            "subscription incompatible qos event: total_count=%d, total_count_change=%d,"
            "last_policy_kind=%d",
            info.total_count,
            info.total_count_change,
            info.last_policy_kind);
        };
    }

    subscription_ = create_subscription<std_msgs::msg::String>(
      topic,
      qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        received_messages_.push_back(msg->data);
        RCLCPP_INFO(get_logger(), "subscriber heard: %s", msg->data.c_str());
      },
      options);
    observation_timer_ = create_wall_timer(500ms, [this]() {
          RCLCPP_INFO(get_logger(), "publisher_count observation: %zu", count_publishers(topic_));
    });
    finish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
        timeout_sec)),
      [this]() {
        finish_timer_->cancel();
        observation_timer_->cancel();
        done_ = true;
      });
    RCLCPP_INFO(
      get_logger(), "subscriber ready: topic=%s, deadline=%.3fs", topic.c_str(), deadline_sec);
  }

  bool done() const {return done_;}

  bool evaluate()
  {
    bool ok = true;
    if (expect_none_) {
      ok = ok && received_messages_.empty();
    } else {
      ok = ok && static_cast<int>(received_messages_.size()) >= expect_at_least_;
    }
    if (require_deadline_event_) {
      ok = ok && !deadline_events_.empty();
    }
    if (require_incompatible_event_) {
      ok = ok && !incompatible_events_.empty();
    }

    if (ok) {
      if (expect_none_) {
        RCLCPP_INFO(get_logger(), "deadline subscriber ok: received nothing");
      } else {
        RCLCPP_INFO(get_logger(), "deadline subscriber ok: received_count=%zu",
            received_messages_.size());
      }
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "deadline subscriber failed: received_count=%zu, deadline_events=%zu,"
        "incompatible_events=%zu",
        received_messages_.size(), deadline_events_.size(), incompatible_events_.size());
    }
    return ok;
  }

private:
  std::string topic_;
  int expect_at_least_;
  bool expect_none_;
  bool require_deadline_event_;
  bool require_incompatible_event_;
  bool done_{false};
  std::vector<std::string> received_messages_;
  std::vector<std::pair<int, int>> deadline_events_;
  std::vector<std::tuple<int, int, int>> incompatible_events_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr observation_timer_;
  rclcpp::TimerBase::SharedPtr finish_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic;
  double deadline_sec = 0.2;
  double timeout_sec = 8.0;
  int expect_at_least = 0;
  bool expect_none = false;
  bool require_deadline_event = false;
  bool require_incompatible_event = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto next_value = [&](const char * name) -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error(std::string("missing value for ") + name);
        }
        return argv[++i];
      };
    if (arg == "--topic") {
      topic = next_value("--topic");
    } else if (arg == "--deadline-sec") {
      deadline_sec = std::stod(next_value("--deadline-sec"));
    } else if (arg == "--timeout") {
      timeout_sec = std::stod(next_value("--timeout"));
    } else if (arg == "--expect-at-least") {
      expect_at_least = std::stoi(next_value("--expect-at-least"));
    } else if (arg == "--expect-none") {
      expect_none = true;
    } else if (arg == "--require-deadline-event") {
      require_deadline_event = true;
    } else if (arg == "--require-incompatible-event") {
      require_incompatible_event = true;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (topic.empty()) {
    throw std::runtime_error("--topic is required");
  }
  if (expect_none && expect_at_least > 0) {
    throw std::runtime_error("--expect-none and --expect-at-least cannot be used together");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeadlineSubscriberCpp>(
    topic, deadline_sec, timeout_sec, expect_at_least, expect_none, require_deadline_event,
    require_incompatible_event);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    executor.spin_once(100ms);
  }
  auto ok = node->evaluate();
  executor.remove_node(node);
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
