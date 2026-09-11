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

#include <algorithm>
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

rmw_qos_durability_policy_t parse_durability(const std::string & value)
{
  if (value == "transient_local") {
    return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  }
  return RMW_QOS_POLICY_DURABILITY_VOLATILE;
}

rmw_qos_reliability_policy_t parse_reliability(const std::string & value)
{
  if (value == "best_effort") {
    return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }
  return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
}

class DurabilitySubscriberCpp : public rclcpp::Node
{
public:
  DurabilitySubscriberCpp(
    const std::string & topic,
    const std::string & durability,
    const std::string & reliability,
    const std::string & expect,
    bool expect_none,
    bool enable_incompatible_events,
    double timeout_sec)
  : Node("qos_durability_sub_cpp",
      rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false)),
    expect_(expect),
    expect_none_(expect_none),
    done_(false)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.durability(parse_durability(durability));
    qos.reliability(parse_reliability(reliability));

    rclcpp::SubscriptionOptions options;
    if (enable_incompatible_events) {
      options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
          incompatible_events_++;
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
        RCLCPP_INFO(get_logger(), "received: %s", msg->data.c_str());
      },
      options);
    observation_timer_ = create_wall_timer(
      500ms,
      [this]() {
        RCLCPP_INFO(
          get_logger(),
          "publisher_count observation: %zu",
          count_publishers(subscription_->get_topic_name()));
      });

    RCLCPP_INFO(
      get_logger(),
      "subscriber ready: topic=%s, durability=%s, reliability=%s",
      topic.c_str(),
      durability.c_str(),
      reliability.c_str());

    finish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
        timeout_sec)),
      [this]() {
        finish_timer_->cancel();
        evaluate();
      });
  }

  bool ok() const
  {
    return ok_;
  }

  bool done() const
  {
    return done_;
  }

private:
  void evaluate()
  {
    if (!expect_none_) {
      ok_ = std::find(received_messages_.begin(), received_messages_.end(), expect_) !=
        received_messages_.end();
      if (ok_) {
        RCLCPP_INFO(get_logger(), "durability subscriber ok: expected=%s", expect_.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "durability subscriber failed: received_count=%zu, publisher_count=%zu,"
          "incompatible_events=%d",
          received_messages_.size(),
          count_publishers(subscription_->get_topic_name()),
          incompatible_events_);
      }
    } else {
      ok_ = received_messages_.empty();
      if (ok_) {
        RCLCPP_INFO(get_logger(), "durability subscriber ok: received nothing");
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "durability subscriber failed: received_count=%zu, publisher_count=%zu,"
          "incompatible_events=%d",
          received_messages_.size(),
          count_publishers(subscription_->get_topic_name()),
          incompatible_events_);
      }
    }
    done_ = true;
  }

  std::string expect_;
  bool expect_none_;
  bool ok_{false};
  bool done_;
  int incompatible_events_{0};
  std::vector<std::string> received_messages_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr finish_timer_;
  rclcpp::TimerBase::SharedPtr observation_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic = "qos_durability_topic";
  std::string durability;
  std::string reliability = "reliable";
  std::string expect;
  bool expect_none = false;
  bool enable_incompatible_events = false;
  double timeout_sec = 3.0;

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
    } else if (arg == "--durability") {
      durability = next_value("--durability");
    } else if (arg == "--reliability") {
      reliability = next_value("--reliability");
    } else if (arg == "--expect") {
      expect = next_value("--expect");
    } else if (arg == "--timeout") {
      timeout_sec = std::stod(next_value("--timeout"));
    } else if (arg == "--expect-none") {
      expect_none = true;
    } else if (arg == "--enable-incompatible-events") {
      enable_incompatible_events = true;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (durability.empty()) {
    throw std::runtime_error("--durability is required");
  }
  if ((expect.empty() && !expect_none) || (!expect.empty() && expect_none)) {
    throw std::runtime_error("exactly one of --expect or --expect-none is required");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DurabilitySubscriberCpp>(
    topic, durability, reliability, expect, expect_none, enable_incompatible_events, timeout_sec);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    executor.spin_once(100ms);
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return node->ok() ? 0 : 1;
}
