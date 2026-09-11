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
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{

rmw_qos_liveliness_policy_t parse_liveliness(const std::string & value)
{
  if (value == "manual_by_topic") {
    return RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  }
  return RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
}

class LivelinessSubscriberCpp : public rclcpp::Node
{
public:
  LivelinessSubscriberCpp(
    const std::string & topic,
    const std::string & liveliness,
    double lease_sec,
    double timeout_sec,
    int expect_at_least,
    bool require_not_alive_event)
  : Node("qos_liveliness_sub_cpp",
      rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false)),
    topic_(topic),
    expect_at_least_(expect_at_least),
    require_not_alive_event_(require_not_alive_event)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.liveliness(parse_liveliness(liveliness));
    qos.liveliness_lease_duration(rclcpp::Duration::from_seconds(lease_sec));

    rclcpp::SubscriptionOptions options;
    if (require_not_alive_event_) {
      options.event_callbacks.liveliness_callback =
        [this](rclcpp::QOSLivelinessChangedInfo & info) {
          liveliness_events_.emplace_back(
            info.alive_count,
            info.not_alive_count,
            info.alive_count_change,
            info.not_alive_count_change);
          RCLCPP_INFO(
            get_logger(),
            "subscription liveliness event: alive_count=%d, not_alive_count=%d,"
            "alive_count_change=%d, not_alive_count_change=%d",
            info.alive_count,
            info.not_alive_count,
            info.alive_count_change,
            info.not_alive_count_change);
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

    observe_timer_ = create_wall_timer(500ms, [this]() {
          RCLCPP_INFO(get_logger(), "publisher_count observation: %zu", count_publishers(topic_));
    });
    finish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
        timeout_sec)),
      [this]() {
        finish_timer_->cancel();
        observe_timer_->cancel();
        evaluate();
      });

    RCLCPP_INFO(
      get_logger(),
      "subscriber ready: topic=%s, liveliness=%s, lease=%.3fs",
      topic.c_str(),
      liveliness.c_str(),
      lease_sec);
  }

  bool done() const
  {
    return done_;
  }

  bool ok() const
  {
    return ok_;
  }

private:
  void evaluate()
  {
    size_t not_alive_events = 0;
    for (const auto & event : liveliness_events_) {
      if (std::get<3>(event) > 0) {
        ++not_alive_events;
      }
    }

    ok_ = static_cast<int>(received_messages_.size()) >= expect_at_least_;
    if (require_not_alive_event_) {
      ok_ = ok_ && not_alive_events >= 1;
    }

    if (ok_) {
      RCLCPP_INFO(
        get_logger(),
        "liveliness subscriber ok: received_count=%zu, not_alive_events=%zu",
        received_messages_.size(),
        not_alive_events);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "liveliness subscriber failed: received_count=%zu, event_count=%zu",
        received_messages_.size(),
        liveliness_events_.size());
    }

    done_ = true;
  }

  std::string topic_;
  int expect_at_least_;
  bool require_not_alive_event_;
  bool done_{false};
  bool ok_{false};
  std::vector<std::string> received_messages_;
  std::vector<std::tuple<int32_t, int32_t, int32_t, int32_t>> liveliness_events_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr observe_timer_;
  rclcpp::TimerBase::SharedPtr finish_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic;
  std::string liveliness = "automatic";
  double lease_sec = 0.5;
  double timeout_sec = 6.0;
  int expect_at_least = 1;
  bool require_not_alive_event = false;

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
    } else if (arg == "--liveliness") {
      liveliness = next_value("--liveliness");
    } else if (arg == "--lease-sec") {
      lease_sec = std::stod(next_value("--lease-sec"));
    } else if (arg == "--timeout") {
      timeout_sec = std::stod(next_value("--timeout"));
    } else if (arg == "--expect-at-least") {
      expect_at_least = std::stoi(next_value("--expect-at-least"));
    } else if (arg == "--require-not-alive-event") {
      require_not_alive_event = true;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (topic.empty()) {
    throw std::runtime_error("--topic is required");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivelinessSubscriberCpp>(
    topic, liveliness, lease_sec, timeout_sec, expect_at_least, require_not_alive_event);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    executor.spin_once(100ms);
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return node->ok() ? 0 : 1;
}
