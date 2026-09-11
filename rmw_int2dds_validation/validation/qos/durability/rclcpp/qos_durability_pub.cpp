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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_options.hpp"
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

class DurabilityPublisherCpp : public rclcpp::Node
{
public:
  DurabilityPublisherCpp(
    const std::string & topic,
    const std::string & durability,
    const std::string & reliability,
    const std::string & seed,
    double seed_delay_sec,
    double keep_alive_sec,
    int repeat_count,
    double repeat_period_sec,
    int wait_for_subscription_count,
    double wait_timeout_sec,
    bool enable_incompatible_events)
  : Node("qos_durability_pub_cpp",
      rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false)),
    topic_(topic),
    seed_(seed),
    keep_alive_sec_(keep_alive_sec),
    repeat_count_(repeat_count),
    repeat_period_sec_(repeat_period_sec),
    wait_for_subscription_count_(wait_for_subscription_count),
    wait_timeout_sec_(wait_timeout_sec)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.durability(parse_durability(durability));
    qos.reliability(parse_reliability(reliability));

    rclcpp::PublisherOptions options;
    if (enable_incompatible_events) {
      options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSOfferedIncompatibleQoSInfo & info) {
          incompatible_events_++;
          RCLCPP_INFO(
            get_logger(),
            "publisher incompatible qos event: total_count=%d, total_count_change=%d,"
            "last_policy_kind=%d",
            info.total_count,
            info.total_count_change,
            info.last_policy_kind);
        };
    }

    publisher_ = create_publisher<std_msgs::msg::String>(topic, qos, options);
    observation_timer_ = create_wall_timer(500ms, [this]() {
          RCLCPP_INFO(get_logger(), "subscriber_count observation: %zu", count_subscribers(topic_));
    });
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
          std::max(seed_delay_sec, 0.01))),
      [this]() {publish_once();});
    shutdown_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
          std::max(seed_delay_sec + keep_alive_sec_, 0.1))),
      [this]() {
        shutdown_timer_->cancel();
        observation_timer_->cancel();
        done_ = true;
        RCLCPP_INFO(get_logger(), "publisher keep-alive finished");
      });

    start_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "publisher ready: topic=%s, durability=%s, reliability=%s",
      topic.c_str(),
      durability.c_str(),
      reliability.c_str());
  }

  bool done() const
  {
    return done_;
  }

private:
  void publish_once()
  {
    if (wait_for_subscription_count_ > 0) {
      const auto current_count = count_subscribers(topic_);
      if (static_cast<int>(current_count) < wait_for_subscription_count_) {
        const auto elapsed = (now() - start_time_).seconds();
        if (elapsed > wait_timeout_sec_) {
          publish_timer_->cancel();
          observation_timer_->cancel();
          shutdown_timer_->cancel();
          done_ = true;
          RCLCPP_ERROR(
            get_logger(),
            "publisher wait-for-match failed: expected=%d, current=%zu",
            wait_for_subscription_count_,
            current_count);
        }
        return;
      }
    }

    std_msgs::msg::String msg;
    msg.data = seed_;
    publisher_->publish(msg);
    publish_count_++;
    RCLCPP_INFO(get_logger(), "published seed #%d: %s", publish_count_, seed_.c_str());

    if (publish_count_ >= repeat_count_) {
      publish_timer_->cancel();
    } else {
      publish_timer_->reset();
      publish_timer_->cancel();
      publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
            repeat_period_sec_)),
        [this]() {publish_once();});
    }
  }

  std::string topic_;
  std::string seed_;
  double keep_alive_sec_;
  int repeat_count_;
  double repeat_period_sec_;
  int wait_for_subscription_count_;
  double wait_timeout_sec_;
  bool done_{false};
  int publish_count_{0};
  int incompatible_events_{0};
  rclcpp::Time start_time_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr observation_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic = "qos_durability_topic";
  std::string durability;
  std::string reliability = "reliable";
  std::string seed;
  double seed_delay_sec = 0.0;
  double keep_alive_sec = 5.0;
  int repeat_count = 1;
  double repeat_period_sec = 0.5;
  int wait_for_subscription_count = 0;
  double wait_timeout_sec = 5.0;
  bool enable_incompatible_events = false;

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
    } else if (arg == "--seed") {
      seed = next_value("--seed");
    } else if (arg == "--seed-delay") {
      seed_delay_sec = std::stod(next_value("--seed-delay"));
    } else if (arg == "--keep-alive") {
      keep_alive_sec = std::stod(next_value("--keep-alive"));
    } else if (arg == "--repeat-count") {
      repeat_count = std::stoi(next_value("--repeat-count"));
    } else if (arg == "--repeat-period") {
      repeat_period_sec = std::stod(next_value("--repeat-period"));
    } else if (arg == "--wait-for-subscriber-count") {
      wait_for_subscription_count = std::stoi(next_value("--wait-for-subscriber-count"));
    } else if (arg == "--wait-timeout") {
      wait_timeout_sec = std::stod(next_value("--wait-timeout"));
    } else if (arg == "--enable-incompatible-events") {
      enable_incompatible_events = true;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (durability.empty()) {
    throw std::runtime_error("--durability is required");
  }
  if (seed.empty()) {
    throw std::runtime_error("--seed is required");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DurabilityPublisherCpp>(
    topic,
    durability,
    reliability,
    seed,
    seed_delay_sec,
    keep_alive_sec,
    repeat_count,
    repeat_period_sec,
    wait_for_subscription_count,
    wait_timeout_sec,
    enable_incompatible_events);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    executor.spin_once(100ms);
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
