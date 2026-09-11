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

#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{

class DeadlinePublisherCpp : public rclcpp::Node
{
public:
  DeadlinePublisherCpp(
    const std::string & topic,
    double deadline_sec,
    double publish_period_sec,
    const std::string & seed_prefix,
    double seed_delay_sec,
    double keep_alive_sec,
    int repeat_count,
    int wait_for_subscription_count,
    double wait_timeout_sec,
    bool enable_deadline_events,
    bool enable_incompatible_events)
  : Node("qos_deadline_pub_cpp",
      rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false)),
    topic_(topic),
    seed_prefix_(seed_prefix),
    publish_period_sec_(publish_period_sec),
    keep_alive_sec_(keep_alive_sec),
    repeat_count_(repeat_count),
    wait_for_subscription_count_(wait_for_subscription_count),
    wait_timeout_sec_(wait_timeout_sec)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.deadline(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
        deadline_sec)));

    rclcpp::PublisherOptions options;
    if (enable_deadline_events) {
      options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineOfferedInfo & info) {
          RCLCPP_INFO(
            get_logger(),
            "publisher deadline event: total_count=%d, total_count_change=%d",
            info.total_count,
            info.total_count_change);
        };
    }
    if (enable_incompatible_events) {
      options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSOfferedIncompatibleQoSInfo & info) {
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
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(seed_delay_sec, 0.01))),
      [this]() {publish_once();});
    shutdown_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(seed_delay_sec + keep_alive_sec_, 0.1))),
      [this]() {
        shutdown_timer_->cancel();
        observation_timer_->cancel();
        done_ = true;
        RCLCPP_INFO(get_logger(), "publisher keep-alive finished");
      });

    start_time_ = now();
    RCLCPP_INFO(
      get_logger(), "publisher ready: topic=%s, deadline=%.3fs", topic.c_str(), deadline_sec);
  }

  bool done() const {return done_;}

private:
  void publish_once()
  {
    if (wait_for_subscription_count_ > 0) {
      auto current_count = count_subscribers(topic_);
      if (static_cast<int>(current_count) < wait_for_subscription_count_) {
        if ((now() - start_time_).seconds() > wait_timeout_sec_) {
          publish_timer_->cancel();
          observation_timer_->cancel();
          shutdown_timer_->cancel();
          done_ = true;
          RCLCPP_ERROR(
            get_logger(), "publisher wait-for-match failed: expected=%d, current=%zu",
            wait_for_subscription_count_, current_count);
        }
        return;
      }
    }

    std_msgs::msg::String msg;
    msg.data = seed_prefix_ + "-" + std::to_string(publish_count_);
    publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "published: %s", msg.data.c_str());
    publish_count_++;

    if (publish_count_ >= repeat_count_) {
      publish_timer_->cancel();
    } else {
      publish_timer_->reset();
      publish_timer_->cancel();
      publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(publish_period_sec_)),
        [this]() {publish_once();});
    }
  }

  std::string topic_;
  std::string seed_prefix_;
  double publish_period_sec_;
  double keep_alive_sec_;
  int repeat_count_;
  int wait_for_subscription_count_;
  double wait_timeout_sec_;
  bool done_{false};
  int publish_count_{0};
  rclcpp::Time start_time_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr observation_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic;
  double deadline_sec = 0.2;
  double publish_period_sec = 0.5;
  std::string seed_prefix = "tick";
  double seed_delay_sec = 0.0;
  double keep_alive_sec = 3.0;
  int repeat_count = 3;
  int wait_for_subscription_count = 0;
  double wait_timeout_sec = 5.0;
  bool enable_deadline_events = false;
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
    } else if (arg == "--deadline-sec") {
      deadline_sec = std::stod(next_value("--deadline-sec"));
    } else if (arg == "--publish-period") {
      publish_period_sec = std::stod(next_value("--publish-period"));
    } else if (arg == "--seed-prefix") {
      seed_prefix = next_value("--seed-prefix");
    } else if (arg == "--seed-delay") {
      seed_delay_sec = std::stod(next_value("--seed-delay"));
    } else if (arg == "--keep-alive") {
      keep_alive_sec = std::stod(next_value("--keep-alive"));
    } else if (arg == "--repeat-count") {
      repeat_count = std::stoi(next_value("--repeat-count"));
    } else if (arg == "--wait-for-subscriber-count") {
      wait_for_subscription_count = std::stoi(next_value("--wait-for-subscriber-count"));
    } else if (arg == "--wait-timeout") {
      wait_timeout_sec = std::stod(next_value("--wait-timeout"));
    } else if (arg == "--enable-deadline-events") {
      enable_deadline_events = true;
    } else if (arg == "--enable-incompatible-events") {
      enable_incompatible_events = true;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (topic.empty()) {
    throw std::runtime_error("--topic is required");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeadlinePublisherCpp>(
    topic, deadline_sec, publish_period_sec, seed_prefix, seed_delay_sec, keep_alive_sec,
    repeat_count,
    wait_for_subscription_count, wait_timeout_sec, enable_deadline_events,
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
