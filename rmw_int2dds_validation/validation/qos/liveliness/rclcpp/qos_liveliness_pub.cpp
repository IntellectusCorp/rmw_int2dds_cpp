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

class LivelinessPublisherCpp : public rclcpp::Node
{
public:
  LivelinessPublisherCpp(
    const std::string & topic,
    const std::string & liveliness,
    double lease_sec,
    const std::string & seed_prefix,
    double seed_delay_sec,
    int repeat_count,
    double repeat_period_sec,
    double assert_period_sec,
    double keep_alive_sec,
    int wait_for_subscription_count,
    double wait_timeout_sec)
  : Node("qos_liveliness_pub_cpp",
      rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false)),
    topic_(topic),
    seed_prefix_(seed_prefix),
    repeat_count_(repeat_count),
    repeat_period_sec_(repeat_period_sec),
    assert_period_sec_(assert_period_sec),
    wait_for_subscription_count_(wait_for_subscription_count),
    wait_timeout_sec_(wait_timeout_sec)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.liveliness(parse_liveliness(liveliness));
    qos.liveliness_lease_duration(rclcpp::Duration::from_seconds(lease_sec));

    publisher_ = create_publisher<std_msgs::msg::String>(topic, qos);
    observe_timer_ = create_wall_timer(500ms, [this]() {
          RCLCPP_INFO(get_logger(), "subscriber_count observation: %zu", count_subscribers(topic_));
    });
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(seed_delay_sec, 0.01))),
      [this]() {publish_once();});
    shutdown_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(seed_delay_sec + keep_alive_sec, 0.1))),
      [this]() {
        shutdown_timer_->cancel();
        observe_timer_->cancel();
        if (assert_timer_) {
          assert_timer_->cancel();
        }
        done_ = true;
        RCLCPP_INFO(get_logger(), "publisher keep-alive finished");
      });
    if (assert_period_sec_ > 0.0) {
      assert_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(std::max(assert_period_sec_, 0.01))),
        [this]() {assert_once();});
    }

    start_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "publisher ready: topic=%s, liveliness=%s, lease=%.3fs, assert_period=%.3fs",
      topic.c_str(),
      liveliness.c_str(),
      lease_sec,
      assert_period_sec_);
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
        if ((now() - start_time_).seconds() > wait_timeout_sec_) {
          publish_timer_->cancel();
          observe_timer_->cancel();
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
    msg.data = seed_prefix_ + "-" + std::to_string(publish_count_);
    publisher_->publish(msg);
    ++publish_count_;
    RCLCPP_INFO(get_logger(), "published: %s", msg.data.c_str());

    if (publish_count_ >= repeat_count_) {
      publish_timer_->cancel();
    } else {
      publish_timer_->reset();
      publish_timer_->cancel();
      publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(repeat_period_sec_)),
        [this]() {publish_once();});
    }
  }

  void assert_once()
  {
    if (publish_count_ == 0) {
      return;
    }
    if (!publisher_->assert_liveliness()) {
      RCLCPP_ERROR(get_logger(), "assert_liveliness failed");
      return;
    }
    ++assert_count_;
    RCLCPP_INFO(get_logger(), "asserted liveliness #%d", assert_count_);
  }

  std::string topic_;
  std::string seed_prefix_;
  int repeat_count_;
  double repeat_period_sec_;
  double assert_period_sec_;
  int wait_for_subscription_count_;
  double wait_timeout_sec_;
  bool done_{false};
  int publish_count_{0};
  int assert_count_{0};
  rclcpp::Time start_time_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr observe_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr assert_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic;
  std::string liveliness = "automatic";
  double lease_sec = 0.5;
  std::string seed_prefix = "alive";
  double seed_delay_sec = 0.1;
  int repeat_count = 1;
  double repeat_period_sec = 0.5;
  double assert_period_sec = 0.0;
  double keep_alive_sec = 2.0;
  int wait_for_subscription_count = 0;
  double wait_timeout_sec = 5.0;

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
    } else if (arg == "--seed-prefix") {
      seed_prefix = next_value("--seed-prefix");
    } else if (arg == "--seed-delay") {
      seed_delay_sec = std::stod(next_value("--seed-delay"));
    } else if (arg == "--repeat-count") {
      repeat_count = std::stoi(next_value("--repeat-count"));
    } else if (arg == "--repeat-period") {
      repeat_period_sec = std::stod(next_value("--repeat-period"));
    } else if (arg == "--assert-period") {
      assert_period_sec = std::stod(next_value("--assert-period"));
    } else if (arg == "--keep-alive") {
      keep_alive_sec = std::stod(next_value("--keep-alive"));
    } else if (arg == "--wait-for-subscriber-count") {
      wait_for_subscription_count = std::stoi(next_value("--wait-for-subscriber-count"));
    } else if (arg == "--wait-timeout") {
      wait_timeout_sec = std::stod(next_value("--wait-timeout"));
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (topic.empty()) {
    throw std::runtime_error("--topic is required");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivelinessPublisherCpp>(
    topic,
    liveliness,
    lease_sec,
    seed_prefix,
    seed_delay_sec,
    repeat_count,
    repeat_period_sec,
    assert_period_sec,
    keep_alive_sec,
    wait_for_subscription_count,
    wait_timeout_sec);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    executor.spin_once(100ms);
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
