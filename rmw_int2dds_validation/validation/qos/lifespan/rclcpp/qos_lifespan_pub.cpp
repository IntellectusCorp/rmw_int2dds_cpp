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

rmw_qos_durability_policy_t parse_durability(const std::string & value)
{
  if (value == "volatile") {
    return RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
  return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
}

class LifespanPublisherCpp : public rclcpp::Node
{
public:
  LifespanPublisherCpp(
    const std::string & topic,
    double lifespan_sec,
    const std::string & durability,
    const std::string & seed,
    double seed_delay_sec,
    int repeat_count,
    double repeat_period_sec,
    double keep_alive_sec,
    int wait_for_subscription_count,
    double wait_timeout_sec)
  : Node("qos_lifespan_pub_cpp",
      rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false)),
    topic_(topic),
    seed_(seed),
    repeat_count_(repeat_count),
    repeat_period_sec_(repeat_period_sec),
    wait_for_subscription_count_(wait_for_subscription_count),
    wait_timeout_sec_(wait_timeout_sec)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(parse_durability(durability));
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    qos.lifespan(rclcpp::Duration::from_seconds(lifespan_sec));

    publisher_ = create_publisher<std_msgs::msg::String>(topic, qos);
    observe_timer_ = create_wall_timer(500ms, [this]() {
          RCLCPP_INFO(get_logger(), "subscriber_count observation: %zu",
          publisher_->get_subscription_count());
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
        done_ = true;
        RCLCPP_INFO(get_logger(), "publisher keep-alive finished");
      });

    start_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "publisher ready: topic=%s, durability=%s, lifespan=%.3fs",
      topic.c_str(),
      durability.c_str(),
      lifespan_sec);
  }

  bool done() const {return done_;}

private:
  void publish_once()
  {
    if (wait_for_subscription_count_ > 0) {
      const auto current_count = publisher_->get_subscription_count();
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
    msg.data = seed_;
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

  std::string topic_;
  std::string seed_;
  int repeat_count_;
  double repeat_period_sec_;
  int wait_for_subscription_count_;
  double wait_timeout_sec_;
  bool done_{false};
  int publish_count_{0};
  rclcpp::Time start_time_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr observe_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic;
  double lifespan_sec = 0.5;
  std::string durability = "transient_local";
  std::string seed = "lifespan-alive";
  double seed_delay_sec = 0.1;
  int repeat_count = 1;
  double repeat_period_sec = 0.5;
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
    } else if (arg == "--lifespan-sec") {
      lifespan_sec = std::stod(next_value("--lifespan-sec"));
    } else if (arg == "--durability") {
      durability = next_value("--durability");
    } else if (arg == "--seed") {
      seed = next_value("--seed");
    } else if (arg == "--seed-delay") {
      seed_delay_sec = std::stod(next_value("--seed-delay"));
    } else if (arg == "--repeat-count") {
      repeat_count = std::stoi(next_value("--repeat-count"));
    } else if (arg == "--repeat-period") {
      repeat_period_sec = std::stod(next_value("--repeat-period"));
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
  auto node = std::make_shared<LifespanPublisherCpp>(
    topic,
    lifespan_sec,
    durability,
    seed,
    seed_delay_sec,
    repeat_count,
    repeat_period_sec,
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
