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

class LifespanSubscriberCpp : public rclcpp::Node
{
public:
  LifespanSubscriberCpp(
    const std::string & topic,
    const std::string & durability,
    double timeout_sec,
    const std::string & expect,
    bool expect_none,
    bool require_publisher_observed)
  : Node("qos_lifespan_sub_cpp",
      rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false)),
    topic_(topic),
    expect_(expect),
    expect_none_(expect_none),
    require_publisher_observed_(require_publisher_observed)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(parse_durability(durability));
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    subscription_ = create_subscription<std_msgs::msg::String>(
      topic,
      qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        received_messages_.push_back(msg->data);
        RCLCPP_INFO(get_logger(), "subscriber heard: %s", msg->data.c_str());
      });

    observe_timer_ = create_wall_timer(500ms, [this]() {
          const auto current_count = subscription_->get_publisher_count();
          max_publisher_count_ = std::max(max_publisher_count_, current_count);
          if (current_count > 0) {
            publisher_observed_ = true;
          }
          RCLCPP_INFO(get_logger(), "publisher_count observation: %zu", current_count);
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
      "subscriber ready: topic=%s, durability=%s",
      topic.c_str(),
      durability.c_str());
  }

  bool done() const {return done_;}
  bool ok() const {return ok_;}

private:
  void evaluate()
  {
    if (expect_none_) {
      ok_ = received_messages_.empty();
      if (require_publisher_observed_) {
        ok_ = ok_ && publisher_observed_;
      }
      if (ok_) {
        if (require_publisher_observed_) {
          RCLCPP_INFO(
            get_logger(), "lifespan subscriber ok: received nothing after observing publisher");
        } else {
          RCLCPP_INFO(get_logger(), "lifespan subscriber ok: received nothing");
        }
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "lifespan subscriber failed: received_count=%zu, publisher_observed=%s,"
          "max_publisher_count=%zu",
          received_messages_.size(),
          publisher_observed_ ? "true" : "false",
          max_publisher_count_);
      }
    } else {
      ok_ = received_messages_.size() == 1 && received_messages_.front() == expect_;
      if (ok_) {
        RCLCPP_INFO(get_logger(), "lifespan subscriber ok: expected=%s", expect_.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "lifespan subscriber failed: expected=%s, received_count=%zu",
          expect_.c_str(),
          received_messages_.size());
      }
    }
    done_ = true;
  }

  std::string topic_;
  std::string expect_;
  bool expect_none_;
  bool require_publisher_observed_;
  bool done_{false};
  bool ok_{false};
  bool publisher_observed_{false};
  size_t max_publisher_count_{0};
  std::vector<std::string> received_messages_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr observe_timer_;
  rclcpp::TimerBase::SharedPtr finish_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic;
  std::string durability = "transient_local";
  double timeout_sec = 6.0;
  std::string expect;
  bool expect_none = false;
  bool require_publisher_observed = false;

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
    } else if (arg == "--timeout") {
      timeout_sec = std::stod(next_value("--timeout"));
    } else if (arg == "--expect") {
      expect = next_value("--expect");
    } else if (arg == "--expect-none") {
      expect_none = true;
    } else if (arg == "--require-publisher-observed") {
      require_publisher_observed = true;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (topic.empty()) {
    throw std::runtime_error("--topic is required");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifespanSubscriberCpp>(
    topic, durability, timeout_sec, expect, expect_none, require_publisher_observed);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    executor.spin_once(100ms);
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return node->ok() ? 0 : 1;
}
