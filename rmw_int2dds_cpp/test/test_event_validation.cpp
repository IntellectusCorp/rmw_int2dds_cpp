#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "rmw/event.h"
#include "rmw/error_handling.h"
#if __has_include("rmw/events_statuses/events_statuses.h")
#include "rmw/events_statuses/events_statuses.h"
#else
// Foxy: liveliness/deadline statuses live in rmw/types.h, the
// incompatible-QoS ones in rmw/incompatible_qos_events_statuses.h.
#include "rmw/incompatible_qos_events_statuses.h"
#include "rmw/types.h"
#endif
#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw/publisher_options.h"
#include "rmw/qos_profiles.h"
#include "rmw/rmw.h"
#include "rmw/subscription_options.h"

#include "int2dds-ffi.h"
#include "rmw_int2dds_cpp/types.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "std_msgs/msg/string.hpp"

#if __has_include("rmw/events_statuses/matched.h")
#include "rmw/events_statuses/matched.h"
#define RMW_INT2DDS_HAS_MATCHED_EVENT_API 1
#else
#define RMW_INT2DDS_HAS_MATCHED_EVENT_API 0
#endif

// RMW_EVENT_MESSAGE_LOST and rmw_event_set_callback are Humble additions;
// Foxy has neither.
#if __has_include("rmw/events_statuses/message_lost.h")
#define RMW_INT2DDS_HAS_MESSAGE_LOST_EVENT_API 1
#else
#define RMW_INT2DDS_HAS_MESSAGE_LOST_EVENT_API 0
#endif

#if __has_include("rmw/event_callback_type.h")
#define RMW_INT2DDS_HAS_EVENT_CALLBACK_API 1
#else
#define RMW_INT2DDS_HAS_EVENT_CALLBACK_API 0
#endif

#if __has_include("rmw/events_statuses/incompatible_type.h")
#include "rmw/events_statuses/incompatible_type.h"
#define RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT_API 1
#else
#define RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT_API 0
#endif

namespace
{

constexpr int kEventAttempts = 30;
constexpr uint64_t kWaitNs = 200000000;
constexpr uint64_t kShortSleepMs = 50;
constexpr uint64_t kDeadlineNs = 200000000;

void
print_error_and_return(const char * label)
{
  const rmw_error_string_t error_string = rmw_get_error_string();
  std::cerr << "[ERROR] " << label;
  if (error_string.str[0] != '\0') {
    std::cerr << ": " << error_string.str;
  }
  std::cerr << std::endl;
  rmw_reset_error();
}

bool
check_ret(rmw_ret_t ret, const char * label)
{
  if (ret == RMW_RET_OK) {
    return true;
  }
  print_error_and_return(label);
  return false;
}

#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
bool
expect_ret(rmw_ret_t ret, rmw_ret_t expected, const char * label)
{
  if (ret == expected) {
    return true;
  }

  std::cerr << "[ERROR] " << label << ": expected ret=" << expected << " actual ret=" << ret;
  const rmw_error_string_t error_string = rmw_get_error_string();
  if (error_string.str[0] != '\0') {
    std::cerr << " message=" << error_string.str;
  }
  std::cerr << std::endl;
  rmw_reset_error();
  return false;
}
#endif

void
ignore_ret(rmw_ret_t ret)
{
  (void)ret;
}

#if RMW_INT2DDS_HAS_EVENT_CALLBACK_API
void
noop_event_callback(const void * user_data, size_t number_of_events)
{
  (void)user_data;
  (void)number_of_events;
}
#endif

struct TestContext
{
  rcutils_allocator_t allocator{rcutils_get_default_allocator()};
  rmw_init_options_t init_options{rmw_get_zero_initialized_init_options()};
  rmw_context_t context{rmw_get_zero_initialized_context()};
  rmw_node_t * publisher_node{nullptr};
  rmw_node_t * subscription_node{nullptr};
  rmw_publisher_t * publisher{nullptr};
  rmw_subscription_t * subscription{nullptr};
  rmw_wait_set_t * wait_set{nullptr};
  const rosidl_message_type_support_t * type_support{
    rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::String>()
  };
};

void
cleanup_context(TestContext & ctx)
{
  if (ctx.publisher != nullptr && ctx.publisher_node != nullptr) {
    ignore_ret(rmw_destroy_publisher(ctx.publisher_node, ctx.publisher));
    ctx.publisher = nullptr;
  }
  if (ctx.subscription != nullptr && ctx.subscription_node != nullptr) {
    ignore_ret(rmw_destroy_subscription(ctx.subscription_node, ctx.subscription));
    ctx.subscription = nullptr;
  }
  if (ctx.wait_set != nullptr) {
    ignore_ret(rmw_destroy_wait_set(ctx.wait_set));
    ctx.wait_set = nullptr;
  }
  if (ctx.publisher_node != nullptr) {
    ignore_ret(rmw_destroy_node(ctx.publisher_node));
    ctx.publisher_node = nullptr;
  }
  if (ctx.subscription_node != nullptr) {
    ignore_ret(rmw_destroy_node(ctx.subscription_node));
    ctx.subscription_node = nullptr;
  }
  if (ctx.context.implementation_identifier != nullptr) {
    ignore_ret(rmw_shutdown(&ctx.context));
    ignore_ret(rmw_context_fini(&ctx.context));
    ctx.context = rmw_get_zero_initialized_context();
  }
  if (ctx.init_options.impl != nullptr) {
    ignore_ret(rmw_init_options_fini(&ctx.init_options));
    ctx.init_options = rmw_get_zero_initialized_init_options();
  }
}

rmw_node_t *
create_node(TestContext & ctx, const char * name)
{
#if __has_include("rmw/get_network_flow_endpoints.h")
  return rmw_create_node(&ctx.context, name, "/");
#else
  // Foxy still passes the node's domain id and localhost flag; hand over the
  // values the context was initialized with (the default domain is 0).
  const size_t domain_id = ctx.init_options.domain_id == RMW_DEFAULT_DOMAIN_ID ?
    0u : ctx.init_options.domain_id;
  return rmw_create_node(&ctx.context, name, "/", domain_id, false);
#endif
}

bool
init_context(TestContext & ctx, const char * publisher_node_name, const char * subscription_node_name)
{
  if (!check_ret(rmw_init_options_init(&ctx.init_options, ctx.allocator), "rmw_init_options_init")) {
    return false;
  }

  ctx.init_options.enclave = rcutils_strdup("/", ctx.allocator);
  if (ctx.init_options.enclave == nullptr) {
    std::cerr << "[ERROR] failed to allocate enclave" << std::endl;
    cleanup_context(ctx);
    return false;
  }

  if (!check_ret(rmw_init(&ctx.init_options, &ctx.context), "rmw_init")) {
    cleanup_context(ctx);
    return false;
  }

  ctx.publisher_node = create_node(ctx, publisher_node_name);
  if (ctx.publisher_node == nullptr) {
    print_error_and_return("rmw_create_node(publisher)");
    cleanup_context(ctx);
    return false;
  }

  ctx.subscription_node = create_node(ctx, subscription_node_name);
  if (ctx.subscription_node == nullptr) {
    print_error_and_return("rmw_create_node(subscription)");
    cleanup_context(ctx);
    return false;
  }

  ctx.wait_set = rmw_create_wait_set(&ctx.context, 1);
  if (ctx.wait_set == nullptr) {
    print_error_and_return("rmw_create_wait_set");
    cleanup_context(ctx);
    return false;
  }

  return true;
}

bool
create_publisher(
  TestContext & ctx,
  const std::string & topic_name,
  const rmw_qos_profile_t & qos)
{
  rmw_publisher_options_t options = rmw_get_default_publisher_options();
  ctx.publisher = rmw_create_publisher(
    ctx.publisher_node,
    ctx.type_support,
    topic_name.c_str(),
    &qos,
    &options);
  if (ctx.publisher == nullptr) {
    print_error_and_return("rmw_create_publisher");
    return false;
  }
  return true;
}

bool
create_subscription(
  TestContext & ctx,
  const std::string & topic_name,
  const rmw_qos_profile_t & qos)
{
  rmw_subscription_options_t options = rmw_get_default_subscription_options();
  ctx.subscription = rmw_create_subscription(
    ctx.subscription_node,
    ctx.type_support,
    topic_name.c_str(),
    &qos,
    &options);
  if (ctx.subscription == nullptr) {
    print_error_and_return("rmw_create_subscription");
    return false;
  }
  return true;
}

template<typename StatusT, typename Predicate>
bool
wait_for_event(
  rmw_event_t & event,
  rmw_wait_set_t * wait_set,
  StatusT & status,
  Predicate predicate,
  const char * label)
{
  rmw_ret_t wait_ret = RMW_RET_TIMEOUT;
  bool taken = false;

  for (int attempt = 0; attempt < kEventAttempts; ++attempt) {
    void * event_entries[1] = {&event};
    rmw_events_t events{1, event_entries};
    rmw_time_t timeout{0, kWaitNs};

    wait_ret = rmw_wait(nullptr, nullptr, nullptr, nullptr, &events, wait_set, &timeout);
    if (wait_ret == RMW_RET_TIMEOUT) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kShortSleepMs));
      continue;
    }
    if (!check_ret(wait_ret, label)) {
      return false;
    }
    if (events.events[0] == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kShortSleepMs));
      continue;
    }
    if (!check_ret(rmw_take_event(&event, &status, &taken), "rmw_take_event")) {
      return false;
    }
    if (taken && predicate(status)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kShortSleepMs));
  }

  std::cerr << "[ERROR] " << label << ": timed out waiting for event" << std::endl;
  return false;
}

bool
print_offered_deadline_ffi_status(TestContext & ctx)
{
  if (ctx.publisher == nullptr || ctx.publisher->data == nullptr) {
    return false;
  }

  auto * publisher_data = static_cast<rmw_int2dds_cpp::PublisherData *>(ctx.publisher->data);
  if (publisher_data->datawriter == nullptr) {
    return false;
  }

  Int2DdsOfferedDeadlineMissedStatus ffi_status{};
  const Int2DdsRet ret = int2dds_datawriter_get_offered_deadline_missed_status(
    publisher_data->datawriter, &ffi_status);
  if (ret != INT2DDS_RET_OK) {
    std::cerr << "[DIAG] direct offered deadline status query failed, ret=" << ret << std::endl;
    return false;
  }

  std::cerr << "[DIAG] direct offered deadline status"
            << " total_count=" << ffi_status.total_count
            << " total_count_change=" << ffi_status.total_count_change
            << std::endl;
  return ffi_status.total_count >= 1;
}

bool
wait_for_subscription_matched(
  TestContext & ctx,
  const std::string & topic_name,
  const char * label)
{
#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
  (void)topic_name;
  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_subscription_event_init(&event, ctx.subscription, RMW_EVENT_SUBSCRIPTION_MATCHED),
      "rmw_subscription_event_init(subscription_matched)"))
  {
    return false;
  }

  rmw_matched_status_t status{};
  return wait_for_event(
    event, ctx.wait_set, status,
    [](const rmw_matched_status_t & matched) {
      return matched.total_count >= 1 && matched.current_count >= 1;
    },
    label);
#else
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    size_t publisher_count = 0;
    size_t subscriber_count = 0;
    const bool graph_ok =
      check_ret(
      rmw_count_publishers(ctx.subscription_node, topic_name.c_str(), &publisher_count),
      "rmw_count_publishers") &&
      check_ret(
      rmw_count_subscribers(ctx.publisher_node, topic_name.c_str(), &subscriber_count),
      "rmw_count_subscribers");
    if (graph_ok && publisher_count >= 1 && subscriber_count >= 1) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kShortSleepMs));
  }
  std::cerr << "[ERROR] " << label << ": timed out waiting for graph match" << std::endl;
  return false;
#endif
}

[[maybe_unused]] bool
wait_for_publication_matched(TestContext & ctx, const char * label)
{
#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_publisher_event_init(&event, ctx.publisher, RMW_EVENT_PUBLICATION_MATCHED),
      "rmw_publisher_event_init(publication_matched)"))
  {
    return false;
  }

  rmw_matched_status_t status{};
  return wait_for_event(
    event, ctx.wait_set, status,
    [](const rmw_matched_status_t & matched) {
      return matched.total_count >= 1 && matched.current_count >= 1;
    },
    label);
#else
  (void)ctx;
  std::cerr << "[ERROR] " << label << ": matched events are not available in this rmw API" << std::endl;
  return false;
#endif
}

bool
publish_once(TestContext & ctx, const char * text)
{
  std_msgs::msg::String message;
  message.data = text;
  return check_ret(rmw_publish(ctx.publisher, &message, nullptr), "rmw_publish");
}

bool
test_support_matrix()
{
#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
  struct EventSupport
  {
    rmw_event_type_t event_type;
    bool expected;
    const char * name;
  };

  const EventSupport checks[] = {
    {RMW_EVENT_PUBLICATION_MATCHED, true, "RMW_EVENT_PUBLICATION_MATCHED"},
    {RMW_EVENT_OFFERED_DEADLINE_MISSED, true, "RMW_EVENT_OFFERED_DEADLINE_MISSED"},
    {RMW_EVENT_OFFERED_QOS_INCOMPATIBLE, true, "RMW_EVENT_OFFERED_QOS_INCOMPATIBLE"},
    {RMW_EVENT_LIVELINESS_LOST, true, "RMW_EVENT_LIVELINESS_LOST"},
    {RMW_EVENT_SUBSCRIPTION_MATCHED, true, "RMW_EVENT_SUBSCRIPTION_MATCHED"},
    {RMW_EVENT_REQUESTED_DEADLINE_MISSED, true, "RMW_EVENT_REQUESTED_DEADLINE_MISSED"},
    {RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, true, "RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE"},
    {RMW_EVENT_LIVELINESS_CHANGED, true, "RMW_EVENT_LIVELINESS_CHANGED"},
    {RMW_EVENT_MESSAGE_LOST, true, "RMW_EVENT_MESSAGE_LOST"},
#if RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT_API
    {RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE, true, "RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE"},
    {RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE, true, "RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE"},
#endif
  };

  for (const auto & check : checks) {
    const bool supported = rmw_event_type_is_supported(check.event_type);
    if (supported != check.expected) {
      std::cerr << "[ERROR] support matrix mismatch for " << check.name <<
        ": expected=" << check.expected << " actual=" << supported << std::endl;
      return false;
    }
  }

  std::cout << "[OK] support matrix validation passed" << std::endl;
  return true;
#else
  std::cout << "[INFO] support matrix validation skipped: matched-event support query is not available"
            << std::endl;
  return true;
#endif
}

bool
test_init_and_callback_smoke()
{
  TestContext ctx;
  if (!init_context(ctx, "event_init_pub_node", "event_init_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_init";
  const rmw_qos_profile_t default_qos = rmw_qos_profile_default;
  if (!create_publisher(ctx, topic_name, default_qos) || !create_subscription(ctx, topic_name, default_qos)) {
    cleanup_context(ctx);
    return false;
  }

  const rmw_event_type_t publisher_events[] = {
#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
    RMW_EVENT_PUBLICATION_MATCHED,
#endif
    RMW_EVENT_OFFERED_DEADLINE_MISSED,
    RMW_EVENT_OFFERED_QOS_INCOMPATIBLE,
    RMW_EVENT_LIVELINESS_LOST,
  };
  const rmw_event_type_t subscription_events[] = {
#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
    RMW_EVENT_SUBSCRIPTION_MATCHED,
#endif
    RMW_EVENT_REQUESTED_DEADLINE_MISSED,
    RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE,
    RMW_EVENT_LIVELINESS_CHANGED,
#if RMW_INT2DDS_HAS_MESSAGE_LOST_EVENT_API
    RMW_EVENT_MESSAGE_LOST,
#endif
  };

  for (rmw_event_type_t event_type : publisher_events) {
    rmw_event_t event = rmw_get_zero_initialized_event();
    if (!check_ret(rmw_publisher_event_init(&event, ctx.publisher, event_type), "publisher_event_init")) {
      cleanup_context(ctx);
      return false;
    }
#if RMW_INT2DDS_HAS_EVENT_CALLBACK_API
    if (!check_ret(
        rmw_event_set_callback(&event, noop_event_callback, reinterpret_cast<const void *>(0x1)),
        "rmw_event_set_callback(publisher)"))
    {
      cleanup_context(ctx);
      return false;
    }
    if (!check_ret(rmw_event_set_callback(&event, nullptr, nullptr), "rmw_event_set_callback(clear)")) {
      cleanup_context(ctx);
      return false;
    }
#endif
    ignore_ret(rmw_event_fini(&event));
  }

  for (rmw_event_type_t event_type : subscription_events) {
    rmw_event_t event = rmw_get_zero_initialized_event();
    if (!check_ret(
        rmw_subscription_event_init(&event, ctx.subscription, event_type),
        "subscription_event_init"))
    {
      cleanup_context(ctx);
      return false;
    }
#if RMW_INT2DDS_HAS_EVENT_CALLBACK_API
    if (!check_ret(
        rmw_event_set_callback(&event, noop_event_callback, reinterpret_cast<const void *>(0x2)),
        "rmw_event_set_callback(subscription)"))
    {
      cleanup_context(ctx);
      return false;
    }
    if (!check_ret(rmw_event_set_callback(&event, nullptr, nullptr), "rmw_event_set_callback(clear)")) {
      cleanup_context(ctx);
      return false;
    }
#endif
    ignore_ret(rmw_event_fini(&event));
  }

#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
  {
    rmw_event_t unsupported_publisher_event = rmw_get_zero_initialized_event();
    if (!expect_ret(
        rmw_publisher_event_init(
          &unsupported_publisher_event, ctx.publisher, RMW_EVENT_LIVELINESS_CHANGED),
        RMW_RET_UNSUPPORTED,
        "publisher unsupported event")) {
      cleanup_context(ctx);
      return false;
    }
  }
#endif

  cleanup_context(ctx);
  std::cout << "[OK] init/callback validation passed" << std::endl;
  return true;
}

bool
test_message_lost_take_no_event()
{
#if !RMW_INT2DDS_HAS_MESSAGE_LOST_EVENT_API
  std::cout << "[INFO] test_message_lost_take_no_event skipped: RMW_EVENT_MESSAGE_LOST is not part of this rmw API"
            << std::endl;
  return true;
#else
  TestContext ctx;
  if (!init_context(ctx, "message_lost_take_pub_node", "message_lost_take_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_message_lost_take";
  const rmw_qos_profile_t default_qos = rmw_qos_profile_default;
  if (!create_publisher(ctx, topic_name, default_qos) ||
    !create_subscription(ctx, topic_name, default_qos))
  {
    cleanup_context(ctx);
    return false;
  }

  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_subscription_event_init(&event, ctx.subscription, RMW_EVENT_MESSAGE_LOST),
      "rmw_subscription_event_init(message_lost)"))
  {
    cleanup_context(ctx);
    return false;
  }

  rmw_message_lost_status_t status{};
  bool taken = false;
  if (!check_ret(rmw_take_event(&event, &status, &taken), "rmw_take_event(message_lost)")) {
    ignore_ret(rmw_event_fini(&event));
    cleanup_context(ctx);
    return false;
  }

  // rmw_take_event is a level-triggered status read: it always reports
  // taken=true with the current status snapshot. With no message loss the
  // snapshot counts must still be zero.
  if (!taken || status.total_count != 0 || status.total_count_change != 0) {
    std::cerr << "[ERROR] message_lost no-loss take mismatch: taken=" << taken
              << " total_count=" << status.total_count
              << " total_count_change=" << status.total_count_change << std::endl;
    ignore_ret(rmw_event_fini(&event));
    cleanup_context(ctx);
    return false;
  }

  ignore_ret(rmw_event_fini(&event));
  cleanup_context(ctx);
  std::cout << "[OK] message_lost take no-loss validation passed" << std::endl;
  return true;
#endif
}

bool
test_message_lost_status_condition_mask()
{
#if !RMW_INT2DDS_HAS_MESSAGE_LOST_EVENT_API
  std::cout << "[INFO] test_message_lost_status_condition_mask skipped: RMW_EVENT_MESSAGE_LOST is not part of this rmw API"
            << std::endl;
  return true;
#else
  TestContext ctx;
  if (!init_context(ctx, "message_lost_mask_pub_node", "message_lost_mask_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_message_lost_mask";
  const rmw_qos_profile_t default_qos = rmw_qos_profile_default;
  if (!create_publisher(ctx, topic_name, default_qos) ||
    !create_subscription(ctx, topic_name, default_qos))
  {
    cleanup_context(ctx);
    return false;
  }

  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_subscription_event_init(&event, ctx.subscription, RMW_EVENT_MESSAGE_LOST),
      "rmw_subscription_event_init(message_lost_mask)"))
  {
    cleanup_context(ctx);
    return false;
  }

  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(ctx.subscription->data);
  if (sub_data == nullptr || sub_data->status_condition == nullptr) {
    std::cerr << "[ERROR] message_lost event has no status condition" << std::endl;
    ignore_ret(rmw_event_fini(&event));
    cleanup_context(ctx);
    return false;
  }

  uint32_t enabled_mask = 0;
  const Int2DdsRet ret = int2dds_statuscondition_get_enabled_statuses(
    sub_data->status_condition, &enabled_mask);
  if (ret != INT2DDS_RET_OK) {
    std::cerr << "[ERROR] failed to query message_lost status condition mask ret=" << ret
              << std::endl;
    ignore_ret(rmw_event_fini(&event));
    cleanup_context(ctx);
    return false;
  }

  if ((enabled_mask & INT2DDS_STATUS_SAMPLE_LOST) == 0) {
    std::cerr << "[ERROR] message_lost status condition mask missing SAMPLE_LOST"
              << " enabled_mask=" << enabled_mask << std::endl;
    ignore_ret(rmw_event_fini(&event));
    cleanup_context(ctx);
    return false;
  }

  ignore_ret(rmw_event_fini(&event));
  cleanup_context(ctx);
  std::cout << "[OK] message_lost status mask validation passed"
            << " enabled_mask=" << enabled_mask << std::endl;
  return true;
#endif
}

bool
test_subscription_matched_runtime()
{
#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
  TestContext ctx;
  if (!init_context(ctx, "sub_match_pub_node", "sub_match_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_subscription_matched";
  const rmw_qos_profile_t default_qos = rmw_qos_profile_default;
  if (!create_subscription(ctx, topic_name, default_qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_subscription_event_init(&event, ctx.subscription, RMW_EVENT_SUBSCRIPTION_MATCHED),
      "rmw_subscription_event_init(subscription_matched)")) {
    cleanup_context(ctx);
    return false;
  }

  if (!create_publisher(ctx, topic_name, default_qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_matched_status_t status{};
  const bool success = wait_for_event(
    event, ctx.wait_set, status,
    [](const rmw_matched_status_t & matched) {
      return matched.total_count >= 1 &&
             matched.total_count_change >= 1 &&
             matched.current_count >= 1 &&
             matched.current_count_change >= 1;
    },
    "subscription matched wait");

  if (success) {
    std::cout << "[OK] subscription matched runtime validation passed"
              << " total_count=" << status.total_count
              << " total_count_change=" << status.total_count_change
              << " current_count=" << status.current_count
              << " current_count_change=" << status.current_count_change
              << std::endl;
  }

  cleanup_context(ctx);
  return success;
#else
  std::cout << "[INFO] subscription matched runtime validation skipped: matched events are not available"
            << std::endl;
  return true;
#endif
}

bool
test_publication_matched_runtime()
{
#if RMW_INT2DDS_HAS_MATCHED_EVENT_API
  TestContext ctx;
  if (!init_context(ctx, "pub_match_pub_node", "pub_match_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_publication_matched";
  const rmw_qos_profile_t default_qos = rmw_qos_profile_default;
  if (!create_publisher(ctx, topic_name, default_qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_publisher_event_init(&event, ctx.publisher, RMW_EVENT_PUBLICATION_MATCHED),
      "rmw_publisher_event_init(publication_matched)")) {
    cleanup_context(ctx);
    return false;
  }

  if (!create_subscription(ctx, topic_name, default_qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_matched_status_t status{};
  const bool success = wait_for_event(
    event, ctx.wait_set, status,
    [](const rmw_matched_status_t & matched) {
      return matched.total_count >= 1 &&
             matched.total_count_change >= 1 &&
             matched.current_count >= 1 &&
             matched.current_count_change >= 1;
    },
    "publication matched wait");

  if (success) {
    std::cout << "[OK] publication matched runtime validation passed"
              << " total_count=" << status.total_count
              << " total_count_change=" << status.total_count_change
              << " current_count=" << status.current_count
              << " current_count_change=" << status.current_count_change
              << std::endl;
  }

  cleanup_context(ctx);
  return success;
#else
  std::cout << "[INFO] publication matched runtime validation skipped: matched events are not available"
            << std::endl;
  return true;
#endif
}

bool
test_requested_incompatible_qos_runtime()
{
  TestContext ctx;
  if (!init_context(ctx, "req_qos_pub_node", "req_qos_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_requested_qos";
  rmw_qos_profile_t publisher_qos = rmw_qos_profile_default;
  rmw_qos_profile_t subscription_qos = rmw_qos_profile_default;
  publisher_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  subscription_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  if (!create_subscription(ctx, topic_name, subscription_qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_subscription_event_init(&event, ctx.subscription, RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE),
      "rmw_subscription_event_init(requested_qos_incompatible)")) {
    cleanup_context(ctx);
    return false;
  }

  if (!create_publisher(ctx, topic_name, publisher_qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_requested_qos_incompatible_event_status_t status{};
  const bool success = wait_for_event(
    event, ctx.wait_set, status,
    [](const rmw_requested_qos_incompatible_event_status_t & qos_status) {
      return qos_status.total_count >= 1 &&
             qos_status.total_count_change >= 1 &&
             qos_status.last_policy_kind == RMW_QOS_POLICY_RELIABILITY;
    },
    "requested qos incompatible wait");

  if (success) {
    std::cout << "[OK] requested incompatible qos runtime validation passed"
              << " total_count=" << status.total_count
              << " total_count_change=" << status.total_count_change
              << " last_policy_kind=" << status.last_policy_kind
              << std::endl;
  }

  cleanup_context(ctx);
  return success;
}

bool
test_offered_incompatible_qos_runtime()
{
  TestContext ctx;
  if (!init_context(ctx, "off_qos_pub_node", "off_qos_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_offered_qos";
  rmw_qos_profile_t publisher_qos = rmw_qos_profile_default;
  rmw_qos_profile_t subscription_qos = rmw_qos_profile_default;
  publisher_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  subscription_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  if (!create_publisher(ctx, topic_name, publisher_qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_publisher_event_init(&event, ctx.publisher, RMW_EVENT_OFFERED_QOS_INCOMPATIBLE),
      "rmw_publisher_event_init(offered_qos_incompatible)")) {
    cleanup_context(ctx);
    return false;
  }

  if (!create_subscription(ctx, topic_name, subscription_qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_offered_qos_incompatible_event_status_t status{};
  const bool success = wait_for_event(
    event, ctx.wait_set, status,
    [](const rmw_offered_qos_incompatible_event_status_t & qos_status) {
      return qos_status.total_count >= 1 &&
             qos_status.total_count_change >= 1 &&
             qos_status.last_policy_kind == RMW_QOS_POLICY_RELIABILITY;
    },
    "offered qos incompatible wait");

  if (success) {
    std::cout << "[OK] offered incompatible qos runtime validation passed"
              << " total_count=" << status.total_count
              << " total_count_change=" << status.total_count_change
              << " last_policy_kind=" << status.last_policy_kind
              << std::endl;
  }

  cleanup_context(ctx);
  return success;
}

bool
test_requested_deadline_missed_runtime()
{
  TestContext ctx;
  if (!init_context(ctx, "req_deadline_pub_node", "req_deadline_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_requested_deadline";
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.deadline.sec = 0;
  qos.deadline.nsec = kDeadlineNs;

  if (!create_publisher(ctx, topic_name, qos) || !create_subscription(ctx, topic_name, qos)) {
    cleanup_context(ctx);
    return false;
  }
  if (!wait_for_subscription_matched(ctx, topic_name, "requested deadline pre-match")) {
    cleanup_context(ctx);
    return false;
  }

  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_subscription_event_init(&event, ctx.subscription, RMW_EVENT_REQUESTED_DEADLINE_MISSED),
      "rmw_subscription_event_init(requested_deadline_missed)")) {
    cleanup_context(ctx);
    return false;
  }

  if (!publish_once(ctx, "deadline-prime")) {
    cleanup_context(ctx);
    return false;
  }

  rmw_requested_deadline_missed_status_t status{};
  const bool success = wait_for_event(
    event, ctx.wait_set, status,
    [](const rmw_requested_deadline_missed_status_t & deadline_status) {
      return deadline_status.total_count >= 1 && deadline_status.total_count_change >= 1;
    },
    "requested deadline missed wait");

  if (success) {
    std::cout << "[OK] requested deadline missed runtime validation passed"
              << " total_count=" << status.total_count
              << " total_count_change=" << status.total_count_change
              << std::endl;
  }

  cleanup_context(ctx);
  return success;
}

bool
test_offered_deadline_missed_runtime()
{
  TestContext ctx;
  if (!init_context(ctx, "off_deadline_pub_node", "off_deadline_sub_node")) {
    return false;
  }

  const std::string topic_name = "/event_validation_offered_deadline";
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.deadline.sec = 0;
  qos.deadline.nsec = 500000000;

  if (!create_publisher(ctx, topic_name, qos)) {
    cleanup_context(ctx);
    return false;
  }

  rmw_event_t event = rmw_get_zero_initialized_event();
  if (!check_ret(
      rmw_publisher_event_init(&event, ctx.publisher, RMW_EVENT_OFFERED_DEADLINE_MISSED),
      "rmw_publisher_event_init(offered_deadline_missed)")) {
    cleanup_context(ctx);
    return false;
  }

  if (!publish_once(ctx, "deadline-prime-1")) {
    cleanup_context(ctx);
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  if (!publish_once(ctx, "deadline-prime-2")) {
    cleanup_context(ctx);
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  rmw_offered_deadline_missed_status_t status{};
  const bool success = wait_for_event(
    event, ctx.wait_set, status,
    [](const rmw_offered_deadline_missed_status_t & deadline_status) {
      return deadline_status.total_count >= 1 && deadline_status.total_count_change >= 1;
    },
    "offered deadline missed wait");

  if (!success) {
    print_offered_deadline_ffi_status(ctx);
  }

  if (success) {
    std::cout << "[OK] offered deadline missed runtime validation passed"
              << " total_count=" << status.total_count
              << " total_count_change=" << status.total_count_change
              << std::endl;
  }

  cleanup_context(ctx);
  return success;
}

}  // namespace

int
main()
{
  const bool success =
    test_support_matrix() &&
    test_init_and_callback_smoke() &&
    test_message_lost_status_condition_mask() &&
    test_message_lost_take_no_event() &&
    test_subscription_matched_runtime() &&
    test_publication_matched_runtime() &&
    test_requested_incompatible_qos_runtime() &&
    test_offered_incompatible_qos_runtime() &&
    test_requested_deadline_missed_runtime() &&
    test_offered_deadline_missed_runtime();

  if (!success) {
    std::cerr << "[FAIL] event validation suite failed" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "[OK] event validation suite passed" << std::endl;
  std::cout << "[INFO] liveliness events are covered by support/init/callback validation only" << std::endl;
  return EXIT_SUCCESS;
}
