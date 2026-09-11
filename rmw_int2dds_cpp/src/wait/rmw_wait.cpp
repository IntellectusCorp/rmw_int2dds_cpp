// Copyright 2024 Int2DDS Project
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

#include <cstring>
#include <atomic>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#if __has_include("rmw/events_statuses/incompatible_type.h")
#define RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT 1
#include "rmw/events_statuses/incompatible_type.h"
#endif

#include "int2dds-ffi.h"  // NOLINT(build/include)
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "waitset_registry.hpp"  // NOLINT(build/include)

namespace
{

struct WaitProfile
{
  std::atomic<uint64_t> count{0};
  std::atomic<uint64_t> ok_count{0};
  std::atomic<uint64_t> timeout_count{0};
  std::atomic<uint64_t> total_us{0};
  std::atomic<uint64_t> attach_us{0};
  std::atomic<uint64_t> wait_us{0};
  std::atomic<uint64_t> detach_us{0};
  std::atomic<uint64_t> ready_us{0};
};

WaitProfile g_wait_profile;

bool
profile_enabled()
{
  return std::getenv("RMW_INT2DDS_PROFILE") != nullptr;
}

uint64_t
elapsed_us(
  const std::chrono::steady_clock::time_point & start,
  const std::chrono::steady_clock::time_point & end)
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
}

void
record_wait_profile(
  uint64_t attach_us,
  uint64_t wait_us,
  uint64_t detach_us,
  uint64_t ready_us,
  uint64_t total_us,
  Int2DdsRet wait_ret)
{
  const uint64_t n = g_wait_profile.count.fetch_add(1, std::memory_order_relaxed) + 1;
  if (wait_ret == INT2DDS_RET_OK) {
    g_wait_profile.ok_count.fetch_add(1, std::memory_order_relaxed);
  } else if (wait_ret == INT2DDS_RET_TIMEOUT) {
    g_wait_profile.timeout_count.fetch_add(1, std::memory_order_relaxed);
  }
  g_wait_profile.attach_us.fetch_add(attach_us, std::memory_order_relaxed);
  g_wait_profile.wait_us.fetch_add(wait_us, std::memory_order_relaxed);
  g_wait_profile.detach_us.fetch_add(detach_us, std::memory_order_relaxed);
  g_wait_profile.ready_us.fetch_add(ready_us, std::memory_order_relaxed);
  g_wait_profile.total_us.fetch_add(total_us, std::memory_order_relaxed);

  if (n % 300 == 0) {
    const double divisor = static_cast<double>(n);
    std::fprintf(
      stderr,
      "RMW_INT2DDS_WAIT_PROFILE count=%lu ok=%lu timeout=%lu total_avg_us=%.3f attach_avg_us=%.3f wait_avg_us=%.3f detach_avg_us=%.3f ready_avg_us=%.3f\n",
      n,
      g_wait_profile.ok_count.load(std::memory_order_relaxed),
      g_wait_profile.timeout_count.load(std::memory_order_relaxed),
      static_cast<double>(g_wait_profile.total_us.load(std::memory_order_relaxed)) / divisor,
      static_cast<double>(g_wait_profile.attach_us.load(std::memory_order_relaxed)) / divisor,
      static_cast<double>(g_wait_profile.wait_us.load(std::memory_order_relaxed)) / divisor,
      static_cast<double>(g_wait_profile.detach_us.load(std::memory_order_relaxed)) / divisor,
      static_cast<double>(g_wait_profile.ready_us.load(std::memory_order_relaxed)) / divisor);
  }
}

uint32_t
event_type_to_status_mask(rmw_event_type_t event_type)
{
  // No RMW_EVENT_MESSAGE_LOST case: that event is Galactic+ and Foxy has none.
  switch (event_type) {
    case RMW_EVENT_LIVELINESS_LOST:
      return INT2DDS_STATUS_LIVELINESS_LOST;
    case RMW_EVENT_OFFERED_DEADLINE_MISSED:
      return INT2DDS_STATUS_OFFERED_DEADLINE_MISSED;
    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE:
      return INT2DDS_STATUS_OFFERED_INCOMPATIBLE_QOS;
#ifdef RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT
    case RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE:
      return INT2DDS_STATUS_OFFERED_INCOMPATIBLE_TYPE;
#endif
#ifdef RMW_EVENT_PUBLICATION_MATCHED
    case RMW_EVENT_PUBLICATION_MATCHED:
      return INT2DDS_STATUS_PUBLICATION_MATCHED;
#endif
    case RMW_EVENT_LIVELINESS_CHANGED:
      return INT2DDS_STATUS_LIVELINESS_CHANGED;
    case RMW_EVENT_REQUESTED_DEADLINE_MISSED:
      return INT2DDS_STATUS_REQUESTED_DEADLINE_MISSED;
    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE:
      return INT2DDS_STATUS_REQUESTED_INCOMPATIBLE_QOS;
#ifdef RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT
    case RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE:
      return INT2DDS_STATUS_REQUESTED_INCOMPATIBLE_TYPE;
#endif
#ifdef RMW_EVENT_SUBSCRIPTION_MATCHED
    case RMW_EVENT_SUBSCRIPTION_MATCHED:
      return INT2DDS_STATUS_SUBSCRIPTION_MATCHED;
#endif
    default:
      return 0;
  }
}

/// Convert RMW timeout to nanoseconds
/// Returns -1 for infinite wait, 0 for poll, positive for actual timeout
int64_t
convert_timeout_to_ns(const rmw_time_t * wait_timeout)
{
  if (wait_timeout == nullptr) {
    return -1;  // Infinite wait
  }

  if (wait_timeout->sec == 0 && wait_timeout->nsec == 0) {
    return 0;  // Poll (no wait)
  }

  // Nanosecond resolution: int2dds_waitset_wait_ns honors sub-millisecond
  // timeouts, so an rcl timer's exact deadline is observed without rounding up
  // (rounding up made sub-ms timers fire one period early).
  if (wait_timeout->sec >= INT64_MAX / 1000000000 - 1) {
    return -1;  // Effectively infinite; avoid overflow
  }
  return static_cast<int64_t>(wait_timeout->sec) * 1000000000 +
         static_cast<int64_t>(wait_timeout->nsec);
}

/// Check if a subscription has data available
bool
subscription_has_data(rmw_int2dds_cpp::SubscriptionData * sub_data)
{
  if (sub_data == nullptr || sub_data->datareader == nullptr) {
    return false;
  }
  bool has_cached_data = false;
  Int2DdsRet cache_ret = int2dds_datareader_has_data(
    sub_data->datareader, &has_cached_data);
  if (cache_ret != INT2DDS_RET_OK || !has_cached_data) {
    return false;
  }
  if (!sub_data->ignore_local_publications || sub_data->node_data == nullptr) {
    return true;
  }
  // ignore_local_publications drops same-node samples at take time, not on
  // reception, so the reader keeps a DATA_AVAILABLE trigger for a local sample
  // this subscription will discard. That wakes rmw_wait on the local sample, and
  // a sibling subscription that must receive it can be reported not-ready before
  // its own copy is cached. Drain the leading local samples here -- they would be
  // dropped on take anyway: peek each front sample's origin, take-and-drop it only
  // when it is local, and stop at the first genuinely remote sample (left in the
  // cache for the application). Draining clears the trigger, so a local-only queue
  // leaves this subscription not-ready and rmw_wait blocks until real data
  // arrives. read/take share take_buffer_mutex with rmw_take so the application's
  // take never interleaves; the lock order (buffer then node) matches rmw_take.
  constexpr size_t kPeekBufferSize = 64 * 1024;
  std::lock_guard<std::mutex> buffer_lock(sub_data->take_buffer_mutex);
  if (sub_data->take_buffer.size() < kPeekBufferSize) {
    sub_data->take_buffer.resize(kPeekBufferSize);
  }
  for (;; ) {
    uintptr_t peek_size = 0;
    Int2DdsSampleInfo info;
    Int2DdsRet read_ret = int2dds_datareader_read_serialized_w_info(
      sub_data->datareader, sub_data->take_buffer.data(), sub_data->take_buffer.size(),
      &peek_size, &info);
    if (read_ret == INT2DDS_RET_NO_DATA) {
      return false;  // only local samples were queued; all drained
    }
    if (read_ret != INT2DDS_RET_OK) {
      return true;  // cannot inspect (e.g. sample larger than buffer): never hide data
    }
    bool local = false;
    {
      std::lock_guard<std::mutex> node_lock(sub_data->node_data->entities_mutex);
      for (const auto & pub_gid : sub_data->node_data->publishers) {
        if (memcmp(
            pub_gid.data, info.publication_handle, sizeof(info.publication_handle)) == 0)
        {
          local = true;
          break;
        }
      }
    }
    if (!local) {
      return true;  // genuine remote data is pending (not consumed)
    }
    uintptr_t take_size = 0;
    Int2DdsSampleInfo take_info;
    Int2DdsRet take_ret = int2dds_datareader_take_serialized_w_info(
      sub_data->datareader, sub_data->take_buffer.data(), sub_data->take_buffer.size(),
      &take_size, &take_info);
    if (take_ret != INT2DDS_RET_OK) {
      return true;  // could not drop the local sample: never hide data
    }
    // Loop to inspect the next front sample.
  }
}

/// Check if a guard condition is triggered
bool
guard_condition_is_triggered(const rmw_int2dds_cpp::GuardConditionData * gc_data)
{
  if (gc_data == nullptr || gc_data->guard_condition == nullptr) {
    return false;
  }

  bool triggered = false;
  Int2DdsRet ret = int2dds_guardcondition_get_trigger_value(gc_data->guard_condition, &triggered);
  if (ret != INT2DDS_RET_OK) {
    return false;
  }

  // Reset trigger value after checking (like taking the trigger)
  if (triggered) {
    int2dds_guardcondition_set_trigger_value(gc_data->guard_condition, false);
  }

  return triggered;
}

/// Read a guard condition trigger without consuming it
///
/// guard_condition_is_triggered clears the trigger as a side effect, so it
/// cannot be used before the blocking wait: consuming a trigger up front would
/// leave int2dds_waitset_wait_ex_ns blocked on a condition that is no longer
/// set. This peek leaves the trigger in place for the post-wait scan to consume
/// exactly once.
bool
guard_condition_peek(const rmw_int2dds_cpp::GuardConditionData * gc_data)
{
  if (gc_data == nullptr || gc_data->guard_condition == nullptr) {
    return false;
  }

  bool triggered = false;
  Int2DdsRet ret = int2dds_guardcondition_get_trigger_value(gc_data->guard_condition, &triggered);
  if (ret != INT2DDS_RET_OK) {
    return false;
  }
  return triggered;
}

/// Check if a service has a request available
bool
service_has_request(const rmw_int2dds_cpp::ServiceData * srv_data)
{
  if (srv_data == nullptr || srv_data->request_reader == nullptr) {
    return false;
  }
  bool has_cached_data = false;
  Int2DdsRet cache_ret = int2dds_datareader_has_data(
    srv_data->request_reader, &has_cached_data);
  if (cache_ret == INT2DDS_RET_OK && has_cached_data) {
    return true;
  }
  return false;
}

/// Check if a client has a response available
bool
client_has_response(const rmw_int2dds_cpp::ClientData * cli_data)
{
  if (cli_data == nullptr || cli_data->response_reader == nullptr) {
    return false;
  }
  bool has_cached_data = false;
  Int2DdsRet cache_ret = int2dds_datareader_has_data(
    cli_data->response_reader, &has_cached_data);
  if (cache_ret == INT2DDS_RET_OK && has_cached_data) {
    return true;
  }
  return false;
}

/// Check if an event status condition is triggered
bool
event_is_triggered(const rmw_event_t * event)
{
  auto * event_data = static_cast<rmw_int2dds_cpp::EventData *>(event->data);
  if (event_data == nullptr) {
    return false;
  }

  uint32_t target_mask = event_type_to_status_mask(event_data->event_type);
  if (target_mask == 0) {
    return false;
  }

  uint32_t status_changes = 0;
  Int2DdsRet ret = INT2DDS_RET_ERROR;
  if (event_data->is_publisher) {
    auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(event_data->entity_data);
    if (pub_data == nullptr || pub_data->datawriter == nullptr) {
      return false;
    }
    ret = int2dds_datawriter_get_status_changes(pub_data->datawriter, &status_changes);
  } else {
    auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(event_data->entity_data);
    if (sub_data == nullptr || sub_data->datareader == nullptr) {
      return false;
    }
#ifdef RMW_INT2DDS_HAS_MATCHED_EVENT
    if (event_data->event_type == RMW_EVENT_SUBSCRIPTION_MATCHED) {
      std::lock_guard<std::mutex> lock(sub_data->matched_mutex);
      return sub_data->matched_changed;
    }
#endif
    ret = int2dds_datareader_get_status_changes(sub_data->datareader, &status_changes);
  }

  return ret == INT2DDS_RET_OK && (status_changes & target_mask) != 0;
}

bool
ensure_status_condition_mask(
  Int2DdsDataReader * datareader,
  Int2DdsStatusCondition ** cached_status_condition,
  uint32_t required_mask)
{
  if (datareader == nullptr || cached_status_condition == nullptr) {
    return false;
  }

  if (*cached_status_condition == nullptr) {
    Int2DdsRet ret = int2dds_datareader_get_statuscondition(datareader, cached_status_condition);
    if (ret != INT2DDS_RET_OK) {
      return false;
    }
    ret = int2dds_statuscondition_set_enabled_statuses(*cached_status_condition, 0u);
    if (ret != INT2DDS_RET_OK) {
      return false;
    }
  }

  uint32_t enabled_mask = 0;
  Int2DdsRet ret = int2dds_statuscondition_get_enabled_statuses(
    *cached_status_condition, &enabled_mask);
  if (ret != INT2DDS_RET_OK) {
    return false;
  }

  if ((enabled_mask & required_mask) == required_mask) {
    return true;
  }

  ret = int2dds_statuscondition_set_enabled_statuses(
    *cached_status_condition, enabled_mask | required_mask);
  return ret == INT2DDS_RET_OK;
}

Int2DdsStatusCondition *
refresh_event_status_condition(rmw_int2dds_cpp::EventData * event_data)
{
  if (event_data == nullptr) {
    return nullptr;
  }

  // int2dds_*_get_statuscondition allocates a new FFI handle on every call and
  // the handle must be released with int2dds_statuscondition_delete. Keep
  // exactly one handle per entity, owned by the entity data and released in
  // the entity destroy path; events only borrow it. Resolving through the
  // entity data on every call (instead of caching per event) keeps the borrow
  // valid when destroy_subscription_reader_entities recreates the reader for a
  // content-filter update.
  Int2DdsStatusCondition * status_condition = nullptr;
  if (event_data->is_publisher) {
    auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(event_data->entity_data);
    if (pub_data == nullptr || pub_data->datawriter == nullptr) {
      return nullptr;
    }
    if (pub_data->status_condition == nullptr) {
      Int2DdsRet ret =
        int2dds_datawriter_get_statuscondition(pub_data->datawriter, &status_condition);
      if (ret != INT2DDS_RET_OK || status_condition == nullptr) {
        return nullptr;
      }
      pub_data->status_condition = status_condition;
    }
    status_condition = pub_data->status_condition;
  } else {
    auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(event_data->entity_data);
    if (sub_data == nullptr || sub_data->datareader == nullptr) {
      return nullptr;
    }
    if (sub_data->status_condition == nullptr) {
      Int2DdsRet ret =
        int2dds_datareader_get_statuscondition(sub_data->datareader, &status_condition);
      if (ret != INT2DDS_RET_OK || status_condition == nullptr) {
        return nullptr;
      }
      sub_data->status_condition = status_condition;
    }
    status_condition = sub_data->status_condition;
  }

  uint32_t status_mask = event_type_to_status_mask(event_data->event_type);
  if (status_mask == 0) {
    return nullptr;
  }

  uint32_t enabled_mask = 0;
  if (
    int2dds_statuscondition_get_enabled_statuses(status_condition, &enabled_mask) != INT2DDS_RET_OK)
  {
    return nullptr;
  }

  if ((enabled_mask & status_mask) == status_mask) {
    return status_condition;
  }

  if (
    int2dds_statuscondition_set_enabled_statuses(
      status_condition, enabled_mask | status_mask) != INT2DDS_RET_OK)
  {
    return nullptr;
  }

  return status_condition;
}

/// Level-triggered readiness scan over every entity handed to rmw_wait
///
/// Every probe reads entity state directly, so the scan needs nothing attached
/// to the wait set. It covers the same ground the wait set does: reader status
/// conditions map to int2dds_datareader_has_data, event status conditions to
/// event_is_triggered, and guard conditions to guard_condition_peek. Running it
/// before attaching lets the common "something is already pending" case skip the
/// attach/wait/detach round trip entirely, and WaitSet::wait re-evaluates
/// trigger values on entry, so deferring the attach cannot lose a wake-up.
///
/// Guard conditions are peeked, never consumed: the post-wait scan owns the
/// single allowed consumption per call.
bool
any_entity_ready(
  const rmw_subscriptions_t * subscriptions,
  const rmw_guard_conditions_t * guard_conditions,
  const rmw_services_t * services,
  const rmw_clients_t * clients,
  const rmw_events_t * events)
{
  if (subscriptions != nullptr) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      if (subscriptions->subscribers[i] != nullptr) {
        auto * sub_data =
          static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscriptions->subscribers[i]);
        if (subscription_has_data(sub_data)) {
          return true;
        }
      }
    }
  }

  if (guard_conditions != nullptr) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      if (guard_conditions->guard_conditions[i] != nullptr) {
        auto * gc_data =
          static_cast<rmw_int2dds_cpp::GuardConditionData *>(guard_conditions->guard_conditions[i]);
        if (guard_condition_peek(gc_data)) {
          return true;
        }
      }
    }
  }

  if (services != nullptr) {
    for (size_t i = 0; i < services->service_count; ++i) {
      if (services->services[i] != nullptr) {
        auto * srv_data = static_cast<rmw_int2dds_cpp::ServiceData *>(services->services[i]);
        if (service_has_request(srv_data)) {
          return true;
        }
      }
    }
  }

  if (clients != nullptr) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      if (clients->clients[i] != nullptr) {
        auto * cli_data = static_cast<rmw_int2dds_cpp::ClientData *>(clients->clients[i]);
        if (client_has_response(cli_data)) {
          return true;
        }
      }
    }
  }

  if (events != nullptr) {
    for (size_t i = 0; i < events->event_count; ++i) {
      if (events->events[i] != nullptr) {
        auto * event = static_cast<rmw_event_t *>(events->events[i]);
        if (event->implementation_identifier == rmw_int2dds_cpp::implementation_identifier &&
          event_is_triggered(event))
        {
          return true;
        }
      }
    }
  }

  return false;
}

/// Attach every entity handed to rmw_wait to the wait set
///
/// True when the incoming entity array differs from the one cached on the
/// previous rmw_wait call, meaning the waitset attachments must be rebuilt.
bool
require_reattach(const std::vector<void *> & cached, size_t count, void ** ary)
{
  if (ary == nullptr || count == 0) {
    return !cached.empty();
  }

  if (count != cached.size()) {
    return true;
  }

  return std::memcmp(cached.data(), ary, count * sizeof(void *)) != 0;
}

bool
require_reattach_events(
  const std::vector<rmw_int2dds_cpp::WaitSetCachedEvent> & cached, const rmw_events_t * events)
{
  const size_t count = events != nullptr ? events->event_count : 0;

  if (count != cached.size()) {
    return true;
  }

  for (size_t i = 0; i < count; ++i) {
    auto * event = static_cast<rmw_event_t *>(events->events[i]);
    if (event == nullptr || cached[i].entity_data != event->data ||
      cached[i].event_type != event->event_type)
    {
      return true;
    }
  }

  return false;
}

// Attach the condition if new, then stamp it with the current generation so
// detach_stale_attachments keeps it. False when the FFI attach failed.
bool
ensure_condition_attached(
  rmw_int2dds_cpp::WaitSetData * ws_data, Int2DdsStatusCondition * condition)
{
  auto it = ws_data->attached_conditions.find(condition);
  if (it != ws_data->attached_conditions.end()) {
    it->second = ws_data->attach_generation;
    return true;
  }

  if (int2dds_waitset_attach_statuscondition(ws_data->waitset, condition) == INT2DDS_RET_OK) {
    ws_data->attached_conditions.emplace(condition, ws_data->attach_generation);
    return true;
  }
  return false;
}

// ensure_condition_attached for a guard condition.
bool
ensure_guard_attached(
  rmw_int2dds_cpp::WaitSetData * ws_data, Int2DdsGuardCondition * guard)
{
  auto it = ws_data->attached_guards.find(guard);
  if (it != ws_data->attached_guards.end()) {
    it->second = ws_data->attach_generation;
    return true;
  }

  if (int2dds_waitset_attach_guardcondition(ws_data->waitset, guard) == INT2DDS_RET_OK) {
    ws_data->attached_guards.emplace(guard, ws_data->attach_generation);
    return true;
  }
  return false;
}

// Detach conditions left with an older generation, the leftovers from entities
// dropped since the last rebuild.
void
detach_stale_attachments(rmw_int2dds_cpp::WaitSetData * ws_data)
{
  const uint64_t generation = ws_data->attach_generation;

  for (auto it = ws_data->attached_conditions.begin();
    it != ws_data->attached_conditions.end(); )
  {
    if (it->second != generation) {
      int2dds_waitset_detach_statuscondition(ws_data->waitset, it->first);
      it = ws_data->attached_conditions.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = ws_data->attached_guards.begin(); it != ws_data->attached_guards.end(); ) {
    if (it->second != generation) {
      int2dds_waitset_detach_guardcondition(ws_data->waitset, it->first);
      it = ws_data->attached_guards.erase(it);
    } else {
      ++it;
    }
  }
}

// Attach new conditions, re-stamp existing ones, and record the incoming arrays
// for the next identity check. Widening a mask here must precede the wait, since
// int2dds_statuscondition_set_enabled_statuses does not wake a blocked wait set.
void
stamp_desired_attachments(
  rmw_int2dds_cpp::WaitSetData * ws_data,
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events)
{
  ws_data->cached_subscriptions.clear();
  ws_data->cached_guard_conditions.clear();
  ws_data->cached_services.clear();
  ws_data->cached_clients.clear();
  ws_data->cached_events.clear();

  bool all_attached = true;

  // Attach subscriptions
  if (subscriptions != nullptr) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      ws_data->cached_subscriptions.push_back(subscriptions->subscribers[i]);
      if (subscriptions->subscribers[i] != nullptr) {
        auto * sub_data =
          static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscriptions->subscribers[i]);
        if (sub_data != nullptr && sub_data->datareader != nullptr) {
          bool ready = ensure_status_condition_mask(
            sub_data->datareader, &sub_data->status_condition, INT2DDS_STATUS_DATA_AVAILABLE);
          if (ready && sub_data->status_condition != nullptr) {
            all_attached =
              ensure_condition_attached(ws_data, sub_data->status_condition) && all_attached;
          } else {
            all_attached = false;
          }
        } else {
          // reader momentarily null: rebuild attachments on the next rmw_wait
          all_attached = false;
        }
      }
    }
  }

  // Attach guard conditions
  if (guard_conditions != nullptr) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      ws_data->cached_guard_conditions.push_back(guard_conditions->guard_conditions[i]);
      if (guard_conditions->guard_conditions[i] != nullptr) {
        auto * gc_data =
          static_cast<rmw_int2dds_cpp::GuardConditionData *>(guard_conditions->guard_conditions[i]);
        if (gc_data != nullptr && gc_data->guard_condition != nullptr) {
          all_attached = ensure_guard_attached(ws_data, gc_data->guard_condition) && all_attached;
        }
      }
    }
  }

  // Attach services (request readers)
  if (services != nullptr) {
    for (size_t i = 0; i < services->service_count; ++i) {
      ws_data->cached_services.push_back(services->services[i]);
      if (services->services[i] != nullptr) {
        auto * srv_data = static_cast<rmw_int2dds_cpp::ServiceData *>(services->services[i]);
        if (srv_data != nullptr && srv_data->request_reader != nullptr) {
          bool ready = ensure_status_condition_mask(
            srv_data->request_reader,
            &srv_data->request_status_condition,
            INT2DDS_STATUS_DATA_AVAILABLE);
          if (ready && srv_data->request_status_condition != nullptr) {
            all_attached =
              ensure_condition_attached(ws_data, srv_data->request_status_condition) &&
              all_attached;
          } else {
            all_attached = false;
          }
        } else {
          // reader momentarily null: rebuild attachments on the next rmw_wait
          all_attached = false;
        }
      }
    }
  }

  // Attach clients (response readers)
  if (clients != nullptr) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      ws_data->cached_clients.push_back(clients->clients[i]);
      if (clients->clients[i] != nullptr) {
        auto * cli_data = static_cast<rmw_int2dds_cpp::ClientData *>(clients->clients[i]);
        if (cli_data != nullptr && cli_data->response_reader != nullptr) {
          bool ready = ensure_status_condition_mask(
            cli_data->response_reader,
            &cli_data->response_status_condition,
            INT2DDS_STATUS_DATA_AVAILABLE);
          if (ready && cli_data->response_status_condition != nullptr) {
            all_attached =
              ensure_condition_attached(ws_data, cli_data->response_status_condition) &&
              all_attached;
          } else {
            all_attached = false;
          }
        } else {
          // reader momentarily null: rebuild attachments on the next rmw_wait
          all_attached = false;
        }
      }
    }
  }

  // Attach events (status conditions)
  if (events != nullptr) {
    for (size_t i = 0; i < events->event_count; ++i) {
      auto * event = static_cast<rmw_event_t *>(events->events[i]);

      rmw_int2dds_cpp::WaitSetCachedEvent cached_event;
      if (event != nullptr) {
        cached_event.entity_data = event->data;
        cached_event.event_type = event->event_type;
      }
      ws_data->cached_events.push_back(cached_event);

      if (event == nullptr ||
        event->implementation_identifier != rmw_int2dds_cpp::implementation_identifier)
      {
        continue;
      }
      auto * event_data = static_cast<rmw_int2dds_cpp::EventData *>(event->data);
      Int2DdsStatusCondition * status_condition = refresh_event_status_condition(event_data);
      if (status_condition != nullptr) {
        all_attached = ensure_condition_attached(ws_data, status_condition) && all_attached;
      } else if (event_data != nullptr &&
        event_type_to_status_mask(event_data->event_type) != 0)
      {
        // The event type is supported, so a null condition is transient (e.g. the
        // entity's reader/writer is momentarily null while a content-filter update
        // recreates it). Flag unattached so the next rmw_wait rebuilds and retries.
        // A null for an unsupported type (mask == 0) is permanent and is left
        // unflagged so it does not force a rebuild every call.
        all_attached = false;
      }
    }
  }

  if (!all_attached) {
    // An attach failed. Drop the cache so the next call rebuilds and retries,
    // restoring the pre-cache behavior of attaching afresh on every call.
    ws_data->cached_subscriptions.clear();
    ws_data->cached_guard_conditions.clear();
    ws_data->cached_services.clear();
    ws_data->cached_clients.clear();
    ws_data->cached_events.clear();
  }
}

}  // namespace

extern "C"
{

rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  const bool profile = profile_enabled();
  const auto total_t0 = std::chrono::steady_clock::now();
  uint64_t attach_elapsed_us = 0;
  uint64_t wait_elapsed_us = 0;
  uint64_t detach_elapsed_us = 0;
  uint64_t ready_elapsed_us = 0;

  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);

  if (wait_set->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("wait set not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * ws_data = static_cast<rmw_int2dds_cpp::WaitSetData *>(wait_set->data);
  if (ws_data == nullptr || ws_data->waitset == nullptr) {
    RMW_SET_ERROR_MSG("wait set data is null");
    return RMW_RET_ERROR;
  }

  // inuse rejects concurrent rmw_wait, which the rmw contract does not allow.
  // cache_busy marks the sections that read or rewrite the cache/attachments;
  // entity-destroy cache cleaning waits it out instead of skipping, so a
  // destroy can never delete a condition handle this wait set still holds. It
  // is cleared before the blocking FFI wait: by then the attachments mirror
  // the current entity set, which a concurrent destroy cannot touch.
  {
    std::lock_guard<std::mutex> lock(ws_data->lock);
    if (ws_data->inuse) {
      RMW_SET_ERROR_MSG("concurrent calls to rmw_wait on a single wait set are not supported");
      return RMW_RET_ERROR;
    }
    ws_data->inuse = true;
    ws_data->cache_busy = true;
  }

  struct InUseGuard
  {
    rmw_int2dds_cpp::WaitSetData * ws_data;
    ~InUseGuard()
    {
      {
        std::lock_guard<std::mutex> lock(ws_data->lock);
        ws_data->inuse = false;
        ws_data->cache_busy = false;
      }
      ws_data->cache_cv.notify_all();
    }
  } inuse_guard{ws_data};

  // Convert timeout
  const int64_t timeout_ns = convert_timeout_to_ns(wait_timeout);

  // Decide readiness before touching the wait set. Every probe reads entity
  // state directly, so nothing has to be attached for the scan to be accurate,
  // and WaitSet::wait re-evaluates trigger values on entry, so attaching later
  // cannot lose a wake-up. When something is already pending - the common case
  // under load - this skips the attach/wait/detach round trip entirely, which
  // also avoids leaving a stale trigger flag behind on the wait set.
  const auto pre_scan_t0 = std::chrono::steady_clock::now();
  const bool data_already_ready =
    any_entity_ready(subscriptions, guard_conditions, services, clients, events);
  if (profile) {
    ready_elapsed_us = elapsed_us(pre_scan_t0, std::chrono::steady_clock::now());
  }

  Int2DdsRet wait_ret = INT2DDS_RET_OK;
  if (!data_already_ready) {
    if (timeout_ns == 0) {
      // Poll: the scan above already covers every condition the wait set could
      // report, so a zero timeout wait cannot observe anything it missed.
      wait_ret = INT2DDS_RET_TIMEOUT;
    } else {
      // The executor passes the same entity set on almost every call. Rebuild
      // the attachments only when the set changed since the previous call.
      // Otherwise the conditions attached last time are still in place.
      const auto attach_t0 = std::chrono::steady_clock::now();
      if (
        require_reattach(
          ws_data->cached_subscriptions,
          subscriptions != nullptr ? subscriptions->subscriber_count : 0,
          subscriptions != nullptr ? subscriptions->subscribers : nullptr) ||
        require_reattach(
          ws_data->cached_guard_conditions,
          guard_conditions != nullptr ? guard_conditions->guard_condition_count : 0,
          guard_conditions != nullptr ? guard_conditions->guard_conditions : nullptr) ||
        require_reattach(
          ws_data->cached_services,
          services != nullptr ? services->service_count : 0,
          services != nullptr ? services->services : nullptr) ||
        require_reattach(
          ws_data->cached_clients,
          clients != nullptr ? clients->client_count : 0,
          clients != nullptr ? clients->clients : nullptr) ||
        require_reattach_events(ws_data->cached_events, events))
      {
        ++ws_data->attach_generation;
        stamp_desired_attachments(
          ws_data, subscriptions, guard_conditions, services, clients, events);
        const auto detach_t0 = std::chrono::steady_clock::now();
        detach_stale_attachments(ws_data);
        if (profile) {
          detach_elapsed_us = elapsed_us(detach_t0, std::chrono::steady_clock::now());
        }
      }
      if (profile) {
        attach_elapsed_us = elapsed_us(attach_t0, std::chrono::steady_clock::now());
      }

      // Attachments now mirror the current entity set, so a destroy of any
      // entity not in it finds nothing of that entity cached here: cleaners
      // may run or skip this wait set while it blocks. Release them before
      // the wait so a destroy never stalls for the wait timeout.
      {
        std::lock_guard<std::mutex> lock(ws_data->lock);
        ws_data->cache_busy = false;
      }
      ws_data->cache_cv.notify_all();

      // waitset_wait_ex_ns was consolidated into wait_ex_ns, which returns the triggered
      // conditions; the scan below re-checks conditions itself, so free the sequence.
      const auto wait_t0 = std::chrono::steady_clock::now();
      Int2DdsConditionSeq * triggered = nullptr;
      wait_ret = int2dds_waitset_wait_ex_ns(ws_data->waitset, timeout_ns, &triggered);
      if (triggered) {
        int2dds_condition_seq_delete(triggered);
      }
      if (profile) {
        wait_elapsed_us = elapsed_us(wait_t0, std::chrono::steady_clock::now());
      }

      // Attachments stay in place for the next call; they are only rebuilt
      // when the entity set changes or an entity is destroyed.

      // A wake with nothing genuinely ready happens when an ignore_local
      // subscription's DATA_AVAILABLE trigger was already latched for a local
      // sample: the readiness scan drains and rejects that sample, but the wake
      // itself already returned. Returning here would report a sibling
      // subscription not-ready before its own copy is cached. Bounded-poll the
      // readiness scan until something is ready or the caller's timeout elapses.
      // The scan drains freshly-arrived local samples each pass, so this cannot
      // spin on a latched trigger, and the hard deadline bounds it absolutely.
      if (wait_ret == INT2DDS_RET_OK && timeout_ns > 0) {
        const auto deadline = total_t0 + std::chrono::nanoseconds(timeout_ns);
        while (!any_entity_ready(subscriptions, guard_conditions, services, clients, events)) {
          if (std::chrono::steady_clock::now() >= deadline) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }
    }
  }

  // Check for timeout
  if (wait_ret == INT2DDS_RET_TIMEOUT) {
    // Nullify all entries - nothing was triggered
    if (subscriptions != nullptr) {
      for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
        subscriptions->subscribers[i] = nullptr;
      }
    }
    if (guard_conditions != nullptr) {
      for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
        guard_conditions->guard_conditions[i] = nullptr;
      }
    }
    if (services != nullptr) {
      for (size_t i = 0; i < services->service_count; ++i) {
        services->services[i] = nullptr;
      }
    }
    if (clients != nullptr) {
      for (size_t i = 0; i < clients->client_count; ++i) {
        clients->clients[i] = nullptr;
      }
    }
    if (events != nullptr) {
      for (size_t i = 0; i < events->event_count; ++i) {
        events->events[i] = nullptr;
      }
    }
    if (profile) {
      record_wait_profile(
        attach_elapsed_us, wait_elapsed_us, detach_elapsed_us, ready_elapsed_us,
        elapsed_us(total_t0, std::chrono::steady_clock::now()), wait_ret);
    }
    return RMW_RET_TIMEOUT;
  }

  if (wait_ret != INT2DDS_RET_OK) {
    if (profile) {
      record_wait_profile(
        attach_elapsed_us, wait_elapsed_us, detach_elapsed_us, ready_elapsed_us,
        elapsed_us(total_t0, std::chrono::steady_clock::now()), wait_ret);
    }
    RMW_SET_ERROR_MSG("wait failed");
    return RMW_RET_ERROR;
  }

  // Check which entities have data/are triggered
  // Nullify entries that don't have data

  // Check subscriptions
  const auto ready_t0 = std::chrono::steady_clock::now();
  if (subscriptions != nullptr) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      if (subscriptions->subscribers[i] != nullptr) {
        auto * sub_data =
          static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscriptions->subscribers[i]);
        bool has_data = subscription_has_data(sub_data);
        if (!has_data) {
          subscriptions->subscribers[i] = nullptr;
        }
      }
    }
  }

  // Check guard conditions
  if (guard_conditions != nullptr) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      if (guard_conditions->guard_conditions[i] != nullptr) {
        auto * gc_data =
          static_cast<rmw_int2dds_cpp::GuardConditionData *>(guard_conditions->guard_conditions[i]);
        if (!guard_condition_is_triggered(gc_data)) {
          guard_conditions->guard_conditions[i] = nullptr;
        }
      }
    }
  }

  // Check services
  if (services != nullptr) {
    for (size_t i = 0; i < services->service_count; ++i) {
      if (services->services[i] != nullptr) {
        auto * srv_data = static_cast<rmw_int2dds_cpp::ServiceData *>(services->services[i]);
        if (!service_has_request(srv_data)) {
          services->services[i] = nullptr;
        }
      }
    }
  }

  // Check clients
  if (clients != nullptr) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      if (clients->clients[i] != nullptr) {
        auto * cli_data = static_cast<rmw_int2dds_cpp::ClientData *>(clients->clients[i]);
        if (!client_has_response(cli_data)) {
          clients->clients[i] = nullptr;
        }
      }
    }
  }

  if (events != nullptr) {
    for (size_t i = 0; i < events->event_count; ++i) {
      if (events->events[i] != nullptr) {
        auto * event = static_cast<rmw_event_t *>(events->events[i]);
        if (event->implementation_identifier != rmw_int2dds_cpp::implementation_identifier ||
          !event_is_triggered(event))
        {
          events->events[i] = nullptr;
        }
      }
    }
  }

  if (profile) {
    // ready_elapsed_us already holds the pre-wait scan; add the post-wait one.
    ready_elapsed_us += elapsed_us(ready_t0, std::chrono::steady_clock::now());
    record_wait_profile(
      attach_elapsed_us, wait_elapsed_us, detach_elapsed_us, ready_elapsed_us,
      elapsed_us(total_t0, std::chrono::steady_clock::now()), wait_ret);
  }

  return RMW_RET_OK;
}

}  // extern "C"
