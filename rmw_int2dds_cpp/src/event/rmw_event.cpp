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

#include <algorithm>
#include <mutex>
#include <new>

#include "rmw/rmw.h"
#include "rmw/event.h"
#include "rmw/error_handling.h"
// Foxy has no rmw/events_statuses/ (Galactic+): the liveliness and deadline
// statuses live in rmw/types.h, the incompatible-QoS ones and
// rmw_qos_policy_kind_t in rmw/incompatible_qos_events_statuses.h.
#include "rmw/incompatible_qos_events_statuses.h"
#include "rmw/types.h"

// ROS 2 Iron and newer (e.g. Jazzy) expose matched events; stock Humble does not
// define the RMW_EVENT_*_MATCHED enum values nor rmw/events_statuses/matched.h.
// Feature-detect so a single source builds on both distros: matched event cases
// compile only where the API actually exists.
#if __has_include("rmw/events_statuses/matched.h")
#define RMW_INT2DDS_HAS_MATCHED_EVENT 1
#include "rmw/events_statuses/matched.h"
#endif

#if __has_include("rmw/events_statuses/incompatible_type.h")
#define RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT 1
#include "rmw/events_statuses/incompatible_type.h"
#endif

#include "int2dds-ffi.h"  // NOLINT(build/include): vendored FFI header
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "../common/listeners.hpp"  // NOLINT(build/include)
#include "../wait/waitset_registry.hpp"  // NOLINT(build/include)


namespace
{


[[maybe_unused]] bool
is_supported_publisher_event(rmw_event_type_t event_type)
{
  switch (event_type) {
    case RMW_EVENT_LIVELINESS_LOST:
    case RMW_EVENT_OFFERED_DEADLINE_MISSED:
    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE:
#ifdef RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT
    case RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE:
#endif
#ifdef RMW_INT2DDS_HAS_MATCHED_EVENT
    case RMW_EVENT_PUBLICATION_MATCHED:
#endif
      return true;
    default:
      return false;
  }
}

[[maybe_unused]] bool
is_supported_subscription_event(rmw_event_type_t event_type)
{
  switch (event_type) {
    case RMW_EVENT_LIVELINESS_CHANGED:
    case RMW_EVENT_REQUESTED_DEADLINE_MISSED:
    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE:
#ifdef RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT
    case RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE:
#endif
#ifdef RMW_INT2DDS_HAS_MATCHED_EVENT
    case RMW_EVENT_SUBSCRIPTION_MATCHED:
#endif
      return true;
    default:
      return false;
  }
}

[[maybe_unused]] bool
is_supported_event_type(rmw_event_type_t event_type)
{
  return is_supported_publisher_event(event_type) || is_supported_subscription_event(event_type);
}

[[maybe_unused]] uint32_t
event_type_to_status_mask(rmw_event_type_t event_type)
{
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
#ifdef RMW_INT2DDS_HAS_MATCHED_EVENT
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
#ifdef RMW_INT2DDS_HAS_MATCHED_EVENT
    case RMW_EVENT_SUBSCRIPTION_MATCHED:
      return INT2DDS_STATUS_SUBSCRIPTION_MATCHED;
#endif
    default:
      return 0;
  }
}

[[maybe_unused]] Int2DdsStatusCondition *
ensure_publisher_status_condition(rmw_int2dds_cpp::PublisherData * pub_data)
{
  if (pub_data == nullptr || pub_data->datawriter == nullptr) {
    return nullptr;
  }

  if (pub_data->status_condition != nullptr) {
    return pub_data->status_condition;
  }

  Int2DdsStatusCondition * status_condition = nullptr;
  Int2DdsRet ret = int2dds_datawriter_get_statuscondition(pub_data->datawriter, &status_condition);
  if (ret != INT2DDS_RET_OK) {
    return nullptr;
  }
  ret = int2dds_statuscondition_set_enabled_statuses(status_condition, 0u);
  if (ret != INT2DDS_RET_OK) {
    return nullptr;
  }

  pub_data->status_condition = status_condition;
  return pub_data->status_condition;
}

[[maybe_unused]] Int2DdsStatusCondition *
ensure_subscription_status_condition(rmw_int2dds_cpp::SubscriptionData * sub_data)
{
  if (sub_data == nullptr || sub_data->datareader == nullptr) {
    return nullptr;
  }

  if (sub_data->status_condition != nullptr) {
    return sub_data->status_condition;
  }

  Int2DdsStatusCondition * status_condition = nullptr;
  Int2DdsRet ret = int2dds_datareader_get_statuscondition(sub_data->datareader, &status_condition);
  if (ret != INT2DDS_RET_OK) {
    return nullptr;
  }
  ret = int2dds_statuscondition_set_enabled_statuses(status_condition, 0u);
  if (ret != INT2DDS_RET_OK) {
    return nullptr;
  }

  sub_data->status_condition = status_condition;
  return sub_data->status_condition;
}

[[maybe_unused]] rmw_ret_t
configure_status_condition(
  Int2DdsStatusCondition * status_condition,
  rmw_event_type_t event_type)
{
  if (status_condition == nullptr) {
    RMW_SET_ERROR_MSG("status condition is null");
    return RMW_RET_ERROR;
  }

  uint32_t status_mask = event_type_to_status_mask(event_type);
  if (status_mask == 0) {
    return RMW_RET_UNSUPPORTED;
  }

  uint32_t enabled_mask = 0;
  Int2DdsRet ret = int2dds_statuscondition_get_enabled_statuses(status_condition, &enabled_mask);
  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to read status condition mask");
    return RMW_RET_ERROR;
  }

  ret = int2dds_statuscondition_set_enabled_statuses(
    status_condition, enabled_mask | status_mask);
  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to configure status condition");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

[[maybe_unused]] rmw_int2dds_cpp::EventData *
allocate_event_data()
{
  return new (std::nothrow) rmw_int2dds_cpp::EventData();
}

rmw_qos_policy_kind_t
int2dds_qos_policy_to_rmw(Int2DdsQosPolicyId policy_id)
{
  switch (policy_id) {
    case Durability:
      return RMW_QOS_POLICY_DURABILITY;
    case Deadline:
      return RMW_QOS_POLICY_DEADLINE;
    case Liveliness:
      return RMW_QOS_POLICY_LIVELINESS;
    case Reliability:
      return RMW_QOS_POLICY_RELIABILITY;
    case History:
      return RMW_QOS_POLICY_HISTORY;
    case Lifespan:
      return RMW_QOS_POLICY_LIFESPAN;
    default:
      return RMW_QOS_POLICY_INVALID;
  }
}

rmw_ret_t
take_publisher_event(
  rmw_int2dds_cpp::EventData * event_data,
  void * event_info,
  bool * changed)
{
  *changed = false;
  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(event_data->entity_data);
  if (pub_data == nullptr || pub_data->datawriter == nullptr) {
    RMW_SET_ERROR_MSG("publisher event data is null");
    return RMW_RET_ERROR;
  }

  switch (event_data->event_type) {
    case RMW_EVENT_LIVELINESS_LOST: {
        auto * status = static_cast<rmw_liveliness_lost_status_t *>(event_info);
        Int2DdsLivelinessLostStatus ffi_status{};
        Int2DdsRet ret = int2dds_datawriter_get_liveliness_lost_status(
          pub_data->datawriter,
          &ffi_status);
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get liveliness lost status");
          return RMW_RET_ERROR;
        }
        status->total_count = ffi_status.total_count;
        status->total_count_change = ffi_status.total_count_change;
        *changed = (ffi_status.total_count_change != 0) ||
          (event_data->last_total_count != static_cast<size_t>(ffi_status.total_count));
        event_data->last_total_count = static_cast<size_t>(ffi_status.total_count);
        return RMW_RET_OK;
      }
    case RMW_EVENT_OFFERED_DEADLINE_MISSED: {
        auto * status = static_cast<rmw_offered_deadline_missed_status_t *>(event_info);
        Int2DdsOfferedDeadlineMissedStatus ffi_status{};
        Int2DdsRet ret = int2dds_datawriter_get_offered_deadline_missed_status(
          pub_data->datawriter, &ffi_status);
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get offered deadline missed status");
          return RMW_RET_ERROR;
        }
        status->total_count = ffi_status.total_count;
        status->total_count_change = ffi_status.total_count_change;
        *changed = (ffi_status.total_count_change != 0) ||
          (event_data->last_total_count != static_cast<size_t>(ffi_status.total_count));
        event_data->last_total_count = static_cast<size_t>(ffi_status.total_count);
        return RMW_RET_OK;
      }
    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE: {
        auto * status = static_cast<rmw_offered_qos_incompatible_event_status_t *>(event_info);
        Int2DdsOfferedIncompatibleQosStatus ffi_status{};
        Int2DdsRet ret = int2dds_datawriter_get_offered_incompatible_qos_status(
          pub_data->datawriter, &ffi_status);
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get offered incompatible qos status");
          return RMW_RET_ERROR;
        }
        status->total_count = ffi_status.total_count;
        status->total_count_change = ffi_status.total_count_change;
        status->last_policy_kind = int2dds_qos_policy_to_rmw(ffi_status.last_policy_id);
        *changed = (ffi_status.total_count_change != 0) ||
          (event_data->last_total_count != static_cast<size_t>(ffi_status.total_count));
        event_data->last_total_count = static_cast<size_t>(ffi_status.total_count);
        return RMW_RET_OK;
      }
#ifdef RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT
    case RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE: {
        auto * status = static_cast<rmw_incompatible_type_status_t *>(event_info);
        Int2DdsOfferedIncompatibleTypeStatus ffi_status{};
        Int2DdsRet ret = int2dds_datawriter_get_offered_incompatible_type_status(
          pub_data->datawriter, &ffi_status);
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get offered incompatible type status");
          return RMW_RET_ERROR;
        }
        status->total_count = ffi_status.total_count;
        status->total_count_change = ffi_status.total_count_change;
        *changed = (ffi_status.total_count_change != 0) ||
          (event_data->last_total_count != static_cast<size_t>(ffi_status.total_count));
        event_data->last_total_count = static_cast<size_t>(ffi_status.total_count);
        return RMW_RET_OK;
      }
#endif
#ifdef RMW_INT2DDS_HAS_MATCHED_EVENT
    case RMW_EVENT_PUBLICATION_MATCHED: {
        auto * status = static_cast<rmw_matched_status_t *>(event_info);
        Int2DdsPublicationMatchedStatus matched_status = {};
        Int2DdsRet ret = int2dds_datawriter_get_publication_matched_status(
          pub_data->datawriter, &matched_status);
        const int32_t total_count = matched_status.total_count;
        const int32_t current_count = matched_status.current_count;
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get publication matched status");
          return RMW_RET_ERROR;
        }

        const size_t total_count_sz = static_cast<size_t>(total_count);
        const size_t current_count_sz = static_cast<size_t>(current_count);
        const size_t prev_total = event_data->last_total_count;
        const size_t prev_current = event_data->last_current_count;

        status->total_count = total_count_sz;
        status->total_count_change = total_count_sz >=
          prev_total ? (total_count_sz - prev_total) : 0;
        status->current_count = current_count_sz;
        status->current_count_change =
          static_cast<int32_t>(current_count) - static_cast<int32_t>(prev_current);

        event_data->last_total_count = total_count_sz;
        event_data->last_current_count = current_count_sz;
        *changed = (status->total_count_change != 0) || (status->current_count_change != 0);
        return RMW_RET_OK;
      }
#endif
    default:
      return RMW_RET_UNSUPPORTED;
  }
}

rmw_ret_t
take_subscription_event(
  rmw_int2dds_cpp::EventData * event_data,
  void * event_info,
  bool * changed)
{
  *changed = false;
  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(event_data->entity_data);
  if (sub_data == nullptr || sub_data->datareader == nullptr) {
    RMW_SET_ERROR_MSG("subscription event data is null");
    return RMW_RET_ERROR;
  }

  switch (event_data->event_type) {
    case RMW_EVENT_LIVELINESS_CHANGED: {
        auto * status = static_cast<rmw_liveliness_changed_status_t *>(event_info);
        Int2DdsLivelinessChangedStatus ffi_status{};
        Int2DdsRet ret = int2dds_datareader_get_liveliness_changed_status(
          sub_data->datareader, &ffi_status);
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get liveliness changed status");
          return RMW_RET_ERROR;
        }
        status->alive_count = ffi_status.alive_count;
        status->not_alive_count = ffi_status.not_alive_count;
        status->alive_count_change = ffi_status.alive_count_change;
        status->not_alive_count_change = ffi_status.not_alive_count_change;
        *changed = (ffi_status.alive_count_change != 0) ||
          (ffi_status.not_alive_count_change != 0) ||
          (event_data->last_total_count != static_cast<size_t>(ffi_status.alive_count)) ||
          (event_data->last_current_count != static_cast<size_t>(ffi_status.not_alive_count));
        event_data->last_total_count = static_cast<size_t>(ffi_status.alive_count);
        event_data->last_current_count = static_cast<size_t>(ffi_status.not_alive_count);
        return RMW_RET_OK;
      }
    case RMW_EVENT_REQUESTED_DEADLINE_MISSED: {
        auto * status = static_cast<rmw_requested_deadline_missed_status_t *>(event_info);
        Int2DdsRequestedDeadlineMissedStatus ffi_status{};
        Int2DdsRet ret = int2dds_datareader_get_requested_deadline_missed_status(
          sub_data->datareader, &ffi_status);
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get requested deadline missed status");
          return RMW_RET_ERROR;
        }
        status->total_count = ffi_status.total_count;
        status->total_count_change = ffi_status.total_count_change;
        *changed = (ffi_status.total_count_change != 0) ||
          (event_data->last_total_count != static_cast<size_t>(ffi_status.total_count));
        event_data->last_total_count = static_cast<size_t>(ffi_status.total_count);
        return RMW_RET_OK;
      }
    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE: {
        auto * status = static_cast<rmw_requested_qos_incompatible_event_status_t *>(event_info);
        Int2DdsRequestedIncompatibleQosStatus ffi_status{};
        Int2DdsRet ret = int2dds_datareader_get_requested_incompatible_qos_status(
          sub_data->datareader, &ffi_status);
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get requested incompatible qos status");
          return RMW_RET_ERROR;
        }
        status->total_count = ffi_status.total_count;
        status->total_count_change = ffi_status.total_count_change;
        status->last_policy_kind = int2dds_qos_policy_to_rmw(ffi_status.last_policy_id);
        *changed = (ffi_status.total_count_change != 0) ||
          (event_data->last_total_count != static_cast<size_t>(ffi_status.total_count));
        event_data->last_total_count = static_cast<size_t>(ffi_status.total_count);
        return RMW_RET_OK;
      }
#ifdef RMW_INT2DDS_HAS_INCOMPATIBLE_TYPE_EVENT
    case RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE: {
        auto * status = static_cast<rmw_incompatible_type_status_t *>(event_info);
        Int2DdsRequestedIncompatibleTypeStatus ffi_status{};
        Int2DdsRet ret = int2dds_datareader_get_requested_incompatible_type_status(
          sub_data->datareader, &ffi_status);
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get requested incompatible type status");
          return RMW_RET_ERROR;
        }
        status->total_count = ffi_status.total_count;
        status->total_count_change = ffi_status.total_count_change;
        *changed = (ffi_status.total_count_change != 0) ||
          (event_data->last_total_count != static_cast<size_t>(ffi_status.total_count));
        event_data->last_total_count = static_cast<size_t>(ffi_status.total_count);
        return RMW_RET_OK;
      }
#endif
#ifdef RMW_INT2DDS_HAS_MATCHED_EVENT
    case RMW_EVENT_SUBSCRIPTION_MATCHED: {
        auto * status = static_cast<rmw_matched_status_t *>(event_info);
        Int2DdsSubscriptionMatchedStatus matched_status = {};
        Int2DdsRet ret = int2dds_datareader_get_subscription_matched_status(
          sub_data->datareader, &matched_status);
        const int32_t total_count = matched_status.total_count;
        const int32_t current_count = matched_status.current_count;
        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to get subscription matched status");
          return RMW_RET_ERROR;
        }

        const size_t total_count_sz = static_cast<size_t>(total_count);
        const size_t current_count_sz = static_cast<size_t>(current_count);
        const size_t prev_total = event_data->last_total_count;
        const size_t prev_current = event_data->last_current_count;

        status->total_count = total_count_sz;
        status->total_count_change = total_count_sz >=
          prev_total ? (total_count_sz - prev_total) : 0;
        status->current_count = current_count_sz;
        status->current_count_change =
          static_cast<int32_t>(current_count) - static_cast<int32_t>(prev_current);

        event_data->last_total_count = total_count_sz;
        event_data->last_current_count = current_count_sz;
        *changed = (status->total_count_change != 0) || (status->current_count_change != 0);
        return RMW_RET_OK;
      }
#endif
    default:
      return RMW_RET_UNSUPPORTED;
  }
}

}  // namespace

// Central EventData deleter, shared by rmw_event_fini and the entity
// destroy paths, so every EventData is freed through one place.
namespace rmw_int2dds_cpp
{
void free_event_data(EventData * event_data)
{
  if (event_data == nullptr) {
    return;
  }
  delete event_data;
}
}  // namespace rmw_int2dds_cpp

extern "C"
{
rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * rmw_event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("publisher not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (rmw_event->implementation_identifier != nullptr || rmw_event->data != nullptr) {
    RMW_SET_ERROR_MSG("event already initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (!is_supported_publisher_event(event_type)) {
    return RMW_RET_UNSUPPORTED;
  }

  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(publisher->data);
  if (pub_data == nullptr) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  Int2DdsStatusCondition * status_condition = ensure_publisher_status_condition(pub_data);
  if (status_condition == nullptr) {
    RMW_SET_ERROR_MSG("failed to get publisher status condition");
    return RMW_RET_ERROR;
  }

  rmw_ret_t ret = configure_status_condition(status_condition, event_type);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto * event_data = allocate_event_data();
  if (event_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate event data");
    return RMW_RET_BAD_ALLOC;
  }

  event_data->event_type = event_type;
  event_data->entity_data = pub_data;
  event_data->is_publisher = true;

  // Hand ownership to the publisher so the EventData is freed on
  // rmw_destroy_publisher even when rcl never calls rmw_event_fini.
  {
    std::lock_guard<std::mutex> guard(pub_data->event_mutex);
    pub_data->events.push_back(event_data);
  }

  rmw_event->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  rmw_event->data = event_data;
  rmw_event->event_type = event_type;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * rmw_event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (rmw_event->implementation_identifier != nullptr || rmw_event->data != nullptr) {
    RMW_SET_ERROR_MSG("event already initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (!is_supported_subscription_event(event_type)) {
    return RMW_RET_UNSUPPORTED;
  }

  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscription->data);
  if (sub_data == nullptr) {
    RMW_SET_ERROR_MSG("subscription data is null");
    return RMW_RET_ERROR;
  }

  Int2DdsStatusCondition * status_condition = ensure_subscription_status_condition(sub_data);
  if (status_condition == nullptr) {
    RMW_SET_ERROR_MSG("failed to get subscription status condition");
    return RMW_RET_ERROR;
  }

  rmw_ret_t ret = configure_status_condition(status_condition, event_type);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto * event_data = allocate_event_data();
  if (event_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate event data");
    return RMW_RET_BAD_ALLOC;
  }

  event_data->event_type = event_type;
  event_data->entity_data = sub_data;
  event_data->is_publisher = false;

  // Hand ownership to the subscription so the EventData is freed on
  // rmw_destroy_subscription even when rcl never calls rmw_event_fini.
  {
    std::lock_guard<std::mutex> guard(sub_data->event_mutex);
    sub_data->events.push_back(event_data);
  }

  rmw_event->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  rmw_event->data = event_data;
  rmw_event->event_type = event_type;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_event(
  const rmw_event_t * event_handle,
  void * event_info,
  bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(event_handle, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  if (event_handle->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("event handle not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * event_data = static_cast<rmw_int2dds_cpp::EventData *>(event_handle->data);
  if (event_data == nullptr) {
    RMW_SET_ERROR_MSG("event data is null");
    return RMW_RET_ERROR;
  }

  if (!is_supported_event_type(event_data->event_type)) {
    return RMW_RET_UNSUPPORTED;
  }

  // rmw events are level-triggered status reads: rmw_take_event always
  // reports taken=true with the current status snapshot (even when nothing
  // changed since the last take), matching the reference RMW behavior that
  // rclcpp::EventHandler::execute and rcl_take_event rely on.
  if (event_data->entity_data == nullptr) {
    RMW_SET_ERROR_MSG("event entity data is null");
    return RMW_RET_ERROR;
  }

  bool changed = false;
  rmw_ret_t rmw_ret = event_data->is_publisher ?
    take_publisher_event(event_data, event_info, &changed) :
    take_subscription_event(event_data, event_info, &changed);
  if (rmw_ret != RMW_RET_OK) {
    return rmw_ret;
  }

  *taken = true;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_event_fini(rmw_event_t * event)
{
  if (event != nullptr && event->data != nullptr) {
    auto * event_data = static_cast<rmw_int2dds_cpp::EventData *>(event->data);
    // Detach from the owning entity's tracking list first, so a later
    // entity destroy will not double-free this EventData. (rcl currently never
    // calls this path - measured fini count 0 - but stay robust if it does.)
    if (event_data->entity_data != nullptr) {
      if (event_data->is_publisher) {
        auto * pd = static_cast<rmw_int2dds_cpp::PublisherData *>(event_data->entity_data);
        std::lock_guard<std::mutex> guard(pd->event_mutex);
        auto & v = pd->events;
        v.erase(std::remove(v.begin(), v.end(), event_data), v.end());
      } else {
        auto * sd = static_cast<rmw_int2dds_cpp::SubscriptionData *>(event_data->entity_data);
        std::lock_guard<std::mutex> guard(sd->event_mutex);
        auto & v = sd->events;
        v.erase(std::remove(v.begin(), v.end(), event_data), v.end());
      }
    }
    // The EventData pointer is a wait set cache key (cached_events).
    rmw_int2dds_cpp::waitset_registry_clean_caches();
    rmw_int2dds_cpp::free_event_data(event_data);
    event->data = nullptr;
  }
  return RMW_RET_OK;
}

// rmw_event_set_callback and RMW_EVENT_MESSAGE_LOST are Humble additions with
// no Foxy counterpart.

bool
rmw_event_type_is_supported(rmw_event_type_t rmw_event_type)
{
  return is_supported_event_type(rmw_event_type);
}
}  // extern "C"
