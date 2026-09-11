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

#ifndef RMW_INT2DDS_CPP__TYPES_HPP_
#define RMW_INT2DDS_CPP__TYPES_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rmw/event.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rmw_dds_common/context.hpp"

#include "rmw_int2dds_cpp/foxy_compat.hpp"

// Forward declarations from int2dds FFI
extern "C" {
struct Int2DdsParticipantFactory;
struct Int2DdsParticipant;
struct Int2DdsPublisher;
struct Int2DdsSubscriber;
struct Int2DdsTopic;
struct Int2DdsContentFilteredTopic;
struct Int2DdsDataWriter;
struct Int2DdsDataReader;
struct Int2DdsDataWriterQos;
struct Int2DdsDataReaderQos;
struct Int2DdsWaitSet;
struct Int2DdsGuardCondition;
struct Int2DdsStatusCondition;
struct Int2DdsConditionSeq;
}

namespace rmw_int2dds_cpp
{

// int2dds-ffi extensibility values for int2dds_create_topic.
// 0 = Final, 1 = Appendable, 2 = Mutable.
// ROS 2 IDL messages default to Mutable extensibility.
constexpr int32_t INT2DDS_EXTENSIBILITY_FINAL = 0;
constexpr int32_t INT2DDS_EXTENSIBILITY_APPENDABLE = 1;
constexpr int32_t INT2DDS_EXTENSIBILITY_MUTABLE = 2;

struct ServiceData;
struct ClientData;
struct EventData;

/// One user event-callback registration (rmw_event_callback_t plus the backlog
/// of occurrences that fired before the callback was set). Fired from int2dds
/// listener threads; all access goes through the owning entity's
/// listener_mutex.
struct CallbackSlot
{
  rmw_event_callback_t callback{nullptr};
  const void * user_data{nullptr};
  size_t unread{0};
};

/// Context implementation data
struct ContextData
{
  Int2DdsParticipantFactory * factory{nullptr};
  Int2DdsParticipant * participant{nullptr};
  Int2DdsPublisher * default_publisher{nullptr};
  Int2DdsSubscriber * default_subscriber{nullptr};

  Int2DdsGuardCondition * graph_guard_condition{nullptr};

  // Standard rmw_dds_common node-graph discovery (ros_discovery_info topic).
  // Replaces the heuristic service-name-based node inference. Built across phases.
  std::unique_ptr<rmw_dds_common::Context> common;
  Int2DdsTopic * discovery_topic{nullptr};
  Int2DdsDataWriter * discovery_writer{nullptr};
  Int2DdsDataReader * discovery_reader{nullptr};

  size_t domain_id{0};
  // True when the user requested localhost-only discovery (Humble: the init-options
  // localhost_only field). Applied via multicast_ttl=0 so multicast SPDP stays on
  // the host; local discovery is unaffected. Set in rmw_init.
  bool localhost_only{false};
  bool is_shutdown{false};
  std::atomic<int> ref_count{0};
  std::mutex mutex;

  // Remote endpoints (DDS GUID -> is_reader) mirrored from int2DDS DDS
  // discovery into rmw_dds_common GraphCache. Guarded by remote_sync_mutex.
  std::mutex remote_sync_mutex;
  std::map<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, bool> synced_remote_entities;
};

/// Create the DDS resources a context needs: participant factory, participant,
/// default publisher/subscriber, graph guard condition and node-graph discovery.
/// Uses context_data->domain_id, so that must be set first.
///
/// rmw_destroy_node releases these once the last node is gone (a client library
/// may keep the context alive, so rmw_context_fini may never run). rmw_create_node
/// calls this again to bring them back, mirroring rmw_fastrtps, where
/// increment_context_impl_ref_count() recreates what
/// decrement_context_impl_ref_count() released. Rolls back on failure and leaves
/// every pointer null.
///
/// \return RMW_RET_OK on success, otherwise the error is already set.
rmw_ret_t
acquire_context_resources(ContextData * context_data, const char * enclave);

/// Release what acquire_context_resources() created, nulling every pointer so a
/// later acquire or fini is safe.
void
release_context_resources(ContextData * context_data);

/// Node implementation data
struct NodeData
{
  ContextData * context_data{nullptr};
  std::string name;
  std::string namespace_;
  rmw_gid_t gid;
  rmw_guard_condition_t * graph_guard_condition{nullptr};
  mutable std::mutex entities_mutex;

  // Entities owned by this node
  std::vector<rmw_gid_t> publishers;
  std::vector<rmw_gid_t> subscriptions;
  std::vector<rmw_gid_t> services;
  std::vector<rmw_gid_t> clients;
  std::vector<ServiceData *> live_services;
  std::vector<ClientData *> live_clients;
};

/// Publisher implementation data
struct PublisherData
{
  Int2DdsDataWriter * datawriter{nullptr};
  Int2DdsTopic * topic{nullptr};
  Int2DdsStatusCondition * status_condition{nullptr};

  const rosidl_message_type_support_t * type_support{nullptr};
  rmw_qos_profile_t qos;
  rmw_gid_t gid;

  std::string topic_name;
  std::string type_name;

  NodeData * node_data{nullptr};
  std::atomic<size_t> serialized_capacity_hint{0};

  // Matched-event delivery bridge. The int2dds DataWriter listener (registered
  // when a PUBLICATION_MATCHED rmw_event is created) runs on an int2dds thread,
  // so all access is serialized through matched_mutex. matched_callback is the
  // user rmw event callback; matched_unread holds occurrences that arrived
  // before a callback was registered (flushed when one is set).
  std::mutex matched_mutex;
  rmw_event_callback_t matched_callback{nullptr};
  const void * matched_user_data{nullptr};
  size_t matched_unread{0};
  // Highest matched total_count already accounted for (matched_mutex). Lets
  // rmw_event_set_callback reconstruct the backlog of matches that occurred
  // while no listener was active.
  size_t matched_total_seen{0};

  // Writer status-event delivery bridge (see CallbackSlot); fed by the
  // combined int2dds DataWriter listener registered at creation.
  std::mutex listener_mutex;
  CallbackSlot offered_deadline_missed_slot;
  CallbackSlot offered_incompatible_qos_slot;
  CallbackSlot offered_incompatible_type_slot;
  CallbackSlot liveliness_lost_slot;

  // EventData objects created by rmw_publisher_event_init for this
  // publisher. rcl/rclpy never call rmw_event_fini (measured: fini count 0), so
  // unless the entity owns them, every create/destroy churn leaks one EventData
  // per event type (int2dds-specific; the reference RMW ties event data to the
  // entity likewise). Guarded by event_mutex; freed in rmw_destroy_publisher.
  std::mutex event_mutex;
  std::vector<EventData *> events;
};

/// Subscription implementation data
struct SubscriptionData
{
  Int2DdsDataReader * datareader{nullptr};
  Int2DdsTopic * topic{nullptr};
  Int2DdsContentFilteredTopic * content_filtered_topic{nullptr};
  Int2DdsStatusCondition * status_condition{nullptr};

  const rosidl_message_type_support_t * type_support{nullptr};
  rmw_qos_profile_t qos;
  rmw_gid_t gid;

  std::string topic_name;
  std::string type_name;
  std::string content_filter_expression;
  std::vector<std::string> content_filter_parameters;

  NodeData * node_data{nullptr};
  std::atomic<uint64_t> reception_sequence{0};

  // Mirror of rmw_subscription_options.ignore_local_publications so rmw_wait can
  // decide, without the rmw_subscription_t, whether readiness must skip a pending
  // sample that came from this node's own publisher (see subscription_has_data).
  bool ignore_local_publications{false};

  // Matched-event delivery bridge (see PublisherData::matched_mutex). Populated
  // by the int2dds DataReader listener for SUBSCRIPTION_MATCHED events.
  std::mutex matched_mutex;
  rmw_event_callback_t matched_callback{nullptr};
  const void * matched_user_data{nullptr};
  size_t matched_unread{0};
  // See PublisherData::matched_total_seen.
  size_t matched_total_seen{0};
  // RMW-internal matched-change flag (matched_mutex): set by the DataReader
  // matched listener on every match/unmatch, cleared when the event is taken.
  bool matched_changed{false};

  // Data/status delivery bridge (see CallbackSlot); fed by the combined
  // int2dds DataReader listener registered at creation. new_message_slot backs
  // rmw_subscription_set_on_new_message_callback; the others back
  // rmw_event_set_callback for the corresponding event types.
  std::mutex listener_mutex;
  CallbackSlot new_message_slot;
  CallbackSlot requested_deadline_missed_slot;
  CallbackSlot requested_incompatible_qos_slot;
  CallbackSlot requested_incompatible_type_slot;
  CallbackSlot message_lost_slot;
  CallbackSlot liveliness_changed_slot;

  // Reuse a receive buffer across takes to avoid per-message large allocations on
  // high-rate subscriptions. Access is serialized with take_buffer_mutex because
  // rmw_take and rmw_take_serialized_message may share the same scratch buffer.
  std::mutex take_buffer_mutex;
  std::vector<uint8_t> take_buffer;

  // EventData objects created by rmw_subscription_event_init for this
  // subscription. rcl/rclpy never call rmw_event_fini (measured: fini count 0),
  // so unless the entity owns them, every create/destroy churn leaks one
  // EventData per event type (int2dds-specific; the reference RMW ties event
  // data to the entity likewise). Guarded by event_mutex; freed in
  // rmw_destroy_subscription.
  std::mutex event_mutex;
  std::vector<EventData *> events;
};

/// Guard condition implementation data
struct GuardConditionData
{
  Int2DdsGuardCondition * guard_condition{nullptr};
  bool owns_guard_condition{true};
};

/// Wait set implementation data
// Identity of an event cached by a wait set: the entity data pointer plus the
// event type, since rmw_event_t wrappers are not stable across rmw_wait calls.
struct WaitSetCachedEvent
{
  void * entity_data{nullptr};
  rmw_event_type_t event_type{RMW_EVENT_INVALID};
};

struct WaitSetData
{
  Int2DdsWaitSet * waitset{nullptr};

  // Guards the cached/attached state below against entity-destroy cache cleaning.
  std::mutex lock;
  std::condition_variable cache_cv;
  bool inuse{false};
  // True while rmw_wait reads or rewrites the cache/attachments outside the
  // blocking FFI wait. Cleaners wait this flag out (signalled via cache_cv);
  // it never spans int2dds_waitset_wait_ex_ns, so the wait is always short.
  bool cache_busy{false};

  // Entity data pointers from the previous rmw_wait call. When the incoming
  // arrays are identical, the conditions attached last time are still attached
  // and the whole attach/detach cycle is skipped.
  std::vector<void *> cached_subscriptions;
  std::vector<void *> cached_guard_conditions;
  std::vector<void *> cached_services;
  std::vector<void *> cached_clients;
  std::vector<WaitSetCachedEvent> cached_events;

  // Attached conditions, each stamped with the attach_generation that last
  // wanted it. A rebuild detaches whatever keeps an older stamp.
  uint64_t attach_generation{0};
  std::unordered_map<Int2DdsStatusCondition *, uint64_t> attached_conditions;
  std::unordered_map<Int2DdsGuardCondition *, uint64_t> attached_guards;
};

/// Service implementation data
struct ServiceData
{
  // Request handling (like subscription)
  Int2DdsDataReader * request_reader{nullptr};
  Int2DdsTopic * request_topic{nullptr};
  Int2DdsStatusCondition * request_status_condition{nullptr};

  // Response sending (like publisher)
  Int2DdsDataWriter * response_writer{nullptr};
  Int2DdsTopic * response_topic{nullptr};

  const rosidl_service_type_support_t * type_support{nullptr};
  rmw_qos_profile_t qos;
  rmw_gid_t gid;

  std::string service_name;

  NodeData * node_data{nullptr};
  bool detached{false};

  // Backs rmw_service_set_on_new_request_callback; fed by the int2dds
  // DataReader listener on the request reader.
  std::mutex listener_mutex;
  CallbackSlot new_request_slot;
  std::mutex request_take_mutex;
  std::mutex response_write_mutex;
};

/// Client implementation data
struct ClientData
{
  // Request sending (like publisher)
  Int2DdsDataWriter * request_writer{nullptr};
  Int2DdsTopic * request_topic{nullptr};

  // Response handling (like subscription)
  Int2DdsDataReader * response_reader{nullptr};
  Int2DdsTopic * response_topic{nullptr};
  Int2DdsStatusCondition * response_status_condition{nullptr};

  const rosidl_service_type_support_t * type_support{nullptr};
  rmw_qos_profile_t qos;
  rmw_gid_t gid;

  std::string service_name;

  NodeData * node_data{nullptr};
  std::atomic<int64_t> sequence_number{0};
  bool detached{false};

  // Backs rmw_client_set_on_new_response_callback; fed by the int2dds
  // DataReader listener on the response reader. Fires for every sample on the
  // shared response topic; foreign responses produce a spurious wake-up that
  // the executor resolves by taking nothing.
  std::mutex listener_mutex;
  CallbackSlot new_response_slot;
  std::mutex response_take_mutex;
};

/// Event implementation data
struct EventData
{
  // The backing DDS status condition is owned by the entity data
  // (PublisherData/SubscriptionData::status_condition, released in the entity
  // destroy path) and resolved on demand instead of being cached here, so a
  // reader recreated for a content-filter update cannot leave a dangling
  // alias behind.
  rmw_event_type_t event_type{RMW_EVENT_INVALID};

  // Can be publisher or subscription implementation data.
  void * entity_data{nullptr};
  bool is_publisher{false};

  // Stored for rmw_event_set_callback support.
  rmw_event_callback_t callback{nullptr};
  const void * user_data{nullptr};

  // Tracks unread occurrences for callback/event delivery bookkeeping.
  std::atomic<size_t> unread_count{0};

  // Used to derive matched-status deltas from count-based FFI getters.
  size_t last_total_count{0};
  size_t last_current_count{0};
};

// Free an EventData created by rmw_*_event_init. Used by rmw_event_fini
// and by the entity destroy paths that reclaim events rcl never finalized, so
// all frees stay accounted in one place.
void free_event_data(EventData * event_data);

}  // namespace rmw_int2dds_cpp

#endif  // RMW_INT2DDS_CPP__TYPES_HPP_
