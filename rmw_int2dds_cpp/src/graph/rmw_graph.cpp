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

#include <tuple>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <optional>

#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"

// ROS 2 Lyrical adds per-service endpoint introspection (rmw 7.9, ros2/rmw#371);
// older distros lack both the rmw headers and the GraphCache support for it.
#if __has_include("rmw/get_service_endpoint_info.h")
#define RMW_INT2DDS_HAS_SERVICE_ENDPOINT_INFO 1
#include "rmw/get_service_endpoint_info.h"
#include "rmw/service_endpoint_info.h"
#include "rmw/service_endpoint_info_array.h"
#endif
#include "rmw/names_and_types.h"
#include "rmw/sanity_checks.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw/topic_endpoint_info.h"
#include "rmw/validate_node_name.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_full_topic_name.h"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "rmw_dds_common/qos.hpp"

#include "int2dds-ffi.h"  // NOLINT(build/include_subdir): vendored FFI header
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "discovery.hpp"  // NOLINT(build/include_subdir): same dir

// Helper to demangle service name (remove "rq"/"rr" prefix)
static std::string demangle_service_name(const std::string & service_name)
{
  if (service_name.size() > 2) {
    if (service_name.compare(0, 2, "rq") == 0 ||
      service_name.compare(0, 2, "rr") == 0)
    {
      std::string demangled = service_name.substr(2);
      // Remove "Request" or "Reply" suffix
      size_t pos = demangled.find("Request");
      if (pos != std::string::npos) {
        return demangled.substr(0, pos);
      }
      pos = demangled.find("Reply");
      if (pos != std::string::npos) {
        return demangled.substr(0, pos);
      }
      return demangled;
    }
  }
  return service_name;
}

static std::string demangle_dds_message_type_name(const std::string & type_name)
{
  const std::string marker = "::dds_::";
  const size_t marker_pos = type_name.find(marker);
  if (marker_pos == std::string::npos) {
    return type_name;
  }

  std::string demangled = type_name;
  demangled.replace(marker_pos, marker.size(), "::");
  if (!demangled.empty() && demangled.back() == '_') {
    demangled.pop_back();
  }

  std::string::size_type pos = 0;
  while ((pos = demangled.find("::", pos)) != std::string::npos) {
    demangled.replace(pos, 2, "/");
    pos += 1;
  }

  return demangled;
}

static std::string demangle_dds_service_type_name(const std::string & type_name)
{
  std::string demangled = demangle_dds_message_type_name(type_name);
  auto strip_suffix = [](const std::string & value, const char * suffix) -> std::string {
      const size_t suffix_len = std::strlen(suffix);
      if (value.size() >= suffix_len &&
        value.compare(value.size() - suffix_len, suffix_len, suffix) == 0)
      {
        return value.substr(0, value.size() - suffix_len);
      }
      return value;
    };

  demangled = strip_suffix(demangled, "_Request");
  demangled = strip_suffix(demangled, "_Response");
  return demangled;
}

constexpr std::chrono::milliseconds kGraphSnapshotTimeout{0};

static bool is_service_request_topic(const std::string & topic_name)
{
  return topic_name.compare(0, 2, "rq") == 0 && topic_name.find("Request") != std::string::npos;
}

static bool is_service_reply_topic(const std::string & topic_name)
{
  return topic_name.compare(0, 2, "rr") == 0 &&
         (topic_name.find("Reply") != std::string::npos ||
         topic_name.find("Response") != std::string::npos);
}

static bool is_service_topic(const std::string & topic_name)
{
  return is_service_request_topic(topic_name) || is_service_reply_topic(topic_name);
}

static bool is_zero_discovery_key(const std::array<uint8_t, 12> & key)
{
  return std::all_of(key.begin(), key.end(), [](uint8_t value) {return value == 0;});
}

static std::array<uint8_t, 12> participant_key_from_endpoint_guid(
  const std::array<uint8_t, 16> & guid)
{
  std::array<uint8_t, 12> key = {};
  std::copy_n(guid.begin(), 12, key.begin());
  return key;
}

static bool read_publication_string(
  Int2DdsRet (*fn)(const Int2DdsPublicationBuiltinTopicData *, uint8_t *, uintptr_t, uintptr_t *),
  const Int2DdsPublicationBuiltinTopicData * data,
  std::string * out)
{
  uint8_t buffer[512] = {};
  uintptr_t out_len = 0;
  const Int2DdsRet ret = fn(data, buffer, sizeof(buffer), &out_len);
  if (ret != INT2DDS_RET_OK) {
    return false;
  }
  *out = std::string(
    reinterpret_cast<const char *>(buffer),
    std::min<size_t>(std::strlen(reinterpret_cast<const char *>(buffer)), out_len));
  return true;
}

static bool read_subscription_string(
  Int2DdsRet (*fn)(const Int2DdsSubscriptionBuiltinTopicData *, uint8_t *, uintptr_t, uintptr_t *),
  const Int2DdsSubscriptionBuiltinTopicData * data,
  std::string * out)
{
  uint8_t buffer[512] = {};
  uintptr_t out_len = 0;
  const Int2DdsRet ret = fn(data, buffer, sizeof(buffer), &out_len);
  if (ret != INT2DDS_RET_OK) {
    return false;
  }
  *out = std::string(
    reinterpret_cast<const char *>(buffer),
    std::min<size_t>(std::strlen(reinterpret_cast<const char *>(buffer)), out_len));
  return true;
}

static void for_each_publication_snapshot(
  rmw_int2dds_cpp::ContextData * context_data,
  int timeout_ms,
  const std::function<void(Int2DdsPublicationBuiltinTopicData *)> & callback)
{
  // The count/by-index accessors were consolidated into a single snapshot sequence.
  // ALIVE only: this pass feeds add_entity, and a departed instance carries no announcement
  // to build an entity from. Departures come from collect_departed_publications below.
  Int2DdsPublicationBuiltinTopicDataSeq * seq = nullptr;
  if (
    int2dds_participant_take_discovered_publications_snapshot_filtered(
      context_data->participant, timeout_ms, INT2DDS_INSTANCE_STATE_ALIVE, &seq) !=
    INT2DDS_RET_OK ||
    seq == nullptr)
  {
    return;
  }

  uintptr_t count = 0;
  if (int2dds_publication_builtin_topic_data_seq_length(seq, &count) == INT2DDS_RET_OK) {
    for (uintptr_t i = 0; i < count; ++i) {
      Int2DdsPublicationBuiltinTopicData * publication = nullptr;
      if (
        int2dds_publication_builtin_topic_data_seq_get(seq, i, &publication) == INT2DDS_RET_OK &&
        publication != nullptr)
      {
        callback(publication);
      }
    }
  }
  int2dds_publication_builtin_topic_data_seq_delete(seq);
}

static void for_each_subscription_snapshot(
  rmw_int2dds_cpp::ContextData * context_data,
  int timeout_ms,
  const std::function<void(Int2DdsSubscriptionBuiltinTopicData *)> & callback)
{
  // ALIVE only, for the same reason as for_each_publication_snapshot.
  Int2DdsSubscriptionBuiltinTopicDataSeq * seq = nullptr;
  if (
    int2dds_participant_take_discovered_subscriptions_snapshot_filtered(
      context_data->participant, timeout_ms, INT2DDS_INSTANCE_STATE_ALIVE, &seq) !=
    INT2DDS_RET_OK ||
    seq == nullptr)
  {
    return;
  }

  uintptr_t count = 0;
  if (int2dds_subscription_builtin_topic_data_seq_length(seq, &count) == INT2DDS_RET_OK) {
    for (uintptr_t i = 0; i < count; ++i) {
      Int2DdsSubscriptionBuiltinTopicData * subscription = nullptr;
      if (
        int2dds_subscription_builtin_topic_data_seq_get(seq, i, &subscription) == INT2DDS_RET_OK &&
        subscription != nullptr)
      {
        callback(subscription);
      }
    }
  }
  int2dds_subscription_builtin_topic_data_seq_delete(seq);
}

// Endpoint GUIDs DDS has reported as gone.
//
// A dispose travels as a key with an empty payload, so identity has to come from the instance
// handle rather than from an announcement. The core pins the instance handle to be the endpoint
// GUID (int2DDS test `the_instance_handle_is_the_endpoint_guid`), which is what makes a departure
// actionable from the handle alone.
//
// A failure here returns what was collected so far, which is empty on the first call. That is the
// intended direction of error: a departure that is missed is retried on the next pass, since a
// read does not consume the instance, whereas a departure that is invented deletes a live entity.
static std::vector<std::array<uint8_t, 16>> collect_departed_publications(
  rmw_int2dds_cpp::ContextData * context_data,
  int timeout_ms)
{
  std::vector<std::array<uint8_t, 16>> departed;
  Int2DdsPublicationBuiltinTopicDataSeq * seq = nullptr;
  if (
    int2dds_participant_take_discovered_publications_snapshot_filtered(
      context_data->participant, timeout_ms,
      INT2DDS_INSTANCE_STATE_NOT_ALIVE_DISPOSED | INT2DDS_INSTANCE_STATE_NOT_ALIVE_NO_WRITERS,
      &seq) != INT2DDS_RET_OK ||
    seq == nullptr)
  {
    return departed;
  }

  uintptr_t count = 0;
  if (int2dds_publication_builtin_topic_data_seq_length(seq, &count) == INT2DDS_RET_OK) {
    for (uintptr_t i = 0; i < count; ++i) {
      std::array<uint8_t, 16> endpoint_guid = {};
      if (
        int2dds_publication_builtin_topic_data_seq_get_instance_handle(
          seq, i, reinterpret_cast<uint8_t(*)[16]>(&endpoint_guid)) == INT2DDS_RET_OK)
      {
        departed.push_back(endpoint_guid);
      }
    }
  }
  int2dds_publication_builtin_topic_data_seq_delete(seq);
  return departed;
}

// See collect_departed_publications.
static std::vector<std::array<uint8_t, 16>> collect_departed_subscriptions(
  rmw_int2dds_cpp::ContextData * context_data,
  int timeout_ms)
{
  std::vector<std::array<uint8_t, 16>> departed;
  Int2DdsSubscriptionBuiltinTopicDataSeq * seq = nullptr;
  if (
    int2dds_participant_take_discovered_subscriptions_snapshot_filtered(
      context_data->participant, timeout_ms,
      INT2DDS_INSTANCE_STATE_NOT_ALIVE_DISPOSED | INT2DDS_INSTANCE_STATE_NOT_ALIVE_NO_WRITERS,
      &seq) != INT2DDS_RET_OK ||
    seq == nullptr)
  {
    return departed;
  }

  uintptr_t count = 0;
  if (int2dds_subscription_builtin_topic_data_seq_length(seq, &count) == INT2DDS_RET_OK) {
    for (uintptr_t i = 0; i < count; ++i) {
      std::array<uint8_t, 16> endpoint_guid = {};
      if (
        int2dds_subscription_builtin_topic_data_seq_get_instance_handle(
          seq, i, reinterpret_cast<uint8_t(*)[16]>(&endpoint_guid)) == INT2DDS_RET_OK)
      {
        departed.push_back(endpoint_guid);
      }
    }
  }
  int2dds_subscription_builtin_topic_data_seq_delete(seq);
  return departed;
}

// P3: mirror remote (other-participant) DDS endpoints discovered via int2dds
// built-in discovery into the standard rmw_dds_common GraphCache entity layer,
// keyed by their DDS endpoint GUID so the node-association data carried over
// Converts a core builtin-topic-data duration (sec/nsec, with an infinite value
// encoded as 0x7fffffff/0x7fffffff) to rmw_time_t, mapping infinite to
// RMW_DURATION_INFINITE so ros2 reports "Infinite" instead of "0 nanoseconds".
static rmw_time_t core_duration_to_rmw(int32_t sec, uint32_t nsec)
{
  if (sec == 0x7fffffff && nsec == 0x7fffffffu) {
    return RMW_DURATION_INFINITE;
  }
  return {static_cast<uint64_t>(sec), static_cast<uint64_t>(nsec)};
}

// Maps a core liveliness kind (0=AUTOMATIC, 1=MANUAL_BY_PARTICIPANT, 2=MANUAL_BY_TOPIC).
// Core MANUAL_BY_PARTICIPANT has no non-deprecated rmw equivalent; it is reported as
// MANUAL_BY_TOPIC (the supported manual-assertion policy), as ROS 2 no longer uses
// per-participant manual liveliness.
static rmw_qos_liveliness_policy_t core_liveliness_to_rmw(int32_t kind)
{
  switch (kind) {
    case 0:
      return RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    case 1:
    case 2:
      return RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    default:
      return RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
  }
}

// Builds the rmw QoS profile reported by get_*_info_by_topic from the discovered
// endpoint's propagated QoS. Reliability/durability/liveliness and the deadline,
// lifespan and liveliness-lease durations are taken from DDS discovery; history and
// depth are not carried over the wire so they stay at the profile defaults, the same
// as the reference RMWs. Durations default to infinite (the sentinel) when a getter
// is unavailable, matching a default-QoS endpoint.
static rmw_qos_profile_t build_remote_qos(
  int32_t reliability_kind, int32_t durability_kind, int32_t liveliness_kind,
  int32_t lease_sec, uint32_t lease_nsec, int32_t deadline_sec, uint32_t deadline_nsec,
  int32_t lifespan_sec, uint32_t lifespan_nsec)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = (reliability_kind == 0) ?
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT : RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  switch (durability_kind) {
    case 0:
      qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
      break;
    case 1:
      qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      break;
    default:
      qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
      break;
  }
  qos.liveliness = core_liveliness_to_rmw(liveliness_kind);
  qos.liveliness_lease_duration = core_duration_to_rmw(lease_sec, lease_nsec);
  qos.deadline = core_duration_to_rmw(deadline_sec, deadline_nsec);
  qos.lifespan = core_duration_to_rmw(lifespan_sec, lifespan_nsec);
  return qos;
}

// Reads an endpoint's USER_DATA via the given getter and isolates one "key=value;"
// pair. The user_data may carry several pairs and trailing padding that the
// rmw_dds_common parsers do not tolerate, and a bare find() of "typehash=" would
// also match inside "sertypehash=", so a match must start the buffer or follow ';'.
template<typename DataT>
static std::string read_endpoint_user_data_pair(
  DataT * data,
  Int2DdsRet (* get_user_data)(const DataT *, uint8_t *, uintptr_t, uintptr_t *),
  const std::string & key)
{
  uintptr_t size = 0;
  if (get_user_data(data, nullptr, 0, &size) != INT2DDS_RET_OK || size == 0) {
    return {};
  }
  std::vector<uint8_t> buf(size);
  if (get_user_data(data, buf.data(), buf.size(), &size) != INT2DDS_RET_OK) {
    return {};
  }
  const std::string ud(reinterpret_cast<const char *>(buf.data()), size);
  std::string::size_type pos = ud.find(key);
  while (pos != std::string::npos && pos != 0 && ud[pos - 1] != ';') {
    pos = ud.find(key, pos + 1);
  }
  if (pos == std::string::npos) {
    return {};
  }
  const auto semi = ud.find(';', pos);
  return (semi == std::string::npos) ? (ud.substr(pos) + ';') : ud.substr(pos, semi - pos + 1);
}

// Parses the rosidl type hash ("typehash=...;") that rmw_publisher/rmw_subscription
// encoded at creation, so get_*_info_by_topic reports the real type hash instead of
// a zero (INVALID) value. Returns a zero hash when no typehash key is present.
template<typename DataT>
static rosidl_type_hash_t read_endpoint_type_hash(
  DataT * data,
  Int2DdsRet (* get_user_data)(const DataT *, uint8_t *, uintptr_t, uintptr_t *))
{
  rosidl_type_hash_t type_hash{};
  const std::string pair = read_endpoint_user_data_pair(data, get_user_data, "typehash=");
  if (pair.empty()) {
    return type_hash;
  }
  rmw_dds_common::parse_type_hash_from_user_data(
    reinterpret_cast<const uint8_t *>(pair.data()), pair.size(), type_hash);
  return type_hash;
}

// Parses the service type hash ("sertypehash=...;") that Lyrical+ service/client
// endpoints append to USER_DATA. Zero hash when absent or on older distros.
template<typename DataT>
static rosidl_type_hash_t read_endpoint_service_type_hash(
  DataT * data,
  Int2DdsRet (* get_user_data)(const DataT *, uint8_t *, uintptr_t, uintptr_t *))
{
  rosidl_type_hash_t service_type_hash{};
#ifdef RMW_INT2DDS_HAS_SERVICE_ENDPOINT_INFO
  const std::string pair = read_endpoint_user_data_pair(data, get_user_data, "sertypehash=");
  if (!pair.empty()) {
    rmw_dds_common::parse_sertype_hash_from_user_data(
      reinterpret_cast<const uint8_t *>(pair.data()), pair.size(), service_type_hash);
  }
#else
  (void)data;
  (void)get_user_data;
#endif
  return service_type_hash;
}

// ros_discovery_info joins with the topic/type recorded here. Reconciles against
// ContextData::synced_remote_entities so endpoints that have departed are removed.
// Local endpoints are registered separately by the create/destroy hooks; the
// ros_discovery_info topic itself is skipped.
// Reads each discovered remote participant's USER_DATA ("enclave=<value>;") and registers
// its enclave in the GraphCache so get_node_names_with_enclaves reports the real enclave
// instead of "". The gid is the participant key, matching the per-endpoint participant_gid
// and the gid published in ros_discovery_info. Defaults to "/" when no enclave is encoded.
static void sync_remote_participant_enclaves(rmw_int2dds_cpp::ContextData * context_data)
{
  if (context_data == nullptr || !context_data->common || context_data->participant == nullptr) {
    return;
  }
  uintptr_t count = 0;
  if (int2dds_participant_get_discovered_participants(
      context_data->participant, nullptr, 0, &count) != INT2DDS_RET_OK || count == 0)
  {
    return;
  }
  std::vector<std::array<uint8_t, 16>> handles(count);
  if (int2dds_participant_get_discovered_participants(
      context_data->participant,
      reinterpret_cast<uint8_t(*)[16]>(handles.data()), count, &count) != INT2DDS_RET_OK)
  {
    return;
  }
  for (size_t i = 0; i < count; ++i) {
    Int2DdsParticipantBuiltinTopicData * data = nullptr;
    if (int2dds_participant_get_discovered_participant_data(
        context_data->participant,
        reinterpret_cast<const uint8_t(*)[16]>(&handles[i]), &data) != INT2DDS_RET_OK ||
      data == nullptr)
    {
      continue;
    }
    std::array<uint8_t, 512> buf{};
    uintptr_t size = 0;
    std::string enclave = "/";
    if (int2dds_participant_builtin_topic_data_get_user_data(
        data, buf.data(), buf.size(), &size) == INT2DDS_RET_OK && size > 0)
    {
      const std::string user_data(reinterpret_cast<const char *>(buf.data()), size);
      const std::string marker = "enclave=";
      const auto start = user_data.find(marker);
      if (start != std::string::npos) {
        const auto value_start = start + marker.size();
        const auto end = user_data.find(';', value_start);
        enclave = user_data.substr(
          value_start, end == std::string::npos ? std::string::npos : end - value_start);
      }
    }
    std::array<uint8_t, 12> participant_key = {};
    int2dds_participant_builtin_topic_data_get_key(
      data, reinterpret_cast<uint8_t(*)[12]>(&participant_key));
    int2dds_participant_builtin_topic_data_destroy(data);

    rmw_gid_t participant_gid = {};
    std::memcpy(participant_gid.data, participant_key.data(), 12);
    context_data->common->graph_cache.add_participant(participant_gid, enclave);
  }
}

static void bootstrap_remote_entities(rmw_int2dds_cpp::ContextData * context_data)
{
  if (context_data == nullptr || !context_data->common) {
    return;
  }

  // Telling a remote endpoint from one of our own only needs our own key.
  //
  // This used to ask for the list of discovered participants and keep an endpoint only if its
  // participant appeared in it. That list was whatever had answered so far -- the lookup returned
  // as soon as a single participant had -- so on a graph of 49 participants it came back with 6,
  // and every endpoint behind the other 43 was treated as belonging to nobody. Our own participant
  // key is known directly, so the list buys nothing.
  std::array<uint8_t, 12> local_participant_key = {};
  std::memcpy(
    local_participant_key.data(), context_data->common->gid.data, local_participant_key.size());

  struct RemoteEntity
  {
    std::string topic_name;
    std::string type_name;
    bool is_reader;
    std::array<uint8_t, 12> participant_key;
    rmw_qos_profile_t qos;
    rosidl_type_hash_t type_hash;
    rosidl_type_hash_t service_type_hash;
  };
  std::map<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, RemoteEntity> current;

  for_each_publication_snapshot(
    context_data, static_cast<int>(kGraphSnapshotTimeout.count()),
    [&](Int2DdsPublicationBuiltinTopicData * publication) {
      std::array<uint8_t, 16> endpoint_guid = {};
      if (int2dds_publication_builtin_topic_data_get_endpoint_guid(
          publication, reinterpret_cast<uint8_t(*)[16]>(&endpoint_guid)) != INT2DDS_RET_OK)
      {
        return;
      }
      std::array<uint8_t, 12> participant_key = {};
      const Int2DdsRet participant_ret = int2dds_publication_builtin_topic_data_get_participant_key(
        publication, reinterpret_cast<uint8_t(*)[12]>(&participant_key));
      const auto effective_key =
      (participant_ret == INT2DDS_RET_OK && !is_zero_discovery_key(participant_key)) ?
      participant_key : participant_key_from_endpoint_guid(endpoint_guid);
      if (effective_key == local_participant_key) {
        return;  // our own endpoint; the local side registers those itself
      }
      std::string topic_name;
      std::string type_name;
      if (!read_publication_string(
          int2dds_publication_builtin_topic_data_get_topic_name, publication, &topic_name) ||
      !read_publication_string(
          int2dds_publication_builtin_topic_data_get_type_name, publication, &type_name))
      {
        return;
      }
      if (topic_name == "ros_discovery_info") {
        return;
      }
      std::array<uint8_t, RMW_GID_STORAGE_SIZE> key = {};
      std::memcpy(key.data(), endpoint_guid.data(), RMW_GID_STORAGE_SIZE);
      int32_t reliability_kind = 1;
      int32_t durability_kind = 0;
      int32_t liveliness_kind = 0;
      int32_t lease_sec = 0x7fffffff, deadline_sec = 0x7fffffff, lifespan_sec = 0x7fffffff;
      uint32_t lease_nsec = 0x7fffffffu, deadline_nsec = 0x7fffffffu, lifespan_nsec = 0x7fffffffu;
      int2dds_publication_builtin_topic_data_get_reliability_kind(publication, &reliability_kind);
      int2dds_publication_builtin_topic_data_get_durability_kind(publication, &durability_kind);
      int2dds_publication_builtin_topic_data_get_liveliness_kind(publication, &liveliness_kind);
      int2dds_publication_builtin_topic_data_get_liveliness_lease_duration(
        publication, &lease_sec, &lease_nsec);
      int2dds_publication_builtin_topic_data_get_deadline(publication, &deadline_sec,
      &deadline_nsec);
      int2dds_publication_builtin_topic_data_get_lifespan(publication, &lifespan_sec,
      &lifespan_nsec);
      current[key] = RemoteEntity{
        topic_name, type_name, false, effective_key,
        build_remote_qos(
          reliability_kind, durability_kind, liveliness_kind, lease_sec, lease_nsec,
          deadline_sec, deadline_nsec, lifespan_sec, lifespan_nsec),
        read_endpoint_type_hash(
          publication, int2dds_publication_builtin_topic_data_get_user_data),
        read_endpoint_service_type_hash(
          publication, int2dds_publication_builtin_topic_data_get_user_data)};
    });

  for_each_subscription_snapshot(
    context_data, static_cast<int>(kGraphSnapshotTimeout.count()),
    [&](Int2DdsSubscriptionBuiltinTopicData * subscription) {
      std::array<uint8_t, 16> endpoint_guid = {};
      if (int2dds_subscription_builtin_topic_data_get_endpoint_guid(
          subscription, reinterpret_cast<uint8_t(*)[16]>(&endpoint_guid)) != INT2DDS_RET_OK)
      {
        return;
      }
      std::array<uint8_t, 12> participant_key = {};
      const Int2DdsRet participant_ret =
      int2dds_subscription_builtin_topic_data_get_participant_key(
        subscription, reinterpret_cast<uint8_t(*)[12]>(&participant_key));
      const auto effective_key =
      (participant_ret == INT2DDS_RET_OK && !is_zero_discovery_key(participant_key)) ?
      participant_key : participant_key_from_endpoint_guid(endpoint_guid);
      if (effective_key == local_participant_key) {
        return;  // our own endpoint; the local side registers those itself
      }
      std::string topic_name;
      std::string type_name;
      if (!read_subscription_string(
          int2dds_subscription_builtin_topic_data_get_topic_name, subscription, &topic_name) ||
      !read_subscription_string(
          int2dds_subscription_builtin_topic_data_get_type_name, subscription, &type_name))
      {
        return;
      }
      if (topic_name == "ros_discovery_info") {
        return;
      }
      std::array<uint8_t, RMW_GID_STORAGE_SIZE> key = {};
      std::memcpy(key.data(), endpoint_guid.data(), RMW_GID_STORAGE_SIZE);
      int32_t reliability_kind = 1;
      int32_t durability_kind = 0;
      int32_t liveliness_kind = 0;
      int32_t lease_sec = 0x7fffffff, deadline_sec = 0x7fffffff;
      uint32_t lease_nsec = 0x7fffffffu, deadline_nsec = 0x7fffffffu;
      int2dds_subscription_builtin_topic_data_get_reliability_kind(subscription, &reliability_kind);
      int2dds_subscription_builtin_topic_data_get_durability_kind(subscription, &durability_kind);
      int2dds_subscription_builtin_topic_data_get_liveliness_kind(subscription, &liveliness_kind);
      int2dds_subscription_builtin_topic_data_get_liveliness_lease_duration(
        subscription, &lease_sec, &lease_nsec);
      int2dds_subscription_builtin_topic_data_get_deadline(
        subscription, &deadline_sec, &deadline_nsec);
      // Subscriptions do not carry lifespan (writer-only QoS); report it as infinite.
      current[key] = RemoteEntity{
        topic_name, type_name, true, effective_key,
        build_remote_qos(
          reliability_kind, durability_kind, liveliness_kind, lease_sec, lease_nsec,
          deadline_sec, deadline_nsec, 0x7fffffff, 0x7fffffffu),
        read_endpoint_type_hash(
          subscription, int2dds_subscription_builtin_topic_data_get_user_data),
        read_endpoint_service_type_hash(
          subscription, int2dds_subscription_builtin_topic_data_get_user_data)};
    });

  // Departures come from DDS saying so, never from absence in the pass above.
  //
  // A snapshot reports what the builtin reader cache holds at that instant, not what is alive.
  // Deleting whatever is missing from it cannot tell an endpoint that has departed from one whose
  // announcement has not arrived yet, and while a graph is still coming up those look identical.
  // What that cost was whole participants dropping out at once -- every parameter service of a
  // node going together, to be rediscovered moments later.
  std::vector<std::pair<std::array<uint8_t, 16>, bool>> gone;
  for (const auto & guid : collect_departed_publications(
      context_data, static_cast<int>(kGraphSnapshotTimeout.count())))
  {
    gone.emplace_back(guid, false);
  }
  for (const auto & guid : collect_departed_subscriptions(
      context_data, static_cast<int>(kGraphSnapshotTimeout.count())))
  {
    gone.emplace_back(guid, true);
  }

  std::lock_guard<std::mutex> lock(context_data->remote_sync_mutex);
  auto & synced = context_data->synced_remote_entities;

  for (const auto & departed : gone) {
    std::array<uint8_t, RMW_GID_STORAGE_SIZE> key = {};
    std::memcpy(key.data(), departed.first.data(), departed.first.size());
    auto it = synced.find(key);
    if (it == synced.end()) {
      continue;  // never ours, or already forgotten: a read leaves the instance in place, so the
                 // same departure comes back on every pass
    }
    rmw_gid_t gid = {};
    std::memcpy(gid.data, key.data(), RMW_GID_STORAGE_SIZE);
    context_data->common->graph_cache.remove_entity(gid, departed.second);
    synced.erase(it);
  }

  for (const auto & entry : current) {
    if (synced.find(entry.first) != synced.end()) {
      continue;
    }
    rmw_gid_t gid = {};
    std::memcpy(gid.data, entry.first.data(), RMW_GID_STORAGE_SIZE);
    // Real participant key (GUID prefix) so rmw_dds_common associates this endpoint with
    // the node that owns it (get_*_info_by_topic / by_node). Matches the participant gid
    // published in ros_discovery_info (also the GUID prefix); an empty value here left
    // every discovered endpoint reported as _CREATED_BY_BARE_DDS_APP_.
    rmw_gid_t participant_gid = {};
    std::memcpy(participant_gid.data, entry.second.participant_key.data(), 12);
#ifdef RMW_INT2DDS_HAS_SERVICE_ENDPOINT_INFO
    // Only service endpoints carry a sertypehash; version 0 means unset.
    const rosidl_type_hash_t & service_type_hash = entry.second.service_type_hash;
    context_data->common->graph_cache.add_entity(
      gid, entry.second.topic_name, entry.second.type_name, entry.second.type_hash,
      participant_gid, entry.second.qos, entry.second.is_reader,
      service_type_hash.version != 0 ? &service_type_hash : nullptr);
#else
    context_data->common->graph_cache.add_entity(
      gid, entry.second.topic_name, entry.second.type_name, entry.second.type_hash,
      participant_gid, entry.second.qos, entry.second.is_reader);
#endif
    synced[entry.first] = entry.second.is_reader;
  }
}

// Per-query graph reconcile (lightweight). Endpoint arrivals are mirrored into the GraphCache
// incrementally by the SEDP push callback (rmw_int2dds_endpoint_discovery_cb), and the full ALIVE
// snapshot is seeded once at bootstrap (bootstrap_remote_entities), so a graph query no longer
// pulls the O(N) ALIVE snapshot on every call -- that pull dominated graph-query latency. What a
// query still does is prune departures the callback may have missed: a cheap pass (the NOT_ALIVE
// snapshot is normally empty) that keeps stale endpoints from lingering, with no background thread
// and no periodic timer.
static void sync_remote_entities_to_common(rmw_int2dds_cpp::ContextData * context_data)
{
  if (context_data == nullptr || !context_data->common) {
    return;
  }

  std::vector<std::pair<std::array<uint8_t, 16>, bool>> gone;
  for (const auto & guid : collect_departed_publications(
      context_data, static_cast<int>(kGraphSnapshotTimeout.count())))
  {
    gone.emplace_back(guid, false);
  }
  for (const auto & guid : collect_departed_subscriptions(
      context_data, static_cast<int>(kGraphSnapshotTimeout.count())))
  {
    gone.emplace_back(guid, true);
  }
  if (gone.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(context_data->remote_sync_mutex);
  auto & synced = context_data->synced_remote_entities;
  for (const auto & departed : gone) {
    std::array<uint8_t, RMW_GID_STORAGE_SIZE> key = {};
    std::memcpy(key.data(), departed.first.data(), departed.first.size());
    auto it = synced.find(key);
    if (it == synced.end()) {
      continue;
    }
    rmw_gid_t gid = {};
    std::memcpy(gid.data, key.data(), RMW_GID_STORAGE_SIZE);
    context_data->common->graph_cache.remove_entity(gid, departed.second);
    synced.erase(it);
  }
}

// ============================================================================
// SEDP push: incremental consumer of the core endpoint-discovery callback.
//
// Fires on the core SEDP thread for each remote endpoint discovered or disposed.
// Mirrors the per-endpoint logic of sync_remote_entities_to_common(), but for a
// single endpoint, so the graph_cache stays current without a per-query resync.
// All access to context_data->common is done under remote_sync_mutex with a null
// check; fini_discovery resets common under the same mutex, so a callback racing
// teardown either completes before the reset or sees a null common and skips.
// ============================================================================
extern "C" void rmw_int2dds_endpoint_discovery_cb(
  void * ctx, int32_t /*is_writer*/, int32_t is_alive,
  const Int2DdsPublicationBuiltinTopicData * pub_data,
  const Int2DdsSubscriptionBuiltinTopicData * sub_data,
  const uint8_t(*guid)[16])
{
  auto * context_data = static_cast<rmw_int2dds_cpp::ContextData *>(ctx);
  if (context_data == nullptr) {
    return;
  }

  // Disposed: drop the entity we previously added for this GUID.
  if (is_alive == 0) {
    std::array<uint8_t, RMW_GID_STORAGE_SIZE> key = {};
    std::memcpy(key.data(), guid, 16);
    std::lock_guard<std::mutex> lock(context_data->remote_sync_mutex);
    if (!context_data->common) {
      return;
    }
    auto & synced = context_data->synced_remote_entities;
    auto it = synced.find(key);
    if (it == synced.end()) {
      return;
    }
    rmw_gid_t gid = {};
    std::memcpy(gid.data, key.data(), RMW_GID_STORAGE_SIZE);
    context_data->common->graph_cache.remove_entity(gid, it->second);
    synced.erase(it);
    return;
  }

  // Alive writer.
  if (pub_data != nullptr) {
    auto * publication = const_cast<Int2DdsPublicationBuiltinTopicData *>(pub_data);
    std::array<uint8_t, 16> endpoint_guid = {};
    if (int2dds_publication_builtin_topic_data_get_endpoint_guid(
        publication, reinterpret_cast<uint8_t(*)[16]>(&endpoint_guid)) != INT2DDS_RET_OK)
    {
      return;
    }
    std::array<uint8_t, 12> participant_key = {};
    const Int2DdsRet participant_ret = int2dds_publication_builtin_topic_data_get_participant_key(
      publication, reinterpret_cast<uint8_t(*)[12]>(&participant_key));
    const auto effective_key =
      (participant_ret == INT2DDS_RET_OK && !is_zero_discovery_key(participant_key)) ?
      participant_key : participant_key_from_endpoint_guid(endpoint_guid);
    std::string topic_name;
    std::string type_name;
    if (!read_publication_string(
        int2dds_publication_builtin_topic_data_get_topic_name, publication, &topic_name) ||
      !read_publication_string(
        int2dds_publication_builtin_topic_data_get_type_name, publication, &type_name))
    {
      return;
    }
    if (topic_name == "ros_discovery_info") {
      return;
    }
    int32_t reliability_kind = 1;
    int32_t durability_kind = 0;
    int32_t liveliness_kind = 0;
    int32_t lease_sec = 0x7fffffff;
    int32_t deadline_sec = 0x7fffffff;
    int32_t lifespan_sec = 0x7fffffff;
    uint32_t lease_nsec = 0x7fffffffu;
    uint32_t deadline_nsec = 0x7fffffffu;
    uint32_t lifespan_nsec = 0x7fffffffu;
    int2dds_publication_builtin_topic_data_get_reliability_kind(publication, &reliability_kind);
    int2dds_publication_builtin_topic_data_get_durability_kind(publication, &durability_kind);
    int2dds_publication_builtin_topic_data_get_liveliness_kind(publication, &liveliness_kind);
    int2dds_publication_builtin_topic_data_get_liveliness_lease_duration(
      publication, &lease_sec, &lease_nsec);
    int2dds_publication_builtin_topic_data_get_deadline(publication, &deadline_sec, &deadline_nsec);
    int2dds_publication_builtin_topic_data_get_lifespan(publication, &lifespan_sec, &lifespan_nsec);
    const rmw_qos_profile_t qos = build_remote_qos(
      reliability_kind, durability_kind, liveliness_kind, lease_sec, lease_nsec,
      deadline_sec, deadline_nsec, lifespan_sec, lifespan_nsec);

    std::array<uint8_t, RMW_GID_STORAGE_SIZE> key = {};
    std::memcpy(key.data(), endpoint_guid.data(), endpoint_guid.size());
    std::lock_guard<std::mutex> lock(context_data->remote_sync_mutex);
    if (!context_data->common) {
      return;
    }
    std::array<uint8_t, 12> local_participant_key = {};
    std::memcpy(
      local_participant_key.data(), context_data->common->gid.data,
      local_participant_key.size());
    if (effective_key == local_participant_key) {
      return;  // our own endpoint; the local side registers those itself
    }
    auto & synced = context_data->synced_remote_entities;
    if (synced.find(key) != synced.end()) {
      return;
    }
    rmw_gid_t gid = {};
    std::memcpy(gid.data, key.data(), RMW_GID_STORAGE_SIZE);
    rmw_gid_t participant_gid = {};
    std::memcpy(participant_gid.data, effective_key.data(), 12);
    // Jazzy's GraphCache takes the type hash; the overload without it is
    // deprecated and would register this endpoint as RIHS01_0000... The push
    // callback wins the dedup race against sync_remote_entities_to_common, so
    // dropping the hash here would make it permanent.
    //
    // The same argument applies to the service type hash below: this path and
    // sync_remote_entities_to_common share the synced_remote_entities dedup, so
    // whichever gets there first decides what the graph cache holds. Registering
    // without it here left every remote service/client endpoint reporting an
    // unset hash from rmw_get_{clients,servers}_info_by_service - the local
    // endpoints looked right because common_add_local_entity does pass it.
#ifdef RMW_INT2DDS_HAS_SERVICE_ENDPOINT_INFO
    // Only service endpoints carry a sertypehash; version 0 means unset.
    const rosidl_type_hash_t service_type_hash =
      read_endpoint_service_type_hash(
      publication, int2dds_publication_builtin_topic_data_get_user_data);
    context_data->common->graph_cache.add_entity(
      gid, topic_name, type_name,
      read_endpoint_type_hash(
        publication, int2dds_publication_builtin_topic_data_get_user_data),
      participant_gid, qos, false,
      service_type_hash.version != 0 ? &service_type_hash : nullptr);
#else
    context_data->common->graph_cache.add_entity(
      gid, topic_name, type_name,
      read_endpoint_type_hash(
        publication, int2dds_publication_builtin_topic_data_get_user_data),
      participant_gid, qos, false);
#endif
    synced[key] = false;
    return;
  }

  // Alive reader.
  if (sub_data != nullptr) {
    auto * subscription = const_cast<Int2DdsSubscriptionBuiltinTopicData *>(sub_data);
    std::array<uint8_t, 16> endpoint_guid = {};
    if (int2dds_subscription_builtin_topic_data_get_endpoint_guid(
        subscription, reinterpret_cast<uint8_t(*)[16]>(&endpoint_guid)) != INT2DDS_RET_OK)
    {
      return;
    }
    std::array<uint8_t, 12> participant_key = {};
    const Int2DdsRet participant_ret =
      int2dds_subscription_builtin_topic_data_get_participant_key(
      subscription, reinterpret_cast<uint8_t(*)[12]>(&participant_key));
    const auto effective_key =
      (participant_ret == INT2DDS_RET_OK && !is_zero_discovery_key(participant_key)) ?
      participant_key : participant_key_from_endpoint_guid(endpoint_guid);
    std::string topic_name;
    std::string type_name;
    if (!read_subscription_string(
        int2dds_subscription_builtin_topic_data_get_topic_name, subscription, &topic_name) ||
      !read_subscription_string(
        int2dds_subscription_builtin_topic_data_get_type_name, subscription, &type_name))
    {
      return;
    }
    if (topic_name == "ros_discovery_info") {
      return;
    }
    int32_t reliability_kind = 1;
    int32_t durability_kind = 0;
    int32_t liveliness_kind = 0;
    int32_t lease_sec = 0x7fffffff;
    int32_t deadline_sec = 0x7fffffff;
    uint32_t lease_nsec = 0x7fffffffu;
    uint32_t deadline_nsec = 0x7fffffffu;
    int2dds_subscription_builtin_topic_data_get_reliability_kind(subscription, &reliability_kind);
    int2dds_subscription_builtin_topic_data_get_durability_kind(subscription, &durability_kind);
    int2dds_subscription_builtin_topic_data_get_liveliness_kind(subscription, &liveliness_kind);
    int2dds_subscription_builtin_topic_data_get_liveliness_lease_duration(
      subscription, &lease_sec, &lease_nsec);
    int2dds_subscription_builtin_topic_data_get_deadline(
      subscription, &deadline_sec, &deadline_nsec);
    const rmw_qos_profile_t qos = build_remote_qos(
      reliability_kind, durability_kind, liveliness_kind, lease_sec, lease_nsec,
      deadline_sec, deadline_nsec, 0x7fffffff, 0x7fffffffu);

    std::array<uint8_t, RMW_GID_STORAGE_SIZE> key = {};
    std::memcpy(key.data(), endpoint_guid.data(), endpoint_guid.size());
    std::lock_guard<std::mutex> lock(context_data->remote_sync_mutex);
    if (!context_data->common) {
      return;
    }
    std::array<uint8_t, 12> local_participant_key = {};
    std::memcpy(
      local_participant_key.data(), context_data->common->gid.data,
      local_participant_key.size());
    if (effective_key == local_participant_key) {
      return;
    }
    auto & synced = context_data->synced_remote_entities;
    if (synced.find(key) != synced.end()) {
      return;
    }
    rmw_gid_t gid = {};
    std::memcpy(gid.data, key.data(), RMW_GID_STORAGE_SIZE);
    rmw_gid_t participant_gid = {};
    std::memcpy(participant_gid.data, effective_key.data(), 12);
    // Same as the writer branch above: the service type hash has to ride along
    // here too, or the push path's dedup win makes its absence permanent.
#ifdef RMW_INT2DDS_HAS_SERVICE_ENDPOINT_INFO
    const rosidl_type_hash_t service_type_hash =
      read_endpoint_service_type_hash(
      subscription, int2dds_subscription_builtin_topic_data_get_user_data);
    context_data->common->graph_cache.add_entity(
      gid, topic_name, type_name,
      read_endpoint_type_hash(
        subscription, int2dds_subscription_builtin_topic_data_get_user_data),
      participant_gid, qos, true,
      service_type_hash.version != 0 ? &service_type_hash : nullptr);
#else
    context_data->common->graph_cache.add_entity(
      gid, topic_name, type_name,
      read_endpoint_type_hash(
        subscription, int2dds_subscription_builtin_topic_data_get_user_data),
      participant_gid, qos, true);
#endif
    synced[key] = true;
    return;
  }
}

extern "C" void rmw_int2dds_endpoint_discovery_noop(
  void *, int32_t, int32_t,
  const Int2DdsPublicationBuiltinTopicData *,
  const Int2DdsSubscriptionBuiltinTopicData *,
  const uint8_t(*)[16]) {}

namespace rmw_int2dds_cpp
{

void enable_endpoint_push(ContextData * context_data)
{
  if (context_data == nullptr || context_data->participant == nullptr) {
    return;
  }
  int2dds_participant_set_endpoint_discovery_callback(
    context_data->participant, ::rmw_int2dds_endpoint_discovery_cb, context_data);
  // Bootstrap: seed endpoints discovered before the callback was registered.
  // The dedup against synced_remote_entities makes this idempotent with the
  // callback, so the ordering register-then-seed loses nothing.
  ::bootstrap_remote_entities(context_data);
}

void disable_endpoint_push(ContextData * context_data)
{
  if (context_data == nullptr || context_data->participant == nullptr) {
    return;
  }
  int2dds_participant_set_endpoint_discovery_callback(
    context_data->participant, ::rmw_int2dds_endpoint_discovery_noop, nullptr);
}

}  // namespace rmw_int2dds_cpp


static rmw_ret_t validate_string_array_output(rcutils_string_array_t * array)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(array, RMW_RET_INVALID_ARGUMENT);
  if (rmw_check_zero_rmw_string_array(array) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("string array is not zero initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

static rmw_ret_t validate_names_and_types_output(rmw_names_and_types_t * names_and_types)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(names_and_types, RMW_RET_INVALID_ARGUMENT);
  if (rmw_names_and_types_check_zero(names_and_types) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("names and types array is not zero initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

static rmw_ret_t validate_allocator(rcutils_allocator_t * allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RMW_SET_ERROR_MSG("allocator is invalid");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

static rmw_ret_t validate_topic_endpoint_info_array_output(
  rmw_topic_endpoint_info_array_t * endpoint_info_array)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(endpoint_info_array, RMW_RET_INVALID_ARGUMENT);
  if (rmw_topic_endpoint_info_array_check_zero(endpoint_info_array) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("topic endpoint info array is not zero initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

static rmw_ret_t validate_ros_topic_name(const char * topic_name)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);

  int validation_result = RMW_TOPIC_VALID;
  size_t invalid_index = 0;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, &invalid_index);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  if (validation_result != RMW_TOPIC_VALID) {
    RMW_SET_ERROR_MSG("invalid topic name");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

// P3: validate the node handle and recover its ContextData (with the standard
// rmw_dds_common discovery context), without depending on the heuristic GraphCache.
static rmw_ret_t check_node_and_get_context(
  const rmw_node_t * node,
  rmw_int2dds_cpp::ContextData ** context_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }
  auto * node_data = static_cast<rmw_int2dds_cpp::NodeData *>(node->data);
  if (node_data == nullptr || node_data->context_data == nullptr) {
    RMW_SET_ERROR_MSG("node data is null");
    return RMW_RET_ERROR;
  }
  if (!node_data->context_data->common) {
    RMW_SET_ERROR_MSG("discovery context not initialized");
    return RMW_RET_ERROR;
  }
  *context_data = node_data->context_data;
  return RMW_RET_OK;
}

// Validate only the format of a target node name/namespace (existence is reported
// by the rmw_dds_common GraphCache query itself via RMW_RET_NODE_NAME_NON_EXISTENT).
static rmw_ret_t validate_node_format(const char * node_name, const char * node_namespace)
{
  int validation_result = 0;
  rmw_ret_t ret = rmw_validate_node_name(node_name, &validation_result, nullptr);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  if (validation_result != RMW_NODE_NAME_VALID) {
    RMW_SET_ERROR_MSG("invalid node name");
    return RMW_RET_INVALID_ARGUMENT;
  }
  ret = rmw_validate_namespace(node_namespace, &validation_result, nullptr);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  if (validation_result != RMW_NAMESPACE_VALID) {
    RMW_SET_ERROR_MSG("invalid node namespace");
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

// Demangle functions for the rmw_dds_common GraphCache queries. Each returns ""
// for topics that should be excluded from the result set.
static std::string demangle_ros_topic_only(const std::string & dds_topic)
{
  // "rt/<topic>" -> "/<topic>"; service/infra topics are filtered out.
  if (dds_topic.size() > 2 && dds_topic.compare(0, 2, "rt") == 0) {
    return dds_topic.substr(2);
  }
  return "";
}
static std::string demangle_service_request_only(const std::string & dds_topic)
{
  // Service request topics ("rq<service>Request") -> service name; others filtered.
  if (is_service_request_topic(dds_topic)) {
    return demangle_service_name(dds_topic);
  }
  return "";
}
static std::string identity_demangle(const std::string & value)
{
  return value;
}
static std::string demangle_service_from_topic(const std::string & dds_topic)
{
  // Any service request/reply topic ("rq<svc>Request" / "rr<svc>Reply") -> service
  // name; non-service topics are filtered out.
  if (is_service_topic(dds_topic)) {
    return demangle_service_name(dds_topic);
  }
  return "";
}

extern "C"
{
rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  ret = validate_string_array_output(node_names);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_string_array_output(node_namespaces);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  // Standard node discovery: read from the rmw_dds_common GraphCache, populated
  // from ros_discovery_info (replaces the service-name heuristic).
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  return context_data->common->graph_cache.get_node_names(
    node_names, node_namespaces, nullptr, &allocator);
}

rmw_ret_t
rmw_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  ret = validate_string_array_output(node_names);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_string_array_output(node_namespaces);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_string_array_output(enclaves);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  // get_node_names does not call sync_remote_entities_to_common, so register the
  // discovered remote participants' enclaves here before reading them back.
  sync_remote_participant_enclaves(context_data);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  return context_data->common->graph_cache.get_node_names(
    node_names, node_namespaces, enclaves, &allocator);
}

rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_types_output(topic_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  sync_remote_entities_to_common(context_data);

  return context_data->common->graph_cache.get_names_and_types(
    no_demangle ? identity_demangle : demangle_ros_topic_only,
    no_demangle ? identity_demangle : demangle_dds_message_type_name,
    allocator, topic_names_and_types);
}

rmw_ret_t
rmw_get_service_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_types_output(service_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  sync_remote_entities_to_common(context_data);

  // Services span request ("rq...Request") and reply ("rr...Reply") topics; both
  // demangle to the same service name, so the service appears once with its type.
  return context_data->common->graph_cache.get_names_and_types(
    demangle_service_from_topic, demangle_dds_service_type_name,
    allocator, service_names_and_types);
}

rmw_ret_t
rmw_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_types_output(topic_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_node_format(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  // Standard rmw_dds_common: ingest remote endpoints, then read the node's writers.
  sync_remote_entities_to_common(context_data);

  return context_data->common->graph_cache.get_writer_names_and_types_by_node(
    node_name, node_namespace,
    no_demangle ? identity_demangle : demangle_ros_topic_only,
    no_demangle ? identity_demangle : demangle_dds_message_type_name,
    allocator, topic_names_and_types);
}

rmw_ret_t
rmw_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_types_output(topic_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  ret = validate_node_format(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  sync_remote_entities_to_common(context_data);

  return context_data->common->graph_cache.get_reader_names_and_types_by_node(
    node_name, node_namespace,
    no_demangle ? identity_demangle : demangle_ros_topic_only,
    no_demangle ? identity_demangle : demangle_dds_message_type_name,
    allocator, topic_names_and_types);
}

rmw_ret_t
rmw_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_types_output(service_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_node_format(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  sync_remote_entities_to_common(context_data);

  // A service exposes a request reader (rq<service>Request); read the node's
  // readers and keep only service-request topics.
  return context_data->common->graph_cache.get_reader_names_and_types_by_node(
    node_name, node_namespace,
    demangle_service_request_only, demangle_dds_service_type_name,
    allocator, service_names_and_types);
}

rmw_ret_t
rmw_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_names_and_types_output(service_names_and_types);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_node_format(node_name, node_namespace);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  sync_remote_entities_to_common(context_data);

  // A client exposes a request writer (rq<service>Request); read the node's
  // writers and keep only service-request topics.
  return context_data->common->graph_cache.get_writer_names_and_types_by_node(
    node_name, node_namespace,
    demangle_service_request_only, demangle_dds_service_type_name,
    allocator, service_names_and_types);
}

rmw_ret_t
rmw_get_publishers_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * publishers_info)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_topic_endpoint_info_array_output(publishers_info);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  // No per-query resync: the SEDP push callback keeps the graph_cache current.
  const std::string lookup_topic =
    no_mangle ? std::string(topic_name) : "rt" + std::string(topic_name);
  return context_data->common->graph_cache.get_writers_info_by_topic(
    lookup_topic,
    no_mangle ? identity_demangle : demangle_dds_message_type_name,
    allocator, publishers_info);
}

rmw_ret_t
rmw_get_subscriptions_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * subscriptions_info)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = validate_topic_endpoint_info_array_output(subscriptions_info);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  // No per-query resync: the SEDP push callback keeps the graph_cache current.
  const std::string lookup_topic =
    no_mangle ? std::string(topic_name) : "rt" + std::string(topic_name);
  return context_data->common->graph_cache.get_readers_info_by_topic(
    lookup_topic,
    no_mangle ? identity_demangle : demangle_dds_message_type_name,
    allocator, subscriptions_info);
}

rmw_ret_t
rmw_count_publishers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);
  ret = validate_ros_topic_name(topic_name);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  // No per-query resync: the SEDP push callback keeps writer counts current.
  const std::string lookup_topic = "rt" + std::string(topic_name);
  return context_data->common->graph_cache.get_writer_count(lookup_topic, count);
}

rmw_ret_t
rmw_count_subscribers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);
  ret = validate_ros_topic_name(topic_name);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  // No per-query resync: the SEDP push callback keeps reader counts current.
  const std::string lookup_topic = "rt" + std::string(topic_name);
  return context_data->common->graph_cache.get_reader_count(lookup_topic, count);
}

rmw_ret_t
rmw_count_clients(
  const rmw_node_t * node,
  const char * service_name,
  size_t * count)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  sync_remote_entities_to_common(context_data);

  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);
  int service_name_validation = 0;
  if (rmw_validate_full_topic_name(service_name, &service_name_validation, nullptr) != RMW_RET_OK ||
    service_name_validation != RMW_TOPIC_VALID)
  {
    RMW_SET_ERROR_MSG("invalid service name");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Each client owns the request writer on "rq<service>Request".
  const std::string request_topic = "rq" + std::string(service_name) + "Request";
  return context_data->common->graph_cache.get_writer_count(request_topic, count);
}

rmw_ret_t
rmw_count_services(
  const rmw_node_t * node,
  const char * service_name,
  size_t * count)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  sync_remote_entities_to_common(context_data);

  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);
  int service_name_validation = 0;
  if (rmw_validate_full_topic_name(service_name, &service_name_validation, nullptr) != RMW_RET_OK ||
    service_name_validation != RMW_TOPIC_VALID)
  {
    RMW_SET_ERROR_MSG("invalid service name");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Each service owns the request reader on "rq<service>Request".
  const std::string request_topic = "rq" + std::string(service_name) + "Request";
  return context_data->common->graph_cache.get_reader_count(request_topic, count);
}

#ifdef RMW_INT2DDS_HAS_SERVICE_ENDPOINT_INFO
// A service travels as two mangled DDS topics: "rq<service>Request" and
// "rr<service>Reply" (the names rmw_service.cpp/rmw_client.cpp create; note
// ros_service_to_dds_response_topic() in topic_names.cpp says "Response" but is
// not what the service/client creation path uses). A client owns a reply reader
// plus a request writer, a server the mirror pair; the GraphCache assembles
// per-node service endpoint entries from the reader/writer views of the two topics.
static rmw_ret_t get_service_endpoints_info(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * service_name,
  bool no_mangle,
  bool servers,
  rmw_service_endpoint_info_array_t * endpoints_info)
{
  rmw_int2dds_cpp::ContextData * context_data = nullptr;
  rmw_ret_t ret = check_node_and_get_context(node, &context_data);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
  ret = validate_allocator(allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(endpoints_info, RMW_RET_INVALID_ARGUMENT);
  if (rmw_service_endpoint_info_array_check_zero(endpoints_info) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("service endpoint info array is not zero initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (no_mangle) {
    // The DDS topics backing a service only exist in mangled form, so an
    // unmangled lookup has nothing to name (same contract as rmw_fastrtps).
    RMW_SET_ERROR_MSG(
      "'no_mangle' is not supported for services; use "
      "rmw_get_publishers_info_by_topic / rmw_get_subscriptions_info_by_topic "
      "on the mangled topic names instead");
    return RMW_RET_INVALID_ARGUMENT;
  }

  sync_remote_entities_to_common(context_data);

  const std::string request_topic = "rq" + std::string(service_name) + "Request";
  const std::string response_topic = "rr" + std::string(service_name) + "Reply";
  const std::string & reader_topic = servers ? request_topic : response_topic;
  const std::string & writer_topic = servers ? response_topic : request_topic;

  rmw_topic_endpoint_info_array_t readers_info =
    rmw_get_zero_initialized_topic_endpoint_info_array();
  rmw_topic_endpoint_info_array_t writers_info =
    rmw_get_zero_initialized_topic_endpoint_info_array();

  ret = context_data->common->graph_cache.get_readers_info_by_topic(
    reader_topic, demangle_dds_service_type_name, allocator, &readers_info);
  if (ret == RMW_RET_OK) {
    ret = context_data->common->graph_cache.get_writers_info_by_topic(
      writer_topic, demangle_dds_service_type_name, allocator, &writers_info);
  }
  if (ret == RMW_RET_OK) {
    ret = servers ?
      context_data->common->graph_cache.get_servers_info_by_service(
      &readers_info, &writers_info, allocator, endpoints_info) :
      context_data->common->graph_cache.get_clients_info_by_service(
      &readers_info, &writers_info, allocator, endpoints_info);
  }

  // The two intermediate views are temporary on success and failure alike.
  const rmw_ret_t fini_readers = rmw_topic_endpoint_info_array_fini(&readers_info, allocator);
  const rmw_ret_t fini_writers = rmw_topic_endpoint_info_array_fini(&writers_info, allocator);
  if (ret == RMW_RET_OK && (fini_readers != RMW_RET_OK || fini_writers != RMW_RET_OK)) {
    ret = RMW_RET_ERROR;
  }
  return ret;
}

rmw_ret_t
rmw_get_clients_info_by_service(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * service_name,
  bool no_mangle,
  rmw_service_endpoint_info_array_t * clients_info)
{
  return get_service_endpoints_info(
    node, allocator, service_name, no_mangle, false, clients_info);
}

rmw_ret_t
rmw_get_servers_info_by_service(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * service_name,
  bool no_mangle,
  rmw_service_endpoint_info_array_t * servers_info)
{
  return get_service_endpoints_info(
    node, allocator, service_name, no_mangle, true, servers_info);
}
#endif  // RMW_INT2DDS_HAS_SERVICE_ENDPOINT_INFO
}  // extern "C"
