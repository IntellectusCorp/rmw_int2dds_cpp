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

#ifndef COMMON__TYPE_HASH_QOS_HPP_
#define COMMON__TYPE_HASH_QOS_HPP_

#include <cstdint>
#include <string>

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

#include "rmw/types.h"

#include "rmw_int2dds_cpp/foxy_compat.hpp"  // RMW_DURATION_INFINITE on Foxy

#include "int2dds-ffi.h"  // NOLINT(build/include)

namespace rmw_int2dds_cpp
{

// Resolve system-default / unknown QoS policies to the concrete defaults that
// int2dds actually applies, so rmw_*_get_actual_qos reports usable values (the
// RMW conformance suite requires actual QoS to differ from system_default/unknown).
inline void
resolve_actual_qos(rmw_qos_profile_t * qos)
{
  if (qos == nullptr) {
    return;
  }
  if (qos->history == RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT ||
    qos->history == RMW_QOS_POLICY_HISTORY_UNKNOWN)
  {
    qos->history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }
  if (qos->depth == 0) {
    qos->depth = 10;
  }
  if (qos->reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ||
    qos->reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN)
  {
    qos->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if (qos->durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ||
    qos->durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN)
  {
    qos->durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
#ifdef RMW_QOS_DEADLINE_BEST_AVAILABLE
  // Publishers and subscriptions negotiate BEST_AVAILABLE against existing
  // endpoints (qos_profile_get_best_available_for_topic_*), so it never reaches
  // here for them. Services and clients do not run that negotiation, so a
  // BEST_AVAILABLE request survives; map it to the concrete value the create
  // path actually applies (Reliable/Volatile) so get_actual_qos does not report
  // the unresolved BEST_AVAILABLE marker.
  if (qos->reliability == RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE) {
    qos->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if (qos->durability == RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE) {
    qos->durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
#endif
  if (qos->liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
    qos->liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN)
  {
    qos->liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  }
  // A default ({0,0}) deadline/lifespan/lease duration resolves to infinite,
  // which is the effective DDS default.
  const rmw_time_t infinite_duration = RMW_DURATION_INFINITE;
  if (qos->deadline.sec == 0 && qos->deadline.nsec == 0) {
    qos->deadline = infinite_duration;
  }
  if (qos->lifespan.sec == 0 && qos->lifespan.nsec == 0) {
    qos->lifespan = infinite_duration;
  }
  if (qos->liveliness_lease_duration.sec == 0 && qos->liveliness_lease_duration.nsec == 0) {
    qos->liveliness_lease_duration = infinite_duration;
  }
}

inline std::string
encode_message_type_hash_user_data(const rosidl_message_type_support_t * type_support)
{
  // ROS 2 Humble does not expose the newer rosidl type hash hooks used for
  // DDS user_data QoS matching. Leave user_data empty on this branch.
  (void)type_support;
  return {};
}

inline std::string
encode_service_request_type_hash_user_data(const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  return {};
}

inline std::string
encode_service_response_type_hash_user_data(const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  return {};
}

inline void
apply_type_hash_user_data(Int2DdsDataWriterQos * qos, const std::string & user_data)
{
  if (qos == nullptr || user_data.empty()) {
    return;
  }

  int2dds_datawriter_qos_set_user_data(
    qos,
    reinterpret_cast<const uint8_t *>(user_data.data()),
    user_data.size());
}

inline void
apply_type_hash_user_data(Int2DdsDataReaderQos * qos, const std::string & user_data)
{
  if (qos == nullptr || user_data.empty()) {
    return;
  }

  int2dds_datareader_qos_set_user_data(
    qos,
    reinterpret_cast<const uint8_t *>(user_data.data()),
    user_data.size());
}

}  // namespace rmw_int2dds_cpp

#endif  // COMMON__TYPE_HASH_QOS_HPP_
