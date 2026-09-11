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

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "rmw/rmw.h"
#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/validate_full_topic_name.h"

#include "rcutils/allocator.h"
#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "int2dds-ffi.h"  // NOLINT(build/include)
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "../wait/waitset_registry.hpp"  // NOLINT(build/include)
#include "../common/listeners.hpp"  // NOLINT(build/include)
#include "../graph/graph_guard.hpp"
#include "../graph/discovery.hpp"
#include "../common/type_hash_qos.hpp"

// Forward declarations from common utilities
namespace rmw_int2dds_cpp
{
rmw_gid_t generate_publisher_gid();
std::string ros_topic_to_dds_topic(const std::string & ros_topic);
std::string ros_topic_to_dds_topic(
  const char * ros_topic_name,
  bool avoid_ros_namespace_conventions);
std::string get_type_name(const rosidl_message_type_support_t * type_support);
std::string get_dds_type_name(const rosidl_message_type_support_t * type_support);
}

namespace
{

bool
is_unspecified_duration(const rmw_time_t & duration)
{
  const rmw_time_t unspecified = RMW_DURATION_UNSPECIFIED;
  return duration.sec == unspecified.sec && duration.nsec == unspecified.nsec;
}

bool
is_best_available_deadline(const rmw_time_t & duration)
{
#ifdef RMW_QOS_DEADLINE_BEST_AVAILABLE
  const rmw_time_t best_available = RMW_QOS_DEADLINE_BEST_AVAILABLE;
  return duration.sec == best_available.sec && duration.nsec == best_available.nsec;
#else
  (void)duration;
  return false;
#endif
}

bool
is_explicit_liveliness_policy(rmw_qos_liveliness_policy_t liveliness)
{
  if (liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
    liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN)
  {
    return false;
  }
#ifdef RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE
  if (liveliness == RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE) {
    return false;
  }
#endif
  return true;
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
int32_t
liveliness_to_int2dds(rmw_qos_liveliness_policy_t liveliness)
{
  switch (liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE:
      return INT2DDS_QOS_LIVELINESS_MANUAL_BY_PARTICIPANT;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      return INT2DDS_QOS_LIVELINESS_MANUAL_BY_TOPIC;
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
    default:
      return INT2DDS_QOS_LIVELINESS_AUTOMATIC;
  }
}
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

int64_t
duration_to_ns(const rmw_time_t & duration)
{
  return static_cast<int64_t>(duration.sec) * 1000000000LL + static_cast<int64_t>(duration.nsec);
}

bool
diag_rmw_qos_enabled()
{
  return std::getenv("INT2DDS_DIAG_RMW_QOS") != nullptr;
}

bool
has_unknown_qos_policy(const rmw_qos_profile_t & qos)
{
  return qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN ||
         qos.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
         qos.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN ||
         qos.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
}

rmw_qos_profile_t
resolve_system_default_qos(rmw_qos_profile_t qos)
{
  if (qos.history == RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT) {
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }
  if (qos.depth == 0 && qos.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    qos.depth = rmw_qos_profile_default.depth;
  }
  if (qos.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT) {
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if (qos.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT) {
    qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
  if (qos.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT) {
    qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  }
  return qos;
}

int32_t
ros_type_to_int2dds_field(uint8_t ros_type)
{
  switch (ros_type) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOL:
      return INT2DDS_FIELD_BOOL;
    case rosidl_typesupport_introspection_c__ROS_TYPE_BYTE:
      return INT2DDS_FIELD_BYTE;
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
      return INT2DDS_FIELD_CHAR8;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
      return INT2DDS_FIELD_INT8;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
      return INT2DDS_FIELD_INT16;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
      return INT2DDS_FIELD_INT32;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
      return INT2DDS_FIELD_INT64;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
      return INT2DDS_FIELD_UINT8;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
      return INT2DDS_FIELD_UINT16;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
      return INT2DDS_FIELD_UINT32;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
      return INT2DDS_FIELD_UINT64;
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      return INT2DDS_FIELD_FLOAT32;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      return INT2DDS_FIELD_FLOAT64;
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      return INT2DDS_FIELD_STRING;
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
      return INT2DDS_FIELD_WSTRING;
    default:
      return -1;
  }
}

Int2DdsRet
add_type_info_member(
  Int2DdsTypeInfo * type_info,
  const char * name,
  uint8_t ros_type,
  bool is_array,
  size_t array_size,
  bool is_upper_bound,
  bool is_key,
  const void * members)
{
  if (name == nullptr || type_info == nullptr || members != nullptr) {
    return INT2DDS_RET_UNSUPPORTED;
  }

  const int32_t field_type = ros_type_to_int2dds_field(ros_type);
  if (field_type < 0) {
    return INT2DDS_RET_UNSUPPORTED;
  }

  const int32_t flags = is_key ? INT2DDS_MEMBER_KEY : 0;
  if (!is_array) {
    return int2dds_type_info_add_field(type_info, name, field_type, flags);
  }

  if (is_upper_bound || array_size == 0) {
    const uint32_t bound = is_upper_bound ? static_cast<uint32_t>(array_size) : 0U;
    return int2dds_type_info_add_sequence_field(type_info, name, field_type, bound, flags);
  }

  return int2dds_type_info_add_array_field(
    type_info, name, field_type, static_cast<uint32_t>(array_size), flags);
}

bool
build_type_info_from_introspection(
  const rosidl_message_type_support_t * introspection_ts,
  const std::string & dds_type_name,
  Int2DdsTypeInfo ** type_info_out)
{
  if (introspection_ts == nullptr || introspection_ts->data == nullptr ||
    type_info_out == nullptr)
  {
    return false;
  }

  *type_info_out = nullptr;
  Int2DdsRet dds_ret = int2dds_type_info_create(
    dds_type_name.c_str(), rmw_int2dds_cpp::INT2DDS_EXTENSIBILITY_APPENDABLE, type_info_out);
  if (dds_ret != INT2DDS_RET_OK || *type_info_out == nullptr) {
    return false;
  }

  auto cleanup = [&]() {
      if (*type_info_out != nullptr) {
        int2dds_type_info_destroy(*type_info_out);
        *type_info_out = nullptr;
      }
    };

  if (std::strcmp(
      introspection_ts->typesupport_identifier,
      rosidl_typesupport_introspection_c__identifier) == 0)
  {
    auto * members =
      static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(introspection_ts->data);
    if (members == nullptr) {
      cleanup();
      return false;
    }
    for (uint32_t i = 0; i < members->member_count_; ++i) {
      const auto & member = members->members_[i];
      dds_ret = add_type_info_member(
        *type_info_out,
        member.name_,
        member.type_id_,
        member.is_array_,
        member.array_size_,
        member.is_upper_bound_,
        false,
        member.members_);
      if (dds_ret != INT2DDS_RET_OK) {
        cleanup();
        return false;
      }
    }
    return true;
  }

  if (std::strcmp(
      introspection_ts->typesupport_identifier,
      rosidl_typesupport_introspection_cpp::typesupport_identifier) == 0)
  {
    auto * members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_ts->
      data);
    if (members == nullptr) {
      cleanup();
      return false;
    }
    for (uint32_t i = 0; i < members->member_count_; ++i) {
      const auto & member = members->members_[i];
      dds_ret = add_type_info_member(
        *type_info_out,
        member.name_,
        member.type_id_,
        member.is_array_,
        member.array_size_,
        member.is_upper_bound_,
        false,
        member.members_);
      if (dds_ret != INT2DDS_RET_OK) {
        cleanup();
        return false;
      }
    }
    return true;
  }

  cleanup();
  return false;
}

[[maybe_unused]] Int2DdsRet
create_topic_with_introspection_type_info(
  const Int2DdsParticipant * participant,
  const std::string & dds_topic_name,
  const std::string & dds_type_name,
  const rosidl_message_type_support_t * introspection_ts,
  Int2DdsTopic ** topic_out)
{
  Int2DdsTypeInfo * type_info = nullptr;
  if (!build_type_info_from_introspection(introspection_ts, dds_type_name, &type_info)) {
    return INT2DDS_RET_UNSUPPORTED;
  }

  const Int2DdsRet dds_ret = int2dds_create_topic_with_type_info(
    participant, dds_topic_name.c_str(), type_info, nullptr, topic_out);
  int2dds_type_info_destroy(type_info);
  return dds_ret;
}

}  // namespace

extern "C"
{

rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_publisher_options_t * publisher_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return nullptr;
  }

  // Foxy's rmw_publisher_options_t has no require_unique_network_flow_endpoints
  // (Galactic+), so there is no network-flow requirement to reject here.

  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = 0;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (ret != RMW_RET_OK) {
      return nullptr;
    }
    if (validation_result != RMW_TOPIC_VALID) {
      RMW_SET_ERROR_MSG("invalid topic name");
      return nullptr;
    }
  }

  if (has_unknown_qos_policy(*qos_policies)) {
    RMW_SET_ERROR_MSG("unknown QoS policy is invalid for publisher");
    return nullptr;
  }

  const rmw_qos_profile_t actual_qos = resolve_system_default_qos(*qos_policies);

  auto * node_data = static_cast<rmw_int2dds_cpp::NodeData *>(node->data);
  if (node_data == nullptr || node_data->context_data == nullptr) {
    RMW_SET_ERROR_MSG("node data is null");
    return nullptr;
  }

  auto * context_data = node_data->context_data;
  if (context_data->is_shutdown) {
    RMW_SET_ERROR_MSG("context already shutdown");
    return nullptr;
  }

  // Get the introspection type support
  const rosidl_message_type_support_t * introspection_ts =
    get_message_typesupport_handle(
    type_support, rosidl_typesupport_introspection_c__identifier);
  if (introspection_ts == nullptr) {
    // The introspection_c lookup fails (and sets an rmw error) for C++ typesupport
    // messages. Clear it before the fallback so a recovered lookup leaves no stale
    // error that would later surface from unrelated error paths.
    rmw_reset_error();
    introspection_ts = get_message_typesupport_handle(
      type_support, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  }
  if (introspection_ts == nullptr) {
    RMW_SET_ERROR_MSG("failed to get introspection type support");
    return nullptr;
  }

  // Create publisher data
  auto * pub_data = new (std::nothrow) rmw_int2dds_cpp::PublisherData();
  if (pub_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate publisher data");
    return nullptr;
  }

  pub_data->type_support = introspection_ts;
  pub_data->qos = actual_qos;
  pub_data->gid = rmw_int2dds_cpp::generate_publisher_gid();
  pub_data->node_data = node_data;
  pub_data->topic_name = topic_name;
  pub_data->type_name = rmw_int2dds_cpp::get_type_name(introspection_ts);
  std::string dds_type_name = rmw_int2dds_cpp::get_dds_type_name(introspection_ts);

  if (diag_rmw_qos_enabled()) {
    RCUTILS_LOG_INFO_NAMED(
      "rmw_int2dds_cpp",
      "create_publisher topic=%s reliability=%d durability=%d history=%d depth=%zu "
      "deadline_ns=%lld lifespan_ns=%lld liveliness=%d lease_ns=%lld",
      topic_name,
      static_cast<int>(actual_qos.reliability),
      static_cast<int>(actual_qos.durability),
      static_cast<int>(actual_qos.history),
      actual_qos.depth,
      static_cast<long long>(duration_to_ns(actual_qos.deadline)),
      static_cast<long long>(duration_to_ns(actual_qos.lifespan)),
      static_cast<int>(actual_qos.liveliness),
      static_cast<long long>(duration_to_ns(actual_qos.liveliness_lease_duration)));
  }

  // Convert ROS topic name to DDS topic name
  std::string dds_topic_name = rmw_int2dds_cpp::ros_topic_to_dds_topic(
    topic_name, qos_policies->avoid_ros_namespace_conventions);

  Int2DdsRet dds_ret = int2dds_create_topic(
    context_data->participant,
    dds_topic_name.c_str(),
    dds_type_name.c_str(),
    rmw_int2dds_cpp::INT2DDS_EXTENSIBILITY_MUTABLE,
    nullptr,  // Use default topic QoS
    &pub_data->topic);
  if (dds_ret != INT2DDS_RET_OK) {
    delete pub_data;
    RMW_SET_ERROR_MSG("failed to create DDS topic");
    return nullptr;
  }

  // Create DataWriter QoS based on RMW QoS
  Int2DdsDataWriterQos * writer_qos = nullptr;
  int2dds_datawriter_qos_create_default(&writer_qos);
  rmw_int2dds_cpp::apply_type_hash_user_data(
    writer_qos, rmw_int2dds_cpp::encode_message_type_hash_user_data(introspection_ts));

  // Configure QoS from rmw_qos_profile
  // max_blocking_time_ns: 100ms default for reliable
  if (actual_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    int2dds_datawriter_qos_set_reliability(writer_qos, INT2DDS_QOS_RELIABILITY_RELIABLE, 100000000);
  } else {
    int2dds_datawriter_qos_set_reliability(writer_qos, INT2DDS_QOS_RELIABILITY_BEST_EFFORT, 0);
  }

  if (actual_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    int2dds_datawriter_qos_set_durability(writer_qos, INT2DDS_QOS_DURABILITY_TRANSIENT_LOCAL);
  } else {
    int2dds_datawriter_qos_set_durability(writer_qos, INT2DDS_QOS_DURABILITY_VOLATILE);
  }

  if (!is_unspecified_duration(actual_qos.deadline) &&
    !is_best_available_deadline(actual_qos.deadline))
  {
    int2dds_datawriter_qos_set_deadline(writer_qos, duration_to_ns(actual_qos.deadline));
  }

  if (!is_unspecified_duration(actual_qos.lifespan)) {
    int2dds_datawriter_qos_set_lifespan(writer_qos, duration_to_ns(actual_qos.lifespan));
  }

  if (is_explicit_liveliness_policy(actual_qos.liveliness) ||
    !is_unspecified_duration(actual_qos.liveliness_lease_duration))
  {
    int32_t liveliness_kind = INT2DDS_QOS_LIVELINESS_AUTOMATIC;
    int64_t lease_duration_ns = 0;
    Int2DdsRet liveliness_ret = int2dds_datawriter_qos_get_liveliness(
      writer_qos, &liveliness_kind, &lease_duration_ns);
    if (liveliness_ret != INT2DDS_RET_OK) {
      liveliness_kind = INT2DDS_QOS_LIVELINESS_AUTOMATIC;
      lease_duration_ns = 0;
    }

    if (is_explicit_liveliness_policy(actual_qos.liveliness)) {
      liveliness_kind = liveliness_to_int2dds(actual_qos.liveliness);
    }
    if (!is_unspecified_duration(actual_qos.liveliness_lease_duration)) {
      lease_duration_ns = duration_to_ns(actual_qos.liveliness_lease_duration);
    }

    int2dds_datawriter_qos_set_liveliness(writer_qos, liveliness_kind, lease_duration_ns);
  }

  // History
  if (actual_qos.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
    int2dds_datawriter_qos_set_history(writer_qos, INT2DDS_QOS_HISTORY_KEEP_ALL, 0);
  } else {
    size_t depth = actual_qos.depth;
    if (depth == 0) {
      depth = 1;  // Default depth
    }
    int2dds_datawriter_qos_set_history(writer_qos, INT2DDS_QOS_HISTORY_KEEP_LAST, depth);
  }

  // Create DataWriter
  dds_ret = int2dds_create_datawriter(
    context_data->default_publisher,
    pub_data->topic,
    writer_qos,
    nullptr,
    0,
    &pub_data->datawriter);
  int2dds_datawriter_qos_destroy(writer_qos);

  if (dds_ret != INT2DDS_RET_OK) {
    int2dds_delete_topic(pub_data->topic);
    delete pub_data;
    RMW_SET_ERROR_MSG("failed to create DataWriter");
    return nullptr;
  }

  {
    uint8_t endpoint_guid[16];
    if (int2dds_datawriter_get_guid(pub_data->datawriter, &endpoint_guid) == INT2DDS_RET_OK) {
      std::memcpy(pub_data->gid.data, endpoint_guid, sizeof(endpoint_guid));
    }
  }

  // Allocate RMW publisher
  rmw_publisher_t * publisher = rmw_publisher_allocate();
  if (publisher == nullptr) {
    int2dds_delete_datawriter(pub_data->datawriter);
    int2dds_delete_topic(pub_data->topic);
    delete pub_data;
    RMW_SET_ERROR_MSG("failed to allocate publisher");
    return nullptr;
  }

  publisher->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  publisher->data = pub_data;
  publisher->topic_name = rcutils_strdup(topic_name, node->context->options.allocator);
  publisher->options = *publisher_options;
  publisher->can_loan_messages = false;

  if (publisher->topic_name == nullptr) {
    rmw_publisher_free(publisher);
    int2dds_delete_datawriter(pub_data->datawriter);
    int2dds_delete_topic(pub_data->topic);
    delete pub_data;
    RMW_SET_ERROR_MSG("failed to allocate topic name");
    return nullptr;
  }

  // Track publisher in node
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    node_data->publishers.push_back(pub_data->gid);
  }

  // Listener starts with an empty mask; refreshed when user callbacks register
  rmw_int2dds_cpp::refresh_publisher_listener(pub_data);

  // Notify graph-change waiters that this publisher was added.
  rmw_int2dds_cpp::trigger_graph_guard_condition(context_data);

  if (context_data->common) {
    rmw_int2dds_cpp::common_add_local_entity(
      context_data, pub_data->gid, dds_topic_name, pub_data->type_name,
      pub_data->qos, /*is_reader=*/ false);
    rmw_int2dds_cpp::common_associate_local_writer(
      context_data, pub_data->gid, node_data->name, node_data->namespace_);
  }

  return publisher;
}

rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (publisher->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("publisher not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * node_data = static_cast<rmw_int2dds_cpp::NodeData *>(node->data);
  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(publisher->data);

  if (node_data == nullptr || pub_data == nullptr) {
    RMW_SET_ERROR_MSG("data is null");
    return RMW_RET_ERROR;
  }

  auto * context_data = node_data->context_data;

  // Remove from node's publisher list
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    auto & pubs = node_data->publishers;
    for (auto it = pubs.begin(); it != pubs.end(); ++it) {
      if (std::memcmp(it->data, pub_data->gid.data, RMW_GID_STORAGE_SIZE) == 0) {
        pubs.erase(it);
        break;
      }
    }
  }

  if (context_data != nullptr) {
    rmw_int2dds_cpp::trigger_graph_guard_condition(context_data);
  }

  if (context_data != nullptr && context_data->common) {
    rmw_int2dds_cpp::common_dissociate_local_writer(
      context_data, pub_data->gid, node_data->name, node_data->namespace_);
    rmw_int2dds_cpp::common_remove_local_entity(
      context_data, pub_data->gid, /*is_reader=*/ false);
  }

  // Delete DDS entities
  rmw_int2dds_cpp::waitset_registry_clean_caches();
  if (pub_data->status_condition != nullptr) {
    int2dds_statuscondition_delete(pub_data->status_condition);
    pub_data->status_condition = nullptr;
  }

  if (pub_data->datawriter != nullptr && context_data != nullptr) {
    int2dds_delete_datawriter(pub_data->datawriter);
  }

  if (pub_data->topic != nullptr && context_data != nullptr) {
    int2dds_delete_topic(pub_data->topic);
  }

  // Free topic name
  if (publisher->topic_name != nullptr && node->context != nullptr) {
    node->context->options.allocator.deallocate(
      const_cast<char *>(publisher->topic_name),
      node->context->options.allocator.state);
  }

  // Reclaim EventData that rcl never finalized (it never calls
  // rmw_event_fini), which otherwise leaks one object per event type on every
  // publisher create/destroy churn. The waitset caches were cleaned above.
  {
    std::lock_guard<std::mutex> guard(pub_data->event_mutex);
    for (auto * ev : pub_data->events) {
      rmw_int2dds_cpp::free_event_data(ev);
    }
    pub_data->events.clear();
  }

  delete pub_data;
  rmw_publisher_free(publisher);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("publisher not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(publisher->data);
  if (pub_data == nullptr || pub_data->datawriter == nullptr) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  Int2DdsPublicationMatchedStatus matched_status = {};
  Int2DdsRet ret = int2dds_datawriter_get_publication_matched_status(
    pub_data->datawriter, &matched_status);
  const int32_t current_count = matched_status.current_count;
  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to get publication matched status");
    return RMW_RET_ERROR;
  }

  *subscription_count = static_cast<size_t>(current_count);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("publisher not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(publisher->data);
  if (pub_data == nullptr) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  *qos = pub_data->qos;
  // Report concrete values for system-default/unknown policies (actual QoS), on a
  // copy only -- do not mutate the stored QoS, which other paths rely on.
  rmw_int2dds_cpp::resolve_actual_qos(qos);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("publisher not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(publisher->data);
  if (pub_data == nullptr || pub_data->datawriter == nullptr) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  Int2DdsRet dds_ret = int2dds_datawriter_assert_liveliness(pub_data->datawriter);
  if (dds_ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to assert DataWriter liveliness");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  (void)publisher;
  (void)type_support;
  (void)ros_message;
  RMW_SET_ERROR_MSG("rmw_borrow_loaned_message is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  (void)publisher;
  (void)loaned_message;
  RMW_SET_ERROR_MSG("rmw_return_loaned_message_from_publisher is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("publisher not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(publisher->data);
  if (pub_data == nullptr) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  gid->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  std::memcpy(gid->data, pub_data->gid.data, RMW_GID_STORAGE_SIZE);

  return RMW_RET_OK;
}

// rmw_publisher_wait_for_all_acked (Humble) and
// rmw_publisher_get_network_flow_endpoints (Galactic) are not part of the Foxy
// rmw API, and neither is an on-new-subscription callback hook.

}  // extern "C"
