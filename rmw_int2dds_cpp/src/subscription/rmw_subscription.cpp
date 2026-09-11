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
#include <memory>
#include <sstream>
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
#include "../common/type_hash_qos.hpp"
#include "../graph/discovery.hpp"
#include "../graph/graph_guard.hpp"

// Forward declarations from common utilities
namespace rmw_int2dds_cpp
{
rmw_gid_t generate_subscription_gid();
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

struct TopicFieldDescriptors
{
  std::vector<std::string> names;
  std::vector<const char *> name_ptrs;
  std::vector<uint32_t> types;
  std::vector<uint8_t> is_key;
  bool has_key{false};
};

// Maps a ROS introspection type id to the int2dds field-type enum
// (INT2DDS_FIELD_*). Defined further below; forward-declared here so the
// content-filter field-descriptor builder can reuse the single source of truth
// instead of its own ad-hoc table.
int32_t ros_type_to_int2dds_field(uint8_t ros_type);

bool
append_field_descriptor(
  const char * name,
  uint8_t ros_type,
  bool is_array,
  bool is_upper_bound,
  bool is_key,
  TopicFieldDescriptors & descriptors)
{
  if (name == nullptr || is_array || is_upper_bound || ros_type == 0) {
    return false;
  }

  // int2dds_create_topic_with_field_descriptors takes its own scalar-only field
  // codes, NOT the INT2DDS_FIELD_* constants used everywhere else. The core says
  // so itself in ffi/src/topic.rs (field_descriptor_type): "a distinct encoding
  // from the INT2DDS_FIELD_* constants", mapping 0..9 to String, Int32, UInt32,
  // Int16, UInt16, Int64, UInt64, Int8, UInt8, Bool and rejecting everything
  // else. Feeding it INT2DDS_FIELD_* registers every field under the wrong type
  // and content filtering silently stops matching, so keep this table in step
  // with that function rather than with ros_type_to_int2dds_field().
  //
  // Types outside the table (float, double, char, wchar, nested, sequences) have
  // no code, so return false and let the caller fall back to an unfiltered
  // subscription instead of registering a wrong one.
  uint32_t field_type = 0;
  switch (ros_type) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      field_type = 0;
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
      field_type = 1;
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
      field_type = 2;
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
      field_type = 3;
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
      field_type = 4;
      break;
    // int64/uint64 have codes (5 and 6) but filtering on them does not work:
    // the core aligns to the absolute buffer offset instead of the offset within
    // the payload, so an 8-byte field is read 4 bytes past its real position
    // (ffi/src/data.rs, cdr_parse_field_value). Narrower fields are unaffected
    // because offset 4 already satisfies their alignment. Registering them
    // anyway would set is_cft_enabled and then deliver everything, which is
    // worse than reporting no filter: callers can filter themselves once they
    // know the middleware will not. Restore these two once the core is fixed.
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
      return false;
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
      field_type = 7;
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
      field_type = 8;
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOL:
      field_type = 9;
      break;
    default:
      return false;
  }
  descriptors.names.emplace_back(name);
  descriptors.types.push_back(field_type);
  descriptors.is_key.push_back(is_key ? 1U : 0U);
  descriptors.has_key = descriptors.has_key || is_key;
  return true;
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

bool
populate_field_descriptors(
  const rosidl_message_type_support_t * introspection_ts,
  TopicFieldDescriptors & descriptors)
{
  if (introspection_ts == nullptr || introspection_ts->data == nullptr) {
    return false;
  }

  if (std::strcmp(
      introspection_ts->typesupport_identifier,
      rosidl_typesupport_introspection_c__identifier) == 0)
  {
    auto * members =
      static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(introspection_ts->data);
    if (members == nullptr) {
      return false;
    }
    for (uint32_t i = 0; i < members->member_count_; ++i) {
      const auto & member = members->members_[i];
      if (!append_field_descriptor(
          member.name_,
          member.type_id_,
          member.is_array_,
          member.is_upper_bound_,
          false,
          descriptors))
      {
        return false;
      }
    }
  } else if (std::strcmp(
      introspection_ts->typesupport_identifier,
      rosidl_typesupport_introspection_cpp::typesupport_identifier) == 0)
  {
    auto * members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_ts->
      data);
    if (members == nullptr) {
      return false;
    }
    for (uint32_t i = 0; i < members->member_count_; ++i) {
      const auto & member = members->members_[i];
      if (!append_field_descriptor(
          member.name_,
          member.type_id_,
          member.is_array_,
          member.is_upper_bound_,
          false,
          descriptors))
      {
        return false;
      }
    }
  } else {
    return false;
  }

  descriptors.name_ptrs.reserve(descriptors.names.size());
  for (const auto & name : descriptors.names) {
    descriptors.name_ptrs.push_back(name.c_str());
  }
  return true;
}

std::string
gid_to_hex(const rmw_gid_t & gid)
{
  std::ostringstream stream;
  stream << std::hex;
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    stream.width(2);
    stream.fill('0');
    stream << static_cast<unsigned int>(gid.data[i]);
  }
  return stream.str();
}

std::string
make_content_filtered_topic_name(const rmw_int2dds_cpp::SubscriptionData * sub_data)
{
  return rmw_int2dds_cpp::ros_topic_to_dds_topic(sub_data->topic_name) +
         "__cft_" + gid_to_hex(sub_data->gid);
}

Int2DdsDataReaderQos *
create_reader_qos(const rmw_qos_profile_t & qos_policies, const std::string & type_hash_user_data)
{
  Int2DdsDataReaderQos * reader_qos = nullptr;
  int2dds_datareader_qos_create_default(&reader_qos);

  if (reader_qos == nullptr) {
    return nullptr;
  }

  rmw_int2dds_cpp::apply_type_hash_user_data(reader_qos, type_hash_user_data);

  if (qos_policies.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    int2dds_datareader_qos_set_reliability(
      reader_qos, INT2DDS_QOS_RELIABILITY_RELIABLE, 100000000);
  } else {
    int2dds_datareader_qos_set_reliability(
      reader_qos, INT2DDS_QOS_RELIABILITY_BEST_EFFORT, 0);
  }

  if (qos_policies.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    int2dds_datareader_qos_set_durability(
      reader_qos, INT2DDS_QOS_DURABILITY_TRANSIENT_LOCAL);
  } else {
    int2dds_datareader_qos_set_durability(
      reader_qos, INT2DDS_QOS_DURABILITY_VOLATILE);
  }

  if (!is_unspecified_duration(qos_policies.deadline) &&
    !is_best_available_deadline(qos_policies.deadline))
  {
    int2dds_datareader_qos_set_deadline(reader_qos, duration_to_ns(qos_policies.deadline));
  }

  if (is_explicit_liveliness_policy(qos_policies.liveliness) ||
    !is_unspecified_duration(qos_policies.liveliness_lease_duration))
  {
    int32_t liveliness_kind = INT2DDS_QOS_LIVELINESS_AUTOMATIC;
    int64_t lease_duration_ns = 0;
    Int2DdsRet liveliness_ret = int2dds_datareader_qos_get_liveliness(
      reader_qos, &liveliness_kind, &lease_duration_ns);
    if (liveliness_ret != INT2DDS_RET_OK) {
      liveliness_kind = INT2DDS_QOS_LIVELINESS_AUTOMATIC;
      lease_duration_ns = 0;
    }

    if (is_explicit_liveliness_policy(qos_policies.liveliness)) {
      liveliness_kind = liveliness_to_int2dds(qos_policies.liveliness);
    }
    if (!is_unspecified_duration(qos_policies.liveliness_lease_duration)) {
      lease_duration_ns = duration_to_ns(qos_policies.liveliness_lease_duration);
    }

    int2dds_datareader_qos_set_liveliness(reader_qos, liveliness_kind, lease_duration_ns);
  }

  if (qos_policies.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
    int2dds_datareader_qos_set_history(reader_qos, INT2DDS_QOS_HISTORY_KEEP_ALL, 0);
  } else {
    size_t depth = qos_policies.depth;
    if (depth == 0) {
      depth = 1;
    }
    int2dds_datareader_qos_set_history(reader_qos, INT2DDS_QOS_HISTORY_KEEP_LAST, depth);
  }

  return reader_qos;
}

void
destroy_subscription_reader_entities(
  rmw_int2dds_cpp::ContextData * context_data,
  rmw_int2dds_cpp::SubscriptionData * sub_data)
{
  if (sub_data == nullptr || context_data == nullptr) {
    return;
  }

  rmw_int2dds_cpp::waitset_registry_clean_caches();
  if (sub_data->status_condition != nullptr) {
    int2dds_statuscondition_delete(sub_data->status_condition);
    sub_data->status_condition = nullptr;
  }

  if (sub_data->datareader != nullptr) {
    int2dds_delete_datareader(sub_data->datareader);
    sub_data->datareader = nullptr;
  }

  if (sub_data->content_filtered_topic != nullptr) {
    int2dds_delete_contentfilteredtopic(sub_data->content_filtered_topic);
    sub_data->content_filtered_topic = nullptr;
  }
}

Int2DdsRet
create_subscription_reader(
  rmw_int2dds_cpp::ContextData * context_data,
  rmw_int2dds_cpp::SubscriptionData * sub_data)
{
  if (context_data == nullptr || sub_data == nullptr) {
    return INT2DDS_RET_INVALID_ARGUMENT;
  }

  Int2DdsDataReaderQos * reader_qos = create_reader_qos(
    sub_data->qos,
    rmw_int2dds_cpp::encode_message_type_hash_user_data(sub_data->type_support));
  if (reader_qos == nullptr) {
    return INT2DDS_RET_ERROR;
  }

  Int2DdsRet dds_ret = INT2DDS_RET_ERROR;
  if (sub_data->content_filter_expression.empty()) {
    dds_ret = int2dds_create_datareader(
      context_data->default_subscriber,
      sub_data->topic,
      reader_qos,
      nullptr,
      0,
      &sub_data->datareader);
  } else {
    std::vector<const char *> parameter_ptrs;
    parameter_ptrs.reserve(sub_data->content_filter_parameters.size());
    for (const auto & parameter : sub_data->content_filter_parameters) {
      parameter_ptrs.push_back(parameter.c_str());
    }

    std::string cft_name = make_content_filtered_topic_name(sub_data);
    dds_ret = int2dds_create_contentfilteredtopic(
      context_data->participant,
      cft_name.c_str(),
      sub_data->topic,
      sub_data->content_filter_expression.c_str(),
      parameter_ptrs.empty() ? nullptr : parameter_ptrs.data(),
      parameter_ptrs.size(),
      &sub_data->content_filtered_topic);
    if (dds_ret == INT2DDS_RET_OK) {
      dds_ret = int2dds_create_datareader_cft(
        context_data->default_subscriber,
        sub_data->content_filtered_topic,
        reader_qos,
        nullptr,
        0,
        &sub_data->datareader);
      if (dds_ret != INT2DDS_RET_OK && sub_data->content_filtered_topic != nullptr) {
        int2dds_delete_contentfilteredtopic(sub_data->content_filtered_topic);
        sub_data->content_filtered_topic = nullptr;
      }
    }
  }

  int2dds_datareader_qos_destroy(reader_qos);
  return dds_ret;
}

}  // namespace

extern "C"
{

rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_subscription_options_t * subscription_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return nullptr;
  }

  // Foxy's rmw_subscription_options_t has no require_unique_network_flow_endpoints
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
    RMW_SET_ERROR_MSG("unknown QoS policy is invalid for subscription");
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

  // Create subscription data
  auto * sub_data = new (std::nothrow) rmw_int2dds_cpp::SubscriptionData();
  if (sub_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate subscription data");
    return nullptr;
  }

  sub_data->type_support = introspection_ts;
  sub_data->qos = actual_qos;
  sub_data->gid = rmw_int2dds_cpp::generate_subscription_gid();
  sub_data->node_data = node_data;
  sub_data->ignore_local_publications = subscription_options->ignore_local_publications;
  sub_data->topic_name = topic_name;
  sub_data->type_name = rmw_int2dds_cpp::get_type_name(introspection_ts);
  std::string dds_type_name = rmw_int2dds_cpp::get_dds_type_name(introspection_ts);

  if (diag_rmw_qos_enabled()) {
    RCUTILS_LOG_INFO_NAMED(
      "rmw_int2dds_cpp",
      "create_subscription topic=%s reliability=%d durability=%d history=%d depth=%zu "
      "deadline_ns=%lld liveliness=%d lease_ns=%lld",
      topic_name,
      static_cast<int>(actual_qos.reliability),
      static_cast<int>(actual_qos.durability),
      static_cast<int>(actual_qos.history),
      actual_qos.depth,
      static_cast<long long>(duration_to_ns(actual_qos.deadline)),
      static_cast<int>(actual_qos.liveliness),
      static_cast<long long>(duration_to_ns(actual_qos.liveliness_lease_duration)));
  }

  // Convert ROS topic name to DDS topic name
  std::string dds_topic_name = rmw_int2dds_cpp::ros_topic_to_dds_topic(
    topic_name, qos_policies->avoid_ros_namespace_conventions);

  // Content filtered topics arrived in Humble: Foxy's subscription options carry
  // no content_filter_options, so content_filter_expression stays empty and the
  // content-filtered reader path below is never taken on this branch.

  // Create DDS Topic
  Int2DdsRet dds_ret = INT2DDS_RET_ERROR;
  bool create_cft_topic = !sub_data->content_filter_expression.empty();
  if (create_cft_topic) {
    TopicFieldDescriptors descriptors;
    if (!populate_field_descriptors(introspection_ts, descriptors)) {
      sub_data->content_filter_expression.clear();
      sub_data->content_filter_parameters.clear();
      create_cft_topic = false;
    }
  }

  if (create_cft_topic) {
    TopicFieldDescriptors descriptors;
    if (!populate_field_descriptors(introspection_ts, descriptors)) {
      sub_data->content_filter_expression.clear();
      sub_data->content_filter_parameters.clear();
      create_cft_topic = false;
    } else {
      std::unique_ptr<bool[]> field_is_key(new bool[descriptors.is_key.size()]);
      for (size_t i = 0; i < descriptors.is_key.size(); ++i) {
        field_is_key[i] = descriptors.is_key[i] != 0U;
      }

      dds_ret = int2dds_create_topic_with_field_descriptors(
        context_data->participant,
        dds_topic_name.c_str(),
        dds_type_name.c_str(),
        rmw_int2dds_cpp::INT2DDS_EXTENSIBILITY_MUTABLE,
        nullptr,
        descriptors.name_ptrs.data(),
        descriptors.types.data(),
        field_is_key.get(),
        descriptors.names.size(),
        &sub_data->topic);
      if (dds_ret != INT2DDS_RET_OK) {
        sub_data->content_filter_expression.clear();
        sub_data->content_filter_parameters.clear();
        sub_data->topic = nullptr;
        create_cft_topic = false;
      }
    }
  }

  if (!create_cft_topic) {
    dds_ret = int2dds_create_topic(
      context_data->participant,
      dds_topic_name.c_str(),
      dds_type_name.c_str(),
      rmw_int2dds_cpp::INT2DDS_EXTENSIBILITY_MUTABLE,
      nullptr,  // Use default topic QoS
      &sub_data->topic);
  }

  if (dds_ret != INT2DDS_RET_OK) {
    delete sub_data;
    RMW_SET_ERROR_MSG("failed to create DDS topic");
    return nullptr;
  }

  // Create DataReader
  dds_ret = create_subscription_reader(context_data, sub_data);
  if (dds_ret != INT2DDS_RET_OK) {
    int2dds_delete_topic(sub_data->topic);
    delete sub_data;
    RMW_SET_ERROR_MSG("failed to create DataReader");
    return nullptr;
  }

  {
    uint8_t endpoint_guid[16];
    if (int2dds_datareader_get_guid(sub_data->datareader, &endpoint_guid) == INT2DDS_RET_OK) {
      std::memcpy(sub_data->gid.data, endpoint_guid, sizeof(endpoint_guid));
    }
  }

  // Allocate RMW subscription
  rmw_subscription_t * subscription = rmw_subscription_allocate();
  if (subscription == nullptr) {
    int2dds_delete_datareader(sub_data->datareader);
    int2dds_delete_topic(sub_data->topic);
    delete sub_data;
    RMW_SET_ERROR_MSG("failed to allocate subscription");
    return nullptr;
  }

  subscription->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  subscription->data = sub_data;
  subscription->topic_name = rcutils_strdup(topic_name, node->context->options.allocator);
  subscription->options = *subscription_options;
  subscription->can_loan_messages = false;

  if (subscription->topic_name == nullptr) {
    rmw_subscription_free(subscription);
    int2dds_delete_datareader(sub_data->datareader);
    int2dds_delete_topic(sub_data->topic);
    delete sub_data;
    RMW_SET_ERROR_MSG("failed to allocate topic name");
    return nullptr;
  }

  // Track subscription in node
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    node_data->subscriptions.push_back(sub_data->gid);
  }

  // Listener starts with an empty mask; refreshed when user callbacks register
  rmw_int2dds_cpp::refresh_subscription_listener(sub_data);
  rmw_int2dds_cpp::trigger_graph_guard_condition(context_data);

  if (context_data->common) {
    rmw_int2dds_cpp::common_add_local_entity(
      context_data, sub_data->gid, dds_topic_name, sub_data->type_name,
      sub_data->qos, /*is_reader=*/ true);
    rmw_int2dds_cpp::common_associate_local_reader(
      context_data, sub_data->gid, node_data->name, node_data->namespace_);
  }

  return subscription;
}

rmw_ret_t
rmw_destroy_subscription(rmw_node_t * node, rmw_subscription_t * subscription)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * node_data = static_cast<rmw_int2dds_cpp::NodeData *>(node->data);
  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscription->data);

  if (node_data == nullptr || sub_data == nullptr) {
    RMW_SET_ERROR_MSG("data is null");
    return RMW_RET_ERROR;
  }

  auto * context_data = node_data->context_data;

  // Remove from node's subscription list
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    auto & subs = node_data->subscriptions;
    for (auto it = subs.begin(); it != subs.end(); ++it) {
      if (std::memcmp(it->data, sub_data->gid.data, RMW_GID_STORAGE_SIZE) == 0) {
        subs.erase(it);
        break;
      }
    }
  }

  if (context_data != nullptr) {
    rmw_int2dds_cpp::trigger_graph_guard_condition(context_data);
  }

  if (context_data != nullptr && context_data->common) {
    rmw_int2dds_cpp::common_dissociate_local_reader(
      context_data, sub_data->gid, node_data->name, node_data->namespace_);
    rmw_int2dds_cpp::common_remove_local_entity(
      context_data, sub_data->gid, /*is_reader=*/ true);
  }

  // Delete DDS entities
  destroy_subscription_reader_entities(context_data, sub_data);

  if (sub_data->topic != nullptr && context_data != nullptr) {
    int2dds_delete_topic(sub_data->topic);
  }

  // Free topic name
  if (subscription->topic_name != nullptr && node->context != nullptr) {
    node->context->options.allocator.deallocate(
      const_cast<char *>(subscription->topic_name),
      node->context->options.allocator.state);
  }

  // Reclaim EventData that rcl never finalized (it never calls
  // rmw_event_fini), which otherwise leaks one object per event type on every
  // subscription create/destroy churn. The waitset caches were cleaned above.
  {
    std::lock_guard<std::mutex> guard(sub_data->event_mutex);
    for (auto * ev : sub_data->events) {
      rmw_int2dds_cpp::free_event_data(ev);
    }
    sub_data->events.clear();
  }

  delete sub_data;
  rmw_subscription_free(subscription);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_count, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscription->data);
  if (sub_data == nullptr || sub_data->datareader == nullptr) {
    RMW_SET_ERROR_MSG("subscription data is null");
    return RMW_RET_ERROR;
  }

  Int2DdsSubscriptionMatchedStatus matched_status = {};
  Int2DdsRet ret = int2dds_datareader_get_subscription_matched_status(
    sub_data->datareader, &matched_status);
  const int32_t current_count = matched_status.current_count;
  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to get subscription matched status");
    return RMW_RET_ERROR;
  }

  *publisher_count = static_cast<size_t>(current_count);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscription->data);
  if (sub_data == nullptr) {
    RMW_SET_ERROR_MSG("subscription data is null");
    return RMW_RET_ERROR;
  }

  *qos = sub_data->qos;
  // Report concrete values for system-default/unknown policies on a copy only.
  rmw_int2dds_cpp::resolve_actual_qos(qos);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_init_subscription_allocation is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_fini_subscription_allocation(rmw_subscription_allocation_t * allocation)
{
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_fini_subscription_allocation is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

// Not part of the Foxy rmw API: rmw_subscription_{set,get}_content_filter and
// rmw_subscription_set_on_new_message_callback (Humble),
// rmw_subscription_get_network_flow_endpoints (Galactic).

}  // extern "C"
