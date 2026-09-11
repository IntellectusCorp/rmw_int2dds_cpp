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
#include <cstring>
#include <string>

#include "rmw/rmw.h"
#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/validate_full_topic_name.h"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "rosidl_typesupport_c/service_type_support_dispatch.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

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
rmw_gid_t generate_service_gid();
std::string ros_topic_to_dds_topic(const std::string & ros_topic);
std::string get_service_request_type_name(const rosidl_service_type_support_t * type_support);
std::string get_service_response_type_name(const rosidl_service_type_support_t * type_support);
}

namespace
{

bool
has_unknown_qos_policy(const rmw_qos_profile_t & qos)
{
  return qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN ||
         qos.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
         qos.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN ||
         qos.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
}

// Duration/liveliness helpers mirrored from rmw_create_publisher/subscription so
// services/clients apply deadline, lifespan and liveliness to the DDS entities
// instead of dropping them (they used to set only reliability/durability/history).
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
#ifdef RMW_QOS_DEADLINE_BEST_AVAILABLE
  if (liveliness == RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE) {
    return false;
  }
#endif
  return true;
}

int32_t
liveliness_to_int2dds(rmw_qos_liveliness_policy_t liveliness)
{
  switch (liveliness) {
// ROS 2 Lyrical removed the deprecated MANUAL_BY_NODE policy; detect via a header
// introduced in the same release (enum values are invisible to __has_include).
#if !__has_include("rmw/get_service_endpoint_info.h")
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE:
#pragma GCC diagnostic pop
      return INT2DDS_QOS_LIVELINESS_MANUAL_BY_PARTICIPANT;
#endif
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      return INT2DDS_QOS_LIVELINESS_MANUAL_BY_TOPIC;
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
    default:
      return INT2DDS_QOS_LIVELINESS_AUTOMATIC;
  }
}

int64_t
duration_to_ns(const rmw_time_t & duration)
{
  return static_cast<int64_t>(duration.sec) * 1000000000LL + static_cast<int64_t>(duration.nsec);
}

void
set_writer_deadline_lifespan_liveliness(
  Int2DdsDataWriterQos * writer_qos, const rmw_qos_profile_t & qos)
{
  if (!is_unspecified_duration(qos.deadline) && !is_best_available_deadline(qos.deadline)) {
    int2dds_datawriter_qos_set_deadline(writer_qos, duration_to_ns(qos.deadline));
  }
  if (!is_unspecified_duration(qos.lifespan)) {
    int2dds_datawriter_qos_set_lifespan(writer_qos, duration_to_ns(qos.lifespan));
  }
  if (is_explicit_liveliness_policy(qos.liveliness) ||
    !is_unspecified_duration(qos.liveliness_lease_duration))
  {
    int32_t liveliness_kind = INT2DDS_QOS_LIVELINESS_AUTOMATIC;
    int64_t lease_duration_ns = 0;
    if (int2dds_datawriter_qos_get_liveliness(
        writer_qos, &liveliness_kind, &lease_duration_ns) != INT2DDS_RET_OK)
    {
      liveliness_kind = INT2DDS_QOS_LIVELINESS_AUTOMATIC;
      lease_duration_ns = 0;
    }
    if (is_explicit_liveliness_policy(qos.liveliness)) {
      liveliness_kind = liveliness_to_int2dds(qos.liveliness);
    }
    if (!is_unspecified_duration(qos.liveliness_lease_duration)) {
      lease_duration_ns = duration_to_ns(qos.liveliness_lease_duration);
    }
    int2dds_datawriter_qos_set_liveliness(writer_qos, liveliness_kind, lease_duration_ns);
  }
}

void
set_reader_deadline_liveliness(
  Int2DdsDataReaderQos * reader_qos, const rmw_qos_profile_t & qos)
{
  if (!is_unspecified_duration(qos.deadline) && !is_best_available_deadline(qos.deadline)) {
    int2dds_datareader_qos_set_deadline(reader_qos, duration_to_ns(qos.deadline));
  }
  if (is_explicit_liveliness_policy(qos.liveliness) ||
    !is_unspecified_duration(qos.liveliness_lease_duration))
  {
    int32_t liveliness_kind = INT2DDS_QOS_LIVELINESS_AUTOMATIC;
    int64_t lease_duration_ns = 0;
    if (int2dds_datareader_qos_get_liveliness(
        reader_qos, &liveliness_kind, &lease_duration_ns) != INT2DDS_RET_OK)
    {
      liveliness_kind = INT2DDS_QOS_LIVELINESS_AUTOMATIC;
      lease_duration_ns = 0;
    }
    if (is_explicit_liveliness_policy(qos.liveliness)) {
      liveliness_kind = liveliness_to_int2dds(qos.liveliness);
    }
    if (!is_unspecified_duration(qos.liveliness_lease_duration)) {
      lease_duration_ns = duration_to_ns(qos.liveliness_lease_duration);
    }
    int2dds_datareader_qos_set_liveliness(reader_qos, liveliness_kind, lease_duration_ns);
  }
}

void
set_writer_history(Int2DdsDataWriterQos * writer_qos, const rmw_qos_profile_t & qos)
{
  if (qos.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
    int2dds_datawriter_qos_set_history(writer_qos, INT2DDS_QOS_HISTORY_KEEP_ALL, 0);
  } else {
    size_t depth = qos.depth;
    if (depth == 0) {
      depth = 1;
    }
    int2dds_datawriter_qos_set_history(writer_qos, INT2DDS_QOS_HISTORY_KEEP_LAST, depth);
  }
}

void
set_reader_history(Int2DdsDataReaderQos * reader_qos, const rmw_qos_profile_t & qos)
{
  if (qos.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
    int2dds_datareader_qos_set_history(reader_qos, INT2DDS_QOS_HISTORY_KEEP_ALL, 0);
  } else {
    size_t depth = qos.depth;
    if (depth == 0) {
      depth = 1;
    }
    int2dds_datareader_qos_set_history(reader_qos, INT2DDS_QOS_HISTORY_KEEP_LAST, depth);
  }
}

// Reliability and durability used to be pinned to RELIABLE/VOLATILE here, which
// dropped whatever the caller asked for. Two endpoints therefore always matched,
// even when their profiles were incompatible under the DDS request/offer rules --
// rmw_fastrtps_cpp and rmw_cyclonedds_cpp both refuse the match in that case, so a
// service that only worked on this RMW would break on any other one. Map the
// requested policies the same way rmw_create_publisher and rmw_create_subscription
// already do; system-default values are resolved beforehand by resolve_actual_qos.
void
set_writer_reliability_durability(
  Int2DdsDataWriterQos * writer_qos, const rmw_qos_profile_t & qos)
{
  if (qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
    int2dds_datawriter_qos_set_reliability(writer_qos, INT2DDS_QOS_RELIABILITY_BEST_EFFORT, 0);
  } else {
    int2dds_datawriter_qos_set_reliability(
      writer_qos, INT2DDS_QOS_RELIABILITY_RELIABLE, 1000000000);
  }

  if (qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    int2dds_datawriter_qos_set_durability(writer_qos, INT2DDS_QOS_DURABILITY_TRANSIENT_LOCAL);
  } else {
    int2dds_datawriter_qos_set_durability(writer_qos, INT2DDS_QOS_DURABILITY_VOLATILE);
  }
}

void
set_reader_reliability_durability(
  Int2DdsDataReaderQos * reader_qos, const rmw_qos_profile_t & qos)
{
  if (qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
    int2dds_datareader_qos_set_reliability(reader_qos, INT2DDS_QOS_RELIABILITY_BEST_EFFORT, 0);
  } else {
    int2dds_datareader_qos_set_reliability(
      reader_qos, INT2DDS_QOS_RELIABILITY_RELIABLE, 1000000000);
  }

  if (qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    int2dds_datareader_qos_set_durability(reader_qos, INT2DDS_QOS_DURABILITY_TRANSIENT_LOCAL);
  } else {
    int2dds_datareader_qos_set_durability(reader_qos, INT2DDS_QOS_DURABILITY_VOLATILE);
  }
}

}  // namespace

extern "C"
{

rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return nullptr;
  }

  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = 0;
    rmw_ret_t ret = rmw_validate_full_topic_name(service_name, &validation_result, nullptr);
    if (ret != RMW_RET_OK) {
      return nullptr;
    }
    if (validation_result != RMW_TOPIC_VALID) {
      RMW_SET_ERROR_MSG("invalid service name");
      return nullptr;
    }
  }

  if (has_unknown_qos_policy(*qos_policies)) {
    RMW_SET_ERROR_MSG("unknown QoS policy is invalid for service");
    return nullptr;
  }

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

  // Create service data
  auto * srv_data = new (std::nothrow) rmw_int2dds_cpp::ServiceData();
  if (srv_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate service data");
    return nullptr;
  }

  const rosidl_service_type_support_t * introspection_ts = get_service_typesupport_handle(
    type_support, rosidl_typesupport_introspection_c__identifier);
  if (introspection_ts == nullptr) {
    // The introspection_c lookup fails (and sets an rmw error) for C++ typesupport
    // services. Clear it before the fallback so a recovered lookup leaves no stale
    // error that would later surface from unrelated error paths.
    rmw_reset_error();
    introspection_ts = get_service_typesupport_handle(
      type_support, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  }
  if (introspection_ts == nullptr) {
    delete srv_data;
    RMW_SET_ERROR_MSG("failed to get introspection service type support");
    return nullptr;
  }

  srv_data->type_support = introspection_ts;
  srv_data->qos = *qos_policies;
  // Resolve system-default values up front so the profile stored here is the one
  // actually applied below, which is what rmw_service_*_get_actual_qos reports.
  rmw_int2dds_cpp::resolve_actual_qos(&srv_data->qos);
  srv_data->gid = rmw_int2dds_cpp::generate_service_gid();
  srv_data->node_data = node_data;
  srv_data->service_name = service_name;

  // Create topic names for request and response
  std::string request_topic_name = "rq" + std::string(service_name) + "Request";
  std::string response_topic_name = "rr" + std::string(service_name) + "Reply";

  std::string request_type_name = rmw_int2dds_cpp::get_service_request_type_name(type_support);
  std::string response_type_name = rmw_int2dds_cpp::get_service_response_type_name(type_support);

  // Create request topic
  Int2DdsRet dds_ret = int2dds_create_topic(
    context_data->participant,
    request_topic_name.c_str(),
    request_type_name.c_str(),
    rmw_int2dds_cpp::INT2DDS_EXTENSIBILITY_MUTABLE,
    nullptr,
    &srv_data->request_topic);
  if (dds_ret != INT2DDS_RET_OK) {
    delete srv_data;
    RMW_SET_ERROR_MSG("failed to create request topic");
    return nullptr;
  }

  // Create response topic
  dds_ret = int2dds_create_topic(
    context_data->participant,
    response_topic_name.c_str(),
    response_type_name.c_str(),
    rmw_int2dds_cpp::INT2DDS_EXTENSIBILITY_MUTABLE,
    nullptr,
    &srv_data->response_topic);
  if (dds_ret != INT2DDS_RET_OK) {
    int2dds_delete_topic(srv_data->request_topic);
    delete srv_data;
    RMW_SET_ERROR_MSG("failed to create response topic");
    return nullptr;
  }

  // Create DataReader QoS for requests
  Int2DdsDataReaderQos * reader_qos = nullptr;
  int2dds_datareader_qos_create_default(&reader_qos);
  rmw_int2dds_cpp::apply_type_hash_user_data(
    reader_qos, rmw_int2dds_cpp::encode_service_request_type_hash_user_data(introspection_ts));
  set_reader_reliability_durability(reader_qos, srv_data->qos);
  set_reader_deadline_liveliness(reader_qos, srv_data->qos);
  set_reader_history(reader_qos, srv_data->qos);

  // Create request DataReader
  dds_ret = int2dds_create_datareader(
    context_data->default_subscriber,
    srv_data->request_topic,
    reader_qos,
    nullptr,
    0,
    &srv_data->request_reader);
  int2dds_datareader_qos_destroy(reader_qos);

  if (dds_ret != INT2DDS_RET_OK) {
    int2dds_delete_topic(srv_data->response_topic);
    int2dds_delete_topic(srv_data->request_topic);
    delete srv_data;
    RMW_SET_ERROR_MSG("failed to create request DataReader");
    return nullptr;
  }

  // Create DataWriter QoS for responses
  Int2DdsDataWriterQos * writer_qos = nullptr;
  int2dds_datawriter_qos_create_default(&writer_qos);
  rmw_int2dds_cpp::apply_type_hash_user_data(
    writer_qos, rmw_int2dds_cpp::encode_service_response_type_hash_user_data(introspection_ts));
  set_writer_reliability_durability(writer_qos, srv_data->qos);
  set_writer_deadline_lifespan_liveliness(writer_qos, srv_data->qos);
  set_writer_history(writer_qos, srv_data->qos);

  // Create response DataWriter
  dds_ret = int2dds_create_datawriter(
    context_data->default_publisher,
    srv_data->response_topic,
    writer_qos,
    nullptr,
    0,
    &srv_data->response_writer);
  int2dds_datawriter_qos_destroy(writer_qos);

  if (dds_ret != INT2DDS_RET_OK) {
    int2dds_delete_datareader(srv_data->request_reader);
    int2dds_delete_topic(srv_data->response_topic);
    int2dds_delete_topic(srv_data->request_topic);
    delete srv_data;
    RMW_SET_ERROR_MSG("failed to create response DataWriter");
    return nullptr;
  }

  // Allocate RMW service
  rmw_service_t * service = rmw_service_allocate();
  if (service == nullptr) {
    int2dds_delete_datawriter(srv_data->response_writer);
    int2dds_delete_datareader(srv_data->request_reader);
    int2dds_delete_topic(srv_data->response_topic);
    int2dds_delete_topic(srv_data->request_topic);
    delete srv_data;
    RMW_SET_ERROR_MSG("failed to allocate service");
    return nullptr;
  }

  service->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  service->data = srv_data;
  service->service_name = rcutils_strdup(service_name, node->context->options.allocator);

  if (service->service_name == nullptr) {
    rmw_service_free(service);
    int2dds_delete_datawriter(srv_data->response_writer);
    int2dds_delete_datareader(srv_data->request_reader);
    int2dds_delete_topic(srv_data->response_topic);
    int2dds_delete_topic(srv_data->request_topic);
    delete srv_data;
    RMW_SET_ERROR_MSG("failed to allocate service name");
    return nullptr;
  }

  // Track service in node
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    node_data->services.push_back(srv_data->gid);
    node_data->live_services.push_back(srv_data);
  }

  // Listener starts with an empty mask; refreshed when user callbacks register
  rmw_int2dds_cpp::refresh_service_listener(srv_data);
  rmw_int2dds_cpp::trigger_graph_guard_condition(context_data);

  if (context_data->common) {
    rmw_gid_t request_reader_gid{};
    rmw_gid_t response_writer_gid{};
    uint8_t guid[16];
    if (int2dds_datareader_get_guid(srv_data->request_reader, &guid) == INT2DDS_RET_OK) {
      std::memcpy(request_reader_gid.data, guid, sizeof(guid));
    }
    if (int2dds_datawriter_get_guid(srv_data->response_writer, &guid) == INT2DDS_RET_OK) {
      std::memcpy(response_writer_gid.data, guid, sizeof(guid));
    }
    rmw_int2dds_cpp::common_add_local_entity(
      context_data, request_reader_gid, request_topic_name, request_type_name,
      srv_data->qos, /*is_reader=*/ true);
    rmw_int2dds_cpp::common_add_local_entity(
      context_data, response_writer_gid, response_topic_name, response_type_name,
      srv_data->qos, /*is_reader=*/ false);
    rmw_int2dds_cpp::common_associate_local_reader(
      context_data, request_reader_gid, node_data->name, node_data->namespace_);
    rmw_int2dds_cpp::common_associate_local_writer(
      context_data, response_writer_gid, node_data->name, node_data->namespace_);
  }

  return service;
}

rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (service->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("service not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * node_data = static_cast<rmw_int2dds_cpp::NodeData *>(node->data);
  auto * srv_data = static_cast<rmw_int2dds_cpp::ServiceData *>(service->data);

  if (srv_data == nullptr) {
    RMW_SET_ERROR_MSG("data is null");
    return RMW_RET_ERROR;
  }

  if (!srv_data->detached) {
    if (node_data == nullptr) {
      RMW_SET_ERROR_MSG("data is null");
      return RMW_RET_ERROR;
    }

    {
      std::lock_guard<std::mutex> lock(node_data->entities_mutex);

      // Remove from node's service list
      auto & srvs = node_data->services;
      for (auto it = srvs.begin(); it != srvs.end(); ++it) {
        if (std::memcmp(it->data, srv_data->gid.data, RMW_GID_STORAGE_SIZE) == 0) {
          srvs.erase(it);
          break;
        }
      }

      auto & live_services = node_data->live_services;
      live_services.erase(
        std::remove(live_services.begin(), live_services.end(), srv_data),
        live_services.end());
    }

    if (node_data->context_data != nullptr) {
      rmw_int2dds_cpp::trigger_graph_guard_condition(node_data->context_data);
    }

    if (node_data->context_data != nullptr && node_data->context_data->common) {
      rmw_gid_t request_reader_gid{};
      rmw_gid_t response_writer_gid{};
      uint8_t guid[16];
      if (int2dds_datareader_get_guid(srv_data->request_reader, &guid) == INT2DDS_RET_OK) {
        std::memcpy(request_reader_gid.data, guid, sizeof(guid));
      }
      if (int2dds_datawriter_get_guid(srv_data->response_writer, &guid) == INT2DDS_RET_OK) {
        std::memcpy(response_writer_gid.data, guid, sizeof(guid));
      }
      rmw_int2dds_cpp::common_dissociate_local_reader(
        node_data->context_data, request_reader_gid, node_data->name, node_data->namespace_);
      rmw_int2dds_cpp::common_dissociate_local_writer(
        node_data->context_data, response_writer_gid, node_data->name, node_data->namespace_);
      rmw_int2dds_cpp::common_remove_local_entity(
        node_data->context_data, request_reader_gid, /*is_reader=*/ true);
      rmw_int2dds_cpp::common_remove_local_entity(
        node_data->context_data, response_writer_gid, /*is_reader=*/ false);
    }

    // Delete DDS entities
    rmw_int2dds_cpp::waitset_registry_clean_caches();
    if (srv_data->request_status_condition != nullptr) {
      int2dds_statuscondition_delete(srv_data->request_status_condition);
      srv_data->request_status_condition = nullptr;
    }
    if (srv_data->response_writer != nullptr) {
      int2dds_delete_datawriter(srv_data->response_writer);
    }
    if (srv_data->request_reader != nullptr) {
      int2dds_delete_datareader(srv_data->request_reader);
    }
    if (srv_data->response_topic != nullptr) {
      int2dds_delete_topic(srv_data->response_topic);
    }
    if (srv_data->request_topic != nullptr) {
      int2dds_delete_topic(srv_data->request_topic);
    }
  } else {
    // The detach path already released and nulled the reader/writer/topics but
    // left request_status_condition live and srv_data still referenced by any
    // wait-set cache. Clean the caches and release the condition before srv_data
    // is freed (no double free: the detach path never touched the condition).
    rmw_int2dds_cpp::waitset_registry_clean_caches();
    if (srv_data->request_status_condition != nullptr) {
      int2dds_statuscondition_delete(srv_data->request_status_condition);
      srv_data->request_status_condition = nullptr;
    }
  }

  // Free service name
  if (service->service_name != nullptr && node->context != nullptr) {
    node->context->options.allocator.deallocate(
      const_cast<char *>(service->service_name),
      node->context->options.allocator.state);
  }

  delete srv_data;
  rmw_service_free(service);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  if (node == nullptr) {
    RMW_SET_ERROR_MSG("node argument is null");
    return RMW_RET_ERROR;
  }
  if (client == nullptr) {
    RMW_SET_ERROR_MSG("client argument is null");
    return RMW_RET_ERROR;
  }
  if (is_available == nullptr) {
    RMW_SET_ERROR_MSG("is_available argument is null");
    return RMW_RET_ERROR;
  }

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (client->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("client not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * cli_data = static_cast<rmw_int2dds_cpp::ClientData *>(client->data);
  if (cli_data == nullptr) {
    RMW_SET_ERROR_MSG("client data is null");
    return RMW_RET_ERROR;
  }

  // Availability must reflect the *current* match count in both directions.
  // The first out-param is total_count (cumulative, never decreases); using it
  // would keep a service "available" forever after its server is destroyed.
  // The second out-param is current_count, which drops on unmatch.
  int32_t request_current = 0;
  int32_t response_current = 0;

  if (cli_data->request_writer != nullptr) {
    Int2DdsPublicationMatchedStatus s = {};
    int2dds_datawriter_get_publication_matched_status(cli_data->request_writer, &s);
    request_current = s.current_count;
  }

  if (cli_data->response_reader != nullptr) {
    Int2DdsSubscriptionMatchedStatus s = {};
    int2dds_datareader_get_subscription_matched_status(cli_data->response_reader, &s);
    response_current = s.current_count;
  }

  // Service is available only while currently matched on both directions
  *is_available = (request_current > 0) && (response_current > 0);

  return RMW_RET_OK;
}

// Not part of the Foxy rmw API (Humble): rmw_service_request_subscription_get_actual_qos,
// rmw_service_response_publisher_get_actual_qos and
// rmw_service_set_on_new_request_callback.

}  // extern "C"
