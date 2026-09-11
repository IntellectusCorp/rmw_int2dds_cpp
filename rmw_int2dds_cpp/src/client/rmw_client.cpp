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
rmw_gid_t generate_client_gid();
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
// client that only worked on this RMW would break on any other one. Map the
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

rmw_client_t *
rmw_create_client(
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
    RMW_SET_ERROR_MSG("unknown QoS policy is invalid for client");
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

  // Create client data
  auto * cli_data = new (std::nothrow) rmw_int2dds_cpp::ClientData();
  if (cli_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate client data");
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
    delete cli_data;
    RMW_SET_ERROR_MSG("failed to get introspection service type support");
    return nullptr;
  }

  cli_data->type_support = introspection_ts;
  cli_data->qos = *qos_policies;
  // Resolve system-default values up front so the profile stored here is the one
  // actually applied below, which is what rmw_client_*_get_actual_qos reports.
  rmw_int2dds_cpp::resolve_actual_qos(&cli_data->qos);
  cli_data->gid = rmw_int2dds_cpp::generate_client_gid();
  cli_data->node_data = node_data;
  cli_data->service_name = service_name;

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
    &cli_data->request_topic);
  if (dds_ret != INT2DDS_RET_OK) {
    delete cli_data;
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
    &cli_data->response_topic);
  if (dds_ret != INT2DDS_RET_OK) {
    int2dds_delete_topic(cli_data->request_topic);
    delete cli_data;
    RMW_SET_ERROR_MSG("failed to create response topic");
    return nullptr;
  }

  // Create DataWriter QoS for requests
  Int2DdsDataWriterQos * writer_qos = nullptr;
  int2dds_datawriter_qos_create_default(&writer_qos);
  rmw_int2dds_cpp::apply_type_hash_user_data(
    writer_qos, rmw_int2dds_cpp::encode_service_request_type_hash_user_data(introspection_ts));
  set_writer_reliability_durability(writer_qos, cli_data->qos);
  set_writer_deadline_lifespan_liveliness(writer_qos, cli_data->qos);
  set_writer_history(writer_qos, cli_data->qos);

  // Create request DataWriter
  dds_ret = int2dds_create_datawriter(
    context_data->default_publisher,
    cli_data->request_topic,
    writer_qos,
    nullptr,
    0,
    &cli_data->request_writer);
  int2dds_datawriter_qos_destroy(writer_qos);

  if (dds_ret != INT2DDS_RET_OK) {
    int2dds_delete_topic(cli_data->response_topic);
    int2dds_delete_topic(cli_data->request_topic);
    delete cli_data;
    RMW_SET_ERROR_MSG("failed to create request DataWriter");
    return nullptr;
  }

  // Create DataReader QoS for responses
  Int2DdsDataReaderQos * reader_qos = nullptr;
  int2dds_datareader_qos_create_default(&reader_qos);
  rmw_int2dds_cpp::apply_type_hash_user_data(
    reader_qos, rmw_int2dds_cpp::encode_service_response_type_hash_user_data(introspection_ts));
  set_reader_reliability_durability(reader_qos, cli_data->qos);
  set_reader_deadline_liveliness(reader_qos, cli_data->qos);
  set_reader_history(reader_qos, cli_data->qos);

  // Create response DataReader
  dds_ret = int2dds_create_datareader(
    context_data->default_subscriber,
    cli_data->response_topic,
    reader_qos,
    nullptr,
    0,
    &cli_data->response_reader);
  int2dds_datareader_qos_destroy(reader_qos);

  if (dds_ret != INT2DDS_RET_OK) {
    int2dds_delete_datawriter(cli_data->request_writer);
    int2dds_delete_topic(cli_data->response_topic);
    int2dds_delete_topic(cli_data->request_topic);
    delete cli_data;
    RMW_SET_ERROR_MSG("failed to create response DataReader");
    return nullptr;
  }

  // Allocate RMW client
  rmw_client_t * client = rmw_client_allocate();
  if (client == nullptr) {
    int2dds_delete_datareader(cli_data->response_reader);
    int2dds_delete_datawriter(cli_data->request_writer);
    int2dds_delete_topic(cli_data->response_topic);
    int2dds_delete_topic(cli_data->request_topic);
    delete cli_data;
    RMW_SET_ERROR_MSG("failed to allocate client");
    return nullptr;
  }

  client->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  client->data = cli_data;
  client->service_name = rcutils_strdup(service_name, node->context->options.allocator);

  if (client->service_name == nullptr) {
    rmw_client_free(client);
    int2dds_delete_datareader(cli_data->response_reader);
    int2dds_delete_datawriter(cli_data->request_writer);
    int2dds_delete_topic(cli_data->response_topic);
    int2dds_delete_topic(cli_data->request_topic);
    delete cli_data;
    RMW_SET_ERROR_MSG("failed to allocate service name");
    return nullptr;
  }

  // Track client in node
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    node_data->clients.push_back(cli_data->gid);
    node_data->live_clients.push_back(cli_data);
  }

  // Listener starts with an empty mask; refreshed when user callbacks register
  rmw_int2dds_cpp::refresh_client_listener(cli_data);
  rmw_int2dds_cpp::trigger_graph_guard_condition(context_data);

  if (context_data->common) {
    rmw_gid_t request_writer_gid{};
    rmw_gid_t response_reader_gid{};
    uint8_t guid[16];
    if (int2dds_datawriter_get_guid(cli_data->request_writer, &guid) == INT2DDS_RET_OK) {
      std::memcpy(request_writer_gid.data, guid, sizeof(guid));
    }
    if (int2dds_datareader_get_guid(cli_data->response_reader, &guid) == INT2DDS_RET_OK) {
      std::memcpy(response_reader_gid.data, guid, sizeof(guid));
    }
    rmw_int2dds_cpp::common_add_local_entity(
      context_data, request_writer_gid, request_topic_name, request_type_name,
      cli_data->qos, /*is_reader=*/ false);
    rmw_int2dds_cpp::common_add_local_entity(
      context_data, response_reader_gid, response_topic_name, response_type_name,
      cli_data->qos, /*is_reader=*/ true);
    rmw_int2dds_cpp::common_associate_local_writer(
      context_data, request_writer_gid, node_data->name, node_data->namespace_);
    rmw_int2dds_cpp::common_associate_local_reader(
      context_data, response_reader_gid, node_data->name, node_data->namespace_);
  }

  return client;
}

rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (client->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("client not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * node_data = static_cast<rmw_int2dds_cpp::NodeData *>(node->data);
  auto * cli_data = static_cast<rmw_int2dds_cpp::ClientData *>(client->data);

  if (node_data == nullptr || cli_data == nullptr) {
    RMW_SET_ERROR_MSG("data is null");
    return RMW_RET_ERROR;
  }

  if (!cli_data->detached) {
    // Remove from node's client list
    {
      std::lock_guard<std::mutex> lock(node_data->entities_mutex);
      auto & clis = node_data->clients;
      for (auto it = clis.begin(); it != clis.end(); ++it) {
        if (std::memcmp(it->data, cli_data->gid.data, RMW_GID_STORAGE_SIZE) == 0) {
          clis.erase(it);
          break;
        }
      }

      auto & live_clients = node_data->live_clients;
      live_clients.erase(
        std::remove(live_clients.begin(), live_clients.end(), cli_data),
        live_clients.end());
    }

    if (node_data->context_data != nullptr) {
      rmw_int2dds_cpp::trigger_graph_guard_condition(node_data->context_data);
    }

    if (node_data->context_data != nullptr && node_data->context_data->common) {
      rmw_gid_t request_writer_gid{};
      rmw_gid_t response_reader_gid{};
      uint8_t guid[16];
      if (int2dds_datawriter_get_guid(cli_data->request_writer, &guid) == INT2DDS_RET_OK) {
        std::memcpy(request_writer_gid.data, guid, sizeof(guid));
      }
      if (int2dds_datareader_get_guid(cli_data->response_reader, &guid) == INT2DDS_RET_OK) {
        std::memcpy(response_reader_gid.data, guid, sizeof(guid));
      }
      rmw_int2dds_cpp::common_dissociate_local_writer(
        node_data->context_data, request_writer_gid, node_data->name, node_data->namespace_);
      rmw_int2dds_cpp::common_dissociate_local_reader(
        node_data->context_data, response_reader_gid, node_data->name, node_data->namespace_);
      rmw_int2dds_cpp::common_remove_local_entity(
        node_data->context_data, request_writer_gid, /*is_reader=*/ false);
      rmw_int2dds_cpp::common_remove_local_entity(
        node_data->context_data, response_reader_gid, /*is_reader=*/ true);
    }

    // Delete DDS entities
    rmw_int2dds_cpp::waitset_registry_clean_caches();
    if (cli_data->response_status_condition != nullptr) {
      int2dds_statuscondition_delete(cli_data->response_status_condition);
      cli_data->response_status_condition = nullptr;
    }
    if (cli_data->response_reader != nullptr) {
      int2dds_delete_datareader(cli_data->response_reader);
    }
    if (cli_data->request_writer != nullptr) {
      int2dds_delete_datawriter(cli_data->request_writer);
    }
    if (cli_data->response_topic != nullptr) {
      int2dds_delete_topic(cli_data->response_topic);
    }
    if (cli_data->request_topic != nullptr) {
      int2dds_delete_topic(cli_data->request_topic);
    }
  } else {
    // The detach path already released and nulled the reader/writer/topics but
    // left response_status_condition live and cli_data still referenced by any
    // wait-set cache. Clean the caches and release the condition before cli_data
    // is freed (no double free: the detach path never touched the condition).
    rmw_int2dds_cpp::waitset_registry_clean_caches();
    if (cli_data->response_status_condition != nullptr) {
      int2dds_statuscondition_delete(cli_data->response_status_condition);
      cli_data->response_status_condition = nullptr;
    }
  }

  // Free service name
  if (client->service_name != nullptr && node->context != nullptr) {
    node->context->options.allocator.deallocate(
      const_cast<char *>(client->service_name),
      node->context->options.allocator.state);
  }

  delete cli_data;
  rmw_client_free(client);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_gid_for_client(const rmw_client_t * client, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);

  if (client->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("client not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * cli_data = static_cast<rmw_int2dds_cpp::ClientData *>(client->data);
  if (cli_data == nullptr) {
    RMW_SET_ERROR_MSG("client data is null");
    return RMW_RET_ERROR;
  }

  gid->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  std::memcpy(gid->data, cli_data->gid.data, RMW_GID_STORAGE_SIZE);

  return RMW_RET_OK;
}

// Not part of the Foxy rmw API (Humble): rmw_client_request_publisher_get_actual_qos,
// rmw_client_response_subscription_get_actual_qos and
// rmw_client_set_on_new_response_callback.

}  // extern "C"
