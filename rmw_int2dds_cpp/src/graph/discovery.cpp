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

#include "discovery.hpp"

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "rcutils/allocator.h"
#include "rcutils/logging_macros.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/serialized_message.h"

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

#include "rmw_dds_common/context.hpp"
#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"

#include "int2dds-ffi.h"  // NOLINT(build/include): vendored FFI header

#include "rmw_int2dds_cpp/types.hpp"
#include "../common/type_hash_qos.hpp"
#include "graph_guard.hpp"

namespace rmw_int2dds_cpp
{

// Defined in common/gid.cpp.
rmw_gid_t generate_participant_gid();
// Defined in common/topic_names.cpp.
std::string get_dds_type_name(const rosidl_message_type_support_t * type_support);

namespace
{

// Literal topic name shared by all ROS 2 RMWs for node-graph discovery.
constexpr const char * kDiscoveryTopicName = "ros_discovery_info";

const rosidl_message_type_support_t *
get_discovery_type_support()
{
  return rosidl_typesupport_cpp::get_message_type_support_handle<
    rmw_dds_common::msg::ParticipantEntitiesInfo>();
}

// Serialize a ParticipantEntitiesInfo message and write it on the discovery writer.
// Reuses int2dds' standard CDR path (rmw_serialize), identical to user messages.
rmw_ret_t publish_entities_info(
  Int2DdsDataWriter * writer,
  const rosidl_message_type_support_t * type_support,
  const void * msg)
{
  if (writer == nullptr || type_support == nullptr || msg == nullptr) {
    return RMW_RET_ERROR;
  }

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  rmw_ret_t ret = rmw_serialized_message_init(&serialized, 0u, &allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  ret = rmw_serialize(msg, type_support, &serialized);
  if (ret == RMW_RET_OK) {
    const Int2DdsRet wret = int2dds_datawriter_write_serialized(
      writer, serialized.buffer, serialized.buffer_length);
    if (wret != INT2DDS_RET_OK) {
      ret = RMW_RET_ERROR;
    }
  }

  std::ignore = rmw_serialized_message_fini(&serialized);
  return ret;
}

rmw_ret_t publish_common_graph_message(
  ContextData * context_data,
  const rmw_dds_common::msg::ParticipantEntitiesInfo & msg)
{
  if (context_data == nullptr || context_data->discovery_writer == nullptr) {
    return RMW_RET_ERROR;
  }

  return publish_entities_info(
    context_data->discovery_writer,
    get_discovery_type_support(),
    &msg);
}

// Listener loop: take ParticipantEntitiesInfo samples from the discovery reader,
// drop our own, and merge remote ones into the rmw_dds_common GraphCache.
void listener_loop(
  ContextData * context_data,
  const rosidl_message_type_support_t * type_support)
{
  std::vector<uint8_t> buffer(65536);
  while (context_data->common->thread_is_running.load()) {
    uintptr_t actual_size = 0;
    bool valid = false;
    const Int2DdsRet take = int2dds_datareader_take_serialized(
      context_data->discovery_reader, buffer.data(), buffer.size(), &actual_size, &valid);

    // A sample larger than our buffer stays in the reader cache, and this reader is
    // KEEP_ALL: retrying at the same size would spin on that one sample forever and
    // block every participant queued behind it. Grow to the size the FFI reports and
    // take again instead.
    if (take == INT2DDS_RET_BUFFER_TOO_SMALL) {
      // Only grow, and only within reason. Retrying without a bigger buffer would
      // spin this thread at full speed - the very failure mode the growth is here
      // to remove - and an implausible length must not turn into a huge
      // allocation whose bad_alloc escapes the listener thread.
      constexpr uintptr_t kMaxDiscoverySampleBytes = 16u * 1024u * 1024u;
      if (actual_size <= buffer.size() || actual_size > kMaxDiscoverySampleBytes) {
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_int2dds_cpp",
          "discovery reader reported an unusable sample size %zu (buffer is %zu); "
          "dropping this take",
          static_cast<size_t>(actual_size), buffer.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      RCUTILS_LOG_WARN_NAMED(
        "rmw_int2dds_cpp",
        "discovery sample needs %zu bytes, buffer is %zu; growing",
        static_cast<size_t>(actual_size), buffer.size());
      buffer.resize(actual_size);
      continue;
    }

    if (take != INT2DDS_RET_OK || !valid || actual_size == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
    serialized.buffer = buffer.data();
    serialized.buffer_length = actual_size;
    serialized.buffer_capacity = buffer.size();
    serialized.allocator = rcutils_get_default_allocator();

    rmw_dds_common::msg::ParticipantEntitiesInfo msg;
    if (rmw_deserialize(&serialized, type_support, &msg) != RMW_RET_OK) {
      continue;
    }

    using rmw_dds_common::operator==;
    rmw_gid_t msg_gid;
    rmw_dds_common::convert_msg_to_gid(&msg.gid, &msg_gid);
    if (msg_gid == context_data->common->gid) {
      continue;
    }

    context_data->common->graph_cache.update_participant_entities(msg);
    trigger_graph_guard_condition(context_data);
  }
}

}  // namespace

rmw_ret_t init_discovery(ContextData * context_data, const char * enclave)
{
  if (context_data == nullptr || context_data->participant == nullptr) {
    return RMW_RET_ERROR;
  }

  // Typesupport for the discovery message (generic handle + introspection handle).
  const rosidl_message_type_support_t * type_support = get_discovery_type_support();
  const rosidl_message_type_support_t * introspection_ts =
    get_message_typesupport_handle(
    type_support, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (type_support == nullptr || introspection_ts == nullptr) {
    RMW_SET_ERROR_MSG("failed to obtain ParticipantEntitiesInfo typesupport");
    return RMW_RET_ERROR;
  }
  const std::string dds_type_name = get_dds_type_name(introspection_ts);
  const std::string type_hash_user_data =
    encode_message_type_hash_user_data(introspection_ts);

  // ros_discovery_info topic (literal name, no ROS mangling).
  Int2DdsRet ret = int2dds_create_topic(
    context_data->participant, kDiscoveryTopicName, dds_type_name.c_str(),
    INT2DDS_EXTENSIBILITY_MUTABLE, nullptr, &context_data->discovery_topic);
  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to create ros_discovery_info topic");
    return RMW_RET_ERROR;
  }

  // Writer: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(1).
  Int2DdsDataWriterQos * writer_qos = nullptr;
  int2dds_datawriter_qos_create_default(&writer_qos);
  apply_type_hash_user_data(writer_qos, type_hash_user_data);
  int2dds_datawriter_qos_set_reliability(writer_qos, INT2DDS_QOS_RELIABILITY_RELIABLE, 100000000);
  int2dds_datawriter_qos_set_durability(writer_qos, INT2DDS_QOS_DURABILITY_TRANSIENT_LOCAL);
  int2dds_datawriter_qos_set_history(writer_qos, INT2DDS_QOS_HISTORY_KEEP_LAST, 1);
  ret = int2dds_create_datawriter(
    context_data->default_publisher, context_data->discovery_topic, writer_qos,
    nullptr, 0,
    &context_data->discovery_writer);
  int2dds_datawriter_qos_destroy(writer_qos);
  if (ret != INT2DDS_RET_OK) {
    int2dds_delete_topic(context_data->discovery_topic);
    context_data->discovery_topic = nullptr;
    RMW_SET_ERROR_MSG("failed to create ros_discovery_info writer");
    return RMW_RET_ERROR;
  }

  // Reader: RELIABLE, TRANSIENT_LOCAL, KEEP_ALL.
  Int2DdsDataReaderQos * reader_qos = nullptr;
  int2dds_datareader_qos_create_default(&reader_qos);
  apply_type_hash_user_data(reader_qos, type_hash_user_data);
  int2dds_datareader_qos_set_reliability(reader_qos, INT2DDS_QOS_RELIABILITY_RELIABLE, 100000000);
  int2dds_datareader_qos_set_durability(reader_qos, INT2DDS_QOS_DURABILITY_TRANSIENT_LOCAL);
  int2dds_datareader_qos_set_history(reader_qos, INT2DDS_QOS_HISTORY_KEEP_ALL, 0);
  ret = int2dds_create_datareader(
    context_data->default_subscriber, context_data->discovery_topic, reader_qos,
    nullptr, 0,
    &context_data->discovery_reader);
  int2dds_datareader_qos_destroy(reader_qos);
  if (ret != INT2DDS_RET_OK) {
    int2dds_delete_datawriter(context_data->discovery_writer);
    context_data->discovery_writer = nullptr;
    int2dds_delete_topic(context_data->discovery_topic);
    context_data->discovery_topic = nullptr;
    RMW_SET_ERROR_MSG("failed to create ros_discovery_info reader");
    return RMW_RET_ERROR;
  }

  // rmw_dds_common Context: holds the standard GraphCache. The participant gid must be
  // the real RTPS participant key (GUID prefix), not a synthetic value: remote endpoints
  // discovered over SEDP carry that same participant key, so rmw_dds_common can associate
  // an endpoint with the node that owns it.
  context_data->common = std::make_unique<rmw_dds_common::Context>();
  context_data->common->gid = generate_participant_gid();
  {
    uint8_t writer_guid[16] = {};
    if (int2dds_datawriter_get_guid(
        context_data->discovery_writer, reinterpret_cast<uint8_t(*)[16]>(&writer_guid)) ==
      INT2DDS_RET_OK)
    {
      std::memset(context_data->common->gid.data, 0, sizeof(context_data->common->gid.data));
      std::memcpy(context_data->common->gid.data, writer_guid, 12);
    }
  }
  context_data->common->pub = nullptr;
  context_data->common->sub = nullptr;
  context_data->common->listener_thread_gc = nullptr;
  context_data->common->graph_guard_condition = nullptr;
  context_data->common->thread_is_running.store(false);

  context_data->common->graph_cache.set_on_change_callback(
    [context_data]() {
      trigger_graph_guard_condition(context_data);
    });

  // Register this participant so node/endpoint associations have a home.
  // Use the context enclave so get_node_names_with_enclaves reports it.
  context_data->common->graph_cache.add_participant(
    context_data->common->gid, enclave != nullptr ? enclave : "/");

  context_data->common->thread_is_running.store(true);
  context_data->common->listener_thread =
    std::thread(listener_loop, context_data, type_support);

  // Consume core SEDP endpoint discovery incrementally (and bootstrap current state).
  enable_endpoint_push(context_data);

  return RMW_RET_OK;
}

void fini_discovery(ContextData * context_data)
{
  if (context_data == nullptr) {
    return;
  }
  // Stop new callbacks from touching context_data before we tear it down.
  disable_endpoint_push(context_data);
  if (context_data->common) {
    context_data->common->graph_cache.clear_on_change_callback();
    context_data->common->thread_is_running.store(false);
    if (context_data->common->listener_thread.joinable()) {
      context_data->common->listener_thread.join();
    }
  }
  if (context_data->discovery_reader != nullptr) {
    int2dds_delete_datareader(context_data->discovery_reader);
    context_data->discovery_reader = nullptr;
  }
  if (context_data->discovery_writer != nullptr) {
    int2dds_delete_datawriter(context_data->discovery_writer);
    context_data->discovery_writer = nullptr;
  }
  if (context_data->discovery_topic != nullptr) {
    int2dds_delete_topic(context_data->discovery_topic);
    context_data->discovery_topic = nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(context_data->remote_sync_mutex);
    context_data->common.reset();
  }
}

rmw_ret_t announce_node(
  ContextData * context_data, const std::string & name,
  const std::string & ns)
{
  if (context_data == nullptr || !context_data->common) {
    return RMW_RET_OK;
  }

  std::lock_guard<std::mutex> lock(context_data->common->node_update_mutex);
  auto msg = context_data->common->graph_cache.add_node(context_data->common->gid, name, ns);
  return publish_common_graph_message(context_data, msg);
}

rmw_ret_t withdraw_node(
  ContextData * context_data, const std::string & name,
  const std::string & ns)
{
  if (context_data == nullptr || !context_data->common) {
    return RMW_RET_OK;
  }

  std::lock_guard<std::mutex> lock(context_data->common->node_update_mutex);
  auto msg = context_data->common->graph_cache.remove_node(context_data->common->gid, name, ns);
  return publish_common_graph_message(context_data, msg);
}

rmw_ret_t common_add_local_entity(
  ContextData * context_data,
  const rmw_gid_t & gid,
  const std::string & topic_name,
  const std::string & type_name,
  const rmw_qos_profile_t & qos,
  bool is_reader)
{
  if (context_data == nullptr || !context_data->common) {
    return RMW_RET_OK;
  }

  std::lock_guard<std::mutex> lock(context_data->common->node_update_mutex);
  context_data->common->graph_cache.add_entity(
    gid, topic_name, type_name, context_data->common->gid, qos, is_reader);
  return RMW_RET_OK;
}

rmw_ret_t common_remove_local_entity(
  ContextData * context_data,
  const rmw_gid_t & gid,
  bool is_reader)
{
  if (context_data == nullptr || !context_data->common) {
    return RMW_RET_OK;
  }

  std::lock_guard<std::mutex> lock(context_data->common->node_update_mutex);
  context_data->common->graph_cache.remove_entity(gid, is_reader);
  return RMW_RET_OK;
}

rmw_ret_t common_associate_local_writer(
  ContextData * context_data,
  const rmw_gid_t & writer_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  if (context_data == nullptr || !context_data->common) {
    return RMW_RET_OK;
  }

  std::lock_guard<std::mutex> lock(context_data->common->node_update_mutex);
  auto msg = context_data->common->graph_cache.associate_writer(
    writer_gid, context_data->common->gid, node_name, node_namespace);
  return publish_common_graph_message(context_data, msg);
}

rmw_ret_t common_dissociate_local_writer(
  ContextData * context_data,
  const rmw_gid_t & writer_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  if (context_data == nullptr || !context_data->common) {
    return RMW_RET_OK;
  }

  std::lock_guard<std::mutex> lock(context_data->common->node_update_mutex);
  auto msg = context_data->common->graph_cache.dissociate_writer(
    writer_gid, context_data->common->gid, node_name, node_namespace);
  return publish_common_graph_message(context_data, msg);
}

rmw_ret_t common_associate_local_reader(
  ContextData * context_data,
  const rmw_gid_t & reader_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  if (context_data == nullptr || !context_data->common) {
    return RMW_RET_OK;
  }

  std::lock_guard<std::mutex> lock(context_data->common->node_update_mutex);
  auto msg = context_data->common->graph_cache.associate_reader(
    reader_gid, context_data->common->gid, node_name, node_namespace);
  return publish_common_graph_message(context_data, msg);
}

rmw_ret_t common_dissociate_local_reader(
  ContextData * context_data,
  const rmw_gid_t & reader_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  if (context_data == nullptr || !context_data->common) {
    return RMW_RET_OK;
  }

  std::lock_guard<std::mutex> lock(context_data->common->node_update_mutex);
  auto msg = context_data->common->graph_cache.dissociate_reader(
    reader_gid, context_data->common->gid, node_name, node_namespace);
  return publish_common_graph_message(context_data, msg);
}

}  // namespace rmw_int2dds_cpp
