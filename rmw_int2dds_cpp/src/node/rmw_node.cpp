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

#include "rmw/rmw.h"
#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "int2dds-ffi.h"  // NOLINT(build/include)
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "../graph/discovery.hpp"
#include "../graph/graph_guard.hpp"

// Forward declarations from gid.cpp
namespace rmw_int2dds_cpp
{
rmw_gid_t generate_participant_gid();
}

extern "C" rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition);

namespace
{

std::string
make_node_fqn(const rmw_int2dds_cpp::NodeData * node_data)
{
  if (node_data->namespace_ == "/") {
    return "/" + node_data->name;
  }
  return node_data->namespace_ + "/" + node_data->name;
}

bool
is_hidden_builtin_service(
  const rmw_int2dds_cpp::NodeData * node_data,
  const rmw_int2dds_cpp::ServiceData * service_data)
{
  if (node_data == nullptr || service_data == nullptr) {
    return false;
  }

  static const char * kHiddenServiceSuffixes[] = {
    "/describe_parameters",
    "/get_parameter_types",
    "/get_parameters",
    "/get_type_description",
    "/list_parameters",
    "/set_parameters",
    "/set_parameters_atomically",
  };

  const std::string node_fqn = make_node_fqn(node_data);
  for (const auto * suffix : kHiddenServiceSuffixes) {
    if (service_data->service_name == node_fqn + suffix) {
      return true;
    }
  }
  return false;
}

void
detach_service_entities(rmw_int2dds_cpp::ServiceData * service_data)
{
  if (service_data == nullptr || service_data->detached) {
    return;
  }

  if (service_data->response_writer != nullptr) {
    int2dds_delete_datawriter(service_data->response_writer);
    service_data->response_writer = nullptr;
  }
  if (service_data->request_reader != nullptr) {
    int2dds_delete_datareader(service_data->request_reader);
    service_data->request_reader = nullptr;
  }
  if (service_data->response_topic != nullptr) {
    int2dds_delete_topic(service_data->response_topic);
    service_data->response_topic = nullptr;
  }
  if (service_data->request_topic != nullptr) {
    int2dds_delete_topic(service_data->request_topic);
    service_data->request_topic = nullptr;
  }

  service_data->detached = true;
  service_data->node_data = nullptr;
}

void
detach_client_entities(rmw_int2dds_cpp::ClientData * client_data)
{
  if (client_data == nullptr || client_data->detached) {
    return;
  }

  if (client_data->response_reader != nullptr) {
    int2dds_delete_datareader(client_data->response_reader);
    client_data->response_reader = nullptr;
  }
  if (client_data->request_writer != nullptr) {
    int2dds_delete_datawriter(client_data->request_writer);
    client_data->request_writer = nullptr;
  }
  if (client_data->response_topic != nullptr) {
    int2dds_delete_topic(client_data->response_topic);
    client_data->response_topic = nullptr;
  }
  if (client_data->request_topic != nullptr) {
    int2dds_delete_topic(client_data->request_topic);
    client_data->request_topic = nullptr;
  }

  client_data->detached = true;
  client_data->node_data = nullptr;
}

void
detach_hidden_builtin_services(rmw_int2dds_cpp::NodeData * node_data)
{
  if (node_data == nullptr) {
    return;
  }

  std::vector<rmw_int2dds_cpp::ServiceData *> services_to_detach;
  std::vector<rmw_gid_t> detached_gids;
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    if (node_data->live_services.empty()) {
      return;
    }

    detached_gids.reserve(node_data->live_services.size());

    auto & live_services = node_data->live_services;
    live_services.erase(
      std::remove_if(
        live_services.begin(),
        live_services.end(),
        [&](rmw_int2dds_cpp::ServiceData * service_data) {
          if (!is_hidden_builtin_service(node_data, service_data)) {
            return false;
          }
          services_to_detach.push_back(service_data);
          detached_gids.push_back(service_data->gid);
          return true;
        }),
      live_services.end());

    if (detached_gids.empty()) {
      return;
    }

    auto & services = node_data->services;
    services.erase(
      std::remove_if(
        services.begin(),
        services.end(),
        [&](const rmw_gid_t & gid) {
          return std::any_of(
            detached_gids.begin(),
            detached_gids.end(),
            [&](const rmw_gid_t & detached_gid) {
              return std::memcmp(gid.data, detached_gid.data, RMW_GID_STORAGE_SIZE) == 0;
            });
        }),
      services.end());
  }

  for (auto * service_data : services_to_detach) {
    detach_service_entities(service_data);
  }
}

void
detach_remaining_clients(rmw_int2dds_cpp::NodeData * node_data)
{
  if (node_data == nullptr) {
    return;
  }

  std::vector<rmw_int2dds_cpp::ClientData *> clients_to_detach;
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    if (node_data->clients.empty()) {
      return;
    }

    clients_to_detach = node_data->live_clients;
    node_data->clients.clear();
    node_data->live_clients.clear();
  }

  for (auto * client_data : clients_to_detach) {
    detach_client_entities(client_data);
  }
}

}  // namespace

extern "C"
{

rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  bool localhost_only)
{
  // Foxy still hands the node's domain id and localhost flag to every node, but
  // rcl_init already resolved both into the init options before rmw_init built
  // the participant, so the context runs with them. rmw_cyclonedds_cpp on Foxy
  // ignores them the same way.
  static_cast<void>(domain_id);
  static_cast<void>(localhost_only);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(namespace_, nullptr);

  if (context->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("context not from this implementation");
    return nullptr;
  }

  // Validate node name
  int validation_result = 0;
  rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);
  if (ret != RMW_RET_OK) {
    return nullptr;
  }
  if (validation_result != RMW_NODE_NAME_VALID) {
    RMW_SET_ERROR_MSG("invalid node name");
    return nullptr;
  }

  // Validate namespace
  ret = rmw_validate_namespace(namespace_, &validation_result, nullptr);
  if (ret != RMW_RET_OK) {
    return nullptr;
  }
  if (validation_result != RMW_NAMESPACE_VALID) {
    RMW_SET_ERROR_MSG("invalid namespace");
    return nullptr;
  }

  auto * context_data = reinterpret_cast<rmw_int2dds_cpp::ContextData *>(context->impl);
  if (context_data == nullptr) {
    RMW_SET_ERROR_MSG("context not initialized");
    return nullptr;
  }

  if (context_data->is_shutdown) {
    RMW_SET_ERROR_MSG("context already shutdown");
    return nullptr;
  }

  // Bring the DDS resources back if rmw_destroy_node released them when the last
  // node went away. Without this the context would be usable only once, which is
  // how rclcpp exercises it: one context, nodes created and destroyed repeatedly.
  // rmw_fastrtps does the same from increment_context_impl_ref_count().
  {
    std::lock_guard<std::mutex> lock(context_data->mutex);
    if (context_data->participant == nullptr) {
      if (rmw_int2dds_cpp::acquire_context_resources(
          context_data, context->options.enclave) != RMW_RET_OK)
      {
        return nullptr;
      }
    }
  }

  // Create node data
  auto * node_data = new (std::nothrow) rmw_int2dds_cpp::NodeData();
  if (node_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate node data");
    return nullptr;
  }

  node_data->context_data = context_data;
  node_data->name = name;
  node_data->namespace_ = namespace_;
  node_data->gid = rmw_int2dds_cpp::generate_participant_gid();

  // Allocate the node
  rmw_node_t * node = rmw_node_allocate();
  if (node == nullptr) {
    delete node_data;
    RMW_SET_ERROR_MSG("failed to allocate node");
    return nullptr;
  }

  node->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  node->data = node_data;
  node->name = rcutils_strdup(name, context->options.allocator);
  node->namespace_ = rcutils_strdup(namespace_, context->options.allocator);
  node->context = context;

  if (node->name == nullptr || node->namespace_ == nullptr) {
    if (node->name != nullptr) {
      context->options.allocator.deallocate(
        const_cast<char *>(node->name),
        context->options.allocator.state);
    }
    if (node->namespace_ != nullptr) {
      context->options.allocator.deallocate(
        const_cast<char *>(node->namespace_),
        context->options.allocator.state);
    }
    rmw_node_free(node);
    delete node_data;
    RMW_SET_ERROR_MSG("failed to allocate node name or namespace");
    return nullptr;
  }

  // Increment context reference count
  context_data->ref_count++;

  // Notify graph-change waiters that this node was added.
  rmw_int2dds_cpp::trigger_graph_guard_condition(context_data);

  // Standard node-graph discovery: announce this node via ros_discovery_info.
  (void)rmw_int2dds_cpp::announce_node(context_data, name, namespace_);

  return node;
}

rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * node_data = static_cast<rmw_int2dds_cpp::NodeData *>(node->data);
  if (node_data == nullptr) {
    RMW_SET_ERROR_MSG("node data is null");
    return RMW_RET_ERROR;
  }

  // rclpy uses destroy_when_not_in_use() for entities, so node destruction can
  // race slightly ahead of the final entity cleanup callbacks. Give those
  // deferred destructors a short grace period before reporting an error.
  detach_hidden_builtin_services(node_data);

  constexpr auto kDetachWait = std::chrono::milliseconds(2000);
  constexpr auto kDetachPoll = std::chrono::milliseconds(10);
  const auto deadline = std::chrono::steady_clock::now() + kDetachWait;

  auto has_live_entities = [&]() {
      std::lock_guard<std::mutex> lock(node_data->entities_mutex);
      return !node_data->publishers.empty() ||
             !node_data->subscriptions.empty() ||
             !node_data->services.empty() ||
             !node_data->clients.empty();
    };

  while (has_live_entities() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(kDetachPoll);
  }

  // rclpy's hidden type-description service may still be pending destruction
  // even after its Python wrapper has been removed. Proactively detach any
  // remaining tracked services so node teardown can complete cleanly.
  std::vector<rmw_int2dds_cpp::ServiceData *> services_to_detach;
  {
    std::lock_guard<std::mutex> lock(node_data->entities_mutex);
    if (!node_data->services.empty()) {
      services_to_detach = node_data->live_services;
      node_data->services.clear();
      node_data->live_services.clear();
    }
  }
  for (auto * service_data : services_to_detach) {
    detach_service_entities(service_data);
  }

  // Some low-level rmw tests intentionally leave a client alive until fixture
  // teardown after querying its actual QoS. Detach the DDS endpoints so node
  // teardown remains compatible with that contract.
  detach_remaining_clients(node_data);

  // Entities may still be tracked here when a caller's rmw_destroy_<entity>
  // failed and never removed itself from the node. Per the rmw contract, node
  // destruction must not be blocked by that: proceed and let the orphaned
  // entities remain the caller's responsibility.

  // Notify graph-change waiters that this node was removed.
  if (node_data->context_data != nullptr) {
    rmw_int2dds_cpp::trigger_graph_guard_condition(node_data->context_data);
  }

  // Standard node-graph discovery: withdraw this node from ros_discovery_info.
  if (node_data->context_data != nullptr) {
    (void)rmw_int2dds_cpp::withdraw_node(
      node_data->context_data,
      node_data->name,
      node_data->namespace_);
  }

  if (node_data->graph_guard_condition != nullptr) {
    const rmw_ret_t destroy_ret =
      rmw_destroy_guard_condition(node_data->graph_guard_condition);
    (void)destroy_ret;
    node_data->graph_guard_condition = nullptr;
  }

  // Decrement context reference count. Release the DDS resources once the last
  // node is gone instead of waiting for rmw_context_fini: a client library may
  // keep the context object alive (rclcpp holds internal references to it), so
  // rmw_context_fini never runs and every node create/destroy cycle leaks the
  // participant's sockets, eventfds and epoll instances.
  //
  // rmw_create_node brings the resources back when a node is created on a
  // context that has none, so a context stays reusable. This mirrors
  // rmw_fastrtps, where decrement_context_impl_ref_count() and
  // increment_context_impl_ref_count() are symmetric. Releasing without that
  // symmetric path left the context permanently unusable and broke every
  // rclcpp test that recreates nodes on one context.
  if (node_data->context_data != nullptr) {
    auto * ctx = node_data->context_data;
    std::lock_guard<std::mutex> lock(ctx->mutex);
    if (--ctx->ref_count <= 1 && ctx->participant != nullptr) {
      rmw_int2dds_cpp::release_context_resources(ctx);
    }
  }

  // Free name and namespace
  if (node->name != nullptr && node->context != nullptr) {
    node->context->options.allocator.deallocate(
      const_cast<char *>(node->name),
      node->context->options.allocator.state);
  }
  if (node->namespace_ != nullptr && node->context != nullptr) {
    node->context->options.allocator.deallocate(
      const_cast<char *>(node->namespace_),
      node->context->options.allocator.state);
  }

  delete node_data;
  rmw_node_free(node);

  return RMW_RET_OK;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  if (node->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("node not from this implementation");
    return nullptr;
  }

  auto * node_data = static_cast<rmw_int2dds_cpp::NodeData *>(node->data);
  if (node_data == nullptr || node_data->context_data == nullptr) {
    RMW_SET_ERROR_MSG("node data is null");
    return nullptr;
  }

  if (node_data->graph_guard_condition != nullptr) {
    return node_data->graph_guard_condition;
  }

  auto * gc_data = new (std::nothrow) rmw_int2dds_cpp::GuardConditionData();
  if (gc_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate graph guard condition data");
    return nullptr;
  }

  gc_data->guard_condition = node_data->context_data->graph_guard_condition;
  gc_data->owns_guard_condition = false;

  rmw_guard_condition_t * graph_guard_condition = rmw_guard_condition_allocate();
  if (graph_guard_condition == nullptr) {
    delete gc_data;
    RMW_SET_ERROR_MSG("failed to allocate graph guard condition wrapper");
    return nullptr;
  }

  graph_guard_condition->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  graph_guard_condition->data = gc_data;
  graph_guard_condition->context = node->context;

  node_data->graph_guard_condition = graph_guard_condition;
  (void)int2dds_guardcondition_set_trigger_value(gc_data->guard_condition, true);
  return node_data->graph_guard_condition;
}

}  // extern "C"
