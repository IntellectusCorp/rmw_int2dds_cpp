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

#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "rmw/init.h"
#include "rmw/init_options.h"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "int2dds-ffi.h"  // NOLINT(build/include)
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "../graph/discovery.hpp"
#include "../wait/waitset_registry.hpp"

namespace rmw_int2dds_cpp
{

void
release_context_resources(ContextData * context_data)
{
  if (context_data == nullptr) {
    return;
  }

  if (context_data->common) {
    fini_discovery(context_data);
  }
  if (context_data->graph_guard_condition != nullptr) {
    // Defensive: make sure no wait set still references this guard before freeing
    // it, covering out-of-contract teardown where the context outlives a wait set.
    waitset_registry_clean_caches();
    int2dds_guardcondition_delete(context_data->graph_guard_condition);
    context_data->graph_guard_condition = nullptr;
  }
  if (context_data->default_subscriber != nullptr) {
    int2dds_delete_subscriber(context_data->default_subscriber);
    context_data->default_subscriber = nullptr;
  }
  if (context_data->default_publisher != nullptr) {
    int2dds_delete_publisher(context_data->default_publisher);
    context_data->default_publisher = nullptr;
  }
  if (context_data->participant != nullptr) {
    int2dds_participant_delete_contained_entities(context_data->participant);
    int2dds_delete_participant(context_data->participant);
    context_data->participant = nullptr;
  }
  if (context_data->factory != nullptr) {
    int2dds_domain_participant_factory_finalize(context_data->factory);
    context_data->factory = nullptr;
  }
}

rmw_ret_t
acquire_context_resources(ContextData * context_data, const char * enclave)
{
  if (context_data == nullptr) {
    RMW_SET_ERROR_MSG("context data is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  Int2DdsRet ret = int2dds_domain_participant_factory_get_instance(&context_data->factory);
  if (ret != INT2DDS_RET_OK) {
    context_data->factory = nullptr;
    RMW_SET_ERROR_MSG("failed to get participant factory");
    return RMW_RET_ERROR;
  }

  if (!context_data->localhost_only) {
    ret = int2dds_create_participant(
      context_data->factory,
      static_cast<int32_t>(context_data->domain_id),
      nullptr,
      &context_data->participant);
  } else {
    // localhost_only: multicast_ttl=0 keeps multicast SPDP on the host, so remote
    // hosts cannot auto-discover us (local discovery is unaffected). A plain
    // property; no core change or optional feature plugin.
    Int2DdsParticipantQos * qos = nullptr;
    ret = int2dds_participant_qos_create_default(&qos);
    if (ret == INT2DDS_RET_OK) {
      ret = int2dds_participant_qos_set_multicast_ttl(qos, 0);
      if (ret == INT2DDS_RET_OK) {
        ret = int2dds_create_participant(
          context_data->factory,
          static_cast<int32_t>(context_data->domain_id),
          qos,
          &context_data->participant);
      }
      int2dds_participant_qos_destroy(qos);
    }
  }
  if (ret != INT2DDS_RET_OK) {
    context_data->participant = nullptr;
    release_context_resources(context_data);
    RMW_SET_ERROR_MSG("failed to create participant");
    return RMW_RET_ERROR;
  }

  ret = int2dds_create_publisher(
    context_data->participant, nullptr,
    &context_data->default_publisher);
  if (ret != INT2DDS_RET_OK) {
    context_data->default_publisher = nullptr;
    release_context_resources(context_data);
    RMW_SET_ERROR_MSG("failed to create default publisher");
    return RMW_RET_ERROR;
  }

  ret = int2dds_create_subscriber(
    context_data->participant, nullptr,
    &context_data->default_subscriber);
  if (ret != INT2DDS_RET_OK) {
    context_data->default_subscriber = nullptr;
    release_context_resources(context_data);
    RMW_SET_ERROR_MSG("failed to create default subscriber");
    return RMW_RET_ERROR;
  }

  ret = int2dds_guardcondition_new(&context_data->graph_guard_condition);
  if (ret != INT2DDS_RET_OK) {
    context_data->graph_guard_condition = nullptr;
    release_context_resources(context_data);
    RMW_SET_ERROR_MSG("failed to create graph guard condition");
    return RMW_RET_ERROR;
  }

  if (init_discovery(context_data, enclave) != RMW_RET_OK) {
    release_context_resources(context_data);
    RMW_SET_ERROR_MSG("failed to initialize node-graph discovery");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

}  // namespace rmw_int2dds_cpp

extern "C"
{

rmw_ret_t
rmw_init_options_init(
  rmw_init_options_t * init_options,
  rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);

  if (init_options->implementation_identifier != nullptr) {
    RMW_SET_ERROR_MSG("init_options already initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  init_options->instance_id = 0;
  init_options->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  init_options->allocator = allocator;
  init_options->impl = nullptr;
  init_options->enclave = nullptr;
  init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
  init_options->security_options = rmw_get_default_security_options();
  init_options->localhost_only = RMW_LOCALHOST_ONLY_DEFAULT;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_copy(
  const rmw_init_options_t * src,
  rmw_init_options_t * dst)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);

  if (src->implementation_identifier == nullptr) {
    RMW_SET_ERROR_MSG("src init_options is not initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (src->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("src init_options not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (dst->implementation_identifier != nullptr) {
    RMW_SET_ERROR_MSG("dst init_options already initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  *dst = *src;

  if (src->enclave != nullptr) {
    dst->enclave = rcutils_strdup(src->enclave, src->allocator);
    if (dst->enclave == nullptr) {
      return RMW_RET_BAD_ALLOC;
    }
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);

  if (init_options->implementation_identifier == nullptr) {
    RMW_SET_ERROR_MSG("init_options is not initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (init_options->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("init_options not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (init_options->enclave != nullptr) {
    init_options->allocator.deallocate(init_options->enclave, init_options->allocator.state);
    init_options->enclave = nullptr;
  }

  *init_options = rmw_get_zero_initialized_init_options();
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  if (options->implementation_identifier == nullptr) {
    RMW_SET_ERROR_MSG("options is not initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (options->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("options not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (options->enclave == nullptr) {
    RMW_SET_ERROR_MSG("options enclave is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (context->implementation_identifier != nullptr) {
    RMW_SET_ERROR_MSG("context already initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Default the DATA_FRAG fragment size to 1344 bytes for writers created through
  // this RMW, unless INT2DDS_DATA_FRAG_SIZE is already set. The int2dds core reads
  // this env when a writer's DataFrag QoS is unset; seeding it here scopes the 1344
  // default to the ROS/RMW path without changing the core's own default (65000).
  // overwrite=0 preserves any user-provided value.
  setenv("INT2DDS_DATA_FRAG_SIZE", "1344", 0);
  setenv("INT2DDS_MAX_MESSAGE_SIZE", "13440", 0);
  // Seed a large UDP socket buffer for the ROS/RMW path. The core's
  // default is small, so under high-rate / large-message reliable traffic the
  // kernel receive buffer overflows and drops fragments, which surfaces as lost
  // samples even under RELIABLE. overwrite=0 preserves a user-provided value.
  // NOTE: the kernel caps this at net.core.rmem_max/wmem_max, so deployments
  // must also raise those (standard DDS requirement) for the full effect.
  setenv("INT2DDS_UDP_SOCKET_BUFFER", "8388608", 0);
  // Create context data
  auto * context_data = new (std::nothrow) rmw_int2dds_cpp::ContextData();
  if (context_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate context data");
    return RMW_RET_BAD_ALLOC;
  }

  // Determine domain ID before narrowing to the DDS API's integer type.
  size_t actual_domain_id = options->domain_id;
  if (actual_domain_id == RMW_DEFAULT_DOMAIN_ID) {
    actual_domain_id = 0;  // Default to domain 0
  }
  context_data->domain_id = actual_domain_id;
#if __has_include("rmw/localhost.h")
  // Foxy and Humble express localhost-only via the init-options localhost_only
  // field (rcl_init resolves ROS_LOCALHOST_ONLY into it before rmw_init).
  context_data->localhost_only = options->localhost_only == RMW_LOCALHOST_ONLY_ENABLED;
#endif

  if (rmw_int2dds_cpp::acquire_context_resources(context_data, options->enclave) != RMW_RET_OK) {
    delete context_data;
    return RMW_RET_ERROR;
  }

  context_data->is_shutdown = false;
  context_data->ref_count = 1;

  // Set up context
  context->instance_id = options->instance_id;
  // rmw_context_t has no actual_domain_id on Foxy (Galactic+); rcl keeps the
  // resolved domain id itself (rcl_node_get_domain_id).
  context->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  context->impl = reinterpret_cast<rmw_context_impl_t *>(context_data);

  // Copy options
  rmw_ret_t rmw_ret = rmw_init_options_copy(options, &context->options);
  if (rmw_ret != RMW_RET_OK) {
    // Clean up on failure. Use the shared teardown so the discovery listener
    // thread is stopped and joined (skipping it would std::terminate when the
    // Context's joinable thread is destroyed) and the participant's contained
    // entities are deleted before the participant itself (otherwise it is
    // orphaned with PRECONDITION_NOT_MET, leaking sockets/eventfds/epoll).
    rmw_int2dds_cpp::release_context_resources(context_data);
    delete context_data;
    context->impl = nullptr;
    return rmw_ret;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  if (context->implementation_identifier == nullptr) {
    RMW_SET_ERROR_MSG("context is not initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (context->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("context not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * context_data = reinterpret_cast<rmw_int2dds_cpp::ContextData *>(context->impl);
  if (context_data == nullptr) {
    RMW_SET_ERROR_MSG("context not initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  context_data->is_shutdown = true;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  if (context->implementation_identifier == nullptr) {
    RMW_SET_ERROR_MSG("context is not initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (context->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("context not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * context_data = reinterpret_cast<rmw_int2dds_cpp::ContextData *>(context->impl);
  if (context_data == nullptr) {
    RMW_SET_ERROR_MSG("context not initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (!context_data->is_shutdown) {
    RMW_SET_ERROR_MSG("context not shut down");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Release whatever is still held. rmw_destroy_node may already have done this
  // when the last node went away, in which case every pointer is null and this
  // is a no-op. Entities still attached to the participant (a node's built-in
  // parameter services / rosout publisher) are deleted first: otherwise
  // int2dds_delete_participant refuses with PRECONDITION_NOT_MET and orphans the
  // participant, leaking its sockets, eventfds and epoll instances. A context
  // reclaimed by the client library's garbage collector (rather than an explicit
  // destroy) can reach here with those entities still attached.
  rmw_int2dds_cpp::release_context_resources(context_data);

  delete context_data;

  const rmw_ret_t options_fini_ret = rmw_init_options_fini(&context->options);
  (void)options_fini_ret;

  *context = rmw_get_zero_initialized_context();

  return RMW_RET_OK;
}

}  // extern "C"
