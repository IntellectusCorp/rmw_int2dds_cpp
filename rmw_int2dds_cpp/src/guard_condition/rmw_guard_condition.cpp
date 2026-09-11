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

#include "int2dds-ffi.h"  // NOLINT(build/include)
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "../wait/waitset_registry.hpp"  // NOLINT(build/include)

extern "C"
{

rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);

  if (context->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("context not from this implementation");
    return nullptr;
  }

  // Create guard condition data
  auto * gc_data = new (std::nothrow) rmw_int2dds_cpp::GuardConditionData();
  if (gc_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate guard condition data");
    return nullptr;
  }

  // Create DDS guard condition
  Int2DdsRet ret = int2dds_guardcondition_new(&gc_data->guard_condition);
  if (ret != INT2DDS_RET_OK) {
    delete gc_data;
    RMW_SET_ERROR_MSG("failed to create DDS guard condition");
    return nullptr;
  }

  // Allocate RMW guard condition
  rmw_guard_condition_t * guard_condition = rmw_guard_condition_allocate();
  if (guard_condition == nullptr) {
    int2dds_guardcondition_delete(gc_data->guard_condition);
    delete gc_data;
    RMW_SET_ERROR_MSG("failed to allocate guard condition");
    return nullptr;
  }

  guard_condition->implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  guard_condition->data = gc_data;
  guard_condition->context = context;

  return guard_condition;
}

rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);

  if (guard_condition->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("guard condition not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * gc_data = static_cast<rmw_int2dds_cpp::GuardConditionData *>(guard_condition->data);
  if (gc_data != nullptr) {
    // Clean regardless of handle ownership: gc_data itself is what wait set
    // caches key on.
    rmw_int2dds_cpp::waitset_registry_clean_caches();
    // rcl hands rmw_wait this pointer and rclcpp may destroy it mid-scan, so
    // the registry frees it once no wait set can reach it.
    rmw_int2dds_cpp::waitset_registry_retire_guard(gc_data);
  }

  rmw_guard_condition_free(guard_condition);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);

  if (guard_condition->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("guard condition not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * gc_data = static_cast<rmw_int2dds_cpp::GuardConditionData *>(guard_condition->data);
  if (gc_data == nullptr || gc_data->guard_condition == nullptr) {
    RMW_SET_ERROR_MSG("guard condition data is null");
    return RMW_RET_ERROR;
  }

  Int2DdsRet ret = int2dds_guardcondition_set_trigger_value(gc_data->guard_condition, true);
  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to trigger guard condition");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

}  // extern "C"
