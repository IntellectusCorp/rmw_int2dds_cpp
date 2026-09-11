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

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "int2dds-ffi.h"  // NOLINT(build/include)
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "rmw_int2dds_cpp/cdr_serializer.hpp"

namespace
{

struct SerializedWriteLoanGuard
{
  Int2DdsSerializedWriteLoan * loan{nullptr};

  ~SerializedWriteLoanGuard()
  {
    if (loan != nullptr) {
      (void)int2dds_datawriter_abort_serialized_write(loan);
    }
  }

  SerializedWriteLoanGuard() = default;
  SerializedWriteLoanGuard(const SerializedWriteLoanGuard &) = delete;
  SerializedWriteLoanGuard & operator=(const SerializedWriteLoanGuard &) = delete;

  Int2DdsSerializedWriteLoan * release()
  {
    Int2DdsSerializedWriteLoan * released = loan;
    loan = nullptr;
    return released;
  }
};

uint64_t
now_us()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool
profile_enabled()
{
  return std::getenv("RMW_INT2DDS_PROFILE") != nullptr;
}

struct PublishProfileCounters
{
  std::atomic<uint64_t> count{0};
  std::atomic<uint64_t> prepared_count{0};
  std::atomic<uint64_t> fallback_count{0};
  std::atomic<uint64_t> prepare_us{0};
  std::atomic<uint64_t> serialize_us{0};
  std::atomic<uint64_t> commit_us{0};
  std::atomic<uint64_t> fallback_write_us{0};
  std::atomic<uint64_t> total_us{0};
};

PublishProfileCounters g_publish_profile;

void
record_publish_profile(
  bool prepared,
  uint64_t prepare_us,
  uint64_t serialize_us,
  uint64_t commit_us,
  uint64_t fallback_write_us,
  uint64_t total_us)
{
  const uint64_t count = g_publish_profile.count.fetch_add(1, std::memory_order_relaxed) + 1;
  if (prepared) {
    g_publish_profile.prepared_count.fetch_add(1, std::memory_order_relaxed);
  } else {
    g_publish_profile.fallback_count.fetch_add(1, std::memory_order_relaxed);
  }
  g_publish_profile.prepare_us.fetch_add(prepare_us, std::memory_order_relaxed);
  g_publish_profile.serialize_us.fetch_add(serialize_us, std::memory_order_relaxed);
  g_publish_profile.commit_us.fetch_add(commit_us, std::memory_order_relaxed);
  g_publish_profile.fallback_write_us.fetch_add(fallback_write_us, std::memory_order_relaxed);
  g_publish_profile.total_us.fetch_add(total_us, std::memory_order_relaxed);

  if (count % 50 == 0) {
    const double d = static_cast<double>(count);
    std::fprintf(
      stderr,
      "RMW_INT2DDS_PUBLISH_PROFILE count=%lu prepared=%lu fallback=%lu "
      "total_avg_us=%.3f prepare_avg_us=%.3f serialize_avg_us=%.3f "
      "commit_avg_us=%.3f fallback_write_avg_us=%.3f\n",
      count,
      g_publish_profile.prepared_count.load(std::memory_order_relaxed),
      g_publish_profile.fallback_count.load(std::memory_order_relaxed),
      g_publish_profile.total_us.load(std::memory_order_relaxed) / d,
      g_publish_profile.prepare_us.load(std::memory_order_relaxed) / d,
      g_publish_profile.serialize_us.load(std::memory_order_relaxed) / d,
      g_publish_profile.commit_us.load(std::memory_order_relaxed) / d,
      g_publish_profile.fallback_write_us.load(std::memory_order_relaxed) / d);
  }
}

}  // namespace

extern "C"
{

rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)allocation;

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("publisher not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(publisher->data);
  if (pub_data == nullptr || pub_data->datawriter == nullptr) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  const bool profile = profile_enabled();
  const uint64_t total_t0 = profile ? now_us() : 0;
  uint64_t prepare_us = 0;
  uint64_t serialize_us = 0;
  uint64_t commit_us = 0;
  uint64_t fallback_write_us = 0;

  auto serialize_message = [&](rmw_int2dds_cpp::CdrSerializer & serializer) -> bool {
      if (pub_data->type_support->typesupport_identifier ==
        rosidl_typesupport_introspection_c__identifier)
      {
        const auto * members =
          static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
          pub_data->type_support->data);
        return serializer.serialize_message_c(ros_message, members);
      }
      if (pub_data->type_support->typesupport_identifier ==
        rosidl_typesupport_introspection_cpp::typesupport_identifier)
      {
        const auto * members =
          static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          pub_data->type_support->data);
        return serializer.serialize_message_cpp(ros_message, members);
      }
      return false;
    };

  Int2DdsRet ret = INT2DDS_RET_ERROR;
  size_t serialized_size = 0;

  const size_t capacity_hint = pub_data->serialized_capacity_hint.load(std::memory_order_relaxed);
  if (capacity_hint > 0) {
    uint8_t * loan_buffer = nullptr;
    size_t loan_capacity = 0;
    SerializedWriteLoanGuard loan_guard;
    const uint64_t prepare_t0 = profile ? now_us() : 0;
    ret = int2dds_datawriter_prepare_serialized_write(
      pub_data->datawriter,
      capacity_hint,
      &loan_buffer,
      &loan_capacity,
      &loan_guard.loan);
    if (profile) {
      prepare_us = now_us() - prepare_t0;
    }

    if (ret == INT2DDS_RET_OK && loan_buffer != nullptr && loan_guard.loan != nullptr) {
      rmw_int2dds_cpp::CdrSerializer serializer(loan_buffer, loan_capacity);
      const uint64_t serialize_t0 = profile ? now_us() : 0;
      const bool serialization_success = serialize_message(serializer);
      if (profile) {
        serialize_us = now_us() - serialize_t0;
      }
      serialized_size = serializer.get_size();

      if (serialization_success && !serializer.has_error()) {
        const uint64_t commit_t0 = profile ? now_us() : 0;
        ret = int2dds_datawriter_commit_serialized_write(
          pub_data->datawriter,
          loan_guard.release(),
          serialized_size);
        if (profile) {
          commit_us = now_us() - commit_t0;
        }

        if (ret != INT2DDS_RET_OK) {
          RMW_SET_ERROR_MSG("failed to commit prepared serialized message");
          return RMW_RET_ERROR;
        }

        if (profile) {
          record_publish_profile(
            true,
            prepare_us,
            serialize_us,
            commit_us,
            0,
            now_us() - total_t0);
        }
        return RMW_RET_OK;
      }

      if (!serialization_success) {
        RMW_SET_ERROR_MSG("failed to serialize message");
        return RMW_RET_ERROR;
      }
    }
  }

  rmw_int2dds_cpp::CdrSerializer serializer;
  const uint64_t serialize_t0 = profile ? now_us() : 0;
  const bool serialization_success = serialize_message(serializer);
  if (profile) {
    serialize_us = now_us() - serialize_t0;
  }

  if (!serialization_success) {
    RMW_SET_ERROR_MSG("failed to serialize message");
    return RMW_RET_ERROR;
  }
  serialized_size = serializer.get_size();
  pub_data->serialized_capacity_hint.store(serialized_size, std::memory_order_relaxed);

  // Write to DDS
  const uint64_t write_t0 = profile ? now_us() : 0;
  ret = int2dds_datawriter_write_serialized(
    pub_data->datawriter,
    serializer.get_data(),
    serialized_size);
  if (profile) {
    fallback_write_us = now_us() - write_t0;
  }

  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to write message");
    return RMW_RET_ERROR;
  }

  if (profile) {
    record_publish_profile(
      false,
      prepare_us,
      serialize_us,
      0,
      fallback_write_us,
      now_us() - total_t0);
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)allocation;

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("publisher not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * pub_data = static_cast<rmw_int2dds_cpp::PublisherData *>(publisher->data);
  if (pub_data == nullptr || pub_data->datawriter == nullptr) {
    RMW_SET_ERROR_MSG("publisher data is null");
    return RMW_RET_ERROR;
  }

  // Write already serialized message directly
  Int2DdsRet ret = int2dds_datawriter_write_serialized(
    pub_data->datawriter,
    serialized_message->buffer,
    serialized_message->buffer_length);

  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to write serialized message");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)ros_message;
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_publish_loaned_message is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_init_publisher_allocation is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_fini_publisher_allocation(rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_fini_publisher_allocation is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
