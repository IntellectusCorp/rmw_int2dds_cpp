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

#include <tuple>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

#include "rmw/rmw.h"
#if __has_include("rmw/dynamic_message_type_support.h")
#include "rmw/dynamic_message_type_support.h"
#define RMW_INT2DDS_HAS_DYNAMIC_MESSAGE_TYPE_SUPPORT 1
#endif
#include "rmw/error_handling.h"
#include "rmw/types.h"

#include "rcutils/time.h"

#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "int2dds-ffi.h"  // NOLINT(build/include): vendored FFI header
#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/types.hpp"
#include "rmw_int2dds_cpp/cdr_serializer.hpp"
#include "../common/take_with_info.hpp"  // NOLINT(build/include)

// Held per subscription, so this multiplies by reader count -- size it for the common
// case. Larger payloads grow the buffer on demand; get_data reports the size it needs.
static constexpr size_t DEFAULT_RECEIVE_BUFFER_SIZE = 64 * 1024;

namespace
{

struct SerializedLoanGuard
{
  Int2DdsSerializedLoan * loan{nullptr};

  ~SerializedLoanGuard()
  {
    if (loan != nullptr) {
      (void)int2dds_datareader_return_serialized_loan(loan);
    }
  }

  SerializedLoanGuard() = default;
  SerializedLoanGuard(const SerializedLoanGuard &) = delete;
  SerializedLoanGuard & operator=(const SerializedLoanGuard &) = delete;
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

struct TakeProfileCounters
{
  std::atomic<uint64_t> count{0};
  std::atomic<uint64_t> take_us{0};
  std::atomic<uint64_t> deserialize_us{0};
  std::atomic<uint64_t> info_us{0};
  std::atomic<uint64_t> total_us{0};
};

TakeProfileCounters g_take_profile;

void
record_take_profile(
  uint64_t take_us,
  uint64_t deserialize_us,
  uint64_t info_us,
  uint64_t total_us)
{
  const uint64_t count = g_take_profile.count.fetch_add(1, std::memory_order_relaxed) + 1;
  g_take_profile.take_us.fetch_add(take_us, std::memory_order_relaxed);
  g_take_profile.deserialize_us.fetch_add(deserialize_us, std::memory_order_relaxed);
  g_take_profile.info_us.fetch_add(info_us, std::memory_order_relaxed);
  g_take_profile.total_us.fetch_add(total_us, std::memory_order_relaxed);

  if (count % 50 == 0) {
    const double d = static_cast<double>(count);
    std::fprintf(
      stderr,
      "RMW_INT2DDS_TAKE_PROFILE count=%lu total_avg_us=%.3f "
      "take_avg_us=%.3f deserialize_avg_us=%.3f info_avg_us=%.3f\n",
      count,
      g_take_profile.total_us.load(std::memory_order_relaxed) / d,
      g_take_profile.take_us.load(std::memory_order_relaxed) / d,
      g_take_profile.deserialize_us.load(std::memory_order_relaxed) / d,
      g_take_profile.info_us.load(std::memory_order_relaxed) / d);
  }
}

/// Fill rmw message info from a DDS SampleInfo (info may be null when the
/// sample was taken through a path that cannot provide it).
void
fill_message_info_from_sample(
  rmw_message_info_t * message_info,
  const Int2DdsSampleInfo * info,
  uint64_t reception_sequence)
{
  message_info->source_timestamp =
    info != nullptr ? rmw_int2dds_cpp::sample_info_source_timestamp_ns(*info) : 0;
  std::ignore = rcutils_system_time_now(&message_info->received_timestamp);
  // Foxy's rmw_message_info_t has no publication/reception sequence numbers
  // (Humble); the counter is still kept so the take paths match the humble branch.
  static_cast<void>(reception_sequence);
  // int2dds has no intra-process transport and its SampleInfo carries no
  // intra-process flag, so a received sample is never intra-process. This is the
  // single point to revisit if int2dds ever gains an intra-process delivery path.
  message_info->from_intra_process = false;
  memset(message_info->publisher_gid.data, 0, RMW_GID_STORAGE_SIZE);
  message_info->publisher_gid.implementation_identifier =
    rmw_int2dds_cpp::implementation_identifier;
  if (info != nullptr) {
    static_assert(sizeof(info->publication_handle) <= RMW_GID_STORAGE_SIZE, "gid size");
    memcpy(
      message_info->publisher_gid.data, info->publication_handle,
      sizeof(info->publication_handle));
  }
}

/// True when publication_handle names a publisher created on the same node as
/// this subscription. Local publisher gids are the writer endpoint GUIDs that the
/// take path also reports as publication_handle (they compare equal byte-for-byte),
/// so an exact match identifies a same-node publication. Lets
/// ignore_local_publications drop only local samples instead of every sample.
bool
is_local_publication(
  const rmw_int2dds_cpp::SubscriptionData * sub_data,
  const Int2DdsSampleInfo & info)
{
  rmw_int2dds_cpp::NodeData * node_data = sub_data->node_data;
  if (node_data == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(node_data->entities_mutex);
  for (const auto & pub_gid : node_data->publishers) {
    if (memcmp(pub_gid.data, info.publication_handle, sizeof(info.publication_handle)) == 0) {
      return true;
    }
  }
  return false;
}

/// Deserialize a full serialized CDR sample (including its 4-byte encapsulation
/// header) into a typed ROS message using the subscription's introspection
/// typesupport. Shared by the single take and the batch take_sequence path.
rmw_ret_t
deserialize_cdr_into_message(
  const rmw_int2dds_cpp::SubscriptionData * sub_data,
  const uint8_t * serialized_data,
  size_t data_size,
  void * ros_message)
{
  if (data_size < 4 || serialized_data == nullptr) {
    RMW_SET_ERROR_MSG("received data too small for CDR header");
    return RMW_RET_ERROR;
  }
  // Create deserializer starting after the CDR encapsulation header (4 bytes)
  rmw_int2dds_cpp::CdrDeserializer deserializer(serialized_data + 4, data_size - 4);
  bool deserialize_success = false;
  if (sub_data->type_support->typesupport_identifier ==
    rosidl_typesupport_introspection_c__identifier)
  {
    const auto * members = static_cast<
      const rosidl_typesupport_introspection_c__MessageMembers *>(
      sub_data->type_support->data);
    deserialize_success = deserializer.deserialize_message_c(ros_message, members);
  } else if (sub_data->type_support->typesupport_identifier ==  // NOLINT(readability/braces)
    rosidl_typesupport_introspection_cpp::typesupport_identifier)
  {
    const auto * members = static_cast<
      const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      sub_data->type_support->data);
    deserialize_success = deserializer.deserialize_message_cpp(ros_message, members);
  } else {
    RMW_SET_ERROR_MSG("unknown type support identifier");
    return RMW_RET_ERROR;
  }
  if (!deserialize_success) {
    RMW_SET_ERROR_MSG("failed to deserialize message");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

/// Helper function to take a single message from a subscription
/// Returns RMW_RET_OK if data was taken, RMW_RET_ERROR on error
/// Sets *taken to true if data was actually received
rmw_ret_t
take_message_internal(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  *taken = false;

  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscription->data);
  if (sub_data == nullptr || sub_data->datareader == nullptr) {
    RMW_SET_ERROR_MSG("subscription data is null");
    return RMW_RET_ERROR;
  }
  const bool profile = profile_enabled();
  const uint64_t total_t0 = profile ? now_us() : 0;
  uint64_t take_us = 0;
  uint64_t deserialize_us = 0;
  uint64_t info_us = 0;

  size_t data_size = 0;
  bool valid_data = false;
  const uint8_t * serialized_data = nullptr;
  SerializedLoanGuard loan_guard;
  Int2DdsSampleInfo sample_info;
  bool have_sample_info = false;
  std::unique_lock<std::mutex> buffer_lock;

  // Take data from DDS. When the caller wants message info -- or when
  // ignore_local_publications must inspect each sample's origin -- go through the
  // batch API (the only serialized take that exposes SampleInfo); otherwise keep
  // the loaned path, which avoids copying into an intermediate buffer.
  const bool want_sample_info =
    message_info != nullptr || subscription->options.ignore_local_publications;
  const uint64_t take_t0 = profile ? now_us() : 0;
  Int2DdsRet ret;
  if (want_sample_info) {
    buffer_lock = std::unique_lock<std::mutex>(sub_data->take_buffer_mutex);
    if (sub_data->take_buffer.size() < DEFAULT_RECEIVE_BUFFER_SIZE) {
      sub_data->take_buffer.resize(DEFAULT_RECEIVE_BUFFER_SIZE);
    }
    ret = rmw_int2dds_cpp::take_one_serialized_with_info(
      sub_data->datareader,
      sub_data->take_buffer,
      &data_size,
      &valid_data,
      &sample_info);
    serialized_data = sub_data->take_buffer.data();
    have_sample_info = ret == INT2DDS_RET_OK;
  } else {
    ret = int2dds_datareader_take_serialized_loaned(
      sub_data->datareader,
      &serialized_data,
      &data_size,
      &valid_data,
      &loan_guard.loan);
  }
  if (profile) {
    take_us = now_us() - take_t0;
  }

  if (ret == INT2DDS_RET_NO_DATA) {
    // No data available - not an error
    return RMW_RET_OK;
  }

  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to take data from DDS");
    return RMW_RET_ERROR;
  }

  if (!valid_data) {
    // Data is not valid (e.g., disposed instance)
    return RMW_RET_OK;
  }

  if (subscription->options.ignore_local_publications && have_sample_info &&
    is_local_publication(sub_data, sample_info))
  {
    // Honour ignore_local_publications by dropping only samples from this node's
    // own publishers; genuinely remote data is still delivered. The sample was
    // consumed, so taken stays false and the caller sees no message here.
    return RMW_RET_OK;
  }

  // Deserialize the serialized sample into the typed message (shared helper).
  const uint64_t deserialize_t0 = profile ? now_us() : 0;
  const rmw_ret_t deser_ret = deserialize_cdr_into_message(
    sub_data, serialized_data, data_size, ros_message);
  if (profile) {
    deserialize_us = now_us() - deserialize_t0;
  }
  if (deser_ret != RMW_RET_OK) {
    return deser_ret;
  }

  *taken = true;

  // Update reception sequence
  sub_data->reception_sequence.fetch_add(1);

  // Fill message info if requested
  const uint64_t info_t0 = profile ? now_us() : 0;
  if (message_info != nullptr) {
    fill_message_info_from_sample(
      message_info,
      have_sample_info ? &sample_info : nullptr,
      sub_data->reception_sequence.load());
  }
  if (profile) {
    info_us = now_us() - info_t0;
    record_take_profile(take_us, deserialize_us, info_us, now_us() - total_t0);
  }

  return RMW_RET_OK;
}

/// Helper function to take a serialized message (no deserialization)
rmw_ret_t
take_serialized_message_internal(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  *taken = false;

  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscription->data);
  if (sub_data == nullptr || sub_data->datareader == nullptr) {
    RMW_SET_ERROR_MSG("subscription data is null");
    return RMW_RET_ERROR;
  }

  std::lock_guard<std::mutex> buffer_lock(sub_data->take_buffer_mutex);
  if (sub_data->take_buffer.size() < DEFAULT_RECEIVE_BUFFER_SIZE) {
    sub_data->take_buffer.resize(DEFAULT_RECEIVE_BUFFER_SIZE);
  }
  size_t data_size = 0;
  bool valid_data = false;
  Int2DdsSampleInfo sample_info;

  // Take through the batch API so the SampleInfo travels with the payload.
  Int2DdsRet ret = rmw_int2dds_cpp::take_one_serialized_with_info(
    sub_data->datareader,
    sub_data->take_buffer,
    &data_size,
    &valid_data,
    &sample_info);

  if (ret == INT2DDS_RET_NO_DATA) {
    return RMW_RET_OK;
  }

  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to take data from DDS");
    return RMW_RET_ERROR;
  }

  if (!valid_data) {
    return RMW_RET_OK;
  }

  if (subscription->options.ignore_local_publications &&
    is_local_publication(sub_data, sample_info))
  {
    // See take_message_internal: drop only this node's own publications.
    return RMW_RET_OK;
  }

  // Resize serialized message buffer if needed
  if (serialized_message->buffer_capacity < data_size) {
    rmw_ret_t resize_ret = rmw_serialized_message_resize(serialized_message, data_size);
    if (resize_ret != RMW_RET_OK) {
      RMW_SET_ERROR_MSG("failed to resize serialized message");
      return RMW_RET_ERROR;
    }
  }

  // Copy data to serialized message
  memcpy(serialized_message->buffer, sub_data->take_buffer.data(), data_size);
  serialized_message->buffer_length = data_size;

  *taken = true;

  // Update reception sequence
  sub_data->reception_sequence.fetch_add(1);

  // Fill message info if requested
  if (message_info != nullptr) {
    fill_message_info_from_sample(
      message_info, &sample_info, sub_data->reception_sequence.load());
  }

  return RMW_RET_OK;
}

}  // namespace

extern "C"
{
rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  return take_message_internal(subscription, ros_message, taken, nullptr);
}

rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  return take_message_internal(subscription, ros_message, taken, message_info);
}

rmw_ret_t
rmw_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (count == 0) {
    RMW_SET_ERROR_MSG("count cannot be 0");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (message_sequence->capacity < count) {
    RMW_SET_ERROR_MSG("message sequence capacity is less than count");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (message_info_sequence->capacity < count) {
    RMW_SET_ERROR_MSG("message info sequence capacity is less than count");
    return RMW_RET_INVALID_ARGUMENT;
  }

  *taken = 0;

  auto * sub_data = static_cast<rmw_int2dds_cpp::SubscriptionData *>(subscription->data);
  if (sub_data == nullptr || sub_data->datareader == nullptr) {
    RMW_SET_ERROR_MSG("subscription data is null");
    return RMW_RET_ERROR;
  }

  // A single DDS take for up to `count` samples -- a true batch instead of `count`
  // separate single takes. Every returned sample is consumed by this one call, so a
  // filtered-out sample (invalid, or same-node when ignore_local_publications) is
  // skipped in place rather than truncating the rest of the batch.
  std::lock_guard<std::mutex> buffer_lock(sub_data->take_buffer_mutex);
  if (sub_data->take_buffer.size() < DEFAULT_RECEIVE_BUFFER_SIZE) {
    sub_data->take_buffer.resize(DEFAULT_RECEIVE_BUFFER_SIZE);
  }

  Int2DdsSampleSeq * seq = nullptr;
  Int2DdsRet ret = int2dds_datareader_take_serialized_batch(
    sub_data->datareader, static_cast<int32_t>(count), &seq);
  if (ret == INT2DDS_RET_NO_DATA || seq == nullptr) {
    if (seq != nullptr) {
      int2dds_sample_seq_delete(seq);
    }
    message_sequence->size = 0;
    message_info_sequence->size = 0;
    return RMW_RET_OK;
  }
  if (ret != INT2DDS_RET_OK) {
    int2dds_sample_seq_delete(seq);
    RMW_SET_ERROR_MSG("failed to batch-take from DDS");
    return RMW_RET_ERROR;
  }

  const uintptr_t available = int2dds_sample_seq_length(seq);
  for (uintptr_t i = 0; i < available && *taken < count; ++i) {
    Int2DdsSampleInfo sample_info;
    std::memset(&sample_info, 0, sizeof(sample_info));
    if (int2dds_sample_seq_get_info(seq, i, &sample_info) != INT2DDS_RET_OK) {
      continue;
    }
    if (!sample_info.valid_data) {
      continue;  // disposed / no-data instance
    }
    if (subscription->options.ignore_local_publications &&
      is_local_publication(sub_data, sample_info))
    {
      continue;  // drop only this node's own publications (A2)
    }

    uintptr_t actual_size = 0;
    Int2DdsRet dret = int2dds_sample_seq_get_data(
      seq, i, sub_data->take_buffer.data(), sub_data->take_buffer.size(), &actual_size);
    if (dret != INT2DDS_RET_OK && actual_size > sub_data->take_buffer.size()) {
      sub_data->take_buffer.resize(actual_size + 256);
      dret = int2dds_sample_seq_get_data(
        seq, i, sub_data->take_buffer.data(), sub_data->take_buffer.size(), &actual_size);
    }
    if (dret != INT2DDS_RET_OK) {
      int2dds_sample_seq_delete(seq);
      RMW_SET_ERROR_MSG("failed to read batch sample data");
      return RMW_RET_ERROR;
    }

    const rmw_ret_t deser_ret = deserialize_cdr_into_message(
      sub_data, sub_data->take_buffer.data(), actual_size,
      message_sequence->data[*taken]);
    if (deser_ret != RMW_RET_OK) {
      int2dds_sample_seq_delete(seq);
      return deser_ret;
    }

    sub_data->reception_sequence.fetch_add(1);
    fill_message_info_from_sample(
      &message_info_sequence->data[*taken],
      &sample_info,
      sub_data->reception_sequence.load());
    (*taken)++;
  }
  int2dds_sample_seq_delete(seq);

  message_sequence->size = *taken;
  message_info_sequence->size = *taken;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  return take_serialized_message_internal(subscription, serialized_message, taken, nullptr);
}

rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("subscription not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  return take_serialized_message_internal(subscription, serialized_message, taken, message_info);
}

rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)loaned_message;
  (void)taken;
  (void)allocation;
  // Loaned messages are not supported by int2dds
  RMW_SET_ERROR_MSG("rmw_take_loaned_message is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)loaned_message;
  (void)taken;
  (void)message_info;
  (void)allocation;
  // Loaned messages are not supported by int2dds
  RMW_SET_ERROR_MSG("rmw_take_loaned_message_with_info is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  (void)subscription;
  (void)loaned_message;
  // Loaned messages are not supported by int2dds
  RMW_SET_ERROR_MSG("rmw_return_loaned_message_from_subscription is not supported");
  return RMW_RET_UNSUPPORTED;
}

#ifdef RMW_INT2DDS_HAS_DYNAMIC_MESSAGE_TYPE_SUPPORT
rmw_ret_t
rmw_take_dynamic_message(
  const rmw_subscription_t * subscription,
  rosidl_dynamic_typesupport_dynamic_data_t * dynamic_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)dynamic_message;
  (void)taken;
  (void)allocation;
  // Dynamic message type support is not provided by int2dds
  RMW_SET_ERROR_MSG("rmw_take_dynamic_message is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_dynamic_message_with_info(
  const rmw_subscription_t * subscription,
  rosidl_dynamic_typesupport_dynamic_data_t * dynamic_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)dynamic_message;
  (void)taken;
  (void)message_info;
  (void)allocation;
  // Dynamic message type support is not provided by int2dds
  RMW_SET_ERROR_MSG("rmw_take_dynamic_message_with_info is not supported by rmw_int2dds_cpp");
  return RMW_RET_UNSUPPORTED;
}
#endif
}  // extern "C"
