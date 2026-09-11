// Copyright 2026 Int2DDS Project
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

#include "listeners.hpp"  // NOLINT(build/include)

#include <algorithm>
#include <mutex>

#include "int2dds-ffi.h"  // NOLINT(build/include)

namespace rmw_int2dds_cpp
{

namespace
{

// All trampolines run on int2dds listener threads (not the ROS executor):
// they only touch CallbackSlot fields under the entity's listener_mutex and
// either deliver the user rmw event callback or remember the occurrence for a
// later registration to flush.
void
fire_slot(std::mutex & mutex, CallbackSlot & slot)
{
  std::lock_guard<std::mutex> lock(mutex);
  if (slot.callback != nullptr) {
    slot.callback(slot.user_data, 1);
  } else {
    slot.unread++;
  }
}

extern "C" {

// --- DataReader (subscription) trampolines ---

void
on_sub_data_available(Int2DdsDataReader * /*reader*/, Int2DdsUserContext user_context)
{
  auto * sub_data = static_cast<SubscriptionData *>(user_context);
  if (sub_data != nullptr) {
    fire_slot(sub_data->listener_mutex, sub_data->new_message_slot);
  }
}

void
on_sub_matched(
  Int2DdsDataReader * /*reader*/,
  const Int2DdsSubscriptionMatchedStatus * status,
  Int2DdsUserContext user_context)
{
  auto * sub_data = static_cast<SubscriptionData *>(user_context);
  if (sub_data == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(sub_data->matched_mutex);
  sub_data->matched_changed = true;
  if (status != nullptr && status->total_count > 0) {
    sub_data->matched_total_seen =
      std::max(sub_data->matched_total_seen, static_cast<size_t>(status->total_count));
  }
  if (sub_data->matched_callback != nullptr) {
    sub_data->matched_callback(sub_data->matched_user_data, 1);
  } else {
    sub_data->matched_unread++;
  }
}

void
on_sub_requested_deadline_missed(
  Int2DdsDataReader * /*reader*/,
  const Int2DdsRequestedDeadlineMissedStatus * /*status*/,
  Int2DdsUserContext user_context)
{
  auto * sub_data = static_cast<SubscriptionData *>(user_context);
  if (sub_data != nullptr) {
    fire_slot(sub_data->listener_mutex, sub_data->requested_deadline_missed_slot);
  }
}

void
on_sub_requested_incompatible_qos(
  Int2DdsDataReader * /*reader*/,
  const Int2DdsRequestedIncompatibleQosStatus * /*status*/,
  Int2DdsUserContext user_context)
{
  auto * sub_data = static_cast<SubscriptionData *>(user_context);
  if (sub_data != nullptr) {
    fire_slot(sub_data->listener_mutex, sub_data->requested_incompatible_qos_slot);
  }
}

void
on_sub_sample_lost(
  Int2DdsDataReader * /*reader*/,
  const Int2DdsSampleLostStatus * /*status*/,
  Int2DdsUserContext user_context)
{
  auto * sub_data = static_cast<SubscriptionData *>(user_context);
  if (sub_data != nullptr) {
    fire_slot(sub_data->listener_mutex, sub_data->message_lost_slot);
  }
}

void
on_sub_liveliness_changed(
  Int2DdsDataReader * /*reader*/,
  const Int2DdsLivelinessChangedStatus * /*status*/,
  Int2DdsUserContext user_context)
{
  auto * sub_data = static_cast<SubscriptionData *>(user_context);
  if (sub_data != nullptr) {
    fire_slot(sub_data->listener_mutex, sub_data->liveliness_changed_slot);
  }
}

// --- DataWriter (publisher) trampolines ---

void
on_pub_matched(
  Int2DdsDataWriter * /*writer*/,
  const Int2DdsPublicationMatchedStatus * status,
  Int2DdsUserContext user_context)
{
  auto * pub_data = static_cast<PublisherData *>(user_context);
  if (pub_data == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(pub_data->matched_mutex);
  if (status != nullptr && status->total_count > 0) {
    pub_data->matched_total_seen =
      std::max(pub_data->matched_total_seen, static_cast<size_t>(status->total_count));
  }
  if (pub_data->matched_callback != nullptr) {
    pub_data->matched_callback(pub_data->matched_user_data, 1);
  } else {
    pub_data->matched_unread++;
  }
}

void
on_pub_offered_deadline_missed(
  Int2DdsDataWriter * /*writer*/,
  const Int2DdsOfferedDeadlineMissedStatus * /*status*/,
  Int2DdsUserContext user_context)
{
  auto * pub_data = static_cast<PublisherData *>(user_context);
  if (pub_data != nullptr) {
    fire_slot(pub_data->listener_mutex, pub_data->offered_deadline_missed_slot);
  }
}

void
on_pub_offered_incompatible_qos(
  Int2DdsDataWriter * /*writer*/,
  const Int2DdsOfferedIncompatibleQosStatus * /*status*/,
  Int2DdsUserContext user_context)
{
  auto * pub_data = static_cast<PublisherData *>(user_context);
  if (pub_data != nullptr) {
    fire_slot(pub_data->listener_mutex, pub_data->offered_incompatible_qos_slot);
  }
}

void
on_pub_liveliness_lost(
  Int2DdsDataWriter * /*writer*/,
  const Int2DdsLivelinessLostStatus * /*status*/,
  Int2DdsUserContext user_context)
{
  auto * pub_data = static_cast<PublisherData *>(user_context);
  if (pub_data != nullptr) {
    fire_slot(pub_data->listener_mutex, pub_data->liveliness_lost_slot);
  }
}

// --- Service/client request/response readers (data available only) ---

void
on_service_data_available(Int2DdsDataReader * /*reader*/, Int2DdsUserContext user_context)
{
  auto * srv_data = static_cast<ServiceData *>(user_context);
  if (srv_data != nullptr) {
    fire_slot(srv_data->listener_mutex, srv_data->new_request_slot);
  }
}

void
on_client_data_available(Int2DdsDataReader * /*reader*/, Int2DdsUserContext user_context)
{
  auto * cli_data = static_cast<ClientData *>(user_context);
  if (cli_data != nullptr) {
    fire_slot(cli_data->listener_mutex, cli_data->new_response_slot);
  }
}

}  // extern "C"

}  // namespace

void
set_callback_slot(
  std::mutex & mutex,
  CallbackSlot & slot,
  rmw_event_callback_t callback,
  const void * user_data)
{
  std::lock_guard<std::mutex> lock(mutex);
  slot.callback = callback;
  slot.user_data = user_data;
  if (callback != nullptr && slot.unread > 0) {
    callback(user_data, slot.unread);
    slot.unread = 0;
  }
}

void
refresh_publisher_listener(PublisherData * pub_data)
{
  if (pub_data == nullptr || pub_data->datawriter == nullptr) {
    return;
  }
  uint32_t mask = 0;
  {
    std::lock_guard<std::mutex> lock(pub_data->listener_mutex);
    if (pub_data->offered_deadline_missed_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_OFFERED_DEADLINE_MISSED;
    }
    if (pub_data->offered_incompatible_qos_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_OFFERED_INCOMPATIBLE_QOS;
    }
    if (pub_data->liveliness_lost_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_LIVELINESS_LOST;
    }
  }
  {
    std::lock_guard<std::mutex> lock(pub_data->matched_mutex);
    if (pub_data->matched_callback != nullptr) {
      mask |= INT2DDS_STATUS_PUBLICATION_MATCHED;
    }
  }
  Int2DdsDataWriterListener listener{};
  listener.on_publication_matched = on_pub_matched;
  listener.on_offered_deadline_missed = on_pub_offered_deadline_missed;
  listener.on_offered_incompatible_qos = on_pub_offered_incompatible_qos;
  listener.on_liveliness_lost = on_pub_liveliness_lost;
  listener.user_context = pub_data;
  (void)int2dds_datawriter_set_listener(pub_data->datawriter, &listener, mask);
}

void
refresh_subscription_listener(SubscriptionData * sub_data)
{
  if (sub_data == nullptr || sub_data->datareader == nullptr) {
    return;
  }
  uint32_t mask = 0;
  {
    std::lock_guard<std::mutex> lock(sub_data->listener_mutex);
    if (sub_data->new_message_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_DATA_AVAILABLE;
    }
    if (sub_data->requested_deadline_missed_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_REQUESTED_DEADLINE_MISSED;
    }
    if (sub_data->requested_incompatible_qos_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_REQUESTED_INCOMPATIBLE_QOS;
    }
    if (sub_data->message_lost_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_SAMPLE_LOST;
    }
    if (sub_data->liveliness_changed_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_LIVELINESS_CHANGED;
    }
  }
  {
    std::lock_guard<std::mutex> lock(sub_data->matched_mutex);
    if (sub_data->matched_callback != nullptr) {
      mask |= INT2DDS_STATUS_SUBSCRIPTION_MATCHED;
    }
  }
  Int2DdsDataReaderListener listener{};
  listener.on_data_available = on_sub_data_available;
  listener.on_subscription_matched = on_sub_matched;
  listener.on_requested_deadline_missed = on_sub_requested_deadline_missed;
  listener.on_requested_incompatible_qos = on_sub_requested_incompatible_qos;
  listener.on_sample_lost = on_sub_sample_lost;
  listener.on_liveliness_changed = on_sub_liveliness_changed;
  listener.user_context = sub_data;
  (void)int2dds_datareader_set_listener(sub_data->datareader, &listener, mask);
}

void
refresh_service_listener(ServiceData * srv_data)
{
  if (srv_data == nullptr || srv_data->request_reader == nullptr) {
    return;
  }
  uint32_t mask = 0;
  {
    std::lock_guard<std::mutex> lock(srv_data->listener_mutex);
    if (srv_data->new_request_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_DATA_AVAILABLE;
    }
  }
  Int2DdsDataReaderListener listener{};
  listener.on_data_available = on_service_data_available;
  listener.user_context = srv_data;
  (void)int2dds_datareader_set_listener(srv_data->request_reader, &listener, mask);
}

void
refresh_client_listener(ClientData * cli_data)
{
  if (cli_data == nullptr || cli_data->response_reader == nullptr) {
    return;
  }
  uint32_t mask = 0;
  {
    std::lock_guard<std::mutex> lock(cli_data->listener_mutex);
    if (cli_data->new_response_slot.callback != nullptr) {
      mask |= INT2DDS_STATUS_DATA_AVAILABLE;
    }
  }
  Int2DdsDataReaderListener listener{};
  listener.on_data_available = on_client_data_available;
  listener.user_context = cli_data;
  (void)int2dds_datareader_set_listener(cli_data->response_reader, &listener, mask);
}

}  // namespace rmw_int2dds_cpp
