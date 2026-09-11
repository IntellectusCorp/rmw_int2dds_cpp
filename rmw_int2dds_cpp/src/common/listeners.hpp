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

#ifndef COMMON__LISTENERS_HPP_
#define COMMON__LISTENERS_HPP_

#include <mutex>

#include "rmw_int2dds_cpp/foxy_compat.hpp"
#include "rmw_int2dds_cpp/types.hpp"

namespace rmw_int2dds_cpp
{

/// Re-register the int2dds listener for an entity with a mask derived from
/// which user callbacks are currently registered. A DDS listener consumes the
/// status change it handles, which would starve wait-set delivery, so a status
/// bit is enabled only while a user callback (events-executor path) wants it;
/// entities driven purely by wait sets keep an empty mask.
void refresh_publisher_listener(PublisherData * pub_data);
void refresh_subscription_listener(SubscriptionData * sub_data);
void refresh_service_listener(ServiceData * srv_data);
void refresh_client_listener(ClientData * cli_data);

/// Store a user callback into a slot and flush the backlog (rmw semantics:
/// occurrences that fired before registration are delivered immediately).
/// Callers must follow up with the matching refresh_*_listener().
void set_callback_slot(
  std::mutex & mutex,
  CallbackSlot & slot,
  rmw_event_callback_t callback,
  const void * user_data);

}  // namespace rmw_int2dds_cpp

#endif  // COMMON__LISTENERS_HPP_
