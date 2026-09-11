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

#include "waitset_registry.hpp"  // NOLINT(build/include)

#include <algorithm>
#include <mutex>
#include <new>
#include <utility>
#include <vector>

#include "int2dds-ffi.h"  // NOLINT(build/include): vendored FFI header

namespace rmw_int2dds_cpp
{

namespace
{
std::mutex & registry_mutex()
{
  static std::mutex * m = new std::mutex();  // leaked on purpose: valid during static destruction
  return *m;
}
std::vector<WaitSetData *> & wait_sets()
{
  static std::vector<WaitSetData *> * v = new std::vector<WaitSetData *>();
  return *v;
}
std::vector<GuardConditionData *> & retired_guards()
{
  static std::vector<GuardConditionData *> * v = new std::vector<GuardConditionData *>();
  return *v;
}
}  // namespace

bool
waitset_registry_add(WaitSetData * ws_data)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  try {
    wait_sets().push_back(ws_data);
  } catch (const std::bad_alloc &) {
    return false;  // report insert failure so the caller aborts wait-set creation
  }
  return true;
}

void
waitset_registry_remove(WaitSetData * ws_data)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  wait_sets().erase(
    std::remove(wait_sets().begin(), wait_sets().end(), ws_data), wait_sets().end());
}

void
waitset_registry_retire_guard(GuardConditionData * gc_data)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  retired_guards().push_back(gc_data);
}

void
waitset_registry_clean_caches()
{
  std::lock_guard<std::mutex> lock(registry_mutex());

  // Reclaiming needs every wait set lock held at once, since rmw_wait takes
  // the same lock before it sets inuse. Most calls have nothing to reclaim.
  const bool reclaiming = !retired_guards().empty();
  std::vector<std::unique_lock<std::mutex>> held;
  if (reclaiming) {
    held.reserve(wait_sets().size());
  }

  bool any_inuse = false;
  for (auto * ws_data : wait_sets()) {
    std::unique_lock<std::mutex> ws_lock(ws_data->lock);
    // cache_busy never spans the blocking FFI wait, so this wait is short.
    ws_data->cache_cv.wait(ws_lock, [ws_data] {return !ws_data->cache_busy;});
    if (!ws_data->inuse) {
      waitset_detach_all(ws_data);
    } else {
      // Blocked in the FFI wait: its attachments cannot be touched from here,
      // so the handles it holds have to stay valid.
      any_inuse = true;
    }
    if (reclaiming) {
      held.push_back(std::move(ws_lock));
    }
  }

  if (!reclaiming || any_inuse) {
    return;
  }

  // Nothing is inside rmw_wait and every wait set was just detached, so no
  // thread can still reach these records.
  for (auto * gc_data : retired_guards()) {
    if (gc_data->owns_guard_condition && gc_data->guard_condition != nullptr) {
      int2dds_guardcondition_delete(gc_data->guard_condition);
    }
    delete gc_data;
  }
  retired_guards().clear();
}

void
waitset_detach_all(WaitSetData * ws_data)
{
  for (const auto & entry : ws_data->attached_conditions) {
    int2dds_waitset_detach_statuscondition(ws_data->waitset, entry.first);
  }
  for (const auto & entry : ws_data->attached_guards) {
    int2dds_waitset_detach_guardcondition(ws_data->waitset, entry.first);
  }
  ws_data->attached_conditions.clear();
  ws_data->attached_guards.clear();

  ws_data->cached_subscriptions.clear();
  ws_data->cached_guard_conditions.clear();
  ws_data->cached_services.clear();
  ws_data->cached_clients.clear();
  ws_data->cached_events.clear();
}

}  // namespace rmw_int2dds_cpp
