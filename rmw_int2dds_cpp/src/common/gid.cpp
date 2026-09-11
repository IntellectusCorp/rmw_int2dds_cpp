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
#include <cstring>
#include <random>

#ifdef _WIN32
#include <process.h>
#define getpid _getpid
#else
#include <unistd.h>
#endif

#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rmw/ret_types.h"
#include "rmw/error_handling.h"
#include "rmw_int2dds_cpp/identifier.hpp"

namespace rmw_int2dds_cpp
{

// Entity type constants
constexpr uint8_t GID_TYPE_PARTICIPANT = 0;
constexpr uint8_t GID_TYPE_PUBLISHER = 1;
constexpr uint8_t GID_TYPE_SUBSCRIPTION = 2;
constexpr uint8_t GID_TYPE_SERVICE = 3;
constexpr uint8_t GID_TYPE_CLIENT = 4;

// Prefix to identify int2dds GIDs
constexpr uint8_t GID_PREFIX[4] = {'I', 'N', 'T', '2'};

// Global counter for unique GID generation
static std::atomic<uint32_t> g_gid_counter{0};

rmw_gid_t generate_gid(uint8_t entity_type)
{
  rmw_gid_t gid;
  gid.implementation_identifier = implementation_identifier;

  // Clear the GID data
  std::memset(gid.data, 0, RMW_GID_STORAGE_SIZE);

  // Layout: [4B prefix][4B PID][4B counter][4B type + padding]
  // Bytes 0-3: Prefix "INT2"
  std::memcpy(&gid.data[0], GID_PREFIX, 4);

  // Bytes 4-7: per-process identity. A random value generated once per process
  // (not the raw PID) so two processes on different hosts do not collide when
  // PIDs happen to match -- the RPC correlation key (bytes 4..11) then stays
  // unique across hosts too. Wire size is unchanged (still 4 bytes).
  static const uint32_t process_id = []() {
      std::random_device rd;
      return static_cast<uint32_t>(rd());
    } ();
  std::memcpy(&gid.data[4], &process_id, sizeof(process_id));

  // Bytes 8-11: Monotonic counter
  uint32_t counter = g_gid_counter.fetch_add(1);
  std::memcpy(&gid.data[8], &counter, sizeof(counter));

  // Byte 12: Entity type
  gid.data[12] = entity_type;

  // Bytes 13-15: Reserved (zeroed)

  return gid;
}

rmw_gid_t generate_publisher_gid()
{
  return generate_gid(GID_TYPE_PUBLISHER);
}

rmw_gid_t generate_subscription_gid()
{
  return generate_gid(GID_TYPE_SUBSCRIPTION);
}

rmw_gid_t generate_service_gid()
{
  return generate_gid(GID_TYPE_SERVICE);
}

rmw_gid_t generate_client_gid()
{
  return generate_gid(GID_TYPE_CLIENT);
}

rmw_gid_t generate_participant_gid()
{
  return generate_gid(GID_TYPE_PARTICIPANT);
}

bool gids_equal(const rmw_gid_t & gid1, const rmw_gid_t & gid2)
{
  return std::memcmp(gid1.data, gid2.data, RMW_GID_STORAGE_SIZE) == 0;
}

void copy_gid(rmw_gid_t & dest, const rmw_gid_t & src)
{
  dest.implementation_identifier = src.implementation_identifier;
  std::memcpy(dest.data, src.data, RMW_GID_STORAGE_SIZE);
}

}  // namespace rmw_int2dds_cpp

extern "C"
{

rmw_ret_t
rmw_compare_gids_equal(const rmw_gid_t * gid1, const rmw_gid_t * gid2, bool * result)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);

  if (gid1->implementation_identifier != rmw_int2dds_cpp::implementation_identifier ||
    gid2->implementation_identifier != rmw_int2dds_cpp::implementation_identifier)
  {
    RMW_SET_ERROR_MSG("gid not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  *result = (std::memcmp(gid1->data, gid2->data, RMW_GID_STORAGE_SIZE) == 0);
  return RMW_RET_OK;
}

}  // extern "C"
