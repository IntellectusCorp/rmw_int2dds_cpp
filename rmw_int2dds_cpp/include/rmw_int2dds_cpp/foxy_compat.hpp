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

#ifndef RMW_INT2DDS_CPP__FOXY_COMPAT_HPP_
#define RMW_INT2DDS_CPP__FOXY_COMPAT_HPP_

// Shims for rmw definitions that the humble branch relies on but that only
// arrived after Foxy. Each one defers to the real header when it exists.

#include <cstddef>

#include "rmw/types.h"

// rmw/time.h (RMW_DURATION_*) arrived in Galactic. Foxy writes "unset" as
// {0, 0} and reports DDS infinity as {INT64_MAX / 1e9, INT64_MAX % 1e9}, which
// are exactly the values Galactic+ gave these macros.
#if __has_include("rmw/time.h")
#include "rmw/time.h"
#else
#ifndef RMW_DURATION_INFINITE
#define RMW_DURATION_INFINITE {9223372036LL, 854775807LL}
#endif
#ifndef RMW_DURATION_UNSPECIFIED
#define RMW_DURATION_UNSPECIFIED {0LL, 0LL}
#endif
#endif

// rmw/event_callback_type.h arrived in Humble with the events executor. Foxy
// has no rmw API that installs such a callback, so nothing registers one on
// this branch; the type only keeps the shared listener plumbing compiling. It is
// declared inside rmw_int2dds_cpp so this installed header does not add a global
// rmw_* name that Foxy itself does not define.
#if __has_include("rmw/event_callback_type.h")
#include "rmw/event_callback_type.h"
#else
namespace rmw_int2dds_cpp
{
typedef void (* rmw_event_callback_t)(const void * user_data, size_t number_of_events);
}  // namespace rmw_int2dds_cpp
#endif

#endif  // RMW_INT2DDS_CPP__FOXY_COMPAT_HPP_
