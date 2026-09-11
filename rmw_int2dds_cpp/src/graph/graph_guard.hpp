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

#ifndef GRAPH__GRAPH_GUARD_HPP_
#define GRAPH__GRAPH_GUARD_HPP_

#include "int2dds-ffi.h"  // NOLINT(build/include)
#include "rmw_int2dds_cpp/types.hpp"

namespace rmw_int2dds_cpp
{

inline void trigger_graph_guard_condition(ContextData * context_data)
{
  if (context_data == nullptr || context_data->graph_guard_condition == nullptr) {
    return;
  }

  (void)int2dds_guardcondition_set_trigger_value(
    context_data->graph_guard_condition,
    true);
}

}  // namespace rmw_int2dds_cpp

#endif  // GRAPH__GRAPH_GUARD_HPP_
