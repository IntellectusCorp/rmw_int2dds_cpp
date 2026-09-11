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

#ifndef COMMON__TAKE_WITH_INFO_HPP_
#define COMMON__TAKE_WITH_INFO_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "int2dds-ffi.h"  // NOLINT(build/include)

namespace rmw_int2dds_cpp
{

/// Take exactly one serialized sample together with its SampleInfo.
///
/// Uses the batch take API (the plain serialized take APIs do not expose
/// SampleInfo). The buffer is grown as needed to fit the sample.
/// Returns INT2DDS_RET_NO_DATA when nothing was available.
Int2DdsRet
take_one_serialized_with_info(
  const Int2DdsDataReader * reader,
  std::vector<uint8_t> & buffer,
  size_t * data_size,
  bool * valid_data,
  Int2DdsSampleInfo * info);

/// Convert a SampleInfo source timestamp to a single nanosecond value.
int64_t
sample_info_source_timestamp_ns(const Int2DdsSampleInfo & info);

}  // namespace rmw_int2dds_cpp

#endif  // COMMON__TAKE_WITH_INFO_HPP_
