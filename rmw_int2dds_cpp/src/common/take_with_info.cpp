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

#include "take_with_info.hpp"  // NOLINT(build/include)

#include <algorithm>
#include <cstring>
#include <vector>

namespace rmw_int2dds_cpp
{

namespace
{
constexpr size_t kGrowthPaddingBytes = 256;
}  // namespace

Int2DdsRet
take_one_serialized_with_info(
  const Int2DdsDataReader * reader,
  std::vector<uint8_t> & buffer,
  size_t * data_size,
  bool * valid_data,
  Int2DdsSampleInfo * info)
{
  *data_size = 0;
  *valid_data = false;

  Int2DdsSampleSeq * seq = nullptr;
  Int2DdsRet ret = int2dds_datareader_take_serialized_batch(reader, 1, &seq);
  if (ret != INT2DDS_RET_OK) {
    if (seq != nullptr) {
      int2dds_sample_seq_delete(seq);
    }
    return ret;
  }
  if (seq == nullptr || int2dds_sample_seq_length(seq) == 0) {
    if (seq != nullptr) {
      int2dds_sample_seq_delete(seq);
    }
    return INT2DDS_RET_NO_DATA;
  }

  std::memset(info, 0, sizeof(*info));
  ret = int2dds_sample_seq_get_info(seq, 0, info);
  if (ret != INT2DDS_RET_OK) {
    int2dds_sample_seq_delete(seq);
    return ret;
  }
  *valid_data = info->valid_data;

  uintptr_t actual_size = 0;
  ret = int2dds_sample_seq_get_data(seq, 0, buffer.data(), buffer.size(), &actual_size);
  if (ret != INT2DDS_RET_OK && actual_size > buffer.size()) {
    buffer.resize(actual_size + kGrowthPaddingBytes);
    ret = int2dds_sample_seq_get_data(seq, 0, buffer.data(), buffer.size(), &actual_size);
  }
  int2dds_sample_seq_delete(seq);

  if (ret != INT2DDS_RET_OK) {
    return ret;
  }
  *data_size = actual_size;
  return INT2DDS_RET_OK;
}

int64_t
sample_info_source_timestamp_ns(const Int2DdsSampleInfo & info)
{
  return static_cast<int64_t>(info.source_timestamp_sec) * 1000000000LL +
         static_cast<int64_t>(info.source_timestamp_nanosec);
}

}  // namespace rmw_int2dds_cpp
