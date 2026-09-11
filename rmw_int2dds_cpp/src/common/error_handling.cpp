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

#include "rmw/error_handling.h"
#include "rmw/types.h"

#include "int2dds-ffi.h"  // NOLINT(build/include)

namespace rmw_int2dds_cpp
{

rmw_ret_t convert_int2dds_ret(Int2DdsRet ret)
{
  switch (ret) {
    case INT2DDS_RET_OK:
      return RMW_RET_OK;
    case INT2DDS_RET_ERROR:
      return RMW_RET_ERROR;
    case INT2DDS_RET_TIMEOUT:
      return RMW_RET_TIMEOUT;
    case INT2DDS_RET_UNSUPPORTED:
      return RMW_RET_UNSUPPORTED;
    case INT2DDS_RET_INVALID_ARGUMENT:
      return RMW_RET_INVALID_ARGUMENT;
    case INT2DDS_RET_NO_DATA:
      return RMW_RET_OK;  // No data is not an error for take operations
    case INT2DDS_RET_PRECONDITION_NOT_MET:
      return RMW_RET_ERROR;
    case INT2DDS_RET_NULL_POINTER:
      return RMW_RET_INVALID_ARGUMENT;
    default:
      return RMW_RET_ERROR;
  }
}

const char * int2dds_ret_to_string(Int2DdsRet ret)
{
  switch (ret) {
    case INT2DDS_RET_OK:
      return "OK";
    case INT2DDS_RET_ERROR:
      return "ERROR";
    case INT2DDS_RET_TIMEOUT:
      return "TIMEOUT";
    case INT2DDS_RET_UNSUPPORTED:
      return "UNSUPPORTED";
    case INT2DDS_RET_INVALID_ARGUMENT:
      return "INVALID_ARGUMENT";
    case INT2DDS_RET_ALREADY_DELETED:
      return "ALREADY_DELETED";
    case INT2DDS_RET_NOT_ENABLED:
      return "NOT_ENABLED";
    case INT2DDS_RET_IMMUTABLE_POLICY:
      return "IMMUTABLE_POLICY";
    case INT2DDS_RET_INCONSISTENT_POLICY:
      return "INCONSISTENT_POLICY";
    case INT2DDS_RET_PRECONDITION_NOT_MET:
      return "PRECONDITION_NOT_MET";
    case INT2DDS_RET_OUT_OF_RESOURCES:
      return "OUT_OF_RESOURCES";
    case INT2DDS_RET_ILLEGAL_OPERATION:
      return "ILLEGAL_OPERATION";
    case INT2DDS_RET_NO_DATA:
      return "NO_DATA";
    case INT2DDS_RET_NULL_POINTER:
      return "NULL_POINTER";
    default:
      return "UNKNOWN";
  }
}

}  // namespace rmw_int2dds_cpp
