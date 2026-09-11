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

#include <cstdlib>

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#include "rcutils/logging.h"

#include "rmw_int2dds_cpp/identifier.hpp"
#include "rmw_int2dds_cpp/visibility_control.h"

namespace rmw_int2dds_cpp
{

const char * const implementation_identifier = "rmw_int2dds_cpp";
const char * const serialization_format = "cdr";

}  // namespace rmw_int2dds_cpp

extern "C"
{

namespace
{

const char *
severity_to_int2dds_level(rmw_log_severity_t severity)
{
  switch (severity) {
    case RMW_LOG_SEVERITY_DEBUG:
      return "debug";
    case RMW_LOG_SEVERITY_INFO:
      return "info";
    case RMW_LOG_SEVERITY_WARN:
      return "warn";
    case RMW_LOG_SEVERITY_ERROR:
      return "error";
    case RMW_LOG_SEVERITY_FATAL:
      return "error";
    default:
      return nullptr;
  }
}

}  // namespace

const char *
rmw_get_implementation_identifier(void)
{
  return rmw_int2dds_cpp::implementation_identifier;
}

const char *
rmw_get_serialization_format(void)
{
  return rmw_int2dds_cpp::serialization_format;
}

rmw_ret_t
rmw_set_log_severity(rmw_log_severity_t severity)
{
  const char * level = severity_to_int2dds_level(severity);
  if (level == nullptr) {
    RMW_SET_ERROR_MSG("unsupported log severity");
    return RMW_RET_INVALID_ARGUMENT;
  }

  rcutils_logging_set_default_logger_level(static_cast<int>(severity));

  // int2dds configures its Rust logger from env vars during initialization.
  // Keep the requested severity in sync for future contexts and subprocesses.
  if (setenv("INT2DDS_CONSOLE_LOG_LEVEL", level, 1) != 0) {
    RMW_SET_ERROR_MSG("failed to set INT2DDS_CONSOLE_LOG_LEVEL");
    return RMW_RET_ERROR;
  }
  if (setenv("INT2DDS_FILE_LOG_LEVEL", level, 1) != 0) {
    RMW_SET_ERROR_MSG("failed to set INT2DDS_FILE_LOG_LEVEL");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

// rmw_feature_supported (rmw/features.h) is Humble+; Foxy has no feature query.

}  // extern "C"
