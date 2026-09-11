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

#include <tuple>
#include <cstring>
#include <vector>

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#include "rcutils/time.h"

#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

#include "int2dds-ffi.h"  // NOLINT(build/include): vendored FFI header
#include "rmw_int2dds_cpp/identifier.hpp"
#include <mutex>
#include "rmw_int2dds_cpp/types.hpp"
#include "rmw_int2dds_cpp/cdr_serializer.hpp"
#include "../common/take_with_info.hpp"  // NOLINT(build/include)

// Service request/response wire format:
// [CDR header 4 bytes] [8 bytes client key] [8 bytes sequence number] [CDR payload]

namespace
{

constexpr size_t SERVICE_HEADER_SIZE = 16;  // 8 bytes client key + 8 bytes sequence
constexpr size_t DEFAULT_BUFFER_SIZE = 65536;
constexpr size_t CLIENT_KEY_SIZE = 8;

/// Copy the stable per-client routing key from an rmw_int2dds_cpp GID.
///
/// The first 8 bytes of the generated GID are [INT2 prefix][pid], which is not
/// unique for multiple clients in the same process.  Bytes 4..11 are
/// [pid][entity counter], which are stable across request/response round-trips
/// and distinguish clients created in the same process.
void
copy_client_key_from_gid(uint8_t * dest, const rmw_gid_t & gid)
{
  memcpy(dest, gid.data + 4, CLIENT_KEY_SIZE);
}

bool
client_key_matches(const uint8_t * key, const rmw_int2dds_cpp::ClientData * cli_data)
{
  uint8_t expected[CLIENT_KEY_SIZE] = {};
  copy_client_key_from_gid(expected, cli_data->gid);
  return memcmp(key, expected, CLIENT_KEY_SIZE) == 0;
}

/// Build service header for request with client's routing key and sequence number
void
build_request_header(
  std::vector<uint8_t> & buffer,
  const rmw_int2dds_cpp::ClientData * cli_data,
  int64_t sequence_number)
{
  // Reserve space for header
  size_t start = buffer.size();
  buffer.resize(start + SERVICE_HEADER_SIZE);

  // Copy unique 8-byte client key
  copy_client_key_from_gid(buffer.data() + start, cli_data->gid);

  // Copy sequence number
  memcpy(buffer.data() + start + 8, &sequence_number, 8);
}

/// Extract service header from received response
bool
extract_response_header(
  const uint8_t * data,
  size_t data_size,
  rmw_request_id_t * request_id,
  const uint8_t ** payload,
  size_t * payload_size)
{
  if (data_size < 4 + SERVICE_HEADER_SIZE) {
    return false;
  }

  // Skip CDR header (4 bytes)
  const uint8_t * header = data + 4;

  // Copy client key (8 bytes, padded to RMW_GID_STORAGE_SIZE)
  memset(request_id->writer_guid, 0, RMW_GID_STORAGE_SIZE);
  memcpy(request_id->writer_guid, header, CLIENT_KEY_SIZE);

  // Copy sequence number
  memcpy(&request_id->sequence_number, header + 8, 8);

  // Point to payload
  *payload = data + 4 + SERVICE_HEADER_SIZE;
  *payload_size = data_size - 4 - SERVICE_HEADER_SIZE;

  return true;
}

}  // namespace

extern "C"
{
rmw_ret_t
rmw_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);

  if (client->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("client not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * cli_data = static_cast<rmw_int2dds_cpp::ClientData *>(client->data);
  if (cli_data == nullptr || cli_data->request_writer == nullptr) {
    RMW_SET_ERROR_MSG("client data is null");
    return RMW_RET_ERROR;
  }

  // Get next sequence number
  int64_t seq = cli_data->sequence_number.fetch_add(1) + 1;
  *sequence_id = seq;

  // Serialize request
  rmw_int2dds_cpp::CdrSerializer serializer;

  bool serialize_success = false;
  if (cli_data->type_support->typesupport_identifier ==
    rosidl_typesupport_introspection_c__identifier)
  {
    const auto * srv_members = static_cast<
      const rosidl_typesupport_introspection_c__ServiceMembers *>(
      cli_data->type_support->data);
    serialize_success = serializer.serialize_message_c(
      ros_request, srv_members->request_members_);
  } else if (cli_data->type_support->typesupport_identifier ==  // NOLINT(readability/braces)
    rosidl_typesupport_introspection_cpp::typesupport_identifier)
  {
    const auto * srv_members = static_cast<
      const rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      cli_data->type_support->data);
    serialize_success = serializer.serialize_message_cpp(
      ros_request, srv_members->request_members_);
  } else {
    RMW_SET_ERROR_MSG("unknown type support");
    return RMW_RET_ERROR;
  }

  if (!serialize_success) {
    RMW_SET_ERROR_MSG("failed to serialize request");
    return RMW_RET_ERROR;
  }

  // Build complete message with header
  std::vector<uint8_t> send_buffer;

  // CDR encapsulation header
  send_buffer.push_back(0x00);  // CDR_LE
  send_buffer.push_back(0x01);
  send_buffer.push_back(0x00);
  send_buffer.push_back(0x00);

  // Service header with client routing key and sequence number
  build_request_header(send_buffer, cli_data, seq);

  // Append serialized payload (skip the CDR header from serializer since we added our own)
  const auto & payload = serializer.get_buffer();
  if (payload.size() > 4) {
    send_buffer.insert(send_buffer.end(), payload.begin() + 4, payload.end());
  }

  // Send request
  Int2DdsRet ret = int2dds_datawriter_write_serialized(
    cli_data->request_writer,
    send_buffer.data(),
    send_buffer.size());

  if (ret != INT2DDS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to send request");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  *taken = false;

  if (client->implementation_identifier != rmw_int2dds_cpp::implementation_identifier) {
    RMW_SET_ERROR_MSG("client not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  auto * cli_data = static_cast<rmw_int2dds_cpp::ClientData *>(client->data);
  if (cli_data == nullptr || cli_data->response_reader == nullptr) {
    RMW_SET_ERROR_MSG("client data is null");
    return RMW_RET_ERROR;
  }

  // Allocate buffer for receiving data
  std::vector<uint8_t> buffer(DEFAULT_BUFFER_SIZE);
  const uint8_t * payload = nullptr;
  size_t payload_size = 0;
  Int2DdsSampleInfo sample_info;

  std::lock_guard<std::mutex> take_lk(cli_data->response_take_mutex);
  while (true) {
    size_t data_size = 0;
    bool valid_data = false;

    // Take response from DDS together with its SampleInfo.  Each client has
    // its own DataReader on the shared response topic, so foreign responses
    // can be consumed from this reader and skipped without stealing them from
    // the owning client's reader.
    Int2DdsRet ret = rmw_int2dds_cpp::take_one_serialized_with_info(
      cli_data->response_reader,
      buffer,
      &data_size,
      &valid_data,
      &sample_info);

    if (ret == INT2DDS_RET_NO_DATA) {
      return RMW_RET_OK;
    }

    if (ret != INT2DDS_RET_OK) {
      RMW_SET_ERROR_MSG("failed to take response");
      return RMW_RET_ERROR;
    }

    if (!valid_data) {
      continue;
    }

    // Extract header
    rmw_service_info_t candidate_header{};
    if (!extract_response_header(
        buffer.data(), data_size, &candidate_header.request_id, &payload, &payload_size))
    {
      RMW_SET_ERROR_MSG("failed to extract response header");
      return RMW_RET_ERROR;
    }

    if (!client_key_matches(
        reinterpret_cast<const uint8_t *>(candidate_header.request_id.writer_guid), cli_data))
    {
      continue;
    }

    request_header->request_id = candidate_header.request_id;
    break;
  }

  // Fill service info
  request_header->source_timestamp =
    rmw_int2dds_cpp::sample_info_source_timestamp_ns(sample_info);
  std::ignore = rcutils_system_time_now(&request_header->received_timestamp);

  // Deserialize response message
  rmw_int2dds_cpp::CdrDeserializer deserializer(payload, payload_size);

  bool deserialize_success = false;
  if (cli_data->type_support->typesupport_identifier ==
    rosidl_typesupport_introspection_c__identifier)
  {
    const auto * srv_members = static_cast<
      const rosidl_typesupport_introspection_c__ServiceMembers *>(
      cli_data->type_support->data);
    deserialize_success = deserializer.deserialize_message_c(
      ros_response, srv_members->response_members_);
  } else if (cli_data->type_support->typesupport_identifier ==  // NOLINT(readability/braces)
    rosidl_typesupport_introspection_cpp::typesupport_identifier)
  {
    const auto * srv_members = static_cast<
      const rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      cli_data->type_support->data);
    deserialize_success = deserializer.deserialize_message_cpp(
      ros_response, srv_members->response_members_);
  } else {
    RMW_SET_ERROR_MSG("unknown type support");
    return RMW_RET_ERROR;
  }

  if (!deserialize_success) {
    RMW_SET_ERROR_MSG("failed to deserialize response");
    return RMW_RET_ERROR;
  }

  *taken = true;
  return RMW_RET_OK;
}
}  // extern "C"
