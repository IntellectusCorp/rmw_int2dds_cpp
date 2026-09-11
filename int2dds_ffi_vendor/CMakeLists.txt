cmake_minimum_required(VERSION 3.12)
project(int2dds_ffi_vendor NONE)

find_package(ament_cmake REQUIRED)

# ============================================================================
# Prebuilt artifact configuration
#
#   int2DDS publishes one tarball per OS on GitHub Releases. Each tarball
#   bundles every architecture for that OS plus a manifest:
#
#       int2dds-ffi.h                       (C API header)
#       int2dds-ffi.manifest.yaml           (per-arch file + sha256 + min_glibc)
#       linux-x86_64/libint2dds_ffi.so      (amd64,  gnu)
#       linux-x86_64-musl/libint2dds_ffi.so (amd64,  musl)
#       linux-aarch64/libint2dds_ffi.so     (arm64,  gnu)
#       linux-aarch64-musl/libint2dds_ffi.so(arm64,  musl)
#       linux-armhf/libint2dds_ffi.so       (armv7)
#
#   This package downloads the tarball for the host OS, selects the artifact
#   matching the host arch + libc, verifies its sha256 against the manifest,
#   and installs it as include/ + lib/.
# ============================================================================

set(INT2DDS_FFI_VERSION "0.1.5")
set(INT2DDS_FFI_RELEASE_TAG "v${INT2DDS_FFI_VERSION}")
set(INT2DDS_FFI_BASE_URL
  "https://github.com/IntellectusCorp/int2dds_ffi_vendor/releases/download/${INT2DDS_FFI_RELEASE_TAG}")

# glibc vs musl. ROS binaries target glibc; override with -DINT2DDS_FFI_LIBC=musl.
set(INT2DDS_FFI_LIBC "gnu" CACHE STRING "C library variant of the prebuilt int2DDS FFI (gnu|musl)")

# ----------------------------------------------------------------------------
# Detect OS -> per-OS asset name
# ----------------------------------------------------------------------------
if(WIN32)
  set(_os "windows")
  set(_asset "int2dds-ffi-${INT2DDS_FFI_VERSION}-windows.zip")
elseif(APPLE)
  set(_os "macos")
  set(_asset "int2dds-ffi-${INT2DDS_FFI_VERSION}-macos.tar.gz")
else()
  set(_os "linux")
  set(_asset "int2dds-ffi-${INT2DDS_FFI_VERSION}-linux.tar.gz")
endif()

# ----------------------------------------------------------------------------
# Detect architecture -> in-tarball subdirectory token
# ----------------------------------------------------------------------------
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _proc)
if(_proc MATCHES "x86_64|amd64")
  set(_arch_token "x86_64")
elseif(_proc MATCHES "aarch64|arm64")
  set(_arch_token "aarch64")
elseif(_proc MATCHES "armv7|armhf|arm")
  set(_arch_token "armhf")
else()
  message(FATAL_ERROR "int2dds_ffi_vendor: unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}'")
endif()

# armhf ships only a gnu build; everything else honours the libc selector.
if(_arch_token STREQUAL "armhf")
  set(_subdir "${_os}-${_arch_token}")
elseif(INT2DDS_FFI_LIBC STREQUAL "musl")
  set(_subdir "${_os}-${_arch_token}-musl")
else()
  set(_subdir "${_os}-${_arch_token}")
endif()

if(WIN32)
  set(_libname "int2dds_ffi.dll")
elseif(APPLE)
  set(_libname "libint2dds_ffi.dylib")
else()
  set(_libname "libint2dds_ffi.so.${INT2DDS_FFI_VERSION}")
endif()
set(_member "${_subdir}/${_libname}")

# ----------------------------------------------------------------------------
# Download + extract at configure time
# ----------------------------------------------------------------------------
set(_dl_dir "${CMAKE_CURRENT_BINARY_DIR}/download")
set(_extract_dir "${CMAKE_CURRENT_BINARY_DIR}/prebuilt")
set(_tarball "${_dl_dir}/${_asset}")
set(_url "${INT2DDS_FFI_BASE_URL}/${_asset}")

file(MAKE_DIRECTORY "${_dl_dir}")
message(STATUS "int2dds_ffi_vendor: fetching ${_url}")
file(DOWNLOAD "${_url}" "${_tarball}"
  STATUS _dl_status
  SHOW_PROGRESS
  TLS_VERIFY ON)
list(GET _dl_status 0 _dl_code)
if(NOT _dl_code EQUAL 0)
  list(GET _dl_status 1 _dl_msg)
  message(FATAL_ERROR "int2dds_ffi_vendor: download failed (${_dl_code}): ${_dl_msg}\n  URL: ${_url}")
endif()

file(REMOVE_RECURSE "${_extract_dir}")
file(MAKE_DIRECTORY "${_extract_dir}")
# file(ARCHIVE_EXTRACT) needs CMake 3.18, but Foxy targets Ubuntu 20.04 whose
# CMake is 3.16. `cmake -E tar xf` detects .tar.gz/.zip by content and keeps the
# SONAME symlink chain intact.
execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xf "${_tarball}"
  WORKING_DIRECTORY "${_extract_dir}"
  RESULT_VARIABLE _extract_result)
if(NOT _extract_result EQUAL 0)
  message(FATAL_ERROR "int2dds_ffi_vendor: failed to extract ${_asset} (${_extract_result})")
endif()

set(_header "${_extract_dir}/int2dds-ffi.h")
set(_lib "${_extract_dir}/${_member}")
set(_manifest "${_extract_dir}/int2dds-ffi.manifest.yaml")

if(NOT EXISTS "${_header}")
  message(FATAL_ERROR "int2dds_ffi_vendor: missing int2dds-ffi.h in ${_asset}")
endif()
if(NOT EXISTS "${_lib}")
  message(FATAL_ERROR
    "int2dds_ffi_vendor: artifact '${_member}' not found in ${_asset}\n"
    "  (host: ${CMAKE_SYSTEM_PROCESSOR}, libc: ${INT2DDS_FFI_LIBC})")
endif()

# ----------------------------------------------------------------------------
# Verify the selected library against the sha256 recorded in the manifest
# ----------------------------------------------------------------------------
if(EXISTS "${_manifest}")
  file(STRINGS "${_manifest}" _manifest_lines)
  set(_expected_sha "")
  set(_seen_member FALSE)
  foreach(_line IN LISTS _manifest_lines)
    if(_line MATCHES "file:[ \t]*${_member}[ \t]*$")
      set(_seen_member TRUE)
    elseif(_seen_member AND _line MATCHES "sha256:[ \t]*([0-9a-fA-F]+)")
      set(_expected_sha "${CMAKE_MATCH_1}")
      break()
    elseif(_seen_member AND _line MATCHES "file:")
      break()  # moved to the next artifact without finding a sha
    endif()
  endforeach()

  if(_expected_sha STREQUAL "")
    message(WARNING "int2dds_ffi_vendor: no sha256 for '${_member}' in manifest; skipping verification.")
  else()
    file(SHA256 "${_lib}" _actual_sha)
    string(TOLOWER "${_expected_sha}" _expected_sha)
    string(TOLOWER "${_actual_sha}" _actual_sha)
    if(NOT _actual_sha STREQUAL _expected_sha)
      message(FATAL_ERROR
        "int2dds_ffi_vendor: sha256 mismatch for '${_member}'\n"
        "  expected: ${_expected_sha}\n  actual:   ${_actual_sha}")
    endif()
    message(STATUS "int2dds_ffi_vendor: verified ${_member} (sha256 ok)")
  endif()
else()
  message(WARNING "int2dds_ffi_vendor: manifest not found in ${_asset}; skipping verification.")
endif()

# ----------------------------------------------------------------------------
# Normalise into the package install prefix: include/ + lib/
# ----------------------------------------------------------------------------
install(FILES "${_header}" DESTINATION "include")
file(GLOB _libfiles "${_extract_dir}/${_subdir}/libint2dds_ffi.so*")
install(FILES ${_libfiles} DESTINATION "lib")
if(WIN32)
  # On Windows also install the import library next to the runtime DLL.
  set(_implib "${_extract_dir}/${_subdir}/int2dds_ffi.lib")
  if(EXISTS "${_implib}")
    install(FILES "${_implib}" DESTINATION "lib")
  endif()
endif()

ament_export_include_directories(include)
ament_package(CONFIG_EXTRAS "cmake/int2dds_ffi_vendor-extras.cmake")
