cmake_minimum_required(VERSION 3.14)
project(rmw_int2dds_cpp)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(
  RMW_INT2DDS_BUILD_EVENT_VALIDATION
  "Build RMW event validation executable"
  ON)
# RMW_INT2DDS_BUILD_MATCHED_EVENT_TESTS is declared further down, after
# find_package(rmw): its default is probed from the installed rmw headers rather
# than guessed from the environment.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
# package.xml declares this; loading it is what sets BUILD_SHARED_LIBS ON and
# defines ROS_PACKAGE_NAME. Has to run before add_library().
find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rmw_dds_common REQUIRED)
find_package(std_msgs REQUIRED)

# NOTE: rclcpp and rcl are deliberately absent from this package. They sit ABOVE
# it in the ROS 2 stack:
#
#   rclcpp -> rcl -> rmw_implementation -> rmw_int2dds_cpp
#                    (via the rmw_implementation_packages group)
#
# Depending on either would close a build dependency cycle and drag the whole
# client library stack into an RMW plugin. No upstream RMW implementation does
# it (rmw_fastrtps, rmw_cyclonedds, rmw_connextdds, rmw_zenoh: zero references).
# The rclcpp-based QoS/perf probes live in the sibling rmw_int2dds_validation
# package instead, mirroring test_rmw_zenoh_cpp in ros2/rmw_zenoh.

# The matched-event smoke executable is built entirely on rmw matched-event APIs
# that only exist in newer rmw headers, so ask the headers whether they are there
# - the same question test_event_validation.cpp asks with __has_include.
#
# It must NOT be inferred from $ENV{ROS_DISTRO}: ros_environment is the only
# package that exports that variable, and the ROS build farm never installs it
# (the devel job builds in a bare ubuntu image carrying only the declared rosdep
# keys), so it is empty there and a distro comparison silently picks the wrong
# branch. ros:humble-ros-base does set it, which is why guessing from the distro
# worked in GitHub Actions and broke only on the farm.
find_file(RMW_INT2DDS_MATCHED_EVENT_HEADER
  NAMES rmw/events_statuses/matched.h
  HINTS ${rmw_INCLUDE_DIRS})
mark_as_advanced(RMW_INT2DDS_MATCHED_EVENT_HEADER)
if(RMW_INT2DDS_MATCHED_EVENT_HEADER)
  set(RMW_INT2DDS_BUILD_MATCHED_EVENT_TESTS_DEFAULT ON)
  message(STATUS "rmw matched-event API found: ${RMW_INT2DDS_MATCHED_EVENT_HEADER}")
else()
  set(RMW_INT2DDS_BUILD_MATCHED_EVENT_TESTS_DEFAULT OFF)
  message(STATUS
    "rmw matched-event API not available (searched: ${rmw_INCLUDE_DIRS}) - "
    "the matched-event smoke target is disabled for this rmw version.")
endif()
option(
  RMW_INT2DDS_BUILD_MATCHED_EVENT_TESTS
  "Build matched-event smoke executable that depends on newer RMW matched-event APIs"
  ${RMW_INT2DDS_BUILD_MATCHED_EVENT_TESTS_DEFAULT})

# ============================================================================
# int2dds-ffi library
# ----------------------------------------------------------------------------
# The prebuilt int2DDS FFI library is provided by the int2dds_ffi_vendor
# package, which downloads the artifact for this platform and exposes the
# int2dds_ffi::int2dds_ffi imported target (include dir + shared library).
# ============================================================================
find_package(int2dds_ffi_vendor REQUIRED)

if(WIN32)
  set(PLATFORM_LIBS ws2_32 ntdll userenv iphlpapi ole32)
elseif(APPLE)
  set(PLATFORM_LIBS pthread)
else()  # Linux
  set(PLATFORM_LIBS pthread dl)
endif()

# ============================================================================
# RMW Library
# ============================================================================
add_library(rmw_int2dds_cpp SHARED
  # Main entry point
  src/rmw_int2dds.cpp

  # Initialization
  src/init/rmw_init.cpp

  # Node management
  src/node/rmw_node.cpp

  # Publisher
  src/publisher/rmw_publisher.cpp
  src/publisher/rmw_publish.cpp

  # Subscription
  src/subscription/rmw_subscription.cpp
  src/subscription/rmw_take.cpp

  # Guard condition
  src/guard_condition/rmw_guard_condition.cpp

  # Wait set
  src/wait/rmw_wait.cpp
  src/wait/rmw_wait_set.cpp
  src/wait/waitset_registry.cpp

  # Service
  src/service/rmw_service.cpp
  src/service/rmw_service_request.cpp

  # Client
  src/client/rmw_client.cpp
  src/client/rmw_client_request.cpp

  # Event
  src/event/rmw_event.cpp

  # Graph
  src/graph/rmw_graph.cpp
  src/graph/discovery.cpp

  # QoS: rmw_qos_profile_check_compatible (src/qos/rmw_qos.cpp on the humble
  # branch) is Galactic+ and its helper rmw_dds_common/qos.hpp does not exist on
  # Foxy, so there is no QoS source here.

  # Serialization
  src/serialization/rmw_serialize.cpp
  src/serialization/cdr_serializer.cpp

  # Common utilities
  src/common/error_handling.cpp
  src/common/gid.cpp
  src/common/listeners.cpp
  src/common/take_with_info.cpp
  src/common/topic_names.cpp
)

target_compile_definitions(rmw_int2dds_cpp PRIVATE "RMW_INT2DDS_CPP_BUILDING_DLL")

target_include_directories(rmw_int2dds_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rmw_int2dds_cpp
  int2dds_ffi::int2dds_ffi
  ${PLATFORM_LIBS}
)

ament_target_dependencies(rmw_int2dds_cpp
  rcutils
  rmw
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp
  rosidl_typesupport_c
  rosidl_typesupport_cpp
  rmw_dds_common
)

# Configure as RMW library
configure_rmw_library(rmw_int2dds_cpp)

# Register RMW implementation
register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_introspection_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_introspection_cpp"
)

# ============================================================================
# Test connection executable (for development testing)
# ============================================================================
add_executable(test_connection test/test_connection.cpp)
target_include_directories(test_connection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_connection int2dds_ffi::int2dds_ffi ${PLATFORM_LIBS})

# D4: the batch take_sequence functional check needs rclcpp and rcl, so it lives
# in the rmw_int2dds_validation package.

if(RMW_INT2DDS_BUILD_MATCHED_EVENT_TESTS)
  add_executable(test_event_smoke test/test_event_smoke.cpp)
  target_include_directories(test_event_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_event_smoke rmw_int2dds_cpp int2dds_ffi::int2dds_ffi ${PLATFORM_LIBS})
  ament_target_dependencies(test_event_smoke
    rcutils
    rmw
    rosidl_runtime_cpp
    rosidl_typesupport_cpp
    std_msgs
  )
endif()

if(RMW_INT2DDS_BUILD_EVENT_VALIDATION)
  add_executable(test_event_validation test/test_event_validation.cpp)
  target_include_directories(test_event_validation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_event_validation rmw_int2dds_cpp int2dds_ffi::int2dds_ffi ${PLATFORM_LIBS})
  # test_event_validation.cpp drives the rmw C API directly - it includes no
  # rclcpp header, so rclcpp was never a real dependency of this target.
  ament_target_dependencies(test_event_validation
    rcutils
    rmw
    rosidl_runtime_cpp
    rosidl_typesupport_cpp
    std_msgs
  )
endif()

# ============================================================================
# Installation
# ============================================================================
install(TARGETS rmw_int2dds_cpp
  EXPORT export_rmw_int2dds_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS test_connection
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(RMW_INT2DDS_BUILD_MATCHED_EVENT_TESTS)
  install(TARGETS test_event_smoke
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(RMW_INT2DDS_BUILD_EVENT_VALIDATION)
  install(TARGETS test_event_validation
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# The int2dds FFI runtime library is installed by the int2dds_ffi_vendor
# package. On Windows, copy the DLL next to the freshly built binaries so the
# development/test executables can locate it at runtime.
if(WIN32)
  add_custom_command(TARGET rmw_int2dds_cpp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:int2dds_ffi::int2dds_ffi> $<TARGET_FILE_DIR:rmw_int2dds_cpp>
    COMMENT "Copying int2dds_ffi.dll to build directory")

  add_custom_command(TARGET test_connection POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:int2dds_ffi::int2dds_ffi> $<TARGET_FILE_DIR:test_connection>
    COMMENT "Copying int2dds_ffi.dll for test_connection")
endif()

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  file(GLOB_RECURSE RMW_INT2DDS_LINT_EXCLUDE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/ros2_documentation_pr/*"
  )
  list(APPEND RMW_INT2DDS_LINT_EXCLUDE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/CHANGELOG.rst"
    "${CMAKE_CURRENT_SOURCE_DIR}/CLA-Corporate.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/CLA-Individual.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/CODE_OF_CONDUCT.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/CONTRIBUTING.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/DOCUMENTATION.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/NOTICE"
    "${CMAKE_CURRENT_SOURCE_DIR}/QUALITY_DECLARATION.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/SECURITY.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/STRUCTURE.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/THIRD_PARTY_LICENSES.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/TRADEMARK_POLICY.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/int2dds-ffi.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_connection.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_event_smoke.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_event_validation.cpp"
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    ${RMW_INT2DDS_LINT_EXCLUDE_FILES}
  )
  # Foxy's ament_lint_auto predates AMENT_LINT_AUTO_FILE_EXCLUDE, so copyright
  # and uncrustify are added below with the same exclude list. Their Foxy
  # command-line --exclude matches base names rather than paths, so the list is
  # passed as base names.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  set(RMW_INT2DDS_LINT_EXCLUDE_NAMES)
  foreach(_exclude_file ${RMW_INT2DDS_LINT_EXCLUDE_FILES})
    get_filename_component(_exclude_name "${_exclude_file}" NAME)
    list(APPEND RMW_INT2DDS_LINT_EXCLUDE_NAMES "${_exclude_name}")
  endforeach()
  ament_copyright("${CMAKE_CURRENT_SOURCE_DIR}" --exclude ${RMW_INT2DDS_LINT_EXCLUDE_NAMES})
  ament_uncrustify("${CMAKE_CURRENT_SOURCE_DIR}" --exclude ${RMW_INT2DDS_LINT_EXCLUDE_NAMES})

  # ament_lint_auto never forwarded AMENT_LINT_AUTO_FILE_EXCLUDE to flake8.
  # Keep the package lint targets explicit so generated/runtime validation
  # assets do not fail release lint.
  ament_flake8(--exclude "${RMW_INT2DDS_LINT_EXCLUDE_FILES}")
  # Lint only the paths we own. Passing explicit paths (before FILTERS) works
  # identically on every ament_cmake_cpplint version, unlike EXCLUDE which is
  # single-value on Humble/Jazzy and silently turns the exclude list into
  # positional lint targets. include/ is narrowed to our namespace so a
  # vendored include/int2dds-ffi.h can never enter the lint scope.
  # Foxy's cpplint predates the build/include_subdir category and reports that
  # check as build/include, so the include lines it applies to carry
  # NOLINT(build/include) instead of being filtered package-wide.
  ament_cpplint(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/rmw_int2dds_cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    FILTERS
      -build/include_order
      -build/include_subdir
      -legal/copyright
      -readability/braces
      -runtime/int
      -whitespace/blank_line
      -whitespace/comments
      -whitespace/line_length
  )

  # Wait set unit tests. rmw_create_wait_set and rmw_create_guard_condition
  # only validate the context implementation identifier, so these run without a
  # DDS participant: no discovery, no sockets, no traffic, and no flakiness
  # from either.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rmw_wait_guards test/test_rmw_wait_guards.cpp
    TIMEOUT 120
    ENV RMW_IMPLEMENTATION=rmw_int2dds_cpp ROS_DOMAIN_ID=71
  )
  if(TARGET test_rmw_wait_guards)
    target_include_directories(test_rmw_wait_guards PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_rmw_wait_guards
      rmw_int2dds_cpp
      int2dds_ffi::int2dds_ffi
      ${PLATFORM_LIBS}
    )
    ament_target_dependencies(test_rmw_wait_guards rmw rcutils)
    if(WIN32)
      add_custom_command(TARGET test_rmw_wait_guards POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:int2dds_ffi::int2dds_ffi> $<TARGET_FILE_DIR:test_rmw_wait_guards>
        COMMENT "Copying int2dds_ffi.dll for test_rmw_wait_guards")
    endif()
  endif()
endif()

# ============================================================================
# Export
# ============================================================================
ament_export_include_directories(include)
ament_export_libraries(rmw_int2dds_cpp)
ament_export_targets(export_rmw_int2dds_cpp)
ament_export_dependencies(
  int2dds_ffi_vendor
  rcutils
  rmw
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp
  rosidl_typesupport_c
  rosidl_typesupport_cpp
  rmw_dds_common
)

ament_package()
