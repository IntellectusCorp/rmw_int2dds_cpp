cmake_minimum_required(VERSION 3.14)
project(rmw_int2dds_validation)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# These probes are the reason this package exists separately from
# rmw_int2dds_cpp: they need rclcpp, and an RMW implementation cannot depend on
# rclcpp without closing a build dependency cycle. See the layering note in
# rmw_int2dds_cpp/CMakeLists.txt.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_int2dds_cpp REQUIRED)

# ============================================================================
# Probes
# ----------------------------------------------------------------------------
# Every probe is a standalone rclcpp node that selects the middleware at run
# time through RMW_IMPLEMENTATION, so none of them link rmw_int2dds_cpp and all
# of them build and install identically.
# ============================================================================
function(add_probe target source)
  add_executable(${target} ${source})
  ament_target_dependencies(${target} ${ARGN})
  install(TARGETS ${target}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

# QoS: durability
add_probe(qos_durability_sub_cpp
  validation/qos/durability/rclcpp/qos_durability_sub.cpp rclcpp std_msgs)
add_probe(qos_durability_pub_cpp
  validation/qos/durability/rclcpp/qos_durability_pub.cpp rclcpp std_msgs)

# QoS: history depth
add_probe(qos_history_depth_sub_cpp
  validation/qos/history_depth/rclcpp/qos_history_depth_sub.cpp rclcpp std_msgs)
add_probe(qos_history_depth_pub_cpp
  validation/qos/history_depth/rclcpp/qos_history_depth_pub.cpp rclcpp std_msgs)

# QoS: deadline
add_probe(qos_deadline_sub_cpp
  validation/qos/deadline/rclcpp/qos_deadline_sub.cpp rclcpp std_msgs)
add_probe(qos_deadline_pub_cpp
  validation/qos/deadline/rclcpp/qos_deadline_pub.cpp rclcpp std_msgs)

# QoS: liveliness
add_probe(qos_liveliness_sub_cpp
  validation/qos/liveliness/rclcpp/qos_liveliness_sub.cpp rclcpp std_msgs)
add_probe(qos_liveliness_pub_cpp
  validation/qos/liveliness/rclcpp/qos_liveliness_pub.cpp rclcpp std_msgs)

# QoS: lifespan
add_probe(qos_lifespan_sub_cpp
  validation/qos/lifespan/rclcpp/qos_lifespan_sub.cpp rclcpp std_msgs)
add_probe(qos_lifespan_pub_cpp
  validation/qos/lifespan/rclcpp/qos_lifespan_pub.cpp rclcpp std_msgs)

# Content-filtered topics
# Content filtered topics arrived in Humble; Foxy has neither the rmw API nor
# rclcpp::ContentFilterOptions, so the probe is built only where the rmw
# headers provide it.
find_file(RMW_INT2DDS_VALIDATION_CFT_HEADER
  NAMES rmw/subscription_content_filter_options.h
  HINTS ${rmw_INCLUDE_DIRS}
  NO_DEFAULT_PATH)
mark_as_advanced(RMW_INT2DDS_VALIDATION_CFT_HEADER)
if(RMW_INT2DDS_VALIDATION_CFT_HEADER)
  add_probe(content_filter_lifecycle_check_cpp
    validation/cft/rclcpp/content_filter_lifecycle_check.cpp rclcpp std_msgs)
else()
  message(STATUS "rmw content filter API not available - skipping content_filter_lifecycle_check_cpp")
endif()

# Executor callbacks - publishes nothing, so no std_msgs here.
add_probe(callback_executor_smoke_cpp
  validation/callback/rclcpp/callback_executor_smoke.cpp rclcpp)

# Performance: latency / throughput / readiness
add_probe(perf_latency_pub_cpp
  validation/perf/rclcpp/perf_latency_pub_cpp.cpp rclcpp std_msgs)
add_probe(perf_latency_sub_cpp
  validation/perf/rclcpp/perf_latency_sub_cpp.cpp rclcpp std_msgs)
add_probe(perf_throughput_pub_cpp
  validation/perf/rclcpp/perf_throughput_pub_cpp.cpp rclcpp std_msgs)
add_probe(perf_throughput_sub_cpp
  validation/perf/rclcpp/perf_throughput_sub_cpp.cpp rclcpp std_msgs)
add_probe(perf_readiness_pub_cpp
  validation/perf/rclcpp/perf_readiness_pub_cpp.cpp rclcpp std_msgs)
add_probe(perf_readiness_sub_cpp
  validation/perf/rclcpp/perf_readiness_sub_cpp.cpp rclcpp std_msgs)

# ============================================================================
# D4: batch take_sequence functional check
# ----------------------------------------------------------------------------
# Unlike the probes above this one links the middleware directly instead of
# going through rmw_implementation, so it needs rmw_int2dds_cpp at build time.
# It was not installed while it lived in rmw_int2dds_cpp; that is preserved
# here, so run it from the build tree.
#
# It also used to be guarded by `if(rcl_FOUND)` because rcl was probed with
# find_package(rcl QUIET) and never declared in package.xml. On the ROS build
# farm, which installs only declared rosdep keys, that guard silently dropped
# the target. rcl is a declared dependency of this package, so the check now
# builds everywhere.
# ============================================================================
add_executable(test_take_sequence test/test_take_sequence.cpp)
ament_target_dependencies(test_take_sequence
  rclcpp
  rcl
  rmw
  rcutils
  std_msgs
  rmw_int2dds_cpp
)

# NOTE: no ament_lint_auto here. validation/* was excluded from every linter in
# rmw_int2dds_cpp (AMENT_LINT_AUTO_FILE_EXCLUDE), and these are the same files;
# enabling lint now would fail on sources that have never been linted. Add it
# deliberately, in its own change, once the sources are cleaned up.

ament_package()
