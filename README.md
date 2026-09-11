# ROS 2 RMW for int2DDS

<div align="center">

A **ROS 2 RMW implementation** that binds the **int2DDS**
DDS/RTPS middleware to the ROS 2 middleware (RMW) interface.

</div>

## Overview

**rmw_int2dds_cpp** lets ROS 2 applications run on top of **int2DDS**, a Rust
implementation of the OMG DDS standard (RTPS 2.5). It implements the ROS 2 `rmw`
C interface so that any ROS 2 stack (rclcpp, rclpy, ros2 CLI, tools) can use
int2DDS as its middleware via `RMW_IMPLEMENTATION=rmw_int2dds_cpp`.

> Status: **work in progress** — APIs and test results are being stabilized
> ahead of a request for Tier 3 status in [REP 2000](https://ros.org/reps/rep-2000.html).

### Supported ROS 2 distributions

| Distribution | Status |
|--------------|--------|
| Foxy Fitzroy (LTS)     | Supported (verified) |
| Humble Hawksbill (LTS) | Supported (verified) |
| Jazzy Jalisco (LTS)    | Supported (verified) |
| Lyrical Luth (LTS)     | Supported (verified) |

### Supported platforms

| Platform | Architectures | Status |
|----------|---------------|--------|
| Ubuntu (Linux) | amd64 | Supported (verified) |
| Ubuntu (Linux) | arm64 | Target (library builds; board validation pending) |

## Quick Start

```bash
# 1) Get the sources into your ROS 2 workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b foxy https://github.com/IntellectusCorp/rmw_int2dds.git

# 2) Build
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to rmw_int2dds_cpp
source install/setup.bash

# 3) Select int2DDS as the middleware
export RMW_IMPLEMENTATION=rmw_int2dds_cpp

# 4) Run any ROS 2 demo
ros2 run demo_nodes_cpp talker
# in another terminal (same RMW_IMPLEMENTATION):
ros2 run demo_nodes_cpp listener
```

## Binary install (.deb)

Prebuilt packages let you skip `colcon build`. Download the two `.deb`s for your
distro + architecture from the
[Releases](https://github.com/IntellectusCorp/rmw_int2dds/releases) page, then:

```bash
sudo apt install ./ros-foxy-int2dds-ffi-vendor_*_amd64.deb \
                 ./ros-foxy-rmw-int2dds-cpp_*_amd64.deb
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_int2dds_cpp
ros2 run demo_nodes_cpp talker
```

`apt install ./file.deb` installs the file and resolves its dependencies (the rmw
package pulls in the vendor package automatically). The RMW library and its
ament-index marker install into `/opt/ros/foxy/`, so once the environment is
sourced only `RMW_IMPLEMENTATION` needs to be set.

Supported: **jazzy / humble / rolling** × **amd64 / arm64**.
**armhf** is best-effort — there are no official ROS 2 armhf apt packages, so an
armhf `.deb` only works on a system where ROS 2 was itself built from source for
armhf.

To build the packages yourself: `packaging/build-deb.sh <distro> <arch>` (needs
Docker; see `packaging/` for the build and verification scripts).

## Repository layout

This repository holds two ROS 2 packages, released together:

| Package | Role |
|---------|------|
| [`rmw_int2dds_cpp/`](rmw_int2dds_cpp/) | The RMW implementation itself |
| [`int2dds_ffi_vendor/`](int2dds_ffi_vendor/) | Fetches the prebuilt int2DDS FFI library and exports it to CMake |

## Middleware library dependency

This package links against the **int2DDS FFI library** (`libint2dds_ffi.so*`
and `int2dds-ffi.h`), which is provided by the `int2dds_ffi_vendor` package in
this repository. At CMake configure time the vendor package downloads the
per-OS tarball from the
[int2dds_ffi_vendor releases](https://github.com/IntellectusCorp/int2dds_ffi_vendor/releases),
selects the artifact matching the host architecture and libc, verifies it
against the `sha256` recorded in the bundled manifest, and exports the
`int2dds_ffi::int2dds_ffi` CMake target used by this RMW package.

Building therefore needs outbound network access to `github.com`. The FFI
version is pinned in one place: `INT2DDS_FFI_VERSION` in
[int2dds_ffi_vendor/CMakeLists.txt](int2dds_ffi_vendor/CMakeLists.txt).

## Test status

All results below were produced by running the listed suites directly; see
`doc/` for methodology. Same-vendor and cross-vendor integration tests use the
official ROS 2 repositories (`rmw_implementation`, `system_tests`).

| Suite | Lyrical | Jazzy | Humble | Foxy |
|---|---|---|---|---|
| `test_rmw_implementation` (RMW conformance gate) | 16/16 | 16/16 | 15/15 | 13/13 |
| `test_communication` same-RMW | 34/34 | 30/30 | 29/29 | 28/28 |
| `test_quality_of_service` | 4/4 | 4/4 | 3/3 | n/a (not registered upstream) |
| `test_rclcpp` | 25/25 | 25/25 | 25/25 | 23/23 |
| Cross-vendor vs `rmw_fastrtps_cpp` | 8/8 | 8/8 | 8/8 | 8/8 |
| Cross-vendor vs `rmw_cyclonedds_cpp` | 8/8 excluding `WStrings` (see Known issues) | 8/8 | 8/8 | 8/8 |
| `test_cli_remapping` | 1/1 | 1/1 | 1/1 | 1/1 |
| `test_security` | 6/6 | 6/6 | 6/6 | 6/6 |
| In-repo QoS check scripts | 6/6 | 6/6 | 6/6 | 6/6 |
| `rosdoc2 build` | pass | pass | pass | pass |
| `ament_lint` suite | 162 tests, 0 failures, 42 skipped | 162 tests, 0 failures, 42 skipped | 154 tests, 0 failures, 40 skipped | 154 tests, 0 failures, 0 skipped |

Cross-vendor scope: the upstream suite skips service/action combinations for
**all** vendor pairs on every distro, so the cross-vendor rows cover the 8
pub/sub direction/language combinations only.

Counting notes (the full run/skip/fail decomposition of every cell was
verified against the per-test xunit/gtest XML results):

- Totals differ across columns only where the upstream suite itself differs by
  distro version (keyed-type tests, `test_event`, `best_available` QoS, and
  Foxy's older suites), never because a test was dropped. Foxy's
  `test_quality_of_service` only builds its test programs
  (`ament_add_gtest_executable`) and registers no ctest tests, hence n/a; QoS
  itself is supported on Foxy and is covered there by the in-repo QoS check
  scripts.
- `test_rclcpp`: 25 (23 on Foxy) is the actually-run count; the raw
  ctest entry count adds (installed RMW vendors − 1) × 2 upstream-skipped
  cross-RMW `node_name` variants, so it varies by environment.
- Upstream-skipped cases inside otherwise-run suites (6 loaned-message /
  allocator cases in the Lyrical gate, 5 in Jazzy) are counted in ctest's
  headline totals even though they do not run.
- `ament_lint`: the count is the eight linters' combined xunit testcase total
  (162 on Lyrical and Jazzy, 154 on Humble and Foxy); running `colcon test-result`
  over the whole build directory prints 8 more (170 / 162) because it also sums the
  ctest summary file that wraps those same eight linters. The skips (42 on
  Lyrical and Jazzy, 40 on Humble) are ament_cppcheck's performance guard for
  cppcheck 2.x (set `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` to run it). The same
  guard skips the cppcheck entry in `test_security`. Foxy's ament_cppcheck has no
  such guard (Ubuntu 20.04 ships cppcheck 1.90), so nothing is skipped there.
- Foxy's `rosdoc2 build` ran with rosdoc2 in a Python 3.10 environment and
  doxygen 1.9.8.

## Known issues

- `spin_all_fail_wait_set_clear` (rclcpp): int2DDS delivers same-participant
  samples asynchronously, so this error-injection robustness test does not
  observe the mocked wait-set clear within its short (~1 ms) window. No data
  loss or crash occurs, and this is not an RMW conformance-gate test; it is
  tracked as a known limitation.
- DDS-Security (SROS 2) is not supported yet (see `doc/security.rst`).
- Lyrical cross-vendor vs `rmw_cyclonedds_cpp`: the `WStrings` message type is
  not interoperable in either direction. This is a vendor-level wstring
  wire-format mismatch, not an int2DDS defect: on Lyrical, CycloneDDS
  serializes wstring as UTF-16 (2 bytes/char, byte-length prefix) while
  FastDDS and int2DDS use 4 bytes/char with a character-count prefix
  (verified by serializing the same message under the int2DDS, FastDDS, and
  CycloneDDS RMWs — int2DDS output is byte-identical to FastDDS; RTI Connext
  is in the same 4-byte camp because `rmw_connextdds` serializes ROS messages
  through `rosidl_typesupport_fastrtps`). Upstream `test_communication`
  acknowledges the same incompatibility by excluding `WStrings` from the
  FastDDS×CycloneDDS and Connext×CycloneDDS pairs ("CycloneDDS don't FastRTPS
  interoperate for WString"); int2DDS is simply not on that vendor-name
  exclusion list, so the case runs and fails. All other 13 message types pass
  in both directions.

## Documentation

- Installation: [doc/installation.rst](rmw_int2dds_cpp/doc/installation.rst)
- Usage: [doc/usage.rst](rmw_int2dds_cpp/doc/usage.rst)
- QoS mapping: [doc/qos_mapping.rst](rmw_int2dds_cpp/doc/qos_mapping.rst)
- Security: [doc/security.rst](rmw_int2dds_cpp/doc/security.rst) — **note: DDS-Security / SROS 2 is not supported yet**
- Examples: [examples/](rmw_int2dds_cpp/examples/)
- API docs are published at `docs.ros.org/en/{humble,jazzy}/p/rmw_int2dds_cpp/` once released.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). All contributions are subject to the
[Code of Conduct](CODE_OF_CONDUCT.md) and the project CLA
([individual](CLA-Individual.md) / [corporate](CLA-Corporate.md)).

## License

Licensed under the [Apache License 2.0](LICENSE). See [NOTICE](NOTICE) and
[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
"int2DDS" and related marks are trademarks of Intellectus Corp.; see
[TRADEMARK_POLICY.md](TRADEMARK_POLICY.md).

## Contact

Intellectus Corp. — int2dds@int2.us
