#!/usr/bin/env bash
# Verify a built deb set installs cleanly on a fresh container and the RMW loads.
# Usage: packaging/verify-deb.sh <foxy|jazzy|humble|rolling> <amd64|arm64>
set -euo pipefail

DISTRO="${1:?usage: verify-deb.sh <foxy|jazzy|humble|rolling> <amd64|arm64>}"
ARCH="${2:?usage: verify-deb.sh <foxy|jazzy|humble|rolling> <amd64|arm64>}"
case "${ARCH}" in
  amd64) PLATFORM=linux/amd64 ;;
  arm64) PLATFORM=linux/arm64 ;;
  *) echo "verify supports amd64|arm64 only (armhf is best-effort, unverified)" >&2; exit 1 ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIST="${REPO_ROOT}/dist/${DISTRO}/${ARCH}"
[ -d "${DIST}" ] || { echo "no debs at ${DIST}; run build-deb first" >&2; exit 1; }

docker run --privileged --rm tonistiigi/binfmt --install all >/dev/null 2>&1 || true

docker run --rm --platform "${PLATFORM}" \
  -e ROS_DISTRO="${DISTRO}" \
  -v "${DIST}":/debs \
  "ros:${DISTRO}-ros-base" bash -c '
    set -eo pipefail
    apt-get update
    apt-get install -y \
      /debs/ros-${ROS_DISTRO}-int2dds-ffi-vendor_*.deb \
      /debs/ros-${ROS_DISTRO}-rmw-int2dds-cpp_*.deb
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u
    echo "--- installed rmw files ---"
    dpkg -L ros-${ROS_DISTRO}-rmw-int2dds-cpp | grep -E "librmw_int2dds_cpp\.so|rmw_implementation"
    echo "--- RMW load smoke test ---"
    RMW_IMPLEMENTATION=rmw_int2dds_cpp python3 -c "import rclpy; rclpy.init(); \
n=rclpy.create_node(\"deb_verify\"); print(\"RMW OK:\", n.get_name()); \
n.destroy_node(); rclpy.shutdown()"
  '
echo "VERIFY OK: ${DISTRO}/${ARCH}"
