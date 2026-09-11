#!/usr/bin/env bash
# Host-side launcher: build the two int2DDS .debs for one (distro, arch).
# Runs the build inside a matching ros:<distro>-ros-base container (native or QEMU).
#
# Usage: packaging/build-deb.sh <foxy|jazzy|humble|rolling> <amd64|arm64|armhf>
#
# On Windows Git Bash, prefix with MSYS_NO_PATHCONV=1 so docker -v mount paths
# are not mangled:  MSYS_NO_PATHCONV=1 packaging/build-deb.sh jazzy arm64
set -euo pipefail

DISTRO="${1:?usage: build-deb.sh <foxy|jazzy|humble|rolling> <amd64|arm64|armhf>}"
ARCH="${2:?usage: build-deb.sh <foxy|jazzy|humble|rolling> <amd64|arm64|armhf>}"

case "${ARCH}" in
  amd64) PLATFORM=linux/amd64 ;;
  arm64) PLATFORM=linux/arm64 ;;
  armhf) PLATFORM=linux/arm/v7 ;;
  *) echo "unknown arch '${ARCH}' (expected amd64|arm64|armhf)" >&2; exit 1 ;;
esac
case "${DISTRO}" in
  foxy|jazzy|humble|rolling) ;;
  *) echo "unknown distro '${DISTRO}' (expected foxy|jazzy|humble|rolling)" >&2; exit 1 ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# int2dds_ffi_vendor is a package inside this repository (not a sibling checkout).
if [ ! -f "${REPO_ROOT}/int2dds_ffi_vendor/package.xml" ]; then
  echo "int2dds_ffi_vendor/package.xml missing from ${REPO_ROOT}; incomplete checkout?" >&2
  exit 1
fi

# QEMU for cross-arch emulation (no-op when building the native arch).
docker run --privileged --rm tonistiigi/binfmt --install all >/dev/null 2>&1 || true

docker run --rm --platform "${PLATFORM}" \
  -e ROS_DISTRO="${DISTRO}" \
  -e RMW_REPO=/ws/rmw_int2dds \
  -v "${REPO_ROOT}":/ws/rmw_int2dds \
  "ros:${DISTRO}-ros-base" \
  bash /ws/rmw_int2dds/packaging/in-container-build.sh
