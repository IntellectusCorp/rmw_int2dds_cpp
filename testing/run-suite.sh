#!/usr/bin/env bash
# Host-side launcher: run the full RMW test matrix for one distro inside a
# matching ros:<distro>-ros-base container.
#
# Usage: testing/run-suite.sh <foxy|humble|jazzy|lyrical|rolling>
#
# Source selection: a local branch named after the distro wins over
# origin/<distro>, so a checkout carrying unpushed work is what gets tested.
# The chosen commit is recorded in <workdir>/out/source-commit.txt.
#
# Env:
#   RMW_TEST_WORKDIR  parent of the per-distro work tree (default /tmp/rmw_int2dds_test)
set -euo pipefail

DISTRO="${1:?usage: run-suite.sh <foxy|humble|jazzy|lyrical|rolling>}"
case "${DISTRO}" in
  foxy|humble|jazzy|lyrical|rolling) ;;
  *) echo "unknown distro '${DISTRO}' (expected foxy|humble|jazzy|lyrical|rolling)" >&2; exit 1 ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${RMW_TEST_WORKDIR:-/tmp/rmw_int2dds_test}/${DISTRO}"

if git -C "${REPO_ROOT}" show-ref --verify --quiet "refs/heads/${DISTRO}"; then
  REF="${DISTRO}"
elif git -C "${REPO_ROOT}" show-ref --verify --quiet "refs/remotes/origin/${DISTRO}"; then
  REF="origin/${DISTRO}"
else
  echo "no branch '${DISTRO}' locally or on origin" >&2
  exit 1
fi

# A stale build tree silently mixes distros; start each run from scratch.
# An interrupted run leaves root-owned build output behind, so the wipe happens
# in a throwaway root container rather than needing sudo on the host.
if [ -e "${WORK}" ]; then
  rm -rf "${WORK}" 2>/dev/null || \
    docker run --rm -v "$(dirname "${WORK}")":/wipe alpine:3 rm -rf "/wipe/${DISTRO}"
fi
mkdir -p "${WORK}/rmw_ws/src/rmw_int2dds" "${WORK}/test_ws/src" "${WORK}/out"

git -C "${REPO_ROOT}" archive "${REF}" | tar -x -C "${WORK}/rmw_ws/src/rmw_int2dds"
{
  echo "distro:  ${DISTRO}"
  echo "ref:     ${REF}"
  echo "commit:  $(git -C "${REPO_ROOT}" rev-parse "${REF}")"
  echo "subject: $(git -C "${REPO_ROOT}" log -1 --format=%s "${REF}")"
  echo "started: $(date -Is)"
} | tee "${WORK}/out/source-commit.txt"

if [ ! -f "${WORK}/rmw_ws/src/rmw_int2dds/int2dds_ffi_vendor/package.xml" ]; then
  echo "int2dds_ffi_vendor missing from ${REF}; incomplete branch?" >&2
  exit 1
fi

# --shm-size: the default 64 MB /dev/shm starves Fast DDS' shared-memory
# transport, which the cross-vendor cases rely on.
docker run --rm \
  --shm-size=2g \
  -e ROS_DISTRO="${DISTRO}" \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
  -v "${WORK}":/ws \
  -v "${REPO_ROOT}/testing":/testing:ro \
  "ros:${DISTRO}-ros-base" \
  bash /testing/in-container-test.sh 2>&1 | tee "${WORK}/out/run.log"
