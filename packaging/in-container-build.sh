#!/usr/bin/env bash
# Build the int2DDS vendor + rmw .debs. Runs INSIDE a ros:<distro>-ros-base container.
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"
RMW_REPO="${RMW_REPO:?RMW_REPO must be set (mount of rmw_int2dds/)}"
# Both packages live in this repository.
VENDOR_SRC="${RMW_REPO}/int2dds_ffi_vendor"
RMW_SRC="${RMW_REPO}/rmw_int2dds_cpp"
ARCH="$(dpkg --print-architecture)"
OUT="${RMW_REPO}/dist/${ROS_DISTRO}/${ARCH}"

echo "::group::install build tooling"
apt-get update
apt-get install -y python3-bloom fakeroot debhelper dh-python python3-yaml ca-certificates
echo "::endgroup::"

echo "::group::register rosdep sources"
# The base image may not have rosdep initialized; init is idempotent.
rosdep init 2>/dev/null || true
mkdir -p /etc/ros/rosdep/sources.list.d
sed "s/@ROS_DISTRO@/${ROS_DISTRO}/g" \
  "${RMW_REPO}/packaging/rosdep/int2dds.yaml.in" > /tmp/int2dds-rosdep.yaml
echo "yaml file:///tmp/int2dds-rosdep.yaml" \
  > /etc/ros/rosdep/sources.list.d/50-int2dds.list
# Foxy is end-of-life, and a plain `rosdep update` drops EOL distros, which
# would leave every ros-foxy-* key unresolvable for bloom and rosdep install.
rosdep update --include-eol-distros
echo "::endgroup::"

mkdir -p "${OUT}"
# ROS's setup.bash references unset variables internally (e.g.
# AMENT_TRACE_SETUP_FILES), which trips `set -u`. Relax -u only for the source.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# Build a package to a .deb. Args: <source_dir> [skip_keys].
# Builds in a SCRATCH copy under /tmp so the mounted host working tree is never
# polluted with generated debian/ or obj-*/ directories.
build_pkg () {
  local src="$1"
  local skip_keys="${2:-}"
  local name work
  name="$(basename "${src}")"
  work="/tmp/debbuild/${name}"
  rm -rf "${work}"
  mkdir -p "${work}"
  cp -a "${src}/." "${work}/"
  ( cd "${work}"
    rm -rf debian obj-* .obj-*
    bloom-generate rosdebian --ros-distro "${ROS_DISTRO}"
    if [ -n "${skip_keys}" ]; then
      rosdep install --from-paths . --ignore-src -y \
        --rosdistro "${ROS_DISTRO}" --skip-keys "${skip_keys}"
    else
      rosdep install --from-paths . --ignore-src -y --rosdistro "${ROS_DISTRO}"
    fi
    fakeroot debian/rules binary )
  # bloom writes the .deb to the scratch parent (/tmp/debbuild)
  mv /tmp/debbuild/ros-"${ROS_DISTRO}"-*.deb "${OUT}"/
}

echo "::group::build vendor deb"
build_pkg "${VENDOR_SRC}"           # CMake downloads libint2dds_ffi.so here → baked into deb
echo "::endgroup::"

echo "::group::install vendor deb (so rmw can find/link it)"
apt-get install -y "${OUT}"/ros-"${ROS_DISTRO}"-int2dds-ffi-vendor_*.deb
echo "::endgroup::"

echo "::group::build rmw deb"
# Skip this key at rosdep-install time:
#  - int2dds_ffi_vendor: already installed above as a local .deb (not in any apt repo)
build_pkg "${RMW_SRC}" "int2dds_ffi_vendor"
echo "::endgroup::"

echo "Artifacts in ${OUT}:"
ls -la "${OUT}"
