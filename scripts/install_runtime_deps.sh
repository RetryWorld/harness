#!/bin/sh
# Install the runtime half of the release artifact. The public installer calls
# this after extraction but before executing rearguard, because the ROS-enabled
# binary is dynamically linked against Jazzy.
set -eu

mode="${1:-install}"
die() { echo "error: $*" >&2; exit 1; }
have_runtime() {
  [ -r /opt/ros/jazzy/setup.sh ] &&
    [ -d /opt/ros/jazzy ] &&
    find /opt/ros/jazzy -type f \( -name 'librosbag2_storage_mcap.so' -o -name 'librosbag2_storage_mcap.so.*' \) \
      -print -quit 2>/dev/null | grep -q .
}

if have_runtime; then
  exit 0
fi
[ "$mode" != "--check" ] || die "ROS 2 Jazzy with rosbag2 MCAP storage is not installed"
[ "$(uname -s)" = Linux ] || die "the edge runtime supports Linux only"
[ -r /etc/os-release ] || die "cannot identify the operating system"
# shellcheck disable=SC1091
. /etc/os-release
[ "${ID:-}" = ubuntu ] && [ "${VERSION_CODENAME:-}" = noble ] ||
  die "automatic ROS 2 Jazzy installation requires Ubuntu 24.04 (noble)"
command -v apt-get >/dev/null 2>&1 || die "apt-get is required"
command -v curl >/dev/null 2>&1 || die "curl is required"
command -v sudo >/dev/null 2>&1 || die "sudo is required to install ROS runtime packages"

export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y ca-certificates curl software-properties-common
sudo add-apt-repository -y universe

if [ ! -e /etc/apt/sources.list.d/ros2.sources ]; then
  ros_source_version="${ROS_APT_SOURCE_VERSION:-1.3.0}"
  ros_source_deb="$(mktemp /tmp/ros2-apt-source.XXXXXX.deb)"
  trap 'rm -f "$ros_source_deb"' EXIT
  curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.noble_all.deb" \
    -o "$ros_source_deb"
  sudo apt-get install -y "$ros_source_deb"
fi

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-base \
  ros-jazzy-rclcpp \
  ros-jazzy-rosbag2-cpp \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-trajectory-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-rosgraph-msgs \
  ros-jazzy-rmw-cyclonedds-cpp

have_runtime || die "ROS installation completed without the MCAP storage plugin"
