#!/usr/bin/env bash
###############################################################################
# install.sh — megarover-v3-fastlio2
#
# Spreads this repository's files into the ROS 2 workspace layout used by the
# MegaRover V3 + FAST-LIO2 project:
#
#   fastlio_ws/src/fastlio2_pose/                        -> ~/fastlio_ws/src/fastlio2_pose/
#   fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml      -> ~/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml
#   ros2_ws/src/rover_control/                           -> ~/ros2_ws/src/rover_control/
#
# The mid360.yaml REPLACES the default config shipped with Ericsii/FAST_LIO_ROS2
# (that is why this script is meant to be run AFTER you have already cloned and
# built FAST-LIO2 per the README -- it overrides the default library config with
# the tuning used on this robot).
#
# imgs/, README.md, LICENSE and this script stay in the repository.
#
# Usage:
#   ./install.sh              copy files only (idempotent; replaced cleanly)
#   ./install.sh --build      copy + colcon build fastlio_ws (fastlio2_pose) and
#                             ros2_ws (rover_control)
#   ./install.sh --help       show this header
#
# Custom locations / ROS distro via environment variables:
#   FASTLIO_WS=~/my_fastlio_ws ROS2_WS=~/my_ros2_ws ROS_DISTRO=jazzy ./install.sh --build
###############################################################################
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FASTLIO_WS="${FASTLIO_WS:-$HOME/fastlio_ws}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

DO_BUILD=0
case "${1:-}" in
  "") ;;
  --build) DO_BUILD=1 ;;
  -h|--help) sed -n '2,27p' "$0"; exit 0 ;;
  *) echo "Unknown option: '$1'  (valid: --build, --help)" >&2; exit 1 ;;
esac

info() { printf '\033[1;32m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[ warn ]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[ error]\033[0m %s\n' "$*" >&2; }

copy_tree() { # copy_tree <src> <dst>
  if command -v rsync >/dev/null 2>&1; then
    rsync -a "$1" "$2"
  else
    cp -a "$1" "$2"
  fi
}

###############################################################################
# 1. Sanity checks
###############################################################################
info "repo:   $REPO_ROOT"
info "target: FASTLIO_WS=$FASTLIO_WS  ROS2_WS=$ROS2_WS  ROS_DISTRO=$ROS_DISTRO"

MISSING=0
for p in fastlio_ws/src/fastlio2_pose \
         fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml \
         ros2_ws/src/rover_control; do
  if [ ! -e "$REPO_ROOT/$p" ]; then
    err "missing from repository: $p"
    MISSING=1
  fi
done
[ "$MISSING" -eq 1 ] && { err "incomplete clone — aborting."; exit 1; }

# FAST-LIO2 / livox driver must be installed separately first.
[ -d "$FASTLIO_WS/src/FAST_LIO_ROS2" ] \
  || warn "FAST_LIO_ROS2 not cloned in $FASTLIO_WS/src — clone it first (see README step 2)."
[ -e "$FASTLIO_WS/src/FAST_LIO_ROS2/config/mid360.yaml" ] \
  || warn "no stock mid360.yaml found — install.sh will add ours anyway."
[ -d "/opt/ros/$ROS_DISTRO" ] \
  || warn "/opt/ros/$ROS_DISTRO not found — install ROS 2 $ROS_DISTRO before --build"

###############################################################################
# 2. Create workspace skeletons and copy files
###############################################################################
mkdir -p "$FASTLIO_WS/src" "$ROS2_WS/src"

# --- fastlio2_pose package --------------------------------------------------
rm -rf "${FASTLIO_WS:?}/src/fastlio2_pose"
copy_tree "$REPO_ROOT/fastlio_ws/src/fastlio2_pose" "$FASTLIO_WS/src/fastlio2_pose"
info "fastlio2_pose/ -> $FASTLIO_WS/src/fastlio2_pose/"

# --- FAST-LIO2 mid360.yaml config (replaces the default library config) -----
mkdir -p "$FASTLIO_WS/src/FAST_LIO_ROS2/config"
install -m 0644 \
  "$REPO_ROOT/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml" \
  "$FASTLIO_WS/src/FAST_LIO_ROS2/config/mid360.yaml"
info "mid360.yaml -> $FASTLIO_WS/src/FAST_LIO_ROS2/config/mid360.yaml"

# --- ros2_ws package --------------------------------------------------------
rm -rf "${ROS2_WS:?}/src/rover_control"
copy_tree "$REPO_ROOT/ros2_ws/src/rover_control" "$ROS2_WS/src/rover_control"
info "rover_control/ -> $ROS2_WS/src/rover_control/"

# sanity: files landed where the launch file expects them
for f in "$FASTLIO_WS/src/fastlio2_pose/src/fastlio2_pose_node.cpp" \
         "$ROS2_WS/src/rover_control/launch/rover_bringup_launch.py" \
         "$ROS2_WS/src/rover_control/rviz/rviz_config.rviz"; do
  [ -e "$f" ] || warn "expected file not found after copy: $f"
done

###############################################################################
# 3. Optional: colcon build both workspaces
###############################################################################
if [ "$DO_BUILD" -eq 1 ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"

  info "colcon build: $FASTLIO_WS (fastlio2_pose)"
  (cd "$FASTLIO_WS" && colcon build \
      --packages-select fastlio2_pose \
      --symlink-install)

  info "colcon build: $ROS2_WS (rover_control)"
  (cd "$ROS2_WS" && colcon build \
      --packages-select rover_control \
      --symlink-install)
fi

###############################################################################
# 4. Summary
###############################################################################
cat <<EOF

$(info "done.")
Workspace layout now:
  $FASTLIO_WS/src/fastlio2_pose/
  $FASTLIO_WS/src/FAST_LIO_ROS2/config/mid360.yaml
  $ROS2_WS/src/rover_control/

NOT installed by this script (install separately, see README):
  ~/Livox-SDK2                      Livox SDK2            (step 1)
  ~/fastlio_ws/src/livox_ros_driver2   Livox ROS 2 driver (step 2)
  ~/fastlio_ws/src/FAST_LIO_ROS2        FAST-LIO2 algorithm (step 2)
  ~/uros_ws                      micro-ROS Agent (step 0)

Next steps:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source ~/fastlio_ws/install/setup.bash
  source ~/ros2_ws/install/setup.bash

Build FAST-LIO2 + livox driver are expected to be rebuilt if you regenerate
them afterwards; fastlio2_pose and rover_control are built by --build.

Run the full system:
  ros2 launch rover_control rover_bringup_launch.py
EOF
