#!/usr/bin/env bash
# Conda environments for the integrations pip cannot install: Project Chrono (pychrono, BSD-3) and
# ROS 2 Jazzy (RoboStack: Nav2, ros2_control) plus a small workspace with PWRI's OPERA packages.
# Everything here is FOSS; com3_msgs (OPERA's vessel action) carries no licence and is built for tests only.
#
#   tools/integration-envs.sh [PREFIX]          # default PREFIX=$HOME/loam-envs
#
# Then:
#   LOAM_CHRONO_PYTHON=$PREFIX/mm/envs/chrono/bin/python \
#   LOAM_ROS_RUN="env MAMBA_ROOT_PREFIX=$PREFIX/mm $PREFIX/mm/bin/micromamba run -n ros" \
#   LOAM_OPERA_WS=$PREFIX/opera_ws  pytest tests/test_chrono.py tests/test_opera.py
set -euo pipefail
PREFIX=${1:-$HOME/loam-envs}
LOAM=$(cd "$(dirname "$0")/.." && pwd)
export MAMBA_ROOT_PREFIX=$PREFIX/mm
MM=$PREFIX/mm/bin/micromamba
mkdir -p "$PREFIX/mm/bin" "$PREFIX/src" "$PREFIX/opera_ws/src"
[ -x "$MM" ] || { curl -fsSL -o "$MM" \
    https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64; chmod +x "$MM"; }

"$MM" create -y -q -n chrono -c projectchrono -c conda-forge python=3.12 pychrono=10.0.0 numpy paho-mqtt pytest
"$MM" create -y -q -n ros -c robostack-jazzy -c conda-forge python=3.12 \
    ros-jazzy-ros-base ros-jazzy-navigation2 ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-navfn-planner ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher ros-jazzy-rmw-cyclonedds-cpp \
    colcon-common-extensions ros-jazzy-ament-cmake ros-jazzy-rosidl-default-generators cmake make compilers \
    numpy paho-mqtt pytest
for env in chrono ros; do "$MM" run -n $env pip install -q --no-deps -e "$LOAM"; done

for r in zx200_ros2 ic120_ros2 com3_ros; do
    [ -d "$PREFIX/src/$r" ] || git clone -q --depth 1 "https://github.com/pwri-opera/$r" "$PREFIX/src/$r"
done
ln -sfn "$PREFIX/src/zx200_ros2/zx200_description" "$PREFIX/opera_ws/src/"
ln -sfn "$PREFIX/src/ic120_ros2/ic120_description" "$PREFIX/opera_ws/src/"
ln -sfn "$PREFIX/src/com3_ros/com3_msgs" "$PREFIX/opera_ws/src/"
(cd "$PREFIX/opera_ws" && "$MM" run -n ros colcon build --packages-select zx200_description ic120_description com3_msgs)
echo "ready: $PREFIX"
