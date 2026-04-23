#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS setup scripts may read unset variables internally, so temporarily disable nounset.
set +u
source /opt/ros/jazzy/setup.bash

if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  echo "[astar_demo] install/setup.bash 不存在，先执行 colcon build"
  cd "${WORKSPACE_DIR}"
  colcon build --symlink-install
fi

source "${WORKSPACE_DIR}/install/setup.bash"
set -u

echo "[astar_demo] ROS bridge 将监听 0.0.0.0:9090"
echo "[astar_demo] 手机端请连接主机局域网 IP 和端口 9090"

LAUNCH_ARGS=("$@")
has_rviz_arg=false
for arg in "${LAUNCH_ARGS[@]}"; do
  if [[ "${arg}" == use_rviz:=* ]]; then
    has_rviz_arg=true
    break
  fi
done

if [ "${has_rviz_arg}" = false ]; then
  LAUNCH_ARGS+=(use_rviz:=true)
fi

exec ros2 launch astar_demo demo.launch.py "${LAUNCH_ARGS[@]}"
