#!/usr/bin/env bash
# Usage: from ~/ie4060_ws/src/ie4060_maze: bash scripts/install_project.sh
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS Jazzy not installed in /opt/ros/jazzy'; exit 1
fi
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
python3 "$ROOT/scripts/prepare_configs.py" --initial-pose auto
python3 "$ROOT/scripts/generate_reference_map.py"
cd "$ROOT/../.."
colcon build --symlink-install --packages-select ie4060_maze
printf '\nInstalled ie4060_maze. In NEW terminals:\n'
printf '  source /opt/ros/jazzy/setup.bash\n'
printf '  source ~/ie4060_ws/install/setup.bash\n'
printf '  export TURTLEBOT3_MODEL=burger\n'
