#!/usr/bin/env bash
set -eo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_base="${ROS2_010_BUILD_BASE:-/tmp/ros2_010_build}"
install_base="${ROS2_010_INSTALL_BASE:-/tmp/ros2_010_install}"
log_base="${ROS2_010_LOG_BASE:-/tmp/ros2_010_log}"

cd "${repository_root}"
source /opt/ros/jazzy/setup.bash

python3 -m pytest -q
python3 -m compileall -q models tests ros2_ws/src

colcon --log-base "${log_base}" build \
  --base-paths ros2_ws/src \
  --build-base "${build_base}" \
  --install-base "${install_base}" \
  --symlink-install

source "${install_base}/setup.bash"

# These checks also run through colcon below. Running the source linters
# explicitly makes their scope and failure mode visible before launch tests.
ctest \
  --test-dir "${build_base}/robot_supervisor_interfaces" \
  --output-on-failure \
  -R "lint|flake8|pep257|xmllint"
(
  cd ros2_ws/src/robot_supervisor
  python3 -m pytest -q -m linter
)
(
  cd ros2_ws/src/robot_supervisor_sim
  python3 -m pytest -q -m linter
)

colcon --log-base "${log_base}" test \
  --base-paths ros2_ws/src \
  --build-base "${build_base}" \
  --install-base "${install_base}" \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base "${build_base}" \
  --verbose

# Exercise the real multi-process launch tests explicitly as well as through
# colcon so a launch-test collection regression cannot be hidden by unit lint.
(
  cd ros2_ws/src/robot_supervisor_sim
  python3 -m pytest -q -m launch_test
)

PYTHONPATH="${repository_root}/ros2_ws/src/robot_supervisor" \
  python3 -m robot_supervisor.generate_evidence \
    --report /tmp/ros2_010_synthetic_validation_report.json \
    --trace /tmp/ros2_010_synthetic_message_trace.csv

cmp \
  data/processed/ros2_010_synthetic_validation_report.json \
  /tmp/ros2_010_synthetic_validation_report.json
cmp \
  data/processed/ros2_010_synthetic_message_trace.csv \
  /tmp/ros2_010_synthetic_message_trace.csv

if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git diff --check
fi
