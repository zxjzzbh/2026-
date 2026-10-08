#!/usr/bin/env bash
# Install only into this new bundle's environment using known local wheels.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
wheel_source="${1:-/home/pi/smartcar-vision-live-20260927/wheels}"
if [[ ! -d "$wheel_source" || ! -d "$project_root/test-wheels" ]]; then
  echo 'Missing offline dependency wheels' >&2
  exit 1
fi
if [[ -e "$project_root/.venv" ]]; then
  echo 'Environment already exists; inspect it before reinstalling' >&2
  exit 1
fi
python3 -m venv "$project_root/.venv"
"$project_root/.venv/bin/python" -m pip install --no-index \
  --find-links "$wheel_source" --find-links "$project_root/test-wheels" \
  'setuptools==78.1.0' 'wheel==0.48.0' 'numpy==2.2.6' \
  'opencv-python-headless==4.11.0.86' 'PyYAML==6.0.2' 'pytest==8.3.5'
"$project_root/.venv/bin/python" -m pip install --no-index --no-build-isolation --no-deps \
  --editable "$project_root/vision"
