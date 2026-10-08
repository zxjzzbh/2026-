#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
# Build only the local daemon and its library. No global install or GPIO tests.
make -C "$project_root/drivers/pigpio" -j2 pigpiod
