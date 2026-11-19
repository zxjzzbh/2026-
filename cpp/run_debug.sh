#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
task="${1:-observe}"
if [[ $# -gt 0 ]]; then shift; fi
case "$task" in observe|zebra|traffic|obstacle|parking) ;; *) echo 'task: observe zebra traffic obstacle parking' >&2; exit 2;; esac
exec ./build/smartcar --task "$task" --config config/race.json "$@"
