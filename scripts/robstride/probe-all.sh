#!/bin/bash
# 探测整机 8×灵足 RS03（ID 1~8）
set -eo pipefail
# shellcheck source=../_paths.sh
source "$(cd "$(dirname "$0")/.." && pwd)/_paths.sh"
"$ROBSTRIDE_DIR/probe.sh" 01 02 03 04 05 06 07 08
