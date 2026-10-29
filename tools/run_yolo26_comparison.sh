#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${YOLO_VENV:-/home/rob/yolovision_ws/.venv}"
BACKUPS="$WORKSPACE/src/yolo_vision/models/yolo26_backups"
mkdir -p "$BACKUPS" "$WORKSPACE/output/yolo26_comparison_20261019/.cache"
for scale in n s m l x; do
    weight="$BACKUPS/yolo26${scale}-seg.pt"
    if [[ ! -s "$weight" ]]; then
        curl -fL --retry 3 --connect-timeout 20 -o "$weight.part" \
            "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26${scale}-seg.pt"
        mv "$weight.part" "$weight"
    fi
done
export PYTHONNOUSERSITE=1
export PYTHONPATH="$VENV/lib/python3.10/site-packages:/home/rob/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="$WORKSPACE/output/yolo26_comparison_20261019/.cache/matplotlib"
export YOLO_CONFIG_DIR="$WORKSPACE/output/yolo26_comparison_20261019/.cache/ultralytics"
exec "$VENV/bin/python" "$WORKSPACE/tools/compare_yolo26_images.py" \
    --output "$WORKSPACE/output/yolo26_comparison_20261019/all_scales"
