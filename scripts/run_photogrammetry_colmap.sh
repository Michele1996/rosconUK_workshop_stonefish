#!/usr/bin/env bash
# Run sparse photogrammetry reconstruction on images actually collected in Stonefish.
set -euo pipefail
IMAGES="${1:-$HOME/roscon_photogrammetry/survey/images}"
RESULTS="${2:-$HOME/roscon_photogrammetry/colmap_result}"
if ! command -v colmap >/dev/null 2>&1; then echo "COLMAP not installed. Install it separately." >&2; exit 2; fi
if [[ ! -d "$IMAGES" ]]; then echo "Image folder not found: $IMAGES" >&2; exit 2; fi
mkdir -p "$RESULTS/sparse"
# camera calibration: 960x720, HFOV 70 deg -> approximate PINHOLE intrinsics.
# NOTE: calibrate actual camera if you change the scene camera settings.
FX=$(python3 - <<'PY2'
import math
print(960./(2*math.tan(math.radians(70)/2)))
PY2
)
colmap feature_extractor --database_path "$RESULTS/database.db" --image_path "$IMAGES" \
  --ImageReader.single_camera 1 --ImageReader.camera_model PINHOLE \
  --ImageReader.camera_params "$FX,$FX,480,360"
colmap exhaustive_matcher --database_path "$RESULTS/database.db"
colmap mapper --database_path "$RESULTS/database.db" --image_path "$IMAGES" \
  --output_path "$RESULTS/sparse"
echo "COLMAP sparse models in $RESULTS/sparse; inspect with COLMAP GUI."
