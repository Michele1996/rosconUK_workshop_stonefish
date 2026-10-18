#!/usr/bin/env bash
set -euo pipefail
IMAGES="${1:-images}"
OUT="${2:-reconstruction_colmap}"
mkdir -p "$OUT/sparse"
DB="$OUT/database.db"

colmap feature_extractor \
  --database_path "$DB" \
  --image_path "$IMAGES"

colmap sequential_matcher \
  --database_path "$DB"

colmap mapper \
  --database_path "$DB" \
  --image_path "$IMAGES" \
  --output_path "$OUT/sparse"

echo "Sparse COLMAP reconstruction written to $OUT/sparse"
echo "For the workshop we stop at sparse SfM by default; dense MVS is optional."
