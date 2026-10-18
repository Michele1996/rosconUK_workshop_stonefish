#!/usr/bin/env bash
set -euo pipefail
IMAGES="${1:-images}"
OUT="${2:-reconstruction_glomap}"
mkdir -p "$OUT"
DB="$OUT/database.db"

colmap feature_extractor --database_path "$DB" --image_path "$IMAGES"
colmap sequential_matcher --database_path "$DB"
glomap mapper --database_path "$DB" --image_path "$IMAGES" --output_path "$OUT/sparse"

echo "GLOMAP reconstruction written to $OUT/sparse"
