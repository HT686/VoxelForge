#!/bin/bash
# Packs the AppImage into a single self-installing script. Run build-appimage.sh first.
set -euo pipefail
cd "$(dirname "$0")/.."
V=$(sed -n 's/^project(VoxelForge VERSION \([0-9.]*\).*/\1/p' CMakeLists.txt)
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
cp "dist/VoxelForge-$V-x86_64.AppImage" "$T/VoxelForge.AppImage"; cp packaging/voxelforge.png packaging/voxelforge.desktop packaging/99-voxelforge.rules "$T/"
OUT="dist/VoxelForge-$V-Setup-x86_64.sh"
{ cat packaging/setup-header.sh; tar czf - -C "$T" .; } > "$OUT"; chmod +x "$OUT"
ls -la "$OUT"
