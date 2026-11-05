#!/bin/bash
# Builds the Flatpak bundle. Needs flatpak and org.flatpak.Builder from Flathub.
# Result: dist/VoxelForge-<version>-x86_64.flatpak
set -euo pipefail
cd "$(dirname "$0")/.."
V=$(sed -n 's/^project(VoxelForge VERSION \([0-9.]*\).*/\1/p' CMakeLists.txt)
mkdir -p dist
flatpak run --filesystem="$PWD" org.flatpak.Builder --user --install-deps-from=flathub --force-clean --repo=dist/repo dist/build packaging/io.github.ht686.VoxelForge.yml
flatpak build-bundle --runtime-repo=https://flathub.org/repo/flathub.flatpakrepo dist/repo "dist/VoxelForge-$V-x86_64.flatpak" io.github.ht686.VoxelForge
rm -rf dist/repo dist/build
ls -la dist
