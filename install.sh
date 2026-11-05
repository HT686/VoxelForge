#!/bin/bash
# Baut VoxelForge und installiert es fuer den aktuellen Benutzer (ohne Root-Rechte).
set -e
cd "$(dirname "$0")"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$HOME/.local"
cmake --build build -j"$(nproc)"
cmake --install build
install -Dm644 packaging/voxelforge.desktop "$HOME/.local/share/applications/voxelforge.desktop"
install -Dm644 packaging/voxelforge.png "$HOME/.local/share/icons/hicolor/512x512/apps/voxelforge.png"
# Damit der Desktop das Symbol sofort kennt.
command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -q -t "$HOME/.local/share/icons/hicolor" 2>/dev/null || true
command -v update-desktop-database >/dev/null && update-desktop-database -q "$HOME/.local/share/applications" 2>/dev/null || true
command -v kbuildsycoca6 >/dev/null && kbuildsycoca6 --noincremental >/dev/null 2>&1 || true
echo "Fertig. VoxelForge steht im Anwendungsmenue oder startet mit: voxelforge"
