#!/bin/bash
# Builds the AppImage in an old system, so that it runs on as many distributions as possible.
# Needs podman or docker. Result: dist/VoxelForge-<version>-x86_64.AppImage
set -euo pipefail
cd "$(dirname "$0")/.."
ENGINE=$(command -v podman || command -v docker)
mkdir -p dist
"$ENGINE" run --rm -v "$PWD":/src:ro -v "$PWD/dist":/dist docker.io/library/ubuntu:22.04 bash -euxc '
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq --no-install-recommends g++-12 cmake ninja-build pkg-config git curl ca-certificates file patchelf \
  libturbojpeg0-dev libpng-dev libgl-dev libegl-dev libopengl-dev libwayland-dev wayland-protocols libxkbcommon-dev \
  libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev >/dev/null
export CC=gcc-12 CXX=g++-12
git clone -q --depth 1 --branch 3.4 https://github.com/glfw/glfw.git /tmp/glfw
cmake -S /tmp/glfw -B /tmp/glfw/b -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_DOCS=OFF >/dev/null
cmake --build /tmp/glfw/b >/dev/null; cmake --install /tmp/glfw/b >/dev/null
# The C++ library is linked in: a bundled old one would break graphics drivers of newer systems.
cmake -S /src -B /tmp/b -G Ninja -DCMAKE_BUILD_TYPE=Release -DVOXELFORGE_NATIVE=OFF "-DCMAKE_EXE_LINKER_FLAGS=-static-libstdc++ -static-libgcc" -DCMAKE_BUILD_RPATH_USE_ORIGIN=OFF >/dev/null
cmake --build /tmp/b
/tmp/b/voxelforge-tests | tail -1
A=/tmp/AppDir; mkdir -p $A/usr/bin $A/usr/lib $A/usr/share/applications $A/usr/share/icons/hicolor/512x512/apps
install -m755 /tmp/b/voxelforge /tmp/b/voxelforge-cli $A/usr/bin/
patchelf --set-rpath "\$ORIGIN/../lib" $A/usr/bin/voxelforge $A/usr/bin/voxelforge-cli
# Only what a system may lack. Drivers and the C library always come from the system.
for lib in libgomp.so.1 libturbojpeg.so.0 libpng16.so.16; do cp -L "$(ldconfig -p | grep -m1 "$lib" | sed "s/.*=> //")" $A/usr/lib/; done
cp /src/packaging/voxelforge.desktop $A/usr/share/applications/; cp /src/packaging/voxelforge.desktop $A/
cp /src/packaging/voxelforge.png $A/usr/share/icons/hicolor/512x512/apps/; cp /src/packaging/voxelforge.png $A/; cp /src/packaging/voxelforge.png $A/.DirIcon
cat > $A/AppRun <<"EOF"
#!/bin/sh
HERE="$(dirname "$(readlink -f "$0")")"
if [ "${1:-}" = "cli" ]; then shift; exec "$HERE/usr/bin/voxelforge-cli" "$@"; fi
exec "$HERE/usr/bin/voxelforge" "$@"
EOF
chmod +x $A/AppRun
curl -fsSL -o /tmp/appimagetool https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage
chmod +x /tmp/appimagetool; (cd /tmp && ./appimagetool --appimage-extract >/dev/null)
V=$(sed -n "s/^project(VoxelForge VERSION \([0-9.]*\).*/\1/p" /src/CMakeLists.txt)
ARCH=x86_64 /tmp/squashfs-root/AppRun --no-appstream $A /dist/VoxelForge-$V-x86_64.AppImage
ldd $A/usr/bin/voxelforge | grep -E "not found" || true
'
ls -la dist
