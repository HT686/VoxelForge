#!/bin/sh
# VoxelForge setup: installs the program for this user. Needs nothing but a shell and tar.
#   sh VoxelForge-Setup.sh              install
#   sh VoxelForge-Setup.sh --uninstall  remove it again, projects stay
#   sh VoxelForge-Setup.sh --no-usb-rule  skip the part that needs administrator rights
set -eu
OPT="$HOME/.local/opt/voxelforge"; BIN="$HOME/.local/bin"; APPS="$HOME/.local/share/applications"; ICONS="$HOME/.local/share/icons/hicolor/512x512/apps"; RULE=/etc/udev/rules.d/99-voxelforge.rules
USB_RULE=1; ACTION=install
for a in "$@"; do case "$a" in --uninstall) ACTION=uninstall ;; --no-usb-rule) USB_RULE=0 ;; -h|--help) sed -n 2,5p "$0"; exit 0 ;; *) echo "Unknown option: $a"; exit 2 ;; esac; done
root() { if [ "$(id -u)" = 0 ]; then "$@"; elif command -v sudo >/dev/null; then sudo "$@"; elif command -v pkexec >/dev/null; then pkexec "$@"; else return 1; fi; }
refresh() {
    command -v update-desktop-database >/dev/null && update-desktop-database -q "$APPS" 2>/dev/null || true
    command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -q -t "$HOME/.local/share/icons/hicolor" 2>/dev/null || true
    command -v kbuildsycoca6 >/dev/null && kbuildsycoca6 --noincremental >/dev/null 2>&1 || true
}
if [ "$ACTION" = uninstall ]; then
    rm -rf "$OPT"; rm -f "$BIN/voxelforge" "$BIN/voxelforge-cli" "$APPS/voxelforge.desktop" "$ICONS/voxelforge.png"; refresh
    if [ -e "$RULE" ] && [ "$USB_RULE" = 1 ]; then echo "Removing the USB rule needs administrator rights."; root rm -f "$RULE" && root udevadm control --reload-rules || echo "Not removed: $RULE"; fi
    echo "VoxelForge removed. Projects and settings stay in place."; exit 0
fi
mkdir -p "$OPT" "$BIN" "$APPS" "$ICONS"
LINE=$(awk '/^__PAYLOAD__$/ {print NR+1; exit}' "$0")
tail -n +"$LINE" "$0" | tar xzf - -C "$OPT"
chmod +x "$OPT"/VoxelForge.AppImage
# Unpacked, so that it runs without FUSE and starts faster.
( cd "$OPT" && rm -rf app && ./VoxelForge.AppImage --appimage-extract >/dev/null && mv squashfs-root app && rm -f VoxelForge.AppImage )
ln -sf "$OPT/app/AppRun" "$BIN/voxelforge"
printf '#!/bin/sh\nexec "%s/app/AppRun" cli "$@"\n' "$OPT" > "$BIN/voxelforge-cli"; chmod +x "$BIN/voxelforge-cli"
cp "$OPT/voxelforge.png" "$ICONS/voxelforge.png"
sed "s|^Exec=.*|Exec=$OPT/app/AppRun|" "$OPT/voxelforge.desktop" > "$APPS/voxelforge.desktop"
refresh
if [ "$USB_RULE" = 1 ] && [ ! -e "$RULE" ]; then
    echo "The scanner needs a USB rule once. This asks for administrator rights."
    if root cp "$OPT/99-voxelforge.rules" "$RULE" && root udevadm control --reload-rules && root udevadm trigger --subsystem-match=usb; then echo "USB rule installed. Plug the scanner in again if it is connected."
    else echo "No USB rule installed. Without it the scanner may not be found. Copy it later with:"; echo "  sudo cp $OPT/99-voxelforge.rules $RULE"; fi
fi
echo "VoxelForge installed. Start it from the application menu or with: voxelforge"
case ":$PATH:" in *":$BIN:"*) ;; *) echo "Note: $BIN is not on your PATH, the command needs the full path." ;; esac
exit 0
__PAYLOAD__
