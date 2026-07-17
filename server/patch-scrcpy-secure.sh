#!/bin/bash
# Patch a vanilla scrcpy-server v4.1 jar so its screen-mirror VirtualDisplay is
# created with VIRTUAL_DISPLAY_FLAG_SECURE (0x4) | VIRTUAL_DISPLAY_FLAG_AUTO_MIRROR
# (0x10). A secure output composites FLAG_SECURE app windows (bank / login / pay
# screens) instead of blanking them, so they show up in the remote view instead
# of turning black.
#
# The patched server MUST run as the system uid (1000): CAPTURE_SECURE_VIDEO_OUTPUT
# is a signature|role permission held only by the platform "android" package.
# On Android 16 scrcpy mirrors via DisplayManager (SurfaceControl.createDisplay is
# gone), so the two edits are:
#   1. DisplayManager.createVirtualDisplay(name,w,h,displayId,surface) — the hidden
#      5-arg mirror wrapper — is redirected to the public 6-arg createNewVirtualDisplay
#      with flags 0x14 (SECURE|AUTO_MIRROR). AUTO_MIRROR always tracks the default
#      display, so the displayId argument is dropped (the ReDroid farm only uses
#      display 0; a non-default display_id would be ignored).
#   2. FakeContext package "com.android.shell" -> "android", or the secure
#      createVirtualDisplay binder call fails "packageName must match the calling uid".
#
# Requires: baksmali/smali (apt "smali"), a JRE, python3, unzip, zip.
set -euo pipefail

JAR="${1:?usage: patch-scrcpy-secure.sh <scrcpy-server.jar>}"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

unzip -o -q "$JAR" classes.dex -d "$work"
baksmali d "$work/classes.dex" -o "$work/smali"

python3 - "$work/smali" <<'PY'
import re, sys, pathlib

base = pathlib.Path(sys.argv[1])

dm = base / "com/genymobile/scrcpy/wrappers/DisplayManager.smali"
body = (
    ".method public createVirtualDisplay(Ljava/lang/String;IIILandroid/view/Surface;)Landroid/hardware/display/VirtualDisplay;\n"
    "    .registers 16\n"
    "    .annotation system Ldalvik/annotation/Throws;\n"
    "        value = {\n"
    "            Ljava/lang/Exception;\n"
    "        }\n"
    "    .end annotation\n\n"
    "    move-object v0, p0\n\n"
    "    move-object v1, p1\n\n"
    "    move v2, p2\n\n"
    "    move v3, p3\n\n"
    "    const/16 v4, 0x140\n\n"
    "    move-object v5, p5\n\n"
    "    const/16 v6, 0x14\n\n"
    "    invoke-virtual/range {v0 .. v6}, Lcom/genymobile/scrcpy/wrappers/DisplayManager;->createNewVirtualDisplay(Ljava/lang/String;IIILandroid/view/Surface;I)Landroid/hardware/display/VirtualDisplay;\n\n"
    "    move-result-object v0\n\n"
    "    return-object v0\n"
    ".end method"
)
text, n = re.subn(
    r"\.method public createVirtualDisplay\(Ljava/lang/String;IIILandroid/view/Surface;\)Landroid/hardware/display/VirtualDisplay;.*?\.end method",
    lambda _m: body, dm.read_text(), flags=re.S)
assert n == 1, f"createVirtualDisplay wrapper: matched {n}, want 1"
dm.write_text(text)

fc = base / "com/genymobile/scrcpy/FakeContext.smali"
text, n = re.subn(r'"com\.android\.shell"', '"android"', fc.read_text())
assert n >= 1, "FakeContext package string not found"
fc.write_text(text)

print(f"patched createVirtualDisplay(secure|auto_mirror) + FakeContext->android ({n} refs)")
PY

rm -f "$work/classes.dex"
smali a "$work/smali" -o "$work/classes.dex"
zip -j -q "$JAR" "$work/classes.dex"
echo "patched: $JAR"
