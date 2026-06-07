#!/usr/bin/env bash
# Idempotent native-lib vendoring for the MuJoCo Unity plugin.
# The plugin's MjBindings.cs does [DllImport("mujoco")], so the project needs a
# mujoco.dylib in Assets/Plugins. We grab it from any installed MuJoCo 3.9.0
# (the pip wheel bundles a universal x86_64+arm64 libmujoco.3.9.0.dylib).
set -euo pipefail
cd "$(dirname "$0")/.."
DEST="Assets/Plugins/mujoco.dylib"

if [ -f "$DEST" ]; then
  echo "[setup] $DEST already present ($(lipo -archs "$DEST" 2>/dev/null || echo '?'))."
  exit 0
fi

echo "[setup] locating libmujoco.3.9.0.dylib from a Python mujoco install…"
SRC=""
for PY in "${PY:-python3}" python3; do
  [ -x "$(command -v "$PY" 2>/dev/null || echo "$PY")" ] || continue
  SRC=$("$PY" - <<'PY' 2>/dev/null || true
import glob, os
try:
    import mujoco
    d = os.path.dirname(mujoco.__file__)
    hits = glob.glob(os.path.join(d, "libmujoco*.dylib"))
    print(hits[0] if hits else "")
except Exception:
    print("")
PY
)
  [ -n "$SRC" ] && break
done

if [ -z "$SRC" ] || [ ! -f "$SRC" ]; then
  echo "[setup] could not find a mujoco dylib. Install MuJoCo 3.9.0 (pip install mujoco==3.9.0)"
  echo "        or copy libmujoco.3.9.0.dylib from /Applications/MuJoCo.app/Contents/Frameworks/"
  echo "        to $DEST (renamed to mujoco.dylib)."
  exit 1
fi
cp "$SRC" "$DEST"
echo "[setup] vendored $SRC -> $DEST ($(lipo -archs "$DEST" 2>/dev/null || echo '?'))."
