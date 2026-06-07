#!/usr/bin/env bash
# Keep the ORBIT teleop link alive WITHOUT touching the proximity sensor (no view
# side-effects). Every few seconds, while the Quest is reachable over adb, it:
#   - re-establishes the 7 adb-reverse tunnels (they drop on app/USB cycles -> frozen data)
#   - (re)starts the NetMQ->UDP bridge if it died
#   - relaunches the ORBIT app if it exited (e.g. headset taken off)
# Run detached:  nohup Tools/keepalive.sh >~/orbit_keepalive.log 2>&1 & disown
# Stop:          pkill -f keepalive.sh
PY="${PY:-python3}"     # any python3 with pyzmq (pip install -r Tools/requirements.txt)
BRIDGE="$HOME/Desktop/orca-teleop-unity/Tools/orbit_to_unity.py"
LOG="$HOME/orbit_bridge.log"
PKG="com.ORBIT.Teleoperation"
PORTS="8087 8088 8122 8123 8200 8095 8100"

echo "$(date +%T) [keepalive] started (no proximity changes)"
while true; do
  if adb get-state 2>/dev/null | grep -q device; then
    for p in $PORTS; do adb reverse tcp:$p tcp:$p >/dev/null 2>&1; done   # keep tunnels alive
    if ! pgrep -f orbit_to_unity.py >/dev/null 2>&1; then
      nohup "$PY" -u "$BRIDGE" --debug >>"$LOG" 2>&1 &
      echo "$(date +%T) [keepalive] (re)started bridge"
    fi
    # NOTE: the ORBIT app is launched BY THE USER from the headset (so it opens immersive
    # = the orange screen). adb-launching it lands it backgrounded/flat, so we don't.
  else
    echo "$(date +%T) [keepalive] no adb device (Quest asleep/unplugged) — waiting"
  fi
  sleep 5
done
