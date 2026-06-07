# Bimanual-Unity — YAM + ORCA teleop in Unity (MuJoCo physics)

The bimanual **i2rt YAM** arms + **ORCA** hands scene, running in **Unity** with
**MuJoCo** physics (the embedded `org.mujoco` plugin), teleoped from a **Meta Quest**
via the **ORBIT** hand-tracking app. Your wrists drive the arms (Cartesian IK); your
fingers drive the hands. Desktop-first (macOS); the teleop code is reusable for an
on-device VR build later.

The whole robot is the *exact* `bimanual_yam.xml` — AgileX stand + two 6-DoF YAM arms
+ two 17-DoF tendon-coupled ORCA hands, 46 actuators — not a rebuild.

```
Quest 3 (ORBIT app)                                Mac (this Unity project)
  hand/wrist/head ──NetMQ/adb──► Tools/orbit_to_unity.py ──UDP:9101──► OrbitUdpReceiver
                                 (binds ORBIT ports,                     │ MjFrame  (S·M·S, Unity-LH→robot)
                                  re-emits tagged UDP)                   │ PoseRetargeter (delta-anchor;
                                                                        │   position=world, orientation=body)
                                                                        │ ArmIK (two-stage DLS: shoulder=pos,
                                                                        │   wrist=orientation) → arm actuators
                                                                        │ HandRetarget → 17 hand actuators/side
                                                         MjScene.FixedUpdate → mj_step → Unity renders
```

## Prerequisites
- **Unity 6000.3.9f1** (install via Unity Hub) + a Unity license (Personal is free).
- **Python 3** with **pyzmq** for the host bridge: `pip install -r Tools/requirements.txt`.
- **adb** (Android platform-tools) for the Quest over USB: `brew install android-platform-tools`.
- A Quest with the **ORBIT** app (`com.ORBIT.Teleoperation`) installed, and hand-tracking on.
- `Assets/Plugins/mujoco.dylib` (the native MuJoCo lib) is committed. If it ever goes
  missing, run `Tools/setup.sh` to re-vendor it.

## First-time setup (in Unity)
1. Open this folder as a project in Unity Hub (it picks 6000.3.9f1). First open imports
   the `org.mujoco` plugin + ~170 MB of meshes — give it a few minutes; Console should
   end with **no errors**.
2. **Assets ▸ Import MuJoCo Scene** → `Assets/Mujoco/scenes/v2/bimanual_yam.xml`. This
   builds the body/geom/actuator tree (regenerates `Assets/Local/`, which is gitignored).
3. **Teleop ▸ Set Up Teleop In Scene** (top menu bar) — adds an `MjScene` + a `TeleopController`.
4. Fixed Timestep is already set to **0.002** (Project Settings ▸ Time) — this is required;
   the plugin steps at `Time.fixedDeltaTime`, and the default 0.02 makes the model explode.

> The tuned Inspector values (alignEuler, ikDamping, …) are **not** persisted because no
> `.unity` scene is saved — after "Set Up Teleop In Scene" the component gets code defaults.
> Save a scene (File ▸ Save As) if you want to keep your tuning between sessions.

## Run in sim — no Quest
Press **▶ Play**: the rig stands at its forward home pose. To drive it with a synthetic
operator (validates the whole C# path):
```sh
python3 Tools/fake_operator.py --hands     # both wrists circle + fingers open/close
```
Press **C** in Unity to calibrate; the arms follow the synthetic motion.

## Run with the Quest (ORBIT)
1. Quest on USB, **USB debugging authorized** (accept the prompt in-headset), hand-tracking on.
2. Start the host link (sets up `adb reverse` + forwards UDP, and auto-relaunches the bridge
   if it drops):
   ```sh
   Tools/keepalive.sh        # tunnels + bridge; leave it running
   ```
   (or just the bridge once: `python3 Tools/orbit_to_unity.py`)
3. **Launch the ORBIT app FROM THE HEADSET** while wearing it (you should see its orange
   screen — adb-launching it lands it backgrounded). Set both controllers **down** so
   hand-tracking activates.
4. In Unity: **▶ Play**, hold your hands out, press **C** to calibrate (snaps the arms to
   home and anchors). Move your wrists → arms; your fingers → hands. Press **C** anytime to
   re-snap to home and re-anchor.

Sanity check that hands are streaming (counters climbing):
```sh
grep forwarded ~/orbit_bridge.log | tail -2
```

## Tuning (TeleopController inspector — all live while playing)
The #1 teleop issue is frame alignment. Position and orientation are tuned separately:
- **alignEuler** (deg) — POSITION axes. Move your hand up/forward/right; adjust one axis at
  a time (90° steps) until the robot goes up/forward/right. `flipAxis` fixes a left/right mirror.
- **oriEuler** (deg) — ORIENTATION roll axis. Orientation is body-frame (robot hand mimics
  yours in its own frame). If a wrist roll *tilts* the hand instead of rolling it, bump one
  oriEuler axis by 90 until it rolls cleanly on j6.
- **posScale** — human reach → robot reach.
- **ikDamping** (0.05 crisp ↔ 0.15 smooth), **maxJointSpeed** — IK feel.
- **handInvert** — flip if fingers open when you close.
- **driveArms / driveHands** — isolate one path while tuning.
- **C** = (re)calibrate/snap-to-home, **X** = clear calibration.

## How it works (key implementation notes)
- **Plugin names get a `_<counter>` suffix** at runtime (`UseRawGameObjectNames=false`);
  `MjNames.Strip` makes all body/joint/actuator lookups tolerant of it.
- **Two-stage IK** (`ArmIK`): stage 1 solves position with the shoulder (j1–j3), stage 2
  solves orientation with the wrist (j4–j6), so a wrist twist stays in the wrist. Warm-started
  from the previous solution so the elbow can't flip IK branches (the j3 issue).
- **Retargeting** (`PoseRetargeter`): delta-anchor from a calibrated home. Position is mapped
  in the **world** frame via `Align`; orientation in the **body** frame via `Ori` — decoupled,
  so position changes don't corrupt orientation and vice-versa.
- **Transport**: ORBIT speaks NetMQ; `Tools/orbit_to_unity.py` re-emits it as plain UDP so the
  Unity side needs no third-party DLLs (just `System.Net.Sockets`).

## Repo layout
```
Packages/org.mujoco/         MuJoCo Unity plugin 3.9.0 (embedded; first open needs no network)
Assets/Plugins/mujoco.dylib  native MuJoCo lib (universal x86_64+arm64)
Assets/Mujoco/               bimanual_yam.xml scene + all MJCFs + meshes (arms, hands, stand)
Assets/Scripts/Teleop/       OrbitUdpReceiver, Mat4, PoseRetargeter, ArmIK, HandRetarget,
                             MjNames, TeleopController
Assets/Editor/               Teleop ▸ Set Up Teleop In Scene
Tools/orbit_to_unity.py      ORBIT NetMQ → UDP bridge
Tools/keepalive.sh           keeps adb-reverse tunnels + bridge alive
Tools/fake_operator.py       synthetic operator (drive the sim with no Quest)
Tools/setup.sh               re-vendor mujoco.dylib if missing
Tools/requirements.txt       pyzmq (bridge)
```

## Status / next
Working: scene + physics, two-stage IK arms, finger retarget, snap-to-home calibration,
Quest→bridge→Unity teleop. Tuning in progress — finish dialing `alignEuler` (position) and
`oriEuler` (wrist roll). Possible next steps: save a tuned scene; on-device VR build (the
teleop code is engine-agnostic); fuller `retarget_core` finger geometry (current is curl-based).
