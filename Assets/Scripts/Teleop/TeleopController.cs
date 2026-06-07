// Drives the imported bimanual MuJoCo scene from ORBIT Quest tracking.
//
// Pipeline (mirrors orca-teleop/vr_zmq.py + vr_ik.py):
//   OrbitUdpReceiver (UDP from Tools/orbit_to_unity.py)
//     -> MjFrame congruence  M_robot = S * M_unity * S        (Unity-LH -> robot chirality)
//     -> PoseRetargeter delta-anchor  target = M_robot0 * inv(M_vr0) * M_vr_now
//     -> ArmIK DLS  -> arm_j1..j6 targets  -> MjActuator.Control
//     -> HandRetarget curls -> 17 hand MjActuator.Control per side
//
// Add this component to any GameObject in a scene that also has an MjScene (the
// "Assets > Import MuJoCo Scene" importer creates the body/actuator tree; add an
// MjScene component to the root, then this). It subscribes to MjScene.preUpdateEvent
// so controls are written into Data->ctrl right before each mj_step.
using System;
using System.Collections.Generic;
using Mujoco;
using UnityEngine;

namespace OrcaTeleop {

public class TeleopController : MonoBehaviour {
  [Header("Transport")]
  public int udpPort = 9101;
  [Tooltip("Run Tools/orbit_to_unity.py to feed this. Stale samples older than maxAge are ignored.")]
  public double maxAge = 0.25;

  [Header("Mapping")]
  [Tooltip("Handedness-flip axis for the Unity->robot congruence. Flip if motion is MIRRORED.")]
  public string flipAxis = "z";       // x | y | z | none
  public double posScale = 1.0;
  [Tooltip("POSITION alignment: operator->robot world (deg, Rz*Ry*Rx). Tune ONE axis at a time " +
           "until moving your hand up/forward/right moves the robot up/forward/right.")]
  public Vector3 alignEuler = new Vector3(90, 0, 0);
  [Tooltip("ORIENTATION/ROLL axis offset (deg). Orientation is body-frame (robot mimics your " +
           "hand in its own frame). If a wrist ROLL still tilts the robot hand instead of " +
           "rolling it, bump one axis by 90 until it rolls cleanly.")]
  public Vector3 oriEuler = new Vector3(0, 0, 0);

  [Header("What to drive")]
  public bool driveArms = true;
  public bool driveHands = true;
  [Tooltip("Flip if fingers open when you close (depends on ORCA joint sign).")]
  public bool handInvert = false;
  public int ikIters = 12;

  [Header("Calibration")]
  [Tooltip("Press to (re)calibrate: snaps the arms to the home pose and anchors your current " +
           "hand poses to it. Starts UNcalibrated (arms hold home, not following) until pressed.")]
  public KeyCode calibrateKey = KeyCode.C;
  public KeyCode clearKey = KeyCode.X;

  [Header("Rest pose")]
  [Tooltip("Drive the arms to the forward home pose (j1=±90°) on bind, instead of the " +
           "model's straight-up zero pose. IK overrides this once calibrated.")]
  public bool holdHomePose = true;

  [Header("IK")]
  [Tooltip("Two-stage: shoulder (j1-j3) does position, wrist (j4-j6) does orientation. " +
           "DLS damping — higher = smoother through the wrist singularity, slightly less exact. " +
           "0.05 = crisp, 0.15 = soft.")]
  public double ikDamping = 0.08;
  [Tooltip("Max arm joint speed (rad/s) — caps IK lurches near singularities. Lower = smoother " +
           "but laggier; too low feels under-responsive.")]
  public float maxJointSpeed = 8.0f;

  static readonly string[] SIDES = { "right", "left" };

  // Forward-facing home from sim_yam_bimanual.py: arm_j1 = +90° (left) / 270° (right), rest 0.
  static double HomeAngle(string side, int j) =>
      j == 0 ? (side == "left" ? 1.5708 : 4.7124) : 0.0;

  OrbitUdpReceiver _rx;
  PoseRetargeter _retarget;
  readonly ArmIK[] _ik = new ArmIK[2];
  Dictionary<string, MjActuator> _act;
  double[] _S;
  bool _subscribed, _calibrateRequested;
  string _lastError;
  readonly double[][] _armTarget = { new double[6], new double[6] };   // rate-limited IK targets
  readonly bool[] _armInit = new bool[2];

  void Start() {
    _S = FlipSigns(flipAxis);
    _retarget = new PoseRetargeter { PosScale = posScale };
    _rx = new OrbitUdpReceiver { Port = udpPort };
    _rx.Start();
    Debug.Log($"[Teleop] listening for ORBIT UDP on 127.0.0.1:{udpPort} " +
              "(start Tools/orbit_to_unity.py). Press " + calibrateKey + " to calibrate.");
  }

  void Update() {
    if (Input.GetKeyDown(calibrateKey)) _calibrateRequested = true;
    if (Input.GetKeyDown(clearKey)) { _retarget.Clear(); _armInit[0] = _armInit[1] = false; Debug.Log("[Teleop] calibration cleared"); }
    if (!_subscribed && MjScene.InstanceExists) TrySubscribe();
  }

  unsafe void TrySubscribe() {
    var scene = MjScene.Instance;
    if (scene == null || scene.Model == null) return;
    _act = new Dictionary<string, MjActuator>();
    foreach (var a in FindObjectsByType<MjActuator>(FindObjectsSortMode.None))
      if (!string.IsNullOrEmpty(a.MujocoName)) _act[MjNames.Strip(a.MujocoName)] = a;  // tolerate "_N" suffix
    if (holdHomePose)
      for (int i = 0; i < 2; i++)
        for (int j = 0; j < 6; j++)
          SetCtrl($"{SIDES[i]}_arm_j{j + 1}_actuator", HomeAngle(SIDES[i], j));
    scene.preUpdateEvent += OnPreUpdate;
    _subscribed = true;
    Debug.Log($"[Teleop] bound to MjScene ({_act.Count} actuators).");
  }

  unsafe void OnPreUpdate(object sender, MjStepArgs e) {
    if (e.model == null || e.data == null) return;
    try {
      for (int i = 0; i < 2; i++)
        if (_ik[i] == null) {
          _ik[i] = new ArmIK(e.model, SIDES[i]) { MaxDq = 0.4, Damping = ikDamping };
          if (!_ik[i].Valid) Debug.LogWarning($"[Teleop] ArmIK {SIDES[i]} disabled: {_ik[i].Error}");
        }
      for (int i = 0; i < 2; i++) if (_ik[i] != null && _ik[i].Valid) _ik[i].Damping = ikDamping;  // live-tunable
      _retarget.Align = EulerToR(alignEuler);     // live frame-alignment tuning (position)
      _retarget.Ori = EulerToR(oriEuler);         // live roll-axis tuning (orientation)
      _retarget.PosScale = posScale;

      if (_calibrateRequested) TryCalibrate(e.data);

      var res = new double[6];
      for (int i = 0; i < 2; i++) {
        string side = SIDES[i];
        if (!_rx.TryGetWrist(side, maxAge, out var wp, out _)) continue;
        Mat4 mvr = WristToRobot(wp);

        if (driveArms && _ik[i].Valid && _retarget.IsCalibrated(side) &&
            _retarget.Target(side, mvr, out var pos, out var quat)) {
          _ik[i].Solve(e.data->qpos, _armInit[i] ? _armTarget[i] : null, pos, quat, res, ikIters);
          double maxStep = maxJointSpeed * Time.fixedDeltaTime;     // rate-limit per tick
          for (int j = 0; j < 6; j++) {
            if (!_armInit[i]) {
              _armTarget[i][j] = res[j];
            } else {
              double d = res[j] - _armTarget[i][j];
              if (d > maxStep) d = maxStep; else if (d < -maxStep) d = -maxStep;
              _armTarget[i][j] += d;
            }
            SetCtrl($"{side}_arm_j{j + 1}_actuator", _armTarget[i][j]);
          }
          _armInit[i] = true;
        }

        if (driveHands && _rx.TryGetHand(side, maxAge, out var pts, out _)) {
          var curls = HandRetarget.Compute(pts);
          foreach (var kv in curls) {
            string name = $"{side}_{kv.Key}_actuator";
            if (_act.TryGetValue(name, out var a)) {
              var cr = a.CommonParams.CtrlRange;
              double n = handInvert ? 1.0 - kv.Value : kv.Value;
              a.Control = (float)(cr.x + (cr.y - cr.x) * n);
            }
          }
        }
      }
    } catch (Exception ex) {
      if (ex.Message != _lastError) { _lastError = ex.Message; Debug.LogError("[Teleop] " + ex); }
    }
  }

  // Snap-to-home (re)calibration: for each tracked side, anchor the operator's current
  // wrist to the robot's HOME EE pose (FK'd at the home joints), so the arm jumps to home
  // and then follows your hand from there. Independent of where the arm currently is.
  unsafe void TryCalibrate(MujocoLib.mjData_* d) {
    var homeAng = new double[6];
    var hp = new double[3];
    var hq = new double[4];
    int n = 0;
    for (int i = 0; i < 2; i++) {
      string side = SIDES[i];
      if (_ik[i] == null || !_ik[i].Valid) continue;
      if (!_rx.TryGetWrist(side, maxAge, out var wp, out _)) continue;   // need this wrist
      Mat4 mvr = WristToRobot(wp);
      for (int j = 0; j < 6; j++) homeAng[j] = HomeAngle(side, j);
      _ik[i].EePoseAt(d->qpos, homeAng, hp, hq);                          // hq = wxyz
      Mat4 robot0 = Mat4.FromPosQuatXyzw(hp[0], hp[1], hp[2], hq[1], hq[2], hq[3], hq[0]);
      _retarget.Calibrate(side, mvr, robot0);
      for (int j = 0; j < 6; j++) {                                      // snap to home + seed IK/limiter
        SetCtrl($"{side}_arm_j{j + 1}_actuator", homeAng[j]);
        _armTarget[i][j] = homeAng[j];
      }
      _armInit[i] = true;                                                // IK warm-starts from home -> natural elbow branch
      n++;
    }
    _calibrateRequested = false;
    Debug.Log(n > 0 ? $"[Teleop] calibrated {n} arm(s) -> snapped to home, anchored."
                    : "[Teleop] calibrate: no tracked wrists (hold both hands up, controllers down).");
  }

  // ORBIT wrist pose (Unity frame, XYZW) -> robot frame via S * M * S (vr_zmq).
  Mat4 WristToRobot(double[] wp) =>
      Mat4.FromPosQuatXyzw(wp[0], wp[1], wp[2], wp[3], wp[4], wp[5], wp[6])
          .Congruence(_S[0], _S[1], _S[2]);

  void SetCtrl(string actuatorName, double value) {
    if (_act != null && _act.TryGetValue(actuatorName, out var a)) a.Control = (float)value;
  }

  static double[] EulerToR(Vector3 deg) {
    double x = deg.x * Math.PI / 180.0, y = deg.y * Math.PI / 180.0, z = deg.z * Math.PI / 180.0;
    double cx = Math.Cos(x), sx = Math.Sin(x), cy = Math.Cos(y), sy = Math.Sin(y), cz = Math.Cos(z), sz = Math.Sin(z);
    double[] rx = { 1, 0, 0,  0, cx, -sx,  0, sx, cx };
    double[] ry = { cy, 0, sy,  0, 1, 0,  -sy, 0, cy };
    double[] rz = { cz, -sz, 0,  sz, cz, 0,  0, 0, 1 };
    return MM3(rz, MM3(ry, rx));     // R = Rz * Ry * Rx
  }

  static double[] MM3(double[] a, double[] b) {
    var o = new double[9];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double s = 0; for (int k = 0; k < 3; k++) s += a[i*3 + k] * b[k*3 + j];
        o[i*3 + j] = s;
      }
    return o;
  }

  static double[] FlipSigns(string axis) {
    switch ((axis ?? "z").Trim().ToLowerInvariant()) {
      case "x": return new double[] { -1, 1, 1 };
      case "y": return new double[] { 1, -1, 1 };
      case "z": return new double[] { 1, 1, -1 };
      default:  return new double[] { 1, 1, 1 };
    }
  }

  void OnDestroy() {
    if (_subscribed && MjScene.InstanceExists) MjScene.Instance.preUpdateEvent -= OnPreUpdate;
    _rx?.Stop();
    for (int i = 0; i < 2; i++) _ik[i]?.Dispose();
  }

  void OnGUI() {
    bool r = _rx != null && _rx.TryGetWrist("right", maxAge, out _, out _);
    bool l = _rx != null && _rx.TryGetWrist("left", maxAge, out _, out _);
    string cal = _retarget != null && _retarget.FullyCalibrated() ? "CALIBRATED" : "press C to calibrate";
    GUI.Label(new Rect(10, 10, 720, 22),
      $"[Teleop] wrists R:{(r ? "Y" : "·")} L:{(l ? "Y" : "·")}  |  {cal}  |  " +
      $"UDP {udpPort}  W={_rx?.WristCount} H={_rx?.HandCount}  |  arms={driveArms} hands={driveHands}");
  }
}

}
