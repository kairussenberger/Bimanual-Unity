// Delta-anchor retargeting with a FIXED operator->robot world alignment.
//
// On calibrate we anchor the operator wrist (M_vr0) and the robot HOME EE pose (M_robot0).
// While running, the EE target is the operator's motion SINCE the anchor, expressed in the
// robot WORLD frame through the fixed rotation `Align` (operator/headset axes -> robot axes):
//
//   pos = robot0_pos + Align * (vr_now_pos - vr0_pos) * scale
//   R   = ( Align * (vr_now_R * vr0_R^T) * Align^T ) * robot0_R
//
// CRUCIAL FIX vs the old code (target = M_robot0 * inv(M_vr0) * M_vr_now): position is no
// longer rotated by the END-EFFECTOR's orientation, so "move your hand up" maps to ONE fixed
// world direction set only by Align — not by how you held your wrist at calibration. Tune
// Align (TeleopController.alignEuler) so up/forward/right line up; flipAxis fixes chirality.
namespace OrcaTeleop {

public class PoseRetargeter {
  public double PosScale = 1.0;
  public double[] Align = { 1, 0, 0,  0, 1, 0,  0, 0, 1 };   // 3x3, operator(S-frame) -> robot world (POSITION)
  public double[] Ori   = { 1, 0, 0,  0, 1, 0,  0, 0, 1 };   // 3x3, operator-wrist-local -> robot-hand-local (ROLL axis)

  readonly Mat4[] _vr0 = new Mat4[2];
  readonly Mat4[] _robot0 = new Mat4[2];
  readonly bool[] _cal = new bool[2];

  static int Idx(string side) => side == "right" ? 0 : 1;

  public void Calibrate(string side, Mat4 vrNow, Mat4 robotNow) {
    int i = Idx(side);
    _vr0[i] = vrNow; _robot0[i] = robotNow; _cal[i] = true;
  }

  public bool IsCalibrated(string side) => _cal[Idx(side)];
  public bool FullyCalibrated() => _cal[0] && _cal[1];
  public void Clear() { _cal[0] = _cal[1] = false; }

  // -> (pos[3], quatWxyz[4]) EE target in the MuJoCo world frame, or false if uncalibrated.
  public bool Target(string side, Mat4 vrNow, out double[] pos, out double[] quatWxyz) {
    int i = Idx(side);
    if (!_cal[i]) { pos = null; quatWxyz = null; return false; }
    Mat4 vr0 = _vr0[i], rob0 = _robot0[i];

    // POSITION: world-frame delta, rotated by the fixed alignment (NOT the EE pose).
    double[] dp = { vrNow.t[0] - vr0.t[0], vrNow.t[1] - vr0.t[1], vrNow.t[2] - vr0.t[2] };
    double[] wdp = MV(Align, dp);
    pos = new double[] { rob0.t[0] + PosScale * wdp[0],
                         rob0.t[1] + PosScale * wdp[1],
                         rob0.t[2] + PosScale * wdp[2] };

    // ORIENTATION: BODY-FRAME — the robot hand mimics your hand's rotation in its OWN frame,
    // so a wrist roll rolls the robot wrist (j6) instead of tilting it about a world axis.
    // R = R_home * ( Ori * (R_vr0^T * R_vr_now) * Ori^T );  Ori re-aligns the wrist roll axis.
    double[] dRl = MM(Tr(vr0.R), vrNow.R);                // operator rotation in its calib-local frame
    double[] R = MM(rob0.R, MM(MM(Ori, dRl), Tr(Ori)));   // applied in the robot hand's local frame
    quatWxyz = new Mat4 { R = R, t = new double[3] }.QuatWxyz();
    return true;
  }

  // ---- tiny 3x3 helpers (row-major) ----
  static double[] MV(double[] R, double[] v) => new double[] {
    R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
    R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
    R[6]*v[0] + R[7]*v[1] + R[8]*v[2] };

  static double[] Tr(double[] R) => new double[] { R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8] };

  static double[] MM(double[] a, double[] b) {
    var o = new double[9];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double s = 0; for (int k = 0; k < 3; k++) s += a[i*3 + k] * b[k*3 + j];
        o[i*3 + j] = s;
      }
    return o;
  }
}

}
