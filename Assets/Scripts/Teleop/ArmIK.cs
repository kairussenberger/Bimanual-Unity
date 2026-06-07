// Per-arm 6-DOF damped-least-squares IK, ported from orca-teleop/vr_ik.py.
// Given a target end-effector POSE (pos + quat-wxyz) in the MuJoCo world frame, solve
// arm_j1..arm_j6 so the {side}_hand_mount body lands there. Iterates
//   dq = J^T (J J^T + lambda^2 I)^-1 e   on the 6-vector pose error e=[pos(3);ori(3)],
// on a SCRATCH mjData (mj_makeData) so the live/rendered sim is never disturbed,
// warm-started from the live qpos each tick. Uses the plugin's raw MujocoLib bindings.
using System;
using System.Runtime.InteropServices;
using Mujoco;

namespace OrcaTeleop {

public unsafe class ArmIK : IDisposable {
  public static readonly string[] JOINTS = { "arm_j1","arm_j2","arm_j3","arm_j4","arm_j5","arm_j6" };

  readonly MujocoLib.mjModel_* _m;
  MujocoLib.mjData_* _d;                 // scratch
  readonly int _ee;
  readonly int[] _qadr = new int[6];
  readonly int[] _dadr = new int[6];
  readonly double[] _lo = new double[6];
  readonly double[] _hi = new double[6];
  readonly int _nq, _nv;
  public readonly bool Valid;
  public string Error;
  public int EeBodyId => _ee;        // resolved {side}_hand_mount body id (live model)

  public double Damping = 0.1;
  public double MaxDq = 0.4;

  public ArmIK(MujocoLib.mjModel_* model, string side) {
    _m = model;
    _nq = (int)model->nq; _nv = (int)model->nv;
    _ee = FindByName(model, MujocoLib.mjtObj.mjOBJ_BODY, (int)model->nbody, side + "_hand_mount");
    if (_ee < 0) { Error = $"body {side}_hand_mount not found"; Valid = false; return; }
    for (int i = 0; i < 6; i++) {
      int jid = FindByName(model, MujocoLib.mjtObj.mjOBJ_JOINT, (int)model->njnt, side + "_" + JOINTS[i]);
      if (jid < 0) { Error = $"joint {side}_{JOINTS[i]} not found"; Valid = false; return; }
      _qadr[i] = model->jnt_qposadr[jid];
      _dadr[i] = model->jnt_dofadr[jid];
      _lo[i] = model->jnt_range[2*jid + 0];
      _hi[i] = model->jnt_range[2*jid + 1];
    }
    _d = MujocoLib.mj_makeData(model);
    Valid = _d != null;
    if (!Valid) Error = "mj_makeData failed";
  }

  // Resolve an element id by logical name, tolerating the plugin's "_<counter>" suffix
  // (see MjNames). Tries an exact match first, then a stripped-name scan.
  static int FindByName(MujocoLib.mjModel_* m, MujocoLib.mjtObj type, int count, string target) {
    int id = MujocoLib.mj_name2id(m, (int)type, target);
    if (id >= 0) return id;
    for (int i = 0; i < count; i++) {
      IntPtr p = MujocoLib.mj_id2name(m, (int)type, i);
      if (p == IntPtr.Zero) continue;
      if (MjNames.Strip(Marshal.PtrToStringAnsi(p)) == target) return i;
    }
    return -1;
  }

  // Forward-kinematics the EE pose for a given 6-joint arm config (rest of the model
  // taken from liveQpos). Used to anchor calibration to the HOME pose (snap-to-home),
  // independent of where the arm currently is. Writes pos[3] and quat[4] (wxyz).
  public void EePoseAt(double* liveQpos, double[] armAngles, double[] outPos, double[] outQuatWxyz) {
    for (int i = 0; i < _nq; i++) _d->qpos[i] = liveQpos[i];
    for (int c = 0; c < 6; c++) _d->qpos[_qadr[c]] = armAngles[c];
    MujocoLib.mj_kinematics(_m, _d);
    double* xp = _d->xpos + 3 * _ee;
    double* xq = _d->xquat + 4 * _ee;
    outPos[0] = xp[0]; outPos[1] = xp[1]; outPos[2] = xp[2];
    outQuatWxyz[0] = xq[0]; outQuatWxyz[1] = xq[1]; outQuatWxyz[2] = xq[2]; outQuatWxyz[3] = xq[3];
  }

  // liveQpos = pointer to the live Data->qpos. armSeed (or null) overrides the 6 arm joints
  // as the warm start — pass the PREVIOUS solution for continuity so the elbow can't flip to
  // a far IK branch (the j3 "over-extension"). Writes the 6 joint targets into result[0..5].
  public double Solve(double* liveQpos, double[] armSeed, double[] targetPos, double[] targetQuatWxyz,
                      double[] result, int iters = 20, double tol = 1e-4, double step = 1.0) {
    for (int i = 0; i < _nq; i++) _d->qpos[i] = liveQpos[i];
    if (armSeed != null) for (int c = 0; c < 6; c++) _d->qpos[_qadr[c]] = armSeed[c];

    double residual = 1e9;
    double* jacp = stackalloc double[3 * _nv];
    double* jacr = stackalloc double[3 * _nv];
    double* tq   = stackalloc double[4];
    double* neg  = stackalloc double[4];
    double* dqt  = stackalloc double[4];
    double* erot = stackalloc double[3];
    for (int i = 0; i < 4; i++) tq[i] = targetQuatWxyz[i];

    // Two-stage decoupled DLS (block coordinate descent on disjoint joint sets):
    //   Stage 1 reduces POSITION error using ONLY the shoulder j1-j3.
    //   Stage 2 reduces ORIENTATION error using ONLY the wrist  j4-j6.
    // The shoulder is structurally barred from doing orientation, so a wrist twist can
    // never swing the arm; at the wrist singularity it just softly under-rotates. Cross
    // coupling (wrist nudging position) is mopped up by stage 1 on the next iteration.
    var jp = new double[3, 3];      // position Jacobian, columns = j1-j3
    var jr = new double[3, 3];      // orientation Jacobian, columns = j4-j6
    var A3 = new double[3, 3];
    var y3 = new double[3];
    var ePos = new double[3];
    var eRot = new double[3];
    double l2 = Damping * Damping;
    double resP = 1e9, resR = 1e9;

    for (int it = 0; it < iters; it++) {
      MujocoLib.mj_kinematics(_m, _d);
      MujocoLib.mj_comPos(_m, _d);
      MujocoLib.mj_jacBody(_m, _d, jacp, jacr, _ee);

      double* xp = _d->xpos + 3 * _ee;
      ePos[0] = targetPos[0] - xp[0];
      ePos[1] = targetPos[1] - xp[1];
      ePos[2] = targetPos[2] - xp[2];
      MujocoLib.mju_negQuat(neg, _d->xquat + 4 * _ee);   // R_cur^-1
      MujocoLib.mju_mulQuat(dqt, tq, neg);               // R_err = R_tgt * R_cur^-1
      MujocoLib.mju_quat2Vel(erot, dqt, 1.0);
      eRot[0] = erot[0]; eRot[1] = erot[1]; eRot[2] = erot[2];

      resP = Math.Sqrt(ePos[0]*ePos[0] + ePos[1]*ePos[1] + ePos[2]*ePos[2]);
      resR = Math.Sqrt(eRot[0]*eRot[0] + eRot[1]*eRot[1] + eRot[2]*eRot[2]);
      if (resP < tol && resR < tol) break;

      for (int c = 0; c < 3; c++) {                      // stage 1: position, j1-j3
        int dof = _dadr[c];
        jp[0, c] = jacp[0*_nv + dof]; jp[1, c] = jacp[1*_nv + dof]; jp[2, c] = jacp[2*_nv + dof];
      }
      StageStep(jp, ePos, 0, l2, step, A3, y3);
      for (int c = 0; c < 3; c++) {                      // stage 2: orientation, j4-j6
        int dof = _dadr[c + 3];
        jr[0, c] = jacr[0*_nv + dof]; jr[1, c] = jacr[1*_nv + dof]; jr[2, c] = jacr[2*_nv + dof];
      }
      StageStep(jr, eRot, 3, l2, step, A3, y3);
    }
    residual = resP + resR;
    for (int c = 0; c < 6; c++) result[c] = _d->qpos[_qadr[c]];
    return residual;
  }

  // One damped 3-DOF least-squares step toward task error `e`, applied to the three
  // joints at _qadr[k0..k0+2] and ONLY those:  dq = J^T (J J^T + l2 I)^-1 e.
  unsafe void StageStep(double[,] J, double[] e, int k0, double l2, double step, double[,] A, double[] y) {
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++) {
        double v = 0; for (int c = 0; c < 3; c++) v += J[a, c] * J[b, c];
        A[a, b] = v + (a == b ? l2 : 0);
      }
    Solve3(A, e, y);                          // destroys A and e (both scratch this iter)
    for (int c = 0; c < 3; c++) {
      double dq = 0; for (int r = 0; r < 3; r++) dq += J[r, c] * y[r];
      dq = Math.Max(-MaxDq, Math.Min(MaxDq, step * dq));
      int idx = _qadr[k0 + c];
      _d->qpos[idx] = Math.Max(_lo[k0 + c], Math.Min(_hi[k0 + c], _d->qpos[idx] + dq));
    }
  }

  // In-place Gaussian elimination with partial pivoting for a 3x3 system M x = rhs.
  // Overwrites M and rhs; writes the solution into x.
  static void Solve3(double[,] M, double[] rhs, double[] x) {
    for (int col = 0; col < 3; col++) {
      int piv = col; double best = Math.Abs(M[col, col]);
      for (int r = col + 1; r < 3; r++) { double v = Math.Abs(M[r, col]); if (v > best) { best = v; piv = r; } }
      if (piv != col) {
        for (int c = 0; c < 3; c++) { double t = M[col, c]; M[col, c] = M[piv, c]; M[piv, c] = t; }
        double tb = rhs[col]; rhs[col] = rhs[piv]; rhs[piv] = tb;
      }
      double d = M[col, col];
      if (Math.Abs(d) < 1e-12) d = d < 0 ? -1e-12 : 1e-12;     // nudge off singular
      for (int r = 0; r < 3; r++) {
        if (r == col) continue;
        double f = M[r, col] / d;
        for (int c = col; c < 3; c++) M[r, c] -= f * M[col, c];
        rhs[r] -= f * rhs[col];
      }
    }
    for (int i = 0; i < 3; i++) x[i] = rhs[i] / M[i, i];
  }

  public void Dispose() {
    if (_d != null) { MujocoLib.mj_deleteData(_d); _d = null; }
  }
}

}
