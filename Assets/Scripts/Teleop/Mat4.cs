// Minimal double-precision RIGID transform (3x3 rotation R + translation t), used to
// mirror the numpy/scipy math in orca-teleop/vr_zmq.py EXACTLY and independently of
// Unity's float Matrix4x4 (whose left-handed conventions would only add confusion).
// All teleop frame math (Unity-LH -> robot congruence, delta-anchor) runs through here.
using System;

namespace OrcaTeleop {

public struct Mat4 {
  public double[] R;   // 3x3 row-major rotation
  public double[] t;   // 3 translation

  public static Mat4 Identity() =>
      new Mat4 { R = new double[] {1,0,0, 0,1,0, 0,0,1}, t = new double[] {0,0,0} };

  // Pose from position + quaternion in XYZW (scalar-LAST) order, like ORBIT/scipy.
  public static Mat4 FromPosQuatXyzw(double px, double py, double pz,
                                     double qx, double qy, double qz, double qw) {
    double n = Math.Sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n < 1e-12) n = 1.0;
    qx/=n; qy/=n; qz/=n; qw/=n;
    double xx=qx*qx, yy=qy*qy, zz=qz*qz, xy=qx*qy, xz=qx*qz, yz=qy*qz, wx=qw*qx, wy=qw*qy, wz=qw*qz;
    var r = new double[] {
      1-2*(yy+zz),   2*(xy-wz),   2*(xz+wy),
        2*(xy+wz), 1-2*(xx+zz),   2*(yz-wx),
        2*(xz-wy),   2*(yz+wx), 1-2*(xx+yy),
    };
    return new Mat4 { R = r, t = new double[] {px, py, pz} };
  }

  // Handedness flip by congruence  M' = S * M * S,  S = diag(s0,s1,s2), s in {+1,-1}.
  // For a rigid transform this is  R'[i,j] = s_i*s_j*R[i,j],  t'[i] = s_i*t[i].
  public Mat4 Congruence(double s0, double s1, double s2) {
    var s = new double[] {s0, s1, s2};
    var r = new double[9];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        r[i*3+j] = s[i]*s[j]*R[i*3+j];
    return new Mat4 { R = r, t = new double[] {s0*t[0], s1*t[1], s2*t[2]} };
  }

  public static Mat4 Mul(Mat4 a, Mat4 b) {            // a * b (rigid)
    var r = new double[9];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double v = 0;
        for (int k = 0; k < 3; k++) v += a.R[i*3+k]*b.R[k*3+j];
        r[i*3+j] = v;
      }
    var tt = new double[3];
    for (int i = 0; i < 3; i++)
      tt[i] = a.R[i*3+0]*b.t[0] + a.R[i*3+1]*b.t[1] + a.R[i*3+2]*b.t[2] + a.t[i];
    return new Mat4 { R = r, t = tt };
  }

  public Mat4 Inverse() {                              // rigid inverse: (R^T, -R^T t)
    var r = new double[] { R[0],R[3],R[6], R[1],R[4],R[7], R[2],R[5],R[8] };
    var tt = new double[3];
    for (int i = 0; i < 3; i++)
      tt[i] = -(r[i*3+0]*t[0] + r[i*3+1]*t[1] + r[i*3+2]*t[2]);
    return new Mat4 { R = r, t = tt };
  }

  public void ScaleTranslation(double s) { t[0]*=s; t[1]*=s; t[2]*=s; }

  // Rotation -> quaternion in WXYZ (scalar-FIRST) order, what MuJoCo / ArmIK expect.
  public double[] QuatWxyz() {
    double m00=R[0], m01=R[1], m02=R[2], m10=R[3], m11=R[4], m12=R[5], m20=R[6], m21=R[7], m22=R[8];
    double tr = m00 + m11 + m22, w, x, y, z;
    if (tr > 0) {
      double s = Math.Sqrt(tr + 1.0) * 2; w = 0.25*s;
      x = (m21-m12)/s; y = (m02-m20)/s; z = (m10-m01)/s;
    } else if (m00 > m11 && m00 > m22) {
      double s = Math.Sqrt(1.0 + m00 - m11 - m22) * 2;
      w = (m21-m12)/s; x = 0.25*s; y = (m01+m10)/s; z = (m02+m20)/s;
    } else if (m11 > m22) {
      double s = Math.Sqrt(1.0 + m11 - m00 - m22) * 2;
      w = (m02-m20)/s; x = (m01+m10)/s; y = 0.25*s; z = (m12+m21)/s;
    } else {
      double s = Math.Sqrt(1.0 + m22 - m00 - m11) * 2;
      w = (m10-m01)/s; x = (m02+m20)/s; y = (m12+m21)/s; z = 0.25*s;
    }
    return new double[] { w, x, y, z };
  }

  public double[] Pos() => new double[] { t[0], t[1], t[2] };
}

}
