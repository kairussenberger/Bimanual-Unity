// Finger retarget: ORBIT hand keypoints -> normalized [0,1] targets for the 17 ORCA
// actuators per hand. This is a deliberately simple, robust CURL mapping (per-finger
// bend from the keypoint chain), enough to make the sim fingers follow open/close and
// to validate the actuator wiring end to end. The fuller geometric retarget in
// orca-teleop/retarget_core.py (per-joint MCP/PIP/abduction angles) can be ported in
// later behind this same Compute() -> {actuator-short-key: normalized} interface.
//
// Input is the 26 raw ORBIT keypoints (flattened xyz, Unity frame). We drop ORBIT's
// Palm (index 1) to get the 25 W3C WebXR joints, whose finger chains are:
//   thumb 1-4, index 5-9, middle 10-14, ring 15-19, pinky 20-24 (0 = wrist).
using System;
using System.Collections.Generic;

namespace OrcaTeleop {

public static class HandRetarget {
  // 17 ORCA actuator short keys (full name = $"{side}_{key}_actuator").
  public static readonly string[] KEYS = {
    "wrist",
    "i-abd","i-mcp","i-pip", "m-abd","m-mcp","m-pip",
    "r-abd","r-mcp","r-pip", "p-abd","p-mcp","p-pip",
    "t-cmc","t-abd","t-mcp","t-pip",
  };

  static readonly int[] THUMB  = {1,2,3,4};
  static readonly int[] INDEX  = {5,6,7,8,9};
  static readonly int[] MIDDLE = {10,11,12,13,14};
  static readonly int[] RING   = {15,16,17,18,19};
  static readonly int[] PINKY  = {20,21,22,23,24};

  // pts = 26*3 doubles (raw ORBIT). Returns short-key -> normalized [0,1].
  public static Dictionary<string, double> Compute(double[] pts) {
    var o = new Dictionary<string, double>(17);
    double ci = Curl(pts, INDEX), cm = Curl(pts, MIDDLE),
           cr = Curl(pts, RING),  cp = Curl(pts, PINKY), ct = Curl(pts, THUMB);
    o["i-mcp"] = ci; o["i-pip"] = ci; o["i-abd"] = 0.5;
    o["m-mcp"] = cm; o["m-pip"] = cm; o["m-abd"] = 0.5;
    o["r-mcp"] = cr; o["r-pip"] = cr; o["r-abd"] = 0.5;
    o["p-mcp"] = cp; o["p-pip"] = cp; o["p-abd"] = 0.5;
    o["t-mcp"] = ct; o["t-pip"] = ct; o["t-cmc"] = ct; o["t-abd"] = 0.5;
    o["wrist"] = 0.5;
    return o;
  }

  // 0 = finger straight (chord ~ summed segment length), 1 = fully curled (chord small).
  static double Curl(double[] pts, int[] chain) {
    double seg = 0;
    for (int i = 0; i + 1 < chain.Length; i++) seg += Dist(pts, chain[i], chain[i + 1]);
    if (seg < 1e-9) return 0;
    double chord = Dist(pts, chain[0], chain[chain.Length - 1]);
    double c = 1.0 - chord / seg;
    return c < 0 ? 0 : (c > 1 ? 1 : c);
  }

  // WebXR index (0..24) -> original ORBIT index (Palm at 1 was dropped).
  static int Orig(int webxr) => webxr == 0 ? 0 : webxr + 1;

  static double Dist(double[] pts, int aWebxr, int bWebxr) {
    int a = Orig(aWebxr) * 3, b = Orig(bWebxr) * 3;
    if (b + 2 >= pts.Length || a + 2 >= pts.Length) return 0;
    double dx = pts[a] - pts[b], dy = pts[a+1] - pts[b+1], dz = pts[a+2] - pts[b+2];
    return Math.Sqrt(dx*dx + dy*dy + dz*dz);
  }
}

}
