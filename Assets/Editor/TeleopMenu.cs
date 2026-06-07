// One-click teleop setup after importing the MJCF. The MuJoCo plugin gathers all
// MjComponents in the scene regardless of hierarchy, so a standalone MjScene object
// is fine; TeleopController hangs off it.
using Mujoco;
using UnityEditor;
using UnityEngine;

namespace OrcaTeleop {

public static class TeleopMenu {
  [MenuItem("Teleop/Set Up Teleop In Scene")]
  static void Setup() {
    var scene = Object.FindFirstObjectByType<MjScene>();
    if (scene == null) {
      scene = new GameObject("MjScene").AddComponent<MjScene>();
      Debug.Log("[Teleop] added MjScene (steps the imported model).");
    }
    if (Object.FindFirstObjectByType<TeleopController>() == null) {
      scene.gameObject.AddComponent<TeleopController>();
      Debug.Log("[Teleop] added TeleopController. Press Play, run Tools/orbit_to_unity.py, press C to calibrate.");
    }
    Selection.activeObject = scene.gameObject;
    EditorUtility.DisplayDialog("Teleop",
      "MjScene + TeleopController are in the scene.\n\n" +
      "If you haven't yet: Assets > Import MuJoCo Scene  ->  " +
      "Assets/Mujoco/scenes/v2/bimanual_yam.xml\n\nThen press Play.", "OK");
  }
}

}
