namespace OrcaTeleop {

// The MuJoCo Unity plugin, with MjGlobalSettings.UseRawGameObjectNames = false (the
// DEFAULT), appends "_<counter>" to every element name in the runtime-regenerated
// model (MjcfGenerationContext.GenerateName). So "left_hand_mount" becomes
// "left_hand_mount_42" and "left_i-mcp_actuator" becomes "left_i-mcp_actuator_137".
// Strip() removes a single trailing _<digits> so our logical names match either way
// (also a no-op when UseRawGameObjectNames is true).
public static class MjNames {
  public static string Strip(string s) {
    if (string.IsNullOrEmpty(s)) return s;
    int u = s.LastIndexOf('_');
    if (u <= 0 || u >= s.Length - 1) return s;
    for (int i = u + 1; i < s.Length; i++)
      if (!char.IsDigit(s[i])) return s;        // trailing token isn't pure digits -> keep
    return s.Substring(0, u);
  }
}

}
