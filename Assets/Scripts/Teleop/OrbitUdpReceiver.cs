// Receives the tagged UDP datagrams produced by Tools/orbit_to_unity.py (which binds
// the ORBIT NetMQ ports and re-emits them). Uses ONLY System.Net.Sockets.UdpClient,
// so the Unity project needs no third-party networking DLLs. Latest-frame-wins per
// channel; everything is stored in the raw ORBIT/Unity frame (no conversion here).
//
// Wire IN (UTF-8 text, one datagram per channel):
//   W,right,px,py,pz,qx,qy,qz,qw      wrist pose (Unity XR world, meters, XYZW)
//   H,right,k0x,k0y,k0z,...,k25z       26 hand keypoints (Unity frame)
//   HEAD,px,py,pz,qx,qy,qz,qw
using System;
using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace OrcaTeleop {

public class OrbitUdpReceiver {
  public int Port = 9101;

  class Channel { public double[] data; public double stamp; }
  readonly object _lock = new object();
  readonly Channel[] _wrist = { new Channel(), new Channel() };   // [0]=right [1]=left
  readonly Channel[] _hand  = { new Channel(), new Channel() };
  readonly Channel _head = new Channel();
  public long WristCount, HandCount, HeadCount;

  UdpClient _client;
  Thread _thread;
  volatile bool _stop;
  readonly Stopwatch _clock = Stopwatch.StartNew();

  static int Side(string s) => s == "right" ? 0 : 1;
  public double Now => _clock.Elapsed.TotalSeconds;

  public void Start() {
    _stop = false;
    _client = new UdpClient(new IPEndPoint(IPAddress.Loopback, Port));
    _client.Client.ReceiveTimeout = 200;
    _thread = new Thread(Run) { IsBackground = true, Name = "OrbitUdpReceiver" };
    _thread.Start();
  }

  public void Stop() {
    _stop = true;
    try { _client?.Close(); } catch { }
    _thread = null;
  }

  void Run() {
    var any = new IPEndPoint(IPAddress.Any, 0);
    while (!_stop) {
      byte[] buf;
      try { buf = _client.Receive(ref any); }
      catch (SocketException) { continue; }     // timeout / closed
      catch (ObjectDisposedException) { break; }
      string msg;
      try { msg = System.Text.Encoding.UTF8.GetString(buf); } catch { continue; }
      Ingest(msg);
    }
  }

  void Ingest(string msg) {
    var t = msg.Split(',');
    if (t.Length < 1) return;
    double now = Now;
    try {
      if (t[0] == "W" && t.Length == 9) {
        var d = new double[7];
        for (int i = 0; i < 7; i++) d[i] = P(t[i + 2]);
        int s = Side(t[1]);
        lock (_lock) { _wrist[s].data = d; _wrist[s].stamp = now; }
        Interlocked.Increment(ref WristCount);
      } else if (t[0] == "HEAD" && t.Length == 8) {
        var d = new double[7];
        for (int i = 0; i < 7; i++) d[i] = P(t[i + 1]);
        lock (_lock) { _head.data = d; _head.stamp = now; }
        Interlocked.Increment(ref HeadCount);
      } else if (t[0] == "H" && t.Length >= 2 + 3) {
        int n = (t.Length - 2) / 3;                 // 26 expected
        var d = new double[n * 3];
        for (int i = 0; i < n * 3; i++) d[i] = P(t[i + 2]);
        int s = Side(t[1]);
        lock (_lock) { _hand[s].data = d; _hand[s].stamp = now; }
        Interlocked.Increment(ref HandCount);
      }
    } catch (FormatException) { /* drop malformed datagram */ }
  }

  static double P(string s) => double.Parse(s, CultureInfo.InvariantCulture);

  // ---- accessors (main thread): return false if no fresh sample ----
  public bool TryGetWrist(string side, double maxAge, out double[] pose, out double age) =>
      Get(_wrist[Side(side)], maxAge, out pose, out age);
  public bool TryGetHand(string side, double maxAge, out double[] pts, out double age) =>
      Get(_hand[Side(side)], maxAge, out pts, out age);
  public bool TryGetHead(double maxAge, out double[] pose, out double age) =>
      Get(_head, maxAge, out pose, out age);

  bool Get(Channel c, double maxAge, out double[] data, out double age) {
    lock (_lock) {
      if (c.data == null) { data = null; age = double.MaxValue; return false; }
      age = Now - c.stamp;
      data = c.data;
      return age <= maxAge;
    }
  }
}

}
