"""Minimal rclpy / message mocks so the nodes can be unit-tested without ROS."""
import sys, types


def mod(name, **kw):
    m = types.ModuleType(name); m.__dict__.update(kw); sys.modules[name] = m; return m


class Pub:
    def __init__(self): self.sent = []; self.subs = 0
    def publish(self, m): self.sent.append(m)
    def get_subscription_count(self): return self.subs


class Client:
    def __init__(self): self.ready = True; self.calls = []
    def service_is_ready(self): return self.ready
    def call_async(self, req):
        self.calls.append(req)
        class R: results = [types.SimpleNamespace(successful=True, reason="")]
        class Fut:
            def result(s): return R()
        class F:
            def add_done_callback(f, cb): cb(Fut())
        return F()


class Node:
    def __init__(self, name): self._p = {}; self.subscriptions = {}
    def declare_parameter(self, n, v): self._p[n] = v
    def get_parameter(self, n): return types.SimpleNamespace(value=self._p[n])
    def create_subscription(self, t, topic, cb, qos): self.subscriptions[topic] = cb; return topic
    def destroy_subscription(self, sub): self.subscriptions.pop(sub, None)
    def create_publisher(self, *a, **k): return Pub()
    def create_timer(self, *a, **k): pass
    def create_client(self, *a, **k): return Client()
    def get_subscriptions_info_by_topic(self, topic): return []
    def get_logger(self): return types.SimpleNamespace(**{n: (lambda *a, **k: None) for n in ("info", "warning", "error")})


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0); self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class String:
    def __init__(self, data=""): self.data = data


class Bool:
    def __init__(self, data=False): self.data = data


class Scan:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="laser")


class Time:
    pass


def install():
    mod("rclpy", init=lambda **k: None, spin=lambda n: None, shutdown=lambda: None, ok=lambda: True)
    mod("rclpy.node", Node=Node)
    mod("rclpy.time", Time=Time)
    mod("rclpy.qos", DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1), HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
        ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2), QoSProfile=lambda **k: k)
    mod("geometry_msgs"); mod("geometry_msgs.msg", Twist=Twist)
    mod("std_msgs"); mod("std_msgs.msg", String=String, Bool=Bool)
    mod("sensor_msgs"); mod("sensor_msgs.msg", LaserScan=Scan)
    mod("rcl_interfaces"); mod("rcl_interfaces.msg", Parameter=lambda **k: types.SimpleNamespace(**k),
        ParameterType=types.SimpleNamespace(PARAMETER_STRING=4), ParameterValue=lambda **k: types.SimpleNamespace(**k))
    class Req:
        def __init__(s): s.parameters = []
    mod("rcl_interfaces.srv", SetParameters=types.SimpleNamespace(Request=Req))
