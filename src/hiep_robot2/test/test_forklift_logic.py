import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from hiep_robot2.forklift_logic import ForkConfig, ForkliftCore, HwState

class Sim:
    """Fork (60 mm/s at 100 %) + a robot that drives exactly at the commanded speed."""
    def __init__(s, h=600.0, blocked=False, stalled=False):
        s.h, s.x, s.t, s.blocked, s.stalled = h, 0.0, 0.0, blocked, stalled
        s.hw = HwState(h, False, False, 0, 0.0); s.log = []
    def step(s, core, dt=0.05):
        s.t += dt
        s.hw = HwState(s.h, s.h >= 1599, s.h <= 1, 0, s.t)
        o = core.tick(s.t, s.hw, (s.x, 0.0, 0.0))
        if o.fork_dir != "S" and not s.stalled:
            s.h += (1 if o.fork_dir == "U" else -1) * 0.6 * o.fork_speed * dt * (100 / 100)
            s.h = max(0.0, min(1600.0, s.h))
        if o.drive and not s.blocked:
            s.x += o.lin * dt
        s.log.append((core.step_name(), o.lin, [b["id"] for b in o.bypass], o.fork_dir))
        return o
def run(core, sim, limit=4000):
    for _ in range(limit):
        sim.step(core)
        if core.state != "running":
            break
    return core.state

cfg = ForkConfig()
# ---- PICK
core = ForkliftCore(cfg); sim = Sim(h=600.0)
assert core.start(0.0, {"cmd": "pick", "id": "t1"})[0]
assert run(core, sim) == "done", core.message
print("pick: done, height %.0f mm, carrying %s, final x %.2f m, duration %.0f s" % (sim.h, core.carrying, sim.x, sim.t))
assert core.carrying and abs(sim.h - cfg.lift_height_mm) <= cfg.height_tol_mm + 2 and abs(sim.x - (1.0 - 1.3)) < 0.06
ins = [l for l in sim.log if l[0] == "insert" and l[1] != 0]; ret = [l for l in sim.log if l[0] == "retract" and l[1] != 0]
assert ins and all("fork_insert" in l[2] for l in ins) and all(abs(l[1] - 0.08) < 1e-9 or abs(l[1] - 0.04) < 1e-9 for l in ins)
assert ret and all("fork_insert" not in l[2] for l in ret), "no bypass while reversing out (rear zone stays active)"
assert any("fork_load" in l[2] for l in ret), "own-load rectangle published while carrying"
assert all(l[3] == "S" for l in ins + ret), "fork does not move while driving"
# the insert bypass rectangle sits in front of the tips and is as wide as fork + margin
rect = core._insert_rect(); print("insert rect:", [round(v, 2) for v in rect]); assert rect[1] > cfg.front_x_m and rect[0] < cfg.front_x_m
# ---- DROP (carrying)
sim2 = Sim(h=cfg.lift_height_mm); sim2.x = 0.0
assert core.start(sim.t, {"cmd": "drop", "id": "t2", "drop_height_mm": 900}) == (True, "started")
sim2.t = sim.t
assert run(core, sim2) == "done", core.message
print("drop: done, height %.0f mm, carrying %s, final x %.2f" % (sim2.h, core.carrying, sim2.x))
assert not core.carrying and abs(sim2.h - cfg.travel_height_mm) <= cfg.height_tol_mm + 2
steps_seen = [] ; [steps_seen.append(l[0]) for l in sim2.log if l[0] and (not steps_seen or steps_seen[-1] != l[0])]
print("drop steps:", steps_seen); assert [x for x in steps_seen if x != "carry"] == ["raise_to_level", "insert", "release", "retract", "travel_height"]
# ---- failure modes
c = ForkliftCore(cfg); c.start(0.0, {"cmd": "lift", "height_mm": 500, "id": "f1"}); s = Sim(h=100.0, stalled=True)
assert run(c, s) == "error" and "stalled" in c.message, c.message; print("stall ->", c.message)
c = ForkliftCore(cfg); c.start(0.0, {"cmd": "insert", "id": "f2"}); s = Sim(blocked=True)
assert run(c, s) == "error" and "blocked" in c.message, c.message; print("blocked ->", c.message)
c = ForkliftCore(cfg); c.start(0.0, {"cmd": "lift", "height_mm": 800, "id": "f3"})
assert c.tick(0.1, None, (0, 0, 0)).fork_dir == "S" and c.state == "error" and "no fork state" in c.message; print("no hw ->", c.message)
c = ForkliftCore(cfg); c.start(0.0, {"cmd": "lift", "height_mm": 1500, "id": "f4"})
o = c.tick(0.1, HwState(1000.0, True, False, 0, 0.1), (0, 0, 0)); assert c.state == "error" and "limit" in c.message; print("top limit ->", c.message)
c = ForkliftCore(cfg); c.start(0.0, {"cmd": "lower", "height_mm": 0, "id": "f5"})
o = c.tick(0.1, HwState(5.0, False, True, 0, 0.1), (0, 0, 0)); assert c.state == "done" or c.state == "running"
c = ForkliftCore(cfg); c.start(0.0, {"cmd": "lift", "height_mm": 500, "id": "f6"})
c.tick(0.1, HwState(100.0, False, False, 7, 0.1), (0, 0, 0)); assert c.state == "error" and "fault" in c.message
c = ForkliftCore(cfg); assert c.start(0, {"cmd": "nonsense"}) == (False, "unknown command")
c.start(0, {"cmd": "lift", "height_mm": 500}); assert c.start(0, {"cmd": "lower"}) == (False, "busy")
c.abort("estop"); assert c.state == "error" and c.message == "estop"
c.reset(); assert c.state == "idle"
c.start(0, {"cmd": "set_carrying", "value": True}); assert c.carrying and c.tick(1.0, None, None).bypass[0]["cap"] is False
# lowering to the very bottom completes on the limit switch
c = ForkliftCore(cfg); c.start(0.0, {"cmd": "lower", "height_mm": 0, "id": "f7"})
sim = Sim(h=40.0); assert run(c, sim) == "done"
print("FORKLIFT LOGIC OK")
