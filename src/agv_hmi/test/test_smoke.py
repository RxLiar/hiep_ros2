"""Smoke test: build the main window for both roles, visit every page, retranslate, feed typical ROS messages."""
import json, os, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
os.environ["HOME"] = tempfile.mkdtemp()
from PyQt6.QtWidgets import QApplication
app = QApplication([])
from agv_hmi.ui.main_window import MainWindow, Sidebar
for role in ("engineer", "operator"):
    w = MainWindow(role=role); w.resize(1500, 900); w.show()
    for idx in range(15):
        try:
            w._sidebar.switch(idx); app.processEvents()
        except Exception as exc:
            raise AssertionError(f"{role}: page {idx} failed: {exc}")
    w._retranslate_all()
    w.on_safety_state(json.dumps({"state": "slow", "nearest_m": 0.4, "bypass": [], "out_subscribers": 1, "direct_consumers": [], "profile_sig": "x"}))
    w.on_forklift_status(json.dumps({"state": "idle", "height_mm": 100.0, "carrying": False, "top": False, "bottom": False, "progress": 0}))
    w.on_topic_stats({"/scan": 10.0}); w.on_estop_update(False)
print("SMOKE OK")
