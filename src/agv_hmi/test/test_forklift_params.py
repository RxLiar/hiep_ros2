"""Fork lift page: parameters can be typed, saved to robot_profile.json, sent to ROS, and checked against the node."""
import json, os, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, os.path.join(HERE, "..", "..", "hiep_robot2"))
os.environ["HOME"] = tempfile.mkdtemp()
from unittest import mock
from PyQt6.QtWidgets import QApplication
from PyQt6.QtTest import QTest
from PyQt6.QtCore import Qt
app = QApplication([])
from agv_hmi.core import robot_profile as RP
from agv_hmi.ui.main_window import MainWindow, Sidebar
w = MainWindow(role="engineer"); w.resize(1500, 900); w.show()
sent = []; w.safety_config_signal.connect(sent.append)
with mock.patch("agv_hmi.ui.safety_page.QInputDialog.getText", return_value=("FL-1", True)):
    w._safety._new_profile(False, "forklift")
w._safety.apply()
w._sidebar.switch(Sidebar.IDX_FORKLIFT); app.processEvents()
fp = w._forklift; fp._tabs.setCurrentIndex(1); app.processEvents()
def type_into(sp, text):
    sp.setFocus(); sp.lineEdit().selectAll(); QTest.keyClicks(sp.lineEdit(), text); QTest.keyClick(sp.lineEdit(), Qt.Key.Key_Tab); app.processEvents()
type_into(fp._pw[("fork", "insert_depth_m")], "1.25"); type_into(fp._pw[("fork", "lift_height_mm")], "250"); type_into(fp._pw[("top", "fork_length_m")], "1.15")
type_into(fp._pw[("fork", "insert_speed_mps")], "0.06"); type_into(fp._pw[("fork", "stall_s")], "3")
assert "Chưa lưu" in fp._params_status.text() or "Not saved" in fp._params_status.text()
fp._save_params(); app.processEvents()
cfg = json.loads(sent[-1]); print("published:", cfg["name"], cfg["fork"]["insert_depth_m"], cfg["fork"]["lift_height_mm"], cfg["fork_length_m"], cfg["fork"]["insert_speed_mps"], cfg["fork"]["stall_s"])
assert cfg["fork"]["insert_depth_m"] == 1.25 and cfg["fork"]["lift_height_mm"] == 250 and cfg["fork_length_m"] == 1.15 and cfg["fork"]["stall_s"] == 3.0
stored = RP.active_profile(RP.load_store()); assert stored["fork"]["insert_depth_m"] == 1.25 and stored["fork_length_m"] == 1.15, "saved in ~/.agv_hmi/robot_profile.json"
assert os.path.exists(RP.PROFILE_FILE); print("file:", RP.PROFILE_FILE)
# the test fields follow the saved values; the page shows them after a reload
assert fp._p["depth"].value() == 1.25 and fp._pw[("fork", "insert_depth_m")].value() == 1.25
# node sync check: node reports old values -> warning; same values -> ok
node_cfg = RP.fork_effective_config(RP.active_profile(RP.load_store()))
fp.feed_status({"state": "idle", "cfg": dict(node_cfg, insert_depth_m=1.0)}); print("sync:", fp._sync_lbl.text()); assert "insert_depth_m" in fp._sync_lbl.text()
fp.feed_status({"state": "idle", "cfg": node_cfg}); assert "✅" in fp._sync_lbl.text()
fp.feed_status({"state": "idle"}); assert fp._sync_lbl.text()
# restart: values come back from the file
w2 = MainWindow(role="engineer"); assert w2._forklift._pw[("fork", "insert_depth_m")].value() == 1.25
# defaults button marks the page as unsaved without touching the file
fp._restore_defaults(); assert fp._pw[("fork", "insert_depth_m")].value() == 1.0 and RP.active_profile(RP.load_store())["fork"]["insert_depth_m"] == 1.25
# guide + path label present
assert "Fork" in fp._guide.toPlainText() and "robot_profile.json" in fp._where_lbl.text() and "routes.json" in fp._where_lbl.text()
fp.grab().save("/tmp/forklift_params.png")
print("FORKLIFT PARAMS OK")
