import sys, os, tempfile, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, os.path.join(HERE, "..", "..", "hiep_robot2")); os.environ["HOME"] = tempfile.mkdtemp()
from PyQt6.QtWidgets import QApplication
app = QApplication([])
from agv_hmi.core import robot_profile as RP
from agv_hmi.ui.main_window import MainWindow, Sidebar
from unittest import mock
w = MainWindow(role="engineer"); w.resize(1500, 900); w.show()
sent = []; w.safety_config_signal.connect(sent.append)
pg = w._safety
w._sidebar.switch(Sidebar.IDX_SAFETY); app.processEvents()
# forklift flow end to end with typed values: create a fork-lift profile, type margins, apply, the ROS config gets them
with mock.patch("agv_hmi.ui.safety_page.QInputDialog.getText", return_value=("FL-test", True)):
    pg._new_profile(False, "forklift")
from PyQt6.QtTest import QTest
from PyQt6.QtCore import Qt
def type_into(sp, text):
    sp.setFocus(); sp.lineEdit().selectAll(); QTest.keyClicks(sp.lineEdit(), text); QTest.keyClick(sp.lineEdit(), Qt.Key.Key_Tab); app.processEvents()
show = lambda tag: None
type_into(pg._zs[("slow", "front")], "1.40"); show("after slow front")
type_into(pg._zs[("stop", "front")], "0.45"); show("after stop front")
type_into(pg._zs[("slow", "left")], "0.55"); show("after slow left")
type_into(pg._len, "2.6"); show("after length"); type_into(pg._flen, "1.2"); show("after fork len"); type_into(pg._lx, "0.0"); show("after lidar x")
pg.apply(); cfg = json.loads(sent[-1]); print("published:", cfg["name"], cfg["robot_type"], cfg["length_m"], cfg["fork_length_m"], cfg["stop"]["front"], cfg["slow"]["front"], cfg["slow"]["left"], cfg["lidar_x_m"])
assert cfg["robot_type"] == "forklift" and cfg["length_m"] == 2.6 and cfg["fork_length_m"] == 1.2
assert cfg["stop"]["front"] == 0.45 and cfg["slow"]["front"] == 1.4 and cfg["slow"]["left"] == 0.55 and cfg["lidar_x_m"] == 0.0
st = RP.load_store(); assert st["active"] == "FL-test" and RP.active_profile(st)["slow"]["front"] == 1.4
# the node-side parser accepts exactly what the HMI sends
from hiep_robot2.safety_logic import Profile
node_prof = Profile.from_dict(cfg); assert node_prof.signature() == RP.signature(cfg); assert node_prof.slow["front"] == 1.4 and node_prof.stop["front"] == 0.45
print("FORKLIFT SETUP FLOW OK")
