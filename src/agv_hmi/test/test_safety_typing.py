import sys, os, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); os.environ["HOME"] = tempfile.mkdtemp()
from PyQt6.QtWidgets import QApplication
from PyQt6.QtTest import QTest
from PyQt6.QtCore import Qt
app = QApplication([])
from agv_hmi.ui.safety_page import SafetyPage
pg = SafetyPage(); pg.resize(1300, 800); pg.show(); app.processEvents()

def type_into(sp, text):
    sp.setFocus(); sp.lineEdit().selectAll()
    QTest.keyClicks(sp.lineEdit(), text); app.processEvents()
    QTest.keyClick(sp.lineEdit(), Qt.Key.Key_Tab); app.processEvents()
    return sp.value(), sp.lineEdit().text()

for label, sp, text, expect in (
    ("slow left  (default 0.40, stop 0.10) type 0.55", pg._zs[("slow", "left")], "0.55", 0.55),
    ("slow front (default 1.00, stop 0.25) type 0.80", pg._zs[("slow", "front")], "0.80", 0.80),
    ("stop front type 0.30", pg._zs[("stop", "front")], "0.30", 0.30),
    ("length type 2.4", pg._len, "2.4", 2.4),
    ("lidar x type -0.6", pg._lx, "-0.6", -0.6),
    ("slow speed type 0.12", pg._slin, "0.12", 0.12)):
    v, t = type_into(sp, text)
    assert abs(v - expect) < 1e-6, (label, v, t)
assert pg.editor.p['slow']['left'] == 0.55 and pg.editor.p['stop']['front'] == 0.30
print('TYPING OK')
