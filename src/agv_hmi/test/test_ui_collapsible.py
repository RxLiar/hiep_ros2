"""4.7.0 UI: collapsible panels / groups, sidebar rail, focus mode, joystick chip, animation switch."""
import os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
os.environ["HOME"] = tempfile.mkdtemp()
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt6.QtWidgets import QApplication, QGroupBox
app = QApplication([])
from agv_hmi.core import prefs
from agv_hmi.ui.collapsible import CollapsiblePanel, GroupCollapser
from agv_hmi.ui.joystick_widget import JoystickWidget
from agv_hmi.ui.main_window import MainWindow, Sidebar


def spin(ms):
    end = time.time() + ms / 1000.0
    while time.time() < end:
        app.processEvents(); time.sleep(0.005)


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)


w = MainWindow(role="engineer"); w.resize(1500, 900); w.show(); spin(100)

# -- side panel hides with an animation and comes back
w._sidebar.switch(Sidebar.IDX_NAV); spin(300)
panel = w._nav._panel
full = panel._clip.width()
check(full == 392, f"expanded width {full}")
panel.set_collapsed(True); spin(60)
check(0 < panel._clip.width() < full, "panel should be mid-animation")
spin(400)
check(panel._clip.width() == 0 and panel.is_collapsed() and not panel.content().isVisible(), "panel hidden")
check(prefs.get("ui_panels").get("nav.panel") is True, "state persisted")
panel._handle.click(); spin(450)
check(panel._clip.width() == full and panel.content().isVisible(), "panel shown again")

# -- group boxes fold and unfold
box = [g for g in w._nav.findChildren(QGroupBox) if g.findChild(GroupCollapser) is not None][0]
gc = box.findChild(GroupCollapser)
h0 = box.height()
gc.set_collapsed(True); spin(400)
check(box.height() < h0 / 1.5, f"group folded ({box.height()} vs {h0})")
gc.set_collapsed(False); spin(400)
check(abs(box.height() - h0) <= 4 and box.maximumHeight() > 10000, f"group restored ({box.height()} vs {h0})")

# -- sidebar rail
sb = w._sidebar
w.toggle_sidebar(); spin(450)
check(sb.width() == Sidebar.COMPACT_W and prefs.get("ui_sidebar_compact") is True, f"rail {sb.width()}")
w.toggle_sidebar(); spin(450)
check(sb.width() == Sidebar.EXPANDED_W, f"expanded {sb.width()}")

# -- focus mode hides the sidebar and the side panels, and restores them
w.toggle_focus_mode(); spin(500)
check(sb.isHidden() and panel.is_collapsed(), "focus mode on")
w._sidebar.switch(Sidebar.IDX_ROUTES); spin(100)   # switching pages keeps focus mode
w.toggle_focus_mode(); spin(500)
check(sb.isVisible() and sb.width() == Sidebar.EXPANDED_W and not panel.is_collapsed(), "focus mode off")

# -- joystick chip
j = w._nav._joystick
check(not j.is_collapsed(), "joystick open")
j.toggle_collapsed(); spin(400)
check(j.is_collapsed() and j.mask().boundingRect().width() < 60, "joystick folded to the chip")
j.toggle_collapsed(); spin(400)
check(not j.is_collapsed() and j.mask().isEmpty(), "joystick open again")

# -- animations can be switched off: changes are instant
prefs.set("ui_animations", False)
panel.set_collapsed(True)
check(panel._clip.width() == 0, "instant collapse without animations")
panel.set_collapsed(False)
prefs.set("ui_animations", True)

# -- every page still builds, retranslate keeps working
for idx in range(15):
    w._sidebar.switch(idx); spin(30)
w._retranslate_all()
print("UI COLLAPSIBLE OK")
