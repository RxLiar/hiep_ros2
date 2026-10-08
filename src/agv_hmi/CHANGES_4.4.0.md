# agv_hmi 4.4.0 - hardware layer (hiep_robot2)

Home page
- "Enable/Disable motors" button -> calls std_srvs/SetBool `/motor_enable`
  (KEYA bridge starts disabled: without this the robot never moves from Nav2/joystick).
  If the service is missing a warning explains that the KEYA bridge is not running.
- "Drive motors" card from `/keya_driver_status`: enabled / disabled / driver offline /
  fault codes / e-stop. No data for 3 s (e.g. ESP32 test model) shows "no data", not "off".
- "PLC / E-stop" card from `/plc_connection_status` + `/emergency_stop`. If the PLC is
  offline the card shows "PLC offline", never "normal" (e-stop state is unknown).

Conveyor page
- 8 sensor cells (4 conveyors x 2) instead of 12; ids 9..12 are no longer drawn.

Other
- ErrorHeader warning prefix is translated (`warning_prefix`).
- i18n: 15 new keys in vi/en/ko/ja/zh.
- package.xml: added `std_srvs` dependency.

Files: main.py, ros/ros_node.py, ui/home_page.py, ui/main_window.py,
ui/conveyor_panel.py, ui/error_header.py, ui/i18n.py, version.py, package.xml

Not done: a running mission is not paused on e-stop (Nav2 keeps sending cmd_vel, the KEYA
bridge just ignores it while disabled).
