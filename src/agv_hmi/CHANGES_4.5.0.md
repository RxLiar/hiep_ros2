# agv_hmi 4.5.0

## Map
- `core/map_cleaner.py`: denoise (small blobs / speckle), RANSAC straight wall extraction, snap to dominant
  perpendicular directions, merge collinear pieces and bridge gaps up to `max_gap_m`, join corners/T-junctions,
  drop echo "ghost" lines, redraw walls with uniform thickness. Pillars/machines (non-wall clusters) are kept.
- Mapping page: "Clean map" panel (5 parameters), preview in a background thread, "Show raw map", "Save clean map".
  The HMI writes its own `<name>.pgm/.yaml` (+ `<name>.walls.json` for the vector walls) - it does not use RViz or
  map_saver for the cleaned map. Live SLAM updates are paused while the cleaned map is shown.
- Map widget restyled: soft palette, antialiased vector walls, 1 m grid, scale bar, follow robot, measure tool, minimap.
  Library/Navigation/Routes maps now load through the same renderer (walls + zones are loaded automatically).
- Keep-out / slow zones: draw polygons on the map; saved as `<name>.zones.json`; keep-out zones also export
  `<name>_keepout.pgm/.yaml` (Nav2 costmap filter mask).

## Safety / operation
- Soft STOP button (title bar): zero velocity, motors off, mission cancelled. Not a replacement for the hardware e-stop.
- Physical e-stop (from PLC): red banner, running mission cancelled, motor stays off until re-enabled.
- Motor enable needs a 1 s hold; disable is immediate.
- Alarm center (page + title-bar badge): PLC/e-stop/KEYA offline/fault/lidar lost/localization poor/battery/bumpers/Nav2/ROS.
  History in `~/.agv_hmi/alarms.jsonl`. Toasts and optional beep.
- Pre-flight check before starting a route/mission (only devices that have been seen are required, so the ESP32
  test model is not blocked). Engineers can override.
- Gamepad (`/joy`) teleop with dead-man button. Tower lights via `/light_cmd` (OFF by default; masks are configurable).
- Audit log (`~/.agv_hmi/audit.jsonl`): start/stop, motor, e-stop, map save, queue runs...

## Monitoring
- Diagnostics page: topic rates (/scan, /odom, /wheel/odom, /amcl_pose, /cmd_vel), device state, live plots
  (cmd vs actual speed, wheel rpm, driver voltage), /rosout viewer with filter/export, rosbag record (manual, or
  60 s after a critical alarm), start/stop `hiep_robot2` hardware launch (profile model/real) from the HMI.
- Home: dashboard (speed, distance, moving time, battery, missions/cargo today, alarms) + top-view vehicle diagram.
- Conveyor page: 8 sensor cells. Reports page: per-day mission statistics, chart, CSV export, audit log (engineer).

## Missions
- Stations page: save robot pose as station (normal/charge/home), set robot pose at a station, go to station,
  "go to charging station".
- Schedule page (engineer): weekly timed jobs + run queue. Automatic start is OFF by default and, when enabled,
  only starts if the pre-flight checks pass and no e-stop is active; otherwise a "schedule blocked" alarm is raised.

## Settings
- System box: pre-flight, sound, tower light masks, gamepad mapping, touch/fullscreen mode, backup/restore (zip of
  maps, routes, settings).

## Dependencies
`python3-scipy`, `python3-numpy`, `python3-yaml`, `sensor_msgs`, `rcl_interfaces`, `std_srvs`. Optional: `ros-jazzy-joy`.

## Robot profile & safety zones (page "Robot & Safety", Engineer)
- Several named robot profiles (length, width, body-centre offset, lidar pose) - pick one per robot type.
- Top-view editor: drag the dots on the edges of the SAFETY (orange, slow) and STOP (red) rectangles, or type the
  margins; live lidar points are coloured by zone. Slow always contains stop.
- "Apply & save" writes `~/.agv_hmi/robot_profile.json` and publishes `/safety_config` for `hiep_robot2/safety_monitor_node`.
- The configured size is used to draw the robot on every map, and the zones are drawn around it.
- Alarms: obstacle in the STOP zone, safety monitor lost the lidar. Pre-flight check includes the safety monitor
  (only once it has been seen). Dashboard card shows the safety state.
