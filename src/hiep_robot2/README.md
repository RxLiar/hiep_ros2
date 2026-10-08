# hiep_robot2 v0.4.0

Hardware layer for the AMR project. Navigation, SLAM, AMCL and EKF remain in
`mec_mobile_navigation`.

## Architecture

Traction and conveyor/robot-I/O are intentionally independent:

```text
                       ROS 2 / agv_hmi / Nav2
                         |              |
                      /cmd_vel     /conveyor_cmd
                         |              |
          +--------------+              +----------------+
          |                                                |
   ESP32 traction OR KEYA                         Mega2560 PLC
          |                                                |
    drive motors                         4 conveyors + 8 sensors
                                         bumper L/R
                                         EMG / START / STOP
                                         4 signal outputs
```

This lets the same Mega2560 PLC code run with either traction backend.

## Mega2560 PLC I/O model

There are **4 conveyors**. Each conveyor has exactly **2 end sensors**:

```text
Conveyor 1: Sensor 1 = A, Sensor 2 = B
Conveyor 2: Sensor 3 = A, Sensor 4 = B
Conveyor 3: Sensor 5 = A, Sensor 6 = B
Conveyor 4: Sensor 7 = A, Sensor 8 = B
```

Current software convention:

- `receive`: run A -> B; stop when sensor B detects cargo.
- `send`: run B -> A; after sensor A sees cargo and then clears, stop and mark
  the conveyor empty.
- HMI `duration > 0`: normal time-based stop if transfer has not completed first.
- HMI `duration == 0`: sensor-controlled operation, but firmware still enforces
  `MAX_RUN_TIME_MS` as a hard local safety timeout.

If the real mechanics use the opposite direction, change only the GPIO/direction
configuration at the top of `firmware/mega2560_plc/mega2560_plc.ino`.

## PLC ROS topics

Input to PLC bridge:

- `/conveyor_cmd` (`std_msgs/String`, JSON) - already published by `agv_hmi`.
- `/light_cmd` (`std_msgs/String`, JSON) - four signal outputs.

Output from PLC bridge:

- `/sensor_states` - existing HMI format: `{sensor_id, state}`; IDs 1..8.
- `/conveyor_cargo` - existing HMI format: `{belt_id, has_cargo}`; belts 1..4.
- `/bumper_states` - existing HMI format: `{side, triggered}`.
- `/conveyor_status` - detailed per-conveyor status.
- `/emergency_stop` (`Bool`).
- `/start_button` (`Bool`).
- `/stop_button` (`Bool`).
- `/plc_connection_status` (`String`: online/offline).
- `/plc_status` - complete PLC JSON status.
- `/signal_light_states` - current four-light output state.

`/plc_connection_status` is deliberately separate from the traction
`/connection_status`, so ESP32/KEYA and PLC can run at the same time.

## Serial protocol - Mega2560 PLC

ROS -> PLC:

```text
CV,<seq>,<conveyor_id>,R|S|X,<speed_percent>,<duration_ms>
LED,<seq>,<4_bit_mask>
STOPALL,<seq>
```

PLC -> ROS:

```text
ID,AGV_PLC_MEGA2560,0.4.0
PSTAT,<seq>,<ms>,<sensor_mask>,<bumper_mask>,<emg>,<start>,<stop>,<cargo_mask>,<running_mask>,<fault_mask>,<led_mask>,<m1>,<m2>,<m3>,<m4>
```

The bridge passively auto-detects the PLC serial port. No `/dev/ttyUSB0` number
is hard-coded.

## Firmware GPIO

Edit only the `USER HARDWARE CONFIG` section in:

```text
firmware/mega2560_plc/mega2560_plc.ino
```

The included pin numbers are compilable examples for a normal Arduino Mega, not
an assertion about the terminal mapping of a particular industrial PLC board.

Important: EMG should also disable hazardous motion through a real hardware
safety circuit. Reading EMG in firmware/ROS is for state reporting and an
additional software interlock; it must not be the only emergency-stop path.

## Build

```bash
cd ~/hiep_ros2
rm -rf build/hiep_robot2 install/hiep_robot2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hiep_robot2
source install/setup.bash
```

## Hardware profiles (one entry point)

```bash
# Test model: ESP32 + L298N + encoders  (traction only)
ros2 launch hiep_robot2 agv_hardware.launch.py profile:=model

# Real AGV: KEYA traction + Mega2560 PLC (conveyors, bumpers, EMG, lights)
ros2 launch hiep_robot2 agv_hardware.launch.py profile:=real

# Override the PLC default (auto = off for model, on for real)
ros2 launch hiep_robot2 agv_hardware.launch.py profile:=model plc:=true
```

The Mega2560 firmware does **not** drive the traction motors; it owns conveyors
and robot I/O. Traction is `esp32_bridge_node` (model) or
`keya_driver_bridge_node` (real). The older per-backend launch files below still work.

## Launch

ESP32 traction only:

```bash
ros2 launch hiep_robot2 esp32_hardware.launch.py
```

Mega2560 PLC only:

```bash
ros2 launch hiep_robot2 mega2560_plc.launch.py
```

ESP32 traction + Mega2560 PLC together:

```bash
ros2 launch hiep_robot2 esp32_with_mega2560_plc.launch.py
```

KEYA traction + Mega2560 PLC together:

```bash
ros2 launch hiep_robot2 keya_with_mega2560_plc.launch.py
```

## Example conveyor test

Conveyor 1 receive, 30% speed, 5 s maximum requested duration:

```bash
ros2 topic pub --once /conveyor_cmd std_msgs/msg/String \
  "{data: '{\"conveyor_id\":1,\"mode\":\"receive\",\"speed\":30,\"duration\":5.0}'}"
```

Stop conveyor 1:

```bash
ros2 topic pub --once /conveyor_cmd std_msgs/msg/String \
  "{data: '{\"conveyor_id\":1,\"mode\":\"stop\",\"speed\":0,\"duration\":0}'}"
```

Monitor:

```bash
ros2 topic echo /plc_connection_status
ros2 topic echo /sensor_states
ros2 topic echo /conveyor_status
ros2 topic echo /conveyor_cargo
ros2 topic echo /bumper_states
ros2 topic echo /emergency_stop
```

## HMI note

`agv_hmi` 4.4.0 shows 8 sensor cells (4 conveyors x 2), plus PLC / e-stop /
drive-motor state on the Home page and a motor enable button.


## Changes in 0.4.0

Bridges
- ESP32 bridge: serial reader no longer holds the port lock during `readline()`,
  which could stall the 50 Hz command writer and the whole ROS executor.
- ESP32 bridge: odometry baseline is reset on every (re)connect and when the MCU
  uptime goes backwards (ESP32 reboot). Before, a reset made the encoder counters
  jump to ~0 and the pose teleported.
- Both bridges open ports with `exclusive=True`. With ESP32 + PLC launched
  together, the two auto-detect scans previously raced on the same ttys and could
  steal each other's bytes.
- PLC bridge: re-publishes one sensor, one belt and one bumper per PSTAT frame
  (round-robin, ~0.8 s for all sensors). `agv_hmi` subscribes BEST_EFFORT/depth 1
  and starts after the bridge, so change-only publishing left it stale.

Firmware (both)
- Non-blocking line parser (replaces `readBytesUntil` with a 2 ms timeout that
  returned partial lines and dropped commands). Handles `\r\n`, garbage and
  over-long lines.
- Mega2560: `SENSOR_PIN_MODE` / `SAFETY_PIN_MODE` (INPUT or INPUT_PULLUP) so
  floating inputs and NC e-stop wiring can be fail-safe.

### KEYA traction bridge (real robot: ROS -> USB-RS485 -> KEYA, no Mega in the motor path)

- `transport: rs485` is now accepted (it used to raise). The same 12-byte frames
  are sent; **confirm in your exact KYDAS manual that the RS485 port uses the same
  protocol/baud/addressing** (the code was written from the RS232 manual). Use an
  adapter with automatic direction control. If your adapter echoes TX on RX, set
  `rs485_echo_filter: true` (our query frames start with 0xED like a real reply).
- Odometry now carries covariances (it had none, which lets robot_localization's
  EKF diverge), ignores impossible encoder jumps (driver restart) and resets its
  baseline on reconnect.
- Serial reads no longer hold the lock, so the 40 Hz command writer is not delayed.
- Software e-stop interlock: subscribes `/emergency_stop` (published by the PLC
  bridge at 10 Hz). Pressing it disables + latches the output; releasing it does
  **not** re-enable. `/motor_enable` is refused while e-stop is active. This is a
  software layer only: the real e-stop must cut driver power in hardware.
- `arm_on_start: false` means the robot will not move until motors are enabled:
  use the new HMI "Enable motors" button or
  `ros2 service call /motor_enable std_srvs/srv/SetBool "{data: true}"`.
- `/keya_driver_status` JSON now includes `estop`.
- The PLC bridge republishes `/plc_connection_status` once per second.

## Things to check on the real robot (not changed here)

- `mec_mobile_navigation/config/ekf.yaml` reads `odom0: /odom` (simulation),
  while the bridges publish `/wheel/odom`. Point the EKF at `/wheel/odom` for
  hardware.
- Nav2 asks for `desired_linear_vel: 0.8` and the smoother allows 1.0 m/s, but the
  bridges clamp to `max_linear_speed: 0.5`. Raise the bridge limit or lower Nav2's.
- `keya_hardware.yaml`: `wheel_radius`, `wheel_separation`, `gear_ratio` are
  placeholders marked "must be measured" (the AGV footprint is 1.9 x 0.8 m).
- ESP32 firmware uses `analogWrite`; this needs arduino-esp32 core 3.x. On 2.x use
  `ledcAttach`/`ledcWrite`.
- Auto-detect opens every serial port, and opening an Arduino Mega toggles DTR,
  which resets it. On the real robot prefer fixed udev symlinks
  (`serial_port: /dev/agv_plc`, `auto_detect_serial: false`).
- `agv_hmi` 4.4.0 now shows PLC/e-stop/driver state and has the motor enable button
  (see HMI notes). It does not yet pause a running mission on e-stop.
- If the PLC dies, nothing publishes `/emergency_stop`: add a hardware e-stop path
  and decide whether KEYA should disarm when the PLC goes offline.

## Safety monitor (0.5.0): lidar SAFETY (slow) + STOP zones

```
Nav2 / HMI joystick -> /cmd_vel -> [safety_monitor_node] -> /cmd_vel_safe -> KEYA / ESP32 bridge
```
- `ros2 launch hiep_robot2 agv_hardware.launch.py profile:=real` starts it by default (`safety:=auto`: on for
  `real`, off for `model`). `safety:=true|false` overrides. With safety on, the bridge listens to `/cmd_vel_safe`
  (launch argument `cmd_vel_topic`), so if the monitor dies the bridge stops receiving commands and halts the robot.
- Robot size, body-centre offset, lidar pose and the two rectangles come from `~/.agv_hmi/robot_profile.json`
  (written by the HMI page "Robot & Safety") and live from `/safety_config` (latched JSON). The active profile is
  re-published on `/robot_profile`.
- SAFETY rectangle: lidar points inside it limit the speed (`slow_linear_mps`, `slow_angular_rps`), only when moving
  towards them (or turning). STOP rectangle: motion towards the points is blocked, reversing away stays possible,
  rotation is blocked. Returns from the robot's own body are ignored.
- Fail safe: no `/scan` for `scan_timeout_s` (and `require_scan`) => output zero, state `no_scan`.
- `/safety_state` (JSON, 10 Hz): `ok | slow | stop | no_scan | disabled`, nearest object, counters.
- Nav2 footprint: the node writes the footprint polygon to `/local_costmap/local_costmap` and
  `/global_costmap/global_costmap` (parameter `footprint`) when they are available. This relies on Nav2 accepting
  the dynamic parameter; if your Nav2 rejects it (check the node log), edit `footprint` in the Nav2 yaml
  (the HMI shows the string to paste). The Gazebo URDF is NOT changed.
- Software layer only: it does not replace the hardware e-stop or a safety-rated lidar / PLC chain.
