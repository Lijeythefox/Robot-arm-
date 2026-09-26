# DUM-E firmware

Firmware for the two custom boards of the DUM-E arm (LoboCNC WE-R2.4, based on
[diy_robotics](https://github.com/mathias31415/diy_robotics)):

| Folder | Board | MCU |
|---|---|---|
| `arm_controller/` | DUM-E arm controller v5 (6x DRV8825, endstops, E-stop, OLED) | ESP32-DevKitC-32D |
| `claw_controller/` | ClawGripperPCB (3x N20 tendon fingers, 2x TB6612FNG, 3x INA219) | Waveshare ESP32-C5-MINI-KIT |
| `shared/` | wire protocol headers, WiFi/TCP frame server, `tools/dume_cli.py` | |

Both keep the original diy_robotics 64-byte TCP protocol, so the stock ROS2 packages
(`diy_robotarm_wer24_driver`, `diy_soft_gripper_driver`) keep working unchanged. New features
live in the unused tail of each frame (see [Protocol](#protocol)).

## Hardware review notes

Checked against `DUM-E_arm_controller_v5.kicad_sch` + `.kicad_pcb` (their netlists match
pin for pin) and `ClawGripperPCB.kicad_sch` (the claw `.kicad_pcb` was not available).
Everything in the firmware brief matched the boards. Notes for bench testing and the next
board revision (none of these change the firmware):

- **Arm E-stop:** DRV8825 FLT and EN are unconnected on all six footprints, so SLP only sees
  the ENABLE net and an open E-stop really does sleep the drivers. When open, the ENABLE side
  of the E-stop cable is held low only by the six drivers' internal pull-downs; a 10k
  pull-down on ENABLE would make it stiffer against noise on a long cable.
- **Arm VMOT:** the P6KE43A TVS has a 36.8 V stand-off and starts conducting from ~41 V. At
  36 V there is little margin: keep the Mean Well's trim pot at 36.0 V, not higher. Its clamp
  voltage at full pulse current (~59 V) is above the DRV8825's 47 V absolute maximum, so it
  absorbs energy but cannot guarantee a limit. Moderate `ACC_MAX` keeps regenerated energy low
  when big joints decelerate.
- **Arm title blocks:** the schematic says v4 and the PCB "diy_robotics_arm V03"; the
  content is the v5 design.
- **Arm strapping pins:** a closed endstop on GPIO15 at boot only silences the ROM boot log.
  GPIO2 (endstop 2) is LOW or floating at boot as download mode requires.
- **Claw USB/5 V:** the Waveshare board's VBUS pin is on the J4 +5 V net (see the claw
  section).
- **Claw strapping pins:** the LEDs on GPIO25/26 are boot-mode strapping pins on the ESP32-C5.
  If a board ever fails to boot normally with the LEDs fitted, that is the first suspect.
  C_PWM on GPIO2 (MTMS) only matters for SDIO boot.

## Values still to measure (TBD)

| Where | What |
|---|---|
| arm `AXIS_n_HOME_DIR`, `_HOME_SWITCH_DEG` | which end each switch is at and the joint angle where it triggers |
| arm homing speeds / back-off / max travel | defaults are slow and safe |
| arm `AXIS_n_VEL_MAX` / `ACC_MAX` | from the speed test (start: 2x original) |
| claw `FINGER_x_INVERT` | which direction closes each finger |
| claw thresholds and times | from calibration mode (grip threshold, hard limit, efforts, open time/mode) |
| claw `INA_MAX_CURRENT_A` | N20 stall current at 5 V (sets the INA219 range) |
| claw return mechanism | elastic assumed; `OPEN_MODE` picks how opening ends |

## Building and flashing (Windows, VS Code + PlatformIO)

1. Install VS Code and the PlatformIO extension. Git must be installed (the platform needs it).
2. Open **one project folder at a time**: `arm_controller` or `claw_controller` (File > Open
   Folder). PlatformIO needs the folder that contains `platformio.ini`.
3. Copy `include/secrets.example.h` to `include/secrets.h` and fill in your WiFi SSID and
   password. `secrets.h` is git-ignored.
4. Check the network block at the top of `include/Configuration.h` (static IP, gateway).
5. Build (checkmark icon), connect the board by USB, Upload (arrow icon), then open the
   Serial Monitor (plug icon, 115200 baud). Type `help` for the console commands.

The first build downloads the pioarduino platform (arduino-esp32 3.3.12, ESP-IDF 5.5) and the
libraries from GitHub, which takes a few minutes.

### Why pioarduino

The ESP32-C5 is only supported by arduino-esp32 3.3+, and the official PlatformIO
`espressif32` platform stops at arduino-esp32 2.x. The community
[pioarduino](https://github.com/pioarduino/platform-espressif32) platform packages 3.3.x and is
what Espressif points PlatformIO users to. The arm uses the same pinned platform release
(`55.03.312`) so both boards share one toolchain. Libraries are pinned GitHub tags.

## Arm controller

### What changed from the original firmware

- **Pins, gearing, directions and the ROS2 protocol are unchanged.** `Configuration.h` keeps
  the original names (`STEPPER_n_STEP`, `AXIS_n_MICROSTEPS`, ...).
- Only the motion loop on core 1 touches the steppers. The network task on core 0 now sends it
  commands through FreeRTOS queues. (The original called `moveTo()` from the network task while
  core 1 was stepping the same objects.)
- The OLED is redrawn by its own low-priority task at 4 Hz, not inside the network callback on
  every message.
- The WiFi code (`shared/include/dume/FrameServer.hpp`, replacing `WiFiConnection.hpp`) no
  longer blocks, serves a second port (81) for tools, and accepts a reconnecting ROS2 driver
  without a reboot. `DataFormat.hpp` is replaced by `shared/include/dume/ArmProtocol.h`, which
  is byte-compatible.
- If a client stops streaming setpoints for `COMM_TIMEOUT_MS` (250 ms), all axes decelerate to a
  stop and hold.

### Serial console

115200 baud, newline terminated. `help` lists everything. Stage 1 commands:

| Command | Action |
|---|---|
| `status` | state, positions, homed flags, endstops, faults |
| `endstops` | toggle a live endstop monitor (5 Hz) |
| `enable` / `disable` | wake / sleep all drivers |
| `jog <axis> <deg>` | relative move of one axis (soft limits apply) |
| `home` / `home <axis>` | home all axes (in `HOMING_ORDER`) / one axis |
| `sethome <axis> <deg>` | declare an axis homed at `<deg>` where it stands (axes without a switch) |
| `sethome all` | declare every axis homed at its current position |
| `speedtest <axis> [start max increment]` | speed test (see below), steps/s |
| `stop` | decelerate all axes, abort homing / speed test |
| `reset` | clear a latched E-stop (only once it is closed again); otherwise clear fault flags |

### Endstops

J10-J15 (pin 1 = 3V3, 2 = SIG, 3 = GND). Use **normally-closed switches between SIG and GND**:
LOW = OK, HIGH = triggered *or broken wire*; both count as "at limit".

| Axis | Connector | GPIO | Board | Firmware |
|---|---|---|---|---|
| 1 | J10 | 15 | 10k pull-up, 1k series, 100 nF | input (strapping pin: a closed switch at boot only silences the boot log) |
| 2 | J11 | 2 | 100 nF only | `INPUT_PULLUP` (an external pull-up would block USB flashing) |
| 3 | J12 | 14 | 10k pull-up, 1k series, 100 nF | input |
| 4 | J13 | 36 | 10k pull-up, 100 nF | input only |
| 5 | J14 | 39 | 10k pull-up, 100 nF | input only |
| 6 | J15 | 35 | 10k pull-up, 100 nF | input only |

Config per axis: `AXIS_n_ENDSTOP_ENABLED` (set `false` for switches not fitted yet),
`AXIS_n_HOME_DIR` (-1 = switch at the POS_MIN end, +1 = POS_MAX end). `ENDSTOP_DEBOUNCE_MS`
(default 5 ms) is how long a level must be stable before it counts. That also rejects the
brief glitches GPIO36/39 can show while WiFi is active.

An endstop that reads HIGH while its axis moves **towards** it stops that axis immediately.
Moves away from it are still allowed, so a jog in the other direction frees the axis.

### Homing

Per axis: fast approach towards the switch, back off, slow re-approach, set the position, then
back off the switch again (and optionally park). "Home all" uses `HOMING_ORDER`
(default `{2, 3, 5, 1, 4, 6}`, big joints first) and skips axes without an enabled endstop.

| Config (per axis) | Meaning | Default (TBD) |
|---|---|---|
| `AXIS_n_HOME_DIR` | switch at the -1 (POS_MIN) or +1 (POS_MAX) end | -1 |
| `AXIS_n_HOME_SWITCH_DEG` | joint angle where the switch triggers | `AXIS_n_POS_MIN` |
| `AXIS_n_HOME_FAST_DEG_S` / `_SLOW_DEG_S` | approach speeds | 5 / 1 (axes 1-3), 8 / 2 (4-6) |
| `AXIS_n_HOME_BACKOFF_DEG` | back-off distance | 5 |
| `AXIS_n_HOME_MAX_TRAVEL_DEG` | fail if the switch is not found within this | soft-limit span + 20 |
| `AXIS_n_HOME_PARK_DEG` + `HOMING_PARK_AFTER` | optional move after homing | 0, off |

Homing never exceeds the axis' unhomed speed/accel limits. It fails (and says why on serial and
the OLED) if the switch is not found within the max travel, does not release after backing
off, or is already pressed and stays pressed (stuck switch or broken wire).

Rules that come with homing:

- **Unhomed axes** use `AXIS_n_UNHOMED_VEL_MAX` / `_ACC_MAX` (the original limits). Homed axes
  use `AXIS_n_VEL_MAX` / `_ACC_MAX`.
- **Soft limits** (`AXIS_n_POS_MIN/MAX`) clamp every normal move, as in the original. Once an
  axis is homed they are real joint angles.
- **Setpoint sync.** Homing changes the position frame, and a running ROS2 controller keeps
  sending the pose it held before. After homing (or `sethome`), PC setpoints are ignored until
  they are within `SYNC_TOLERANCE_DEG` (2 deg) of the actual pose on every axis. The status
  shows `faults 0x40 (waiting for PC setpoints to match)`. The next MoveIt/trajectory goal
  starts from the reported pose, so it syncs by itself; so does restarting the controller.
- **Driver sleep** (`IDLE_SLEEP_MODE`): the original slept the drivers whenever the arm was idle.
  That removes holding torque, so a sleeping arm is no longer considered homed. The default
  `IDLE_SLEEP_WHEN_UNHOMED` behaves like the original until something is homed, then keeps the
  drivers awake. `disable` also clears the homed flags.
- Axes without a switch: move them to a known pose and use `sethome <axis> <deg>`
  (or `sethome all` if the arm was powered up in its zero pose, which is what the original
  firmware assumed).

### E-stop

The NC E-stop on J8 sits between GPIO4 (`ENABLE_ALL`) and the ENABLE net that feeds all six
DRV8825 SLP pins and GPIO34 (`HARDWARE_ENABLE`). The v5 schematic and PCB were checked: FLT and
EN are unconnected on every driver, so an open E-stop leaves SLP held low by the chips' internal
pull-downs and the drivers sleep in hardware, whatever the firmware does.

Firmware side:

- While GPIO4 is HIGH (and `ESTOP_SETTLE_MS` after raising it), GPIO34 must read HIGH.
  `ESTOP_DEBOUNCE_MS` of LOW readings **latches** the E-stop: every axis and homing stops,
  GPIO4 goes LOW (so closing the E-stop does not re-energise the motors by itself), setpoints
  are ignored, and the OLED shows `E-STOP: close it, reset`. Over WiFi the response has
  `errorCode = eStop` (4) and `faultFlags` bits `EStopLatched`.
- `reset` raises GPIO4 and reads the net 5 ms later: if the E-stop is closed the drivers stay
  awake holding position; if it is still open GPIO4 drops again and the reset is refused.
- A latched E-stop clears the homed flags (`ESTOP_CLEARS_HOMED`): the drivers were asleep, so
  the arm may have moved. After a reset the setpoint sync rule applies again.
- A PC frame with `emergencyStop != 0` latches the same way (`PC E-STOP` on the OLED). The stock
  ROS2 driver always sends 0.
- With GPIO4 LOW (drivers asleep) the ENABLE net is LOW whatever the E-stop does, so the E-stop
  state is only known while the drivers are enabled. Waking the drivers with the E-stop open
  latches it within ~7 ms, before any step is sent.

### Speed

The original capped every axis at 1000 steps/s (axes 4-6: 400), which is ~37 motor rpm at
1/8 microstepping (~5.9 joint deg/s through the 38.4:1 gearing). Changes:

- **FastAccelStepper** is now the default backend: step pulses are generated by the ESP32's
  MCPWM/PCNT hardware (axes 1-3 on MCPWM0, 4-6 on MCPWM1), and its queue-filling task runs at
  top priority on core 1. Step timing no longer depends on the loop or on WiFi. `Axis.hpp` keeps
  the original interface; to go back to AccelStepper, build the `esp32dev_accelstepper`
  environment (PlatformIO sidebar > esp32dev_accelstepper > Upload), which defines
  `USE_ACCELSTEPPER`.
- Per-axis limits: `AXIS_n_VEL_MAX` / `AXIS_n_ACC_MAX` (homed axes) start at 2x the original;
  `AXIS_n_UNHOMED_VEL_MAX` / `_ACC_MAX` keep the original values.
- Steps per joint degree at 1/8: axes 1-3 = 170.7, axes 4-6 = 40.96. For example 2000 steps/s on
  axis 1 = 11.7 joint deg/s = 75 motor rpm.
- Switching a driver to 1/4 step (DIP: M0 OFF, M1 ON, M2 OFF) halves the step rate needed for
  a given speed; set `AXIS_n_MICROSTEPS 4` to match.
- ROS2/MoveIt has its own joint velocity limits (`joint_limits.yaml` in the MoveIt config) and
  the trajectory speeds come from there. Raise those too, or MoveIt will never ask for more
  speed than before.

**Speed test** (`speedtest <axis>`): the axis moves back and forth over
`AXIS_n_SPEED_TEST_AMPLITUDE_DEG` (45 deg, away from its endstop, inside the soft limits if
homed), starting at `SPEED_TEST_START_STEPS_S` (500) and adding `SPEED_TEST_INCREMENT_STEPS_S`
(500) after every round trip, up to `AXIS_n_SPEED_TEST_MAX`. Each completed round trip prints a
line such as

    A1 6000 steps/s = 35.2 joint deg/s = 225 motor rpm (peak reached 6000)

Watch and listen. When the motor starts to stall, skip or buzz, type `stop`; the last clean
line is the limit. Set `AXIS_n_VEL_MAX` to about 70% of it and tune `ACC_MAX` the same way
(`AXIS_n_SPEED_TEST_ACCEL` is the acceleration used). If a line says `too little travel/accel to
reach it`, the axis never got to that speed in the available travel: raise the amplitude or
the test acceleration. The test stops on an endstop or the E-stop, and afterwards the axis
counts as un-homed (a stall loses steps that cannot be detected without encoders).

## Claw controller

Replaces the original two-servo gripper. Three fingers, each curled by a tendon wound on an N20
gear motor drum (6 V motors run at 5 V); an elastic (TBD) opens the finger when the tendon is
unwound. No encoders: grip and stall detection come from motor current.

### Hardware (checked against `ClawGripperPCB.kicad_sch`)

| Signal | GPIO | Driver pin | Motor |
|---|---|---|---|
| A_IN1 / A_IN2 / A_PWM | 0 / 1 / 4 | U6 AIN1 / AIN2 / PWMA | A (J1) |
| B_IN1 / B_IN2 / B_PWM | 5 / 10 / 23 | U6 BIN1 / BIN2 / PWMB | B (J2) |
| C_IN1 / C_IN2 / C_PWM | 24 / 15 / 2 | U5 AIN1 / AIN2 / PWMA | C (J3) |
| SDA / SCL | 8 / 9 | INA219 U2 (0x40, A), U3 (0x41, B), U4 (0x44, C) | 4.7k pull-ups |
| red / green LED | 26 / 25 | 220R to GND | |

- TB6612 STBY is tied to 3V3; U5 channel B inputs are tied to GND.
- Each 0.1 ohm shunt (R5-R7) sits **in the motor lead**, between the driver output (AO1/BO1) and
  connector pin 1, with INA219 IN+ on the driver side. Current therefore reads positive when
  driving "forward" (IN1 H) and negative in reverse. During the PWM off-time the TB6612 short-
  brakes, so the shunt sees the real averaged motor current. The firmware uses its magnitude.
- PWM is 20 kHz, 10-bit (LEDC).
- Unused: GPIO3, 7, 11 (TX), 12 (RX). USB is native (GPIO13/14), so the build routes `Serial`
  to USB CDC (`ARDUINO_USB_CDC_ON_BOOT=1`).
- The board's VBUS pin is on the same net as J4 +5 V. With USB plugged in and no 5 V on J4, the
  motors run from the PC's USB port. Keep motor tests to J4 power, and check whether the
  Waveshare board has a diode between its USB connector and VBUS before connecting both.

### Current sensing

Each INA219 is set up from `SHUNT_OHMS` (0.1) and `INA_MAX_CURRENT_A` (1.5 A, TBD: the N20 stall
current at 5 V). The firmware picks the smallest shunt range that covers it (1.5 A x 0.1 ohm =
150 mV, so the 160 mV range with 1.6 A full scale) and programs the calibration register so the
current register reads in amps (~0.05 mA resolution). The shunt ADC runs continuously (12 bit,
532 us); each control period (2 ms) reads all three sensors, so every sensor is sampled at
500 Hz (1500 reads/s in total), well above the 200 Hz target.
`CURRENT_FILTER_ALPHA` smooths the magnitude (0.3, a ~6 ms time constant).

A sensor that does not answer at boot, or stops answering, **locks its finger out** (state
`disabled`, red LED blinking slowly) because without it there is no stall protection. It is
retried every second. `ALLOW_MOTOR_WITHOUT_SENSOR` overrides this for bench work.

### Calibration mode

`cal on` streams one CSV line per 20 ms (`CAL_STREAM_HZ`) while you jog the fingers with `move`:

    cal,ms,A_raw_mA,B_raw_mA,C_raw_mA,A_mA,B_mA,C_mA,A_state,B_state,C_state
    cal,51234,-3.2,0.4,1.1,3.0,0.5,1.0,idle,idle,idle

`*_raw_mA` is signed and unfiltered, `*_mA` is the filtered magnitude the grip logic uses.
After every `move` a summary line follows, e.g.
`A close 60% 1500 ms: avg 118 mA after inrush, peak 420 mA`. The first `INRUSH_BLANK_MS`
(150 ms) of each move is excluded from the average because the start-up current is always
high. Both LEDs alternate while calibration mode is on. Copy the CSV into a spreadsheet to plot it.

### Grip logic

Each finger is a state machine driven by time and filtered motor current:

- **close**: drive at `CLOSE_EFFORT_PCT`. After the `INRUSH_BLANK_MS` start-up, once the current
  stays above `FINGER_x_GRIP_THRESHOLD_MA` for `GRIP_CONFIRM_MS` the finger is **gripped**: it
  drops to `HOLD_EFFORT_PCT` (or brakes if that is 0; `HOLD_MAX_MS` switches a long hold to the
  brake). No grip within `CLOSE_TIMEOUT_MS` = fault (timeout).
- **close to force** (`force <mA>`): the same, with the grip threshold = the target current
  (capped at 90% of the hard limit), driven at `FORCE_CLOSE_EFFORT_PCT` so the target is
  reachable. Motor current is roughly proportional to tendon force.
- **open**: reverse at `OPEN_EFFORT_PCT`, ending according to `OPEN_MODE`:
  `OPEN_MODE_TIME` (run `OPEN_TIME_MS`), `OPEN_MODE_CURRENT_BELOW` (current below
  `OPEN_SLACK_BELOW_MA` = tendon slack) or `OPEN_MODE_CURRENT_ABOVE` (current above
  `OPEN_STOP_ABOVE_MA` = hit a stop). The current modes still end at `OPEN_TIME_MS` at the latest.
  Then the motor coasts and the elastic opens the finger.
- **hard limit**: any driven finger (closing, holding, opening, manual) above
  `FINGER_x_HARD_LIMIT_MA` for `HARD_LIMIT_CONFIRM_MS` is braked and reports an overcurrent
  fault. `HARD_LIMIT_CONFIRM_MS` must be longer than `GRIP_CONFIRM_MS` (checked at compile
  time): closing onto something hard can jump past both thresholds at once, and the finger
  should grip and drop to hold power rather than fault.
- A motor is never left at full power in stall: it is either gripped (hold power), faulted
  (braked), or timed out.
- Faults brake the motor. Any new command for that finger clears its fault, so the claw can
  always be opened again; `reset` clears faults without moving.
- Closing on nothing also ends as "gripped" when the finger curls fully and stalls; current
  alone cannot tell those apart. The time to grip (in the log) shows which it was.
- Changing direction always brakes for `DIRECTION_CHANGE_BRAKE_MS` first, and every start ramps
  up over `MOTOR_RAMP_MS`.

| Setting | Default (TBD) | Pick it from calibration (stage 3) |
|---|---|---|
| `FINGER_x_GRIP_THRESHOLD_MA` | 300 | between the free-closing current and the soft-object plateau |
| `FINGER_x_HARD_LIMIT_MA` | 900 | above the hard-object plateau, below the stall current / TB6612 1.2 A |
| `CLOSE_EFFORT_PCT` / `HOLD_EFFORT_PCT` | 60 / 25 | enough to close reliably / to hold without slipping |
| `CLOSE_TIMEOUT_MS` | 3000 | ~1.5x the full-close time |
| `OPEN_MODE`, `OPEN_TIME_MS` | TIME, 1200 | the measured unwind time; current mode if the slack drop is clear |
| `INRUSH_BLANK_MS` | 150 | longer than the start-up spike in the CSV |

### LEDs

| Green | Meaning |
|---|---|
| solid | ready (idle / open) |
| fast blink (5 Hz) | a finger is closing, opening or moving |
| double blink | gripping |
| slow blink (1 Hz) | WiFi not connected (serial still works) |

| Red | Meaning |
|---|---|
| solid | at least one finger faulted (overcurrent / timeout / emergency stop) |
| slow blink | an INA219 is missing: its finger is locked out |

Both alternating: calibration mode.

### Serial console

USB (native, 115200), newline terminated; `finger` = `a`, `b`, `c` or `all` (default all).

| Command | Action |
|---|---|
| `status` | claw state, per-finger state, current, peak, fault |
| `open [finger]` / `close [finger]` | open / close with grip detection |
| `force <mA> [finger]` | close until the current reaches `<mA>` |
| `move <finger> <close\|open> <pct> <ms>` | timed manual move (max 5 s) |
| `stop [finger]` | brake |
| `reset [finger]` | clear faults |
| `cal on` / `cal off` | calibration CSV stream |
| `i2c` | scan the I2C bus |

### Configuration

`claw_controller/include/Configuration.h`: pins, PWM, INA219 setup, grip logic (see the table
above), LED and task settings. Per finger: `FINGER_x_INVERT` (which direction closes, TBD: set so
that `move a close 40 300` curls finger A).

## Protocol

Both boards keep the original diy_robotics framing: TCP, fixed 64-byte request, fixed 64-byte
response, ports 80 (ROS2) and now also 81 (tools, so they can talk while ROS2 holds port 80;
each port serves one client, and a new connection replaces a stale one).

The original fields occupy the first bytes (arm: 0-27, claw: 0-4) and are unchanged. The stock
ROS2 drivers zero-fill the rest of the request, so **an extension is only used when byte
`extMagic` = 0xD5**. Anything else is handled exactly like the original firmware. Responses
always carry the extension; the stock drivers ignore it (the arm driver only reads
`jointPositions`, the gripper service only `gripperState`). Full definitions, with
compile-time checks on the offsets: `shared/include/dume/ArmProtocol.h` and `ClawProtocol.h`.
Both headers use only `<stdint.h>`, so they can be copied into the ROS2 packages later.

Extension commands run **once per new `commandSeq`**; re-sending the same frame polls the
status without repeating the command. The response echoes `lastCommandSeq` and
`lastCommandResult` (1 accepted, 2 rejected, 3 bad argument).

**Arm** request extension: `command` (1 home all, 2 home axis, 3 reset, 4 stop, 5 speed test,
6 disable drivers, 7 set home here, 8 jog, 9 enable drivers), `commandArg` (axis), `commandSeq`,
`extFlags` (bit 0: `jointSetpoints`/`activate` valid, so tools that only send commands leave it
0), `param[3]`. Response extension: `state`, `homedMask`, `endstopMask`, `endstopEnabledMask`,
`faultFlags`, homing axis/phase, `driversEnabled`, `enableNetHigh` (GPIO34), speed-test axis and
speed. `errorCode` now reports 4 E-stop, 5 endstop hit, 6 homing failed, 7 comm timeout.
`emergencyStop` (byte 1, always 0 from ROS2) latches a software E-stop.

**Claw** legacy behaviour: `setGripper` 1 = close (with grip detection), 0 = open, acted on when it
differs from the last open/close command (as the original), and also when a finger is faulted,
so calling the ROS2 service again retries. `gripperState` reports that command. Request
extension: `command` (1 open, 2 close, 3 close to target current, 4 move finger, 5 stop,
6 reset faults, 7 calibration on/off), `commandSeq`, `fingerMask` (bit 0 A, 1 B, 2 C, 0 = all),
`arg`, `effortPct`, `durationMs`, `targetCurrentmA`. Response extension: claw state, per-finger
state, fault, current and peak current (mA), RSSI, flags (calibration, sensor fault).

### dume_cli.py

`shared/tools/dume_cli.py` (Python 3, no extra packages) speaks the extension on port 81:

    python shared/tools/dume_cli.py arm status --loop
    python shared/tools/dume_cli.py arm home 2
    python shared/tools/dume_cli.py arm speedtest 4 --start 500 --max 8000 --inc 500
    python shared/tools/dume_cli.py arm reset
    python shared/tools/dume_cli.py claw close
    python shared/tools/dume_cli.py claw force 400 b
    python shared/tools/dume_cli.py claw move a open 50 800
    python shared/tools/dume_cli.py claw status --loop

`--host` overrides the default addresses (arm 192.168.212.203, claw 192.168.212.202),
`--port 80` talks on the ROS2 port when ROS2 is not running. `arm estop` / `claw estop` send
`emergencyStop`.

## Bench tests

Always: arm unloaded or motors disconnected first, low VMOT first (12 V), E-stop within reach.

### Arm stage 1: endstops

1. **No motor power, no motors.** USB only. Flash, open the monitor. Expect the help text, then
   `[wifi] connected, IP ...`. The OLED shows `OFF  WiFi  PC off` and `End: ......`.
2. `endstops`: all fitted axes show `ok`. Press each switch by hand: that axis shows `HIT` within
   ~5 ms and the OLED shows `X` in its position. Unplug a switch connector: it must show `HIT`
   (broken wire = at limit). Axes you disabled in config show `off`.
3. Hold a switch for a few seconds while WiFi is connected and check that 36/39 never flicker.
4. **VMOT 12 V, motors connected, arm unloaded (or motors off the arm).** `enable`; motors
   should hold. `jog 1 5` then `jog 1 -5`: axis 1 moves and returns. `status` shows the
   positions.
5. Endstop stop: hold axis 1's switch pressed and `jog 1 -30` (towards a HOME_DIR = -1 switch).
   The move must not start. Release, start `jog 1 -30`, press the switch mid-move: the axis stops
   at once and the log prints `A1 endstop triggered`. `jog 1 10` (away) still works while the
   switch is pressed.
6. E-stop (original behaviour at this stage): during a jog, press the E-stop. Motion is
   cancelled and the log prints `ENABLE net LOW`. Release before the next test.
7. After 0.5 s idle the drivers go back to sleep (`status` shows `drivers off`), as in the
   original firmware.
8. ROS2: start the stock driver. The arm follows as before and the OLED shows `PC on`. Stop the
   ROS2 driver mid-motion: within 250 ms the log prints `no setpoints ... axes stopped`.

### Arm stage 2: homing

Start with one axis, arm unloaded, 12 V, hand on the E-stop.

1. Check `AXIS_n_HOME_DIR` first: `jog <n> -5` must move the axis **towards** its switch for
   HOME_DIR = -1. If it moves away, either the switch is at the other end (HOME_DIR = +1) or
   `AXIS_n_INVERT_DIRECTION` is wrong for your wiring. Fix that before homing.
2. `home <n>`. Watch the OLED line `Homing A<n>: fast`, then `back off`, `slow`, `back off`.
   Press the switch by hand during the fast approach if the axis is far from it: the axis must
   stop, back off, and re-approach slowly. The log ends with `A<n> homed` and `homing complete`,
   and `status` shows `homed:yes` and the position = HOME_SWITCH_DEG + back-off.
3. Failure paths:
   - unplug the switch connector (reads HIGH), `home <n>`: expect `switch still pressed after
     backing off` and `HOMING FAILED A<n>` on the OLED, with no approach move;
   - temporarily set `AXIS_n_HOME_MAX_TRAVEL_DEG` to 10 with the axis far from the switch:
     expect `switch not found within 10 deg`;
   - `stop` during homing aborts it.
4. Repeatability: `home <n>` several times, and note the position printed after each (it should
   be the same to within a few hundredths of a degree). Then `jog <n> 20`, `home <n>` again.
5. Soft limits: after homing, `jog <n> 999` stops at `AXIS_n_POS_MAX`.
6. `home` (all): axes run in `HOMING_ORDER`, skipping those without switches.
7. ROS2 sync: with the stock driver running, `home` from the serial console. After homing the
   arm must **not** jump back to the old ROS pose; `status` shows the sync-wait flag. Send a new
   goal from MoveIt: the arm follows again and the log prints `PC setpoints in sync`.

### Arm stage 3: E-stop

1. **No motors.** `enable`, then press the E-stop: the log prints `E-STOP opened`, OLED
   `E-STOP: close it, reset`, `status` shows state `E-STOP`, drivers off. Measure GPIO4 (J8
   pin 1): 0 V.
2. Release the E-stop: nothing changes, the drivers stay off (GPIO4 still 0 V).
3. `reset` with the E-stop still pressed: `reset refused`. Release it, `reset`:
   `E-stop reset - drivers enabled`, J8 pin 1 = 3.3 V.
4. While latched, `jog 1 5`, `home` and `enable` are all rejected.
5. **Motors connected, 12 V.** Jog an axis a long way (`jog 1 90`) and press the E-stop mid-move:
   the motor stops at once (hardware) and the firmware latches. `status` shows every axis
   `homed:no`.
6. Press the E-stop with the drivers asleep (`disable`), then `jog 1 5`: the drivers wake, the
   latch trips within a few ms and the axis does not move.
7. ROS2: while streaming, press the E-stop, release it, `reset`. The arm must not move until a
   new MoveIt goal (setpoint sync).
8. Only then raise VMOT to 36 V and repeat 5.

### Arm stage 4: speed

1. Flash the default (FastAccelStepper) build. Repeat stage 1 step 4 and stage 2 step 2: jog
   and homing must behave exactly as before (same direction, same distances). If an axis runs
   the wrong way, compare with the `esp32dev_accelstepper` build before changing
   `INVERT_DIRECTION`.
2. **One motor on the bench, not in the arm, 12 V.** `speedtest 4 500 6000 500`. Let it ramp and
   note where it stalls. Repeat at 24 V and at 36 V: the stall speed should rise with voltage.
   Check the DRV8825 current limit (Vref) before going to 36 V.
3. In the arm, unloaded, homed: `speedtest <n>` per axis, hand on `stop`/E-stop. Test the
   E-stop mid-test: the latch must trip and the log prints `speed test ... ended`.
4. Stepping under WiFi load: run a speed test at a clean speed while ROS2 (or
   `dume_cli.py status --loop`) hammers the network. There must be no audible change.
5. Set `AXIS_n_VEL_MAX` / `ACC_MAX` to ~70% of the found limits, rebuild, home, and check that
   `jog <n> 90` runs smoothly at the new speed.

### Claw stage 1: motor drivers and LEDs

1. **USB only, J4 unpowered, motors unplugged.** Flash (`claw_controller`), open the monitor.
   Expect `DUM-E claw starting` and the help text. The green LED is on.
2. `move a close 50 1000`: green blinks fast for 1 s. Probe J1 with a multimeter (DC): about
   2.5 V average. `move a open 50 1000`: same magnitude, opposite sign. Repeat for b, c.
3. **J4 at 5 V, one motor on J1, tendon not attached.** `move a close 30 500`: the motor turns;
   note the direction. `move a open 30 500`: it reverses. Set `FINGER_A_INVERT` so that "close"
   winds the tendon. Repeat for B and C.
4. `move a close 80 3000` then `stop a` after one second: the motor stops at once (brake).
5. `move a close 60 2000` followed immediately by `move a open 60 2000`: there is a short brake
   (30 ms) before reversing, never a hard reversal.

### Claw stage 2: INA219 readings

1. **USB + J4 5 V, motors unplugged.** The log shows three lines `INA219 A (0x40) ok: range 1600
   mA`, B (0x41), C (0x44). `i2c` lists 0x40 0x41 0x44. `status` shows all three at ~0 mA
   (a few mA of offset is normal).
2. Lock-out test: temporarily set `INA_ADDR_C 0x45` (no such device), flash, and check that
   the log says `INA219 C (0x45) NOT FOUND - finger C locked out`, the red LED blinks slowly, and
   `move c close 50 500` does nothing. Restore the address.
3. **Motor A plugged in, no tendon.** `move a close 50 2000` and run `status` during the move:
   free-running current (typically 50-150 mA for an N20). Pinch the motor shaft/drum gently
   with a cloth and `status` again: current rises. Note the numbers; stage 3 makes this easier.
4. Repeat for B and C and check each sensor reads its own motor (0x40 = A, 0x41 = B, 0x44 = C).

### Claw stage 3: calibration mode (picking the thresholds)

Do this on the finished claw with tendons and elastics fitted, J4 at 5 V.

1. `cal on`. The LEDs alternate and CSV lines scroll.
2. **Free close:** open finger A by hand, then `move a close 50 800`, shorter or longer until
   the finger just reaches fully curled. Note the summary's average (free-running closing
   current) and how long a full close takes.
3. **Closed on nothing:** `move a close 50 2000` so the finger curls fully and the motor keeps
   pulling. The CSV shows the current climbing when the finger bottoms out; note the plateau.
4. **Gripping objects:** put a soft object (sponge) and a hard one (wooden block) in the claw,
   `move all close 50 2000`, note the plateau per finger.
5. **Open:** `move a open 50 <ms>` starting from closed. Find the time that fully unwinds the
   tendon without winding it back the other way. Watch the current: it usually drops once the
   tendon goes slack (then only the motor's no-load current flows). That drop is what
   `OPEN_MODE_CURRENT_BELOW` detects.
6. Repeat at the effort (PWM %) you intend to use. `cal off`.
7. Record: free close current, grip plateau (soft, hard, nothing), full-close time, open time,
   slack current. These go into the stage 4 settings (see the table under Grip logic).

### Claw stage 4: grip logic

Enter the stage 3 numbers in `Configuration.h` first. J4 at 5 V, claw on the bench.

1. `close` with nothing in the claw: all fingers curl; the log prints `A gripped at ... mA after
   ... ms` for each, green double-blinks. `status` shows `GRIPPED` and currents at hold level.
2. `open`: fingers unwind and open; state `open`, green solid. If a finger stops short or starts
   closing again, adjust `OPEN_TIME_MS` (or switch `OPEN_MODE`).
3. Soft object (sponge): `close`. Each finger must stop on the object and hold without crushing
   it. Repeat with a hard object: grip, no overcurrent fault.
4. `force 200` vs `force 500` on the sponge: visibly different squeeze.
5. Timeout: set `CLOSE_TIMEOUT_MS` to 300 temporarily, `close`: every finger faults with
   `close TIMEOUT`, red LED solid, motors braked. `open` clears the fault and opens.
6. Overcurrent: `move a close 100 3000` with finger A blocked by hand (be gentle): within ~0.2 s
   of the stall `A OVERCURRENT` and the motor stops. Motor and TB6612 must not get hot.
7. Hold: `close` on an object and leave it for a minute; feel the motors (warm is ok, hot is not;
   lower `HOLD_EFFORT_PCT` or set `HOLD_MAX_MS`).

### Claw stage 5: WiFi protocol

1. Set `secrets.h` and the IP block, flash. The log shows `[wifi] connected`; green goes from
   slow blink (no WiFi) to solid.
2. From the PC: `python shared/tools/dume_cli.py claw status`, then `claw close`, `claw open`,
   `claw force 300 a`, `claw move b close 40 500`, `claw stop`. Each prints `accepted` and the
   new state.
3. Stock ROS2 gripper service (port 80): `ros2 service call /gripper_control
   std_srvs/srv/SetBool "{data: true}"` closes with grip detection; `false` opens. The service
   log shows `Gripper State = Closed/Open` as before.
4. While ROS2 is connected, `dume_cli.py claw status --loop` on port 81 must keep working.
5. Kill the ROS2 service without closing the socket (unplug the PC's network), reconnect: the
   claw accepts the new connection without a reboot.
6. `dume_cli.py claw estop`: every finger brakes with `fault: emergency stop`, error code 6;
   `claw open` clears it.
7. Arm: `dume_cli.py arm status --loop` while the stock ROS2 arm driver runs on port 80; then
   `dume_cli.py arm estop` latches the arm E-stop and `dume_cli.py arm reset` clears it.
