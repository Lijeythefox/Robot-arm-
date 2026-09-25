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
- The WiFi code no longer blocks, serves a second port (81) for tools, and accepts a
  reconnecting ROS2 driver without a reboot.
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
| `stop` | decelerate all axes, abort homing |
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
