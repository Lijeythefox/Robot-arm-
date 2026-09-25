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
| `stop` | decelerate all axes |

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
