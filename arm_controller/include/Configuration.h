// DUM-E arm controller v5: every pin, address and tunable lives here.
//
// Units, unless a name says otherwise:
//   positions  - joint degrees, same sign convention as ROS (after AXIS_n_INVERT_DIRECTION)
//   speeds     - motor microsteps per second (the original firmware's unit)
//   accel      - microsteps per second^2
// Values marked TBD are safe placeholders until they have been measured on the real arm.
#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

// ============================================================================================
// Network. Credentials are in include/secrets.h (copy secrets.example.h).
// ============================================================================================
#define ARM_HOSTNAME "dume-arm"
#define ARM_USE_STATIC_IP true
#define IP_ADDRESS 192, 168, 212, 203  // Gripper 202, Arm 203 (as in the original)
// The original used gateway 10.42.0.1, which is not inside 192.168.212.0/24. Set this to your
// router / hotspot address.
#define GATEWAY_IP 192, 168, 212, 1
#define SUBNET_MASK 255, 255, 255, 0

// If a client has been streaming setpoints and then goes quiet for this long, every axis is
// decelerated to a stop and held. ROS2 streams at 50 Hz (20 ms). 0 disables the check.
#define COMM_TIMEOUT_MS 250

// ============================================================================================
// Axes. STEP/DIR pins, gearing and directions are unchanged from the original firmware and
// the v5 board. VEL_MAX / ACC_MAX apply once an axis is homed; the UNHOMED limits (the
// original firmware's values) apply before that.
//
// The homed limits start at 2x the original. At 36 V the motors can go much faster: find each
// axis' real limit with the speed test (see README), then set VEL_MAX to ~70% of the speed at
// which it stalled.
//   joint deg/s = steps/s / (STEPS_PER_REV * MICROSTEPS * GEARING / 360)
//   axes 1-3: 170.7 steps per joint degree at 1/8, axes 4-6: 40.96
// ============================================================================================

// Axis 1 uses a DRV8825 Driver, NEMA 17
#define STEPPER_1_STEP 16
#define STEPPER_1_DIR 13
#define AXIS_1_GEARING 38.4   // i = n_Input/n_Output = 38.4/1 = 38.4
#define AXIS_1_STEPS_PER_REV 200
#define AXIS_1_MICROSTEPS 8   // must match DIP switch SW1: M0 ON, M1 ON, M2 OFF = 1/8
#define AXIS_1_POS_MIN -135
#define AXIS_1_POS_MAX 135
#define AXIS_1_VEL_MAX 2000
#define AXIS_1_ACC_MAX 2000
#define AXIS_1_UNHOMED_VEL_MAX 1000
#define AXIS_1_UNHOMED_ACC_MAX 1000
#define AXIS_1_INVERT_DIRECTION false     // to match axis rotation direction with simulation rotation direction

// Axis 2 uses a DRV8825 Driver, NEMA 17
#define STEPPER_2_STEP 18
#define STEPPER_2_DIR 17
#define AXIS_2_GEARING 38.4
#define AXIS_2_STEPS_PER_REV 200
#define AXIS_2_MICROSTEPS 8
#define AXIS_2_POS_MIN -120
#define AXIS_2_POS_MAX 120
#define AXIS_2_VEL_MAX 2000
#define AXIS_2_ACC_MAX 2000
#define AXIS_2_UNHOMED_VEL_MAX 1000
#define AXIS_2_UNHOMED_ACC_MAX 1000
#define AXIS_2_INVERT_DIRECTION false

// Axis 3 uses a DRV8825 Driver, NEMA 17
#define STEPPER_3_STEP 5
#define STEPPER_3_DIR 19
#define AXIS_3_GEARING 38.4
#define AXIS_3_STEPS_PER_REV 200
#define AXIS_3_MICROSTEPS 8
#define AXIS_3_POS_MIN -150
#define AXIS_3_POS_MAX 150
#define AXIS_3_VEL_MAX 2000
#define AXIS_3_ACC_MAX 2000
#define AXIS_3_UNHOMED_VEL_MAX 1000
#define AXIS_3_UNHOMED_ACC_MAX 1000
#define AXIS_3_INVERT_DIRECTION true

// Axis 4 uses a DRV8825 Driver, MINIBEA
#define STEPPER_4_STEP 25
#define STEPPER_4_DIR 23
#define AXIS_4_GEARING 38.4
#define AXIS_4_STEPS_PER_REV 48
#define AXIS_4_MICROSTEPS 8
#define AXIS_4_POS_MIN -120
#define AXIS_4_POS_MAX 120
#define AXIS_4_VEL_MAX 800
#define AXIS_4_ACC_MAX 1600
#define AXIS_4_UNHOMED_VEL_MAX 400
#define AXIS_4_UNHOMED_ACC_MAX 800
#define AXIS_4_INVERT_DIRECTION true

// Axis 5 uses a DRV8825 Driver, MINIBEA
#define STEPPER_5_STEP 27
#define STEPPER_5_DIR 26
#define AXIS_5_GEARING 38.4
#define AXIS_5_STEPS_PER_REV 48
#define AXIS_5_MICROSTEPS 8
#define AXIS_5_POS_MIN -120
#define AXIS_5_POS_MAX 120
#define AXIS_5_VEL_MAX 800
#define AXIS_5_ACC_MAX 1600
#define AXIS_5_UNHOMED_VEL_MAX 400
#define AXIS_5_UNHOMED_ACC_MAX 800
#define AXIS_5_INVERT_DIRECTION true

// Axis 6 uses a DRV8825 Driver, MINIBEA
#define STEPPER_6_STEP 33
#define STEPPER_6_DIR 32
#define AXIS_6_GEARING 38.4
#define AXIS_6_STEPS_PER_REV 48
#define AXIS_6_MICROSTEPS 8
#define AXIS_6_POS_MIN -175
#define AXIS_6_POS_MAX 175
#define AXIS_6_VEL_MAX 800
#define AXIS_6_ACC_MAX 1600
#define AXIS_6_UNHOMED_VEL_MAX 400
#define AXIS_6_UNHOMED_ACC_MAX 800
#define AXIS_6_INVERT_DIRECTION true

// ============================================================================================
// Speed test (serial "speedtest <axis>"): the axis moves back and forth over AMPLITUDE_DEG,
// starting at SPEED_TEST_START_STEPS_S and adding SPEED_TEST_INCREMENT_STEPS_S after every
// round trip, until SPEED_TEST_MAX, "stop", an endstop or the E-stop. The last speed printed
// before the motor stalls is the limit. Stalls cannot be detected (no encoders), so the axis
// is marked un-homed afterwards.
// ============================================================================================
#define SPEED_TEST_START_STEPS_S 500
#define SPEED_TEST_INCREMENT_STEPS_S 500
#define SPEED_TEST_ABS_MAX_STEPS_S 100000  // hard cap; DRV8825 max is 250 kHz
#define AXIS_1_SPEED_TEST_MAX 32000        // 1200 motor rpm at 1/8
#define AXIS_1_SPEED_TEST_ACCEL 20000
#define AXIS_1_SPEED_TEST_AMPLITUDE_DEG 45
#define AXIS_2_SPEED_TEST_MAX 32000
#define AXIS_2_SPEED_TEST_ACCEL 20000
#define AXIS_2_SPEED_TEST_AMPLITUDE_DEG 45
#define AXIS_3_SPEED_TEST_MAX 32000
#define AXIS_3_SPEED_TEST_ACCEL 20000
#define AXIS_3_SPEED_TEST_AMPLITUDE_DEG 45
#define AXIS_4_SPEED_TEST_MAX 16000        // 2500 motor rpm at 1/8 (48-step motor)
#define AXIS_4_SPEED_TEST_ACCEL 10000
#define AXIS_4_SPEED_TEST_AMPLITUDE_DEG 45
#define AXIS_5_SPEED_TEST_MAX 16000
#define AXIS_5_SPEED_TEST_ACCEL 10000
#define AXIS_5_SPEED_TEST_AMPLITUDE_DEG 45
#define AXIS_6_SPEED_TEST_MAX 16000
#define AXIS_6_SPEED_TEST_ACCEL 10000
#define AXIS_6_SPEED_TEST_AMPLITUDE_DEG 45

// ============================================================================================
// Driver enable / E-stop.
// GPIO4 drives the NC E-stop on J8; the other side of the E-stop is the ENABLE net, which goes
// to every DRV8825 SLP pin and back to GPIO34. With GPIO4 LOW the net is LOW whatever the
// E-stop does, so the E-stop can only be read while the drivers are enabled.
// ============================================================================================
#define ENABLE_ALL 4        // output: HIGH wakes the drivers (through the E-stop)
#define HARDWARE_ENABLE 34  // input only: reads the ENABLE net

// DRV8825 needs up to 1.7 ms after SLP goes high before it accepts STEP pulses.
#define DRIVER_WAKE_MS 2

// E-stop: once the drivers are enabled, the ENABLE net must read HIGH. After ESTOP_SETTLE_MS
// from enabling, ESTOP_DEBOUNCE_MS of LOW readings latch the E-stop: all motion is aborted,
// GPIO4 goes LOW (so closing the E-stop does not re-energise the motors by itself) and nothing
// moves until a "reset" command, which only succeeds once the E-stop is closed again.
#define ESTOP_SETTLE_MS 5
#define ESTOP_DEBOUNCE_MS 2
// The drivers were asleep, so the arm may have moved under gravity: forget the homing.
#define ESTOP_CLEARS_HOMED true

// When to put the drivers to sleep after IDLE_SLEEP_DELAY_MS without motion.
// Sleeping drivers keep motors cool (the original firmware always did this) but give no holding
// torque: a loaded joint can sag and its counted position becomes wrong, so sleeping clears
// the homed flags.
//   IDLE_SLEEP_NEVER         - hold position forever
//   IDLE_SLEEP_WHEN_UNHOMED  - behave like the original until the arm is homed, then hold
//   IDLE_SLEEP_ALWAYS        - always sleep when idle (homing is lost every time)
#define IDLE_SLEEP_NEVER 0
#define IDLE_SLEEP_WHEN_UNHOMED 1
#define IDLE_SLEEP_ALWAYS 2
#define IDLE_SLEEP_MODE IDLE_SLEEP_WHEN_UNHOMED
#define IDLE_SLEEP_DELAY_MS 500

// ============================================================================================
// Endstops J10..J15: pin 1 = 3V3, 2 = SIG, 3 = GND. Normally-closed switch between SIG and GND:
// LOW = OK, HIGH = at the limit OR broken wire. Both are treated as "at limit".
// Set AXIS_n_ENDSTOP_ENABLED false for axes that have no switch fitted yet.
// AXIS_n_HOME_DIR: which end of the axis the switch sits at, -1 = negative (POS_MIN) end,
// +1 = positive (POS_MAX) end, in joint degrees.
// ============================================================================================
#define AXIS_1_ENDSTOP_ENABLED true
#define AXIS_1_ENDSTOP_PIN 15         // 10k pull-up + 1k series on board. Strapping pin: a
#define AXIS_1_ENDSTOP_PULLUP false   //   closed switch at boot only silences the boot log.
#define AXIS_1_HOME_DIR -1

#define AXIS_2_ENDSTOP_ENABLED true
#define AXIS_2_ENDSTOP_PIN 2          // no external pull-up (it would block USB flashing),
#define AXIS_2_ENDSTOP_PULLUP true    //   so the internal pull-up is required
#define AXIS_2_HOME_DIR -1

#define AXIS_3_ENDSTOP_ENABLED true
#define AXIS_3_ENDSTOP_PIN 14         // 10k pull-up + 1k series (GPIO14 drives briefly at boot)
#define AXIS_3_ENDSTOP_PULLUP false
#define AXIS_3_HOME_DIR -1

#define AXIS_4_ENDSTOP_ENABLED true
#define AXIS_4_ENDSTOP_PIN 36         // SENSOR_VP, input only, 10k pull-up on board
#define AXIS_4_ENDSTOP_PULLUP false
#define AXIS_4_HOME_DIR -1

#define AXIS_5_ENDSTOP_ENABLED true
#define AXIS_5_ENDSTOP_PIN 39         // SENSOR_VN, input only, 10k pull-up on board
#define AXIS_5_ENDSTOP_PULLUP false
#define AXIS_5_HOME_DIR -1

#define AXIS_6_ENDSTOP_ENABLED true
#define AXIS_6_ENDSTOP_PIN 35         // input only, 10k pull-up on board
#define AXIS_6_ENDSTOP_PULLUP false
#define AXIS_6_HOME_DIR -1

// Homing, per axis (all TBD: measure on the real arm, defaults are deliberately slow).
//   HOME_SWITCH_DEG    joint angle at which the switch triggers; becomes the axis position
//   HOME_FAST_DEG_S    first approach speed          HOME_SLOW_DEG_S  re-approach speed
//   HOME_BACKOFF_DEG   distance to back off between approaches and after homing
//   HOME_MAX_TRAVEL_DEG  fail if the switch is not found within this distance
//   HOME_PARK_DEG      where to go after homing when HOMING_PARK_AFTER is true
#define AXIS_1_HOME_SWITCH_DEG AXIS_1_POS_MIN
#define AXIS_1_HOME_FAST_DEG_S 5.0
#define AXIS_1_HOME_SLOW_DEG_S 1.0
#define AXIS_1_HOME_BACKOFF_DEG 5.0
#define AXIS_1_HOME_MAX_TRAVEL_DEG (AXIS_1_POS_MAX - AXIS_1_POS_MIN + 20)
#define AXIS_1_HOME_PARK_DEG 0.0

#define AXIS_2_HOME_SWITCH_DEG AXIS_2_POS_MIN
#define AXIS_2_HOME_FAST_DEG_S 5.0
#define AXIS_2_HOME_SLOW_DEG_S 1.0
#define AXIS_2_HOME_BACKOFF_DEG 5.0
#define AXIS_2_HOME_MAX_TRAVEL_DEG (AXIS_2_POS_MAX - AXIS_2_POS_MIN + 20)
#define AXIS_2_HOME_PARK_DEG 0.0

#define AXIS_3_HOME_SWITCH_DEG AXIS_3_POS_MIN
#define AXIS_3_HOME_FAST_DEG_S 5.0
#define AXIS_3_HOME_SLOW_DEG_S 1.0
#define AXIS_3_HOME_BACKOFF_DEG 5.0
#define AXIS_3_HOME_MAX_TRAVEL_DEG (AXIS_3_POS_MAX - AXIS_3_POS_MIN + 20)
#define AXIS_3_HOME_PARK_DEG 0.0

#define AXIS_4_HOME_SWITCH_DEG AXIS_4_POS_MIN
#define AXIS_4_HOME_FAST_DEG_S 8.0
#define AXIS_4_HOME_SLOW_DEG_S 2.0
#define AXIS_4_HOME_BACKOFF_DEG 5.0
#define AXIS_4_HOME_MAX_TRAVEL_DEG (AXIS_4_POS_MAX - AXIS_4_POS_MIN + 20)
#define AXIS_4_HOME_PARK_DEG 0.0

#define AXIS_5_HOME_SWITCH_DEG AXIS_5_POS_MIN
#define AXIS_5_HOME_FAST_DEG_S 8.0
#define AXIS_5_HOME_SLOW_DEG_S 2.0
#define AXIS_5_HOME_BACKOFF_DEG 5.0
#define AXIS_5_HOME_MAX_TRAVEL_DEG (AXIS_5_POS_MAX - AXIS_5_POS_MIN + 20)
#define AXIS_5_HOME_PARK_DEG 0.0

#define AXIS_6_HOME_SWITCH_DEG AXIS_6_POS_MIN
#define AXIS_6_HOME_FAST_DEG_S 8.0
#define AXIS_6_HOME_SLOW_DEG_S 2.0
#define AXIS_6_HOME_BACKOFF_DEG 5.0
#define AXIS_6_HOME_MAX_TRAVEL_DEG (AXIS_6_POS_MAX - AXIS_6_POS_MIN + 20)
#define AXIS_6_HOME_PARK_DEG 0.0

// "home all" order: big joints first. Axes without an enabled endstop are skipped.
#define HOMING_ORDER {2, 3, 5, 1, 4, 6}
// After each axis is homed (and backed off its switch), move it to HOME_PARK_DEG.
#define HOMING_PARK_AFTER false

// After homing (or anything else that changes the position frame) the PC's setpoints are
// ignored until they come within this distance of the actual position on every axis. Stops a
// ROS2 controller that is still holding a pre-homing pose from yanking the arm there.
#define SYNC_TOLERANCE_DEG 2.0

// The board's RC filter is ~1 ms. A level must be stable for this long before it counts.
// GPIO36/39 can glitch while WiFi/ADC are active; this also rejects those.
#define ENDSTOP_DEBOUNCE_MS 5

// ============================================================================================
// OLED (SSD1306 128x64, I2C)
// ============================================================================================
#define OLED_ADDRESS 0x3C
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_FLIP_VERTICAL false
#define OLED_REFRESH_MS 250

// ============================================================================================
// Tasks. Stepping stays on core 1 (Arduino loop); networking, display and console on core 0.
// ============================================================================================
#define CONTROL_TICK_MS 1           // endstop sampling / safety checks / state machine rate
#define NETWORK_TASK_PRIORITY 17    // as in the original
#define NETWORK_TASK_CORE 0

#endif
