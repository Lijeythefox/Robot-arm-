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
// the v5 board. Speed limits apply once an axis is homed; the UNHOMED limits (the original
// values) apply before that.
// ============================================================================================

// Axis 1 uses a DRV8825 Driver, NEMA 17
#define STEPPER_1_STEP 16
#define STEPPER_1_DIR 13
#define AXIS_1_GEARING 38.4   // i = n_Input/n_Output = 38.4/1 = 38.4
#define AXIS_1_STEPS_PER_REV 200
#define AXIS_1_MICROSTEPS 8   // must match DIP switch SW1: M0 ON, M1 ON, M2 OFF = 1/8
#define AXIS_1_POS_MIN -135
#define AXIS_1_POS_MAX 135
#define AXIS_1_VEL_MAX 1000
#define AXIS_1_ACC_MAX 1000
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
#define AXIS_2_VEL_MAX 1000
#define AXIS_2_ACC_MAX 1000
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
#define AXIS_3_VEL_MAX 1000
#define AXIS_3_ACC_MAX 1000
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
#define AXIS_4_VEL_MAX 400
#define AXIS_4_ACC_MAX 800
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
#define AXIS_5_VEL_MAX 400
#define AXIS_5_ACC_MAX 800
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
#define AXIS_6_VEL_MAX 400
#define AXIS_6_ACC_MAX 800
#define AXIS_6_UNHOMED_VEL_MAX 400
#define AXIS_6_UNHOMED_ACC_MAX 800
#define AXIS_6_INVERT_DIRECTION true

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

// Like the original: put the drivers to sleep when nothing has moved for IDLE_SLEEP_DELAY_MS,
// so idle motors stay cool. Sleeping drivers give no holding torque.
#define IDLE_SLEEP_DRIVERS true
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
