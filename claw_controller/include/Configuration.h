// DUM-E 3-finger tendon claw (ClawGripperPCB): every pin, address and tunable lives here.
// Values marked TBD are safe placeholders until they have been measured on the real claw.
#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

// ============================================================================================
// Network. Credentials are in include/secrets.h (copy secrets.example.h).
// ============================================================================================
#define CLAW_HOSTNAME "dume-claw"
#define CLAW_USE_STATIC_IP true
#define IP_ADDRESS 192, 168, 212, 202  // same address as the original gripper
#define GATEWAY_IP 192, 168, 212, 1
#define SUBNET_MASK 255, 255, 255, 0

// ============================================================================================
// Motor drivers: 2x TB6612FNG, STBY tied to 3V3. U6 channel A = motor A (J1), U6 channel B =
// motor B (J2), U5 channel A = motor C (J3); U5 channel B unused (inputs tied to GND).
// Board labels "GP<n>" = GPIO<n>.
// ============================================================================================
#define A_IN1 0
#define A_IN2 1
#define A_PWM 4
#define B_IN1 5
#define B_IN2 10
#define B_PWM 23
#define C_IN1 24
#define C_IN2 15
#define C_PWM 2   // MTMS strapping pin; only matters for SDIO boot, safe here

// ~20 kHz is above hearing. 10 bits gives 0.1% duty steps.
#define MOTOR_PWM_FREQ_HZ 20000
#define MOTOR_PWM_BITS 10

// Which way closes each finger is TBD. "Forward" = IN1 HIGH, IN2 LOW, which drives current
// from AO1/BO1 (connector pin 1) into the motor. Set true to close with the other direction.
#define FINGER_A_INVERT false
#define FINGER_B_INVERT false
#define FINGER_C_INVERT false

// Soft start: ramp the PWM duty up over this time to limit the inrush current.
#define MOTOR_RAMP_MS 40
// Brake between direction changes so the H-bridge never reverses a spinning motor.
#define DIRECTION_CHANGE_BRAKE_MS 30
// Longest manual move allowed (serial "move", network MoveFinger).
#define MAX_MANUAL_MOVE_MS 5000

// ============================================================================================
// Current sensing: 3x INA219 on I2C, 0.1 ohm shunt in each motor lead.
// ============================================================================================
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ_HZ 400000
#define INA_ADDR_A 0x40  // U2, motor A
#define INA_ADDR_B 0x41  // U3, motor B
#define INA_ADDR_C 0x44  // U4, motor C
#define SHUNT_OHMS 0.1
// Highest current that must be measurable (N20 stall current at 5 V, TBD). Picks the INA219
// range: 1.5 A -> 160 mV range (1.6 A full scale, ~0.05 mA resolution). Max 3.2 A.
#define INA_MAX_CURRENT_A 1.5
// Shunt ADC setting (INA219 SADC bits): 0x3 = 12 bit / 532 us, 0x9 = 2-sample avg / 1.06 ms,
// 0xA = 4 samples / 2.13 ms. One fresh conversion per control period at 0x3.
#define INA219_SHUNT_ADC 0x3
// Exponential filter on |current| per control period: 1 = raw, smaller = smoother.
// 0.3 at 500 Hz is a ~6 ms time constant.
#define CURRENT_FILTER_ALPHA 0.3
// Without its current sensor a finger has no stall protection, so it is locked out.
#define ALLOW_MOTOR_WITHOUT_SENSOR false
// A missing / failed sensor is re-initialised this often.
#define SENSOR_RETRY_MS 1000

// Motor start-up (inrush) current is ignored for this long after a finger starts moving.
#define INRUSH_BLANK_MS 150

// ============================================================================================
// Grip logic (all TBD: pick the numbers with calibration mode, see README).
// Currents are the filtered magnitude of the motor current.
// ============================================================================================
#define CLOSE_EFFORT_PCT 60           // PWM while closing
#define FORCE_CLOSE_EFFORT_PCT 90     // PWM for "close to target current", so the target is reachable
#define OPEN_EFFORT_PCT 60            // PWM while opening

// Closing -> Gripped when the current stays above the grip threshold for GRIP_CONFIRM_MS.
#define FINGER_A_GRIP_THRESHOLD_MA 300
#define FINGER_B_GRIP_THRESHOLD_MA 300
#define FINGER_C_GRIP_THRESHOLD_MA 300
#define GRIP_CONFIRM_MS 20
// Give up (fault) if the grip threshold is not reached within this time.
#define CLOSE_TIMEOUT_MS 3000

// Once gripped: keep pulling at HOLD_EFFORT_PCT (0 = hold with the brake instead). If
// HOLD_MAX_MS > 0, switch from PWM to brake after that long to spare the motor.
#define HOLD_EFFORT_PCT 25
#define HOLD_MAX_MS 0

// Hard limit: any driven finger above this for HARD_LIMIT_CONFIRM_MS is stopped with a fault.
// Checked after the inrush time. Must be above the grip threshold, and HARD_LIMIT_CONFIRM_MS
// longer than GRIP_CONFIRM_MS: closing onto a hard object can jump straight past both, and the
// finger should then drop to hold power (gripped), not fault.
#define FINGER_A_HARD_LIMIT_MA 900
#define FINGER_B_HARD_LIMIT_MA 900
#define FINGER_C_HARD_LIMIT_MA 900
#define HARD_LIMIT_CONFIRM_MS 40

// How opening ends (the elastic opens the finger once the tendon is unwound):
//   OPEN_MODE_TIME           run for OPEN_TIME_MS
//   OPEN_MODE_CURRENT_BELOW  stop when the current drops below OPEN_SLACK_BELOW_MA (tendon slack,
//                            motor free-running) for OPEN_CONFIRM_MS, or after OPEN_TIME_MS
//   OPEN_MODE_CURRENT_ABOVE  stop when the current rises above OPEN_STOP_ABOVE_MA (e.g. a hard
//                            stop at the open end) for OPEN_CONFIRM_MS, or after OPEN_TIME_MS
// Do not let the drum keep unwinding: past slack it winds the tendon the other way and closes
// the finger again, so keep OPEN_TIME_MS close to the measured unwind time.
#define OPEN_MODE_TIME 0
#define OPEN_MODE_CURRENT_BELOW 1
#define OPEN_MODE_CURRENT_ABOVE 2
#define OPEN_MODE OPEN_MODE_TIME
#define OPEN_TIME_MS 1200
#define OPEN_SLACK_BELOW_MA 60
#define OPEN_STOP_ABOVE_MA 400
#define OPEN_CONFIRM_MS 50

// ============================================================================================
// Calibration mode (serial "cal on"): streams CSV lines with the live current of every finger
// at CAL_STREAM_HZ, and prints a summary (average after inrush, peak) after every manual move.
// ============================================================================================
#define CAL_STREAM_HZ 50

// ============================================================================================
// Status LEDs (active high through 220R). GPIO25/26 are strapping pins; fine as outputs.
// ============================================================================================
#define LED_RED 26
#define LED_GREEN 25

// ============================================================================================
// Tasks. The ESP32-C5 has one core: motor control and current sampling run in a task above
// the lwIP task (18) and below the WiFi driver (23), so network traffic cannot delay them.
// ============================================================================================
#define CONTROL_PERIOD_MS 2         // 500 Hz control loop
#define CONTROL_TASK_PRIORITY 20
#define NETWORK_TASK_PRIORITY 5

#endif
