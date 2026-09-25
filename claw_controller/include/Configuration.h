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
