// Wire format between the PC (ROS2 gripper service or dume_cli.py) and the DUM-E claw.
//
// Fixed 64-byte frames over TCP, one request -> one response, like the arm.
// Bytes 0..4 are the original diy_robotics gripper layout
// (diy_soft_gripper_driver/DataFormat.hpp) and must never change. The stock driver keeps its
// request in a zero-initialised global, so everything after byte 4 is zero from it:
// extMagic == 0 means "legacy frame" (open/close via setGripper, exactly as before).
//
// Plain C++ with <stdint.h> only, so the same header can be dropped into the ROS2 driver later.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dume {
namespace claw {

constexpr size_t kFrameSize = 64;
constexpr uint16_t kRosPort = 80;
constexpr uint16_t kToolPort = 81;
constexpr uint8_t kExtMagic = 0xD5;
constexpr uint8_t kExtVersion = 1;
constexpr int kNumFingers = 3;  // A, B, C = motors on J1, J2, J3

// Extension commands. A command runs once per new commandSeq value.
// fingerMask: bit0 = A, bit1 = B, bit2 = C; 0 = all fingers.
enum class Command : uint8_t {
  None = 0,
  Open = 1,
  Close = 2,            // close until each finger reaches its grip threshold, then hold
  CloseToForce = 3,     // like Close, but the grip threshold is targetCurrentmA
  MoveFinger = 4,       // arg = +1 close / -1 open, effortPct, durationMs
  Stop = 5,             // brake the selected fingers
  ResetFaults = 6,
  SetCalibration = 7,   // arg = 1 on / 0 off: stream live currents on the serial port
};

enum class CommandResult : uint8_t {
  None = 0,
  Accepted = 1,
  Rejected = 2,
  BadArgument = 3,
};

enum class FingerState : uint8_t {
  Idle = 0,      // motor coasting
  Braked = 1,    // stopped with the H-bridge shorted
  Opening = 2,
  Open = 3,      // open move finished, coasting
  Closing = 4,
  Gripped = 5,   // grip threshold reached, holding at reduced power
  Moving = 6,    // manual timed move
  Fault = 7,
  Disabled = 8,  // no current sensor: motor locked out
};

enum class FingerFault : uint8_t {
  None = 0,
  OverCurrent = 1,
  Timeout = 2,
  NoSensor = 3,
  EStop = 4,
};

// Summary of all fingers.
enum class ClawState : uint8_t {
  Idle = 0,
  Open = 1,
  Opening = 2,
  Closing = 3,
  Gripped = 4,
  Moving = 5,
  Fault = 6,
};

enum StatusFlag : uint8_t {
  kFlagCalibration = 1 << 0,
  kFlagSensorFault = 1 << 1,  // at least one INA219 missing at boot
};

// errorCode byte: 0..3 are the original values, the rest are new (the stock driver ignores it).
enum class ErrorCode : uint8_t {
  noError = 0,
  errorA = 1,
  errorB = 2,
  errorC = 3,
  fingerFault = 4,
  sensorFault = 5,
  eStop = 6,
};

struct __attribute__((packed)) PcToRobotFrame {
  // ---- original layout, do not change ----
  uint8_t messageNumber;
  uint8_t emergencyStop;
  uint8_t reserved0;
  uint8_t enablePower;
  uint8_t setGripper;        // 0 = open, 1 = close
  // ---- extension (all zero from the stock driver) ----
  uint8_t extMagic;
  uint8_t extVersion;
  uint8_t command;           // Command
  uint8_t commandSeq;
  uint8_t fingerMask;
  int8_t arg;                // MoveFinger: +1 close / -1 open; SetCalibration: 1 on / 0 off
  uint8_t effortPct;         // MoveFinger: PWM duty 0..100
  uint16_t durationMs;       // MoveFinger
  uint16_t targetCurrentmA;  // CloseToForce
  uint8_t reserved1[48];
};

struct __attribute__((packed)) RobotToPcFrame {
  // ---- original layout, do not change ----
  uint8_t messageNumber;     // request messageNumber + 1
  uint8_t errorCode;         // ErrorCode
  uint8_t reserved0;
  uint8_t active;            // echo of enablePower
  uint8_t gripperState;      // 0 = open, 1 = closed (last open/close command, as the original)
  // ---- extension ----
  uint8_t extMagic;
  uint8_t extVersion;
  uint8_t clawState;         // ClawState
  uint8_t lastCommandSeq;
  uint8_t lastCommandResult; // CommandResult
  uint8_t fingerState[kNumFingers];  // FingerState
  uint8_t fingerFault[kNumFingers];  // FingerFault
  int16_t currentmA[kNumFingers];    // filtered |motor current|
  int16_t peakmA[kNumFingers];       // peak |current| since the last command
  int8_t rssi;               // WiFi RSSI dBm
  uint8_t flags;             // StatusFlag
  uint8_t reserved1[34];
};

static_assert(sizeof(PcToRobotFrame) == kFrameSize, "PcToRobotFrame must be 64 bytes");
static_assert(sizeof(RobotToPcFrame) == kFrameSize, "RobotToPcFrame must be 64 bytes");
static_assert(offsetof(PcToRobotFrame, setGripper) == 4, "legacy layout changed");
static_assert(offsetof(PcToRobotFrame, extMagic) == 5, "extension must start after legacy part");
static_assert(offsetof(RobotToPcFrame, gripperState) == 4, "legacy layout changed");
static_assert(offsetof(RobotToPcFrame, extMagic) == 5, "extension must start after legacy part");

inline bool hasExtension(const PcToRobotFrame& f) {
  return f.extMagic == kExtMagic && f.extVersion >= 1;
}

}  // namespace claw
}  // namespace dume
