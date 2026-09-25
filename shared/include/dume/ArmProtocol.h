// Wire format between the PC (ROS2 driver or dume_cli.py) and the DUM-E arm controller.
//
// Every message is a fixed 64-byte frame over TCP, one request -> one response.
// Bytes 0..27 are the original diy_robotics layout (diy_robotarm_wer24_driver/data_format.hpp)
// and must never change: the stock ROS2 driver keeps working only as long as they stay put.
//
// Everything from byte 28 on is an optional extension. The stock driver keeps its request in a
// zero-initialised global, so it always sends zeros there; extMagic == 0 therefore means
// "legacy frame" and the firmware behaves like the original. It also only reads jointPositions
// from the response, so extra response fields are invisible to it.
//
// Plain C++ with <stdint.h> only, so the same header can be dropped into the ROS2 driver later.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dume {
namespace arm {

constexpr size_t kFrameSize = 64;
constexpr uint16_t kRosPort = 80;   // what the stock ROS2 driver connects to
constexpr uint16_t kToolPort = 81;  // second port so tools can talk while ROS holds port 80
constexpr uint8_t kExtMagic = 0xD5;
constexpr uint8_t kExtVersion = 1;
constexpr int kNumAxes = 6;

// Commands carried in the extension. A command runs once per new commandSeq value, so a client
// can keep re-sending the same frame (e.g. to poll status) without repeating the action.
enum class Command : uint8_t {
  None = 0,
  HomeAll = 1,         // home every axis with an enabled endstop, in HOMING_ORDER
  HomeAxis = 2,        // commandArg = axis 1..6
  Reset = 3,           // clear a latched E-stop / fault once the E-stop is closed again
  Stop = 4,            // decelerate every axis to a stop, abort homing / speed test
  SpeedTest = 5,       // commandArg = axis; param[0] = start steps/s, param[1] = max steps/s,
                       // param[2] = increment steps/s per round trip (0 = config defaults)
  DisableDrivers = 6,  // stop and put the drivers to sleep (motors go limp)
  SetHomeHere = 7,     // commandArg = axis (0 = all): declare the current position as homed
                       // at param[0] millidegrees. For axes without an endstop.
  Jog = 8,             // commandArg = axis; param[0] = relative move in millidegrees
  EnableDrivers = 9,   // wake the drivers and hold position
};

enum class CommandResult : uint8_t {
  None = 0,
  Accepted = 1,
  Rejected = 2,  // not allowed in the current state (e.g. E-stop latched, busy homing)
  BadArgument = 3,
};

enum class State : uint8_t {
  Disabled = 0,   // drivers asleep (boot, or after DisableDrivers)
  Ready = 1,      // drivers awake, following setpoints / jog commands
  Homing = 2,
  SpeedTest = 3,
  EStop = 4,      // latched; needs Reset after the E-stop is closed
};

// Bit flags in RobotToPcFrame::faultFlags.
enum FaultFlag : uint8_t {
  kFaultEStopOpen = 1 << 0,     // ENABLE net read LOW while GPIO4 drives it HIGH
  kFaultEStopLatched = 1 << 1,  // waiting for Reset
  kFaultEndstopHit = 1 << 2,    // an axis was stopped by its endstop outside homing
  kFaultHomingFailed = 1 << 3,
  kFaultCommTimeout = 1 << 4,   // setpoint stream stopped; axes held in place
  kFaultSoftEStop = 1 << 5,     // PC set emergencyStop
  kFaultSyncWait = 1 << 6,      // ignoring setpoints until they are close to the actual pose
};

// Bit flags in PcToRobotFrame::extFlags.
enum ExtFlag : uint8_t {
  // Tools that only want to send a command set this to 0 so jointSetpoints/activate are ignored.
  // Legacy frames (no extension) always carry valid setpoints.
  kExtSetpointsValid = 1 << 0,
};

// ErrorCode byte of the response. 0..3 are the original values (never used by the original
// firmware beyond noError); the rest are new and ignored by the stock ROS2 driver.
enum class ErrorCode : uint8_t {
  noError = 0,
  errorA = 1,
  errorB = 2,
  errorC = 3,
  eStop = 4,
  endstopHit = 5,
  homingFailed = 6,
  commTimeout = 7,
};

struct __attribute__((packed)) PcToRobotFrame {
  // ---- original layout, do not change ----
  uint8_t messageNumber;
  uint8_t emergencyStop;  // original bool; unused by the stock driver (always 0)
  uint8_t reserved0;
  uint8_t activate;       // "enablePower" in the ROS2 driver
  int32_t jointSetpoints[kNumAxes];  // millidegrees
  // ---- extension (all zero from the stock driver) ----
  uint8_t extMagic;       // kExtMagic when the fields below are valid
  uint8_t extVersion;
  uint8_t command;        // Command
  uint8_t commandArg;     // usually the axis number 1..6
  uint8_t commandSeq;
  uint8_t extFlags;       // ExtFlag
  int32_t param[3];
  uint8_t reserved1[18];
};

struct __attribute__((packed)) RobotToPcFrame {
  // ---- original layout, do not change ----
  uint8_t messageNumber;  // request messageNumber + 1
  uint8_t errorCode;      // ErrorCode
  uint8_t reserved0;
  uint8_t active;         // echo of activate
  int32_t jointPositions[kNumAxes];  // millidegrees
  // ---- extension ----
  uint8_t extMagic;
  uint8_t extVersion;
  uint8_t state;               // State
  uint8_t homedMask;           // bit n-1 = axis n homed
  uint8_t endstopMask;         // bit n-1 = axis n endstop triggered (debounced)
  uint8_t endstopEnabledMask;  // bit n-1 = axis n endstop fitted/enabled in config
  uint8_t faultFlags;          // FaultFlag
  uint8_t homingAxis;          // axis currently homing, 0 = none
  uint8_t homingPhase;         // HomingPhase (see firmware), for display only
  uint8_t lastCommandSeq;
  uint8_t lastCommandResult;   // CommandResult
  uint8_t driversEnabled;      // GPIO4 (ENABLE_ALL) state
  uint8_t enableNetHigh;       // GPIO34 reading of the ENABLE net
  uint8_t speedTestAxis;       // 0 = no speed test
  int32_t speedTestStepsPerSec;
  uint8_t reserved1[18];
};

static_assert(sizeof(PcToRobotFrame) == kFrameSize, "PcToRobotFrame must be 64 bytes");
static_assert(sizeof(RobotToPcFrame) == kFrameSize, "RobotToPcFrame must be 64 bytes");
static_assert(offsetof(PcToRobotFrame, jointSetpoints) == 4, "legacy layout changed");
static_assert(offsetof(PcToRobotFrame, extMagic) == 28, "extension must start after legacy part");
static_assert(offsetof(RobotToPcFrame, jointPositions) == 4, "legacy layout changed");
static_assert(offsetof(RobotToPcFrame, extMagic) == 28, "extension must start after legacy part");

inline bool hasExtension(const PcToRobotFrame& f) {
  return f.extMagic == kExtMagic && f.extVersion >= 1;
}

}  // namespace arm
}  // namespace dume
