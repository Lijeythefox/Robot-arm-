#ifndef SHARED_STATE_HPP
#define SHARED_STATE_HPP

// Everything that crosses between the motion loop (core 1) and the network / display /
// console tasks (core 0). The motion loop is the only code that touches the steppers; the
// other tasks send it commands through queues and read a status snapshot.
//
// (The original firmware called moveTo() from the network task while loop() stepped the same
// AccelStepper objects on the other core, with no locking.)

#include <Arduino.h>
#include <dume/ArmProtocol.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <stdarg.h>

namespace arm = dume::arm;

struct MotionCommand {
  arm::Command type = arm::Command::None;
  uint8_t axis = 0;  // 1..6, 0 = all / not applicable
  int32_t param[3] = {0, 0, 0};
  uint8_t seq = 0;
  bool reportResult = false;  // network commands echo seq/result in the status
};

// Latest setpoints from a streaming client. Only the newest one matters, so it is a
// one-element mailbox that is overwritten, not a queue.
struct SetpointMessage {
  bool activate = false;
  int32_t milliDegrees[arm::kNumAxes] = {};
  uint32_t receivedMs = 0;
};

struct ArmStatus {
  arm::State state = arm::State::Disabled;
  uint8_t homedMask = 0;
  uint8_t endstopMask = 0;
  uint8_t endstopEnabledMask = 0;
  uint8_t faultFlags = 0;
  uint8_t homingAxis = 0;
  uint8_t homingPhase = 0;
  uint8_t lastCommandSeq = 0;
  arm::CommandResult lastCommandResult = arm::CommandResult::None;
  bool driversEnabled = false;
  bool enableNetHigh = false;
  bool active = false;  // last activate flag from the PC
  uint8_t speedTestAxis = 0;
  int32_t speedTestStepsPerSec = 0;
  int32_t positionMilliDeg[arm::kNumAxes] = {};
  uint32_t updatedMs = 0;
};

struct EventText {
  char text[80];
};

namespace shared {

inline QueueHandle_t commandQueue = nullptr;
inline QueueHandle_t setpointMailbox = nullptr;
inline QueueHandle_t eventQueue = nullptr;
inline portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
inline ArmStatus status;
inline volatile bool wifiConnected = false;
// Set by the network task when a PC frame has emergencyStop != 0; consumed by the motion loop.
inline volatile bool pcEmergencyStop = false;

inline void init() {
  commandQueue = xQueueCreate(16, sizeof(MotionCommand));
  setpointMailbox = xQueueCreate(1, sizeof(SetpointMessage));
  eventQueue = xQueueCreate(24, sizeof(EventText));
}

inline bool postCommand(const MotionCommand& cmd) {
  return xQueueSend(commandQueue, &cmd, 0) == pdTRUE;
}

inline void postSetpoints(const SetpointMessage& msg) { xQueueOverwrite(setpointMailbox, &msg); }

inline ArmStatus readStatus() {
  ArmStatus copy;
  portENTER_CRITICAL(&statusMux);
  copy = status;
  portEXIT_CRITICAL(&statusMux);
  return copy;
}

inline void writeStatus(const ArmStatus& s) {
  portENTER_CRITICAL(&statusMux);
  status = s;
  portEXIT_CRITICAL(&statusMux);
}

// Non-blocking log for the motion loop: printing to Serial from core 1 could stall stepping
// when the UART buffer is full, so events are queued and printed by the console task.
inline void logEvent(const char* fmt, ...) {
  EventText ev;
  va_list args;
  va_start(args, fmt);
  vsnprintf(ev.text, sizeof(ev.text), fmt, args);
  va_end(args);
  xQueueSend(eventQueue, &ev, 0);  // drop if full rather than block
}

}  // namespace shared

#endif
