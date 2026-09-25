#ifndef CLAW_SHARED_HPP
#define CLAW_SHARED_HPP

// What crosses between the control task (motors + current sensing) and the network / console.
// Only the control task touches the motors and the I2C bus.

#include <Arduino.h>
#include <dume/ClawProtocol.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <stdarg.h>

namespace claw = dume::claw;

struct ClawCommand {
  claw::Command type = claw::Command::None;
  uint8_t fingerMask = 0;  // 0 = all
  int8_t arg = 0;
  uint8_t effortPct = 0;
  uint16_t durationMs = 0;
  uint16_t targetCurrentmA = 0;
  uint8_t seq = 0;
  bool reportResult = false;
};

struct ClawStatus {
  claw::ClawState state = claw::ClawState::Idle;
  bool gripperState = false;  // legacy: last open (0) / close (1) command
  bool active = false;
  uint8_t flags = 0;
  claw::FingerState fingerState[claw::kNumFingers] = {};
  claw::FingerFault fingerFault[claw::kNumFingers] = {};
  float currentmA[claw::kNumFingers] = {};
  float peakmA[claw::kNumFingers] = {};
  uint8_t lastCommandSeq = 0;
  claw::CommandResult lastCommandResult = claw::CommandResult::None;
};

struct EventText {
  char text[96];
};

namespace shared {

inline QueueHandle_t commandQueue = nullptr;
inline QueueHandle_t eventQueue = nullptr;
inline portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
inline ClawStatus status;
inline volatile bool wifiConnected = false;
inline volatile int8_t rssi = 0;

inline void init() {
  commandQueue = xQueueCreate(16, sizeof(ClawCommand));
  eventQueue = xQueueCreate(24, sizeof(EventText));
}

inline bool postCommand(const ClawCommand& cmd) { return xQueueSend(commandQueue, &cmd, 0) == pdTRUE; }

inline ClawStatus readStatus() {
  ClawStatus copy;
  portENTER_CRITICAL(&statusMux);
  copy = status;
  portEXIT_CRITICAL(&statusMux);
  return copy;
}

inline void writeStatus(const ClawStatus& s) {
  portENTER_CRITICAL(&statusMux);
  status = s;
  portEXIT_CRITICAL(&statusMux);
}

// The control task never prints directly (USB serial can block); events are printed by the
// console in loop().
inline void logEvent(const char* fmt, ...) {
  EventText ev;
  va_list args;
  va_start(args, fmt);
  vsnprintf(ev.text, sizeof(ev.text), fmt, args);
  va_end(args);
  xQueueSend(eventQueue, &ev, 0);
}

}  // namespace shared

#endif
