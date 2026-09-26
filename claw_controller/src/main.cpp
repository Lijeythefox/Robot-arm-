// DUM-E 3-finger tendon claw firmware (ClawGripperPCB, Waveshare ESP32-C5-MINI-KIT).
//
// Single core, so the split is by priority:
//   control task (CONTROL_TASK_PRIORITY): motors, current sensing, grip logic, LEDs
//   network task (NETWORK_TASK_PRIORITY): WiFi + TCP ports 80 (ROS2) and 81 (tools)
//   loop() (priority 1): serial console
#include <Arduino.h>
#include <WiFi.h>  // listed here so PlatformIO's library finder picks it up for FrameServer
#include <dume/ClawProtocol.h>
#include <dume/FrameServer.hpp>

#include "ClawController.hpp"
#include "ClawShared.hpp"
#include "Configuration.h"
#include "SerialConsole.hpp"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy include/secrets.example.h to include/secrets.h and set your WiFi credentials"
#endif

ClawController controller;
SerialConsole console;

// ------------------------------------------------------------------------------------------
// Network
// ------------------------------------------------------------------------------------------
static claw::ErrorCode errorCodeFor(const ClawStatus& s) {
  for (int i = 0; i < claw::kNumFingers; i++)
    if (s.fingerFault[i] == claw::FingerFault::EStop) return claw::ErrorCode::eStop;
  if (s.flags & claw::kFlagSensorFault) return claw::ErrorCode::sensorFault;
  if (s.state == claw::ClawState::Fault) return claw::ErrorCode::fingerFault;
  return claw::ErrorCode::noError;
}

// channel 0 = ROS port, 1 = tool port; each keeps its own command sequence.
static bool handleFrame(uint8_t channel, const uint8_t* request, uint8_t* response, bool newClient) {
  static int16_t lastSeq[2] = {-1, -1};
  if (newClient) lastSeq[channel] = -1;

  claw::PcToRobotFrame rx;
  memcpy(&rx, request, sizeof(rx));
  ClawStatus s = shared::readStatus();
  bool gripperState = s.gripperState;

  // The stock ROS2 driver always sends 0 here; any other client can use it as a software E-stop.
  if (rx.emergencyStop) shared::pcEmergencyStop = true;

  if (!claw::hasExtension(rx)) {
    // Original behaviour: act only when setGripper differs from the last open/close command.
    // Also retry after a fault, so calling the ROS2 service again recovers.
    bool want = rx.setGripper != 0;
    if (want != s.gripperState || s.state == claw::ClawState::Fault) {
      ClawCommand cmd;
      cmd.type = want ? claw::Command::Close : claw::Command::Open;
      shared::postCommand(cmd);
    }
    gripperState = want;  // report the new command, like the original did after executing it
  } else if (rx.command != (uint8_t)claw::Command::None && rx.commandSeq != lastSeq[channel]) {
    lastSeq[channel] = rx.commandSeq;
    ClawCommand cmd;
    cmd.type = (claw::Command)rx.command;
    cmd.fingerMask = rx.fingerMask;
    cmd.arg = rx.arg;
    cmd.effortPct = rx.effortPct;
    cmd.durationMs = rx.durationMs;
    cmd.targetCurrentmA = rx.targetCurrentmA;
    cmd.seq = rx.commandSeq;
    cmd.reportResult = true;
    shared::postCommand(cmd);
    if (cmd.type == claw::Command::Open) gripperState = false;
    if (cmd.type == claw::Command::Close || cmd.type == claw::Command::CloseToForce) gripperState = true;
  }

  claw::RobotToPcFrame tx = {};
  tx.messageNumber = rx.messageNumber + 1;
  tx.errorCode = (uint8_t)errorCodeFor(s);
  tx.active = rx.enablePower;
  tx.gripperState = gripperState;
  tx.extMagic = claw::kExtMagic;
  tx.extVersion = claw::kExtVersion;
  tx.clawState = (uint8_t)s.state;
  tx.lastCommandSeq = s.lastCommandSeq;
  tx.lastCommandResult = (uint8_t)s.lastCommandResult;
  for (int i = 0; i < claw::kNumFingers; i++) {
    tx.fingerState[i] = (uint8_t)s.fingerState[i];
    tx.fingerFault[i] = (uint8_t)s.fingerFault[i];
    tx.currentmA[i] = (int16_t)lroundf(s.currentmA[i]);
    tx.peakmA[i] = (int16_t)lroundf(s.peakmA[i]);
  }
  tx.rssi = shared::rssi;
  tx.flags = s.flags;
  memcpy(response, &tx, sizeof(tx));
  return true;
}

static void networkTask(void*) {
  dume::NetConfig cfg{WIFI_SSID, WIFI_PASSWORD, CLAW_HOSTNAME, CLAW_USE_STATIC_IP,
                      IPAddress(IP_ADDRESS), IPAddress(GATEWAY_IP), IPAddress(SUBNET_MASK)};
  dume::WiFiStation station;
  dume::FrameServer<claw::kFrameSize> rosServer(
      claw::kRosPort, [](const uint8_t* rq, uint8_t* rs, bool nc) { return handleFrame(0, rq, rs, nc); });
  dume::FrameServer<claw::kFrameSize> toolServer(
      claw::kToolPort, [](const uint8_t* rq, uint8_t* rs, bool nc) { return handleFrame(1, rq, rs, nc); });

  station.begin(cfg);
  uint32_t lastRssiMs = 0;
  for (;;) {
    bool up = station.poll();
    shared::wifiConnected = up;
    if (up) {
      if (millis() - lastRssiMs > 1000) {
        lastRssiMs = millis();
        shared::rssi = (int8_t)WiFi.RSSI();
      }
      rosServer.begin();
      toolServer.begin();
      rosServer.poll();
      toolServer.poll();
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ------------------------------------------------------------------------------------------
// Control
// ------------------------------------------------------------------------------------------
static void controlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    controller.tick(millis());
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block when no USB host is listening
  delay(200);
  Serial.println("\nDUM-E claw starting");

  shared::init();
  controller.begin();
  xTaskCreate(controlTask, "control", 6144, nullptr, CONTROL_TASK_PRIORITY, nullptr);
  xTaskCreate(networkTask, "network", 8192, nullptr, NETWORK_TASK_PRIORITY, nullptr);
  SerialConsole::printHelp();
}

void loop() {
  console.poll();
  delay(5);
}
