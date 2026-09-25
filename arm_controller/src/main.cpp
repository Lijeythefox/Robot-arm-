// DUM-E arm controller v5 firmware (ESP32-DevKitC-32D, 6x DRV8825).
// Based on diy_robotics_arm_esp32 by M. Fuhrer, R. Wolf and H. Bornemann.
//
// Core 1: Arduino loop() = ArmController (stepping, endstops, safety, state machine).
// Core 0: network task (ROS2 port 80 + tool port 81), UI task (OLED + serial console).
#include <Arduino.h>
#include <WiFi.h>  // listed here so PlatformIO's library finder picks it up for FrameServer
#include <dume/ArmProtocol.h>
#include <dume/FrameServer.hpp>

#include "ArmController.hpp"
#include "Axis.hpp"
#include "Configuration.h"
#include "OledDisplay.hpp"
#include "SerialConsole.hpp"
#include "SharedState.hpp"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy include/secrets.example.h to include/secrets.h and set your WiFi credentials"
#endif

// Axis order matters: Axis::axisList[0] must be axis 1.
Axis axis1(1, STEPPER_1_STEP, STEPPER_1_DIR, AXIS_1_GEARING, AXIS_1_STEPS_PER_REV, AXIS_1_MICROSTEPS, AXIS_1_POS_MIN, AXIS_1_POS_MAX, AXIS_1_UNHOMED_VEL_MAX, AXIS_1_UNHOMED_ACC_MAX, AXIS_1_INVERT_DIRECTION);
Axis axis2(2, STEPPER_2_STEP, STEPPER_2_DIR, AXIS_2_GEARING, AXIS_2_STEPS_PER_REV, AXIS_2_MICROSTEPS, AXIS_2_POS_MIN, AXIS_2_POS_MAX, AXIS_2_UNHOMED_VEL_MAX, AXIS_2_UNHOMED_ACC_MAX, AXIS_2_INVERT_DIRECTION);
Axis axis3(3, STEPPER_3_STEP, STEPPER_3_DIR, AXIS_3_GEARING, AXIS_3_STEPS_PER_REV, AXIS_3_MICROSTEPS, AXIS_3_POS_MIN, AXIS_3_POS_MAX, AXIS_3_UNHOMED_VEL_MAX, AXIS_3_UNHOMED_ACC_MAX, AXIS_3_INVERT_DIRECTION);
Axis axis4(4, STEPPER_4_STEP, STEPPER_4_DIR, AXIS_4_GEARING, AXIS_4_STEPS_PER_REV, AXIS_4_MICROSTEPS, AXIS_4_POS_MIN, AXIS_4_POS_MAX, AXIS_4_UNHOMED_VEL_MAX, AXIS_4_UNHOMED_ACC_MAX, AXIS_4_INVERT_DIRECTION);
Axis axis5(5, STEPPER_5_STEP, STEPPER_5_DIR, AXIS_5_GEARING, AXIS_5_STEPS_PER_REV, AXIS_5_MICROSTEPS, AXIS_5_POS_MIN, AXIS_5_POS_MAX, AXIS_5_UNHOMED_VEL_MAX, AXIS_5_UNHOMED_ACC_MAX, AXIS_5_INVERT_DIRECTION);
Axis axis6(6, STEPPER_6_STEP, STEPPER_6_DIR, AXIS_6_GEARING, AXIS_6_STEPS_PER_REV, AXIS_6_MICROSTEPS, AXIS_6_POS_MIN, AXIS_6_POS_MAX, AXIS_6_UNHOMED_VEL_MAX, AXIS_6_UNHOMED_ACC_MAX, AXIS_6_INVERT_DIRECTION);

ArmController controller;
OledDisplay display;
SerialConsole console;

// ------------------------------------------------------------------------------------------
// Network
// ------------------------------------------------------------------------------------------
static arm::ErrorCode errorCodeFor(const ArmStatus& s) {
  if (s.faultFlags & (arm::kFaultEStopOpen | arm::kFaultEStopLatched | arm::kFaultSoftEStop))
    return arm::ErrorCode::eStop;
  if (s.faultFlags & arm::kFaultHomingFailed) return arm::ErrorCode::homingFailed;
  if (s.faultFlags & arm::kFaultEndstopHit) return arm::ErrorCode::endstopHit;
  if (s.faultFlags & arm::kFaultCommTimeout) return arm::ErrorCode::commTimeout;
  return arm::ErrorCode::noError;
}

// Handles one 64-byte request. channel 0 = ROS port, 1 = tool port; each keeps its own
// command sequence so the two clients cannot swallow each other's commands.
static bool handleFrame(uint8_t channel, const uint8_t* request, uint8_t* response, bool newClient) {
  static int16_t lastSeq[2] = {-1, -1};
  if (newClient) lastSeq[channel] = -1;

  arm::PcToRobotFrame rx;
  memcpy(&rx, request, sizeof(rx));
  bool ext = arm::hasExtension(rx);
  bool setpointsValid = !ext || (rx.extFlags & arm::kExtSetpointsValid);

  // The stock ROS2 driver always sends 0 here; any other client can use it as a software E-stop.
  if (rx.emergencyStop) shared::pcEmergencyStop = true;

  if (setpointsValid) {
    SetpointMessage msg;
    msg.activate = rx.activate != 0;
    memcpy(msg.milliDegrees, rx.jointSetpoints, sizeof(msg.milliDegrees));
    msg.receivedMs = millis();
    shared::postSetpoints(msg);
  }

  if (ext && rx.command != (uint8_t)arm::Command::None && rx.commandSeq != lastSeq[channel]) {
    lastSeq[channel] = rx.commandSeq;
    MotionCommand cmd;
    cmd.type = (arm::Command)rx.command;
    cmd.axis = rx.commandArg;
    memcpy(cmd.param, rx.param, sizeof(cmd.param));
    cmd.seq = rx.commandSeq;
    cmd.reportResult = true;
    shared::postCommand(cmd);
  }

  ArmStatus s = shared::readStatus();
  arm::RobotToPcFrame tx = {};
  tx.messageNumber = rx.messageNumber + 1;
  tx.errorCode = (uint8_t)errorCodeFor(s);
  tx.active = setpointsValid ? rx.activate : s.active;
  memcpy(tx.jointPositions, s.positionMilliDeg, sizeof(tx.jointPositions));
  tx.extMagic = arm::kExtMagic;
  tx.extVersion = arm::kExtVersion;
  tx.state = (uint8_t)s.state;
  tx.homedMask = s.homedMask;
  tx.endstopMask = s.endstopMask;
  tx.endstopEnabledMask = s.endstopEnabledMask;
  tx.faultFlags = s.faultFlags;
  tx.homingAxis = s.homingAxis;
  tx.homingPhase = s.homingPhase;
  tx.lastCommandSeq = s.lastCommandSeq;
  tx.lastCommandResult = (uint8_t)s.lastCommandResult;
  tx.driversEnabled = s.driversEnabled;
  tx.enableNetHigh = s.enableNetHigh;
  tx.speedTestAxis = s.speedTestAxis;
  tx.speedTestStepsPerSec = s.speedTestStepsPerSec;
  memcpy(response, &tx, sizeof(tx));
  return true;
}

static void networkTask(void*) {
  dume::NetConfig cfg{WIFI_SSID, WIFI_PASSWORD, ARM_HOSTNAME, ARM_USE_STATIC_IP,
                      IPAddress(IP_ADDRESS), IPAddress(GATEWAY_IP), IPAddress(SUBNET_MASK)};
  dume::WiFiStation station;
  dume::FrameServer<arm::kFrameSize> rosServer(
      arm::kRosPort, [](const uint8_t* rq, uint8_t* rs, bool nc) { return handleFrame(0, rq, rs, nc); });
  dume::FrameServer<arm::kFrameSize> toolServer(
      arm::kToolPort, [](const uint8_t* rq, uint8_t* rs, bool nc) { return handleFrame(1, rq, rs, nc); });

  station.begin(cfg);
  for (;;) {
    bool up = station.poll();
    shared::wifiConnected = up;
    if (up) {
      rosServer.begin();
      toolServer.begin();
      rosServer.poll();
      toolServer.poll();
    }
    vTaskDelay(1);
  }
}

// ------------------------------------------------------------------------------------------
// OLED + serial console (both low priority, core 0)
// ------------------------------------------------------------------------------------------
static void uiTask(void*) {
  uint32_t lastDraw = 0;
  for (;;) {
    console.poll();
    if (millis() - lastDraw >= OLED_REFRESH_MS) {
      lastDraw = millis();
      display.render(shared::readStatus(), shared::wifiConnected);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nDUM-E arm controller v5 starting");

  shared::init();
  display.begin();
  display.showMessage("DUM-E arm", "starting...");
  controller.begin();

  xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, NETWORK_TASK_PRIORITY, nullptr,
                          NETWORK_TASK_CORE);
  xTaskCreatePinnedToCore(uiTask, "ui", 6144, nullptr, 1, nullptr, 0);
  SerialConsole::printHelp();
}

void loop() { controller.loopOnce(); }
