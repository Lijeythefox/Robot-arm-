// DUM-E 3-finger tendon claw firmware (ClawGripperPCB, Waveshare ESP32-C5-MINI-KIT).
//
// Single core, so the split is by priority:
//   control task (CONTROL_TASK_PRIORITY): motors, current sensing, grip logic, LEDs
//   network task (NETWORK_TASK_PRIORITY): WiFi + TCP ports 80 (ROS2) and 81 (tools)
//   loop() (priority 1): serial console
#include <Arduino.h>

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
  SerialConsole::printHelp();
}

void loop() {
  console.poll();
  delay(5);
}
