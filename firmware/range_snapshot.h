#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

// Include in the official default.ino after CommonSerial is defined.
// No flight commands. Sensor task writes; query callback takes an atomic copy.
struct OPRangeSnapshot {
  uint32_t sequence = 0;
  uint32_t sampled_ms = 0;
  uint16_t millimeters = 0;
  bool valid = false;
};
static OPRangeSnapshot op_range;
static portMUX_TYPE op_range_mux = portMUX_INITIALIZER_UNLOCKED;

static void op_record_range(uint16_t mm) {
  portENTER_CRITICAL(&op_range_mux);
  ++op_range.sequence;
  op_range.sampled_ms = millis();
  op_range.millimeters = mm;
  op_range.valid = mm >= 20 && mm <= 1200 && mm != 8192 && mm != 65535;
  portEXIT_CRITICAL(&op_range_mux);
}

static int op_range_callback(int argc, char *argv[], char argv2[]) {
  OPRangeSnapshot snapshot;
  portENTER_CRITICAL(&op_range_mux);
  snapshot = op_range;
  portEXIT_CRITICAL(&op_range_mux);
  const uint32_t age = millis() - snapshot.sampled_ms;
  const bool valid = snapshot.valid && snapshot.sequence > 0 && age <= 400;
  // Custom application protocol, NOT an official DJI SDK command.
  CommonSerial.printf("oprange %lu %lu %u %u %lu\r\n",
      (unsigned long)snapshot.sequence, (unsigned long)snapshot.sampled_ms,
      snapshot.millimeters, valid ? 1 : 0, (unsigned long)age);
  return 0;
}
