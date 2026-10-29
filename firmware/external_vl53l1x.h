#pragma once
#include <VL53L1X.h>  // Pololu vl53l1x-arduino; install separately.

// Optional sensor backend. Must be the only active sensor at I2C address 0x29.
// Wire already initialized by the TT default sketch. Do not call Wire.begin again.
// Replace built-in ToF initialization/read path when using this backend.
class OPExternalRange {
 public:
  VL53L1X sensor;
  bool begin() {
    sensor.setTimeout(100);
    if (!sensor.init()) return false;
    sensor.setDistanceMode(VL53L1X::Short);
    if (!sensor.setMeasurementTimingBudget(50000)) return false;
    sensor.startContinuous(60);
    return true;
  }
  // Call from one I2C owner. False means no new measurement, NOT reuse of old data.
  bool poll(uint16_t &mm, bool &valid) {
    if (!sensor.dataReady()) return false;
    mm = sensor.read(false);
    valid = !sensor.timeoutOccurred() && sensor.ranging_data.range_status == VL53L1X::RangeValid;
    return true;
  }
};
