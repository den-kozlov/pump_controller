#pragma once
#include <Arduino.h>
#include "sensor.h"
#include "pump.h"
#include "debug_log.h"

class PumpController {
public:
  PumpController(Sensor* sensor, Pump* pump)
    : sensor(sensor), pump(pump) {}

  virtual ~PumpController() = default;

  virtual void tick() = 0;

protected:
  Sensor* sensor;
  Pump* pump;
};

// Pump controller that activates the pump for a specific duration when high water mark is reached
// activeTime is in seconds
// Warning!!!
// It doesn't care about the low water mark, so potentially it could run the pump dry!
// That's why activeTime should be set carefully, so the pump guarantees to stop before
// the water level drop below the pump inlet
class EmptyingPumpTimeoutController : public PumpController
{
public:
  static const uint16_t DEFAULT_ACTIVE_TIME = 2 * 60; // 2 minutes in seconds
  EmptyingPumpTimeoutController(Sensor* sensor, Pump* pump, uint16_t activeTime = DEFAULT_ACTIVE_TIME)
    : PumpController(sensor, pump) {
      this->activeTime = (uint32_t)activeTime * 1000;
      this->activityTimer = new uTimer<millis>();
      DEBUG_PRINT("EmptyingPumpTimeoutController::EmptyingPumpTimeoutController activeTime: ");
      DEBUG_PRINTLN(this->activeTime);
    }

  void tick() override {
    sensor->tick();
    if (this->activityTimer->running()) {
      if (this->activityTimer->timeout(this->activeTime)) {
        pump->stop();
      }
    } else if (sensor->getLevel() > Sensor::SENSOR_LEVEL_MIN) {
      pump->start();
      this->activityTimer->start();
    };
    pump->tick();
  }
  void setActiveTime(uint16_t time) { activeTime = (uint32_t)time * 1000; }
private:
  uint32_t activeTime;
  uTimer<millis> *activityTimer;
};