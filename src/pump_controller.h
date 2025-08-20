#pragma once
#include <Arduino.h>
#include "sensor.h"
#include "pump.h"

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
// activeTime is in milliseconds
// Warning!!!
// It doesn't care about the low water mark, so potentially it could run the pump dry!
// That's why activeTime should be set carefully, so the pump guarantees to stop before
// the water level drop below the pump inlet
class EmptyingPumpTimeoutController : public PumpController
{
public:
  static const uint16_t DEFAULT_ACTIVE_TIME = 2 * 60 * 1000; // 2 minutes in milliseconds
  EmptyingPumpTimeoutController(Sensor* sensor, Pump* pump, uint16_t activeTime)
    : PumpController(sensor, pump), activeTime(activeTime) {}

  void tick() override {
    sensor->tick();
    if (activeTimer.running()) {
      if (activeTimer.timeout(activeTime)) {
        pump->stop();
      }
    } else if (sensor->getLevel() == Sensor::SENSOR_LEVEL_MAX) {
      pump->start();
      activeTimer.start();
    };
    pump->tick();
  }
  void setActiveTime(uint16_t time) { activeTime = time; }
private:
  uint16_t activeTime;
  uTimer16<millis> activeTimer;
};