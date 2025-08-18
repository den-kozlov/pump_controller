#pragma once
#include <GTimer.h>
#include <Arduino.h>
#include "sensor.h"

enum pumpState_t {
    PUMP_OFF = 0,
    PUMP_ON_WORKING,
    PUMP_ON_IDLE,
    PUMP_EMERGENCY_SHUTDOWN
};

class Pump {
public:
  explicit Pump(int pin, Sensor* sensor)
      : pinNumber(pin), state(PUMP_OFF), activePeriod(1000 * 60 * 2), sensor(sensor), pumpOnDuration(1000 * 5), pumpOffDuration(1000 * 10) {}

  int getPin() const { return pinNumber; }
  pumpState_t getState() const { return state; }
  void setActivePeriod(unsigned long period) { activePeriod = period; }
  void setPumpOnDuration(uint16_t duration) { pumpOnDuration = duration; }
  void setPumpOffDuration(uint16_t duration) { pumpOffDuration = duration; }
  void setPumpActiveDuration(uint16_t duration) { activePeriod = duration; }

  void tick();
  bool isActive() { return activeTimer.running(); }
private:
  unsigned long activePeriod;
  uint16_t pumpOnDuration;
  uint16_t pumpOffDuration;
  int pinNumber;
  pumpState_t state;
  uTimer16<millis> stateTimer;
  uTimer16<millis> activeTimer;
  Sensor* sensor;
};