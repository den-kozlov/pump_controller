#pragma once
#include <GTimer.h>
#include <Arduino.h>
#include "sensor.h"





class Pump {
public:
  static const uint16_t DEFAULT_PUMP_ON_DURATION = 1000 * 5;
  static const uint16_t DEFAULT_PUMP_OFF_DURATION = 1000 * 10;
  enum pumpState_t {
    PUMP_OFF = 0,
    PUMP_ON_WORKING,
    PUMP_ON_IDLE
  };
  
  Pump(uint16_t pumpOnDuration = DEFAULT_PUMP_ON_DURATION, uint16_t pumpOffDuration = DEFAULT_PUMP_OFF_DURATION)
      : state(PUMP_OFF),
        pumpOnDuration(pumpOnDuration),
        pumpOffDuration(pumpOffDuration),
        stateTimer() { }
  virtual ~Pump() = default;
  virtual void tick();
  pumpState_t getState() { return state; }
  void setPumpOnDuration(uint16_t duration) { pumpOnDuration = duration; }
  void setPumpOffDuration(uint16_t duration) { pumpOffDuration = duration; }

  void start();
  void stop();
  virtual void emergencyStop();

protected:
  virtual void turnOn() = 0;
  virtual void turnOff() = 0;
  pumpState_t state;
  uint16_t pumpOnDuration;
  uint16_t pumpOffDuration;
  uTimer16<millis> stateTimer;
};

class RelayPump : public Pump {
public:
  RelayPump(uint8_t pin, uint16_t pumpOnDuration = DEFAULT_PUMP_ON_DURATION, uint16_t pumpOffDuration = DEFAULT_PUMP_OFF_DURATION)
      : Pump(pumpOnDuration, pumpOffDuration), pinNumber(pin) {
          pinMode(pinNumber, OUTPUT);
          digitalWrite(pinNumber, LOW);
      }

  uint8_t getPin() const { return pinNumber; }
protected:
  void turnOn() override {
    digitalWrite(pinNumber, HIGH);
  }

  void turnOff() override {
    digitalWrite(pinNumber, LOW);
  }
private:
  uint8_t pinNumber;
};