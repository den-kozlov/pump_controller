#pragma once
#include <GTimer.h>
#include <Arduino.h>
#include "sensor.h"
#include "debug_log.h"
class Pump {
public:
  static const uint16_t DEFAULT_PUMP_ON_DURATION = 5 * 1000;
  static const uint16_t DEFAULT_PUMP_OFF_DURATION = 10 * 1000;
  enum pumpState_t {
    PUMP_OFF = 0,
    PUMP_ON_WORKING,
    PUMP_ON_IDLE
  };

  Pump(uint16_t pumpOnDuration = DEFAULT_PUMP_ON_DURATION, uint16_t pumpOffDuration = DEFAULT_PUMP_OFF_DURATION) {
    this->state = PUMP_OFF;
    this->pumpOffDuration = pumpOffDuration ;
    this->pumpOnDuration = pumpOnDuration;
    this->stateTimer = uTimer16<millis>();
    this->onStateChange = nullptr;
    DEBUG_PRINT("Pump::Pump pumpOnDuration: ");
    DEBUG_PRINTLN(this->pumpOnDuration);
    DEBUG_PRINT("Pump::Pump pumpOffDuration: ");
    DEBUG_PRINTLN(this->pumpOffDuration);
  }

  virtual ~Pump() = default;
  virtual void tick();
  pumpState_t getState() { return state; }
  void setPumpOnDuration(uint16_t duration) { pumpOnDuration = duration; }
  void setPumpOffDuration(uint16_t duration) { pumpOffDuration = duration; }

  void setOnStateChange(std::function<void()> callback) { onStateChange = callback; }
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
  std::function<void()> onStateChange;
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
    DEBUG_PRINTLN("RelayPump::turnOn");
    if (onStateChange) {
        onStateChange();
    }
  }

  void turnOff() override {
    digitalWrite(pinNumber, LOW);
    DEBUG_PRINTLN("RelayPump::turnOff");
    if (onStateChange) {
        onStateChange();
    }
  }
private:
  uint8_t pinNumber;
};

class PWMPump : public Pump {
public:
  PWMPump(uint8_t pin, uint16_t pumpOnDuration = DEFAULT_PUMP_ON_DURATION, uint16_t pumpOffDuration = DEFAULT_PUMP_OFF_DURATION)
      : Pump(pumpOnDuration, pumpOffDuration), pinNumber(pin) {
          pinMode(pinNumber, OUTPUT);
          digitalWrite(pinNumber, LOW);
          DEBUG_PRINT("PWMPump::PWMPump pin: ");
          DEBUG_PRINTLN(pinNumber);
      }

  uint8_t getPin() const { return pinNumber; }

  void setPWMFrequency(uint16_t freq);
  void setPWMDutyCycle(uint8_t duty) {
    if (duty > 255) duty = 255;
    pwmDutyCycle = duty;
    DEBUG_PRINT("PWMPump::setPWMDutyCycle: ");
    DEBUG_PRINTLN(pwmDutyCycle);
  }

protected:
  void turnOn() override {
    DEBUG_PRINTLN("PWMPump::turnOn");
    analogWrite(pinNumber, pwmDutyCycle);
    if (onStateChange) {
        onStateChange();
    }
  }

  void turnOff() override {
    DEBUG_PRINTLN("PWMPump::turnOff");
    analogWrite(pinNumber, 0);
    if (onStateChange) {
        onStateChange();
    }
  }
private:
  uint8_t pinNumber;
  uint8_t pwmDutyCycle = 128;
};