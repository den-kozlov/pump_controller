#pragma once
#include <GTimer.h>
#include <Arduino.h>

class Sensor {
public:
  static const uint8_t SENSOR_LEVEL_MAX = 255;
  static const uint8_t SENSOR_LEVEL_MIN = 0;

  virtual ~Sensor() = default;
  virtual uint8_t getLevel() = 0;
  virtual void tick() = 0;
};

class TriggerSensor : public Sensor {
public:
  static const uint16_t DEFAULT_RATTLE_THRESHOLD = 1000;

  TriggerSensor(uint8_t pin, uint16_t rattleThreshold = DEFAULT_RATTLE_THRESHOLD)
      : pinNumber(pin), rattleThreshold(rattleThreshold), triggered(false) 
      {
        pinMode(pinNumber, INPUT);
        digitalWrite(pinNumber, LOW);
      }

  uint8_t getPin() const { return pinNumber; }

  void setRattleThreshold(uint16_t threshold) { rattleThreshold = threshold; }


  virtual uint8_t getLevel() { return triggered ? SENSOR_LEVEL_MAX : SENSOR_LEVEL_MIN; }
  virtual void tick();

private:
  uint8_t pinNumber;
  uint16_t rattleThreshold;
  bool triggered;
  uTimer16<millis> rattleTimer;
};