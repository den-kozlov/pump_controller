#pragma once
#include <GTimer.h>
#include <Arduino.h>

class Sensor {
public:
  explicit Sensor(int pin, int rattleThreshold = 1000)
      : pinNumber(pin), rattleThreshold(rattleThreshold), status(false) {}

  int getPin() const { return pinNumber; }
  bool getStatus() const { return status; }

  void setRattleThreshold(int threshold) { rattleThreshold = threshold; }
  void tick();

private:
  int pinNumber;
  int rattleThreshold;
  bool status;
  uTimer16<millis> timer;
};