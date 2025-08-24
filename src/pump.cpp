#include "pump.h"
#include <Arduino.h>
#include "debug_log.h"

void Pump::tick()
{
    switch (state)
    {
    case PUMP_OFF:
      break;

    case PUMP_ON_WORKING:
      // do not interrupt working state if pumpOffDuration is 0
      // in such cases, the pump will continue working until stopped
      if (pumpOffDuration == 0) {
        break;
      }

      if (stateTimer.timeout(pumpOnDuration))
      {
        state = PUMP_ON_IDLE;
        turnOff();
        stateTimer.start();
      }
      break;

    case PUMP_ON_IDLE:
      if (stateTimer.timeout(pumpOffDuration))
      {
        state = PUMP_ON_WORKING;
        turnOn();
        stateTimer.start();
      }
      break;
    };
}

void Pump::start()
{
    if (state == PUMP_OFF)
    {
        DEBUG_PRINTLN("Pump::start");
        state = PUMP_ON_WORKING;
        stateTimer.start();
        turnOn();
    }
}

void Pump::stop()
{
    if (state == PUMP_ON_WORKING || state == PUMP_ON_IDLE)
    {
        DEBUG_PRINTLN("Pump::stop");
        state = PUMP_OFF;
        stateTimer.stop();
        turnOff();
    }
}

void Pump::emergencyStop()
{
    stop();
}

void PWMPump::setPWMFrequency(uint16_t freq) {
  // ESP8266 supports frequencies from 1Hz to 40kHz
  if (freq < 1) freq = 1;
  if (freq > 40000) freq = 40000;
  analogWriteFreq(freq);
  DEBUG_PRINT("PWMPump::setPWMFrequency: ");
  DEBUG_PRINTLN(freq);
}


