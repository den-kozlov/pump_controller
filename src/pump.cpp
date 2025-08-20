#include "pump.h"
#include <Arduino.h>

void Pump::tick()
{
    switch (state)
    {
    case PUMP_OFF:
      break;

    case PUMP_ON_WORKING:
      // do not interrupt working state if pumpOffDuration is 0
      // in such cases, the pump will continue working until stopped
      if (pumpOffDuration == 0)
        break;

      if (stateTimer.timeout(pumpOnDuration))
      {
        state = PUMP_ON_IDLE;
        turnOn();
        stateTimer.start();
      }
      break;

    case PUMP_ON_IDLE:
      if (stateTimer.timeout(pumpOffDuration))
      {
        state = PUMP_ON_WORKING;
        turnOff();
        stateTimer.start();
      }
      break;
    };
}

void Pump::start()
{
    if (state == PUMP_OFF)
    {
        state = PUMP_ON_WORKING;
        stateTimer.start();
        turnOn();
    }
}

void Pump::stop()
{
    if (state == PUMP_ON_WORKING || state == PUMP_ON_IDLE)
    {
        state = PUMP_OFF;
        stateTimer.stop();
        turnOff();
    }
}

void Pump::emergencyStop()
{
    stop();
}