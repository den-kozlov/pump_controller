#include "pump.h"
#include <Arduino.h>

void Pump::tick()
{
    if (!sensor) return;
    bool sensor_state = sensor->getStatus();

    switch (state)
    {
    case PUMP_OFF:
      if (sensor_state)
      {
        Serial.println("Sensor triggered, activating pump.");
        state = PUMP_ON_WORKING;
        digitalWrite(pinNumber, HIGH);
        activeTimer.start();
        stateTimer.start();
      }
      break;

    case PUMP_ON_WORKING:
      if (activeTimer.timeout(activePeriod))
      {
        Serial.println("Pump active period ended. Turning off.");
        state = PUMP_OFF;
        digitalWrite(pinNumber, LOW);
        stateTimer.stop();
        break;
      }
      if (stateTimer.timeout(activePeriod))
      {
        Serial.println("Pump working period ended.");
        state = PUMP_ON_IDLE;
        digitalWrite(pinNumber, LOW);
        stateTimer.start();
      }
      break;

    case PUMP_ON_IDLE:
      if (activeTimer.timeout(activePeriod))
      {
        Serial.println("Pump active period ended. Turning off.");
        state = PUMP_OFF;
        digitalWrite(pinNumber, LOW);
        stateTimer.stop();
        break;
      }
      if (stateTimer.timeout(pumpOffDuration))
      {
        state = PUMP_ON_WORKING;
        digitalWrite(pinNumber, HIGH);
        stateTimer.start();
      }
      break;

    case PUMP_EMERGENCY_SHUTDOWN:
      if (!sensor_state)
      {
        // Water level is above critical threshold. Disabling emergency shutdown.
        Serial.println("Water level is above critical threshold, disabling emergency shutdown.");
        state = PUMP_OFF;
      }
    };
}