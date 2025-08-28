#include "sensor.h"
#include <Arduino.h>
#include <GTimer.h>
#include <debug_log.h>

void TriggerSensor::tick() {
    bool current_state = digitalRead(pinNumber);
    if (activeLow) {
        current_state = !current_state;
    }
    if (triggered != current_state) {
        if (!rattleTimer.running()) {
            rattleTimer.start();
        }
    } else {
        rattleTimer.stop();
    }
    if (rattleTimer.timeout(rattleThreshold)) {
        triggered = !triggered;
        DEBUG_PRINT("Sensor triggered : ");
        DEBUG_PRINTLN(triggered ? "ON" : "OFF");
        if (onLevelChange) {
            onLevelChange();
        }
    }
}