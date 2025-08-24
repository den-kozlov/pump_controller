#include "sensor.h"
#include <Arduino.h>
#include <GTimer.h>
#include <debug_log.h>

void TriggerSensor::tick() {
    // Sensor pins are pulled up by the external resistors.
    // Dry sensor produce HIGH signal on its pin.
    // So we need to invert the logic.
    bool current_state = !digitalRead(pinNumber);
    if (triggered != current_state) {
        if (!rattleTimer.running()) {
            rattleTimer.start();
        }
    } else {
        if (rattleTimer.running()) {
            DEBUG_PRINTLN("Sensor rattle detected");
        }
        rattleTimer.stop();
    }
    if (rattleTimer.timeout(rattleThreshold)) {
        triggered = !triggered;
        DEBUG_PRINT("Sensor triggered : ");
        DEBUG_PRINTLN(triggered ? "ON" : "OFF");
    }
}