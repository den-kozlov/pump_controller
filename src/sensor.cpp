#include "sensor.h"
#include <Arduino.h>
#include <GTimer.h>

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
        rattleTimer.stop();
    }
    if (rattleTimer.timeout(rattleThreshold)) {
        triggered = !triggered;
    }
}