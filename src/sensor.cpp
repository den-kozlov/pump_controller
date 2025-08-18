#include "sensor.h"
#include <Arduino.h>
#include <GTimer.h>

void Sensor::tick() {
    // Sensor pins are pulled up by the external resistors.
    // Dry sensor produce HIGH signal on its pin.
    // So we need to invert the logic.
    bool current_state = !digitalRead(pinNumber);
    if (status != current_state) {
        if (!timer.running()) {
            timer.start();
        }
    } else {
        timer.stop();
    }
    if (timer.timeout(rattleThreshold)) {
        status = !status;
    }
}