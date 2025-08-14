#include <Arduino.h>
#include <GyverDBFile.h>
#include <LittleFS.h>
#include <GyverNTP.h>
#include <SettingsESP.h>
#include <WiFiConnector.h>
#include <ESP8266WiFi.h>
#include <GTimer.h>
GyverDBFile db(&LittleFS, "/data.db");
SettingsESP sett("WiFi config", &db);

DB_KEYS(kk, wifi_ssid, wifi_pass, ntp_server, ntp_offset, rattle_threshold, pump_on_duration, pump_off_duration, apply);

uint8_t lastState;
uint8_t relayPin = D7;
struct sensor_t {
    uint8_t pin;
    uTimer16<millis> *timer;
    bool state = false;
};

const uint8_t numSensors = 4;
// Water level sensors indexed from lower to higher
sensor_t sensors[4] = {
    {D1, new uTimer16<millis>(), false},
    {D2, new uTimer16<millis>(), false},
    {D5, new uTimer16<millis>(), false},
    {D6, new uTimer16<millis>(), false}
};
// Sensor rattle threshold
uint16_t rattleThreshold;

// Pump timer
uint16_t pumpOnDuration;
uint16_t pumpOffDuration;
uTimer<millis> pumpTimer;
enum {
    PUMP_OFF = 0,
    PUMP_ON_WORKING,
    PUMP_ON_IDLE,
    PUMP_EMERGENCY_SHUTDOWN
} pumpState;
uTimer16<millis> debugTimer;

void build(sets::Builder& b) {
    {
        if (b.beginGroup("🛜 WiFi")) {
            b.Input(kk::wifi_ssid, "SSID");
            b.Pass(kk::wifi_pass, "Password");
            if (b.Button(kk::apply, "Connect")) {
                db.update();
                WiFiConnector.connect(db[kk::wifi_ssid], db[kk::wifi_pass]);
            }
            b.endGroup();
        }

        if (b.beginGroup("🕝 NTP")) {
            b.Input(kk::ntp_server, "NTP Server");
            b.Input(kk::ntp_offset, "Time Offset (GMT zone)");
            if (b.Button("Apply")) {
                db.update();
                NTP.setHost(db[kk::ntp_server]);
                NTP.setGMT(db[kk::ntp_offset]);
            }
            b.endGroup();
        }
        if (b.beginGroup("💧 Sensors")) {
            b.Number(kk::rattle_threshold, "Sensor Rattle Threshold (ms)");
            b.Number(kk::pump_on_duration, "Pump On Duration (ms)");
            b.Number(kk::pump_off_duration, "Pump Off Duration (ms)");
            if (b.Button("Apply")) {
                db.update();
                rattleThreshold = db[kk::rattle_threshold];
                pumpOnDuration = db[kk::pump_on_duration];
                pumpOffDuration = db[kk::pump_off_duration];
            }
            b.endGroup();
        }
    }
}

void setup() {
    for (byte i = 0; i < sizeof(sensors); i++) {
        pinMode(sensors[i].pin, INPUT);
        digitalWrite(sensors[i].pin, LOW);
    }
    Serial.begin(9600);
    lastState = 0;
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW);

    LittleFS.begin();
    db.begin();
    db.init(kk::wifi_ssid, "");
    db.init(kk::wifi_pass, "");
    db.init(kk::ntp_server, "pool.ntp.org");
    db.init(kk::ntp_offset, 0);
    db.init(kk::rattle_threshold, 1000);
    db.init(kk::pump_on_duration, 5000);
    db.init(kk::pump_off_duration, 10000);

    WiFiConnector.onConnect([]() {
        Serial.print("Connected! ");
        Serial.println(WiFi.localIP());
        Serial.print("DNS: ");
        Serial.println(WiFi.dnsIP());
        Serial.print("Gateway: ");
        Serial.println(WiFi.gatewayIP());
    });
    WiFiConnector.onError([]() {
        Serial.print("Unable to connet to WiFi! start AP ");
        Serial.println(WiFi.softAPIP());
    });
    WiFiConnector.connect(db[kk::wifi_ssid], db[kk::wifi_pass]);

    sett.begin();
    sett.onBuild(build);

    NTP.onError([]() {
        Serial.print("NTP Error: ");
        Serial.print(NTP.readError());
        Serial.print(", online: ");
        Serial.println(NTP.online() ? "yes" : "no");
    });

    NTP.begin(db[kk::ntp_offset]); 
    NTP.setPeriod(1 * 60 * 60);
    NTP.asyncMode(true);

    rattleThreshold = db[kk::rattle_threshold];
    pumpOnDuration = db[kk::pump_on_duration];
    pumpOffDuration = db[kk::pump_off_duration];
    pumpTimer.stop();
    debugTimer.start();
}

bool readSensors() {
    bool overallStateChanged = false;
    for (byte i = 0; i < numSensors; i++) {
        bool state = digitalRead(sensors[i].pin);
        if (sensors[i].state != state) {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(" state changed to ");
            Serial.println(state ? "HIGH" : "LOW");
            sensors[i].timer.start();
        }
        if (sensors[i].timer.overflow(rattleThreshold)) {
            sensors[i].state = state;
            overallStateChanged = true;
            if (state) {
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" triggered!");
            } else {
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" reset.");
            }
        }
    }
    return overallStateChanged;
}


void updatePumpState() {
    bool allTriggered = true;
    for (byte i = 0; i < numSensors; i++) {
        if (!sensors[i].state) {
            allTriggered = false;
            break;
        }
    };

    switch (pumpState)
    {
    case PUMP_OFF:
      if (allTriggered)
      {
        Serial.println("All sensors triggered, activating pump.");
        pumpState = PUMP_ON_WORKING;
        digitalWrite(relayPin, HIGH);
        pumpTimer.start();
      }
      break;
    case PUMP_ON_WORKING:
      if (pumpTimer.overflow(pumpOnDuration))
      {
        Serial.println("Pump working period ended.");
        pumpState = PUMP_ON_IDLE;
        digitalWrite(relayPin, LOW);
        pumpTimer.start();
      }
      break;
    case PUMP_ON_IDLE:
      if (pumpTimer.overflow(pumpOffDuration))
      {
        // Idle period ended. If water level is above second sensor - activate pump
        if (sensors[1].state)
        {
          Serial.println("Water level is above second sensor, activating pump.");
          pumpState = PUMP_ON_WORKING;
          digitalWrite(relayPin, HIGH);
          pumpTimer.start();
        }
        else
        {
          Serial.println("Water level is below safe threshold, turning off pump.");
          pumpState = PUMP_OFF;
          digitalWrite(relayPin, LOW);
          pumpTimer.stop();
        }
      }
      break;
    case PUMP_EMERGENCY_SHUTDOWN:
      if (sensors[0].state)
      {
        // Water level is above critical threshold. Disabling emergency shutdown.
        Serial.println("Water level is above critical threshold, disabling emergency shutdown.");
        pumpState = PUMP_OFF;
      }
    };

    if (!sensors[0].state && pumpState != PUMP_EMERGENCY_SHUTDOWN) {
        // Emergency shutdown if first sensor is not triggered
        Serial.println("Emergency shutdown activated, water level is below critical threshold.");
        pumpState = PUMP_EMERGENCY_SHUTDOWN;
        digitalWrite(relayPin, LOW);
        pumpTimer.stop();
    }
}

void loop() {
  WiFiConnector.tick();
  sett.tick();
  NTP.tick();
  if (readSensors()) {
      Serial.println("Sensors state changed, updating relay.");
  }
  updatePumpState();
  if (debugTimer.period(1000)) {
      Serial.print("Sensors state: ");
      for (byte i = 0; i < numSensors; i++) {
          Serial.print(sensors[i].state ? "1" : "0");
      }
      Serial.println();
  }
}

