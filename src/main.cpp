#include <Arduino.h>
#include <GyverDBFile.h>
#include <LittleFS.h>
#include <GyverNTP.h>
#include <SettingsESP.h>
#include <WiFiConnector.h>
#include <ESP8266WiFi.h>
#include <GTimer.h>
#include "sensor.h"
#include "pump.h"

GyverDBFile db(&LittleFS, "/data.db");
SettingsESP sett("WiFi config", &db);

DB_KEYS(kk, wifi_ssid, wifi_pass, ntp_server, ntp_offset, rattle_threshold, pump_on_duration, pump_off_duration, pump_active_duration, apply);

uint8_t lastState;
uint8_t relayPin = D7;

const uint8_t numSensors = 4;

Sensor sensor = Sensor(D1);
Pump pump = Pump(D2, &sensor);

// Sensor rattle threshold
uint16_t rattleThreshold;

// Pump timer
uint16_t pumpOnDuration;
uint16_t pumpOffDuration;
uint16_t pumpActiveDuration;
uTimer<millis> pumpTimer;

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
        if (b.beginGroup("⚙️ Sensor settings")) {
            b.Number(kk::rattle_threshold, "Sensor Rattle Threshold (ms)");
            b.Number(kk::pump_on_duration, "Pump On Duration (ms)");
            b.Number(kk::pump_off_duration, "Pump Off Duration (ms)");
            b.Number(kk::pump_active_duration, "Pump Active Duration (ms)");
            if (b.Button("Apply")) {
                db.update();
                rattleThreshold = db[kk::rattle_threshold];
                pumpOnDuration = db[kk::pump_on_duration];
                pumpOffDuration = db[kk::pump_off_duration];
                pumpActiveDuration = db[kk::pump_active_duration];
            }
            b.endGroup();
        }
        if (b.beginGroup("💧 Sensor states")) {
            b.LED("Sensor", sensor.getStatus());
            b.LED("Pump", pump.isActive());
            b.endGroup();
        }
    }
}

void setup() {
    Serial.begin(115200);
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
    db.init(kk::pump_active_duration, 1000 * 60 * 2);

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
    pumpActiveDuration = db[kk::pump_active_duration];
    pinMode(sensor.getPin(), INPUT);
    digitalWrite(sensor.getPin(), LOW);
    sensor.setRattleThreshold(rattleThreshold);
    pump.setPumpOnDuration(pumpOnDuration);
    pump.setPumpOffDuration(pumpOffDuration);
    pump.setPumpActiveDuration(pumpActiveDuration);
    pumpTimer.stop();
    debugTimer.start();
}

void loop() {
  WiFiConnector.tick();
  sett.tick();
  NTP.tick();
  sensor.tick();
  pump.tick();
}

