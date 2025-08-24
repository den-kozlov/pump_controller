
#include <Arduino.h>
#include <GyverDBFile.h>
#include <LittleFS.h>
#include <GyverNTP.h>
#include <SettingsESP.h>
#include <WiFiConnector.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <GTimer.h>
#include "sensor.h"
#include "pump.h"
#include "pump_controller.h"
#include "debug_log.h"

GyverDBFile db(&LittleFS, "/data.db");
SettingsESP sett("WiFi config", &db);

DB_KEYS(kk, wifi_ssid, wifi_pass, ntp_server, ntp_offset, rattle_threshold, pump_on_duration, pump_off_duration, pump_active_duration, 
    pwm_freq, pwm_duty_cycle, apply);

const uint8_t relayPin = D7; // Relay pin for the pump
const uint8_t sensorPin = D1; // Pin for the sensor

Sensor *sensor;
Pump *pump;
PumpController *pump_controller;

const uint32_t NTP_UPDATE_PERIOD = 12 * 60 * 60; // 12 hours in seconds

void onWiFiConnect() {
    DEBUG_PRINT("Connected! ");
    DEBUG_PRINTLN(WiFi.localIP());
    DEBUG_PRINT("DNS: ");
    DEBUG_PRINTLN(WiFi.dnsIP());
    DEBUG_PRINT("Gateway: ");
    DEBUG_PRINTLN(WiFi.gatewayIP());
    if (!MDNS.begin("pump_controller")) {
        DEBUG_PRINTLN("Error starting mDNS");
    } else {
        DEBUG_PRINTLN("mDNS started: pump_controller.local");
    }
}

void build(sets::Builder& b) {
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

    if (b.beginGroup("⚙️ Settings")) {
        b.Number(kk::rattle_threshold, "Sensor Rattle Threshold (ms)");
        b.Number(kk::pump_on_duration, "Pump On Duration (sec)");
        b.Number(kk::pump_off_duration, "Pump Off Duration (sec)");
        b.Number(kk::pump_active_duration, "Pump Active Duration (sec)");
        if (b.Button("Apply")) {
            db.update();
            (static_cast<TriggerSensor*>(sensor))->setRattleThreshold(db[kk::rattle_threshold]);
            pump->setPumpOnDuration(db[kk::pump_on_duration]);
            pump->setPumpOffDuration(db[kk::pump_off_duration]);
            (static_cast<EmptyingPumpTimeoutController*>(pump_controller))->setActiveTime(db[kk::pump_active_duration]);
        }
        b.endGroup();
    }

    if (b.beginGroup("💧 State")) {
        b.LED("Sensor", sensor->getLevel() > Sensor::SENSOR_LEVEL_MIN);
        b.LED("Pump", pump->getState() != Pump::PUMP_OFF);
        b.endGroup();
    }
}

void setup() {
    Serial.begin(115200);

    LittleFS.begin();
    db.begin();
    db.init(kk::wifi_ssid, "");
    db.init(kk::wifi_pass, "");
    db.init(kk::ntp_server, "pool.ntp.org");
    db.init(kk::ntp_offset, 0);
    db.init(kk::rattle_threshold, TriggerSensor::DEFAULT_RATTLE_THRESHOLD);
    db.init(kk::pump_on_duration, Pump::DEFAULT_PUMP_ON_DURATION);
    db.init(kk::pump_off_duration, Pump::DEFAULT_PUMP_OFF_DURATION);
    db.init(kk::pump_active_duration, EmptyingPumpTimeoutController::DEFAULT_ACTIVE_TIME);

    WiFiConnector.onConnect(onWiFiConnect);
    WiFiConnector.onError([]() {
        DEBUG_PRINT("Unable to connet to WiFi! start AP ");
        DEBUG_PRINTLN(WiFi.softAPIP());
    });
    WiFiConnector.connect(db[kk::wifi_ssid], db[kk::wifi_pass]);

    sett.begin();
    sett.onBuild(build);

    NTP.onError([]() {
        DEBUG_PRINT("NTP Error: ");
        DEBUG_PRINT(NTP.readError());
        DEBUG_PRINT(", online: ");
        DEBUG_PRINTLN(NTP.online() ? "yes" : "no");
    });

    NTP.begin(db[kk::ntp_offset]);
    NTP.setPeriod(NTP_UPDATE_PERIOD);
    NTP.asyncMode(true);

    sensor = new TriggerSensor(sensorPin, db[kk::rattle_threshold]);
    pump = new RelayPump(relayPin, db[kk::pump_on_duration], db[kk::pump_off_duration]);

    pump_controller = new EmptyingPumpTimeoutController(sensor, pump, db[kk::pump_active_duration]);
}

void loop() {
    WiFiConnector.tick();
    sett.tick();
    NTP.tick();
    pump_controller->tick();
}