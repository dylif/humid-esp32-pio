#include "config.hpp"
#include "neopixel.hpp"

#include <array>
#include <functional>

#include <Adafruit_NeoPixel.h>
#include <Adafruit_SHT4x.h>
#include <Espressif_MQTT_Client.h>
#include <ThingsBoard.h>
#include <WiFi.h>
#include <Wire.h>

static void giveUp();
static bool getAndSendMeasurements();
static bool waitForConditionWithTimeout(uint8_t timeoutSec,
                                        std::function<bool()> condition);
static bool connect();

static NeoPixel neopixel(neoPixelTrying);
static Adafruit_SHT4x sht4x;
static Espressif_MQTT_Client mqttClient;
static ThingsBoard tb(mqttClient);

static unsigned long lastMeasureMsec;

void setup() {
    Serial.begin(serialBaud);

    neopixel.begin();

    // Delay before starting program to allow serial monitor to connect
    delay(startDelayMsec);

    // Setup SHT4x sensor
    Wire1.begin();
    if (!sht4x.begin(&Wire1)) {
        Serial.println("Failed to connect to SHT4x sensor!");
        giveUp();
    }
    Serial.println("Connected to SHT4x sensor.");

    lastMeasureMsec = millis();
}

void loop() {
    tb.loop();

    if (connect()) {
        neopixel.set(neoPixelNormal);
        if (getAndSendMeasurements()) {
            lastMeasureMsec = millis();
        }
    } else {
        neopixel.set(neoPixelTrying);
    }
}

static void giveUp() {
    neopixel.set(neoPixelHalted);
    Serial.println("Fatal error!");
    while (true) {
        // Do nothing
    }
}

static bool getAndSendMeasurements() {
    unsigned long delta = millis() - lastMeasureMsec;
    if (delta < measurePeriodMsec) {
        return false;
    } else if (delta > measureGiveUpMsec) {
        giveUp();
    }

    sensors_event_t humidityEvent;
    sensors_event_t temperatureEvent;
    if (!sht4x.getEvent(&humidityEvent, &temperatureEvent)) {
        Serial.println("Failed to get measurements from SHT4x sensor!");
        return false;
    }
    const float humidity = humidityEvent.relative_humidity;
    const float temperature = temperatureEvent.temperature;
    
    Serial.printf("delta = %lu, humidity = %05.2f, temperature = %05.2f\r\n",
                  delta,
                  humidity,
                  temperature);

    const auto telemetry = {Telemetry("humidity", humidity),
                            Telemetry("temperature", temperature)};

    return tb.sendTelemetry(telemetry.begin(), telemetry.end());
}

static bool waitForConditionWithTimeout(uint8_t timeoutSec,
                                        std::function<bool()> condition) {
    for (uint8_t i = 0; i < timeoutSec; i++) {
        if (condition()) {
            break;
        }
        delay(1000);
        Serial.printf("%02u/%02u seconds elapsed...\r\n", i + 1, timeoutSec);
    }
    
    return condition();
}

static bool connect() {
    if (tb.connected()) {
        return true;
    }

    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(wifiSsid, wifiPass);
    if (!waitForConditionWithTimeout(
            wifiConnectTimeoutSec,
            std::bind(&WiFiClass::isConnected, WiFi))) {
        Serial.println("Failed to connect to Wi-Fi!");
        return false;
    }
    Serial.println("Connected to Wi-Fi.");

    Serial.println("Connecting to ThingsBoard...");
    tb.connect(thingsBoardHost, thingsBoardAccessToken);
    if (!waitForConditionWithTimeout(
            thingsBoardConnectTimeoutSec,
            std::bind(&ThingsBoard::connected, tb))) {
        Serial.println("Failed to connect to ThingsBoard!");
        return false;
    }
    Serial.println("Connected to ThingsBoard.");

    return true;
}
