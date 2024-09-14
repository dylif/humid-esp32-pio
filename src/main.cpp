#include "config.hpp"

#include <array>

#include <Adafruit_NeoPixel.h>
#include <Adafruit_SHT4x.h>
#include <Espressif_MQTT_Client.h>
#include <ThingsBoard.h>
#include <WiFi.h>
#include <Wire.h>

static void giveUp();
static void setNeoPixel();
static bool getAndSendMeasurements();
static bool connect();

static Adafruit_NeoPixel pixel(neoPixelCount, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static Adafruit_SHT4x sht4x;
static Espressif_MQTT_Client mqttClient;
static ThingsBoard tb(mqttClient);

static unsigned long lastMeasureMsec = 0;

void setup() {
    Serial.begin(serialBaud);

    setNeoPixel();

    // Delay before starting program to allow serial monitor to connect
    delay(startDelayMsec);

    setupSht4x();
}

void loop() {
    if (connect() && getAndSendMeasurements()) {
        lastMeasureMsec = millis();
    }

    tb.loop();
}

void giveUp() {
    while (true) {
        // Do nothing
    }
}

void setNeoPixel() {
#if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

    pixel.begin();
    pixel.setBrightness(neoPixelBrightness);

    // Set NeoPixel to orange
    pixel.fill(neoPixelColor);
    pixel.show();
}

void setupSht4x() {
    Wire1.begin();
    if (!sht4x.begin(&Wire1)) {
        Serial.println("Failed to connect to SHT4x sensor!");
        giveUp();
    }
    Serial.println("Connected to SHT4x sensor.");
}

bool getAndSendMeasurements() {
    if ((millis() - lastMeasureMsec) < measurePeriodMsec) {
        return false;
    }

    sensors_event_t humidityEvent;
    sensors_event_t temperatureEvent;
    if (!sht4x.getEvent(&humidityEvent, &temperatureEvent)) {
        Serial.println("Failed to get measurements from SHT4x sensor!");
        return false;
    }
    float humidity = humidityEvent.relative_humidity;
    float temperature = temperatureEvent.temperature;
    
    Serial.printf("Read measurements: humidity = %05.2f, temperature = %05.2f\r\n", humidity, temperature);
    auto telemetry = {Telemetry("humidity", humidity), Telemetry("temperature", temperature)};

    return tb.sendTelemetry(telemetry.begin(), telemetry.end());
}

bool connect() {
    if (tb.connected()) {
        return true;
    }

    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(wifiSsid, wifiPass);
    for (uint8_t i = 0; i < wifiConnectTimeoutSec; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            break;
        }
        delay(1000);
        Serial.printf("%2u/%2u seconds elapsed\r\n", i + 1, wifiConnectTimeoutSec);
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to Wi-Fi!");
        return false;
    }
    Serial.println("Connected to Wi-Fi.");

    Serial.println("Connecting to ThingsBoard...");
    for (uint8_t i = 0; i < thingsBoardConnectTimeoutSec; i++) {
        if (tb.connect(thingsBoardHost, thingsBoardAccessToken)) {
            break;
        }

        delay(1000);
        Serial.printf("%2u/%2u seconds elapsed\r\n", i + 1, thingsBoardConnectTimeoutSec);
    }
    if (!tb.connected()) {
        Serial.println("Failed to connect to ThingsBoard!");
        return false;
    }
    Serial.println("Connected to ThingsBoard.");

    return true;
}
