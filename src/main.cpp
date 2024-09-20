#include "config.hpp"
#include "neopixel.hpp"
#include "version.hpp"

#include <functional>

#include <Adafruit_SHT4x.h>
#include <Espressif_MQTT_Client.h>
#include <ThingsBoard.h>
#include <WiFi.h>
#include <Wire.h>

[[noreturn]] static void fatalError();
static bool getMeasurements(float &humidity, float &temperature);
static bool sendMeasurements(float humidity, float temperature);
static bool getAndSendMeasurements();
static bool sendCurrentVersion();
static bool waitForConditionWithTimeout(uint8_t timeoutSec,
                                        std::function<bool()> condition);
static bool connect();

static NeoPixel neopixel(neoPixelSendAttempt);
static Adafruit_SHT4x sht4x;
static Espressif_MQTT_Client mqttClient;
static ThingsBoard tb(mqttClient);

static unsigned long lastSendMsec;

void setup() {
    Serial.begin(serialBaud);
    Wire1.begin();
    neopixel.begin();

    // Delay before starting program to allow serial monitor to connect
    delay(startDelayMsec);

    if (!sht4x.begin(&Wire1)) {
        Serial.println("Failed to connect to SHT4x sensor!");
        fatalError();
    }
    if (!connect()) {
        fatalError();
    }
    if (!sendCurrentVersion()) {
        Serial.println("Failed to send version information!");
        fatalError();
    }
    if (!getAndSendMeasurements()) {
        fatalError();
    }

    // Indicate successful setup before starting main loop
    neopixel.set(neoPixelSendSuccess);
}

void loop() {
    // Espressif_MQTT_Client apparently uses its own task to send and receive
    // messages, so this isn't necessary but done for clarity and portability
    std::ignore = tb.loop();

    unsigned long delta = millis() - lastSendMsec;
    if (delta > giveUpMsec) {
        Serial.printf("Failed to send measurements in %lu milliseconds!\r\n",
                      giveUpMsec);
        fatalError();
    }

    auto periodExpired = delta > measurePeriodMsec;
    if (!periodExpired) {
        return;
    }

    neopixel.set(neoPixelSendAttempt);

    if (!connect()) {
        return;
    }

    if (!getAndSendMeasurements()) {
        return;
    }

    neopixel.set(neoPixelSendSuccess);
}

[[noreturn]] static void fatalError() {
    neopixel.set(neoPixelFatalError);
    Serial.println("Fatal error! Reset required!");
    while (true) {
        // Do nothing
    }
}

static bool getMeasurements(float &humidity, float &temperature) {
    sensors_event_t humidityEvent;
    sensors_event_t temperatureEvent;
    if (!sht4x.getEvent(&humidityEvent, &temperatureEvent)) {
        return false;
    }

    humidity = humidityEvent.relative_humidity;
    temperature = temperatureEvent.temperature;

    return true;
}

static bool sendMeasurements(float humidity, float temperature) {
    const auto telemetry = {
        Telemetry("humidity", humidity),
        Telemetry("temperature", temperature),
    };

    return tb.sendTelemetry(telemetry.begin(), telemetry.end());
}

static bool getAndSendMeasurements() {
    float humidity;
    float temperature;
    if (!getMeasurements(humidity, temperature)) {
        Serial.println("Failed to get measurements from SHT4x sensor!");
        return false;
    }

    if (!sendMeasurements(humidity, temperature)) {
        Serial.println("Failed to send measurements to ThingsBoard!");
        return false;
    }
    lastSendMsec = millis();
    Serial.printf("humidity = %05.2f, temperature = %05.2f\r\n",
                  humidity,
                  temperature);

    return true;
}

static bool sendCurrentVersion() {
    const auto telemetry = {
        Telemetry("version_major", currentVersion.major),
        Telemetry("version_minor", currentVersion.minor),
        Telemetry("version_patch", currentVersion.patch),
    };

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
    if (!waitForConditionWithTimeout(wifiConnectTimeoutSec,
                                     std::bind(&WiFiClass::isConnected,
                                               WiFi))) {
        Serial.println("Failed to connect to Wi-Fi!");
        return false;
    }

    Serial.println("Connecting to ThingsBoard...");
    tb.connect(thingsBoardHost, thingsBoardAccessToken);
    if (!waitForConditionWithTimeout(thingsBoardConnectTimeoutSec,
                                     std::bind(&ThingsBoard::connected, tb))) {
        Serial.println("Failed to connect to ThingsBoard!");
        return false;
    }

    return true;
}
