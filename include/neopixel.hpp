#pragma once

#include <cstdint>
#include <optional>

#include <Adafruit_NeoPixel.h>

class NeoPixel {
public:
    struct State {
        uint32_t color;
        uint8_t brightness;

        bool operator==(const State &rhs) const {
            return color == rhs.color && brightness == rhs.brightness;
        }

        bool operator!=(const State &rhs) const {
            return !(*this == rhs);
        }
    };

    NeoPixel() = delete;
    NeoPixel(const State &initial):
        pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800), current{initial} {}

    void begin() {
#if defined(NEOPIXEL_POWER)
        pinMode(NEOPIXEL_POWER, OUTPUT);
        digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
        pixel.begin();

        setUnchecked(current);
    }

    void set(const State &state) {
        if (state != current) {
            setUnchecked(state);
        }
    }

private:
    void setUnchecked(const State &state) {
        pixel.setBrightness(state.brightness);
        pixel.fill(state.color);
        pixel.show();
    }

    Adafruit_NeoPixel pixel;
    State current;
};
