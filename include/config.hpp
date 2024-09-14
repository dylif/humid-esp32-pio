#pragma once

#include "secrets.hpp"

#include <cstdint>

static constexpr unsigned long serialBaud = 115200;

static constexpr uint16_t neoPixelCount = 1;
static constexpr uint8_t neoPixelBrightness = 128;
static constexpr uint32_t neoPixelColor = 0xFFA500; // Orange

static constexpr uint8_t wifiConnectTimeoutSec = 10;
static constexpr uint8_t thingsBoardConnectTimeoutSec = 10;
static constexpr unsigned long measurePeriodMsec = 5000;
static constexpr uint32_t startDelayMsec = 5000;
