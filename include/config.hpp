#pragma once

#include "secrets.hpp"
#include "neopixel.hpp"

#include <cstdint>

static constexpr unsigned long serialBaud = 115200;
static constexpr uint8_t wifiConnectTimeoutSec = 10;
static constexpr uint8_t thingsBoardConnectTimeoutSec = 10;
static constexpr unsigned long measurePeriodMsec = 5000;
static constexpr unsigned long measureGiveUpMsec = 30000;
static constexpr uint32_t startDelayMsec = 5000;

static_assert(measurePeriodMsec < measureGiveUpMsec);

static constexpr NeoPixel::State neoPixelNormal = {0x00FFFF, 64};  // Cyan
static constexpr NeoPixel::State neoPixelTrying = {0xFFA500, 64};  // Orange
static constexpr NeoPixel::State neoPixelHalted = {0xFF0000, 64};  // Red
