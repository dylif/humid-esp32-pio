#pragma once

#include "secrets.hpp"
#include "neopixel.hpp"

#include <cstdint>

static constexpr unsigned long serialBaud = 115200;
static constexpr uint8_t wifiConnectTimeoutSec = 30;
static constexpr uint8_t thingsBoardConnectTimeoutSec = 30;
static constexpr unsigned long measurePeriodMsec = 60000;
static constexpr unsigned long giveUpMsec = 180000;
static constexpr uint32_t startDelayMsec = 5000;

static_assert(measurePeriodMsec < giveUpMsec);

static constexpr NeoPixel::State neoPixelSendSuccess = {0x00FFFF, 64};  // Cyan
static constexpr NeoPixel::State neoPixelSendAttempt = {0xFFA500, 64};  // Amber
static constexpr NeoPixel::State neoPixelFatalError = {0xFF0000, 64};   // Red
