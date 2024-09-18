#pragma once

#include <cstdint>

struct Version {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
};

static constexpr Version currentVersion = { 0, 0, 0 };