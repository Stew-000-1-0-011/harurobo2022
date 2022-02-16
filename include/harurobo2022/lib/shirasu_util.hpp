#pragma once

#include <cstdint>

namespace ShirasuUtil
{
    enum Mode: std::uint8_t
    {
        disable_mode = 0,
        default_mode,
        homing_mode,
        reserved,
        current,
        velocity,
        position
    };

    inline constexpr std::uint16_t target_id(const std::uint16_t base_id) noexcept
    {
        return base_id + 1;
    }
}