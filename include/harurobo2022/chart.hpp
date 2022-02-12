#pragma once

#include <cstdint>
#include <vector>

#include "lib/vec2d.hpp"
#include "lib/circle.hpp"

namespace Harurobo2022
{
    enum Work : std::uint8_t
    {
        transit,
        change_to_manual,
        change_to_auto,
    };

    struct Command final
    {
        StewMath::Circle<float> pass_near_circle{};
        Work work{transit};
    };

    inline std::vector<Command> chart
    {
        #include "../../others/chart.csv"
    };
}
