#pragma once

#include <cstdint>
#include <list>

#include "../include/harurobo2022/lib/vec2d.hpp"
#include "../include/harurobo2022/lib/circle.hpp"

#include "work.hpp"

namespace Harurobo2022
{
    enum Work : std::uint8_t
    {
        transit = 0,

        collector_up,
        collector_down,
        collector_shovel,
        collector_tablecloth,

        change_to_over_fence
    };

    struct Command final
    {
        StewMath::Circle<float> pass_near_circle{};
        Work work{transit};
    };

    namespace Chart::Implement
    {
        inline std::list<Command> chart
        {
            #include "../../others/chart.csv"
        };

        std::list<Command> make_trajectory(std::list<Command> chart) noexcept
        {
            chart.remove_if([](const Command& command){return command.work != transit;});
            return chart;
        }

        std::list<Command>
    }

    inline const std::list<Command> trajectory = Chart::Implement::make_trajectory(Chart::Implement::chart);
    inline const std::list<std::list<Command>> steps = 
}
