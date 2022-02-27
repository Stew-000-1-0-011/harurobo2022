#pragma once

#include <cstdint>
#include <list>

#include "../include/harurobo2022/lib/vec2d.hpp"
#include "../include/harurobo2022/lib/circle.hpp"


namespace Harurobo2022
{
    namespace
    {

    enum class Work : std::uint8_t
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
        Work work{Work::transit};
    };

    namespace Chart::Implement
    {
        inline std::list<Command> chart
        {
            #include "../../others/chart.csv"
        };

        std::list<Command> make_trajectory(std::list<Command> chart) noexcept
        {
            chart.remove_if([](const Command& command){return command.work != Work::transit;});
            return chart;
        }

        std::list<std::list<Command>> make_steps(const std::list<Command>& chart) noexcept
        {
            std::list<std::list<Command>> steps;
            steps.emplace_back();
            auto part_steps = steps.end();
            --part_steps;

            for(const auto& command: chart)
            {
                if(command.work == Work::transit)
                {
                    steps.emplace_back();
                    ++part_steps;
                }

                part_steps->emplace_back(command);
            }

            return steps;
        }
    }

    inline const std::list<Command> trajectory = Chart::Implement::make_trajectory(Chart::Implement::chart);
    inline std::list<std::list<Command>> steps = Chart::Implement::make_steps(Chart::Implement::chart);
}
