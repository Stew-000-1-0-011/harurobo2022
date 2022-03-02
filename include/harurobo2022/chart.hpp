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

            collector_bottom,
            collector_step1,
            collector_step2,
            collector_step3,
            collector_shovel_open,
            collector_shovel_close,
            collector_tablecloth_push,
            collector_tablecloth_pull,

            change_to_over_fence
        };

        struct Command final
        {
            StewLib::Circle<float> pass_near_circle{};
            float target_rot_z{};
            Work work{Work::transit};
        };

        // スタートは入れずゴールを入れる。
        inline std::list<Command> chart
        {
            #include "../../others/chart.csv"
        };

        namespace ChartImplement
        {
            std::list<Command> make_trajectory(std::list<Command> chart) noexcept
            {
                chart.remove_if([](const Command& command) noexcept {return command.work != Work::transit;});
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

        inline std::list<Command> trajectory = ChartImplement::make_trajectory(ChartImplement::chart);
        inline std::list<std::list<Command>> steps = ChartImplement::make_steps(ChartImplement::chart);
    }
}
