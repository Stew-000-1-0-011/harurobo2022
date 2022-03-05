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

            change_to_over_fence,

            game_clear
        };

        struct Command final
        {
            StewLib::Circle<float> pass_near_circle;
            float target_rot_z;
            Work work{Work::transit};
        };

        namespace StaticChart
        {
            // スタートは入れずゴールを入れる。
            inline std::list<Command> chart1
            {
                #include "../../others/chart.cpp"
            };
        }

        struct ChartManager final
        {
            // debug
            std::list<Command> chart{StaticChart::chart1};

            void reset_chart() noexcept
            {
                /* TODO 動的な軌道変更*/
            }

            // 目標姿勢角と直近の仕事
            std::list<Command>::const_iterator current_work;
            // 目標位置
            std::list<Command>::const_iterator target_position;

            void current_work_update() noexcept
            {
                ++current_work;
            }

            void target_position_update() noexcept
            {
                auto iter = target_position;
                for(;iter != chart.cend(); ++iter)
                {
                    if(iter->work == Work::transit)
                    {
                        target_position = iter;
                        current_work = iter;
                        break;
                    }
                }
            }
        };

    //     namespace ChartImplement
    //     {
    //         std::list<Command> make_trajectory(std::list<Command> chart) noexcept
    //         {
    //             chart.remove_if([](const Command& command) noexcept {return command.work != Work::transit;});
    //             return chart;
    //         }

    //         std::list<std::list<Command>> make_steps(const std::list<Command>& chart) noexcept
    //         {
    //             std::list<std::list<Command>> steps;
    //             steps.emplace_back();
    //             auto part_steps = steps.end();
    //             --part_steps;

    //             for(const auto& command: chart)
    //             {
    //                 if(command.work == Work::transit)
    //                 {
    //                     steps.emplace_back();
    //                     ++part_steps;
    //                 }

    //                 part_steps->emplace_back(command);
    //             }

    //             return steps;
    //         }
    //     }

    //     inline std::list<Command> trajectory = ChartImplement::make_trajectory(ChartImplement::chart);
    //     inline std::list<std::list<Command>> steps = ChartImplement::make_steps(ChartImplement::chart);
    }
}
