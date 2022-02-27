#pragma once

#include <cstdint>

#include "lib/vec2d.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace Config
        {
            namespace Wheel  // 機体の中心に固定された座標における各ホイールの位置と正転の向き。
            {
                namespace Pos
                {
                    inline constexpr StewLib::Vec2D<double> FR{/*TODO*/265.51225, 139.42435};
                    inline constexpr StewLib::Vec2D<double> FL{-FR.x, FR.y};
                    inline constexpr StewLib::Vec2D<double> BL{-FR};
                    inline constexpr StewLib::Vec2D<double> BR{FR.x, -FR.y};

                    inline constexpr StewLib::Vec2D<double> all[4]{FR, FL, BL, BR};
                }

                namespace Direction
                {
                    inline constexpr StewLib::Vec2D<double> FR{/*TODO*/0.819152, -0.573576};
                    inline constexpr StewLib::Vec2D<double> FL{/*TODO*/0.819152, 0.573576};
                    inline constexpr StewLib::Vec2D<double> BL{/*TODO*/0.819152, -0.573576};
                    inline constexpr StewLib::Vec2D<double> BR{/*TODO*/0.819152, 0.573576};

                    inline constexpr StewLib::Vec2D<double> all[4]{FR, FL, BL, BR};
                }
            }

            inline constexpr double body_radius{/*TODO*/+Wheel::Pos::FR};
            inline constexpr double wheel_radius{/*TODO*/30};

            inline constexpr double collector_top_position{/*TODO*/};
            inline constexpr double collector_bottom_position{/*TODO*/};

            namespace Limitation
            {
                // 0に設定すると制限がかからなくなる。
                inline constexpr double wheel_vela{/*TODO*/40};
                inline constexpr double wheel_acca{/*TODO*/40};
            
                inline constexpr double body_vell_ratio{/*TODO*/0.5};
                inline constexpr double body_vela_ratio{/*TODO*/0.5};

                inline constexpr double body_vell{/*TODO*/wheel_vela * body_vell_ratio * wheel_radius};
                inline constexpr double body_vela{/*TODO*/wheel_vela * body_vela_ratio * wheel_radius / body_radius};
            }


            namespace ExecutionInterval
            {
                inline constexpr double under_carriage_freq{/*TODO*/1000};
                inline constexpr double manual_commander_freq{/*TODO*/1000};
                inline constexpr double auto_commander_freq{/*TODO*/1000};
                inline constexpr double not_emergency_signal_freq{/*TODO*/100};
            }

            namespace CanId
            {
                namespace Tx
                {
                    namespace DriveMotor
                    {
                        inline constexpr std::uint16_t FR{/*TODO*/0x450};
                        inline constexpr std::uint16_t FL{/*TODO*/0x444};
                        inline constexpr std::uint16_t BL{/*TODO*/0x458};
                        inline constexpr std::uint16_t BR{/*TODO*/0x440};

                        inline constexpr std::uint16_t all[4]{FR, FL, BL, BR};
                    }

                    namespace LiftMotor
                    {
                        inline constexpr std::uint16_t FR{/*TODO*/};
                        inline constexpr std::uint16_t FL{/*TODO*/};
                        inline constexpr std::uint16_t BL{/*TODO*/};
                        inline constexpr std::uint16_t BR{/*TODO*/};

                        inline constexpr std::uint16_t subX{/*TODO*/};
                        inline constexpr std::uint16_t subY{/*TODO*/};

                        inline constexpr std::uint16_t collect{/*TODO*/};
                    }

                    inline constexpr std::uint16_t position_controll_ids[] = {LiftMotor::FR, LiftMotor::FL, LiftMotor::BL, LiftMotor::BR, LiftMotor::subX, LiftMotor::subY, LiftMotor::collect};
                    inline constexpr std::size_t position_controll_ids_size = sizeof(position_controll_ids) / sizeof(position_controll_ids[0]);

                    namespace Emergency
                    {
                        inline constexpr std::uint16_t power{/*TODO*/0x0};
                    }
                }

                namespace Rx
                {
                    inline constexpr std::uint16_t odometry{/*TODO*/0x10};
                    inline constexpr std::uint16_t stopped{/*TODO*/0x8};
                }
            }
        }
    }
}