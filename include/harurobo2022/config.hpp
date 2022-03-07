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
                    inline constexpr StewLib::Vec2D<double> FR{99.3931, -290.65685};
                    inline constexpr StewLib::Vec2D<double> FL{FR.x, -FR.y};
                    inline constexpr StewLib::Vec2D<double> BL{-FR};
                    inline constexpr StewLib::Vec2D<double> BR{-FR.x, FR.y};

                    inline constexpr StewLib::Vec2D<double> all[4]{FR, FL, BL, BR};
                }

                namespace Direction
                {
                    inline constexpr StewLib::Vec2D<double> FR{-0.707106, -0.707106};
                    inline constexpr StewLib::Vec2D<double> FL{-0.707106, 0.707106};
                    inline constexpr StewLib::Vec2D<double> BL{-0.707106, -0.707106};
                    inline constexpr StewLib::Vec2D<double> BR{0.707106, -0.707106};

                    inline constexpr StewLib::Vec2D<double> all[4]{FR, FL, BL, BR};
                }
            }

            inline constexpr double body_radius{+Wheel::Pos::FR};
            inline constexpr double wheel_radius{30};

            inline constexpr double collector_bottom_position{/*TODO*/0};
            inline constexpr double collector_step1_position{/*TODO*/-7.5};
            inline constexpr double collector_step2_position{/*TODO*/-15};
            inline constexpr double collector_step3_position{/*TODO*/-22.5};

            inline constexpr double leg_bottom_position{/*TODO*/0};
            inline constexpr double leg_top_position{/*TODO*/0};

            namespace Limitation
            {
                // 0に設定すると制限がかからなくなる。
                inline constexpr double wheel_vela{/*TODO*/90};
                inline constexpr double wheel_acca{/*TODO*/90};
            
                inline constexpr double body_vell_ratio{/*TODO*/0.5};
                inline constexpr double body_vela_ratio{/*TODO*/0.5};

                inline constexpr double body_vell{wheel_vela * body_vell_ratio * wheel_radius};
                inline constexpr double body_vela{wheel_vela * body_vela_ratio * wheel_radius / body_radius};
            }


            namespace ExecutionInterval
            {
                inline constexpr double under_carriage_freq{1000};
                inline constexpr double manual_commander_freq{1000};
                inline constexpr double auto_commander_freq{1000};
            }

            namespace Pid
            {
                inline constexpr double position_k_p{/*TODO*/10};
                inline constexpr double position_k_i{/*TODO*/0};
                inline constexpr double position_k_d{/*TODO*/0};

                inline constexpr double rot_z_k_p{/*TODO*/10};
                inline constexpr double rot_z_k_i{/*TODO*/0};
                inline constexpr double rot_z_k_d{/*TODO*/0};
            }

            namespace InitialState
            {
                inline constexpr StewLib::Vec2D<double> position{/*TODO*/ 0, 0};
                inline constexpr double rot_z{/*TODO*/ -StewLib::Constant::PI / 4};
            }

            namespace CanId
            {
                namespace Tx
                {
                    namespace DriveMotor
                    {
                        inline constexpr std::uint16_t FR{0x610};
                        inline constexpr std::uint16_t FL{0x400};
                        inline constexpr std::uint16_t BL{0x448};
                        inline constexpr std::uint16_t BR{0x410};

                        inline constexpr std::uint16_t all[4]{FR, FL, BL, BR};
                    }

                    namespace LiftMotor
                    {
                        inline constexpr std::uint16_t FR{0x520};
                        inline constexpr std::uint16_t FL{0x540};
                        inline constexpr std::uint16_t BL{0x4C4};
                        inline constexpr std::uint16_t BR{0x430};

                        inline constexpr std::uint16_t subX{0x620};
                        inline constexpr std::uint16_t subY{0x440};

                        inline constexpr std::uint16_t collector{0x510};
                    }

                    inline constexpr std::uint16_t position_controll_ids[] = {LiftMotor::FR, LiftMotor::FL, LiftMotor::BL, LiftMotor::BR, LiftMotor::subX, LiftMotor::subY, LiftMotor::collector};
                    inline constexpr std::size_t position_controll_ids_size = sizeof(position_controll_ids) / sizeof(position_controll_ids[0]);

                    inline constexpr std::uint16_t stepping_motor{0x500};
                    inline constexpr std::uint16_t table_cloth_active{0x100};
                    inline constexpr std::uint16_t table_cloth_command{0x101};
                }

                namespace Rx
                {
                    inline constexpr std::uint16_t odometry_x{/*TODO*/0x205};
                    inline constexpr std::uint16_t odometry_y{/*TODO*/0x206};
                    inline constexpr std::uint16_t odometry_yaw{/*TODO*/0x207};
                }
            }
        }
    }
}