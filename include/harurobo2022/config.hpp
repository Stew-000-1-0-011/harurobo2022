#pragma once

#include <cstdint>

#include "vec2d.hpp"
#include "literals_config.hpp"

namespace Config
{
    namespace Wheel  // 機体の中心に固定された座標における各ホイールの位置と正転の向き。
    {
        namespace Pos
        {
            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> FR{/*TODO*/};
            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> FL{-FR.x, FR.y};
            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> BL{-FR};
            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> BR{FR.x, -FR.x};

            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> all[4]{FR, FL, BL, BR};
        }

        namespace Direction
        {
            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> FR{/*TODO*/};
            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> FL{FR.x, -FR.y};
            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> BL{-FR};
            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> BR{-FR.x, FR.y};

            inline constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> all[4]{FR, FL, BL, BR};
        }
    }

    inline constexpr QuantityUnit::Literals::M<double> body_radius{/*TODO*/};
    inline constexpr QuantityUnit::Literals::M<double> wheel_radius{/*TODO*/};

    inline constexpr QuantityUnit::Literals::VelA<double> wheel_vela_limit{/*TODO*/10};
    inline constexpr QuantityUnit::Literals::AccA<double> wheel_acca_limit{/*TODO*/100};

    inline constexpr QuantityUnit::Literals::Hz<double> under_carriage_freq{/*TODO*/20};

    namespace CanId
    {
        namespace DriveMotor
        {
            inline constexpr std::uint16_t FR{/*TODO*/};
            inline constexpr std::uint16_t FL{/*TODO*/};
            inline constexpr std::uint16_t BL{/*TODO*/};
            inline constexpr std::uint16_t BR{/*TODO*/};

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
        }

        inline constexpr std::uint16_t collect{/*TODO*/};
        // まだ数枚ある
    }
}
