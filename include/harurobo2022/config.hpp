#pragma once

#include "vec2d.hpp"
#include "literals_config.hpp"

namespace Config
{
    namespace Wheel  // 機体の中心に固定された座標における各ホイールの位置と正転の向き。
    {
        namespace Pos
        {
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> FR{/*TODO*/};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> FL{-FR.x, FR.y};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> BL{-FR};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> BR{FR.x, -FR.x};

            static constexpr StewMath::Vec2D<QuantityUnit::Literals::M<double>> all[4]{FR, FL, BL, BR};
        }

        namespace Direction
        {
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> FR{/*TODO*/};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> FL{FR.x, -FR.y};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> BL{-FR};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> BR{-FR.x, FR.y};

            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> all[4]{FR, FL, BL, BR};
        }
    }

    static constexpr QuantityUnit::Literals::M<double> body_radius{/*TODO*/};
    static constexpr QuantityUnit::Literals::M<double> wheel_radius{/*TODO*/};

    static constexpr QuantityUnit::Literals::VelA<double> wheel_vela_limit{/*TODO*/10};
    static constexpr QuantityUnit::Literals::AccA<double> wheel_acca_limit{/*TODO*/100};

    static constexpr QuantityUnit::Literals::Hz<double> under_carriage_freq{/*TODO*/20};
}
