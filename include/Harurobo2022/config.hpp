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
        }

        namespace Direction
        {
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> FR{/*TODO*/};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> FL{FR.x, -FR.y};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> BL{-FR};
            static constexpr StewMath::Vec2D<QuantityUnit::Literals::Dim0<double>> BR{-FR.x, FR.y};
        }
    }

    static constexpr QuantityUnit::Literals::M<double> body_radius{/*TODO*/};
    static constexpr QuantityUnit::Literals::M<double> wheel_radius{/*TODO*/};

    static constexpr QuantityUnit::Literals::VelL<double> body_vell_limit{/*TODO*/10_VelL};
    static constexpr QuantityUnit::Literals::AccL<double> body_accl_limit{/*TODO*/100_AccL};

    static constexpr QuantityUnit::Literals::Hz<double> under_carriage_freq{/*TODO*/20_Hz};
}