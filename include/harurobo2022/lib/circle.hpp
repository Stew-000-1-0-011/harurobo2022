#pragma once

#include "vec2d.hpp"

namespace StewMath
{
    template<typename T>
    struct Circle
    {
        Vec2D<T> center{};
        T range{};

        constexpr bool is_in(const Vec2D<T>& point) const noexcept
        {
            return ++(point - center) < range * range;
        }
    };
}