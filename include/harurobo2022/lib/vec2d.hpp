#pragma once

// GCC独自拡張前提(cmath関数のconstexpr化はよ)

#include <cmath>

namespace StewLib
{
    namespace
    {
        namespace Constant
        {
            inline constexpr double PI = 3.1415926535897932384626L;
            inline constexpr double PI2 = 2 * PI;
            inline constexpr double PI3 = 3 * PI;
            inline constexpr double PI_2 = PI / 2;
            inline constexpr double PI_4 = PI / 4;
            inline constexpr double PI_3 = PI / 3;
            inline constexpr double PI_6 = PI / 6;

            inline constexpr double ROOT_2 = 1.414'213'562'373'095'048L;
        }
    }
}

namespace StewLib
{
    namespace
    {
        // 使ってるcmathの関数全部をconstexprに実装するのは辛いので、もうGCC拡張で妥協することにした。
        // template<typename T>
        // constexpr T sqrt(const T x) noexcept
        // {
        //     if(!x) return 0;

        //     if(std::is_constant_evaluated())
        //     {
        //             T last_x = x / 2;
        //         T now_x = (x + 1) / 4;

        //         while(now_x != last_x && !(now_x - now_x))
        //         {
        //             last_x = now_x;
        //             now_x = (now_x +  x / now_x) / 2;
        //         }

        //         return now_x;
        //     }
        //     else
        //     {
        //         return std::sqrt(x);
        //     }
        // }


        /*
        要素が二つのベクトル。
        諸々の操作を行うとき、例外を吐いてはいけない。
        諸々の操作を行うとき、二つの要素の型は同じにならなければならない。
        */
        template<typename T>
        struct Vec2D final
        {
            T x;
            T y;

            constexpr Vec2D(const T x, const T y) noexcept:
                x(x),
                y(y)
            {}

            Vec2D() = default;
            Vec2D(const Vec2D&) = default;
            Vec2D& operator=(const Vec2D&) = default;
            Vec2D(Vec2D&&) = default;
            Vec2D& operator=(Vec2D&&) = default;
            ~Vec2D() = default;

            template<typename T2>
            constexpr Vec2D(const Vec2D<T2>& obj) noexcept:
                x(obj.x),
                y(obj.y)
            {}

            template<typename T2>
            constexpr Vec2D& operator=(const Vec2D<T2>& obj) noexcept
                {
                    x = obj.x;
                    y = obj.y;
                    return *this;
                }

            template<typename T2>
            constexpr Vec2D(Vec2D<T2>&& obj) noexcept:
                x(std::move(obj.x)),
                y(std::move(obj.y))
            {}

            template<typename T2>
            constexpr Vec2D& operator=(Vec2D<T2>&& obj) noexcept
            {
                x = std::move(obj.x);
                y = std::move(obj.y);
                return *this;
            }


            constexpr Vec2D& operator+=(const Vec2D& obj) noexcept
            {
                x += obj.x;
                y += obj.y;
                return *this;
            }

            constexpr Vec2D& operator-=(const Vec2D& obj) noexcept
            {
                x -= obj.x;
                y -= obj.y;
                return *this;
            }

            constexpr auto operator++() const noexcept  // ノルムの二乗
            {
                return x * x + y * y;
            }

            constexpr auto operator+() const noexcept  // ノルム
            {
                return std::sqrt(x * x + y * y);
            }

            constexpr auto operator-() const noexcept
            {
                return Vec2D<decltype(-x)>(-x, -y);
            }

            constexpr auto operator~() const noexcept
            {
                const auto norm = std::sqrt(x * x + y * y);
                return Vec2D<decltype(x / norm)>(x / norm, y / norm);
            }

            constexpr auto operator!() const noexcept  // π/2回転
            {
                return Vec2D<decltype(-y)>(-y, x);
            }

            constexpr auto get_angle() const noexcept // (1,0)となす角( [-π,π] )を返す
            {
                return std::atan2(y, x);
            }

        };

        template<typename TL, typename TR>
        constexpr inline auto operator+(const Vec2D<TL>& obj_l, const Vec2D<TR>& obj_r) noexcept -> Vec2D<decltype(obj_l.x + obj_r.x)>
        {
            return {obj_l.x + obj_r.x, obj_l.y + obj_r.y};
        }

        template<typename TL, typename TR>
        constexpr inline auto operator-(const Vec2D<TL>& obj_l, const Vec2D<TR>& obj_r) noexcept -> Vec2D<decltype(obj_l.x - obj_r.x)>
        {
            return {obj_l.x - obj_r.x, obj_l.y - obj_r.y};
        }

        template<typename TL, typename TR>
        constexpr inline auto operator*(const Vec2D<TL>& obj_l, const Vec2D<TR>& obj_r) noexcept
        {
            return obj_l.x * obj_r.x + obj_l.y * obj_r.y;
        }

        template<typename TL, typename TR>
        constexpr inline auto operator*(const TL obj_l, const Vec2D<TR>& obj_r) noexcept -> Vec2D<decltype(obj_l * obj_r.x)>
        {
            return {obj_l * obj_r.x, obj_l * obj_r.y};
        }

        template<typename TL, typename TR>
        constexpr inline auto operator*(const Vec2D<TL>& obj_l, const TR obj_r) noexcept -> Vec2D<decltype(obj_l.x * obj_r)>
        {
            return {obj_l.x * obj_r, obj_l.y * obj_r};
        }

        template<typename TL, typename TR>
        constexpr inline auto operator/(const Vec2D<TL>& obj_l, const TR obj_r) noexcept -> Vec2D<decltype(obj_l.x / obj_r)>
        {
            return {obj_l.x / obj_r, obj_l.y / obj_r};
        }

        template<typename TL, typename TR>
        constexpr inline auto operator/(const Vec2D<TL>& obj_l, const Vec2D<TR>& obj_r) noexcept
        {
            return obj_l.x * obj_r.y - obj_l.y * obj_r.x;
        }

        template<typename T>
        constexpr inline auto rot(const Vec2D<T>& obj, const double angle) noexcept
        {
            const double cos_angle = std::cos(angle);
            const double sin_angle = std::sin(angle);
            const auto tmp_x = obj.x * cos_angle - obj.y * sin_angle;
            const auto tmp_y = obj.x * sin_angle + obj.y * cos_angle;
            return Vec2D<decltype(tmp_x)>(tmp_x, tmp_y);
        }
    }
}