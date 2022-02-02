#pragma once

#include <cmath>
#include <utility>

namespace StewMath{

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


template<typename U>
struct sqrt_func final
{};

template<typename U>
struct atan2_func final
{};

template<typename U>
struct abs_func final
{};

template<typename T>
struct Vec2D final
{
    T x;
    T y;

    constexpr Vec2D(const T x,const T y) noexcept:
        x(x),
        y(y)
    {}

    Vec2D(const Vec2D&) = default;
    Vec2D& operator=(const Vec2D&) = default;
    Vec2D(Vec2D&&) = default;
    Vec2D& operator=(Vec2D&&) = default;
    Vec2D() = default;
    ~Vec2D() = default;

    template<typename U>
    constexpr Vec2D(const Vec2D<U>& obj) noexcept:
        x(obj.x),
        y(obj.y)
    {}

    template<typename U>
    constexpr Vec2D& operator=(const Vec2D<U>& obj) noexcept
        {
            x = obj.x;
            y = obj.y;
            return *this;
        }

    template<typename U>
    constexpr Vec2D(Vec2D<U>&& obj) noexcept:  // std::moveするよう変更
        x(std::move(obj.x)),
        y(std::move(obj.y))
    {}

    template<typename U>
    constexpr Vec2D& operator=(Vec2D<U>&& obj) noexcept  // std::moveするよう変更
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

    constexpr auto operator+() const noexcept // ノルムの二乗
    {
        return x*x + y*y;
    }

    constexpr auto operator-() const noexcept
    {
        return Vec2D<decltype(-x)>(-x,-y);
    }

    constexpr auto operator~() const noexcept
    {
        if constexpr (std::is_empty_v<sqrt_func<T>>)
        {
            const double norm = std::sqrt(x*x + y*y);
            return Vec2D<decltype(x/norm)>(x/norm,y/norm); // 多分、必ず有限数になる。
        }
        else
        {
            const auto norm = sqrt_func<T>::operator()(x*x + y*y);
            return Vec2D<decltype(x/norm)>(x/norm,y/norm);
        }

    }

    constexpr auto operator!() const noexcept // 他の人にばれたらきっと怒られる
    {
        return Vec2D<decltype(-y)>(-y,x);
    }

    // きっとxが少しでも0じゃなきゃ、有限数を返してくれる...。std::atan2を信じろ。
    // 角度にまつわる不具合が生じた場合はatan2のリファレンスをしっかり読むこと。
    constexpr auto getAngle() const noexcept // (1,0)となす角( [-π,π] )を返す
    {
        if constexpr (std::is_empty_v<atan2_func<T>>)
        {
            return std::atan2(y,x);
        }
        else
        {
            return atan2_func<T>::operator()(y, x);
        }
    }

};

template<typename T,typename U>
constexpr inline auto operator+(const Vec2D<T>& obj_r,const Vec2D<U>& obj_l) noexcept -> Vec2D<decltype(obj_r.x + obj_l.x)>
{
    return {obj_r.x + obj_l.x,obj_r.y + obj_l.y};
}

template<typename T,typename U>
constexpr inline auto operator-(const Vec2D<T>& obj_r,const Vec2D<U>& obj_l) noexcept -> Vec2D<decltype(obj_r.x - obj_l.x)>
{
    return {obj_r.x - obj_l.x,obj_r.y - obj_l.y};
}

template<typename T,typename U>
constexpr inline auto operator*(const Vec2D<T>& obj_r,const Vec2D<U>& obj_l) noexcept
{
    return obj_r.x*obj_l.x + obj_r.y*obj_l.y;
}

template<typename T,typename U>
constexpr inline auto operator*(const T obj_r,const Vec2D<U>& obj_l) noexcept -> Vec2D<decltype(obj_r * obj_l.x)>
{
    return {obj_r * obj_l.x,obj_r * obj_l.y};
}

template<typename T,typename U>
constexpr inline auto operator*(const Vec2D<T>& obj_r,const U obj_l) noexcept -> Vec2D<decltype(obj_r.x * obj_l)>
{
    return {obj_r.x * obj_l,obj_r.y * obj_l};
}

template<typename T,typename U>
constexpr inline auto operator/(const Vec2D<T>& obj_r,const U obj_l) noexcept -> Vec2D<decltype(obj_r.x / obj_l)>
{
    return {obj_r.x / obj_l,obj_r.y / obj_l};
}

template<typename T,typename U>
constexpr inline auto operator/(const Vec2D<T>& obj_r,const Vec2D<U>& obj_l) noexcept
{
    return obj_r.x*obj_l.y - obj_r.y*obj_l.x;
}

template<typename T>
constexpr inline auto rot(const Vec2D<T>& obj,const double angle) noexcept
{
    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);
    const auto tmp_x = obj.x * cos_angle - obj.y * sin_angle;
    const auto tmp_y = obj.x * sin_angle + obj.y * cos_angle;
    return Vec2D<decltype(tmp_x)>(tmp_x,tmp_y);
}

template<typename T>
constexpr inline auto abs(const Vec2D<T>& obj) noexcept
{
    if constexpr (std::is_empty_v<abs_func<T>>)
    {
        return Vec2D<decltype(std::abs(obj.x))>(std::abs(obj.x),std::abs(obj.y));
    }
    else
    {
        return Vec2D<decltype(abs_func(obj.x))>(abs_func<T>::operaor()(obj.x),abs_func<T>::operaor()(obj.y));
    }
}

}
