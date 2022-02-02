#pragma once

#include <cstddef>
#include <utility>
#include <type_traits>
#include <string>


namespace QuantityUnit
{
    template<class T>
    struct can_unitize
    {
        static constexpr bool value{std::is_arithmetic_v<T>};
    };

    template<class T>
    inline constexpr bool can_unitize_t = can_unitize<T>::value;

    /*
    物理の次元計算をコンパイル時に行う。
    +,-,比較,(複合)代入,変換は同じ次元同士で可能。*=,/=は不可能。Dim0 <-> primitive は不可能。
    *,/は任意の単位、primitive間で可能
    */
    template<
        int arg_m,
        int arg_s,
        int arg_kg,
        typename arg_T
    >
    struct Unit final
    {
        using T = std::remove_reference_t<std::remove_cv_t<arg_T>>;
        static_assert(can_unitize_t<T>,"arg_T is not arithmetic.");
        static constexpr int m{arg_m};
        static constexpr int s{arg_s};
        static constexpr int kg{arg_kg};

private:
        T value;

public:

        constexpr Unit(const T value) noexcept:
            value(value)
        {}

        Unit() = default;
        Unit(const Unit&) = default;
        Unit(Unit&&) = default;
        Unit& operator=(const Unit&) = default;
        Unit& operator=(Unit&&) = default;
        ~Unit() = default;

        template<typename T2>
        constexpr Unit(const Unit<m, s, kg, T2> obj) noexcept:
            value(obj.get_primitive())
        {}

        constexpr T get_primitive() const noexcept
        {
            return value;
        }

        constexpr Unit operator+() const noexcept
        {
            return *this;
        }

        constexpr Unit operator-() const noexcept
        {
            return -value;
        }

        constexpr Unit& operator+=(const Unit obj) noexcept
        {
            value += obj.value;
            return *this;
        }

        constexpr Unit& operator-=(const Unit obj) noexcept
        {
            value -= obj.value;
            return *this;
        }

        std::string dim_str() const
        {
            return std::string("m:") + std::to_string(m) + ", s:" + std::to_string(s) + ", kg:" + std::to_string(kg) + "\n";
        }

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr auto operator+(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(l.value + r.value)>;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr auto operator-(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(l.value - r.value)>;

        template<int m_l, int s_l, int kg_l, typename TL, int m_r, int s_r, int kg_r, typename TR>
        friend constexpr auto operator*(const Unit<m_l, s_l, kg_l, TL>& l, const Unit<m_r, s_r, kg_r, TR>& r) noexcept
            -> Unit<m_l + m_r, s_l + s_r, kg_l + kg_r, decltype(l.value * r.value)>;

        template<int m_l, int s_l, int kg_l, typename TL, int m_r, int s_r, int kg_r, typename TR>
        friend constexpr auto operator/(const Unit<m_l, s_l, kg_l, TL>& l, const Unit<m_r, s_r, kg_r, TR>& r) noexcept
            -> Unit<m_l - m_r, s_l - s_r, kg_l - kg_r, decltype(l.value / r.value)>;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr bool operator<(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr bool operator>(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr bool operator<=(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr bool operator>=(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr bool operator==(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr bool operator!=(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr auto operator*(const Unit<m, s, kg, TL>& l, const TR r) noexcept -> Unit<m, s, kg, decltype(std::enable_if_t<std::is_arithmetic_v<TR>>(), l.value * r)>;

        template<typename TL, int m, int s, int kg, typename TR>
        friend constexpr auto operator*(const TL l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(std::enable_if_t<std::is_arithmetic_v<TL>>(), l * r.value)>;

        template<int m, int s, int kg, typename TL, typename TR>
        friend constexpr auto operator/(const Unit<m, s, kg, TL>& l, const TR r) noexcept -> Unit<m, s, kg, decltype(std::enable_if_t<std::is_arithmetic_v<TR>>(), l.value / r)>;

        template<typename TL, int m, int s, int kg, typename TR>
        friend constexpr auto operator/(const TL l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(std::enable_if_t<std::is_arithmetic_v<TL>>(), l / r.value)>;

        template<int m, int s, int kg>
        friend constexpr Unit<m, s, kg, unsigned long long int> make_unit(const unsigned long long int digits) noexcept;

        template<int m, int s, int kg>
        friend constexpr Unit<m, s, kg, long double> make_unit(const long double digits) noexcept;
    };


    template<int m, int s, int kg, typename TL, typename TR>
    constexpr auto operator+(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(l.value + r.value)>
    {
        return l.value + r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr auto operator-(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(l.value - r.value)>
    {
        return l.value - r.value;
    }

    template<int m_l, int s_l, int kg_l, typename TL, int m_r, int s_r, int kg_r, typename TR>
    constexpr auto operator*(const Unit<m_l, s_l, kg_l, TL>& l, const Unit<m_r, s_r, kg_r, TR>& r) noexcept
        -> Unit<m_l + m_r, s_l + s_r, kg_l + kg_r, decltype(l.value * r.value)>
    {
        return l.value * r.value;
    }

    template<int m_l, int s_l, int kg_l, typename TL, int m_r, int s_r, int kg_r, typename TR>
    constexpr auto operator/(const Unit<m_l, s_l, kg_l, TL>& l, const Unit<m_r, s_r, kg_r, TR>& r) noexcept
        -> Unit<m_l - m_r, s_l - s_r, kg_l - kg_r, decltype(l.value / r.value)>
    {
        return l.value / r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr bool operator<(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept
    {
        return l.value < r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr bool operator>(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept
    {
        return l.value > r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr bool operator<=(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept
    {
        return l.value <= r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr bool operator>=(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept
    {
        return l.value >= r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr bool operator==(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept
    {
        return l.value == r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr bool operator!=(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept
    {
        return l.value != r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr auto operator*(const Unit<m, s, kg, TL>& l, const TR r) noexcept -> Unit<m, s, kg, decltype(std::enable_if_t<std::is_arithmetic_v<TR>>(), l.value * r)>
    {
        return l.value * r;
    }

    template<typename TL, int m, int s, int kg, typename TR>
    constexpr auto operator*(const TL l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(std::enable_if_t<std::is_arithmetic_v<TL>>(), l * r.value)>
    {
        return l * r.value;
    }

    template<int m, int s, int kg, typename TL, typename TR>
    constexpr auto operator/(const Unit<m, s, kg, TL>& l, const TR r) noexcept -> Unit<m, s, kg, decltype(std::enable_if_t<std::is_arithmetic_v<TR>>(), l.value / r)>
    {
        return l.value / r;
    }

    template<typename TL, int m, int s, int kg, typename TR>
    constexpr auto operator/(const TL l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(std::enable_if_t<std::is_arithmetic_v<TL>>(), l / r.value)>
    {
        return l / r.value;
    }


    namespace TypeCalc
    {
        template<template<typename TL> class UnitL, template<typename TR> class UnitR>
        struct mul
        {
            template<typename T>
            using temp = Unit<UnitL<T>::m + UnitR<T>::m, UnitL<T>::s + UnitR<T>::s, UnitL<T>::kg + UnitR<T>::kg, T>;
        };

        template<template<typename TL> class UnitL, template<typename TR> class UnitR>
        struct div
        {
            template<typename T>
            using temp = Unit<UnitL<T>::m - UnitR<T>::m, UnitL<T>::s - UnitR<T>::s, UnitL<T>::kg - UnitR<T>::kg, T>;
        };

        template<template<typename TL> class UnitL, int exp>
        struct pow
        {
            template<typename T>
            using temp = Unit<UnitL<T>::m * exp, UnitL<T>::s * exp, UnitL<T>::kg * exp, T>;
        };
    }

}

#include <cmath>
#include "vec2d.hpp"

namespace StewMath::Vec2DFunc
{
    template<int m, int s, int kg, typename T>
    struct op_bit_not_<QuantityUnit::Unit<m, s, kg, T>> final
    {
        template<int m_, int s_, int kg_, typename T_>
        using Unit = QuantityUnit::Unit<m_, s_, kg_, T_>;
        using UnitT = Unit<m, s, kg, T>;

        static constexpr Vec2D<Unit<0, 0, 0, T>> func(const UnitT x, const UnitT y) noexcept
        {
            const auto tmp_x = x.get_primitive();
            const auto tmp_y = y.get_primitive();
            const auto norm = std::sqrt(tmp_x * tmp_x + tmp_y * tmp_y);
            return {tmp_x / norm, tmp_y / norm};
        }
    };

    template<int m, int s, int kg, typename T>
    struct get_angle_<QuantityUnit::Unit<m, s, kg, T>> final
    {
        template<int m_, int s_, int kg_, typename T_>
        using Unit = QuantityUnit::Unit<m_, s_, kg_, T_>;
        using UnitT = Unit<m, s, kg, T>;

        static constexpr Unit<0, 0, 0, double> func(const UnitT x, const UnitT y) noexcept
        {
            return Unit<0, 0, 0, double>(std::atan2(y.get_primitive(), x.get_primitive()));
        }
    };

    template<int m, int s, int kg, typename T>
    struct abs_<QuantityUnit::Unit<m, s, kg, T>> final
    {
        template<int m_, int s_, int kg_, typename T_>
        using Unit = QuantityUnit::Unit<m_, s_, kg_, T_>;
        using UnitT = Unit<m, s, kg, T>;

        static constexpr Vec2D<UnitT> func(const UnitT x, const UnitT y) noexcept
        {
            return {std::abs(x.get_primitive()), std::abs(y.get_primitive())};
        }
    };
}