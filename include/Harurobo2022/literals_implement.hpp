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

        constexpr Unit(const T value) noexcept:
            value(value)
        {}

public:
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

    template<int m, int s, int kg>
    constexpr Unit<m, s, kg, unsigned long long int> make_unit(const unsigned long long int digits) noexcept
    {
        return digits;
    }

    template<int m, int s, int kg>
    constexpr Unit<m, s, kg, long double> make_unit(const long double digits) noexcept
    {
        return digits;
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