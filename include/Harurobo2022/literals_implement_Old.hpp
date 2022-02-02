#pragma once

#include <cstddef>
#include <utility>
#include <type_traits>
#include <string>

// とりあえずは算術型にのみ対応。算術型なのでTをムーブしても意味がないと思う。
// (Unit自体はnoexceptなムーブコンストラクタがあると早くなるライブラリがありそうなので用意した。)
// とりあえずほとんどnoexceptにした。
// 算術型のコンストラクタ及びデストラクタはnoexceptだと信じる。代入もnoexceptだと信じる。
// 作業中盤でBoost.Operatorsなるものの存在を知った。もう遅い。
namespace QuantityUnit
{
    // template<
    //     int arg_m,
    //     int arg_s,
    //     int arg_kg,
    //     typename arg_T
    // >
    // class Unit;

    // template<int m, int s, int kg>
    // constexpr Unit<m, s, kg, unsigned long long int> make_unit(const unsigned long long int digits) noexcept;
    // template<int m, int s, int kg>
    // constexpr Unit<m, s, kg, long double> make_unit(const long double digits) noexcept;
    // template<int m, int s, int kg, typename TL, typename TR>
    // constexpr auto operator+(const Unit<m, s, kg, TL>& l, const Unit<m, s, kg, TR>& r) noexcept -> Unit<m, s, kg, decltype(l.value + r.value)>;

    template<class T>
    struct can_unitize
    {
        static constexpr bool value{std::is_arithmetic_v<std::remove_reference_t<std::remove_cv_t<T>>>};
    };

    template<class T>
    inline constexpr bool can_unitize_t = can_unitize<T>::value;

    template<
        int arg_m,
        int arg_s,
        int arg_kg,
        typename arg_T
    >
    class Unit final
    {
        using T = arg_T;
        static_assert(can_unitize_t<T>,"arg_T is not arithmetic.");

        template<int m2, int s2, int kg2, typename T2>
        friend
        struct Unit;

public:
        static constexpr int m{arg_m};
        static constexpr int s{arg_s};
        static constexpr int kg{arg_kg};

private:
        T value;

        constexpr Unit(const T value) noexcept:
            value(value)
        {}


        // = defaultで基本いい感じにしてくれるらしい。(C++20)多分絶対にinlineと可能ならconstexprはやってくれている。
        // noexceptがよくわからなかった。
        Unit() = default;
        Unit(const Unit&) = default;
        Unit(Unit&&) = default;
        Unit& operator=(const Unit&) = default;
        Unit& operator=(Unit&&) = default;
        ~Unit() = default;

        template<typename T2>
        constexpr Unit(const Unit<m, s, kg, T2>& obj) noexcept:
            value(obj.value)
        {}

        template<typename T2>
        constexpr Unit(Unit<m, s, kg, T2>&& obj) noexcept:
            value(obj.value)
        {}

        template<typename T2>
        constexpr Unit& operator=(const Unit<m, s, kg, T2>& obj) noexcept
        {
            value = obj.value;
            return *this;
        }

        template<typename T2>
        constexpr Unit& operator=(Unit<m, s, kg, T2>&& obj) noexcept
        {
            value = obj.value;
            return *this;
        }

        constexpr Unit operator+() const noexcept
        {
            return *this;
        }

        constexpr Unit operator-() const noexcept
        {
            return -value;
        }

        constexpr explicit operator bool() const noexcept
        {
            return value;
        }

        constexpr T get_primitive() const noexcept
        {
            return value;
        }

        // debug
        constexpr operator T() const noexcept
        {
            if constexpr(arg_m || arg_s || arg_kg)
            {
                static_assert([]{return false;}());
            }
            return value;
        }

        // 符号つき整数なら未定義動作、なしなら決まった値。浮動小数ならなんかいろいろ。ようするに例外はでない(はず)
        constexpr Unit& operator+=(const Unit& obj) noexcept
        {
            value += obj.value;
            return *this;
        }

        // まあ例外投げへんやろ！(5:30 AM)
        constexpr Unit& operator-=(const Unit& obj) noexcept
        {
            value -= obj.value;
            return *this;
        }

        std::string dim_str() const
        {
            return std::string("m:") + std::to_string(m) + ", s:" + std::to_string(s) + ", kg:" + std::to_string(kg) + "\n";
        }

    };

    // inline ⊂ constexpr (関数の場合)
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

    template<typename T>
    using Dim0_ = Unit<0, 0, 0, T>;

    // Dim0は算術型と一緒に演算できなきゃいけない
    template<typename TL, typename TR>
    constexpr auto operator+(const Dim0_<TL>& l, const TR r) noexcept -> Dim0_<decltype(std::enable_if_t<std::is_arithmetic_v<TR>>(), l.value + r)>
    {
        return l.value + r;
    }

    template<typename TL, typename TR>
    constexpr auto operator+(const TL l, const Dim0_<TR>& r) noexcept -> Dim0_<decltype(std::enable_if_t<std::is_arithmetic_v<TL>>(), l + r.value)>
    {
        return l + r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator-(const Dim0_<TL>& l, const TR r) noexcept -> Dim0_<decltype(std::enable_if_t<std::is_arithmetic_v<TR>>(), l. value - r)>
    {
        return l.value - r;
    }

    template<typename TL, typename TR>
    constexpr auto operator-(const TL l, const Dim0_<TR>& r) noexcept -> Dim0_<decltype(std::enable_if_t<std::is_arithmetic_v<TL>>(), l - r.value)>
    {
        return l - r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator*(const Dim0_<TL>& l, const TR r) noexcept -> Dim0_<decltype(std::enable_if_t<std::is_arithmetic_v<TR>>(), l. value * r)>
    {
        return l.value * r;
    }

    template<typename TL, typename TR>
    constexpr auto operator*(const TL l, const Dim0_<TR>& r) noexcept -> Dim0_<decltype(std::enable_if_t<std::is_arithmetic_v<TL>>(), l * r.value)>
    {
        return l * r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator/(const Dim0_<TL>& l, const TR r) noexcept -> Dim0_<decltype(std::enable_if_t<std::is_arithmetic_v<TR>>(), l. value / r)>
    {
        return l.value / r;
    }

    template<typename TL, typename TR>
    constexpr auto operator/(const TL l, const Dim0_<TR>& r) noexcept -> Dim0_<decltype(std::enable_if_t<std::is_arithmetic_v<TL>>(), l / r.value)>
    {
        return l / r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator<(const Dim0_<TL>& l, const TR r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TR>>::type(), bool())
    {
        return l.value < r;
    }

    template<typename TL, typename TR>
    constexpr auto operator<(const TL l, const Dim0_<TR>& r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TL>>::type(), bool())
    {
        return l < r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator>(const Dim0_<TL>& l, const TR r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TR>>::type(), bool())
    {
        return l.value > r;
    }

    template<typename TL, typename TR>
    constexpr auto operator>(const TL l, const Dim0_<TR>& r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TL>>::type(), bool())
    {
        return l > r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator<=(const Dim0_<TL>& l, const TR r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TR>>::type(), bool())
    {
        return l.value <= r;
    }

    template<typename TL, typename TR>
    constexpr auto operator<=(const TL l, const Dim0_<TR>& r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TL>>::type(), bool())
    {
        return l <= r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator>=(const Dim0_<TL>& l, const TR r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TR>>::type(), bool())
    {
        return l.value >= r;
    }

    template<typename TL, typename TR>
    constexpr auto operator>=(const TL l, const Dim0_<TR>& r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TL>>::type(), bool())
    {
        return l >= r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator==(const Dim0_<TL>& l, const TR r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TR>>::type(), bool())
    {
        return l.value == r;
    }

    template<typename TL, typename TR>
    constexpr auto operator==(const TL l, const Dim0_<TR>& r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TL>>::type(), bool())
    {
        return l == r.value;
    }

    template<typename TL, typename TR>
    constexpr auto operator!=(const Dim0_<TL>& l, const TR r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TR>>::type(), bool())
    {
        return l.value != r;
    }

    template<typename TL, typename TR>
    constexpr auto operator!=(const TL l, const Dim0_<TR>& r) noexcept -> decltype(std::enable_if_t<std::is_arithmetic_v<TL>>::type(), bool())
    {
        return l != r.value;
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


    // template<int m, int s, int kg>
    // constexpr Unit<m, s, kg, unsigned long long int> make_unit(const unsigned long long int digits) noexcept
    // {
    //     return digits;
    // }

    // template<int m, int s, int kg>
    // constexpr Unit<m, s, kg, long double> make_unit(const long double digits) noexcept
    // {
    //     return digits;
    // }

    namespace TypeCalc
    {
        template<template<typename TL> class UnitL, template<typename TR> class UnitR>
        struct mul
        {
            template<typename T>
            using temp = Unit<UnitL<T>::m + UnitR<T>::m, UnitL<T>::s + UnitR<T>::s, UnitL<T>::kg + UnitR<T>::kg, T>;
        };

        // template<template<typename TL> class UnitL, template<typename TR> class UnitR>
        // using mul_ = typename mul<UnitL,UnitR>::temp;

        template<template<typename TL> class UnitL, template<typename TR> class UnitR>
        struct div
        {
            template<typename T>
            using temp = Unit<UnitL<T>::m - UnitR<T>::m, UnitL<T>::s - UnitR<T>::s, UnitL<T>::kg - UnitR<T>::kg, T>;
        };

        // template<template<typename TL> class UnitL, template<typename TR> class UnitR>
        // using div_ = typename div<UnitL,UnitR>::temp;

        template<template<typename TL> class UnitL, int exp>
        struct pow
        {
            template<typename T>
            using temp = Unit<UnitL<T>::m * exp, UnitL<T>::s * exp, UnitL<T>::kg * exp, T>;
        };
    }

}

// #include <cmath>

// namespace std
// {
//     template<int m, int s, int kg, typename T>
//     constexpr auto sqrt(const QuantityUnit::Unit<m, s, kg, T>& x) noexcept
//     {
//         if constexpr(m % 2 || s % 2 || kg % 2)
//         {
//             static_assert([]{return false;}(),"Sqrt of this has non integer dim.");
//         }
        
//         return QuantityUnit::Unit<m / 2, s / 2, kg / 2, T>(std::sqrt(x.get_primitive()));
//     }

//     template<int m, int s, int kg, typename T>
//     constexpr double atan2(const QuantityUnit::Unit<m, s, kg, T>& y, const QuantityUnit::Unit<m, s, kg, T>& x) noexcept
//     {
//         return std::atan2(y.get_primitive(),x.get_primitive());
//     }

//     template<int m, int s, int kg, typename T>
//     constexpr auto abs(const QuantityUnit::Unit<m, s, kg, T>& x) noexcept
//     {
//         return std::abs(x.get_primitive());
//     }
// }