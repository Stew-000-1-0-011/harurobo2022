#pragma once

#include "literals_implement.hpp"


namespace QuantityUnit::Literals
{
    using TypeCalc::mul;
    using TypeCalc::div;
    using TypeCalc::pow;

    template<typename T>
    using Dim0 = Unit<0, 0, 0, T>;

    template<typename T>
    using M = Unit<1, 0, 0, T>;
    template<typename T>
    using S = Unit<0, 1, 0, T>;
    template<typename T>
    using Kg = Unit<0, 0, 1, T>;

    template<typename T>
    using Rad = Dim0<T>;

    template<typename T>
    using VelL = div<M, S>::temp<T>;

    template<typename T>
    using VelA = div<Rad, S>::temp<T>;

    template<typename T>
    using AccL = div<VelL, S>::temp<T>;

    template<typename T>
    using AccA = div<VelA, S>::temp<T>;

    template<typename T>
    using Hz = div<Rad, S>::temp<T>;
}


// なんでテンプレートが使えないんですか！？
#define define_quantity_unit_literal(UnitTemp) \
    \
constexpr QuantityUnit::Literals::UnitTemp<unsigned long long int> operator "" _##UnitTemp(const unsigned long long int digits) noexcept \
{ \
    return QuantityUnit::make_unit<QuantityUnit::Literals::UnitTemp<unsigned long long int>::m, QuantityUnit::Literals::UnitTemp<unsigned long long int>::s, QuantityUnit::Literals::UnitTemp<unsigned long long int>::kg>(digits); \
} \
\
constexpr QuantityUnit::Literals::UnitTemp<long double> operator "" _##UnitTemp(const long double digits) noexcept \
{ \
    return QuantityUnit::make_unit<QuantityUnit::Literals::UnitTemp<long double>::m, QuantityUnit::Literals::UnitTemp<long double>::s, QuantityUnit::Literals::UnitTemp<long double>::kg>(digits); \
}
// define end


// #define define_quantity_unit_literal(UnitTemp) \
//     \
// constexpr QuantityUnit::Literals::UnitTemp<unsigned long long int> operator "" _##UnitTemp(const unsigned long long int digits) noexcept \
// { \
//     return digits; \
// } \
// \
// constexpr QuantityUnit::Literals::UnitTemp<long double> operator "" _##UnitTemp(const long double digits) noexcept \
// { \
//     return digits; \
// }
// // define end

define_quantity_unit_literal(Dim0)
define_quantity_unit_literal(M)
define_quantity_unit_literal(S)
define_quantity_unit_literal(Kg)

define_quantity_unit_literal(Rad)
define_quantity_unit_literal(VelL)
define_quantity_unit_literal(VelA)
define_quantity_unit_literal(AccL)
define_quantity_unit_literal(AccA)
define_quantity_unit_literal(Hz)

#undef define_quantity_unit_literal