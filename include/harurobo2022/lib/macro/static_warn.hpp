#pragma once

#ifndef __COUNTER__
static_assert(false, "This environment don't have __COUNTER__.");
#endif

#include <type_traits>

#define Stew_concat_inner(a, b) a##b
#define Stew_concat(a, b) Stew_concat_inner(a, b)


#define Stew_static_warn(cond, sentence) \
struct Stew_concat(Stew_static_warn_class_, __COUNTER__)\
{\
    [[deprecated(sentence)]] constexpr static int helper(std::false_type) noexcept {return 0;}\
    constexpr static int helper(std::true_type) noexcept {return 0;}\
    int dummy = helper(std::bool_constant<cond>());\
}
