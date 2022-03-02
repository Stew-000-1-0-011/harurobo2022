// #pragma once 故意

#include "../stringlike_type.hpp"

#define Stew_StringlikeType(name) \
\
struct name final : StewLib::StringlikeTypeImplement::StringlikeTypeBase\
{\
    constexpr static const char * str = #name;\
    constexpr static std::size_t size = sizeof(#name);\
};\

