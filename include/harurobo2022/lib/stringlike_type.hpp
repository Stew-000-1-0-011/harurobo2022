#pragma once

#include <type_traits>

namespace StewLib
{
    namespace
    {
        namespace StringlikeTypeImplement
        {
            struct StringlikeTypeBase{};
        }

        template<class T>
        using is_stringlike_type = std::is_base_of<StringlikeTypeImplement::StringlikeTypeBase, T>;
        template<class T>
        inline constexpr bool is_stringlike_type_v = std::is_base_of_v<StringlikeTypeImplement::StringlikeTypeBase, T>;
    }
}