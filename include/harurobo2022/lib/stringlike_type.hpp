#pragma once

#include <cstddef>
#include <cstring>
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

        template<class StringlikeTypeL, class StringlikeTypeR>
        struct Concat final : StringlikeTypeImplement::StringlikeTypeBase
        {
            constexpr static std::size_t size = StringlikeTypeL::size + StringlikeTypeR::size;
            inline static char str[size];
            inline static const char dummy =
                []() noexcept
                {
                    for(std::size_t i = 0; i < StringlikeTypeL::size; ++i)
                    {
                        str[i] = StringlikeTypeL::str[i];
                    }

                    for(std::size_t i = 0; i < StringlikeTypeR::size; ++i)
                    {
                        str[StringlikeTypeL::size + i] = StringlikeTypeR::str[i];
                    }
                    
                    return 0;
                }();
        };
    }
}