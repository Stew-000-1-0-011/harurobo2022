#pragma once

#include <cstddef>
#include "enumerate.hpp"


// テンプレート引数にリテラル文字列を渡したいだけなのでこの程度の機能だけでいいかな
namespace CheapString
{
    template<std::size_t N>
    struct String final
    {
        char buffer[N]{};

        String() = default;
        String(const String&) = default;
        String(String&&) = default;
        ~String() = default;

private:
        template<size_t ... Numbers>
        constexpr String(const char *const str,StewEnumerate::Enumerate<Numbers ...>) noexcept:
            buffer{str[Numbers]...}
        {}

public:
        constexpr String(const char(&str)[N]) noexcept:
            String(str,typename StewEnumerate::EnumerateMake<N>::type())
        {}

        constexpr operator const char *() const noexcept
        {
            return buffer;
        }
    };
}