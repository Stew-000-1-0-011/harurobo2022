#pragma once

#define define_has_member(member, func_name) \
namespace MyUtility \
{ \
    namespace Implement \
    { \
        template<class T> \
        static constexpr auto has_##func_name(int) noexcept -> decltype(&T::member, bool()) \
        { \
            return true; \
        } \
    \
        template<class T> \
        static constexpr bool has_##func_name(double) noexcept \
        { \
            return false; \
        } \
    } \
     \
    template<class T> \
    inline constexpr bool has_##func_name##_v = Implement::has_##func_name<T>(0); \
}

