// #pragma once 故意
#define Stew_define_has_member(member, func_name)\
\
namespace StewLib\
{\
    namespace\
    {\
\
        namespace HasMemberImplement\
        {\
            template<class T>\
            static constexpr auto has_##func_name(int) noexcept -> decltype(&T::member, bool())\
            {\
                return true;\
            }\
\
            template<class T>\
            static constexpr bool has_##func_name(double) noexcept\
            {\
                return false;\
            }\
        }\
\
        template<class T>\
        inline constexpr bool has_##func_name##_v = HasMemberImplement::has_##func_name<T>(0);\
    }\
}
