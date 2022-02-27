#pragma once


#define Stew_stringlikeType(name) \
\
namespace StewLib::StringlikeTypes\
{\
    namespace\
    {\
        struct name final : StewLib::StringlikeTypeImplement::StringlikeTypeBase\
        {\
            constexpr static const char * str = #name;\
        };\
    }\
}\

