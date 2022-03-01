#pragma once

#include "lib/macro/define_stringlike_type.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace StringlikeTypes
        {
            Stew_StringlikeType(can_tx)
            Stew_StringlikeType(shutdown)
            Stew_StringlikeType(state)
            Stew_StringlikeType(body_twist)
            // グローバル名前空間でないのでいいはず。
            Stew_StringlikeType(_cmd)
            Stew_StringlikeType(_target)
        }
    }
}

#undef Stew_StringlikeType