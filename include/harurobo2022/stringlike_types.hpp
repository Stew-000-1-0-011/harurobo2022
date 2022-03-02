#pragma once

#include "lib/macro/define_stringlike_type.hpp"

namespace Harurobo2022
{
    namespace
    {
        namespace StringlikeTypes
        {
            Stew_StringlikeType(under_carriage_4wheel)
            Stew_StringlikeType(state_manager)
            Stew_StringlikeType(auto_commander)
            Stew_StringlikeType(manual_commander)
            Stew_StringlikeType(can_tx)
            Stew_StringlikeType(shutdown)
            Stew_StringlikeType(state)
            Stew_StringlikeType(body_twist)
            Stew_StringlikeType(odometry)
            Stew_StringlikeType(FR_drive)
            Stew_StringlikeType(FL_drive)
            Stew_StringlikeType(BL_drive)
            Stew_StringlikeType(BR_drive)
            Stew_StringlikeType(FR_lift)
            Stew_StringlikeType(FL_lift)
            Stew_StringlikeType(BL_lift)
            Stew_StringlikeType(BR_lift)
            Stew_StringlikeType(subX_lift)
            Stew_StringlikeType(subY_lift)
            Stew_StringlikeType(collector_lift)
            Stew_StringlikeType(joy)
            Stew_StringlikeType(stepping_motor)
            Stew_StringlikeType(table_cloth)
            // グローバル名前空間でないのでいいはず。
            Stew_StringlikeType(_cmd)
            Stew_StringlikeType(_target)
        }
    }
}

#undef Stew_StringlikeType