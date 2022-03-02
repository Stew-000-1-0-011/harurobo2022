#pragma once

#include <cstdint>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>



namespace Harurobo2022
{
    namespace
    {
        namespace StdMsgsConvertorImplement
        {
            template<class RawData>
            struct StdMsgsConvertor;

#define Stew_define_StdMsgsConvertor(RawData, Message) \
        template<> \
        struct StdMsgsConvertor<RawData> final {using type = std_msgs::Message;};

            Stew_define_StdMsgsConvertor(bool, Bool)
            Stew_define_StdMsgsConvertor(float, Float32)
            Stew_define_StdMsgsConvertor(std::uint8_t, UInt8)

#undef Stew_define_StdMsgsConvertor
        }

        template<class RawData>
        using StdMsgsConvertor = StdMsgsConvertorImplement::StdMsgsConvertor<RawData>::type;
    }
}