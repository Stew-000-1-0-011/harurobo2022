
#include <std_msgs/UInt32.h>

#include "harurobo2022/shirasu_publisher.hpp"

#include "harurobo2022/lib/macro/define_stringlike_type.hpp"
Stew_StringlikeType(motorFR)
#undef Stew_StringlikeType


using namespace Harurobo2022;

struct HogeNode final
{
    ShirasuPublisher<::motorFR, 444> motorFR{};
};

int main()
{
    HogeNode hoge;
    hoge.motorFR.send_cmd(ShirasuUtil::disable_mode);
}