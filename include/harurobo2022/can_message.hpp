#pragma once

namespace Harurobo2022
{
    namespace
    {
        // コンセプト使えるようになったらCanMessageが満たす要件、
        // すなわちメンバ型としてMessageをもち、operator Message() const noexceptを持つことを表すやつを作る。

        template<class T>
        inline constexpr bool CanMessageC = true;

        // 雑だけども。だってメンバ型所持の判定やら書くのめんどいし...。
    }
}