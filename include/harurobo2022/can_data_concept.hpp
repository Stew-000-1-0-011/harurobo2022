#pragma once

namespace Harurobo2022
{
    namespace
    {
        // コンセプト使えるようになったらCanDataが満たす要件、
        // すなわちメンバ型としてMessageをもち、operator Message() const noexceptを持ち、
        // can通信に流すデータを格納するメンバ変数dataをもつことを表明するやつを作る。

        template<class T>
        inline constexpr bool CanDataC = true;

        // 雑だけども。だってメンバ型所持の判定やら書くのめんどいし...。
    }
}