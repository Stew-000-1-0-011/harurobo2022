#pragma once

#include <cstdint>
#include <cstring>

#include <ros/ros.h>
#include <harurobo2022/CanFrame.h>

#include <boost/array.hpp>

#include "topic_message_alias.hpp"

using namespace TopicMessageTypeAlias;
using namespace harurobo2022;

// // GCCだと認められるらしい(GNU拡張)？
// // C++標準では未定義動作？(Strict Alias Ruleわからない)
// // memcopy使うやつ(標準に適合)よりも速いか同等？
// // C99標準に適合するらしい(もちろんテンプレートなんてものはないが)
// // Cの構造体っぽいクラス(スタンダードレイアウト型ってやつなのか？)だけでもここどうにかしてくれないかな...でもパティングあるからダメなのかな
// template<class T>
// union Encapsulator final
// {
//     T value;
//     char arr[sizeof(T)];
// };
// GNU拡張をつかってもいいのかもしれないけど、ビルド時にどうやってGNU拡張を使うと指示すればいいのかわからなかったので、結局標準に適合するほうでやることに。
// 参考：https://yohhoy.hatenadiary.jp/entry/20120220/p1


// // 関数の部分特殊化できないのやっぱ変じゃない...？
// template <class T, bool larger_than_8byte = (sizeof(T) > 8)>
// struct Convertor final
// {
//     using Data = boost::array<std::uint8_t, 8>;
//     static constexpr std::size_t data_size = 1;

//     Convertor(const std::uint16_t id, const T src) noexcept
//     {
//         std::uint8_t dest[8]{};
//         std::memcpy(dest, &src, sizeof(T));

//         Data data;
//         for(int i = 0; i < 8; ++i)
//         {
//             data[i] = dest[8];
//         }

//         Canframe
//     }
// };

// template <class T>
// struct Convertor<T, true> final
// {
//     using Data = boost::array<std::uint8_t, 8>;
//     static constexpr std::size_t data_size = sizeof(T) / 8 + (sizeof(T) % 8)? 1 : 0;
//     Data data[data_size];

//     Convertor(const T& src) noexcept
//     {
//         std::uint8_t dest[8][data_size]{};
//         std::memcpy(dest, &src, sizeof(T));

//         for(int i = 0; i < data_size - 1; ++i) for(int j = 0; j < 8; ++j)
//         {
//             data[i][j] = dest[i][j];
//         }

//         for(int j = 0; j < sizeof(T) - data_size * 8; ++j)
//         {
//             data[data_size - 1][j] = dest[data_size - 1][j];
//         }
//     }
// };

namespace CanPubSub
{
    template<class T>
    auto can_publish(const ros::Publisher& can_tx_pub, const T& data) noexcept -> void
    {
        /* dataをCanFrameに変換して送信する。 */
    }
}