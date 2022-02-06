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


namespace CanPublish
{
    namespace Implement
    {
        template <class T, bool is_not_larger_than_8byte = !(sizeof(T) > 8)>
        struct Convertor final
        {
            using Data = boost::array<std::uint8_t, 8>;

            static constexpr std::size_t bytes_size = sizeof(T) / 8 - (sizeof(T) % 8)? 1 : 0;
            static constexpr std::uint8_t last_size = sizeof(T) % 8;

            Data bytes[bytes_size];
            Data last_byte;

            Convertor(const T& src) noexcept
            {
                // int[N]の参照がint(&)[N]であることを思い出す
                std::uint8_t dest[bytes_size + 1][8];
                std::memcpy(dest, &src, sizeof(T));

                for(int i = 0; i < bytes_size; ++i) for(int j = 0; j < 8; ++j)
                {
                    bytes[i][j] = dest[i][j];
                }

                for(int j = 0; j < last_size; ++j)
                {
                    last_byte[j] = dest[bytes_size][j];
                }
            }
        };

        // こっちのがよくつかわれる
        template <class T>
        struct Convertor<T, true> final
        {
            using Data = boost::array<std::uint8_t, 8>;

            static constexpr std::size_t bytes_size = 0;
            static constexpr std::uint8_t last_size = sizeof(T) % 8;

            Data last_byte;

            Convertor(const T& src) noexcept
            {
                // int[N]の参照がint(&)[N]であることを思い出す
                std::uint8_t dest[8];
                std::memcpy(dest, &src, sizeof(T));

                for(int j = 0; j < last_size; ++j)
                {
                    last_byte[j] = dest[j];
                }
            }
        };
    }

    template<class T>
    void can_publish(const ros::Publisher& can_tx_pub, const std::uint16_t id, const T& data) noexcept
    {
        using namespace Implement;

        CanFrame can_frame;
        auto conv = Convertor{data};

        if constexpr(Convertor<T>::bytes_size)
        {
            for(std::size_t i = 0; i < Convertor<T>::bytes_size; ++i)
            {
                can_frame.data = conv.bytes[i];
                can_frame.dlc = 8;
                can_frame.id= id;

                // ここはよくわからないので過去のコードを見てfalseにしている。
                can_frame.is_error = false;
                can_frame.is_extended = false;
                can_frame.is_rtr = false;

                can_tx_pub.publish(can_frame);
            }
        }

        can_frame.data = conv.last_byte;
        can_frame.dlc = Convertor<T>::last_size;
        can_frame.id= id;

        can_frame.is_error = false;
        can_frame.is_extended = false;
        can_frame.is_rtr = false;

        can_tx_pub.publish(can_frame);
    }
}