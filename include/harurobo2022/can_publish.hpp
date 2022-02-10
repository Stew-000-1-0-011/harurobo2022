#pragma once

#include <cstdint>
#include <cstring>

#include <ros/ros.h>
#include <harurobo2022/CanFrame.h>

#include <boost/array.hpp>

#include "raw_data/raw_data_all.hpp"
#include "topics.hpp"

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


namespace Harurobo2022
{
    namespace CanPublish::Implement
    {
        template <class Message, bool is_not_larger_than_8byte = !(sizeof(RawData<Message>) > 8)>
        struct Convertor final
        {
            using RawData = Harurobo2022::RawData<Message>;
            using Array = boost::array<std::uint8_t, 8>;

            static constexpr std::size_t bytes_size = sizeof(T) / 8 - (sizeof(T) % 8)? 1 : 0;
            static constexpr std::uint8_t last_size = sizeof(T) % 8;

            Array bytes[bytes_size];
            Array last_byte;

            Convertor(const RawData& src) noexcept
            {
                for(size_t i = 0; i < bytes_size; ++i)
                {
                    std::memcpy(&bytes[i][0], (char *)&src + , 8);
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

    template<class Topic>
    struct CanPublisher final
    {
        const ros::Publisher& can_tx_pub;
        
        CanPublisher(ros::Publisher& can_tx_pub) noexcept:
            can_tx_pub(can_tx_pub)
        {}

        inline void publish(const Topic::Message& msg) const noexcept
        {
            
        }
    };
}