#pragma once

#include <cstring>

#include <array>



namespace StewLib
{
    namespace
    {

        // 標準レイアウト型にしよう。アクセス指定子は先頭に一つだけとすること。
        // 標準レイアウト型だと多分こいつをそのままstd::memcpyにぶち込める気がするけど、よくわからなかった。
        // https://timsong-cpp.github.io/cppwp/n4861/class.mem#22
        // std::memcpyしていい条件ってなに...？
        template<class RawData_, bool is_larger_than_8byte_ = (sizeof(RawData_) > 8)>
        struct Serialize
        {
            using RawData = RawData_;
            static constexpr bool is_larger_than_8byte = is_larger_than_8byte_;
            
        // private:
        //     using Chunk = std::array<std::uint8_t, 8>;

        // public:
            static constexpr std::uint8_t last_size = sizeof(RawData);
            static constexpr std::size_t chunks_size = 0;

            // これだけで未定義動作でないと保証できるわけじゃないけど、無いよりマシだろう。
            static_assert(sizeof(Serialize) - (last_size % 8) == sizeof(RawData_), "Oh, my Guess is off.");

            // Chunk last_chunk;
            std::uint8_t last_chunk[last_size]{};

            Serialize(const RawData& raw_data)
            {
                std::memcpy(this, &raw_data, last_size);
            }
        };

        template<class RawData_>
        struct Serialize<RawData_, true>
        {
            using RawData = RawData_;
            static constexpr bool is_larger_than_8byte = true;

        // private:
            // // 要素は隣接している(contiguous_iterator)。
            // // だが、これの配列がstd::uint8_tの配列とみなせるのか(std::memcpy(&obj, &src, size);とかしていいのか)はわからなかった。
            // // 多分無理なんじゃないかな。
            // using Chunk = std::array<std::uint8_t, 8>;
            // まだ理解が浅いので普通の配列にすることにした。
            // そもそもどうせ今回はboost::arrayに代入するわけでして、結局ループなりパラメータパックなりで初期化しないとなので...。
            
            // 配列型じゃないっぽく見えると添え字演算子で混乱しそうなのでやめた。
            // using Chunk = std::uint8_t[8];

        // public:
            static constexpr std::uint8_t last_size = (sizeof(RawData) % 8)? sizeof(RawData) % 8 : 8;
            static constexpr std::size_t chunks_size = (sizeof(RawData) - last_size) / 8;

            // これだけで未定義動作でないと保証できるわけじゃないけど、無いよりマシだろう。
            static_assert(sizeof(Serialize) - (last_size % 8) == sizeof(RawData_), "Oh, my Guess is off.");

            std::uint8_t chunks[chunks_size][8]{};
            alignas(1) std::uint8_t last_chunk[8]{}; // 連続性はこれで保証できているのか？

            Serialize(const RawData& raw_data)
            {
                std::memcpy(this, &raw_data, sizeof(RawData));
            }
        };
    }
}