/*

<utility>にmake_index_sequenceがあるらしい。こちらは非推奨とする。これからはmake_index_sequenceを使うこと。

*/

#pragma once

#include <cstddef>

namespace StewLib
{
    namespace
    {
        template<std::size_t ... Numbers>
        struct Enumerate{};

        namespace EnumerateImplement
        {
            template<std::size_t N,bool IsN,std::size_t ... Numbers>
            struct EnumerateHelper
            {
                using type = Enumerate<Numbers ...>;
            };

            template<std::size_t N,std::size_t ... Numbers>
            struct EnumerateHelper<N,false,Numbers ...>
            {
                using type = typename EnumerateHelper<N,sizeof...(Numbers) == N - 1,Numbers ...,sizeof...(Numbers)>::type;
            };
        }

        template<std::size_t N>
        struct EnumerateMake
        {
            using type = typename EnumerateImplement::EnumerateHelper<N,0 == N - 1,0>::type;
        };

    }
}