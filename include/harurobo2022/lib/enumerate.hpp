#pragma once

#include <cstddef>

namespace StewEnumerate
{

  template<std::size_t ... Numbers>
  struct Enumerate{};

  namespace Detail
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
      using type = typename Detail::EnumerateHelper<N,0 == N - 1,0>::type;
  };

}