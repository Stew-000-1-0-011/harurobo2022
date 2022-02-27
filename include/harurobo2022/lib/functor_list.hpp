// 簡単にはできないのでやめた。

// #pragma once

// namespace StewLib
// {
//     template<class ...>
//     struct List final
//     {};

//     template<class Append, class ArgList>
//     struct Concat final
//     {
//         using type = List
//     };

//     template<class ... Functor>
//     struct FunctorList final
//     {
//         static void work() noexcept
//         {
//             (Functor::func(), ...);
//         }
//     };
// }