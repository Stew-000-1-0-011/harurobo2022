#pragma once

namespace StewLib
{
    namespace
    {
        template<class T>
        struct Pid
        {
            const double k_p;
            const double k_i;
            const double k_d;

            T sum_dev{};  // 0にあたる値
            T last_dev{};  // 多分初期値はどうでもいい

            T operator()(const T& dev) noexcept
            {
                sum_dev += dev;
                last_dev = dev;

                return k_p * dev + k_i * sum_dev + k_d * last_dev;
            }
        };
    }
}