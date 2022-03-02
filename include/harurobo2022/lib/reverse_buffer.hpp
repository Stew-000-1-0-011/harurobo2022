#pragma once

#include <cstddef>
#include <cstdint>

namespace StewLib
{
    namespace
    {
        template<typename T>
        struct ReverseBuffer final
        {
            constexpr static std::size_t size = sizeof(T);
            std::uint8_t buffer[size]{};

            ReverseBuffer() = default;
            ReverseBuffer(const ReverseBuffer&) = default;
            ReverseBuffer& operator=(const ReverseBuffer&) = default;
            ReverseBuffer(ReverseBuffer&&) = default;
            ReverseBuffer& operator=(ReverseBuffer&&) = default;

            ReverseBuffer(const T& obj) noexcept
            {
                std::memcpy(buffer, &obj, size);
                reverse();
            }

            void reverse() noexcept
            {
                std::uint8_t tmp[size];
                for(std::size_t i = 0; i < size / 2; ++i)
                {
                    tmp[i] = buffer[i];
                    buffer[i] = buffer[size - 1 - i];
                    buffer[size - 1 - i] = tmp[i];
                }
            }
        };
    }
}