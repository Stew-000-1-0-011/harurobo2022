#pragma once

#include <list>
#include <functional>

namespace Harurobo2022
{
    namespace
    {
        struct StaticInitDeinit final
        {
            inline static std::list<std::function<void()>> initialize_list{};
            inline static std::list<std::function<void()>> deinitialize_list{};

            StaticInitDeinit() noexcept
            {
                for(auto& init : initialize_list)
                {
                    init();
                }
            }

            ~StaticInitDeinit() noexcept
            {
                for(auto& deinit : deinitialize_list)
                {
                    deinit();
                }
            }
        };
    }
}