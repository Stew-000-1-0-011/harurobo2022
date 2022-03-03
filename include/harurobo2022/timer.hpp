#pragma once

#include <functional>

#include <ros/ros.h>

namespace Harurobo2022
{
    namespace
    {
        class Timer final
        {
            using CallbackSignature = void(const ros::TimerEvent&);

            ros::NodeHandle nh{};

            std::function<CallbackSignature> callback;
            std::function<CallbackSignature> inner_callback;
            ros::Timer tim;

            Timer(const Timer&) = delete;
            Timer& operator=(const Timer&) = delete;
            Timer(Timer&&) = delete;
            Timer& operator=(Timer&&) = delete;

        public:
            template<class F>
            Timer(const double period, const F& callback) noexcept:
                callback{callback},
                inner_callback{callback},
                tim{nh.createTimer(ros::Duration(period), &Timer::callback_wrapper, this)}
            {}

            template<class F>
            void change_callback(const F& changed_callback) noexcept
            {
                inner_callback = changed_callback;
                callback = changed_callback;
            }

            ros::Timer get_tim() const noexcept
            {
                return tim;
            }

            void activate() noexcept
            {
                inner_callback = callback;
            }

            void deactivate() noexcept
            {
                inner_callback = [](const auto&){};
            }

        private:
            void callback_wrapper(const ros::TimerEvent& event) noexcept
            {
                inner_callback(event);
            }

        };
    }
}