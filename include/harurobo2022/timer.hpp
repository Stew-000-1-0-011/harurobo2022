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

            double period;
            std::function<CallbackSignature> callback;
            ros::Timer tim;

            Timer(const Timer&) = delete;
            Timer& operator=(const Timer&) = delete;
            Timer(Timer&&) = delete;
            Timer& operator=(Timer&&) = delete;

        public:
            template<class F>
            Timer(const double period, const F& callback) noexcept:
                period{period},
                callback{callback},
                tim{nh.createTimer(ros::Duration(period), callback)}
            {}

            void change_period(const double changed_period) noexcept
            {
                tim = nh.createTimer(ros::Duration(changed_period), callback);
                period = changed_period;
            }

            template<class F>
            void change_callback(const F& changed_callback) noexcept
            {
                tim = nh.createTimer(ros::Duration(period), changed_callback);
                callback = changed_callback;
            }

            ros::Timer get_tim() const noexcept
            {
                return tim;
            }

            void activate() noexcept
            {
                tim = nh.createTimer(ros::Duration(period), callback);
            }

            void deactivate() noexcept
            {
                tim = nh.createTimer(ros::Duration(100000), [](const auto&) noexcept {});
            }

        };
    }
}