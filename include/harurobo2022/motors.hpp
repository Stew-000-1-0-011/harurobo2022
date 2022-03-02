#pragma once

#include <array>

#include "stringlike_types.hpp"
#include "shirasu_publisher.hpp"
#include "config.hpp"

namespace Harurobo2022
{
    namespace
    {
        struct DriveMotors final
        {
            // キューサイズを指定できないのはどうなんだ...?
            ShirasuPublisher<StringlikeTypes::FR_drive, Config::CanId::Tx::DriveMotor::FR> FR_pub{};
            ShirasuPublisher<StringlikeTypes::FL_drive, Config::CanId::Tx::DriveMotor::FL> FL_pub{};
            ShirasuPublisher<StringlikeTypes::BL_drive, Config::CanId::Tx::DriveMotor::BL> BL_pub{};
            ShirasuPublisher<StringlikeTypes::BR_drive, Config::CanId::Tx::DriveMotor::BR> BR_pub{};

            void send_cmd_all(ShirasuUtil::Mode cmd) noexcept
            {
                // なんでキャストしてコンテナに詰め込めないんだろうか。
                FR_pub.send_cmd(cmd);
                FL_pub.send_cmd(cmd);
                BL_pub.send_cmd(cmd);
                BR_pub.send_cmd(cmd);
            }

            void activate() noexcept
            {
                FR_pub.activate();
                FL_pub.activate();
                BL_pub.activate();
                BR_pub.activate();
            }

            void deactivate() noexcept
            {
                FR_pub.deactivate();
                FL_pub.deactivate();
                BL_pub.deactivate();
                BR_pub.deactivate();
            }
        };

        struct LiftMotors final
        {
            ShirasuPublisher<StringlikeTypes::FR_lift, Config::CanId::Tx::LiftMotor::FR> FR_pub{};
            ShirasuPublisher<StringlikeTypes::FL_lift, Config::CanId::Tx::LiftMotor::FL> FL_pub{};
            ShirasuPublisher<StringlikeTypes::BL_lift, Config::CanId::Tx::LiftMotor::BL> BL_pub{};
            ShirasuPublisher<StringlikeTypes::BR_lift, Config::CanId::Tx::LiftMotor::BR> BR_pub{};
            ShirasuPublisher<StringlikeTypes::subX_lift, Config::CanId::Tx::LiftMotor::subX> subX_pub{};
            ShirasuPublisher<StringlikeTypes::subY_lift, Config::CanId::Tx::LiftMotor::subY> subY_pub{};
            ShirasuPublisher<StringlikeTypes::collector_lift, Config::CanId::Tx::LiftMotor::collector> collector_pub{};

            void send_cmd_all(ShirasuUtil::Mode cmd) noexcept
            {
                FR_pub.send_cmd(cmd);
                FL_pub.send_cmd(cmd);
                BL_pub.send_cmd(cmd);
                BR_pub.send_cmd(cmd);
                subX_pub.send_cmd(cmd);
                subY_pub.send_cmd(cmd);
                collector_pub.send_cmd(cmd);
            }

            void activate() noexcept
            {
                FR_pub.activate();
                FL_pub.activate();
                BL_pub.activate();
                BR_pub.activate();
                subX_pub.activate();
                subY_pub.activate();
                collector_pub.activate();
            }

            void deactivate() noexcept
            {
                FR_pub.deactivate();
                FL_pub.deactivate();
                BL_pub.deactivate();
                BR_pub.deactivate();
                subX_pub.deactivate();
                subY_pub.deactivate();
                collector_pub.deactivate();
            }
        };
    }
}