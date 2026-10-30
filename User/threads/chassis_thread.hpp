#pragma once

#include <array>
#include <span>

#include "chassis.hpp"
#include "topic.hpp"

namespace JLUni
{
    struct ChassisThreadArgs
    {
        /// Chassis controller owning the 2 ms control-cycle state.
        JLUni::Chassis_N::Chassis& chassis;
        /// IMU topic producing timestamped BMI088 samples.
        LibXR::Topic* imu_topic;
    };

    /** @brief Runs IMU prediction and the chassis control cycle at 2 ms intervals. */
    void ChassisThread(ChassisThreadArgs* args);
}
