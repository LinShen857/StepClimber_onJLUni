#pragma once

#include "kalman.hpp"
#include "motor_dji.hpp"

namespace JLUni::Chassis_N
{
    // Thread-owned state. Call Update once per new frame read from LastSlotSet.
    // Positive ratio/radius and filter R; nonnegative Q, in rotor (rad/s)^2 per update.
    class Wheel : public Motor_DJI<MotorDJI_N::traits_3508>
    {
    public:
        explicit constexpr Wheel(const uint8_t id, const float ratio, const float radius, const float q, const float r)
            : fac_speed_(radius / ratio), fac_force_(ratio / radius),
            speed_filter_{.x = 0.F, .P = 1.F, .Q = q / r, .R = 1.F, .K = 0.F},
            id_(id + MotorDJI_N::traits_3508::fb_id_base), inited_(false)
        {}

        /**
         * @brief Update motor data and estimate kalman-filtered speed with a new frame from LastSlotSet.
         */
        void Update(std::span<const uint8_t, 8> payload, uint16_t id);

        /**
         * @brief Reset the wheel state, clearing filtered speed and covariance.
         */
        void Reset();

        /**
         * @brief Get filtered wheel speed in m/s.
         */
        float GetSpeed() const;

        /**
         * @brief Get estimated wheel force in N, calculated from torque and ratio/radius.
         */
        float GetForce() const;

        /**
         * @brief Convert a force in N to a command value.
         */
        int16_t ForceToCommand(float force) const;

    private:
        const float fac_speed_;  // radius / ratio; also rotor torque per unit force
        const float fac_force_;  // ratio / radius
        Algo::kalman_r1_s speed_filter_;  // R is normalized to 1; Q is q / r.
        uint16_t id_;
        bool inited_;
    };
}  // namespace JLUni::Chassis_N
