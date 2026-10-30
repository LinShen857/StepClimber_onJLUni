#pragma once

#include <cstdint>
#include <span>

#include "math_utils.hpp"

namespace JLUni::MotorDJI_N
{
    constexpr float pos_lsb_to_rad = Math::two_pi / 8192.0F;

    struct fb_2006_s
    {
        float pos_rad {0.F};
        float vel_rad_s {0.F};
        float tq_nm {0.F};
    };

    struct fb_3508_6020_s
    {
        float pos_rad {0.F};
        float vel_rad_s {0.F};
        float tq_nm {0.F};
        int8_t temp_c {0};
    };

    struct traits_3508
    {
        using feedback_type = fb_3508_6020_s;
        static constexpr float cmd_limit = 16384.0F;
        static constexpr float tq_fac = 20.0F * 0.3F / cmd_limit;
        static constexpr float tq_to_cmd = cmd_limit / (20.0F * 0.3F);
        static constexpr uint16_t fb_id_base {0x200U};
    };

    struct traits_6020
    {
        using feedback_type = fb_3508_6020_s;
        static constexpr float cmd_limit = 16384.0F;
        static constexpr float tq_fac = 3.0F * 0.741F / cmd_limit;
        static constexpr float tq_to_cmd = cmd_limit / (3.0F * 0.741F);
        static constexpr uint16_t fb_id_base {0x204U};
    };

    struct traits_2006
    {
        using feedback_type = fb_2006_s;
        static constexpr float cmd_limit = 10000.0F;
        static constexpr float tq_fac = 10.0F * 0.18F / cmd_limit;
        static constexpr float tq_to_cmd = cmd_limit / (10.0F * 0.18F);
        static constexpr uint16_t fb_id_base {0x200U};
    };
}  // namespace JLUni::MotorDJI_N

namespace JLUni
{
    template <typename Traits> class Motor_DJI
    {
    public:
        using feedback_type = Traits::feedback_type;
        explicit constexpr Motor_DJI() : feedback_{} {}
        void ParseFrame(const std::span<const uint8_t, 8> payload)
        {
            feedback_.pos_rad = static_cast<float>((static_cast<uint16_t>(payload[0]) << 8U) | payload[1])
                * MotorDJI_N::pos_lsb_to_rad;
            feedback_.vel_rad_s = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(payload[2]) << 8U)
                | payload[3])) * Math::rpm_to_rad_ps;
            feedback_.tq_nm = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(payload[4]) << 8U)
                | payload[5])) * Traits::tq_fac;
            if constexpr (requires { feedback_.temp_c; }) { feedback_.temp_c = static_cast<int8_t>(payload[6]); }
        }

        int16_t CalcCommand(float torque) const
        {
            return static_cast<int16_t>(std::lround(Math::Clamp(torque * Traits::tq_to_cmd, Traits::cmd_limit)));
        }
    protected:
        feedback_type feedback_;
    };
}  // namespace JLUni
