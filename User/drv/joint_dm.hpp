#pragma once

#include <cstdint>
#include <type_traits>

#include "motor_dm.hpp"
#include "joint.hpp"


namespace JLUni
{
    struct gain_dm_s
    {
        uint16_t kp_raw;
        uint16_t kd_raw;
    };

    template <typename Bus, bool UseFd> class Joint_DM : public Motor_DM<Bus, UseFd>
    {
    public:
        Joint_DM(Bus& bus, const uint16_t motor_id, const MotorDM_N::scale_fac_s& scales, gain_dm_s gains)
            : Motor_DM<Bus, UseFd>(bus, motor_id, scales), gains_(gains) {}
        Joint_DM(const Joint_DM&) = delete;
        Joint_DM& operator=(const Joint_DM&) = delete;
        Joint_DM(Joint_DM&&) = delete;
        Joint_DM& operator=(Joint_DM&&) = delete;

        static bool ParseFeedback(void* context, std::span<const uint8_t> payload, JointState& state)
        {
            auto& self = *static_cast<Joint_DM*>(context);
            MotorDM_N::feedback_s feedback{};
            if (self.UpdateFrame(payload, feedback) != MotorDM_N::update_result_e::FEEDBACK)
                return false;
            state = {.pos_rad = feedback.pos_rad, .vel_rad_s = feedback.vel_rad_s,
                .tq_nm = feedback.tq_nm, .ready = feedback.error == MotorDM_N::err_state_e::ENABLED};
            return true;
        }

        static LibXR::ErrorCode HandleUnready(void* context)
        {
            auto& self = *static_cast<Joint_DM*>(context);
            const auto error = self.GetErrorState();
            if (error == MotorDM_N::err_state_e::ENABLED) return self.Stop();
            return self.SendCmd(error == MotorDM_N::err_state_e::DISABLED
                ? MotorDM_N::cmd_byte_e::enable : MotorDM_N::cmd_byte_e::clear_err);
        }

        LibXR::ErrorCode Stop() const { return this->SendCmd(MotorDM_N::cmd_byte_e::disable); }

        static LibXR::ErrorCode Ctrl(void* context, const JointCommand& command)
        {
            auto& self = *static_cast<Joint_DM*>(context);
            return self.CtrlMIT(command.pos_rad, command.vel_rad_s,
                self.gains_.kp_raw, self.gains_.kd_raw, command.tq_ff_nm);
        }

    private:
        const gain_dm_s gains_;
    };

    static_assert(std::is_base_of_v<Motor_DM<LibXR::CAN, false>, Joint_DM<LibXR::CAN, false>>);
    static_assert(!std::is_polymorphic_v<Joint_DM<LibXR::CAN, false>>);
}
