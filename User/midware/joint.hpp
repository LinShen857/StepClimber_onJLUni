#pragma once

#include "libxr_def.hpp"
#include <cstdint>
#include <span>

namespace JLUni
{
    struct JointState
    {
        float pos_rad = 0.0F;
        float vel_rad_s = 0.0F;
        float tq_nm = 0.0F;
        bool ready = false;
    };

    struct JointCommand
    {
        float pos_rad = 0.0F;
        float vel_rad_s = 0.0F;
        float tq_ff_nm = 0.0F;
    };

    struct JointConfig
    {
        float dir;
        float offset_rad;
        float min_rad;
        float max_rad;
        float vel_limit_rad_s;
        float torque_limit_nm;
    };

    // Protocol-independent translation between motor and joint coordinates.
    class Joint
    {
    public:
        using ParseFunction = bool (*)(void*, std::span<const uint8_t>, JointState&);
        using HandleUnreadyFunction = LibXR::ErrorCode (*)(void*);
        using CtrlFunction = LibXR::ErrorCode (*)(void*, const JointCommand&);

        constexpr Joint(const JointConfig& config, void* context, const ParseFunction parse,
            const HandleUnreadyFunction handle_unready, const CtrlFunction send)
            : config_(config), context_(context), parse_(parse), handle_unready_(handle_unready), ctrl_(send)
        {}

        Joint(const Joint&) = delete;
        Joint& operator=(const Joint&) = delete;
        Joint(Joint&&) = delete;
        Joint& operator=(Joint&&) = delete;

        [[nodiscard]] bool IsValid() const
        { return context_ != nullptr && parse_ != nullptr && handle_unready_ != nullptr && ctrl_ != nullptr; }

        bool ParseFeedback(const std::span<const uint8_t> payload, JointState& state) const
        {
            JointState motor{};
            if (!parse_(context_, payload, motor)) return false;
            motor.pos_rad = config_.dir * motor.pos_rad - config_.offset_rad;
            motor.vel_rad_s *= config_.dir;
            motor.tq_nm *= config_.dir;
            state = motor;
            return true;
        }

        [[nodiscard]] LibXR::ErrorCode HandleUnready() const { return handle_unready_(context_); }

        [[nodiscard]] LibXR::ErrorCode Control(const JointCommand& command) const
        {
            return ctrl_(context_, {
                .pos_rad = config_.dir * (command.pos_rad + config_.offset_rad),
                .vel_rad_s = config_.dir * command.vel_rad_s,
                .tq_ff_nm = config_.dir * command.tq_ff_nm});
        }

    private:
        const JointConfig config_;
        void* const context_;
        const ParseFunction parse_;
        const HandleUnreadyFunction handle_unready_;
        const CtrlFunction ctrl_;
    };
}
