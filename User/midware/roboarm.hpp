#pragma once

#include "joint.hpp"
#include "roboarm_model.hpp"

#include <array>
#include <span>


namespace JLUni
{
    template <std::size_t N> class Arm_Controller
    {
    public:
        using Commands = std::array<JointCommand, N>;

        Arm_Controller(std::array<Joint, N>& joints, const Roboarm_N::MDHModel<N>& model)
            : joints_(joints), gravity_(model)
        {}

        Arm_Controller(const Arm_Controller&) = delete;
        Arm_Controller& operator=(const Arm_Controller&) = delete;

        [[nodiscard]] bool Validate() const
        {
            if (!gravity_.IsValid()) return false;
            for (const auto& joint : joints_)
            {
                if (!joint.IsValid()) return false;
            }
            return true;
        }

        LibXR::ErrorCode SetTarget(std::span<const JointCommand, N> commands)
        {
            for (std::size_t i = 0; i < N; ++i) targets_[i] = commands[i];
            target_set_ = true;
            return LibXR::ErrorCode::OK;
        }

        [[nodiscard]] bool AllReady() const noexcept { return all_ready_; }

        LibXR::ErrorCode Control(std::span<const JointState, N> states)
        {
            all_ready_ = true;
            for (std::size_t i = 0; i < N; ++i)
            {
                all_ready_ = all_ready_ && states[i].ready;
            }

            if (!all_ready_)
            {
                target_set_ = false;
                auto result = LibXR::ErrorCode::OK;
                for (std::size_t i = 0; i < N; ++i)
                {
                    const auto action = joints_[i].HandleUnready();
                    if (action != LibXR::ErrorCode::OK && result == LibXR::ErrorCode::OK) { result = action; }
                }
                return result;
            }

            if (!target_set_)
            {
                for (std::size_t i = 0; i < N; ++i) { targets_[i] = {states[i].pos_rad, 0.0F, 0.0F}; }
                target_set_ = true;
                return LibXR::ErrorCode::OK;
            }

            std::array<float, N> angles{};
            for (std::size_t i = 0; i < N; ++i) angles[i] = states[i].pos_rad;
            const auto gravity_torque = gravity_.Calculate(angles);
            Commands commands = targets_;
            for (std::size_t i = 0; i < N; ++i) commands[i].tq_ff_nm += gravity_torque[i];

            auto result = LibXR::ErrorCode::OK;
            for (std::size_t i = 0; i < N; ++i)
            {
                const auto send_result = joints_[i].Control(commands[i]);
                if (send_result != LibXR::ErrorCode::OK && result == LibXR::ErrorCode::OK) { result = send_result; }
            }
            return result;
        }

    private:
        std::array<Joint, N>& joints_;
        Roboarm_N::MDHGravity<N> gravity_;

        Commands targets_{};
        bool all_ready_ = false;
        bool target_set_ = false;
    };
}
