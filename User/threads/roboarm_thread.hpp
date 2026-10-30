#pragma once
#include "roboarm.hpp"
#include "last_buffer.hpp"
#include "last_slot_set.hpp"
#include <cstdint>
#include <span>

namespace JLUni
{
    inline constexpr std::size_t ArmJointCount = 6;
    using ArmStates = Algo::LastSlotSet<JointState, ArmJointCount>;

    struct ArmRxRoute
    {
        uint32_t id;
        uint8_t joint_index;
    };

    struct ArmFeedbackContext
    {
        std::span<Joint, ArmJointCount> joints;
        ArmStates& states;
        std::span<const ArmRxRoute> routes;
    };

    struct RoboarmThreadArgs
    {
        Arm_Controller<ArmJointCount>& controller;
        void (*read_feedback)(void*, std::span<JointState, ArmJointCount>);
        void* feedback_context;
        Algo::LastBuffer<Arm_Controller<ArmJointCount>::Commands>* commands = nullptr;
    };
    void RoboarmControlThread(RoboarmThreadArgs* args);
}
