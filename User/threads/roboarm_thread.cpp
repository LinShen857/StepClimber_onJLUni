#include "roboarm_thread.hpp"
#include "thread.hpp"

namespace JLUni
{
    void RoboarmControlThread(RoboarmThreadArgs* const args)
    {
        if (args == nullptr || args->controller.Validate() == false)  // shouldn't enter runtime
        {
            for (;;)
            {
                LibXR::Thread::Sleep(UINT32_MAX);
            }
        }
        std::array<JointState, ArmJointCount> states{};
        Arm_Controller<ArmJointCount>::Commands commands{};
        LibXR::MillisecondTimestamp wakeup(LibXR::Thread::GetTime());
        for (;;)
        {
            args->read_feedback(args->feedback_context, states);
            if (args->commands != nullptr && args->commands->PeekLatest(commands))
            {
                static_cast<void>(args->controller.SetTarget(commands));
            }
            static_cast<void>(args->controller.Control(states));
            // if (!args->controller.AllReady()) { LibXR::Thread::Sleep(1U); }
            LibXR::Thread::SleepUntil(wakeup, 2U);
        }
    }
}
