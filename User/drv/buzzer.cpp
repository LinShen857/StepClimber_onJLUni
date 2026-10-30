#include "buzzer.hpp"

namespace JLUni
{
    LibXR::ErrorCode Buzzer::SetVolume(const float duty_cycle) const
    {
        return pwm_.SetDutyCycle(duty_cycle);
    }

    LibXR::ErrorCode Buzzer::SetFrequency(const uint32_t frequency) const
    {
        return pwm_.SetConfig({frequency});
    }

    LibXR::ErrorCode Buzzer::SetEnabled(const bool enabled) const
    {
        return enabled ? pwm_.Enable() : pwm_.Disable();
    }
}
