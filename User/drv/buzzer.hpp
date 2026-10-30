#pragma once

#include "libxr.hpp"
#include "pwm.hpp"

namespace JLUni
{
    class Buzzer
    {
        public:
        explicit Buzzer(LibXR::PWM& pwm) : pwm_(pwm) {}
        LibXR::ErrorCode SetVolume(float duty_cycle) const;
        LibXR::ErrorCode SetFrequency(uint32_t frequency) const;
        LibXR::ErrorCode SetEnabled(bool enabled) const;

        private:
        LibXR::PWM& pwm_;
    };
}
