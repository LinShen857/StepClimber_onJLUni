#pragma once


/* step-process the target to prevent sudden value shifts */
struct slope_s
{
    float step;
    float deadzone;
    float target;
    void SetStep(const float step_) { step = step_; }
    void SetDeadzone(const float deadzone_) { deadzone = deadzone_; }
    void SetTarget(const float target_) { target = target_; }
    void IncTarget() { target += step; }
    void DecTarget() { target -= step; }

    [[nodiscard]] bool IsArrived(const float value_) const
    {
        return value_ > target - deadzone && value_ < target + deadzone;
    }

    [[nodiscard]] float Update(const float value_) const
    {
        if (value_ > target - deadzone && value_ < target + deadzone) return value_;
        if (value_ > target + step) return value_ - step;
        if (value_ < target - step) return value_ + step;
        return target;
    }
};
