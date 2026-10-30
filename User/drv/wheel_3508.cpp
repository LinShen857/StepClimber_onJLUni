#include "wheel_3508.hpp"


namespace JLUni::Chassis_N
{
    void Wheel::Update(const std::span<const uint8_t, 8> payload, const uint16_t id)
    {
        if (id != id_) return;
        ParseFrame(payload);
        if (!inited_)
        {
            speed_filter_.Reset(feedback_.vel_rad_s);
            inited_ = true;
        }
        else
        {
            speed_filter_.Update(feedback_.vel_rad_s);
        }
    }

    void Wheel::Reset()
    {
        feedback_ = {};
        speed_filter_.Reset();
        inited_ = false;
    }

    float Wheel::GetSpeed() const { return speed_filter_.x * fac_speed_; }

    float Wheel::GetForce() const { return feedback_.tq_nm * fac_force_; }

    int16_t Wheel::ForceToCommand(const float force) const { return CalcCommand(force * fac_speed_); }
}
