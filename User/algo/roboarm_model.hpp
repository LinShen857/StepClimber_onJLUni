#pragma once

#include "math_utils.hpp"

#include <array>
#include <cmath>
#include <cstddef>


namespace JLUni::Roboarm_N
{
    struct MDHLink
    {
        // T = Rx(alpha) Tx(a) Rz(q + theta_offset) Tz(d).
        // COM is expressed in the FINAL link frame; all units are SI.
        float alpha = 0, a = 0, d = 0, theta_offset = 0, mass = 0;
        Math::Vec3 com{};
    };

    template <std::size_t N> struct MDHModel
    {
        std::array<MDHLink, N> links{};
        // Gravity is expressed in the right-handed model frame; z points up.
        Math::Vec3 gravity{.x = 0, .y = 0, .z = -Math::gravity};
        bool gravity_enabled = false;
    };

    template <std::size_t N> class MDHGravity
    {
        static_assert(N > 0);
    public:
        explicit MDHGravity(const MDHModel<N>& model) : model_(model)
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                cos_alpha_[i] = std::cos(model.links[i].alpha);
                sin_alpha_[i] = std::sin(model.links[i].alpha);
            }
        }

        MDHGravity(const MDHGravity&) = delete;
        MDHGravity& operator=(const MDHGravity&) = delete;

        [[nodiscard]] bool IsValid() const
        {
            if (!model_.gravity.IsFinite()) { return false; }
            for (const auto& link : model_.links)
            {
                if (!std::isfinite(link.alpha) || !std::isfinite(link.a) ||
                    !std::isfinite(link.d) || !std::isfinite(link.theta_offset) ||
                    !std::isfinite(link.mass) || link.mass < 0 || !link.com.IsFinite()) { return false; }
            }
            return true;
        }

        // Fixed member workspace keeps the periodic stack small. Single task only.
        std::array<float, N> Calculate(const std::array<float, N>& angles)
        {
            std::array<float, N> torque{};
            if (!model_.gravity_enabled) { return torque; }
            Math::Mat3 rotation;
            Math::Vec3 translation;
            for (std::size_t i = 0; i < N; ++i)
            {
                const auto& link = model_.links[i];
                rotation = rotation.RotateX(cos_alpha_[i], sin_alpha_[i]);
                translation = translation + rotation.x * link.a;
                origins_[i] = translation;
                axes_[i] = rotation.z;
                const float theta = angles[i] + link.theta_offset;
                rotation = rotation.RotateZ(std::cos(theta), std::sin(theta));
                translation = translation + rotation.z * link.d;
                centers_[i] = translation + rotation.Apply(link.com);
            }
            for (std::size_t j = 0; j < N; ++j)
            {
                for (std::size_t i = j; i < N; ++i)
                {
                    torque[j] -= axes_[j].Dot((centers_[i]
                        - origins_[j]).Cross(model_.gravity * model_.links[i].mass));
                }
            }
            return torque;
        }

    private:
        const MDHModel<N>& model_;
        std::array<float, N> cos_alpha_{}, sin_alpha_{};
        std::array<Math::Vec3, N> origins_{}, axes_{}, centers_{};
    };
}
