#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace JLUni::Math
{
    struct Vec3
    {
        // Right-handed Cartesian vectors; the chassis convention is x forward,
        // y left, z up.
        float x = 0.0F;
        float y = 0.0F;
        float z = 0.0F;

        constexpr Vec3 operator+(const Vec3 other) const
        {
            return {.x = x + other.x, .y = y + other.y, .z = z + other.z};
        }

        constexpr Vec3 operator-(const Vec3 other) const
        {
            return {.x = x - other.x, .y = y - other.y, .z = z - other.z};
        }

        constexpr Vec3 operator*(const float scalar) const
        {
            return {.x = x * scalar, .y = y * scalar, .z = z * scalar};
        }

        constexpr float Dot(const Vec3 other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }

        constexpr Vec3 Cross(const Vec3 other) const
        {
            return {.x = y * other.z - z * other.y, .y = z * other.x - x * other.z,
                .z = x * other.y - y * other.x};
        }

        bool IsFinite() const
        {
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        }
    };

    struct Mat3
    {
        // Columns are the local unit axes expressed in the parent frame.
        Vec3 x {.x = 1.0F, .y = 0.0F, .z = 0.0F};
        Vec3 y {.x = 0.0F, .y = 1.0F, .z = 0.0F};
        Vec3 z {.x = 0.0F, .y = 0.0F, .z = 1.0F};

        constexpr Vec3 Apply(const Vec3& vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        constexpr Mat3 RotateX(const float cosine, const float sine) const
        {
            return {.x = x, .y = y * cosine + z * sine, .z = z * cosine - y * sine};
        }

        constexpr Mat3 RotateZ(const float cosine, const float sine) const
        {
            return {.x = x * cosine + y * sine, .y = y * cosine - x * sine, .z = z};
        }
    };

    inline constexpr float pi = std::numbers::pi_v<float>;
    inline constexpr float gravity = 9.8048F;  // Local gravitational acceleration, m/s^2.
    inline constexpr float two_pi = 2.0f * pi;
    inline constexpr float inv_sqrt2 = 1.0f / std::numbers::sqrt2_v<float>;
    inline constexpr float deg_to_rad = pi / 180.0f;
    inline constexpr float rad_to_deg = 180.0f / pi;
    inline constexpr float rpm_to_rad_ps = pi / 30.0f;
    inline constexpr float rad_ps_to_rpm = 30.0f / pi;

    inline float Sign(const float value)
    {
    	if (value > 0.0f) return 1.0f;
    	if (value < 0.0f) return -1.0f;
    	return 0.0f;
    }

    inline float Clamp(const float value, const float lower, const float upper)
    {
    	return std::clamp(value, lower, upper);
    }

    inline float Clamp(const float value, const float abs_limit)
    {
    	return std::clamp(value, -abs_limit, abs_limit);
    }

    inline void ClampInPlace(float& value, const float lower, const float upper)
    {
    	value = Clamp(value, lower, upper);
    }

    inline void ClampInPlace(float& value, const float max_abs)
    {
    	value = Clamp(value, max_abs);
    }

    inline float ApplyDeadband(const float value, const float width)
    {
    	if (value > width) return value - width;
    	if (value < -width) return value + width;
    	return 0.0f;
    }

    inline float Wrap(const float value, const float lower, const float upper)
    {
    	if (value >= lower && value < upper) return value;
    	const float range = upper - lower;
    	float wrapped = std::fmod(value - lower, range);
    	if (wrapped < 0.0f) wrapped += range;
    	return wrapped + lower;
    }

    inline float WrapRad(const float angle)
    {
    	return Wrap(angle, -pi, pi);
    }

    inline float DiffRad(const float lhs, const float rhs)
    {
    	return WrapRad(lhs - rhs);
    }

    inline float DiffRadSingleTurn(const float lhs, const float rhs)
    {
    	float difference = lhs - rhs;
    	if (difference >= pi) difference -= two_pi;
    	if (difference < -pi) difference += two_pi;
    	return difference;
    }

    inline float Sqrt(const float value)
    {
    	return std::sqrt((std::max)(value, 0.0f));
    }
}
