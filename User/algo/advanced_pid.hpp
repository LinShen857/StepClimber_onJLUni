#pragma once

#include "math_utils.hpp"


namespace JLUni::Algo
{
    struct pid_cfg_s
    {
        const float kp;
        const float ki;
        const float kd;
        const float deadzone;
        const float integral_limit;
        const float output_limit;
    };

    //template <typename T>
    //concept pid = (std::constructible_from<T, const pid_cfg_t&>
    //	|| std::constructible_from<T, const pid_cfg_t&, const pid_cfg_t&>)
    //	&& requires(T t)
    //	{
    //		t.output;
    //		t.Update(float{}, float{});
    //		t.Clear();
    //	};

    // struct pid_s
    // {
    // 	float kp;
    // 	float ki;
    // 	float kd;
    // 	float deadzone;
    // 	float error;
    // 	float error_prev;
    // 	float integral_limit;
    // 	float integral;
    // 	float output_limit;
    // 	float output;
    // 	explicit pid_s(const pid_cfg_s& pid_cfg_)
    // 		: kp(pid_cfg_.kp), ki(pid_cfg_.ki), kd(pid_cfg_.kd), deadzone(pid_cfg_.deadzone), error(0), error_prev(0),
    // 		integral_limit(pid_cfg_.integral_limit), integral(0), output_limit(pid_cfg_.output_limit), output(0)
    // 	{}
    // 	float Update(const float target_, const float feedback_)
    // 	{
    // 		error_prev = error;
    // 		error = target_ - feedback_;
    // 		if (error < deadzone && error > -deadzone) return output;
    // 		integral = JLUni::Math::Clamp(integral + error, integral_limit);
    // 		const float derivative = (error - error_prev) * kd;
    // 		output = error * kp + integral * ki + derivative;
    // 		output = JLUni::Math::Clamp(output, output_limit);
    // 		return output;
    // 	}
    // 	void Clear()
    // 	{
    // 		error = 0;
    // 		error_prev = 0;
    // 		integral = 0;
    // 		output = 0;
    // 	}
    // };

    struct pid_diff_ahead_s
    {
        float kp;
        float ki;
        float kd;
        float deadzone;
        float error;
        float feedback_prev;
        float integral_limit;
        float integral;
        float output_limit;
        float output;
        explicit pid_diff_ahead_s(const pid_cfg_s& pid_cfg_)
            : kp(pid_cfg_.kp), ki(pid_cfg_.ki), kd(pid_cfg_.kd), deadzone(pid_cfg_.deadzone), error(0), feedback_prev(0),
            integral_limit(pid_cfg_.integral_limit), integral(0), output_limit(pid_cfg_.output_limit), output(0)
        {}
        float Update(const float target_, const float feedback_)
        {
            error = target_ - feedback_;
            if (error < deadzone && error > -deadzone)
            {
                feedback_prev = feedback_;  // doubt its necessity
                return output;
            }
            integral = JLUni::Math::Clamp(integral + error, integral_limit);
            const float derivative = (feedback_prev - feedback_) * kd;
            feedback_prev = feedback_;
            output = JLUni::Math::Clamp(error * kp + integral * ki + derivative, output_limit);
            return output;
        }
        void Clear()
        {
            error = 0;
            feedback_prev = 0;
            integral = 0;
            output = 0;
        }
    };

    struct pid_pd_ahead_s
    {
        float kp;
        float ki;
        float kd;
        float deadzone;
        float error;
        float feedback_prev;
        float integral_limit;
        float integral;
        float output_limit;
        float output;
        explicit pid_pd_ahead_s(const pid_cfg_s& pid_cfg_)
            : kp(pid_cfg_.kp), ki(pid_cfg_.ki), kd(pid_cfg_.kd), deadzone(pid_cfg_.deadzone), error(0), feedback_prev(0),
            integral_limit(pid_cfg_.integral_limit), integral(0), output_limit(pid_cfg_.output_limit), output(0)
        {}
        float Update(const float target_, const float feedback_)
        {
            error = target_ - feedback_;
            if (error < deadzone && error > -deadzone)
            {
                feedback_prev = feedback_;
                return output;
            }
            integral = JLUni::Math::Clamp(integral + error, integral_limit);
            const float derivative = (feedback_prev - feedback_) * kd;
            feedback_prev = feedback_;
            output = JLUni::Math::Clamp(-feedback_ * kp + integral * ki + derivative, output_limit);
            return output;
        }
        void Clear()
        {
            error = 0;
            feedback_prev = 0;
            integral = 0;
            output = 0;
        }
    };

    struct pid_filtered_s
    {
        float kp;
        float ki;
        float kd;
        float deadzone;
        float error;
        float feedback_prev;
        float alpha;
        float derivative;
        float integral_limit;
        float integral;
        float output_limit;
        float output;
        explicit pid_filtered_s(const pid_cfg_s& pid_cfg_, const float alpha_)
            : kp(pid_cfg_.kp), ki(pid_cfg_.ki), kd(pid_cfg_.kd), deadzone(pid_cfg_.deadzone), error(0), feedback_prev(0),
            alpha(alpha_), derivative(0), integral_limit(pid_cfg_.integral_limit), integral(0),
            output_limit(pid_cfg_.output_limit), output(0)
        {}
        float Update(const float target_, const float feedback_)
        {
            error = target_ - feedback_;
            if (error < deadzone && error > -deadzone)
            {
                feedback_prev = feedback_;  // doubt its necessity
                return output;
            }
            integral = Math::Clamp(integral + error, integral_limit);
            const float attenuation = 1.f - alpha;
            derivative = derivative * attenuation + (feedback_prev - feedback_) * kd * alpha;
            feedback_prev = feedback_;
            output = Math::Clamp(error * kp + integral * ki + derivative, output_limit);
            return output;
        }
        void Clear()
        {
            error = 0;
            feedback_prev = 0;
            integral = 0;
            output = 0;
        }
    };
}
