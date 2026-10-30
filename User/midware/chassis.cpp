#include "chassis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace JLUni::Chassis_N
{
    // Chassis frame: x forward, y left, z up. Wheel order: FL, FR, RR, RL.
    namespace
    {
        constexpr float kWheelRadius = 0.075F;
        constexpr float kGearRatio = 268.0F / 17.0F;
        constexpr float kRearLegRadius = 0.23F;
        constexpr float kWheelX0[4] = {0.3F, 0.3F, 0.0F, 0.0F};
        constexpr float kWheelY[4] = {0.22F, -0.22F, -0.26F, 0.26F};
        // Wheel-center z at the flat reference posture. y is lateral only.
        constexpr float kWheelZ0[4] = {0.0F, 0.0F, 0.0F, 0.0F};
        constexpr float kFrontContactX = 0.3F;
        constexpr float kFrontContactZ = 0.0F;
        constexpr float kPitchDeadband = 5.0F * Math::deg_to_rad;
        constexpr float kPitchSet = 0.0F;  // Target body pitch in rad, nose-down positive.
        constexpr float kPitchFilterTime = 0.1F;
        constexpr float kRearLegRateMax = 6.0F;
        // FL, FR, RR, RL. Roller geometry and motor direction are separate conventions.
        constexpr float kRollerSign[4] = {1.0F, -1.0F, -1.0F, 1.0F};
        constexpr float kMotorForwardSign[4] = {1.0F, -1.0F, -1.0F, 1.0F};
        constexpr float kMass = 35.0F, kYawInertia = 10.0F;
        constexpr float kContactPowerLimit = 100.0F;

        struct rear_leg_cfg_s
        {
            float x0, center_z, flat_angle, min_rad, max_rad;
        };

        constexpr std::array<rear_leg_cfg_s, 2> kRearLegConfig
        {{
            {.x0 = kWheelX0[2], .center_z = kWheelZ0[2] - kRearLegRadius, .flat_angle = 0.0F,
                .min_rad = 0.0F, .max_rad = Math::pi},
            {.x0 = kWheelX0[3], .center_z = kWheelZ0[3] - kRearLegRadius, .flat_angle = 0.0F,
                .min_rad = 0.0F, .max_rad = Math::pi}
        }};
        constexpr float kMaxWheelForce = MotorDJI_N::traits_3508::cmd_limit
            * MotorDJI_N::traits_3508::tq_fac * kGearRatio / kWheelRadius;
        constexpr Algo::pid_cfg_s kPid  // TODO: consider just use default construct
        {
            .kp = 0.0F, .ki = 0.0F, .kd = 0.0F, .deadzone = 0.0F, .integral_limit = 0.0F, .output_limit = 1.0e10F
        };

        struct power_param_s
        {
            float force_squared, speed_force, speed_squared, constant;
        };

        // FL, FR, RR, RL. PLACEHOLDERS: zero coefficients predict zero power, not actual power protection.
        // Legacy wheel-shaft (omega, torque): a_new=a_old*r^2, c_new=c_old/r^2.
        // Legacy rotor coordinates: replace r above with r/g. The cross/constant terms do not change.
        constexpr std::array<power_param_s, 4> kPowerParams{};
        // TODO: consider cancel checkment here
        static_assert(std::ranges::all_of(kPowerParams, [](const power_param_s& param)
        {
            constexpr float max = std::numeric_limits<float>::max();
            return (param.force_squared >= 0.0F && param.force_squared <= max)
                && (param.speed_force >= -max && param.speed_force <= max)
                && (param.speed_squared >= -max && param.speed_squared <= max)
                && (param.constant >= -max && param.constant <= max);
        }), "Power coefficients must be finite and force_squared nonnegative");

        /**
         * @brief Evaluates one wheel's quadratic power model at a speed and force.
         */
        float EvaluateWheelPower(const power_param_s& param, const float speed, const float force)
        {
            return (param.force_squared * force + param.speed_force * speed) * force
                + param.speed_squared * speed * speed + param.constant;
        }

        /**
         * @brief Intersects an ordered interval with the current scalar bounds.
         */
        bool IntersectBounds(const float lower, const float upper, float& bound_low, float& bound_high)
        {
            bound_low = std::max(bound_low, lower);
            bound_high = std::min(bound_high, upper);
            return bound_low <= bound_high;
        }

        // Intersect a convex quadratic budget with scalar bounds.
        // The normalized solve avoids coefficient overflow.
        /**
         * @brief Finds the feasible interval for a convex quadratic budget inequality.
         */
        bool IntersectConvexBudget(float a, float b, float c, const float budget, float& bound_low, float& bound_high)
        {
            // Move the budget to the constant term so the inequality is <= 0.
            c -= budget;
            if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) || a < 0.0F) return false;
            if (a == 0.0F)
            {
                if (b == 0.0F) return c <= 0.0F;
                const float root = -c / b;
                return b > 0.0F
                    ? IntersectBounds(-std::numeric_limits<float>::infinity(),
                        root, bound_low, bound_high)
                    : IntersectBounds(root, std::numeric_limits<float>::infinity(),
                        bound_low, bound_high);
            }
            // Normalize before forming the discriminant to avoid coefficient overflow.
            const float magnitude = std::max(a, std::max(std::abs(b), std::abs(c)));
            a /= magnitude;
            b /= magnitude;
            c /= magnitude;
            if (a == 0.0F) return false; // unsupported underflow; never claim a feasible solution
            const float discriminant = std::fma(-4.0F * a, c, b * b);
            if (discriminant < 0.0F) return false;
            // The q-form avoids cancellation when the roots have different magnitudes.
            const float q = -0.5F * (b + std::copysign(std::sqrt(discriminant), b));
            const float first = q == 0.0F ? -b / (2.0F * a) : q / a;
            const float second = q == 0.0F ? first : c / q;
            return IntersectBounds(std::min(first, second), std::max(first, second),
                bound_low, bound_high);
        }

        /**
         * @brief Finds the force interval allowed by one wheel's power budget.
         */
        bool IntersectPowerBudget(const power_param_s& param, const float speed,
            const float budget, float& force_low, float& force_high)
        {
            return IntersectConvexBudget(param.force_squared, param.speed_force * speed,
                param.speed_squared * speed * speed + param.constant, budget, force_low, force_high);
        }

        /**
         * @brief Evaluates a PID controller as an affine force expression in the command scale.
         */
        pid_affine_s EvaluatePid(const Algo::pid_diff_ahead_s& pid, const float target_bias,
            const float target_slope, const float feedback)
        {
            const float e0 = target_bias - feedback;
            // Use the existing integral; this tick's error takes effect next tick.
            return
            {
                .bias = pid.kp * e0 + pid.ki * pid.integral + pid.kd * (pid.feedback_prev - feedback),
                .slope = pid.kp * target_slope
            };
        }

        /** @brief Commits the scaled affine PID result to the controller state. */
        void CommitPid(Algo::pid_diff_ahead_s& pid, const pid_affine_s& affine,
                       const float target_bias, const float target_slope, const float feedback, const float scale)
        {
            pid.error = target_bias + scale * target_slope - feedback;
            pid.output = affine.bias + scale * affine.slope;
            pid.integral = Math::Clamp(pid.integral + pid.error, pid.integral_limit);
            pid.feedback_prev = feedback;
        }

        constexpr std::array<float, 4> kLiftClimbPowerWeights{{0.125F, 0.125F, 0.375F, 0.375F}};
        constexpr std::array<float, 4> kRetractPowerWeights{{0.375F, 0.375F, 0.125F, 0.125F}};
    }

    /**
     * @brief Builds FL/FR/RR/RL lever arms in the x-y wheel plane.
     */
    bool geometry_s::Update(const std::array<JointState, 4>& joints, const float pitch_rad)
    {
        // Rear legs change wheel x and xdot.
        float sum = 0.0F, signed_sum = 0.0F;
        for (size_t i = 0; i < 4; ++i)
        {
            float x = kWheelX0[i];
            const float y = kWheelY[i];
            if (i >= 2)
            {
                const float q = joints[i].pos_rad;
                // Rear legs move in the x-z plane; y remains the fixed lateral coordinate.
                x -= kRearLegRadius * std::sin(q);
                xdot[i - 2] = -kRearLegRadius * std::cos(q - pitch_rad) * joints[i].vel_rad_s;
                if (!std::isfinite(xdot[i - 2])) return false;
            }
            lever[i] = kRollerSign[i] * x - y;
            if (!std::isfinite(lever[i])) return false;
            sum += lever[i];
            signed_sum += kRollerSign[i] * lever[i];
        }
        mean = sum * 0.25F;
        signed_mean = signed_sum * 0.25F;
        float residual = 0.0F;
        for (size_t i = 0; i < 4; ++i)
        {
            const float c = lever[i] - mean - kRollerSign[i] * signed_mean;
            residual += c * c;
        }
        // A near-zero residual would make the inverse geometry numerically singular.
        if (!std::isfinite(residual) || residual <= 1.0e-7F) return false;
        inverse_residual = 1.0F / residual;
        return true;
    }

    /**
     * @brief Solves the three body velocity components from wheel velocity sums.
     */
    std::array<float, 3> geometry_s::Solve(const std::array<float, 3>& rhs) const
    {
        const float z = (rhs[2] - mean * rhs[0] - signed_mean * rhs[1]) * inverse_residual;
        return {rhs[0] * 0.25F - mean * z, rhs[1] * 0.25F - signed_mean * z, z};
    }

    /**
     * @brief Evaluates the wheel force represented by a body force solution.
     */
    float geometry_s::Force(const std::array<float, 3>& solution, const size_t i) const
    {
        return solution[0] + kRollerSign[i] * solution[1] + lever[i] * solution[2];
    }

    /**
     * @brief Returns the rear-wheel forward-speed term induced by leg motion.
     */
    float geometry_s::Compensation(const size_t i) const { return i < 2 ? 0.0F : xdot[i - 2]; }

    namespace
    {
        // Parameters are compile-time validated; speed is in the motor's sign convention.
        /**
         * @brief Finds a common wheel-force scale satisfying force and power constraints.
         */
        bool SolveScale(const std::array<pid_affine_s, 4>& force, const std::array<float, 4>& speed,
            const std::array<power_param_s, 4>& params, const float budget, float& scale)
        {
            const bool limited = budget != std::numeric_limits<float>::infinity();
            if (limited && !std::isfinite(budget)) return false;
            float scale_low = -std::numeric_limits<float>::infinity();
            float scale_high = std::numeric_limits<float>::infinity();
            float total_a = 0.0F, total_b = 0.0F, total_c = 0.0F;
            for (size_t i = 0; i < 4; ++i)
            {
                const auto& [bias, slope] = force[i];
                if (!std::isfinite(bias) || !std::isfinite(slope) || !std::isfinite(speed[i])) { return false; }
                if (slope == 0.0F)
                {
                    if (std::abs(bias) > kMaxWheelForce) return false;
                }
                else
                {
                    const float inverse = 1.0F / slope;
                    const float a = (-kMaxWheelForce - bias) * inverse;
                    if (const float b = (kMaxWheelForce - bias) * inverse;
                        !IntersectBounds(std::min(a, b), std::max(a, b), scale_low, scale_high))
                    {
                        return false;
                    }
                }
                if (limited)
                {
                    // Accumulate the total power polynomial after substituting force = bias + scale*slope.
                    const auto& [force_squared_, speed_force_,
                        speed_squared_, constant_] = params[i];
                    const float linear = speed_force_ * speed[i];
                    const float constant = speed_squared_ * speed[i] * speed[i] + constant_;
                    total_a += force_squared_ * slope * slope;
                    total_b += (2.0F * force_squared_ * bias + linear) * slope;
                    total_c += (force_squared_ * bias + linear) * bias + constant;
                }
            }
            scale = std::clamp(1.0F, scale_low, scale_high);
            if (!std::isfinite(scale)) return false;
            if (!limited) return true;
            if (!std::isfinite(total_a) || !std::isfinite(total_b) || !std::isfinite(total_c)) return false;
            const float candidate_power = (total_a * scale + total_b) * scale + total_c;
            if (std::isfinite(candidate_power) && candidate_power <= budget) return true;
            if (!IntersectConvexBudget(total_a, total_b, total_c, budget, scale_low, scale_high)) return false;
            scale = std::clamp(1.0F, scale_low, scale_high);
            return std::isfinite(scale);
        }

        /**
         * @brief Extracts right-handed pitch about +y from a [w, x, y, z] quaternion.
         */
        float PitchFromQuaternion(const VQFPose& q)
        {
            return std::atan2(2.0F * (q[0] * q[2] - q[1] * q[3]), 1.0F - 2.0F * (q[1] * q[1] + q[2] * q[2]));
        }

        /**
         * @brief Returns the nose-down pitch implied by a rear-leg angle and both ground contacts.
         */
        float PosturePitch(const float q, const rear_leg_cfg_s& cfg)
        {
            const float dx = kFrontContactX - (cfg.x0 - kRearLegRadius * std::sin(q));
            const float dz = kFrontContactZ - (cfg.center_z + kRearLegRadius * std::cos(q));
            return dx > 1.0e-3F ? std::atan(dz / dx) : 0.0F;
        }

        /**
         * @brief Solves the rear-leg angle that puts the body at the requested nose-down pitch.
         */
        float RearLegAngleForPitch(const float pitch, const rear_leg_cfg_s& cfg)
        {
            const float cosine = (std::cos(pitch) * (kFrontContactZ - cfg.center_z)
                - (kFrontContactX - cfg.x0) * std::sin(pitch)) / kRearLegRadius;
            return pitch + std::acos(Math::Clamp(cosine, -1.0F, 1.0F));
        }
    }

    /**
     * @brief Initializes chassis buffers, wheel estimators and controller state.
     */
    Chassis::Chassis(Algo::LastBuffer<chassis_cmd_s>* command_buffer, Algo::LastSlotSet<JointState, 4>* leg_buffer,
        Algo::LastSlotSet<std::array<uint8_t, 8>, 4>* wheel_buffer, Algo::LastBuffer<VQFPose>* pose_buffer,
        const std::span<Joint, 4> legs)
        : command_buffer_(command_buffer), leg_states_(leg_buffer), wheel_frames_(wheel_buffer),
        pose_buffer_(pose_buffer), legs_(legs), wheels_{Wheel(1, kGearRatio, kWheelRadius, 0.01F, 0.1F),
            Wheel(2, kGearRatio, kWheelRadius, 0.01F, 0.1F),
            Wheel(3, kGearRatio, kWheelRadius, 0.01F, 0.1F),
            Wheel(4, kGearRatio, kWheelRadius, 0.01F, 0.1F)},
        pid_speed_{Algo::pid_diff_ahead_s(kPid), Algo::pid_diff_ahead_s(kPid), Algo::pid_diff_ahead_s(kPid)},
        pid_wheel_{Algo::pid_diff_ahead_s(kPid), Algo::pid_diff_ahead_s(kPid),
            Algo::pid_diff_ahead_s(kPid), Algo::pid_diff_ahead_s(kPid)}
    {}

    /**
     * @brief Propagates the diagonal velocity observer with acceleration and yaw rate.
     */
    void Chassis::vel_observer_s::Predict(const std::array<float, 3>& acceleration,
        const float gyro_z, const float dt)
    {
        const float vx = x[0], vy = x[1];
        x[0] += (acceleration[0] + gyro_z * vy) * dt;
        x[1] += (acceleration[1] - gyro_z * vx) * dt;
        x[2] = gyro_z;
        for (size_t i = 0; i < x.size(); ++i) covariance[i] += process[i] * dt;
    }

    /**
     * @brief Applies an independent scalar Kalman correction to each velocity component.
     */
    void Chassis::vel_observer_s::Update(const std::array<float, 3>& measurement_value)
    {
        for (size_t i = 0; i < x.size(); ++i)
        {
            const float gain = covariance[i] / (covariance[i] + measurement[i]);
            x[i] += gain * (measurement_value[i] - x[i]);
            covariance[i] *= 1.0F - gain;
        }
    }

    /**
     * @brief Removes gravity and predicts planar velocity entirely in body coordinates.
     */
    void Chassis::PredictImu(const ImuFrame& imu, const float dt)
    {
        VQFPose pose{1.0F, 0.0F, 0.0F, 0.0F};
        if (pose_buffer_ != nullptr) static_cast<void>(pose_buffer_->PeekLatest(pose));
        // VQF [w,x,y,z] rotates body to earth. R^T * earth-up is R's third row.
        const auto [w, x, y, z] = pose;
        const std::array up_body{2.0F * (x * z - w * y), 2.0F * (y * z + w * x),
            1.0F - 2.0F * (x * x + y * y)};
        auto acceleration = imu.accel_m_s2;
        for (size_t i = 0; i < 3; ++i) acceleration[i] -= Math::gravity * up_body[i];
        velocity_observer_.Predict(acceleration, imu.gyro_rad_s[2], dt);
    }

    /**
     * @brief Executes one control tick: reads snapshots, updates targets and drives hardware.
     */
    void Chassis::Update()
    {
        constexpr float kControlPeriodS = 0.002F;
        static constexpr chassis_cmd_s zero_command{};
        const chassis_cmd_s* command = command_buffer_ == nullptr ? &zero_command : command_buffer_->AcquireLatest();
        if (command == nullptr) { command = &zero_command; }  // TODO: why check twice?
        std::array<JointState, 4> joints{};
        for (size_t i = 0; i < 4; ++i)
        {
            if (leg_states_ != nullptr)
            {
                static_cast<void>(leg_states_->ReadLatest(i, joints[i]));
            }
        }
        if (wheel_frames_ != nullptr)
        {
            // Consume the newest frame from each wheel before solving body velocity.
            for (size_t i = 0; i < 4; ++i)
            {
                const auto* frame = wheel_frames_->AcquireLatest(i);
                if (frame != nullptr)
                {
                    wheels_[i].Update(std::span<const uint8_t, 8>{frame->data(), frame->size()},
                           static_cast<uint16_t>(0x201U + i));
                }
                static_cast<void>(wheel_frames_->ReturnLatest());
            }
        }
        VQFPose pose{1.0F, 0.0F, 0.0F, 0.0F};
        if (pose_buffer_ != nullptr) static_cast<void>(pose_buffer_->PeekLatest(pose));
        pitch_est_rad_ += (PitchFromQuaternion(pose) - pitch_est_rad_)
            * Math::Clamp(kControlPeriodS / kPitchFilterTime, 0.0F, 1.0F);
        std::array<JointCommand, 4> commands{};
        UpdateMode(command->mode);
        // Leg targets and wheel drive use the same feedback snapshot for this tick.
        UpdateLegTargets(kControlPeriodS, joints, commands);
        RunDrive(*command, joints);
        for (size_t i = 0; i < 4 && i < legs_.size(); ++i)
        {
            if (joints[i].ready) static_cast<void>(legs_[i].Control(commands[i]));
            else static_cast<void>(legs_[i].HandleUnready());
        }
        if (command_buffer_ != nullptr && command != &zero_command)
            static_cast<void>(command_buffer_->ReturnLatest());
    }

    /**
     * @brief Converts pitch correction into rear-leg x-z trajectory joint targets.
     */
    void Chassis::UpdateLegTargets(const float dt, const std::array<JointState, 4>& joints,
        std::array<JointCommand, 4>& commands)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            commands[i] = {.pos_rad = joints[i].pos_rad};
        }
        if (mode_cur_ == motion_mode_e::flat)
        {
            pitch_est_rad_ = 0.0F;
            for (size_t i = 0; i < 2; ++i)
            {
                commands[i + 2] = {.pos_rad = kRearLegConfig[i].flat_angle};
            }
            return;
        }
        if (mode_cur_ != motion_mode_e::steady) return;

        const float error = pitch_est_rad_ - kPitchSet;
        const float excess = error - Math::Clamp(error, kPitchDeadband);
        const float slew = kRearLegRateMax * dt;

        for (size_t i = 0; i < 2; ++i)
        {
            const size_t joint_index = i + 2;
            const float q_now = joints[joint_index].pos_rad;
            const float target = RearLegAngleForPitch(PosturePitch(q_now, kRearLegConfig[i]) - excess,
                kRearLegConfig[i]);
            const float bounded = Math::Clamp(target, kRearLegConfig[i].min_rad, kRearLegConfig[i].max_rad);
            commands[joint_index] = {.pos_rad = Math::Clamp(bounded, q_now - slew, q_now + slew)};
        }
    }

    /**
     * @brief Applies the legal transitions for the requested chassis motion mode.
     */
    void Chassis::UpdateMode(const motion_mode_e requested)
    {
        switch (mode_cur_)
        {
        case motion_mode_e::flat:
            if (requested == motion_mode_e::steady || requested == motion_mode_e::lift)
                mode_cur_ = requested;
            break;
        case motion_mode_e::steady:
            if (requested == motion_mode_e::flat || requested == motion_mode_e::lift)
                mode_cur_ = requested;
            break;
        case motion_mode_e::lift:
            if (requested == motion_mode_e::climb) mode_cur_ = motion_mode_e::climb;
            else if (requested == motion_mode_e::flat || requested == motion_mode_e::steady)
                mode_cur_ = motion_mode_e::steady;
            break;
        case motion_mode_e::climb:
            if (requested == motion_mode_e::retract) mode_cur_ = motion_mode_e::retract;
            else if (requested == motion_mode_e::flat || requested == motion_mode_e::steady)
                mode_cur_ = motion_mode_e::steady;
            break;
        case motion_mode_e::retract:
            if (requested == motion_mode_e::flat || requested == motion_mode_e::steady)
                mode_cur_ = motion_mode_e::steady;
            break;
        }
    }

    /**
     * @brief Builds force requests for full four-wheel body velocity control.
     */
    void Chassis::BuildFullDriveRequest(const geometry_s& geometry, const chassis_cmd_s& command,
        wheel_drive_request_s& request)
    {
        const std::array speed{wheels_[0].GetSpeed(), wheels_[1].GetSpeed(),
            wheels_[2].GetSpeed(), wheels_[3].GetSpeed()};
        request.active.fill(true);
        request.power_limited = power_limit_ != std::numeric_limits<float>::infinity();
        request.power_budget = power_limit_;
        std::array<float, 3> rhs{};
        for (size_t i = 0; i < 4; ++i)
        {
            const float u = kMotorForwardSign[i] * speed[i] - geometry.Compensation(i);
            rhs[0] += u;
            rhs[1] += kRollerSign[i] * u;
            rhs[2] += geometry.lever[i] * u;
        }
        // The three right-hand sums encode longitudinal, lateral and yaw wheel motion.
        const auto observed = geometry.Solve(rhs);
        velocity_observer_.Update(observed);
        speed_cur_ = velocity_observer_.x;

        auto& body = request.body_pid;
        std::array<float, 3> body_bias{}, body_slope{};
        for (size_t j = 0; j < 3; ++j)
        {
            body[j] = EvaluatePid(pid_speed_[j], 0.0F,
                j == 0 ? command.vx : (j == 1 ? command.vy : command.wz), speed_cur_[j]);
            const float inertia = j == 2 ? kYawInertia : kMass;
            body_bias[j] = body[j].bias * inertia;
            body_slope[j] = body[j].slope * inertia;
        }
        const auto bias_solution = geometry.Solve(body_bias);
        const auto slope_solution = geometry.Solve(body_slope);
        // First pass finds the minimum feasible force and each wheel's extra demand.
        for (size_t i = 0; i < 4; ++i)
        {
            const float sign = kMotorForwardSign[i];
            const float target = sign * (command.vx + kRollerSign[i] * command.vy
                + geometry.lever[i] * command.wz);
            request.wheel_target[i] = target;
            request.wheel_pid[i] = EvaluatePid(pid_wheel_[i],
                sign * geometry.Compensation(i), target, speed[i]);
            const auto [bias, slope] = request.wheel_pid[i];
            request.force[i] = {.bias = bias + sign * geometry.Force(bias_solution, i),
                .slope = slope + sign * geometry.Force(slope_solution, i)};
        }

        if (SolveScale(request.force, speed, kPowerParams, power_limit_, request.scale))
        {
            for (auto& [bias, slope] : request.force) { slope *= request.scale; }
            request.common_scale = true;
            request.power_limited = false;
        }
    }

    /**
     * @brief Builds reduced contact-mode force requests and applies mode weights.
     */
    void Chassis::BuildContactDriveRequest(const geometry_s& geometry, const chassis_cmd_s& command,
        wheel_drive_request_s& request)
    {
        request.active.fill(true);
        if (mode_cur_ == motion_mode_e::lift)
        {
            request.active[0] = false;
            request.active[1] = false;
        }
        request.extra_weight = mode_cur_ == motion_mode_e::retract
            ? kRetractPowerWeights : kLiftClimbPowerWeights;
        request.power_limited = true;
        request.power_budget = kContactPowerLimit;
        for (size_t i = 0; i < 4; ++i)
        {
            if (!request.active[i])
            {
                request.force[i] = {};
                continue;
            }
            const float sign = kMotorForwardSign[i];
            const float target = sign * (command.vx + geometry.lever[i] * command.wz);
            request.wheel_target[i] = target;
            request.wheel_pid[i] = EvaluatePid(pid_wheel_[i], sign * geometry.Compensation(i),
                target, wheels_[i].GetSpeed());
            request.force[i] = request.wheel_pid[i];
        }
    }

    /**
     * @brief Allocates surplus power, redistributes unused shares and serializes wheel forces.
     */
    std::array<float, 4> Chassis::ApplyPowerLimit(const wheel_drive_request_s& request, std::array<uint8_t, 8>& frame)
    {
        frame.fill(0);
        std::array<float, 4> output{};
        std::array<float, 4> requested{};
        std::array<float, 4> minimum_force{};
        std::array<float, 4> minimum_power{};
        std::array<float, 4> extra_need{};
        float minimum_sum = 0.0F;
        // First pass finds the minimum feasible force and each wheel's extra demand.
        for (size_t i = 0; i < 4; ++i)
        {
            if (!request.active[i]) continue;
            const float speed = wheels_[i].GetSpeed();
            const auto& param = kPowerParams[i];
            requested[i] = Math::Clamp(request.force[i].bias + request.force[i].slope, kMaxWheelForce);
            const float power_linear = param.speed_force * speed;
            minimum_force[i] = param.force_squared > 0.0F
                ? -power_linear / (2.0F * param.force_squared)
                : (power_linear != 0.0F ? -std::copysign(kMaxWheelForce, power_linear) : requested[i]);
            minimum_force[i] = Math::Clamp(minimum_force[i], kMaxWheelForce);
            minimum_power[i] = std::max(0.0F, EvaluateWheelPower(param, speed, minimum_force[i]));
            extra_need[i] = std::max(0.0F, EvaluateWheelPower(param, speed, requested[i]) - minimum_power[i]);
            minimum_sum += minimum_power[i];
        }
        power_budget_satisfied_ = !request.power_limited || minimum_sum <= request.power_budget;
        std::array<float, 4> allocation{};
        float remaining = request.power_limited ? std::max(0.0F, request.power_budget - minimum_sum) : 0.0F;
        const bool contact = mode_cur_ != motion_mode_e::flat && mode_cur_ != motion_mode_e::steady;
        // A capped share is redistributed next pass. At least one wheel finishes
        // per pass unless all remaining power is allocated, so four passes suffice.
        for (size_t pass = 0; pass < 4 && remaining > 0.0F; ++pass)
        {
            float weight_sum = 0.0F;
            for (size_t i = 0; i < 4; ++i)
                if (request.active[i] && allocation[i] < extra_need[i])
                    weight_sum += contact ? request.extra_weight[i] : extra_need[i];
            if (weight_sum == 0.0F) break;
            const float available = remaining;
            float distributed = 0.0F;
            for (size_t i = 0; i < 4; ++i)
            {
                if (!request.active[i] || allocation[i] >= extra_need[i]) continue;
                const float weight = contact ? request.extra_weight[i] : extra_need[i];
                const float share = std::min(extra_need[i] - allocation[i], available * (weight / weight_sum));
                allocation[i] += share;
                distributed += share;
            }
            remaining = std::max(0.0F, available - distributed);
        }
        for (size_t i = 0; i < 4; ++i)
        {
            if (!request.active[i]) continue;
            output[i] = request.power_limited ? minimum_force[i] : requested[i];
            if (request.power_limited && power_budget_satisfied_)
            {
                if (allocation[i] >= extra_need[i])
                {
                    output[i] = requested[i];
                }
                else if (allocation[i] > 0.0F)
                {
                    float force_low = -kMaxWheelForce, force_high = kMaxWheelForce;
                    if (IntersectPowerBudget(kPowerParams[i], wheels_[i].GetSpeed(),
                        minimum_power[i] + allocation[i], force_low, force_high))
                        output[i] = std::clamp(requested[i], force_low, force_high);
                }
            }
            output[i] = Math::Clamp(output[i], kMaxWheelForce);
            const auto bits = static_cast<uint16_t>(wheels_[i].ForceToCommand(output[i]));
            frame[2 * i] = static_cast<uint8_t>(bits >> 8);
            frame[2 * i + 1] = static_cast<uint8_t>(bits);
        }
        return output;
    }

    /**
     * @brief Runs the main wheel-control path from geometry to the output frame.
     */
    void Chassis::RunDrive(const chassis_cmd_s& command, const std::array<JointState, 4>& joints)
    {
        std::array<uint8_t, 8> frame{};
        geometry_s geometry{};
        wheel_drive_request_s request{};
        if (!geometry.Update(joints, pitch_est_rad_))
        {
            SendFrame(frame);
            return;
        }
        const bool full_drive = mode_cur_ == motion_mode_e::flat || mode_cur_ == motion_mode_e::steady;
        if (full_drive)
        {
            BuildFullDriveRequest(geometry, command, request);
        }
        else
        {
            BuildContactDriveRequest(geometry, command, request);
        }
        const auto output = ApplyPowerLimit(request, frame);
        // Commit only after the executed forces are known. A fallback has no
        // common target scale: freeze integrals, but advance feedback history.
        const std::array body_target{command.vx, command.vy, command.wz};
        if (full_drive)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                if (request.common_scale)
                    CommitPid(pid_speed_[j], request.body_pid[j], 0.0F, body_target[j], speed_cur_[j], request.scale);
                else
                {
                    pid_speed_[j].error = body_target[j] - speed_cur_[j];
                    pid_speed_[j].feedback_prev = speed_cur_[j];
                }
            }
        }
        for (size_t i = 0; i < 4; ++i)
        {
            const float speed = wheels_[i].GetSpeed();
            if (request.active[i] && (request.common_scale || (!full_drive
                && output[i] == request.force[i].bias + request.force[i].slope)))
            {
                CommitPid(pid_wheel_[i], request.wheel_pid[i], kMotorForwardSign[i] * geometry.Compensation(i),
                    request.wheel_target[i], speed, request.scale);
            }
            else
            {
                pid_wheel_[i].error = request.active[i]
                    ? kMotorForwardSign[i] * geometry.Compensation(i) + request.wheel_target[i] - speed : 0.0F;
                pid_wheel_[i].feedback_prev = speed;
                // In full drive the output also contains body-PID contributions;
                // it cannot be attributed to the wheel PID. Keep its last output.
                if (!full_drive) pid_wheel_[i].output = output[i];
            }
        }
        SendFrame(frame);
    }

    /** @brief Sends or stages the standard eight-byte wheel command frame. */
    void Chassis::SendFrame(const std::array<uint8_t, 8>& frame)
    {
        // TODO: send one standard CAN frame, ID 0x200, DLC 8.
        static_cast<void>(frame);
    }
}
