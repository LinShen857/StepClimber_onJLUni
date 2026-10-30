#pragma once

#include "advanced_pid.hpp"
#include "bmi088.hpp"
#include "joint.hpp"
#include "last_buffer.hpp"
#include "last_slot_set.hpp"
#include "vqf_thread.hpp"
#include "wheel_3508.hpp"

#include <array>
#include <span>


namespace JLUni::Chassis_N
{
    enum class motion_mode_e : uint8_t
    {
        flat,
        lift,
        climb,
        retract,
        steady
    };

    struct chassis_cmd_s
    {
        // Body-frame command: x forward, y left, angular velocity about body z.
        float vx = 0.0F, vy = 0.0F, wz = 0.0F;
        motion_mode_e mode = motion_mode_e::flat;
    };

    struct pid_affine_s { float bias = 0.0F, slope = 0.0F; };

    struct geometry_s
    {
        std::array<float, 4> lever{};
        std::array<float, 2> xdot{};
        float mean = 0.0F;
        float signed_mean = 0.0F;
        float inverse_residual = 0.0F;

        /**
         * @brief Builds wheel lever arms and rear-leg forward-speed compensation.
         */
        bool Update(const std::array<JointState, 4>& joints, float pitch_rad);
        /**
         * @brief Solves wheel velocities for body velocity [vx, vy, wz].
         */
        std::array<float, 3> Solve(const std::array<float, 3>& rhs) const;
        /**
         * @brief Maps a body force/moment solution to one wheel force.
         */
        float Force(const std::array<float, 3>& solution, std::size_t i) const;
        /**
         * @brief Returns forward wheel-speed compensation caused by rear-leg motion.
         */
        float Compensation(std::size_t i) const;
    };

    class Chassis
    {
    public:
        /**
         * @brief Creates a chassis controller over shared command, feedback, pose and leg buffers.
         */
        Chassis(Algo::LastBuffer<chassis_cmd_s>* command_buffer, Algo::LastSlotSet<JointState, 4>* leg_buffer,
            Algo::LastSlotSet<std::array<uint8_t, 8>, 4>* wheel_buffer, Algo::LastBuffer<VQFPose>* pose_buffer,
            std::span<Joint, 4> legs);

        /**
         * @brief Predicts chassis velocity from one timestamped IMU sample.
         */
        void PredictImu(const ImuFrame& imu, float dt);

        /**
         * @brief Runs one 2 ms chassis control cycle and sends leg and wheel outputs.
         */
        void Update();

        // Latest modeled wheel-power result; read from the chassis task only.
        bool IsPowerBudgetSatisfied() const { return power_budget_satisfied_; }

    private:
        struct wheel_drive_request_s
        {
            std::array<pid_affine_s, 4> force{};
            std::array<bool, 4> active{};
            std::array<float, 4> extra_weight{};
            std::array<pid_affine_s, 3> body_pid{};
            std::array<pid_affine_s, 4> wheel_pid{};
            std::array<float, 4> wheel_target{};
            float scale = 1.0F;
            bool common_scale = false;
            bool power_limited = true;
            float power_budget = 0.0F;
        };

        /**
         * @brief Advances the chassis motion-mode state machine.
         */
        void UpdateMode(motion_mode_e requested);

        /**
         * @brief Builds a full four-wheel drive request for flat or steady motion.
         */
        void BuildFullDriveRequest(const geometry_s& geometry, const chassis_cmd_s& command,
            wheel_drive_request_s& request);

        /**
         * @brief Builds a contact-mode drive request with mode-specific wheel weights.
         */
        void BuildContactDriveRequest(const geometry_s& geometry, const chassis_cmd_s& command,
            wheel_drive_request_s& request);

        /**
         * @brief Runs geometry, drive-request, power-limit and CAN-frame generation.
         */
        void RunDrive(const chassis_cmd_s& command, const std::array<JointState, 4>& joints);

        /**
         * @brief Converts requested wheel forces into a power-limited CAN frame.
         */
        std::array<float, 4> ApplyPowerLimit(const wheel_drive_request_s& request,
            std::array<uint8_t, 8>& frame);

        /**
         * @brief Generates rear-leg joint targets that compensate chassis pitch.
         */
        void UpdateLegTargets(float dt, const std::array<JointState, 4>& joints,
            std::array<JointCommand, 4>& commands);

        /**
         * @brief Sends one eight-byte wheel command frame to the chassis CAN bus.
         */
        void SendFrame(const std::array<uint8_t, 8>& frame);

        struct vel_observer_s
        {
            std::array<float, 3> x{};  // Body-frame [vx, vy, gyro_z], planar velocity model.
            std::array<float, 3> covariance{{0.1F, 0.1F, 0.1F}};
            std::array<float, 3> process{{0.8F, 0.8F, 1.2F}};
            std::array<float, 3> measurement{{0.04F, 0.04F, 0.08F}};

            /**
             * @brief Propagates [vx, vy, wz] and diagonal covariance with IMU data.
             */
            void Predict(const std::array<float, 3>& acceleration, float gyro_z, float dt);

            /**
             * @brief Corrects [vx, vy, wz] with the wheel-geometry measurement.
             */
            void Update(const std::array<float, 3>& measurement_value);
        } velocity_observer_;

        motion_mode_e mode_cur_ = motion_mode_e::flat;  // shouldn't be changed outside of UpdateMode()
        Algo::LastBuffer<chassis_cmd_s>* command_buffer_;  // for command
        Algo::LastSlotSet<JointState, 4>* leg_states_;  // read leg joint states
        Algo::LastSlotSet<std::array<uint8_t, 8>, 4>* wheel_frames_;  // store wheel feedback frames
        Algo::LastBuffer<VQFPose>* pose_buffer_;  // for pose
        std::span<Joint, 4> legs_;  // leg joints
        std::array<Wheel, 4> wheels_;  // store wheel states
        std::array<Algo::pid_diff_ahead_s, 3> pid_speed_;
        std::array<Algo::pid_diff_ahead_s, 4> pid_wheel_;
        std::array<float, 3> speed_cur_{};  // observed chassis speed
        float pitch_est_rad_ = 0.0F;
        // Four wheel motors' net bus power budget (W), excluding joints/other loads.
        float power_limit_ = 200.0F;  // will be updated by upper layer
        bool power_budget_satisfied_ = true;
    };
}
