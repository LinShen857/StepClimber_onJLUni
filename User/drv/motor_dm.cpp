#include "motor_dm.hpp"

#include <cstdint>
#include <cstring>

namespace JLUni
{
    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> Motor_DM<Bus, UseFd>::Motor_DM(Bus& can, const uint16_t motor_id,
        const MotorDM_N::scale_fac_s& scales)
        : can_(can), motor_id_(motor_id), scales_(scales)
    {}

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::Send(const uint32_t id,
        const uint8_t* const data, const uint8_t length) const
    {
        if constexpr (UseFd)
        {
            LibXR::FDCAN::FDPack pack{};
            pack.id = id;
            pack.type = LibXR::CAN::Type::STANDARD;
            pack.len = length;
            std::memcpy(pack.data, data, length);
            return can_.AddMessage(pack);
        }
        else
        {
            LibXR::CAN::ClassicPack pack{};
            pack.id = id;
            pack.type = LibXR::CAN::Type::STANDARD;
            pack.dlc = length;
            std::memcpy(pack.data, data, length);
            return can_.AddMessage(pack);
        }
    }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::CtrlMIT(const float pos, const float vel,
        const uint16_t kp_raw, const uint16_t kd_raw, const float tq_ff) const
    {
        const auto pos_raw = static_cast<uint16_t>(pos * scales_.pos_to_raw + 32767.5F);
        const auto vel_raw = static_cast<uint16_t>(vel * scales_.vel_to_raw + 2047.5F);
        const auto tq_raw = static_cast<uint16_t>(tq_ff * scales_.tq_to_raw + 2047.5F);
        const uint8_t data[8]
        {
            static_cast<uint8_t>(pos_raw >> 8U), static_cast<uint8_t>(pos_raw),
            static_cast<uint8_t>(vel_raw >> 4U),
            static_cast<uint8_t>((vel_raw & 0x0FU) << 4U | ((kp_raw >> 8U) & 0x0FU)), static_cast<uint8_t>(kp_raw),
            static_cast<uint8_t>(kd_raw >> 4U), static_cast<uint8_t>((kd_raw & 0x0FU) << 4U | ((tq_raw >> 8U) & 0x0FU)),
            static_cast<uint8_t>(tq_raw)
        };
        return Send(motor_id_, data, 8U);
    }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::CtrlPosVel(const float pos,
        const float vel) const
    {
        uint8_t data[8];
        std::memcpy(data, &pos, sizeof(pos));
        std::memcpy(data + sizeof(pos), &vel, sizeof(vel));
        return Send(static_cast<uint32_t>(motor_id_) + 0x100U, data, 8U);
    }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::CtrlVel(const float vel) const
    {
        uint8_t data[sizeof(float)];
        std::memcpy(data, &vel, sizeof(vel));
        return Send(static_cast<uint32_t>(motor_id_) + 0x200U, data, sizeof(data));
    }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::SendCmd(const MotorDM_N::cmd_byte_e cmd) const
    {
        const uint8_t data[8]{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, static_cast<uint8_t>(cmd)};
        return Send(motor_id_, data, 8U);
    }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> MotorDM_N::err_state_e Motor_DM<Bus, UseFd>::GetErrorState() const noexcept
    { return error_state_.load(std::memory_order_acquire); }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::WriteParam(const MotorDM_N::param_op_e op,
        const uint8_t param, const uint8_t* const data, const uint8_t length)
    {
        if (pending_op_.load(std::memory_order_acquire) != static_cast<uint8_t>(MotorDM_N::param_op_e::NONE))
        {
            return LibXR::ErrorCode::BUSY;
        }
        pending_param_ = op == MotorDM_N::param_op_e::SAVE ? MotorDM_N::param_e::NONE : static_cast<MotorDM_N::param_e>(param);
        pending_op_.store(static_cast<uint8_t>(op), std::memory_order_release);
        const auto result = Send(0x7FFU, data, length);
        if (result != LibXR::ErrorCode::OK)
        {
            pending_param_ = MotorDM_N::param_e::NONE;
            pending_op_.store(static_cast<uint8_t>(MotorDM_N::param_op_e::NONE), std::memory_order_release);
        }
        return result;
    }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::GetParam(const MotorDM_N::param_e param)
    {
        const auto index = static_cast<uint8_t>(param);
        const uint8_t data[4]{static_cast<uint8_t>(motor_id_), static_cast<uint8_t>(motor_id_ >> 8U),
            static_cast<uint8_t>(MotorDM_N::param_op_e::READ), index};
        return WriteParam(MotorDM_N::param_op_e::READ, index, data, 4U);
    }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::SetParam(const MotorDM_N::param_e param,
        const uint32_t raw)
    {
        const auto index = static_cast<uint8_t>(param);
        uint8_t data[8]
        {
            static_cast<uint8_t>(motor_id_), static_cast<uint8_t>(motor_id_ >> 8U),
            static_cast<uint8_t>(MotorDM_N::param_op_e::WRITE), index
        };
        data[4] = static_cast<uint8_t>(raw);
        data[5] = static_cast<uint8_t>(raw >> 8U);
        data[6] = static_cast<uint8_t>(raw >> 16U);
        data[7] = static_cast<uint8_t>(raw >> 24U);
        return WriteParam(MotorDM_N::param_op_e::WRITE, index, data, 8U);
    }

    template <typename Bus, bool UseFd> requires MotorBus<Bus, UseFd> LibXR::ErrorCode Motor_DM<Bus, UseFd>::SaveParam()
    {
        const uint8_t data[4]
        {
            static_cast<uint8_t>(motor_id_), static_cast<uint8_t>(motor_id_ >> 8U),
            static_cast<uint8_t>(MotorDM_N::param_op_e::SAVE), 0x01U
        };
        return WriteParam(MotorDM_N::param_op_e::SAVE, 1U, data, 4U);
    }

    template <typename Bus, bool UseFd>
    requires MotorBus<Bus, UseFd> MotorDM_N::update_result_e Motor_DM<Bus, UseFd>::UpdateFrame(
        const std::span<const uint8_t> payload, MotorDM_N::feedback_s& feedback,
        MotorDM_N::param_res_s* const parameter) noexcept
    {
        const auto length = payload.size();
        const auto* data = payload.data();
        const auto pending = static_cast<MotorDM_N::param_op_e>(pending_op_.load(std::memory_order_acquire));
        if (pending != MotorDM_N::param_op_e::NONE)
        {
            const auto param = pending_param_;
            const auto expected_length = pending == MotorDM_N::param_op_e::SAVE ? 4U : 8U;
            const auto expected_param = pending == MotorDM_N::param_op_e::SAVE ? 1U : static_cast<uint8_t>(param);
            if (length == expected_length
                && static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8U)) == motor_id_
                && data[2] == static_cast<uint8_t>(pending) && data[3] == expected_param)
            {
                if (pending == MotorDM_N::param_op_e::READ && parameter != nullptr)
                {
                    const auto index = static_cast<uint8_t>(param);
                    parameter->param = param;
                    parameter->type = ((index >= 0x07U && index <= 0x0AU) || (index >= 0x0DU && index <= 0x10U)
                        || (index >= 0x23U && index <= 0x25U))
                        ? MotorDM_N::param_type_e::UINT32 : MotorDM_N::param_type_e::FLOAT32;
                    parameter->raw = static_cast<uint32_t>(data[4]) | (static_cast<uint32_t>(data[5]) << 8U)
                        | (static_cast<uint32_t>(data[6]) << 16U) | (static_cast<uint32_t>(data[7]) << 24U);
                }
                pending_param_ = MotorDM_N::param_e::NONE;
                pending_op_.store(static_cast<uint8_t>(MotorDM_N::param_op_e::NONE), std::memory_order_release);
                return MotorDM_N::update_result_e::PARAMETER;
            }
        }

        // Parameter replies carry an operation byte, not a feedback status.
        if (length >= 4U && (data[2] == static_cast<uint8_t>(MotorDM_N::param_op_e::READ)
            || data[2] == static_cast<uint8_t>(MotorDM_N::param_op_e::WRITE)
            || data[2] == static_cast<uint8_t>(MotorDM_N::param_op_e::SAVE))
            && static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8U)) == motor_id_)
            return MotorDM_N::update_result_e::NONE;
        if (length != 8U) return MotorDM_N::update_result_e::NONE;

        constexpr float position_mid = 32767.5F;
        constexpr float velocity_mid = 2047.5F;
        const auto position = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8U) | data[2]);
        const auto velocity = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 4U) | (data[4] >> 4U));
        const auto torque = static_cast<uint16_t>(((data[4] & 0x0FU) << 8U) | data[5]);
        feedback =
        {
            .error = static_cast<MotorDM_N::err_state_e>(data[0] >> 4U),
            .pos_rad = (static_cast<float>(position) - position_mid) * scales_.raw_to_pos,
            .vel_rad_s = (static_cast<float>(velocity) - velocity_mid) * scales_.raw_to_vel,
            .tq_nm = (static_cast<float>(torque) - velocity_mid) * scales_.raw_to_tq,
            .temp_mos_c = static_cast<int8_t>(data[6]), .temp_rotor_c = static_cast<int8_t>(data[7])
        };
        error_state_.store(feedback.error, std::memory_order_release);
        return MotorDM_N::update_result_e::FEEDBACK;
    }
    template class Motor_DM<LibXR::CAN, false>;
    template class Motor_DM<LibXR::FDCAN, false>;
    template class Motor_DM<LibXR::FDCAN, true>;
}  // namespace JLUni
