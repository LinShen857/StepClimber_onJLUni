#pragma once

#include "can.hpp"
#include "libxr.hpp"

#include <concepts>
#include <cstdint>
#include <atomic>
#include <type_traits>
#include <span>

namespace JLUni::MotorDM_N
{
    enum class err_state_e : uint8_t
    {
        DISABLED = 0x00,
        ENABLED = 0x01,
        OVER_VOLTAGE = 0x08,
        UNDER_VOLTAGE = 0x09,
        OVER_CURRENT = 0x0A,
        OVER_TEMP_MOS = 0x0B,
        OVER_TEMP_ROTOR = 0x0C,
        COMMS_LOST = 0x0D,
        OVERLOAD = 0x0E,
    };

    enum class cmd_byte_e : uint8_t
    {
        clear_err = 0xFB,
        enable = 0xFC,
        disable = 0xFD,
        set_zero = 0xFE,
    };

    enum class param_e : uint8_t
    {
        UV_VALUE = 0x00,
        KT_VALUE = 0x01,
        OT_VALUE = 0x02,
        OC_VALUE = 0x03,
        ACC = 0x04,
        DEC = 0x05,
        MAX_SPEED = 0x06,
        MASTER_ID = 0x07,
        ESC_ID = 0x08,
        TIMEOUT = 0x09,
        CONTROL_MODE = 0x0A,
        DAMP = 0x0B,
        INERTIA = 0x0C,
        HW_VERSION = 0x0D,
        SOFTWARE_VERSION = 0x0E,
        SN = 0x0F,
        NPP = 0x10,
        RS = 0x11,
        LS = 0x12,
        FLUX = 0x13,
        GR = 0x14,
        PMAX = 0x15,
        VMAX = 0x16,
        TMAX = 0x17,
        I_BW = 0x18,
        KP_ASR = 0x19,
        KI_ASR = 0x1A,
        KP_APR = 0x1B,
        KI_APR = 0x1C,
        OV_VALUE = 0x1D,
        GREF = 0x1E,
        DETA = 0x1F,
        V_BW = 0x20,
        IQ_C1 = 0x21,
        VL_C1 = 0x22,
        CAN_BR = 0x23,
        SUB_VERSION = 0x24,
        BOOT_VERSION = 0x25,
        U_OFF = 0x32,
        V_OFF = 0x33,
        K1 = 0x34,
        K2 = 0x35,
        DIR = 0x37,
        M_OFF = 0x38,
        IMAX = 0x3B,
        VBUS = 0x3C,
        TPCB = 0x3D,
        TMTR = 0x3E,
        IU_OFF = 0x3F,
        IV_OFF = 0x40,
        IW_OFF = 0x41,
        PM = 0x50,
        XOUT = 0x51,
        NONE = 0xFF  // no conflict yet
    };

    struct data_range_s
    {
        float pos_max_rad = 0.0F;
        float vel_max_rad_s = 0.0F;
        float tq_max_nm = 0.0F;
    };

    struct scale_fac_s
    {
        float pos_to_raw;
        float vel_to_raw;
        float tq_to_raw;
        float raw_to_pos;
        float raw_to_vel;
        float raw_to_tq;
    };

    consteval scale_fac_s CalcMotorScales(const data_range_s& limits)
    {
        return
        {
            .pos_to_raw = 32767.5F / limits.pos_max_rad,
            .vel_to_raw = 2047.5F / limits.vel_max_rad_s,
            .tq_to_raw = 2047.5F / limits.tq_max_nm,
            .raw_to_pos = limits.pos_max_rad / 32767.5F,
            .raw_to_vel = limits.vel_max_rad_s / 2047.5F,
            .raw_to_tq = limits.tq_max_nm / 2047.5F
        };
    }

    struct feedback_s
    {
        err_state_e error = err_state_e::DISABLED;
        float pos_rad = 0.0F;
        float vel_rad_s = 0.0F;
        float tq_nm = 0.0F;
        int8_t temp_mos_c = 0;
        int8_t temp_rotor_c = 0;
    };

    enum class param_op_e : uint8_t
    {
        NONE = 0,
        READ = 0x33,
        WRITE = 0x55,
        SAVE = 0xAA
    };

    enum class param_type_e : uint8_t
    {
        UINT32,
        FLOAT32
    };

    struct param_res_s
    {
        uint32_t raw = 0;
        param_e param = param_e::NONE;
        param_type_e type = param_type_e::FLOAT32;
    };

    enum class update_result_e : uint8_t
    {
        NONE,
        FEEDBACK,
        PARAMETER
    };
}  // namespace JLUni::MotorDM_N

namespace JLUni
{
    template <typename Bus, bool UseFd>
    concept MotorBus = std::derived_from<Bus, LibXR::CAN> && (!UseFd || std::derived_from<Bus, LibXR::FDCAN>);

    template <typename Bus, bool UseFd> requires MotorBus<Bus, UseFd> class Motor_DM
    {
    public:
        using pack_t = std::conditional_t<UseFd, LibXR::FDCAN::FDPack, LibXR::CAN::ClassicPack>;

        Motor_DM(Bus& can, uint16_t motor_id, const MotorDM_N::scale_fac_s& scales);
        Motor_DM(const Motor_DM&) = delete;
        Motor_DM& operator=(const Motor_DM&) = delete;
        Motor_DM(Motor_DM&&) = delete;
        Motor_DM& operator=(Motor_DM&&) = delete;

        MotorDM_N::update_result_e UpdateFrame(std::span<const uint8_t> payload,
            MotorDM_N::feedback_s& feedback, MotorDM_N::param_res_s* parameter = nullptr) noexcept;
        [[nodiscard]] MotorDM_N::err_state_e GetErrorState() const noexcept;
        LibXR::ErrorCode CtrlMIT(float pos, float vel, uint16_t kp_raw, uint16_t kd_raw, float tq_ff) const;
        LibXR::ErrorCode CtrlPosVel(float pos, float vel) const;
        LibXR::ErrorCode CtrlVel(float vel) const;
        LibXR::ErrorCode SendCmd(MotorDM_N::cmd_byte_e cmd) const;
        LibXR::ErrorCode GetParam(MotorDM_N::param_e param);
        LibXR::ErrorCode SetParam(MotorDM_N::param_e param, uint32_t raw);
        LibXR::ErrorCode SaveParam();

    private:
        LibXR::ErrorCode Send(uint32_t id, const uint8_t* data, uint8_t length) const;
        LibXR::ErrorCode WriteParam(MotorDM_N::param_op_e op, uint8_t param, const uint8_t* data, uint8_t length);

        Bus& can_;
        uint16_t motor_id_;
        const MotorDM_N::scale_fac_s& scales_;
        std::atomic<uint8_t> pending_op_{static_cast<uint8_t>(MotorDM_N::param_op_e::NONE)};
        MotorDM_N::param_e pending_param_ = MotorDM_N::param_e::NONE;
        std::atomic<MotorDM_N::err_state_e> error_state_{MotorDM_N::err_state_e::DISABLED};
    };

    extern template class Motor_DM<LibXR::CAN, false>;  // enforce a single instantiation
    extern template class Motor_DM<LibXR::FDCAN, false>;
    extern template class Motor_DM<LibXR::FDCAN, true>;
}  // namespace JLUni

