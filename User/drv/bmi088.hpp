#pragma once

#include <array>

#include "gpio.hpp"
#include "spi.hpp"
#include "libxr.hpp"

namespace JLUni
{
    struct ImuFrame
    {
        // BMI088 raw axes are mounted to the chassis frame: x forward, y left, z up.
        std::array<float, 3> gyro_rad_s;
        std::array<float, 3> accel_m_s2;
    };

    class BMI088
    {
        public:
        BMI088(LibXR::SPI& spi, LibXR::GPIO& accel_cs, LibXR::GPIO& gyro_cs,
            LibXR::GPIO& accel_drdy, LibXR::GPIO& gyro_drdy);

        LibXR::Topic& GetSampleTopic() { return sample_topic_; }

        private:
        enum class Transfer : uint8_t
        {
            NONE,
            ACCEL,
            GYRO,
        };

        void InitAccel() const;
        void InitGyro() const;
        bool ReadAccelRegister(uint8_t reg, uint8_t& value) const;
        bool ReadGyroRegister(uint8_t reg, uint8_t& value) const;
        bool WriteAccelRegister(uint8_t reg, uint8_t value) const;
        bool WriteGyroRegister(uint8_t reg, uint8_t value) const;

        bool TryTransfer(bool in_isr, LibXR::MicrosecondTimestamp timestamp);
        void PublishCompletedTransfer(bool in_isr);
        void Deselect(Transfer transfer) const;

        static void OnDataReady(bool in_isr, BMI088* self);
        static void OnTransferComplete(bool in_isr, BMI088* self, LibXR::ErrorCode result);

        LibXR::SPI& spi_;
        LibXR::GPIO& accel_cs_;
        LibXR::GPIO& gyro_cs_;

        LibXR::Topic sample_topic_;
        LibXR::WriteOperation::Callback transfer_callback_;

        std::array<uint8_t, 19> rx_frame_ {};
        std::array<float, 3> gyro_frame_rad_s_ {};
        bool gyro_frame_ready_ = false;
        bool active_temperature_read_ = false;
        uint8_t data_ready_count_ = 0U;
        float temperature_celsius_ = 0.0F;
        uint16_t transfer_errors_ = 0U;

        // These fields are shared only by the gyro EXTI and SPI completion ISR.
        Transfer active_transfer_ = Transfer::NONE;
        uint8_t tx_command_ = 0U;
        LibXR::MicrosecondTimestamp active_timestamp_ {};
    };
}
