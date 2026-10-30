#include "bmi088.hpp"
#include "math_utils.hpp"

#include <array>

namespace
{
    struct RegisterConfig
    {
        uint8_t reg;
        uint8_t value;
    };

    constexpr uint8_t read_mask = 0x80U;

    constexpr uint8_t acc_chip_id = 0x00U;
    constexpr uint8_t acc_data = 0x12U;
    constexpr uint8_t acc_soft_reset = 0x7EU;
    constexpr uint8_t expected_acc_chip_id = 0x1EU;

    constexpr uint8_t gyro_chip_id = 0x00U;
    constexpr uint8_t gyro_data = 0x02U;
    constexpr uint8_t gyro_soft_reset = 0x14U;
    constexpr uint8_t expected_gyro_chip_id = 0x0FU;

    constexpr RegisterConfig accel_suspend_config {.reg = 0x7CU, .value = 0x00U};

    constexpr RegisterConfig accel_power_enable {.reg = 0x7DU, .value = 0x04U};

    constexpr std::array<RegisterConfig, 4> accel_normal_config {{
        {.reg = 0x40U, .value = 0xACU},  // ACC_CONF: 1600 Hz ODR, Normal bandwidth.
        {.reg = 0x41U, .value = 0x03U},
        {.reg = 0x53U, .value = 0x08U},
        {.reg = 0x58U, .value = 0x04U},
    }};

    constexpr std::array<RegisterConfig, 5> gyro_normal_config {{
        {.reg = 0x0FU, .value = 0x00U},  // GYRO_RANGE: +/-2000 dps.
        {.reg = 0x10U, .value = 0x80U},  // GYRO_BANDWIDTH: 2 kHz ODR, maximum 532 Hz bandwidth.
        {.reg = 0x15U, .value = 0x80U},  // GYRO_INT_CTRL: enable the new-data interrupt.
        {.reg = 0x16U, .value = 0x0CU},  // INT3_INT4_IO_CONF: INT3 push-pull and active low.
        {.reg = 0x18U, .value = 0x01U},  // INT3_INT4_IO_MAP: route new data to INT3.
    }};

    // BMI088 data sheet Rev. 1.9, sections 3, 4.1, 4.8 and 6, define these minimum startup and write delays.
    // Command-specific reset and power-enable delays take precedence over the generic register-write delay.
    constexpr uint32_t power_on_delay_us = 30000U;
    constexpr uint32_t accel_soft_reset_delay_us = 1000U;
    constexpr uint32_t gyro_soft_reset_delay_us = 30000U;
    constexpr uint32_t suspend_mode_write_delay_us = 1000U;
    constexpr uint32_t accel_power_enable_delay_us = 450U;
    constexpr uint32_t normal_mode_write_delay_us = 2U;

    constexpr float accel_scale = 24.0F * JLUni::Math::gravity / 32768.0F;
    constexpr float gyro_scale = 2000.0F * static_cast<float>(LibXR::PI) / (180.0F * 32768.0F);
    constexpr uint8_t temperature_sample_period = 100U;
}

namespace JLUni
{
    bool BMI088::ReadAccelRegister(const uint8_t reg, uint8_t& value) const
    {
        // Every accelerometer SPI read returns one extra dummy byte after the command byte.
        std::array<uint8_t, 3> tx {static_cast<uint8_t>(reg | read_mask), 0U, 0U};
        std::array<uint8_t, 3> rx {};
        LibXR::WriteOperation operation;

        accel_cs_.Write(false);
        const auto result = spi_.ReadAndWrite({rx.data(), rx.size()},
            {tx.data(), tx.size()}, operation);
        accel_cs_.Write(true);
        if (result != LibXR::ErrorCode::OK) { return false; }

        value = rx[2];
        return true;
    }

    bool BMI088::ReadGyroRegister(const uint8_t reg, uint8_t& value) const
    {
        std::array<uint8_t, 2> tx {static_cast<uint8_t>(reg | read_mask), 0U};
        std::array<uint8_t, 2> rx {};
        LibXR::WriteOperation operation;

        gyro_cs_.Write(false);
        const auto result = spi_.ReadAndWrite({rx.data(), rx.size()},
            {tx.data(), tx.size()}, operation);
        gyro_cs_.Write(true);
        if (result != LibXR::ErrorCode::OK) { return false; }

        value = rx[1];
        return true;
    }

    bool BMI088::WriteAccelRegister(const uint8_t reg, const uint8_t value) const
    {
        std::array tx {reg, value};
        LibXR::WriteOperation operation;

        accel_cs_.Write(false);
        const auto result = spi_.Write({tx.data(), tx.size()}, operation);
        accel_cs_.Write(true);
        return result == LibXR::ErrorCode::OK;
    }

    bool BMI088::WriteGyroRegister(const uint8_t reg, const uint8_t value) const
    {
        std::array tx {reg, value};
        LibXR::WriteOperation operation;

        gyro_cs_.Write(false);
        const auto result = spi_.Write({tx.data(), tx.size()}, operation);
        gyro_cs_.Write(true);
        return result == LibXR::ErrorCode::OK;
    }

    void BMI088::InitAccel() const
    {
        uint8_t val = 0U;

        // The first accelerometer read after power-up is invalid and switches its interface from I2C to SPI.
        static_cast<void>(ReadAccelRegister(acc_chip_id, val));

        bool read_ok = false;
        do
        {
            read_ok = ReadAccelRegister(acc_chip_id, val);
        }
        while (!read_ok || val != expected_acc_chip_id);

        bool write_ok = false;
        do
        {
            write_ok = WriteAccelRegister(acc_soft_reset, 0xB6U);
            LibXR::Timebase::DelayMicroseconds(accel_soft_reset_delay_us);
        }
        while (!write_ok);

        // A soft reset restores the interface selection, so another invalid SPI read is required before CHIP_ID.
        static_cast<void>(ReadAccelRegister(acc_chip_id, val));
        do
        {
            read_ok = ReadAccelRegister(acc_chip_id, val);
        }
        while (!read_ok || val != expected_acc_chip_id);

        do
        {
            write_ok = WriteAccelRegister(accel_suspend_config.reg, accel_suspend_config.value);
            LibXR::Timebase::DelayMicroseconds(suspend_mode_write_delay_us);
            read_ok = ReadAccelRegister(accel_suspend_config.reg, val);
        }
        while (!write_ok || !read_ok || val != accel_suspend_config.value);

        do
        {
            write_ok = WriteAccelRegister(accel_power_enable.reg, accel_power_enable.value);
            LibXR::Timebase::DelayMicroseconds(accel_power_enable_delay_us);
            read_ok = ReadAccelRegister(accel_power_enable.reg, val);
        }
        while (!write_ok || !read_ok || val != accel_power_enable.value);

        for (const auto& [reg, value] : accel_normal_config)
        {
            do
            {
                write_ok = WriteAccelRegister(reg, value);
                LibXR::Timebase::DelayMicroseconds(normal_mode_write_delay_us);
                read_ok = ReadAccelRegister(reg, val);
            }
            while (!write_ok || !read_ok || val != value);
        }
    }

    void BMI088::InitGyro() const
    {
        uint8_t val = 0U;
        bool read_ok = false;
        do
        {
            read_ok = ReadGyroRegister(gyro_chip_id, val);
        }
        while (!read_ok || val != expected_gyro_chip_id);

        bool write_ok = false;
        do
        {
            write_ok = WriteGyroRegister(gyro_soft_reset, 0xB6U);
            // The gyroscope must not be accessed during the 30 ms soft-reset interval required by the data sheet.
            LibXR::Timebase::DelayMicroseconds(gyro_soft_reset_delay_us);
        }
        while (!write_ok);

        do
        {
            read_ok = ReadGyroRegister(gyro_chip_id, val);
        }
        while (!read_ok || val != expected_gyro_chip_id);

        for (const auto& [reg, value] : gyro_normal_config)
        {
            do
            {
                write_ok = WriteGyroRegister(reg, value);
                LibXR::Timebase::DelayMicroseconds(normal_mode_write_delay_us);
                read_ok = ReadGyroRegister(reg, val);
            }
            while (!write_ok || !read_ok || val != value);
        }
    }

    void BMI088::Deselect(const Transfer transfer) const
    {
        switch (transfer)
        {
        case Transfer::ACCEL:
            accel_cs_.Write(true);
            break;
        case Transfer::GYRO:
            gyro_cs_.Write(true);
            break;
        case Transfer::NONE:
            break;
        }
    }

    bool BMI088::TryTransfer(const bool in_isr, const LibXR::MicrosecondTimestamp timestamp)
    {
        if (active_transfer_ != Transfer::NONE) { return false; }

        active_transfer_ = Transfer::GYRO;
        active_timestamp_ = timestamp;
        gyro_frame_ready_ = false;
        tx_command_ = gyro_data | read_mask;
        gyro_cs_.Write(false);

        // STM32SPI copies this operation and the command, then zero-fills its internal DMA TX buffer to frame_size.
        LibXR::WriteOperation operation(transfer_callback_);
        const auto result = spi_.ReadAndWrite({rx_frame_.data(), 7U},
            {&tx_command_, 1U}, operation, in_isr);
        if (result == LibXR::ErrorCode::OK) { return true; }

        // An immediate start failure owns no DMA transfer, so release CS and preserve the request for a later ISR.
        Deselect(Transfer::GYRO);
        active_transfer_ = Transfer::NONE;
        active_temperature_read_ = false;
        ++transfer_errors_;
        return false;
    }

    void BMI088::OnDataReady(const bool in_isr, BMI088* const self)
    {
        // The gyroscope data-ready interrupt is the sole BMI088 sampling clock.
        if (self->active_transfer_ != Transfer::NONE) { return; }

        const auto timestamp = LibXR::Timebase::GetMicroseconds();
        const auto next_count = static_cast<uint8_t>(self->data_ready_count_ + 1U);
        self->active_temperature_read_ = next_count >= temperature_sample_period;
        if (self->active_temperature_read_)
        {
            self->data_ready_count_ = 0U;
        }
        else
        {
            self->data_ready_count_ = next_count;
        }
        if (!self->TryTransfer(in_isr, timestamp))
        {
            self->active_temperature_read_ = false;
            if (next_count >= temperature_sample_period)
            {
                self->data_ready_count_ = static_cast<uint8_t>(temperature_sample_period - 1U);
            }
            else
            {
                self->data_ready_count_ = static_cast<uint8_t>(next_count - 1U);
            }
        }
    }

    void BMI088::PublishCompletedTransfer(const bool in_isr)
    {
        switch (active_transfer_)
        {
        case Transfer::ACCEL:
        {
            // Accelerometer payload starts after the command response and its mandatory extra dummy byte.
            const auto x_raw = static_cast<int16_t>(static_cast<uint16_t>(rx_frame_[2])
                | (static_cast<uint16_t>(rx_frame_[3]) << 8U));
            const auto y_raw = static_cast<int16_t>(static_cast<uint16_t>(rx_frame_[4])
                | (static_cast<uint16_t>(rx_frame_[5]) << 8U));
            const auto z_raw = static_cast<int16_t>(static_cast<uint16_t>(rx_frame_[6])
                | (static_cast<uint16_t>(rx_frame_[7]) << 8U));
            if (gyro_frame_ready_)
            {
                ImuFrame frame {
                    .gyro_rad_s = gyro_frame_rad_s_,
                    .accel_m_s2 = {
                        static_cast<float>(x_raw) * accel_scale,
                        static_cast<float>(y_raw) * accel_scale,
                        static_cast<float>(z_raw) * accel_scale,
                    },
                };
                sample_topic_.PublishFromCallback(frame, active_timestamp_, in_isr);
                if (active_temperature_read_)
                {
                    auto raw = static_cast<int16_t>((static_cast<uint16_t>(rx_frame_[17]) << 3U)
                        | (static_cast<uint16_t>(rx_frame_[18]) >> 5U));
                    if ((raw & 0x0400) != 0)
                    {
                        raw = static_cast<int16_t>(raw - 0x0800);
                    }
                    temperature_celsius_ = static_cast<float>(raw) * 0.125F + 23.0F;
                }
                gyro_frame_ready_ = false;
            }
            break;
        }
        case Transfer::GYRO:
        {
            // Gyroscope payload follows the command directly, without the accelerometer's extra dummy byte.
            const auto x_raw = static_cast<int16_t>(static_cast<uint16_t>(rx_frame_[1])
                | (static_cast<uint16_t>(rx_frame_[2]) << 8U));
            const auto y_raw = static_cast<int16_t>(static_cast<uint16_t>(rx_frame_[3])
                | (static_cast<uint16_t>(rx_frame_[4]) << 8U));
            const auto z_raw = static_cast<int16_t>(static_cast<uint16_t>(rx_frame_[5])
                | (static_cast<uint16_t>(rx_frame_[6]) << 8U));
            gyro_frame_rad_s_ = {
                    static_cast<float>(x_raw) * gyro_scale,
                    static_cast<float>(y_raw) * gyro_scale,
                    static_cast<float>(z_raw) * gyro_scale,
            };
            gyro_frame_ready_ = true;
            break;
        }
        case Transfer::NONE:
            break;
        }
    }

    void BMI088::OnTransferComplete(const bool in_isr, BMI088* const self, const LibXR::ErrorCode result)
    {
        const Transfer completed = self->active_transfer_;
        self->Deselect(completed);

        if (result == LibXR::ErrorCode::OK)
        {
            if (completed == Transfer::GYRO)
            {
                self->PublishCompletedTransfer(in_isr);
                if (self->gyro_frame_ready_)
                {
                    self->active_transfer_ = Transfer::ACCEL;
                    self->tx_command_ = acc_data | read_mask;
                    self->accel_cs_.Write(false);
                    LibXR::WriteOperation operation(self->transfer_callback_);
                    const size_t accel_read_size = self->active_temperature_read_ ? 19U : 8U;
                    const auto start_result = self->spi_.ReadAndWrite(
                        {self->rx_frame_.data(), accel_read_size},
                        {&self->tx_command_, 1U}, operation, in_isr);
                    if (start_result == LibXR::ErrorCode::OK) { return; }
                    self->Deselect(Transfer::ACCEL);
                    self->active_transfer_ = Transfer::NONE;
                    self->gyro_frame_ready_ = false;
                    self->active_temperature_read_ = false;
                    ++self->transfer_errors_;
                    return;
                }
            }
            else
            {
                self->PublishCompletedTransfer(in_isr);
            }
        }
        else
        {
            // An asynchronous failure drops only the active sample; the next DRDY resumes normal acquisition.
            if (completed == Transfer::GYRO)
            {
                self->gyro_frame_ready_ = false;
            }
            ++self->transfer_errors_;
        }

        self->active_transfer_ = Transfer::NONE;
        self->gyro_frame_ready_ = false;
        self->active_temperature_read_ = false;
    }

        BMI088::BMI088(LibXR::SPI& spi, LibXR::GPIO& accel_cs, LibXR::GPIO& gyro_cs,
        LibXR::GPIO& accel_drdy, LibXR::GPIO& gyro_drdy)
        : spi_(spi), accel_cs_(accel_cs), gyro_cs_(gyro_cs),
            sample_topic_(LibXR::Topic::CreateTopic<ImuFrame>("/imu/bmi088/sample")),
            transfer_callback_(LibXR::WriteOperation::Callback::Create(OnTransferComplete, this))
    {
        LibXR::Timebase::DelayMicroseconds(power_on_delay_us);
        InitAccel();
        InitGyro();
        // Keep the accelerometer DRDY dependency in the public constructor for
        // compatibility, but use the gyroscope DRDY as the sole sampling clock.
        static_cast<void>(accel_drdy);
        static_cast<void>(gyro_drdy.RegisterCallback(LibXR::GPIO::Callback::Create(OnDataReady, this)));
        static_cast<void>(gyro_drdy.EnableInterrupt());
    }
}
