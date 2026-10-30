#include "chassis_thread.hpp"

#include "libxr.hpp"

namespace JLUni
{
    namespace
    {
        constexpr size_t imu_queue_capacity = 32U;
        constexpr float imu_period_s = 0.0005F;
    }

    /**
     * @brief Drains timestamped IMU samples, predicts velocity and updates the chassis.
     * @param args Thread-owned controller and IMU topic references.
     * @note Wheel and leg feedback/control remain owned by Chassis; this thread only schedules them.
     */
    void ChassisThread(ChassisThreadArgs* const args)
    {
        if (args == nullptr || args->imu_topic == nullptr)
        {
            for (;;)
            {
                LibXR::Thread::Sleep(UINT32_MAX);
            }
        }

        LibXR::SPSCQueue<LibXR::Topic::Message<ImuFrame>> imu_queue(imu_queue_capacity);
        const auto imu_subscriber = LibXR::Topic::QueuedSubscriber(*args->imu_topic, imu_queue);
        LibXR::MicrosecondTimestamp previous_timestamp;
        bool have_timestamp = false;
        LibXR::MillisecondTimestamp wakeup(LibXR::Thread::GetTime());

        static_cast<void>(imu_subscriber);

        for (;;)
        {
            LibXR::Topic::Message<ImuFrame> message {};
            // Process every queued sample so the observer catches up before control output.
            while (imu_queue.Pop(message) == LibXR::ErrorCode::OK)
            {
                // Use the producer timestamp to preserve the actual IMU integration interval.
                const float dt = have_timestamp
                    ? (message.timestamp - previous_timestamp).ToSecondf() : imu_period_s;
                args->chassis.PredictImu(message.data, dt);
                previous_timestamp = message.timestamp;
                have_timestamp = true;
            }

            // Chassis consumes the latest wheel, leg and pose snapshots in one control tick.
            args->chassis.Update();
            // Keep the loop on a stable two-millisecond schedule.
            LibXR::Thread::SleepUntil(wakeup, 2U);
        }
    }
}
