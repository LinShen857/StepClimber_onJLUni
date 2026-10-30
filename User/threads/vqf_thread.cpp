#include "vqf_thread.hpp"

#include "bmi088.hpp"
#include "libxr.hpp"
#include "vqf.hpp"

namespace
{
    constexpr size_t imu_queue_capacity = 32U;
    constexpr vqf_real_t gyro_sample_period_s = 0.0005F;

    VQFParams InitVQFParams()
    {
        VQFParams params;
        params.magDistRejectionEnabled = false;
        params.motionBiasEstEnabled = true;
        params.restBiasEstEnabled = true;
        return params;
    }
}

namespace JLUni
{
    void VQFThread(VQFThreadArgs* const args)
    {
        if (args == nullptr || args->imu_topic == nullptr || args->pose_buffer == nullptr)
        {
            for (;;)
            {
                LibXR::Thread::Sleep(UINT32_MAX);
            }
        }

        LibXR::SPSCQueue<LibXR::Topic::Message<ImuFrame>> imu_queue(imu_queue_capacity);
        auto imu_subscriber = LibXR::Topic::QueuedSubscriber(*args->imu_topic, imu_queue);
        VQF vqf(InitVQFParams(), gyro_sample_period_s, gyro_sample_period_s);
        VQFPose pose {};

        static_cast<void>(imu_subscriber);

        for (;;)
        {
            LibXR::Topic::Message<ImuFrame> message {};

            while (imu_queue.Pop(message) == LibXR::ErrorCode::OK)
            {
                vqf.update(message.data.gyro_rad_s.data(), message.data.accel_m_s2.data());
                vqf.getQuat6D(pose.data());
                static_cast<void>(args->pose_buffer->Publish(pose));
            }

            LibXR::Thread::Sleep(1U);
        }
    }
}
