#pragma once

#include <array>

#include "last_buffer.hpp"
#include "topic.hpp"

namespace JLUni
{
    // [w, x, y, z], body-frame vectors rotated into the earth frame by VQF.
    using VQFPose = std::array<float, 4>;

    struct VQFThreadArgs
    {
        LibXR::Topic* imu_topic;
        Algo::LastBuffer<VQFPose>* pose_buffer;
    };

    void VQFThread(VQFThreadArgs* args);
}
