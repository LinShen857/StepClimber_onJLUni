#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "libxr.hpp"
#include "spi.hpp"

namespace JLUni
{
    class WS2812
    {
        public:
        static constexpr size_t FRAME_SIZE = 128;
        explicit WS2812(LibXR::SPI& spi);
        LibXR::ErrorCode SetRGB(uint8_t red, uint8_t green, uint8_t blue);

        private:
        LibXR::SPI& spi_;
        std::array<uint8_t, FRAME_SIZE> frame_ {};
    };
}  // namespace JLUni
