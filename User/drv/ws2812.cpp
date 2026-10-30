#include "ws2812.hpp"

namespace
{
    constexpr uint8_t logical_high = 0x78;
    constexpr uint8_t logical_low = 0x60;
}

namespace JLUni
{
    WS2812::WS2812(LibXR::SPI& spi) : spi_(spi) {}

    LibXR::ErrorCode WS2812::SetRGB(const uint8_t red, const uint8_t green, const uint8_t blue)
    {
        frame_.fill(0);
        for (size_t bit = 0; bit < 8; ++bit)
        {
            const auto mask = static_cast<uint8_t>(0x80U >> bit);
            frame_[bit] = (green & mask) != 0U ? logical_high : logical_low;
            frame_[8 + bit] = (red & mask) != 0U ? logical_high : logical_low;
            frame_[16 + bit] = (blue & mask) != 0U ? logical_high : logical_low;
        }

        LibXR::WriteOperation operation;
        return spi_.Write({frame_.data(), frame_.size()}, operation);
    }
}
