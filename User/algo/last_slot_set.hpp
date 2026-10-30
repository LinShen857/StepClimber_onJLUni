#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace JLUni::Algo
{
    /**
     * @brief Persistent latest snapshots in Count + 1 slots. SPSC, producer prior to consumer.
     * @note Single-core only: one non-reentrant producer ISR may interrupt the
     *       consumer, never the reverse. Not a general threaded SPSC container.
     *       Only one borrow may be held across the entire set. Input/output
     *       objects must not alias slots or be concurrently modified.
     */
    template <typename Value, std::size_t Count>
    class LastSlotSet
    {
        static_assert(Count > 0U && Count <= 255U);
        static_assert(std::is_default_constructible_v<Value>);
        static_assert(std::is_nothrow_copy_assignable_v<Value>);
        static_assert(std::atomic<std::uint16_t>::is_always_lock_free);

        static constexpr std::uint16_t mask_index = 0xFFU;
        static constexpr std::uint16_t mask_valid = 0x100U;
        static constexpr std::uint16_t mask_reading = 0x200U;
        static constexpr std::uint8_t no_reading_index = 255U;

    public:
        LastSlotSet() { Discard(); }
        LastSlotSet(const LastSlotSet&) = delete;
        LastSlotSet& operator=(const LastSlotSet&) = delete;

        [[nodiscard]] bool Publish(const std::size_t index, const Value& value) noexcept
        {
            if (index >= Count) { return false; }
            auto& descriptor = descriptors_[index];
            const auto before = descriptor.load(std::memory_order_acquire);
            auto target = static_cast<std::uint8_t>(before & mask_index);
            if ((before & mask_reading) != 0U)  // redirect to the spare slot if the consumer is reading
            {
                const auto previous = target;
                target = spare_;
                spare_ = previous;
            }
            slots_[target] = value;
            descriptor.store(static_cast<std::uint16_t>(target | mask_valid), std::memory_order_release);
            return true;
        }

        // Borrow a stable snapshot without copying. The pointer is valid only
        // until ReturnLatest(); reacquire afterwards to observe newer feedback.
        [[nodiscard]] const Value* AcquireLatest(const std::size_t index) noexcept
        {
            if (index >= Count || reading_index_ != no_reading_index) { return nullptr; }
            auto& descriptor = descriptors_[index];
            const auto before = descriptor.fetch_or(mask_reading, std::memory_order_acq_rel);
            reading_index_ = static_cast<std::uint8_t>(index);
            reading_descriptor_ = before;
            if ((before & mask_valid) == 0U)
            {
                static_cast<void>(ReturnLatest());
                return nullptr;
            }
            return &slots_[before & mask_index];
        }

        // End the borrow without consuming the cached value. True means a
        // borrow ended, including when the producer has already redirected it.
        [[nodiscard]] bool ReturnLatest() noexcept
        {
            if (reading_index_ == no_reading_index) { return false; }
            auto& descriptor = descriptors_[reading_index_];
            // A redirected publication must not be cleared by the old reader.
            auto expected = static_cast<std::uint16_t>(reading_descriptor_ | mask_reading);
            static_cast<void>(descriptor.compare_exchange_strong(expected, reading_descriptor_,
                std::memory_order_release, std::memory_order_relaxed));
            reading_index_ = no_reading_index;
            reading_descriptor_ = 0U;
            // The retired slot becomes reusable only when consumer begins next acquisition (acq_rel).
            return true;
        }

        // Copy without consuming the cache. Failure leaves the output unchanged,
        // including when another value is already borrowed from this set.
        [[nodiscard]] bool ReadLatest(const std::size_t index, Value& value) noexcept
        {
            const auto* latest = AcquireLatest(index);
            if (latest == nullptr) { return false; }
            value = *latest;
            static_cast<void>(ReturnLatest());
            return true;
        }

        // Only call while both producer and consumer are stopped, with no borrow.
        void Discard() noexcept
        {
            for (std::size_t i = 0; i < Count; ++i)
            {
                descriptors_[i].store(static_cast<std::uint16_t>(i), std::memory_order_relaxed);
            }
            spare_ = static_cast<std::uint8_t>(Count);
            reading_index_ = no_reading_index;
            reading_descriptor_ = 0U;
        }

    private:
        std::array<Value, Count + 1> slots_{};
        std::array<std::atomic<std::uint16_t>, Count> descriptors_{};
        std::uint8_t spare_ = static_cast<std::uint8_t>(Count);
        // Consumer-owned metadata; never accessed by the producer.
        std::uint8_t reading_index_ = no_reading_index;
        std::uint16_t reading_descriptor_ = 0U;
    };
}
