#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace JLUni::Algo
{
    /**
     * @brief spsc buffer when producer prior to consumer
     * @note Producer and consumer must not reenter their respective operations.
     *       The consumer may hold at most one acquired value at a time.
     */
    template <typename Value>
    class LastBuffer
    {
        enum class SlotState : std::uint8_t
        {
            empty,
            writing,
            ready,
            reading,
        };

        static constexpr std::uint8_t no_reading_index = 2U;

    public:
        static_assert(std::is_default_constructible_v<Value>);
        static_assert(std::is_copy_assignable_v<Value>);
        static_assert(std::atomic<SlotState>::is_always_lock_free);

        LastBuffer() = default;
        LastBuffer(const LastBuffer&) = delete;
        LastBuffer& operator=(const LastBuffer&) = delete;

        /**
         * @brief Publish a new value to the buffer, overwriting the previous value if necessary
         */
        [[nodiscard]] bool Publish(const Value& value)
        {
            // get the index of the slot that is currently published, and try to write to the other slot
            const auto published = published_index_.load(std::memory_order_relaxed);
            auto write_index = static_cast<std::uint8_t>(published ^ 1U);
            // if the other slot is not available, try to write to the currently published slot
            if (!TryStartWriting(write_index))
            {
                write_index = published;
                if (!TryStartWriting(write_index)) { return false; }
            }
            data_[write_index] = value;
            state_[write_index].store(SlotState::ready, std::memory_order_release);
            published_index_.store(write_index, std::memory_order_release);
            return true;
        }

        /**
         * @brief Acquire the latest published value, or fail if a value is already held
         * @note The pointer may only be used until ReleaseLatest() or ReturnLatest().
         *       A producer interrupt during acquisition may leave this call reading an older value.
         */
        [[nodiscard]] const Value* AcquireLatest()
        {
            if (reading_index_ != no_reading_index) { return nullptr; }
            const auto read_index = published_index_.load(std::memory_order_acquire);
            auto expected = SlotState::ready;
            if (!state_[read_index].compare_exchange_strong(expected, SlotState::reading,
                std::memory_order_acq_rel, std::memory_order_relaxed))
            {
                return nullptr;
            }
            reading_index_ = read_index;
            return &data_[read_index];
        }

        /**
         * @brief Release and consume the acquired value, ending pointer access
         */
        [[nodiscard]] bool ReleaseLatest()
        {
            return FinishReading(SlotState::empty);
        }

        /**
         * @brief Return the acquired value without consuming it, ending pointer access
         */
        [[nodiscard]] bool ReturnLatest()
        {
            return FinishReading(SlotState::ready);
        }

        /**
         * @brief Read and consume the latest published value once
         * @note Without a new publication, both read APIs fail after consumption.
         */
        [[nodiscard]] bool ConsumeLatest(Value& value)
        {
            const auto* const latest = AcquireLatest();
            if (latest == nullptr) { return false; }
            value = *latest;
            static_cast<void>(ReleaseLatest());
            return true;
        }

        /**
         * @brief Copy the latest published value without consuming it
         * @note Repeated peeks succeed until the value is consumed or discarded.
         */
        [[nodiscard]] bool PeekLatest(Value& value)
        {
            const auto* const latest = AcquireLatest();
            if (latest == nullptr) { return false; }
            value = *latest;
            static_cast<void>(ReturnLatest());
            return true;
        }

        /**
         * @brief Discard all published values, marking them as empty
         * @note Requires no publication or acquired value in progress.
         */
        void Discard()
        {
            for (auto& slot : state_)
            {
                auto expected = SlotState::ready;
                static_cast<void>(slot.compare_exchange_strong(expected, SlotState::empty,
                    std::memory_order_acq_rel, std::memory_order_relaxed));
            }
        }

    private:
        [[nodiscard]] bool FinishReading(const SlotState state)
        {
            if (reading_index_ == no_reading_index) { return false; }
            state_[reading_index_].store(state, std::memory_order_release);
            reading_index_ = no_reading_index;
            return true;
        }

        /**
         * @brief Attempt to start writing to a slot
         */
        [[nodiscard]] bool TryStartWriting(const std::uint8_t index)
        {
            auto expected = state_[index].load(std::memory_order_acquire);
            while (expected == SlotState::empty || expected == SlotState::ready)
            {
                if (state_[index].compare_exchange_weak(expected, SlotState::writing,
                    std::memory_order_acq_rel, std::memory_order_relaxed))
                {
                    return true;
                }
            }
            return false;
        }

        std::array<Value, 2U> data_{};
        std::array<std::atomic<SlotState>, 2U> state_{{SlotState::empty, SlotState::empty}};
        std::atomic<std::uint8_t> published_index_{0U};
        std::uint8_t reading_index_{no_reading_index};  // Consumer-owned; never accessed by the producer.
    };
}
