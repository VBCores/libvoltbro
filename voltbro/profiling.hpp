#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx.h"
#elif defined(STM32H7)
#include "stm32h7xx.h"
#endif

namespace profiling {

inline uint32_t cycles() {
#ifdef DWT
    return DWT->CYCCNT;
#else
    return 0;
#endif
}

inline void init_cycles() {
#ifdef DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

struct Sample {
    bool selected = false;
    uint32_t start = 0;
};

inline Sample sample_cycles(uint16_t& counter, uint16_t period) {
    if (++counter < period) return {};
    counter = 0;
    return {true, cycles()};
}

/** Rolling one-second frequencies, sampled every 100 one-millisecond ticks. */
template<size_t N>
struct RollingFrequency {
    std::array<uint32_t, N> rate{};
    std::array<uint32_t, N> minimum = [] {
        std::array<uint32_t, N> result{};
        result.fill(UINT32_MAX);
        return result;
    }();
    uint32_t windows = 0;
    std::array<std::array<uint32_t, N>, 10> snapshots{};
    uint8_t index = 0, count = 0, ticks = 0;

    bool record(const std::array<uint32_t, N>& counters) {
        if (counters[0] == 0 || ++ticks != 100) return false;
        ticks = 0;
        const bool ready = count == snapshots.size();
        if (ready) {
            for (size_t i = 0; i < N; ++i) {
                rate[i] = counters[i] - snapshots[index][i];
                minimum[i] = std::min(minimum[i], rate[i]);
            }
            ++windows;
        } else ++count;
        snapshots[index] = counters;
        index = (index + 1) % snapshots.size();
        return ready;
    }
};

/** Fill only unused stack memory below the caller's stack pointer. */
inline void mark_stack(volatile uint32_t* begin, volatile uint32_t* stack_pointer,
                       uint32_t canary = 0xDEADBEEF) {
    while (begin < stack_pointer) *begin++ = canary;
}

/** Return the observed stack high-water mark in 32-bit words. */
inline size_t stack_usage(volatile uint32_t* begin, volatile uint32_t* end,
                         uint32_t canary = 0xDEADBEEF) {
    while (begin < end && *begin == canary) ++begin;
    return static_cast<size_t>(end - begin);
}

struct IntervalStats {
    volatile uint32_t stream_count = 0;
    volatile uint32_t first_second = 0;
    volatile uint32_t second_second = 0;
    volatile uint32_t min_gap_cycles = UINT32_MAX;
    volatile uint32_t max_gap_cycles = 0;
    volatile uint32_t fast_gaps = 0;
    volatile uint32_t slow_gaps = 0;
    volatile uint32_t stream_starts = 0;
    volatile uint32_t first_counter_delta = 0;
    volatile uint32_t second_counter_delta = 0;
    uint32_t start_first_counter = 0;
    uint32_t start_second_counter = 0;
    uint32_t last_cycles = 0;
    uint32_t last_ms = 0;
    uint32_t start_cycles = 0;
    uint32_t elapsed_lo = 0, elapsed_hi = 0;
    volatile uint32_t gap_histogram[51] = {}; // 100 us bins; last bin >=5 ms.

    /** Record one subscription callback and its DWT interval within a continuous stream. */
    [[gnu::noinline, gnu::optimize("Os")]] void record(uint32_t now_ms, uint32_t now_cycles, uint32_t cycles_per_ms,
                uint32_t first_counter = 0, uint32_t second_counter = 0) {
        if (stream_count == 0 || now_ms - last_ms > 500) {
            stream_count = 0;
            first_second = 0;
            second_second = 0;
            min_gap_cycles = UINT32_MAX;
            max_gap_cycles = 0;
            fast_gaps = 0;
            slow_gaps = 0;
            stream_starts = stream_starts + 1;
            start_cycles = now_cycles;
            elapsed_lo = elapsed_hi = 0;
            for (auto& bin : gap_histogram) bin = 0;
            start_first_counter = first_counter;
            start_second_counter = second_counter;
        } else {
            const uint32_t gap_cycles = now_cycles - last_cycles;
            const uint32_t prior = elapsed_lo;
            elapsed_lo += gap_cycles;
            if (elapsed_lo < prior) ++elapsed_hi;
            const uint32_t bin = std::min<uint32_t>(50, gap_cycles / (cycles_per_ms / 10));
            gap_histogram[bin] = gap_histogram[bin] + 1;
            if (gap_cycles < min_gap_cycles) min_gap_cycles = gap_cycles;
            if (gap_cycles > max_gap_cycles) max_gap_cycles = gap_cycles;
            if (gap_cycles < cycles_per_ms / 2) fast_gaps = fast_gaps + 1;
            if (gap_cycles > cycles_per_ms * 3 / 2) slow_gaps = slow_gaps + 1;
        }
        last_ms = now_ms;
        last_cycles = now_cycles;
        first_counter_delta = first_counter - start_first_counter;
        second_counter_delta = second_counter - start_second_counter;
        stream_count = stream_count + 1;
        const uint64_t since_start = (uint64_t(elapsed_hi) << 32) | elapsed_lo;
        if (since_start < cycles_per_ms * 1000u) first_second = first_second + 1;
        else if (since_start < cycles_per_ms * 2000u) second_second = second_second + 1;
    }
};

} // namespace profiling

#ifdef SERVO_REFERENCE_TRACE
struct ServoReferenceSample {
    uint32_t tick;
    float reference_position, reference_velocity, measured_position, measured_velocity;
};
inline volatile ServoReferenceSample servo_reference_trace[64]{};
inline volatile uint32_t servo_reference_trace_count = 0;
inline uint32_t servo_reference_trace_tick = 0;
// Eight samples/s on the 40 kHz control clock; read the ring after stopping.
#define VB_PROFILE_REFERENCE_TRACE(state_expr, position_expr, velocity_expr, clock_tick) do { \
    if (servo_reference_trace_count == 0 || uint32_t((clock_tick) - servo_reference_trace_tick) >= 5000U) { \
        const auto reference_state = (state_expr); \
        auto& sample = servo_reference_trace[servo_reference_trace_count & 63U]; \
        sample.tick = (clock_tick); \
        sample.reference_position = reference_state.position; \
        sample.reference_velocity = reference_state.velocity; \
        sample.measured_position = (position_expr); \
        sample.measured_velocity = (velocity_expr); \
        servo_reference_trace_tick = (clock_tick); \
        servo_reference_trace_count = servo_reference_trace_count + 1; \
    } \
} while (false);
#else
#define VB_PROFILE_REFERENCE_TRACE(state, position, velocity, tick)
#endif

#ifdef FOC_PROFILE
struct ServoScheduleProfile {
    volatile uint32_t calls = 0, references = 0, integrals = 0;
    volatile uint32_t reference_max = 0, integral_max = 0;
    volatile uint32_t plans = 0, plan_max = 0;
};
inline ServoScheduleProfile servo_schedule_profile;
#endif

#ifdef FOC_PROFILE_DETAILED
struct FOCProfile {
    volatile uint32_t total = 0;
    volatile uint32_t sensors = 0;
    volatile uint32_t currents = 0;
    volatile uint32_t outer_loop = 0;
    volatile uint32_t pwm = 0;
};
inline volatile FOCProfile foc_profile;

struct SensorsProfile {
    volatile uint32_t inverter = 0;
    volatile uint32_t angle = 0;
    volatile uint32_t kalman = 0;
    volatile uint32_t total = 0;
};
inline volatile SensorsProfile sensors_profile;

struct KalmanProfile {
    volatile uint32_t start = 0;
    volatile uint32_t mid = 0;
    volatile uint32_t end = 0;
    volatile uint32_t total = 0;
};
inline volatile KalmanProfile kalman_profile;
#endif

#if defined(MONITOR)
inline volatile float I_D = 0;
inline volatile float I_Q = 0;
inline float V_d, V_q;
inline volatile float i_q_error, i_d_error;
inline float d_response, q_response, i_q_set;
inline volatile float control_error_glob = 0;
inline volatile float controller_response_glob = 0;
inline volatile float value_foc_p = 0;
inline volatile float value_foc_v = 0;
inline volatile float value_foc_p_kp = 0;
inline volatile float value_foc_v_kp = 0;
inline volatile float value_foc_t = 0;
inline volatile uint16_t raw_value = 0;
#endif

// Disabled hooks discard arguments, so neither diagnostic state nor work remains.
#ifdef FOC_PROFILE
#define VB_PROFILE_BEGIN(name) const uint32_t name = profiling::cycles();
#define VB_PROFILE_COUNT(counter) counter = counter + 1;
#define VB_PROFILE_END(maximum, start) do { \
    const uint32_t elapsed = profiling::cycles() - (start); \
    if (elapsed > (maximum)) (maximum) = elapsed; \
} while (false);
#define VB_PROFILE_RECORD_IF(condition, counter, maximum, start) do { \
    if (condition) { VB_PROFILE_COUNT(counter) VB_PROFILE_END(maximum, start) } \
} while (false);
#else
#define VB_PROFILE_BEGIN(name)
#define VB_PROFILE_COUNT(counter)
#define VB_PROFILE_END(maximum, start)
#define VB_PROFILE_RECORD_IF(condition, counter, maximum, start)
#endif

#ifdef FOC_PROFILE_DETAILED
#define VB_PROFILE_DETAIL_BEGIN() const uint32_t start_total = profiling::cycles(); uint32_t t_start = profiling::cycles();
#define VB_PROFILE_DETAIL_START() t_start = profiling::cycles();
#define VB_PROFILE_DETAIL_END(field, start) field = profiling::cycles() - (start);
#define VB_PROFILE_DETAIL_SPLIT(field) VB_PROFILE_DETAIL_END(field, t_start) VB_PROFILE_DETAIL_START()
#else
#define VB_PROFILE_DETAIL_BEGIN()
#define VB_PROFILE_DETAIL_START()
#define VB_PROFILE_DETAIL_END(field, start)
#define VB_PROFILE_DETAIL_SPLIT(field)
#endif
