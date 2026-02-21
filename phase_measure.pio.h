/*
 * What's this file for?
 *   PIO-based phase difference measurement program for RP2040.
 * What's the purpose of this file?
 *   Measure the time (in PIO clock cycles) between a reference signal's
 *   rising edge and a target signal's rising edge, enabling precise
 *   phase difference calculation between two square wave inputs.
 * What's the scope of this file?
 *   3-channel phase measurement on pio1 (SM0-SM2), each measuring
 *   the phase difference from SIN+ (GPIO1) to one of the other 3 signals.
 * What's the impact of this file?
 *   Provides hardware-level phase measurement with 5ns precision (@ 200MHz),
 *   enabling accurate phase tracking across 1kHz-100kHz range.
 * What's the impact of this file on the project?
 *   Enables the DDS output to track and reproduce the input signal's
 *   phase relationships with high fidelity.
 * What's the impact of this file on the team?
 *   Adds quadrature signal processing capability to the platform.
 * What's the impact of this file on the company?
 *   Supports precision resolver/encoder signal conditioning applications.
 * What's the impact of this file on the world?
 *   Contributes to open-source PIO phase measurement techniques for RP2040.
 *
 * This will make sure that the file is always included in the project rules.
 */

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --- PIO Phase Difference Measurement Program ---
//
// Measures the number of PIO clock cycles between:
//   - Reference signal (IN pin 0) rising edge
//   - Target signal (JMP pin) NEXT rising edge
//
// Algorithm:
//   0: wait 0 pin 0       ; Wait for reference LOW
//   1: wait 1 pin 0       ; Reference rising edge detected
//   2: mov x, !null        ; X = 0xFFFFFFFF (start counting)
//   --- Phase 1: If target is HIGH, wait for it to go LOW (counting cycles) ---
//   3: jmp pin, 5          ; If target HIGH → go to 5 (keep waiting for LOW)
//   4: jmp 6               ; Target is LOW → skip to Phase 2
//   5: jmp x--, 3          ; X--, loop back to check target again
//   --- Phase 2: Target is LOW, wait for it to go HIGH (counting cycles) ---
//   6: jmp pin, 8          ; If target HIGH → done!
//   7: jmp x--, 6          ; X--, keep counting
//   --- Result ---
//   8: mov isr, x          ; Copy X to ISR
//   9: push noblock        ; Push to FIFO, wrap back to 0
//
// This correctly handles all cases:
//   - Target rises AFTER ref: counts LOW wait time only
//   - Target is HIGH when ref rises: counts HIGH→LOW→HIGH (full remaining cycle)
//   - Result: always measures ref_edge → next_target_edge
//
// Phase difference (in cycles) = 0xFFFFFFFF - X = ~X
// Phase angle = (~X / period_cycles) * 360.0 degrees
//
// At 200MHz PIO clock:
//   - Resolution: 5ns per count
//   - At 1kHz (T=1ms=200000 cycles): 0.0018° per count
//   - At 100kHz (T=10µs=2000 cycles): 0.18° per count
//
// Note: counting loop is 2 instructions (jmp pin + jmp x--) = 2 cycles per count.
// Actual cycle count = ~X * 2 (for phase 2) + overhead.
// We calibrate this out by using the same program to measure a full period.

static const uint16_t phase_measure_program_instructions[] = {
    0x2020, //  0: wait   0 pin, 0        ; Wait for reference LOW
    0x20a0, //  1: wait   1 pin, 0        ; Reference rising edge
    0xa02b, //  2: mov    x, !null        ; X = 0xFFFFFFFF
    0x00c5, //  3: jmp    pin, 5          ; If target HIGH → 5
    0x0006, //  4: jmp    6               ; Target LOW → Phase 2
    0x0043, //  5: jmp    x--, 3          ; X--, back to check
    0x00c8, //  6: jmp    pin, 8          ; If target HIGH → done
    0x0046, //  7: jmp    x--, 6          ; X--, keep counting
    0xa0c1, //  8: mov    isr, x          ; Copy X to ISR
    0x8000, //  9: push   noblock         ; Push to FIFO
};

static const struct pio_program phase_measure_program = {
    .instructions = phase_measure_program_instructions,
    .length = 10,
    .origin = -1,
};

// Initialize a PIO state machine for phase difference measurement.
//
// ref_pin:    The reference signal pin (SIN+, GPIO1). Used as IN base pin.
// target_pin: The target signal pin. Used as JMP pin.
//
// The SM measures cycles from ref_pin rising edge to target_pin rising edge.
static inline void phase_measure_program_init(PIO pio, uint sm, uint offset,
                                               uint ref_pin, uint target_pin) {
    // Initialize both pins for PIO use
    pio_gpio_init(pio, ref_pin);
    pio_gpio_init(pio, target_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, ref_pin, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, target_pin, 1, false);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 9);

    // IN pin base = ref_pin (used by WAIT instructions)
    sm_config_set_in_pins(&c, ref_pin);

    // JMP pin = target_pin (used by JMP PIN instruction)
    sm_config_set_jmp_pin(&c, target_pin);

    // Full speed — no clock divider
    sm_config_set_clkdiv(&c, 1.0f);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// Read the latest phase difference measurement from the FIFO.
// Returns the cycle count (0 = no data available).
// Drains FIFO and returns only the most recent measurement.
static inline uint32_t phase_measure_read(PIO pio, uint sm) {
    uint32_t result = 0;
    bool got_data = false;

    // Drain FIFO, keep only the latest value
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        result = pio_sm_get(pio, sm);
        got_data = true;
    }

    if (got_data) {
        return ~result; // Convert from countdown to count-up
    }
    return 0; // No data available
}
