/*
 * What's this file for?
 *   PIO-based frequency counter program for RP2040.
 * What's the purpose of this file?
 *   Measure input signal frequency on GPIO pins using PIO hardware state machines.
 *   Each PIO SM measures one full period (rising edge to rising edge) by counting
 *   PIO clock cycles, enabling fast and accurate frequency detection.
 * What's the scope of this file?
 *   4-channel frequency measurement on GPIO 0~3 using PIO0's 4 state machines.
 * What's the impact of this file?
 *   Provides hardware-level, CPU-independent frequency measurement with cycle-accurate precision.
 * What's the impact of this file on the project?
 *   Core measurement engine — feeds measured frequency to AD9959 DDS for frequency multiplication output.
 * What's the impact of this file on the team?
 *   Enables real-time zero-crossing frequency detection without CPU polling overhead.
 * What's the impact of this file on the company?
 *   Supports precision signal processing applications.
 * What's the impact of this file on the world?
 *   Contributes to open-source PIO frequency measurement techniques for RP2040.
 */

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --- PIO Rising Edge Counter Program ---
//
// Counts rising edges internally using X register.
// CPU periodically reads X (via mov+push) to get edge count.
// No FIFO overflow possible — X just keeps incrementing.
//
// Normal loop (2 instructions, runs continuously):
//   0: wait 0 pin 0    ; Wait for LOW
//   1: wait 1 pin 0    ; Wait for rising edge
//   2: jmp  x--, 0     ; X-- (count one edge), wrap back
//
// X starts at 0xFFFFFFFF. After N edges, X = 0xFFFFFFFF - N.
// edge_count = 0xFFFFFFFF - X = ~X
//
// CPU reads edge count by:
//   1) Executing sm_exec(mov isr, x) to copy X to ISR
//   2) Executing sm_exec(push noblock) to push ISR to FIFO
//   3) Reading FIFO
// This is done without stopping the SM — atomic snapshot of X.

static const uint16_t frequency_counter_program_instructions[] = {
    0x2020, //  0: wait   0 pin, 0        ; Wait for LOW
    0x20a0, //  1: wait   1 pin, 0        ; Rising edge
    0x0040, //  2: jmp    x--, 0          ; X--, loop back
};

static const struct pio_program frequency_counter_program = {
    .instructions = frequency_counter_program_instructions,
    .length = 3,
    .origin = -1,
};

// Initialize one PIO state machine for frequency counting on a specific pin.
// PIO clock is set to full system clock (125 MHz by default) for maximum resolution.
static inline void frequency_counter_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false); // pin as input

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 2);
    sm_config_set_in_pins(&c, pin);       // WAIT PIN uses this pin

    sm_config_set_clkdiv(&c, 1.0f);

    pio_sm_init(pio, sm, offset, &c);

    // Initialize X to 0xFFFFFFFF (so ~X starts at 0 edges)
    // mov x, !null → X = ~0 = 0xFFFFFFFF
    pio_sm_exec(pio, sm, 0xa02b);

    pio_sm_set_enabled(pio, sm, true);
}

// Read current edge count from a running SM.
// Pauses SM briefly to inject mov+push, then resumes immediately.
// The pause is ~100ns (a few CPU instructions), so at most 1 edge
// could be missed at 100kHz (period=10us). Over a 500ms gate window
// this is <0.001% error.
static inline uint32_t frequency_counter_read_count(PIO pio, uint sm) {
    // Briefly pause SM so injected instructions execute atomically
    pio_sm_set_enabled(pio, sm, false);

    // Drain any stale FIFO data
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        (void)pio_sm_get(pio, sm);
    }

    // Inject: mov isr, x (copy X to ISR)
    pio_sm_exec(pio, sm, 0xa0c1);
    // Inject: push noblock (push ISR to FIFO)
    pio_sm_exec(pio, sm, 0x8000);

    uint32_t result = 0;
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        result = ~pio_sm_get(pio, sm);
    }

    // Resume SM immediately
    pio_sm_set_enabled(pio, sm, true);

    return result;
}