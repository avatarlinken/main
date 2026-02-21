#include <Arduino.h>
#include <SPI.h>
#include "hardware/pio.h"
#include "measure.pio.h"
#include "phase_measure.pio.h"

#define ENABLE_SERIAL_DDS_DEBUG 0
#define DEFAULT_MULTIPLIER_N 20

// --- 管脚定义 ---
#define DDS_RESET_PIN     17
#define DDS_SCLK_PIN      18
#define DDS_MOSI_PIN      19
#define DDS_CS_PIN        20
#define DDS_UPDATE_PIN    21

// --- 频率测量引脚 (GPIO 0~3, 过零检测输入) ---
#define FREQ_INPUT_PIN_0  0
#define FREQ_INPUT_PIN_1  1
#define FREQ_INPUT_PIN_2  2
#define FREQ_INPUT_PIN_3  3
#define NUM_CHANNELS      4

// --- PIO0: Frequency measurement (4 SMs, one per channel) ---
static PIO   pio_freq = pio0;
static uint  pio_offset;
static const uint freq_pins[NUM_CHANNELS] = {
    FREQ_INPUT_PIN_0, FREQ_INPUT_PIN_1, FREQ_INPUT_PIN_2, FREQ_INPUT_PIN_3
};

// --- PIO1: Phase difference measurement (3 SMs) ---
// SM0: SIN+(GPIO1) → COS-(GPIO0), SM1: SIN+(GPIO1) → COS+(GPIO2), SM2: SIN+(GPIO1) → SIN-(GPIO3)
static PIO   pio_phase = pio1;
static uint  pio_phase_offset;
#define PHASE_REF_PIN     FREQ_INPUT_PIN_1  // SIN+ is the reference
#define NUM_PHASE_CH      3                 // 3 target channels (excluding SIN+ itself)
// Target pins for phase measurement (GPIO0, GPIO2, GPIO3)
static const uint phase_target_pins[NUM_PHASE_CH] = {
    FREQ_INPUT_PIN_0, FREQ_INPUT_PIN_2, FREQ_INPUT_PIN_3
};
// Which GPIO channel index each phase SM corresponds to
static const uint8_t phase_target_ch[NUM_PHASE_CH] = {0, 2, 3};

// --- GPIO → AD9959 DDS channel mapping ---
// GPIO 0 (COS-) → IOUT3 (DDS CH3)
// GPIO 1 (SIN+) → IOUT0 (DDS CH0)
// GPIO 2 (COS+) → IOUT1 (DDS CH1)
// GPIO 3 (SIN-) → IOUT2 (DDS CH2)
static const uint8_t gpio_to_dds[NUM_CHANNELS] = {3, 0, 1, 2};

// --- DDS channel (IOUTx) → GPIO mapping (for ordered printing) ---
// OUT0(IOUT0)=SIN+(GPIO1), OUT1(IOUT1)=COS+(GPIO2), OUT2(IOUT2)=SIN-(GPIO3), OUT3(IOUT3)=COS-(GPIO0)
static const uint8_t dds_to_gpio[NUM_CHANNELS] = {1, 2, 3, 0};

// --- Ideal phase offsets (degrees) for quadrature signals ---
// GPIO order: 0=COS-(270°), 1=SIN+(0° ref), 2=COS+(90°), 3=SIN-(180°)
static const double ideal_phase_deg[NUM_CHANNELS] = {270.0, 0.0, 90.0, 180.0};

// --- Measured phase differences (degrees, relative to SIN+) ---
static double measured_phase_deg[NUM_CHANNELS] = {270.0, 0.0, 90.0, 180.0};

// --- 倍频系数 (来自串口, 范围 1~20) ---
static uint16_t multiplier_n = DEFAULT_MULTIPLIER_N;
static bool multiplier_changed = false;

#if ENABLE_SERIAL_DDS_DEBUG
static bool debug_dds_direct_freq_en = false;
static uint32_t debug_dds_direct_freq_hz = 0;
static bool debug_dds_direct_changed = false;
#endif

// --- 测量结果 (整数 Hz) ---
static uint32_t measured_freq[NUM_CHANNELS] = {0};
static uint32_t last_printed_freq[NUM_CHANNELS] = {0};

// --- EMA (Exponential Moving Average) filter for frequency smoothing ---
// Alpha = 0.2: smooth but responsive. Lower = smoother, higher = faster response.
static const double FREQ_EMA_ALPHA = 0.05;  // Increased from 0.05 for faster response
static const double FREQ_EMA_RESET_THRESHOLD = 0.01;  // Reset threshold: 1% (was 5%)
static double freq_ema[NUM_CHANNELS] = {0};
static bool   freq_ema_valid[NUM_CHANNELS] = {false};

// --- Gate-based frequency measurement ---
// Periodically sample PIO edge count + micros() timestamp.
// freq = delta_edges * 1000000 / delta_us
// No FIFO, no overflow, no PIO counting overhead.
static uint32_t prev_edge_count[NUM_CHANNELS] = {0};
static uint32_t prev_sample_us[NUM_CHANNELS] = {0};
static bool     prev_sample_valid[NUM_CHANNELS] = {false};
static uint32_t pending_freq[NUM_CHANNELS] = {0}; // candidate before confirmation
static bool     signal_present[NUM_CHANNELS] = {false}; // track if signal is active
static bool     prev_signal_present[NUM_CHANNELS] = {false}; // for detecting signal loss

// --- 串口输入缓冲 ---
static String serial_buf = "";

// --- 寄存器地址 ---
#define CSR_REG    0x00 
#define FR1_REG    0x01 
#define CFTW0_REG  0x04 
#define ACR_REG    0x06 
#define CPOW0_REG  0x05
#define CW1_REG    0x0A 

// --- 全局变量 ---
uint32_t current_sys_clk = 25000000; 

// --- 智能调节参数 (基于实测数据) ---
// 用户填入：两个校准频率点上，ACR=1023(满量程) 时放大器输出的实测 VPP (mV)
// 电路：反相放大器，增益 = Rf / Ri
const double AMP_RF = 2400.0;           // 反馈电阻 (ohm)
const double AMP_RI = 1000.0;           // 输入电阻 (ohm)
const double AMP_GAIN = AMP_RF / AMP_RI; // 放大器增益 = 2.0

const double TARGET_VPP_MV = 1000.0;    // 目标输出 VPP (mV), 放大器之后

// 校准点1: 低频 (1kHz), ACR=1023 时放大器输出的实测 VPP
const double CAL_FREQ_LOW = 1000.0;           // 低频校准频率 (Hz)
const double CAL_VPP_LOW_MV = 1216.0;          // 1kHz, ACR=1023 时实测 VPP (mV)

// 校准点2: 高频 (2MHz), ACR=1023 时放大器输出的实测 VPP
const double CAL_FREQ_HIGH = 2000000.0;       // 高频校准频率 (Hz)
const double CAL_VPP_HIGH_MV = 1149.0;        // 2MHz, ACR=1023 时实测 VPP (mV)

// 自动计算: 要达到目标 VPP，每个校准点需要的 ACR 值
// ACR = 1023 * (目标VPP / 实测VPP), 钳位到 [0, 1023]
#define CLAMP_ACR(x) ((x) > 1023.0 ? 1023.0 : (x))
const double CAL_ACR_LOW  = CLAMP_ACR(1023.0 * TARGET_VPP_MV / CAL_VPP_LOW_MV);
const double CAL_ACR_HIGH = CLAMP_ACR(1023.0 * TARGET_VPP_MV / CAL_VPP_HIGH_MV);

void ioUpdate() {
  digitalWrite(DDS_UPDATE_PIN, HIGH);
  delayMicroseconds(2); 
  digitalWrite(DDS_UPDATE_PIN, LOW);
  delayMicroseconds(2);
}

void writeRegisterMulti(uint8_t regAddr, uint8_t* data, uint8_t len) {
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DDS_CS_PIN, LOW);
  SPI.transfer(regAddr);
  for (uint8_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(DDS_CS_PIN, HIGH);
  SPI.endTransaction();
}

// --- 设置系统时钟 PLL ---
void setPLL(uint8_t multiplier) {
  if (multiplier < 4) multiplier = 4;
  if (multiplier > 20) multiplier = 20;

  current_sys_clk = 25000000 * multiplier;
  
  // 超过 250MHz 开启 High Range
  uint8_t vco_gain = (current_sys_clk >= 250000000) ? 0x80 : 0x00;
  uint8_t fr1_byte0 = vco_gain | ((multiplier & 0x1F) << 2) | 0x03;
  uint8_t fr1_config[] = {fr1_byte0, 0x00, 0x00};

  writeRegisterMulti(FR1_REG, fr1_config, 3);
  ioUpdate();
  delay(100); 
}

// --- 核心算法：计算补偿后的 ACR 值 ---
uint16_t calculateSmartACR(double freq) {
  // 1. 如果频率低于低频校准点，保持最低 ACR (防止过压)
  if (freq <= CAL_FREQ_LOW) {
    return (uint16_t)CAL_ACR_LOW;
  }
  
  // 2. 如果频率高于高频校准点，保持最大 ACR (硬件极限)
  if (freq >= CAL_FREQ_HIGH) {
    return (uint16_t)CAL_ACR_HIGH;
  }

  // 3. 中间频率：线性插值 (Linear Interpolation)
  // 公式：y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
  double slope = (CAL_ACR_HIGH - CAL_ACR_LOW) / (CAL_FREQ_HIGH - CAL_FREQ_LOW);
  double acr_val = CAL_ACR_LOW + (freq - CAL_FREQ_LOW) * slope;

  return (uint16_t)acr_val;
}

// --- Set frequency, amplitude, and phase for the currently selected channel ---
// phase_deg: phase offset in degrees (0~360)
// If defer_update is true, IO_UPDATE is NOT sent (caller must trigger it).
void setSmartFrequencyAndPhase(double freq_hz, double phase_deg, bool defer_update = false) {
  if (current_sys_clk == 0) return;

  // [Step 1] Smart ACR amplitude
  uint16_t acr_val = calculateSmartACR(freq_hz);
  uint8_t acr_data[3];
  acr_data[0] = 0x00;
  acr_data[1] = 0x10 | ((acr_val >> 8) & 0x03); 
  acr_data[2] = (uint8_t)(acr_val & 0xFF);
  writeRegisterMulti(ACR_REG, acr_data, 3);

  // [Step 2] Phase offset CPOW0 (14-bit, 0~16383 maps to 0~360°)
  // Normalize phase to 0~360
  while (phase_deg < 0.0) phase_deg += 360.0;
  while (phase_deg >= 360.0) phase_deg -= 360.0;
  uint16_t cpow = (uint16_t)(phase_deg * 16384.0 / 360.0) & 0x3FFF;
  uint8_t cpow_data[] = {
    (uint8_t)((cpow >> 8) & 0x3F),
    (uint8_t)(cpow & 0xFF)
  };
  writeRegisterMulti(CPOW0_REG, cpow_data, 2);

  // [Step 3] Frequency tuning word FTW
  uint32_t ftw = (uint32_t)((freq_hz * 4294967296.0) / current_sys_clk);
  uint8_t ftw_data[] = {
    (uint8_t)((ftw >> 24) & 0xFF),
    (uint8_t)((ftw >> 16) & 0xFF),
    (uint8_t)((ftw >> 8) & 0xFF),
    (uint8_t)(ftw & 0xFF)
  };
  writeRegisterMulti(CFTW0_REG, ftw_data, 4);
  writeRegisterMulti(CW1_REG, ftw_data, 4);
  
  // [Step 4] Update
  if (!defer_update) {
    ioUpdate();
  }
}

// --- Convenience wrapper: set frequency only (phase = 0) ---
void setSmartFrequency(double freq_hz) {
  setSmartFrequencyAndPhase(freq_hz, 0.0, false);
}

// --- Sample edge count and compute frequency (gate method) ---
// Called periodically (every UPDATE_INTERVAL_MS).
// Reads PIO edge count + micros(), computes freq from delta.
// Uses two-sample confirmation to filter out transition glitches.
void sampleFrequency(uint8_t ch) {
    // Read count and timestamp as close together as possible
    uint32_t now_count = frequency_counter_read_count(pio_freq, ch);
    uint32_t now_us = micros();

    if (prev_sample_valid[ch]) {
        uint32_t delta_edges = now_count - prev_edge_count[ch]; // handles 32-bit wrap
        uint32_t delta_us = now_us - prev_sample_us[ch];

        if (delta_us > 0 && delta_edges > 0) {
            // freq = delta_edges * 1000000 / delta_us (with rounding)
            uint32_t freq = (uint32_t)(
                ((uint64_t)delta_edges * 1000000ULL + delta_us / 2) / delta_us
            );

            // Two-sample confirmation: only accept if within 2% of pending
            // This filters out transition values when frequency changes
            if (pending_freq[ch] == 0 || 
                (freq > pending_freq[ch] * 98 / 100 && freq < pending_freq[ch] * 102 / 100)) {
                // EMA smoothing filter with adaptive reset threshold
                double threshold_ratio = 1.0 + FREQ_EMA_RESET_THRESHOLD;
                if (!freq_ema_valid[ch] || 
                    freq > freq_ema[ch] * threshold_ratio || freq < freq_ema[ch] / threshold_ratio) {
                    // Significant change (>threshold): reset EMA instantly for fast response
                    freq_ema[ch] = (double)freq;
                    freq_ema_valid[ch] = true;
                } else {
                    // Small change: apply EMA smoothing for noise reduction
                    freq_ema[ch] = FREQ_EMA_ALPHA * (double)freq + (1.0 - FREQ_EMA_ALPHA) * freq_ema[ch];
                }
                uint32_t new_freq = (uint32_t)(freq_ema[ch] + 0.5);
                // 稳定死区: 变化在±2Hz以内则不更新，避免显示跳动
                if (measured_freq[ch] == 0) {
                    measured_freq[ch] = new_freq;
                } else {
                    int32_t delta = (int32_t)new_freq - (int32_t)measured_freq[ch];
                    if (delta < -2 || delta > 2) {
                        measured_freq[ch] = new_freq;
                    }
                }
            }
            pending_freq[ch] = freq;
        } else {
            measured_freq[ch] = 0;
            pending_freq[ch] = 0;
        }
    }

    prev_edge_count[ch] = now_count;
    prev_sample_us[ch] = now_us;
    prev_sample_valid[ch] = true;
}

// --- Sample phase differences from pio1 ---
// Reads phase measurement FIFOs and converts to degrees.
// Uses measured_freq[1] (SIN+ frequency) to determine period.
//
// The PIO counting loop is 2 instructions per iteration, so:
//   actual_clock_cycles ≈ ~X * 2
//   period_in_loop_counts = F_CPU / ref_freq / 2
//   phase = delta_counts / period_counts * 360°
void samplePhase() {
    uint32_t ref_freq = measured_freq[1]; // SIN+ frequency
    if (ref_freq < 1) return; // No reference signal

    // Period in PIO loop iterations (2 clocks per iteration)
    double period_counts = (double)F_CPU / (double)ref_freq / 2.0;

    for (uint8_t i = 0; i < NUM_PHASE_CH; i++) {
        uint32_t delta_counts = phase_measure_read(pio_phase, i);
        if (delta_counts > 0 && period_counts > 0) {
            double phase = (double)delta_counts / period_counts * 360.0;
            // PIO measures REF_edge → TARGET_next_edge (delay), convert to actual phase
            phase = 360.0 - phase;
            // Normalize to 0~360
            while (phase >= 360.0) phase -= 360.0;
            while (phase < 0.0) phase += 360.0;
            // Store measured phase for this GPIO channel
            uint8_t ch = phase_target_ch[i];
            measured_phase_deg[ch] = phase;
        }
    }
    // SIN+ (GPIO1) is always 0° by definition
    measured_phase_deg[1] = 0.0;
}

// --- Parse serial input for multiplier n ---
// Accepts format: "n=5\n" or just "5\n"
void parseSerialInput() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            serial_buf.trim();
            if (serial_buf.length() > 0) {
                // Support "n=5" or "N=5" or just "5"
                String cmd = serial_buf;
                cmd.trim();

#if ENABLE_SERIAL_DDS_DEBUG
                if (cmd.length() >= 2 && (cmd[0] == 'D' || cmd[0] == 'd')) {
                    String hz_str = cmd.substring(1);
                    hz_str.trim();
                    uint32_t hz = (uint32_t)hz_str.toInt();
                    if (hz > 0) {
                        debug_dds_direct_freq_en = true;
                        debug_dds_direct_freq_hz = hz;
                        debug_dds_direct_changed = true;
                        Serial.print("DDS direct freq set to: ");
                        Serial.print(debug_dds_direct_freq_hz);
                        Serial.println(" Hz");
                    } else {
                        debug_dds_direct_freq_en = false;
                        debug_dds_direct_changed = true;
                        Serial.println("DDS direct freq disabled");
                    }
                } else
#endif
                {
                    String val_str = cmd;
                    int eq_pos = cmd.indexOf('=');
                    if (eq_pos >= 0) {
                        val_str = cmd.substring(eq_pos + 1);
                    }
                    val_str.trim();
                    int val = val_str.toInt();
                    if (val >= 1 && val <= 20) {
                        multiplier_n = (uint16_t)val;
                        multiplier_changed = true;
                        Serial.print("Multiplier set to: ");
                        Serial.println(multiplier_n);
                    } else {
                        Serial.println("Invalid multiplier! Range: 1~20");
                    }
                }
            }
            serial_buf = "";
        } else {
            serial_buf += c;
        }
    }
}

// --- Select AD9959 channel (0~3) ---
// NOTE: CSR is a "direct" register — it takes effect immediately after SPI write,
// WITHOUT needing IO_UPDATE. The AD9959 datasheet states CSR is not double-buffered.
// So we do NOT call ioUpdate() here to avoid prematurely latching pending FTW/CPOW data.
void selectChannel(uint8_t ch) {
    // CSR bit mapping: Ch0=bit4, Ch1=bit5, Ch2=bit6, Ch3=bit7
    uint8_t csr_data[] = { (uint8_t)(0x10 << ch) };
    writeRegisterMulti(CSR_REG, csr_data, 1);
}

// --- Select all AD9959 channels ---
void selectAllChannels() {
    uint8_t csr_data[] = {0xF0};
    writeRegisterMulti(CSR_REG, csr_data, 1);
    ioUpdate();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("AD9959 Frequency Multiplier System Starting...");

    Serial.print("System clock (F_CPU): ");
    Serial.print((uint32_t)F_CPU);
    Serial.println(" Hz");

    // --- DDS GPIO Init ---
    pinMode(DDS_RESET_PIN, OUTPUT);
    pinMode(DDS_CS_PIN, OUTPUT);
    pinMode(DDS_UPDATE_PIN, OUTPUT);
    digitalWrite(DDS_RESET_PIN, LOW);
    digitalWrite(DDS_CS_PIN, HIGH);
    digitalWrite(DDS_UPDATE_PIN, LOW);

    // --- SPI Init ---
    SPI.setRX(16);
    SPI.setSCK(DDS_SCLK_PIN);
    SPI.setTX(DDS_MOSI_PIN);
    SPI.begin();

    // --- DDS Reset ---
    digitalWrite(DDS_RESET_PIN, HIGH); delay(10);
    digitalWrite(DDS_RESET_PIN, LOW);  delay(10);

    // --- Set PLL: 4x = 100MHz (max, supports up to ~40MHz output) ---
    setPLL(4);

    // --- Select all channels, set initial frequency to 0 ---
    selectAllChannels();
    setSmartFrequency(0);

    // --- PIO0: Frequency Counter Init ---
    // Load program once, share across all 4 SMs
    pio_offset = pio_add_program(pio_freq, &frequency_counter_program);
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        frequency_counter_program_init(pio_freq, i, pio_offset, freq_pins[i]);
    }

    // --- PIO1: Phase Difference Measurement Init ---
    // 3 SMs measure phase from SIN+(GPIO1) to each of the other 3 signals
    pio_phase_offset = pio_add_program(pio_phase, &phase_measure_program);
    for (uint8_t i = 0; i < NUM_PHASE_CH; i++) {
        phase_measure_program_init(pio_phase, i, pio_phase_offset,
                                   PHASE_REF_PIN, phase_target_pins[i]);
    }

    Serial.println("System Ready.");
    Serial.println("  GPIO 0 = COS-, GPIO 1 = SIN+ (ref), GPIO 2 = COS+, GPIO 3 = SIN-");
    Serial.println("  DDS OUT order: OUT0=SIN+(GPIO1), OUT1=COS+(GPIO2), OUT2=SIN-(GPIO3), OUT3=COS-(GPIO0)");
    Serial.println("  Serial: Send 'n=<1~40>' to set multiplier");
    Serial.print("  Current multiplier: ");
    Serial.println(multiplier_n);
    Serial.println("  Phase measurement: PIO1 (hardware, 5ns resolution)");
    Serial.println("Waiting for input signals...");
}

// --- Throttle interval for DDS update and serial print (ms) ---
// Reduced from 500ms to 100ms for faster frequency tracking response
static const unsigned long UPDATE_INTERVAL_MS = 100;
static unsigned long last_update_ms = 0;

// Track last output phase to detect changes
static double last_output_phase[NUM_CHANNELS] = {-1, -1, -1, -1};

void loop() {
    // --- 1. Parse serial for multiplier changes ---
    parseSerialInput();

    // --- 2. Throttled: sample frequency + phase, update DDS, print ---
    unsigned long now = millis();
    if (now - last_update_ms >= UPDATE_INTERVAL_MS) {
        last_update_ms = now;

        // Sample edge counts from all channels (frequency)
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            sampleFrequency(ch);
        }

        // Sample phase differences (pio1)
        samplePhase();

        // Use SIN+ (GPIO1) frequency as the common reference frequency
        uint32_t ref_freq = measured_freq[1];
        bool ref_present = (ref_freq >= 1);

        // Detect signal state changes
        bool state_changed = false;
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            prev_signal_present[ch] = signal_present[ch];
            signal_present[ch] = (measured_freq[ch] >= 1);
            if (signal_present[ch] != prev_signal_present[ch]) {
                state_changed = true;
            }
        }

        // --- Update DDS outputs ---
        // All 4 DDS channels use the SAME frequency (ref_freq * multiplier)
        // but each gets a different phase offset (ideal quadrature).
        //
        // Strategy:
        //   1. Write all 4 channels' registers (freq + phase) WITHOUT ioUpdate
        //   2. Send ONE ioUpdate at the end to make all changes take effect simultaneously
        //   This ensures all 4 DDS channels update atomically.

        bool need_update = false;

#if ENABLE_SERIAL_DDS_DEBUG
        bool force_output = debug_dds_direct_freq_en;
#else
        bool force_output = false;
#endif

        if (ref_present || force_output) {
            uint32_t output_freq = 0;
#if ENABLE_SERIAL_DDS_DEBUG
            if (force_output) {
                output_freq = debug_dds_direct_freq_hz;
            } else
#endif
            {
                uint64_t tmp = (uint64_t)ref_freq * (uint64_t)multiplier_n;
                if (tmp > 0xFFFFFFFFULL) tmp = 0xFFFFFFFFULL;
                output_freq = (uint32_t)tmp;
            }

            // Clamp to AD9959 max output (Nyquist: sys_clk / 2)
            uint32_t max_out = current_sys_clk / 2;
            if (output_freq > max_out) {
                output_freq = max_out;
            }

            // For each channel, compute the output phase:
            // output_phase = ideal_phase (perfect quadrature)
            // The DDS hardware ensures precise phase relationships.
            for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
                uint8_t dds_ch = gpio_to_dds[ch];
                double out_phase = ideal_phase_deg[ch];

                // Check if we need to update this channel
                if (measured_freq[ch] != last_printed_freq[ch] ||
                    last_output_phase[ch] < 0 || state_changed || multiplier_changed
#if ENABLE_SERIAL_DDS_DEBUG
                    || debug_dds_direct_changed
#endif
                    ) {
                    selectChannel(dds_ch);
                    setSmartFrequencyAndPhase((double)output_freq, out_phase, true);
                    last_output_phase[ch] = out_phase;
                    need_update = true;
                }
            }

            // Single atomic IO_UPDATE for all channels
            if (need_update) {
                ioUpdate();
            }
        } else {
            // No reference signal — turn off all DDS outputs
            if (state_changed) {
                for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
                    uint8_t dds_ch = gpio_to_dds[ch];
                    selectChannel(dds_ch);
                    setSmartFrequencyAndPhase(0, 0, true);
                    last_output_phase[ch] = -1;
                }
                ioUpdate();
                need_update = true;
            }
        }

        // Reset multiplier_changed after DDS update
        multiplier_changed = false;

#if ENABLE_SERIAL_DDS_DEBUG
        debug_dds_direct_changed = false;
#endif

        // --- Print status ---
        if (need_update || state_changed) {
            Serial.print("[n=");
            Serial.print(multiplier_n);
            Serial.print(" f=");
            Serial.print(ref_present ? ref_freq : 0);
            Serial.print("Hz] ");

            for (uint8_t out = 0; out < NUM_CHANNELS; out++) {
                uint8_t ch = dds_to_gpio[out];
                // Signal name (print in DDS OUT order)
                const char* sig_names_out[] = {"SIN+", "COS+", "SIN-", "COS-"};
                Serial.print(sig_names_out[out]);
                Serial.print("(G");
                Serial.print(ch);
                Serial.print("->I");
                Serial.print(out);
                Serial.print("): ");

                if (signal_present[ch]) {
                    Serial.print("m=");
                    Serial.print(measured_phase_deg[ch], 1);
                    Serial.print("° o=");
                    Serial.print(ideal_phase_deg[ch], 0);
                    Serial.print("°");
                } else {
                    Serial.print("OFF");
                }
                Serial.print("  ");
                last_printed_freq[ch] = measured_freq[ch];
            }
            Serial.println();
        }
    }
}