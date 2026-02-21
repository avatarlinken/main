/*
 * 四路正交信号频率倍频系统
 *
 * 功能：对 SIN+/COS+/SIN-/COS- 四路输入做过零检测与频率测量，
 *       经 AD9959 输出 2~20 倍频的正交信号，经 OPA4354 放大至 1Vpp。
 *
 * 代码结构（便于维护与二次开发）：
 *   1. 配置与宏
 *   2. 引脚与硬件映射
 *   3. 频率测量状态变量
 *   4. 幅度校准参数
 *   5. AD9959 寄存器与 SPI 常量
 *   6. DDS 底层接口
 *   7. DDS 频率/幅度/相位设置
 *   8. 频率测量
 *   9. 相位测量
 *  10. 串口解析
 *  11. 主流程 setup / loop
 */

#include <Arduino.h>
#include <SPI.h>
#include "hardware/pio.h"
#include "measure.pio.h"
#include "phase_measure.pio.h"

// =============================================================================
// 1. 配置与宏
// =============================================================================

#define ENABLE_SERIAL_DDS_DEBUG  0
#define DEFAULT_MULTIPLIER_N     1

// 倍频系数范围（串口设置）
#define MULTIPLIER_MIN            1
#define MULTIPLIER_MAX            20

// 频率显示稳定死区：变化在此范围内不更新 measured_freq，避免跳动
#define FREQ_STABILITY_DEADZONE_HZ  5

// 双样本确认：新频率相对 pending 的允许偏差（百分数，用于 98/102）
#define TWO_SAMPLE_PCT_LOW       98
#define TWO_SAMPLE_PCT_HIGH      102

// DDS 基准时钟与 PLL
#define DDS_REF_CLK_HZ           25000000UL
#define PLL_MULT_MIN             4
#define PLL_MULT_MAX             20
#define IO_UPDATE_DELAY_US       2
#define DDS_RESET_DELAY_MS       10
#define PLL_SETTLE_MS            100

// SPI
#define SPI_CLOCK_HZ             2000000

// FTW/CPOW 换算
#define FTW_SCALE                4294967296.0   // 2^32
#define CPOW_FULL_SCALE          16384.0        // 14-bit, 0~16383 -> 0~360°
#define CPOW_DEG_SCALE           (CPOW_FULL_SCALE / 360.0)

// 门控法测频：freq = delta_edges * US_PER_S / delta_us
#define US_PER_S                 1000000ULL

// 主循环节流
#define UPDATE_INTERVAL_MS       100

// =============================================================================
// 2. 引脚与硬件映射
// =============================================================================

#define DDS_RESET_PIN            17
#define DDS_SCLK_PIN             18
#define DDS_MOSI_PIN             19
#define DDS_CS_PIN               20
#define DDS_UPDATE_PIN           21

#define FREQ_INPUT_PIN_0         0
#define FREQ_INPUT_PIN_1         1
#define FREQ_INPUT_PIN_2         2
#define FREQ_INPUT_PIN_3         3
#define NUM_CHANNELS             4

static PIO   pio_freq = pio0;
static uint  pio_offset;
static const uint freq_pins[NUM_CHANNELS] = {
    FREQ_INPUT_PIN_0, FREQ_INPUT_PIN_1, FREQ_INPUT_PIN_2, FREQ_INPUT_PIN_3
};

#define PHASE_REF_PIN            FREQ_INPUT_PIN_1
#define NUM_PHASE_CH             3
static PIO   pio_phase = pio1;
static uint  pio_phase_offset;
static const uint phase_target_pins[NUM_PHASE_CH] = {
    FREQ_INPUT_PIN_0, FREQ_INPUT_PIN_2, FREQ_INPUT_PIN_3
};
static const uint8_t phase_target_ch[NUM_PHASE_CH] = {0, 2, 3};

// GPIO -> DDS 通道：GPIO0(COS-)->IOUT3, GPIO1(SIN+)->IOUT0, ...
static const uint8_t gpio_to_dds[NUM_CHANNELS] = {3, 0, 1, 2};
// DDS 输出顺序打印：OUT0=SIN+(G1), OUT1=COS+(G2), OUT2=SIN-(G3), OUT3=COS-(G0)
static const uint8_t dds_to_gpio[NUM_CHANNELS] = {1, 2, 3, 0};

// 理想相位（度）：0=COS-(270°), 1=SIN+(0°), 2=COS+(90°), 3=SIN-(180°)
static const double ideal_phase_deg[NUM_CHANNELS] = {270.0, 0.0, 90.0, 180.0};
static double measured_phase_deg[NUM_CHANNELS] = {270.0, 0.0, 90.0, 180.0};

// =============================================================================
// 3. 频率测量状态变量
// =============================================================================

static uint16_t multiplier_n = DEFAULT_MULTIPLIER_N;
static bool     multiplier_changed = false;

#if ENABLE_SERIAL_DDS_DEBUG
static bool   debug_dds_direct_freq_en = false;
static uint32_t debug_dds_direct_freq_hz = 0;
static bool   debug_dds_direct_changed = false;
#endif

static uint32_t measured_freq[NUM_CHANNELS] = {0};
static uint32_t last_printed_freq[NUM_CHANNELS] = {0};

// EMA 滤波：alpha 越小越平滑，越大响应越快；超过 RESET_THRESHOLD 则重置 EMA
static const double FREQ_EMA_ALPHA = 0.01;
static const double FREQ_EMA_RESET_THRESHOLD = 0.01;
static double freq_ema[NUM_CHANNELS] = {0};
static bool   freq_ema_valid[NUM_CHANNELS] = {false};

static uint32_t prev_edge_count[NUM_CHANNELS] = {0};
static uint32_t prev_sample_us[NUM_CHANNELS] = {0};
static bool     prev_sample_valid[NUM_CHANNELS] = {false};
static uint32_t pending_freq[NUM_CHANNELS] = {0};
static bool     signal_present[NUM_CHANNELS] = {false};
static bool     prev_signal_present[NUM_CHANNELS] = {false};

static String serial_buf = "";

// =============================================================================
// 4. 幅度校准参数
// =============================================================================

const double AMP_RF = 2400.0;
const double AMP_RI = 1000.0;
const double TARGET_VPP_MV = 1000.0;

const double CAL_FREQ_LOW  = 1000.0;
const double CAL_VPP_LOW_MV  = 1216.0;
const double CAL_FREQ_HIGH = 2000000.0;
const double CAL_VPP_HIGH_MV = 1149.0;

#define ACR_MAX                  1023.0
#define CLAMP_ACR(x)             ((x) > ACR_MAX ? ACR_MAX : (x))
const double CAL_ACR_LOW  = CLAMP_ACR(ACR_MAX * TARGET_VPP_MV / CAL_VPP_LOW_MV);
const double CAL_ACR_HIGH = CLAMP_ACR(ACR_MAX * TARGET_VPP_MV / CAL_VPP_HIGH_MV);

// =============================================================================
// 5. AD9959 寄存器与全局 DDS 状态
// =============================================================================

#define CSR_REG    0x00
#define FR1_REG    0x01
#define CFTW0_REG  0x04
#define CPOW0_REG  0x05
#define ACR_REG    0x06
#define CW1_REG    0x0A

uint32_t current_sys_clk = DDS_REF_CLK_HZ;

// =============================================================================
// 6. DDS 底层接口
// =============================================================================

static void ioUpdate(void) {
    digitalWrite(DDS_UPDATE_PIN, HIGH);
    delayMicroseconds(IO_UPDATE_DELAY_US);
    digitalWrite(DDS_UPDATE_PIN, LOW);
    delayMicroseconds(IO_UPDATE_DELAY_US);
}

static void writeRegisterMulti(uint8_t regAddr, uint8_t* data, uint8_t len) {
    SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(DDS_CS_PIN, LOW);
    SPI.transfer(regAddr);
    for (uint8_t i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }
    digitalWrite(DDS_CS_PIN, HIGH);
    SPI.endTransaction();
}

static void setPLL(uint8_t multiplier) {
    if (multiplier < PLL_MULT_MIN) multiplier = PLL_MULT_MIN;
    if (multiplier > PLL_MULT_MAX) multiplier = PLL_MULT_MAX;

    current_sys_clk = DDS_REF_CLK_HZ * multiplier;

    uint8_t vco_gain = (current_sys_clk >= 250000000UL) ? 0x80 : 0x00;
    uint8_t fr1_byte0 = vco_gain | ((multiplier & 0x1F) << 2) | 0x03;
    uint8_t fr1_config[] = {fr1_byte0, 0x00, 0x00};

    writeRegisterMulti(FR1_REG, fr1_config, 3);
    ioUpdate();
    delay(PLL_SETTLE_MS);
}

static void selectChannel(uint8_t ch) {
    uint8_t csr_data[] = { (uint8_t)(0x10 << ch) };
    writeRegisterMulti(CSR_REG, csr_data, 1);
}

static void selectAllChannels(void) {
    uint8_t csr_data[] = {0xF0};
    writeRegisterMulti(CSR_REG, csr_data, 1);
    ioUpdate();
}

// =============================================================================
// 7. DDS 频率 / 幅度 / 相位设置
// =============================================================================

static uint16_t calculateSmartACR(double freq) {
    if (freq <= CAL_FREQ_LOW)  return (uint16_t)CAL_ACR_LOW;
    if (freq >= CAL_FREQ_HIGH) return (uint16_t)CAL_ACR_HIGH;

    double slope = (CAL_ACR_HIGH - CAL_ACR_LOW) / (CAL_FREQ_HIGH - CAL_FREQ_LOW);
    double acr_val = CAL_ACR_LOW + (freq - CAL_FREQ_LOW) * slope;
    return (uint16_t)acr_val;
}

static void setSmartFrequencyAndPhase(double freq_hz, double phase_deg, bool defer_update) {
    if (current_sys_clk == 0) return;

    uint16_t acr_val = calculateSmartACR(freq_hz);
    uint8_t acr_data[3] = {
        0x00,
        (uint8_t)(0x10 | ((acr_val >> 8) & 0x03)),
        (uint8_t)(acr_val & 0xFF)
    };
    writeRegisterMulti(ACR_REG, acr_data, 3);

    while (phase_deg < 0.0)   phase_deg += 360.0;
    while (phase_deg >= 360.0) phase_deg -= 360.0;
    uint16_t cpow = (uint16_t)(phase_deg * CPOW_DEG_SCALE) & 0x3FFF;
    uint8_t cpow_data[] = {
        (uint8_t)((cpow >> 8) & 0x3F),
        (uint8_t)(cpow & 0xFF)
    };
    writeRegisterMulti(CPOW0_REG, cpow_data, 2);

    uint32_t ftw = (uint32_t)((freq_hz * FTW_SCALE) / current_sys_clk);
    uint8_t ftw_data[] = {
        (uint8_t)((ftw >> 24) & 0xFF),
        (uint8_t)((ftw >> 16) & 0xFF),
        (uint8_t)((ftw >> 8) & 0xFF),
        (uint8_t)(ftw & 0xFF)
    };
    writeRegisterMulti(CFTW0_REG, ftw_data, 4);
    writeRegisterMulti(CW1_REG, ftw_data, 4);

    if (!defer_update) ioUpdate();
}

static void setSmartFrequency(double freq_hz) {
    setSmartFrequencyAndPhase(freq_hz, 0.0, false);
}

// =============================================================================
// 8. 频率测量
// =============================================================================

static void sampleFrequency(uint8_t ch) {
    uint32_t now_count = frequency_counter_read_count(pio_freq, ch);
    uint32_t now_us = micros();

    if (prev_sample_valid[ch]) {
        uint32_t delta_edges = now_count - prev_edge_count[ch];
        uint32_t delta_us = now_us - prev_sample_us[ch];

        if (delta_us > 0 && delta_edges > 0) {
            uint32_t freq = (uint32_t)(
                ((uint64_t)delta_edges * US_PER_S + delta_us / 2) / delta_us
            );

            if (pending_freq[ch] == 0 ||
                (freq > pending_freq[ch] * TWO_SAMPLE_PCT_LOW / 100 &&
                 freq < pending_freq[ch] * TWO_SAMPLE_PCT_HIGH / 100)) {

                double threshold_ratio = 1.0 + FREQ_EMA_RESET_THRESHOLD;
                if (!freq_ema_valid[ch] ||
                    freq > freq_ema[ch] * threshold_ratio ||
                    freq < freq_ema[ch] / threshold_ratio) {
                    freq_ema[ch] = (double)freq;
                    freq_ema_valid[ch] = true;
                } else {
                    freq_ema[ch] = FREQ_EMA_ALPHA * (double)freq +
                        (1.0 - FREQ_EMA_ALPHA) * freq_ema[ch];
                }

                uint32_t new_freq = (uint32_t)(freq_ema[ch] + 0.5);
                if (measured_freq[ch] == 0) {
                    measured_freq[ch] = new_freq;
                } else {
                    int32_t delta = (int32_t)new_freq - (int32_t)measured_freq[ch];
                    if (delta < -(int32_t)FREQ_STABILITY_DEADZONE_HZ ||
                        delta > (int32_t)FREQ_STABILITY_DEADZONE_HZ) {
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

// =============================================================================
// 9. 相位测量
// =============================================================================

static void samplePhase(void) {
    uint32_t ref_freq = measured_freq[1];
    if (ref_freq < 1) return;

    double period_counts = (double)F_CPU / (double)ref_freq / 2.0;

    for (uint8_t i = 0; i < NUM_PHASE_CH; i++) {
        uint32_t delta_counts = phase_measure_read(pio_phase, i);
        if (delta_counts > 0 && period_counts > 0) {
            double phase = (double)delta_counts / period_counts * 360.0;
            phase = 360.0 - phase;
            while (phase >= 360.0) phase -= 360.0;
            while (phase < 0.0) phase += 360.0;
            uint8_t ch = phase_target_ch[i];
            measured_phase_deg[ch] = phase;
        }
    }
    measured_phase_deg[1] = 0.0;
}

// =============================================================================
// 10. 串口解析
// =============================================================================

static void parseSerialInput(void) {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            serial_buf.trim();
            if (serial_buf.length() > 0) {
                String cmd = serial_buf;
                cmd.trim();

#if ENABLE_SERIAL_DDS_DEBUG
                if (cmd.length() >= 2 && (cmd[0] == 'D' || cmd[0] == 'd')) {
                    uint32_t hz = (uint32_t)cmd.substring(1).toInt();
                    if (hz > 0) {
                        debug_dds_direct_freq_en = true;
                        debug_dds_direct_freq_hz = hz;
                        debug_dds_direct_changed = true;
                        Serial.print("DDS direct freq set to: ");
                        Serial.println(debug_dds_direct_freq_hz);
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
                    if (eq_pos >= 0) val_str = cmd.substring(eq_pos + 1);
                    val_str.trim();
                    int val = val_str.toInt();
                    if (val >= MULTIPLIER_MIN && val <= MULTIPLIER_MAX) {
                        multiplier_n = (uint16_t)val;
                        multiplier_changed = true;
                        Serial.print("Multiplier set to: ");
                        Serial.println(multiplier_n);
                    } else {
                        Serial.print("Invalid multiplier! Range: ");
                        Serial.print(MULTIPLIER_MIN);
                        Serial.print("~");
                        Serial.println(MULTIPLIER_MAX);
                    }
                }
            }
            serial_buf = "";
        } else {
            serial_buf += c;
        }
    }
}

// =============================================================================
// 11. 主流程 setup / loop
// =============================================================================

static unsigned long last_update_ms = 0;
static double last_output_phase[NUM_CHANNELS] = {-1, -1, -1, -1};

void setup(void) {
    Serial.begin(115200);
    delay(2000);
    Serial.println("AD9959 Frequency Multiplier System Starting...");
    Serial.print("System clock (F_CPU): ");
    Serial.print((uint32_t)F_CPU);
    Serial.println(" Hz");

    pinMode(DDS_RESET_PIN, OUTPUT);
    pinMode(DDS_CS_PIN, OUTPUT);
    pinMode(DDS_UPDATE_PIN, OUTPUT);
    digitalWrite(DDS_RESET_PIN, LOW);
    digitalWrite(DDS_CS_PIN, HIGH);
    digitalWrite(DDS_UPDATE_PIN, LOW);

    SPI.setRX(16);
    SPI.setSCK(DDS_SCLK_PIN);
    SPI.setTX(DDS_MOSI_PIN);
    SPI.begin();

    digitalWrite(DDS_RESET_PIN, HIGH);
    delay(DDS_RESET_DELAY_MS);
    digitalWrite(DDS_RESET_PIN, LOW);
    delay(DDS_RESET_DELAY_MS);

    setPLL(4);
    selectAllChannels();
    setSmartFrequency(0);

    pio_offset = pio_add_program(pio_freq, &frequency_counter_program);
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        frequency_counter_program_init(pio_freq, i, pio_offset, freq_pins[i]);
    }

    pio_phase_offset = pio_add_program(pio_phase, &phase_measure_program);
    for (uint8_t i = 0; i < NUM_PHASE_CH; i++) {
        phase_measure_program_init(pio_phase, i, pio_phase_offset,
                                  PHASE_REF_PIN, phase_target_pins[i]);
    }

    Serial.println("System Ready.");
    Serial.println("  GPIO 0=COS-, 1=SIN+(ref), 2=COS+, 3=SIN-");
    Serial.println("  DDS OUT: 0=SIN+, 1=COS+, 2=SIN-, 3=COS-");
    Serial.print("  Serial: n=");
    Serial.print(MULTIPLIER_MIN);
    Serial.print("~");
    Serial.print(MULTIPLIER_MAX);
    Serial.println(" to set multiplier");
    Serial.print("  Current multiplier: ");
    Serial.println(multiplier_n);
    Serial.println("Waiting for input signals...");
}

void loop(void) {
    parseSerialInput();

    unsigned long now = millis();
    if (now - last_update_ms < UPDATE_INTERVAL_MS) return;
    last_update_ms = now;

    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        sampleFrequency(ch);
    }
    samplePhase();

    uint32_t ref_freq = measured_freq[1];
    bool ref_present = (ref_freq >= 1);

    bool state_changed = false;
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        prev_signal_present[ch] = signal_present[ch];
        signal_present[ch] = (measured_freq[ch] >= 1);
        if (signal_present[ch] != prev_signal_present[ch]) state_changed = true;
    }

    bool need_update = false;
#if ENABLE_SERIAL_DDS_DEBUG
    bool force_output = debug_dds_direct_freq_en;
#else
    bool force_output = false;
#endif

    if (ref_present || force_output) {
        uint32_t output_freq;
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

        uint32_t max_out = current_sys_clk / 2;
        if (output_freq > max_out) output_freq = max_out;

        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            uint8_t dds_ch = gpio_to_dds[ch];
            double out_phase = ideal_phase_deg[ch];

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

        if (need_update) ioUpdate();
    } else {
        if (state_changed) {
            for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
                selectChannel(gpio_to_dds[ch]);
                setSmartFrequencyAndPhase(0, 0, true);
                last_output_phase[ch] = -1;
            }
            ioUpdate();
            need_update = true;
        }
    }

    multiplier_changed = false;
#if ENABLE_SERIAL_DDS_DEBUG
    debug_dds_direct_changed = false;
#endif

    if (need_update || state_changed) {
        Serial.print("[n=");
        Serial.print(multiplier_n);
        Serial.print(" f=");
        Serial.print(ref_present ? ref_freq : 0);
        Serial.print("Hz] ");

        for (uint8_t out = 0; out < NUM_CHANNELS; out++) {
            uint8_t ch = dds_to_gpio[out];
            const char* sig_names[] = {"SIN+", "COS+", "SIN-", "COS-"};
            Serial.print(sig_names[out]);
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
