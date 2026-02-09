
#include "es8311.h"
#include "es8311_const.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cinttypes>
#include <cmath>

namespace esphome {
namespace es8311 {

static const char *const TAG = "es8311";

// Added for custom volume not sure custom volume was needed
static constexpr float MIN_DB = -50.0f;   // quiet floor (adjust -50 to -80)
static constexpr float MAX_DB = 0.0f;     // unity gain at 1.0
static constexpr float CURVE_EXPONENT = 1.0f;  // 2.0 = quadratic, 3.0 = cubic, 4.0 = steeper

// Mark the component as failed; use only in setup
#define ES8311_ERROR_FAILED(func) \
  if (!(func)) { \
    this->mark_failed(); \
    return; \
  }
// Return false; use outside of setup
#define ES8311_ERROR_CHECK(func) \
  if (!(func)) { \
    return false; \
  }

void ES8311::setup() {
  // Reset
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG00_RESET, 0x1F));
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG00_RESET, 0x00));

  ES8311_ERROR_FAILED(this->configure_clock_());
  ES8311_ERROR_FAILED(this->configure_format_());
  ES8311_ERROR_FAILED(this->configure_mic_());

  // Set initial volume
  this->set_volume(0.75);  // 0.75 = 0xBF = 0dB

  // Power up analog circuitry
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG0D_SYSTEM, 0x01));
  // Enable analog PGA, enable ADC modulator
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG0E_SYSTEM, 0x02));
  // Power up DAC
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG12_SYSTEM, 0x00));
  // Enable output to HP drive
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG13_SYSTEM, 0x10));
  // ADC Equalizer bypass, cancel DC offset in digital domain
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG1C_ADC, 0x6A));
  // Bypass DAC equalizer
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG37_DAC, 0x48));
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG44_GPIO, 0x50));
  // Power On
  ES8311_ERROR_FAILED(this->write_byte(ES8311_REG00_RESET, 0x80));
}

void ES8311::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ES8311 Audio Codec:\n"
                "  Use MCLK: %s\n"
                "  Use Microphone: %s\n"
                "  DAC Bits per Sample: %" PRIu8 "\n"
                "  Sample Rate: %" PRIu32,
                YESNO(this->use_mclk_), YESNO(this->use_mic_), this->resolution_out_, this->sample_frequency_);

  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Failed to initialize!");
    return;
  }
}

//custom volume
bool ES8311::set_volume(float volume) {
  volume = esphome::clamp(volume, 0.0f, 1.0f);

  // Power-law curve 
  float curved = std::pow(volume, CURVE_EXPONENT);

  // Now map the curved (0.0–1.0) value linearly to register 0–255
  uint8_t reg32 = esphome::remap<uint8_t, float>(curved, 0.0f, 1.0f, 0, 255);

  return this->write_byte(ES8311_REG32_DAC, reg32);
}

float ES8311::volume() {
  uint8_t reg32;
  if (!this->read_byte(ES8311_REG32_DAC, &reg32)) {
    return 0.0f;  // read failed → return min
  }

  // Reverse the mapping: register → linear register fraction
  float linear_reg = esphome::remap<float, uint8_t>(reg32, 0, 255, 0.0f, 1.0f);

  // Now apply the **inverse** curve to get back a perceptual-ish 0.0–1.0
  // For power-law:
  float perceptual = std::pow(linear_reg, 1.0f / CURVE_EXPONENT);

  return perceptual;
}
// Original volume code
// bool ES8311::set_volume(float volume) {
//  volume = clamp(volume, 0.0f, 1.0f);
//  uint8_t reg32 = remap<uint8_t, float>(volume, 0.0f, 1.0f, 0, 255);
//  return this->write_byte(ES8311_REG32_DAC, reg32);
//}

//float ES8311::volume() {
//  uint8_t reg32;
//  this->read_byte(ES8311_REG32_DAC, &reg32);
//  return remap<float, uint8_t>(reg32, 0, 255, 0.0f, 1.0f);
//}

uint8_t ES8311::calculate_resolution_value(ES8311Resolution resolution) {
  switch (resolution) {
    case ES8311_RESOLUTION_16:
      return (3 << 2);
    case ES8311_RESOLUTION_18:
      return (2 << 2);
    case ES8311_RESOLUTION_20:
      return (1 << 2);
    case ES8311_RESOLUTION_24:
      return (0 << 2);
    case ES8311_RESOLUTION_32:
      return (4 << 2);
    default:
      return 0;
  }
}

const ES8311Coefficient *ES8311::get_coefficient(uint32_t mclk, uint32_t rate) {
  for (const auto &coefficient : ES8311_COEFFICIENTS) {
    if (coefficient.mclk == mclk && coefficient.rate == rate)
      return &coefficient;
  }
  return nullptr;
}

bool ES8311::configure_clock_() {
  // ─────────────────────────────────────────────
  // Register 0x01 — Clock source & enable
  // 0xBF = external MCLK, not inverted, all clocks on
  // ─────────────────────────────────────────────
  uint8_t reg01 = 0x3F;  // enable internal clocks

  // ES8311: bit7 = 1 → use MCLK pin
  if (this->use_mclk_) {
    reg01 |= BIT(7);
  }

  // bit6 = MCLK inversion
  if (this->mclk_inverted_) {
    reg01 |= BIT(6);
  }

  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG01_CLK_MANAGER, reg01));

  // ─────────────────────────────────────────────
  // Compute MCLK frequency
  // (matches ESP-ADF behavior)
  // ─────────────────────────────────────────────
  uint32_t mclk_frequency =
      this->use_mclk_
          ? this->sample_frequency_ * this->mclk_multiple_
          : this->sample_frequency_ * this->resolution_out_ * 2;

  // ─────────────────────────────────────────────
  // Get coefficient table entry
  // ─────────────────────────────────────────────
  const auto *coefficient =
      get_coefficient(mclk_frequency, this->sample_frequency_);

  if (coefficient == nullptr) {
    ESP_LOGE(TAG,
             "Unable to configure %lu Hz with MCLK %lu Hz",
             this->sample_frequency_, mclk_frequency);
    return false;
  }

  // ─────────────────────────────────────────────
  // Register 0x02 — pre_div / pre_mult
  // (DO NOT OVERRIDE — ADF relies on these values)
  // Working system uses 0x18 here
  // ─────────────────────────────────────────────
  uint8_t reg02;
  ES8311_ERROR_CHECK(this->read_byte(ES8311_REG02_CLK_MANAGER, &reg02));
  reg02 &= 0x07;  // keep low bits
  //reg02 |= (coefficient->pre_div - 1) << 5;
  //reg02 |= (coefficient->pre_mult << 3);
  reg02 |= 0x18;
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG02_CLK_MANAGER, reg02));

  // ─────────────────────────────────────────────
  // Register 0x03 — FS mode / ADC OSR
  // ─────────────────────────────────────────────
  const uint8_t reg03 =
      (coefficient->fs_mode << 6) | coefficient->adc_osr;
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG03_CLK_MANAGER, reg03));

  // ─────────────────────────────────────────────
  // Register 0x04 — DAC OSR
  // ─────────────────────────────────────────────
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG04_CLK_MANAGER, coefficient->dac_osr));

  // ─────────────────────────────────────────────
  // Register 0x05 — ADC / DAC dividers
  // Working system uses 0x00 (÷1 / ÷1)
  // DO NOT FORCE ANY VALUES HERE
  // ─────────────────────────────────────────────
  const uint8_t reg05 = ((coefficient->adc_div - 1) << 4) | ((coefficient->dac_div - 1) << 0);
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG05_CLK_MANAGER, reg05));

  // ─────────────────────────────────────────────
  // Register 0x06 — BCLK divider & polarity
  // ─────────────────────────────────────────────
  uint8_t reg06;
  ES8311_ERROR_CHECK(this->read_byte(ES8311_REG06_CLK_MANAGER, &reg06));

  if (this->sclk_inverted_) {
    reg06 |= BIT(5);
  } else {
    reg06 &= ~BIT(5);
  }

  reg06 &= 0xE0;
  reg06 |= (coefficient->bclk_div < 19)
               ? (coefficient->bclk_div - 1)
               : coefficient->bclk_div;

  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG06_CLK_MANAGER, reg06));

  // ─────────────────────────────────────────────
  // Register 0x07 — LRCK high period
  // ─────────────────────────────────────────────
  uint8_t reg07;
  ES8311_ERROR_CHECK(this->read_byte(ES8311_REG07_CLK_MANAGER, &reg07));
  reg07 &= 0xC0;
  reg07 |= coefficient->lrck_h;
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG07_CLK_MANAGER, reg07));

  // ─────────────────────────────────────────────
  // Register 0x08 — LRCK low period
  // ─────────────────────────────────────────────
  ES8311_ERROR_CHECK(
      this->write_byte(ES8311_REG08_CLK_MANAGER, coefficient->lrck_l));

  return true;
}


bool ES8311::configure_format_() {
  // Configure I2S mode and format
  uint8_t reg00;
  ES8311_ERROR_CHECK(this->read_byte(ES8311_REG00_RESET, &reg00));
  reg00 &= 0xBF;
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG00_RESET, reg00));

  // Configure SDP in resolution
  uint8_t reg09 = calculate_resolution_value(this->resolution_in_);
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG09_SDPIN, reg09));

  // Configure SDP out resolution
  uint8_t reg0a = calculate_resolution_value(this->resolution_out_);
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG0A_SDPOUT, reg0a));

  // Successfully configured the format
  return true;
}

bool ES8311::configure_mic_() {
  uint8_t reg14 = 0x1A;  // Enable analog MIC and max PGA gain
  if (this->use_mic_) {
    reg14 |= BIT(6);  // Enable PDM digital microphone
  }
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG14_SYSTEM, reg14));

  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG16_ADC, this->mic_gain_));  // ADC gain scale up
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG17_ADC, 0xC8));             // Set ADC gain

  // Successfully configured the microphones
  return true;
}

bool ES8311::set_mute_state_(bool mute_state) {
  uint8_t reg31;

  this->is_muted_ = mute_state;

  if (!this->read_byte(ES8311_REG31_DAC, &reg31)) {
    return false;
  }

  if (mute_state) {
    reg31 |= BIT(6) | BIT(5);
  } else {
    reg31 &= ~(BIT(6) | BIT(5));
  }

  return this->write_byte(ES8311_REG31_DAC, reg31);
  delay(100);
}

}  // namespace es8311
}  // namespace esphome
