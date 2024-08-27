#pragma once
#include <stdint.h>
#define FFT_INIT arm_rfft_fast_init_512_f32
enum { N_FFT_GRID = 512 };
enum { LOG_SAMPLE_DIVISOR = 1 };
enum { SAMPLES = 600 >> LOG_SAMPLE_DIVISOR };
enum { AUDIO_CAP = SAMPLES << LOG_SAMPLE_DIVISOR };
enum { MIDI_CAP = 16 };
enum { MAX_MODEL_ORDER = 16 };
enum { MP = (N_FFT_GRID >> 1) + 1 };
enum { MIN_FFT_INDEX = 1, MAX_FFT_INDEX = N_FFT_GRID >> 1 };
enum { LOG_OFFSET = 15 };
enum { OFFSET = 1 << LOG_OFFSET };

struct midiguitar {
  uint8_t note;
  uint16_t len, bend;
  uint32_t arv;
  float input[SAMPLES];
};

/// Converts audio to midi.
uint8_t midiguitar(struct midiguitar *midiguitar,
                   const volatile uint16_t input[AUDIO_CAP], uint16_t k,
                   uint8_t output[MIDI_CAP]);
