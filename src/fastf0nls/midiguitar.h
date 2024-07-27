#pragma once
#include <stdint.h>
#define FFT_INIT arm_rfft_init_8192_q31
enum { SAMPLES = 600 };
enum { AUDIO_CAP = SAMPLES >> 2 };
enum { MIDI_CAP = 16 };
enum { MAX_MODEL_ORDER = 16 };
enum { N_FFT_GRID = 512 * MAX_MODEL_ORDER };
enum { MP = (N_FFT_GRID >> 1) + 1 };
enum { MIN_FFT_INDEX = 14, MAX_FFT_INDEX = N_FFT_GRID >> 1 };
enum { OFFSET = 1 << 23 };

struct midiguitar {
  uint8_t note, len;
  uint16_t bend;
  uint64_t arv;
  float input[SAMPLES];
};

/// Converts audio to midi.
uint8_t midiguitar(struct midiguitar *midiguitar,
                   const volatile uint32_t input[AUDIO_CAP], uint16_t k,
                   uint8_t output[MIDI_CAP]);
