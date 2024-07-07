#include "midiguitar.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/// Intermediate variables used to calculate gamma.
struct gamma {
  float phi, psi, alpha, lambda, mu;
};

/// Does an in place update to gamma for the given order.
static void updateGamma(uint8_t order, uint16_t nPitches, uint16_t nPitchesOld,
                        uint16_t nPitchesOldOld, struct gamma vars[MP],
                        float gamma[], const float ccVectors[][N_FFT_GRID]) {
  const float(*const tf)[N_FFT_GRID] = ccVectors;
  const float(*const hf)[N_FFT_GRID] = ccVectors + 2;
  if (order == 1) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      gamma[i] = 1 / (tf[0][i] + hf[0][i]);
      vars[i].psi = gamma[i];
      vars[i].phi = gamma[i] * tf[1][i];
    }
    return;
  } else if (order == 2) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      const float t1 = tf[0][i] + hf[0][i];
      const float t2 = tf[1][i] + hf[1][i];
      const float t4 = 1 / (t1 * (tf[0][i] + hf[2][i]) - t2 * t2);
      vars[i].alpha = t2 * gamma[i];
      gamma[MP + i] = -t2 * t4;
      gamma[MP + nPitches + i] = t1 * t4;
    }
    return;
  }
  for (uint16_t i = 0; i < nPitches; ++i) {
    vars[i].lambda = tf[order - 1][i] + hf[order - 3][i];
    vars[i].mu = 0;
  }
  for (uint8_t k = 0; k < order - 2; ++k) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      const float t = tf[order - 2 - k][i] + hf[k + order - 2][i];
      const uint16_t j = k * nPitchesOld + i;
      vars[i].lambda -= vars[j].psi * t;
      vars[i].mu = vars[j].phi * t;
    }
  }
  struct gamma t[MP];
  memcpy(t, vars, sizeof(t));
  for (uint8_t k = 0; k < order - 2; ++k) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      const uint16_t j = nPitches * k + i;
      const uint16_t it = nPitchesOld * k + i;
      const float g = gamma[MP * (order - 2) + nPitchesOld * k + i];
      vars[j].phi = t[it].phi + vars[i].lambda * g;
      vars[j].psi = t[it].psi + vars[i].mu * g;
    }
  }
  for (uint16_t i = 0; i < nPitches; ++i) {
    const uint16_t j = nPitches * (order - 2) + i;
    const float g = gamma[(MP + nPitchesOld) * (order - 2) + i];
    vars[j].phi = vars[i].lambda * g;
    vars[j].psi = vars[i].mu * g;
  }
  float t1[nPitches] = {};
  for (uint8_t k = 0; k < order - 1; ++k) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      const float x = tf[order - 1 - k][i] + hf[order - 1 + k][i];
      t1[i] += x * gamma[MP * (order - 2) + nPitchesOld * k + i];
    }
  }
  float t2[nPitches] = {};
  for (uint16_t i = 0; i < nPitches; ++i) {
    t2[i] = vars[i].alpha - t1[i];
    vars[i].alpha = t1[i];
  }
  float t3[nPitches * (order - 1)] = {};
  for (uint8_t k = 0; k < order - 1; ++k) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      t3[k * nPitches + i] =
          t2[i] * gamma[MP * (order - 2) + nPitchesOld * k + i];
      if (k)
        t3[k * nPitches + i] +=
            gamma[MP * (order - 2) + nPitchesOld * (k - 1) + i];
      if (k != order - 2) {
        t3[k * nPitches + i] +=
            gamma[MP * (order - 2) + nPitchesOld * (k + 1) + i];
        t3[k * nPitches + i] -=
            gamma[MP * (order - 3) + nPitchesOldOld * k + i];
      }
      const float t5 =
          vars[nPitches * (order - 2) + i].psi * vars[nPitches * k + i].phi;
      t3[k * nPitches + i] += t5;
      const float t6 =
          vars[nPitches * (order - 2) + i].phi * vars[nPitches * k + i].psi;
      t3[k * nPitches + i] -= t6;
    }
  }
  bzero(t1, sizeof(t1));
  for (uint8_t k = 0; k < order - 1; ++k)
    for (uint16_t i = 0; i < nPitches; ++i)
      t1[i] +=
          (tf[order - 1 - k][i] + hf[order - 1 + k][i]) * t3[k * nPitches + i];
  for (uint16_t i = 0; i < nPitches; ++i) {
    float t5 = t1[i] / gamma[(MP + nPitchesOld) * (order - 2) + i];
    t5 += tf[0][i] + hf[(order - 1) << 1][i];
    gamma[(MP + nPitches) * (order - 1) + i] = 1 / t5;
  }
  float t4[nPitches] = {};
  for (uint16_t i = 0; i < nPitches; ++i)
    t4[i] = gamma[(MP + nPitches) * (order - 1) + i] /
            gamma[(MP + nPitchesOld) * (order - 2) + i];
  for (uint8_t k = 0; k < order - 1; ++k)
    for (uint16_t i = 0; i < nPitches; ++i)
      gamma[MP * (order - 1) + nPitches * k + i] = t3[nPitches * k + i] * t4[i];
}

/// Does an in place update to gamma p for the given order.
static void updateGammaP(uint8_t order, uint16_t nPitches, uint16_t nPitchesOld,
                         uint16_t nPitchesOldOld, float alpha[], float gamma[],
                         const float ccVectors[][N_FFT_GRID]) {
  const float(*const tf)[N_FFT_GRID] = ccVectors;
  const float(*const hf)[N_FFT_GRID] = ccVectors + 2;
  if (order == 1) {
    for (uint16_t i = 0; i < nPitches; ++i)
      gamma[i] = 1 / (tf[0][i] + hf[0][i]);
    return;
  } else if (order == 2) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      const float t1 = tf[0][i] - hf[0][i];
      const float t2 = tf[1][i] - hf[1][i];
      const float t4 = 1 / (t1 * (tf[0][i] - hf[2][i]) - t2 * t2);
      alpha[i] = t2 * gamma[i];
      gamma[MP + i] = -t2 * t4;
      gamma[MP + nPitches + i] = t1 + t4;
    }
    return;
  }
  float t[nPitches] = {};
  for (uint8_t k = 0; k < order - 1; ++k) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      const float t2 = tf[order - 1 - k][i] + hf[order - 1 + k][i];
      t[i] += t2 * gamma[MP * (order - 2) + nPitchesOld * k];
    }
  }
  float t2[nPitches] = {};
  for (uint16_t i = 0; i < nPitches; ++i) {
    t2[i] = alpha[i] - t[i];
    alpha[i] = t[i];
  }
  float t3[nPitches * (order - 1)] = {};
  for (uint8_t k = 0; k < order - 1; ++k) {
    for (uint16_t i = 0; i < nPitches; ++i) {
      t3[k * nPitches + i] =
          t2[i] * gamma[MP * (order - 2) + nPitchesOld * k + i];
      if (k)
        t3[k * nPitches + i] +=
            gamma[MP * (order - 2) + nPitchesOld * (k - 1) + i];
      if (k != order - 2) {
        t3[k * nPitches + i] +=
            gamma[MP * (order - 2) + nPitchesOld * (k + 1) + i];
        t3[k * nPitches + i] -=
            gamma[MP * (order - 3) + nPitchesOldOld * k + i];
      }
    }
  }
  bzero(t, sizeof(t));
  for (uint8_t k = 0; k < order - 1; ++k)
    for (uint16_t i = 0; i < nPitches; ++i)
      t[i] +=
          (tf[order - 1 - k][i] - hf[order - 1 + k][i]) * t3[k * nPitches + i];
  for (uint16_t i = 0; i < nPitches; ++i) {
    float t4 = t[i] / gamma[(MP + nPitchesOld) * (order - 2) + i];
    t4 += tf[0][i] - hf[(order - 1) << 1][i];
    gamma[(MP + nPitches) * (order - 1) + i] = 1 / t4;
  }
  float t4[nPitches] = {};
  for (uint16_t i = 0; i < nPitches; ++i)
    t4[i] = gamma[(MP + nPitches) * (order - 1) + i] /
            gamma[(MP + nPitchesOld) * (order - 2) + i];
  for (uint8_t k = 0; k < order - 1; ++k)
    for (uint16_t i = 0; i < nPitches; ++i)
      gamma[MP * (order - 1) + nPitches * k + i] = t3[nPitches * k + i] * t4[i];
}

int main() {
  printf("#pragma once\n#include \"midiguitar.h\"\n\n");
  printf("const float FFT_SHIFT_VECTOR[MP << 1] = {\n  ");
  for (uint16_t i = 0; i < MP; ++i) {
    const float v = (M_PI * (SAMPLES - 1.0) / N_FFT_GRID) * i;
    printf("%f, %f,", cosf(v), sinf(v));
    if (i + 1 < MP)
      printf(" ");
  }
  static float ccVectors[(MAX_MODEL_ORDER + 1) << 1][N_FFT_GRID];
  uint16_t maxFftIndex = MAX_FFT_INDEX;
  uint16_t nPitches = maxFftIndex - MIN_FFT_INDEX + 1;
  for (uint16_t i = 0; i < nPitches; ++i) {
    ccVectors[0][i] = 0.5 * SAMPLES;
  }
  ccVectors[0][nPitches] = nanf("1");
  for (uint8_t k = 1; k < (MAX_MODEL_ORDER + 1) << 1; ++k) {
    if (k % 2) {
      const uint16_t maxFftIndexOld = maxFftIndex;
      maxFftIndex = MIN(MAX_FFT_INDEX, N_FFT_GRID / (k + 1) - 1);
      nPitches -= maxFftIndexOld - maxFftIndex;
    }
    for (uint16_t i = 0; i < nPitches; ++i) {
      const float t = M_PI * k * (i + MIN_FFT_INDEX) / N_FFT_GRID;
      ccVectors[k][i] = 0.5 * sinf(t * SAMPLES) / sinf(t);
    }
    ccVectors[k][nPitches] = nanf("1");
  }
  maxFftIndex = MAX_FFT_INDEX;
  nPitches = maxFftIndex - MIN_FFT_INDEX + 1;
  uint16_t nPitchesOld = 0;
  float gamma1[MAX_MODEL_ORDER * MP] = {};
  struct gamma vars[MP] = {};
  float gamma2[MAX_MODEL_ORDER * MP] = {};
  float alpha2[MP] = {};
  for (uint8_t order = 1; order < MAX_MODEL_ORDER; ++order) {
    const uint16_t maxFftIndexOld = maxFftIndex;
    maxFftIndex = MIN(MAX_FFT_INDEX, N_FFT_GRID / (2.0 * order) - 1);
    const uint16_t nPitchesOldOld = nPitchesOld;
    nPitchesOld = nPitches;
    nPitches -= maxFftIndexOld - maxFftIndex;
    updateGamma(order, nPitches, nPitchesOld, nPitchesOldOld, vars, gamma1,
                ccVectors);
    updateGammaP(order, nPitches, nPitchesOld, nPitchesOldOld, alpha2, gamma2,
                 ccVectors);
  }
  printf("\n};\n\nconst float GAMMA1[MAX_MODEL_ORDER * MP] = {\n");
  for (uint8_t order = 0; order < MAX_MODEL_ORDER; ++order) {
    printf("  ");
    for (uint16_t i = 0; i < MP; ++i) {
      printf("%f,", gamma1[order * MP + i]);
      if (i + 1 < MP)
        printf(" ");
    }
    printf("\n");
  }
  printf("};\n\nconst float GAMMA2[MAX_MODEL_ORDER * MP] = {\n");
  for (uint8_t order = 0; order < MAX_MODEL_ORDER; ++order) {
    printf("  ");
    for (uint16_t i = 0; i < MP; ++i) {
      printf("%f,", gamma2[order * MP + i]);
      if (i + 1 < MP)
        printf(" ");
    }
    printf("\n");
  }
  printf("};\n\nconst float *CC_VECTORS[(MAX_MODEL_ORDER + 1) << 1] = {\n");
  for (uint8_t i = 0; i < (MAX_MODEL_ORDER + 1) << 1; ++i) {
    printf("  (const float[]){");
    for (uint16_t j = 0; !isnan(ccVectors[i][j]); ++j) {
      printf("%f", ccVectors[i][j]);
      if (!isnan(ccVectors[i][j + 1]))
        printf(", ");
    }
    printf("},\n");
  }
  printf("};\n");
  return 0;
}
