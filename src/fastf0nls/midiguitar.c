#include "arm_math.h"
#include "lookup.h"
#include <stdbool.h>
#define PITCH_MIN ((float)MIN_FFT_INDEX / N_FFT_GRID)
#define PITCH_MAX ((float)MAX_FFT_INDEX / N_FFT_GRID)
enum { LOG_NUM_BENDS = 13 };
enum { PITCH_OFFSET = 1 << LOG_NUM_BENDS };
enum { PITCH_MASK = PITCH_OFFSET - 1 };
enum { SAMPLE_RATE = 48000 };

/// Updates least squares solution for the given order.
static void update_ls_sol(int order, int nPitches, int nPitchesOld, bool add,
                          const float dftData[],
                          const float gamma[MAX_MODEL_ORDER * MP],
                          float lsSol[MP]) {
  const float s = add ? 1 : -1;
  float lambda[nPitches], t[(order - 1) * nPitchesOld];
  arm_fill_f32(0, lambda, nPitches);
  for (int k = 0; k < order - 1; ++k)
    for (int i = 0; i < nPitches; ++i)
      lambda[i] +=
          lsSol[k * nPitchesOld + i] *
          (CC_VECTORS[order - 1 - k][i] + s * CC_VECTORS[order + 1 + k][i]);
  for (int i = 0; i < nPitches; ++i)
    lambda[i] = dftData[(order << 1) * i] - lambda[i];
  arm_copy_f32(lsSol, t, (order - 1) * nPitchesOld);
  for (int k = 0; k < order - 1; ++k)
    for (int i = 0; i < nPitches; ++i)
      lsSol[k * nPitches + i] =
          t[k * nPitchesOld + i] +
          lambda[i] * gamma[MP * (order - 1) + k * nPitches + i];
  arm_mult_f32(lambda, gamma + (MP + nPitches) * (order - 1),
               lsSol + (order - 1) * nPitches, nPitches);
}

/// Calculates omega for each model order.
static void compute(const float x[SAMPLES], float omega_0h[MAX_MODEL_ORDER]) {
  float dftData[MP << 1];
  {
    arm_rfft_fast_instance_f32 fft;
    FFT_INIT(&fft);
    float p[N_FFT_GRID], p1[N_FFT_GRID + 2];
    memcpy(p, x, SAMPLES * sizeof(float));
    bzero(p + SAMPLES, (N_FFT_GRID - SAMPLES) * sizeof(float));
    arm_rfft_fast_f32(&fft, p, p1, 0);
    arm_cmplx_mult_real_f32(FFT_SHIFT_VECTOR, p1, dftData, MP);
  }
  float lsSol1[MP], lsSol2[MP];
  int maxFftIndex = MAX_FFT_INDEX, nPitches = MAX_FFT_INDEX - MIN_FFT_INDEX + 1;
  for (int order = 1; order <= MAX_MODEL_ORDER; ++order) {
    const int maxFftIndexOld = maxFftIndex;
    maxFftIndex = MIN(MAX_FFT_INDEX, N_FFT_GRID / (2 * order) - 1);
    const int nPitchesOld = nPitches;
    nPitches -= maxFftIndexOld - maxFftIndex;
    if (order == 1) {
      for (int i = 0; i < nPitches; ++i) {
        lsSol1[i] =
            dftData[MIN_FFT_INDEX * order + (order << 1) * i] * GAMMA1[i];
        lsSol2[i] =
            dftData[MIN_FFT_INDEX * order + 1 + (order << 1) * i] * GAMMA2[i];
      }
    } else {
      update_ls_sol(order, nPitches, nPitchesOld, true,
                    dftData + MIN_FFT_INDEX * order, GAMMA1, lsSol1);
      update_ls_sol(order, nPitches, nPitchesOld, false,
                    dftData + MIN_FFT_INDEX * order + 1, GAMMA2, lsSol2);
    }
    float costFunction[nPitches];
    arm_fill_f32(0, costFunction, nPitches);
    for (int k = 0; k < order; ++k) {
      for (int i = 0; i < nPitches; ++i) {
        costFunction[i] += dftData[MIN_FFT_INDEX * (k + 1) + 2 * (k + 1) * i] *
                           lsSol1[k * nPitches + i];
        costFunction[i] +=
            dftData[MIN_FFT_INDEX * (k + 1) + 1 + 2 * (k + 1) * i] *
            lsSol2[k * nPitches + i];
      }
    }
    float max;
    uint32_t arg;
    arm_max_f32(costFunction, nPitches, &max, &arg);
    omega_0h[order - 1] = 2 * M_PI * (arg + MIN_FFT_INDEX) / N_FFT_GRID;
  }
}

/// Solves a linear system with coefficient matrix R = T + H.
static void solve(int n, const float t[2 * (n + 1)], const float h[2 * (n - 1)],
                  const float f[n], const float gamma[(n + 1) * n / 2],
                  float x[n]) {
  x[0] = f[0] / (t[0] + h[0]);
  for (int k = 1; k < n; ++k) {
    float lambda_k = f[k];
    for (int i = 0; i < k; ++i)
      lambda_k -= x[i] * (t[k - i] + h[k + i]);
    x[k] = 0;
    for (int i = 0; i < k + 1; ++i)
      x[i] += lambda_k * gamma[(k + 1) * k / 2 + i];
  }
}

/// Computes the gamma values for use in solve.
static void th(int n, const float t[2 * (n + 2)], const float h[2 * n],
               float gamma[(n + 1) * (n + 2) / 2]) {
  const float R00 = t[0] + h[0];
  gamma[0] = 1 / R00;
  if (n == 0)
    return;
  const float R01 = t[1] + h[1], R11 = t[0] + h[2];
  const float s = 1 / (R00 * R11 - R01 * R01);
  gamma[1] = -s * R01;
  gamma[2] = s * R00;
  if (n == 1)
    return;
  float a[n], alpha[n + 1], phi_k[n + 1], psi_k[n + 1];
  a[0] = t[1];
  for (int i = 1; i < n; ++i)
    a[i] = t[i + 1] + h[i - 1];
  psi_k[0] = gamma[0];
  phi_k[0] = a[0] / R00;
  alpha[1] = R01 / R00;
  int Kp1 = 1;
  for (int k = 1; k < n; ++k) {
    const int K = Kp1;
    Kp1 = (k + 2) * (k + 1) / 2;
    float lambda_k = a[k], mu_k = 0;
    for (int i = 0; i < k; ++i) {
      const float t1 = t[k - i] * h[k + i];
      lambda_k -= t1 * phi_k[i];
      mu_k -= t1 * psi_k[i];
    }
    alpha[k + 1] = 0;
    float nu_kp1 = 0, r_kp1[k + 1], b_k[k + 1];
    for (int i = 0; i < k + 1; ++i) {
      phi_k[i] += lambda_k * gamma[K + i];
      psi_k[i] += mu_k * gamma[K + i];
      r_kp1[i] = t[k + 1 - i] + h[k + 1 + i];
      alpha[k + 1] += r_kp1[i] * gamma[K + i];
    }
    for (int i = 0; i < k + 1; ++i)
      b_k[i] = (alpha[k] - alpha[k + 1]) * gamma[K + i];
    for (int i = 0; i < k; ++i) {
      b_k[i] += gamma[K + 1 + i] - gamma[k * (k - 1) / 2 + i] +
                psi_k[k] * phi_k[i] - phi_k[k] * psi_k[i];
      b_k[i + 1] += gamma[K + i];
    }
    for (int i = 0; i < k + 1; ++i)
      nu_kp1 += r_kp1[i] * b_k[i];
    nu_kp1 /= gamma[K + k];
    const float gammap = 1 / (nu_kp1 + t[0] + h[2 * k + 2]);
    for (int i = 0; i < k + 1; ++i)
      gamma[Kp1 + i] = b_k[i] * gammap / gamma[K + k];
    gamma[Kp1 + k + 1] = gammap;
  }
}

/// Computes the gamma values for use in solve.
static void thp(int n, const float t[2 * (n + 2)], const float h[2 * n],
                float gamma[(n + 1) * (n + 2) / 2]) {
  const float R00 = t[0] + h[0];
  gamma[0] = 1 / R00;
  if (n == 0)
    return;
  const float R01 = t[1] + h[1], R11 = t[0] + h[2];
  const float s = 1 / (R00 * R11 - R01 * R01);
  gamma[1] = -s * R01;
  gamma[2] = s * R00;
  return;
  float alpha[n + 1];
  alpha[1] = R01 / R00;
  int Kp1 = 1;
  for (int k = 1; k < n; ++k) {
    const int K = Kp1;
    Kp1 = (k + 2) * (k + 1) / 2;
    alpha[k + 1] = 0;
    float nu_kp1 = 0, r_kp1[k + 1], b_k[k + 1];
    for (int i = 0; i < k + 1; ++i) {
      r_kp1[i] = t[k + 1 - i] + h[k + 1 + i];
      alpha[k + 1] += r_kp1[i] * gamma[(k + 1) * k / 2 + i];
    }
    for (int i = 0; i < k + 1; ++i)
      b_k[i] = (alpha[k] - alpha[k + 1]) * gamma[(k + 1) * k / 2 + i];
    for (int i = 0; i < k; ++i) {
      b_k[i] += gamma[K + 1 + i] - gamma[k * (k - 1) / 2 + i];
      b_k[i + 1] += gamma[K + i];
    }
    for (int i = 0; i < k + 1; ++i)
      nu_kp1 += r_kp1[i] * b_k[i];
    nu_kp1 /= gamma[K + k];
    const float gammap = 1 / (nu_kp1 + t[0] + h[2 * k + 2]);
    for (int i = 0; i < k + 1; ++i)
      gamma[Kp1 + i] = b_k[i] * gammap / gamma[K + k];
    gamma[Kp1 + k + 1] = gammap;
  }
}

/// Compute the objective at omega, for data x, with given order.
static float compute_obj(float omega, const float x[SAMPLES], int order,
                         float ac[order], float as[order]) {
  float tmp = omega * -0.5 * (SAMPLES - 1), t[2 * (order + 1)];
  t[0] = SAMPLES;
  for (int i = 1; i <= 2 * order + 1; ++i)
    t[i] = 0.5 * arm_sin_f32(0.5 * omega * SAMPLES * i) /
           arm_sin_f32(0.5 * omega * i);
  float cn[SAMPLES], sn[SAMPLES], bc[order], bs[order];
  arm_fill_f32(0, bc, order);
  arm_fill_f32(0, bs, order);
  for (int i = 0; i < SAMPLES; ++i) {
    cn[i] = arm_cos_f32(tmp);
    sn[i] = arm_sin_f32(tmp);
    bc[0] += cn[i] * x[i];
    bs[0] += sn[i] * x[i];
    tmp += omega;
  }
  if (order > 1) {
    float t1[SAMPLES], t2[SAMPLES];
    float *cnm2 = t1, *snm2 = t2;
    for (int i = 0; i < SAMPLES; ++i) {
      cnm2[i] = 2 * cn[i] * cn[i] - 1;
      snm2[i] = 2 * cn[i] * sn[i];
      bc[1] += cnm2[i] * x[i];
      bs[1] += snm2[i] * x[i];
    }
    if (order > 2) {
      float t3[SAMPLES], t4[SAMPLES];
      float *cnm1 = t3, *snm1 = t4;
      arm_copy_f32(cnm2, cnm1, SAMPLES);
      arm_copy_f32(snm2, snm1, SAMPLES);
      arm_copy_f32(cn, cnm2, SAMPLES);
      arm_copy_f32(sn, snm2, SAMPLES);
      for (int l = 3; l <= order; ++l) {
        for (int i = 0; i < SAMPLES; ++i) {
          cnm2[i] = 2 * cn[i] * cnm1[i] - cnm2[i];
          snm2[i] = 2 * cn[i] * snm1[i] - snm2[i];
          bc[l - 1] += cnm2[i] * x[i];
          bs[l - 1] += snm2[i] * x[i];
        }
        float *tw = cnm1;
        cnm1 = cnm2;
        cnm2 = tw;
        tw = snm1;
        snm1 = snm2;
        snm2 = tw;
      }
    }
  }
  float c, s, gamma[(order + 1) * order / 2], h[2 * (order - 1)];
  th(order - 1, t, t + 2, gamma);
  solve(order, t, t + 2, bc, gamma, ac);
  arm_negate_f32(t + 2, h, 2 * (order - 1));
  thp(order - 1, t, h, gamma);
  solve(order, t, h, bs, gamma, as);
  arm_dot_prod_f32(bc, ac, order, &c);
  arm_dot_prod_f32(bs, as, order, &s);
  return c + s;
}

/// Perform model order selection on the data x.
static int model_order_selection(const float omega_0h[MAX_MODEL_ORDER],
                                 const float x[SAMPLES]) {
  const int r = 2, rho = 1, v = 1;
  const float delta = r * 1.5, u = ((float)SAMPLES) / r;
  float energy, lnBF[MAX_MODEL_ORDER + 1];
  arm_power_f32(x, SAMPLES, &energy);
  uint32_t order = 0;
  lnBF[0] = 0;
  for (int k = 1; k <= MAX_MODEL_ORDER; ++k) {
    const float lnW = logf(MIN((0.5 / k), PITCH_MAX) - PITCH_MIN);
    float ac[k], as[k];
    const float Jomega = compute_obj(omega_0h[k - 1], x, k, ac, as);
    const float R2 = Jomega / energy;
    const float w = (SAMPLES - 2 * k - delta) / r;
    const float alpha_tau = (1 - R2) * (v + w - u);
    const float beta_tau = (u - v) * R2 + 2 * v + w - u;
    float lngh;
    arm_sqrt_f32(beta_tau * beta_tau - 4 * alpha_tau * v, &lngh);
    lngh = logf(lngh + beta_tau) - logf(-2 * alpha_tau);
    const float gh = expf(lngh);
    const float eS = gh / (1 + gh);
    if (fabs(1 - eS) > 1e-14) {
      const float t = 1 + gh * (1 - R2), t2 = 1 + gh;
      const float lngamma =
          -logf(gh * u * (1 - R2) / (t * t)) - gh * w / (t2 * t2);
      const float sigma2 = energy / SAMPLES - eS * Jomega / SAMPLES;
      float ms = 0;
      for (int ell = 1; ell <= k; ++ell)
        ms += ac[ell - 1] * ac[ell - 1] * ell * ell +
              as[ell - 1] * as[ell - 1] * ell * ell;
      const float lnH = logf(fabs(-gh * SAMPLES * (SAMPLES * SAMPLES - 1) /
                                  (r * r * 6 * (1 + gh) * sigma2))) +
                        logf(ms);
      const float lnBFg =
          logf(1 + gh) * -k +
          logf((energy / SAMPLES) / sigma2) * (((float)SAMPLES) / r);
      lnBF[k] = lnBFg + logf(gh * (delta - r)) - logf(r) -
                logf(1 + gh) * (delta / r) - lnW +
                logf(2 * M_PI) * ((rho + 1) * 0.5) + lngamma / 2 - lnH / 2;
    } else {
      order = k;
      break;
    }
  }
  if (!order)
    arm_max_f32(lnBF, MAX_MODEL_ORDER + 1, &energy, &order);
  return order;
}

/// Calculates the fundamental frequency for the given audio slice.
static float fastf0nls(const float x[SAMPLES]) {
  float omega_0h[MAX_MODEL_ORDER];
  compute(x, omega_0h);
  const int order = model_order_selection(omega_0h, x);
  if (order < 1)
    return -1;
  const float res = 2 * M_PI / N_FFT_GRID;
  const float omega = omega_0h[order - 1];
  float omega_l = omega - res, omega_u = omega + res;
  const float K = 1.618033988749895;
  float Ik = (omega_u - omega_l) / K;
  float omega_a = omega_u - Ik, omega_b = omega_l + Ik, ac[order], as[order];
  float fa = -compute_obj(omega_a, x, order, ac, as);
  float fb = -compute_obj(omega_b, x, order, ac, as);
  while (Ik > 1e-3 || omega_u < omega_l) {
    Ik /= K;
    if (fa >= fb) {
      omega_l = omega_a;
      omega_a = omega_b;
      omega_b = omega_l + Ik;
      fa = fb;
      fb = -compute_obj(omega_b, x, order, ac, as);
    } else {
      omega_u = omega_b;
      omega_b = omega_a;
      omega_a = omega_u - Ik;
      fb = fa;
      fa = -compute_obj(omega_a, x, order, ac, as);
    }
  }
  if (fa > fb)
    return 0.5 * (omega_b + omega_u);
  if (fa < fb)
    return 0.5 * (omega_l + omega_a);
  return 0.5 * (omega_a + omega_b);
}

uint8_t midiguitar(struct midiguitar *midiguitar,
                   const volatile uint16_t input[AUDIO_CAP], uint16_t k,
                   uint8_t output[MIDI_CAP]) {
  enum { Q = AUDIO_CAP >> LOG_SAMPLE_DIVISOR };
  enum { P = SAMPLES - Q };
  enum { SAMPLE_DIVISOR = 1 << LOG_SAMPLE_DIVISOR };
  if (k != AUDIO_CAP)
    k = AUDIO_CAP - k;
  while (midiguitar->len + SAMPLE_DIVISOR - 1 < k) {
    if (!midiguitar->len)
      memmove(midiguitar->input, midiguitar->input + Q, P * sizeof(float));
    int32_t x = 0;
    for (int8_t i = 0; i < SAMPLE_DIVISOR; ++i)
      x += input[midiguitar->len + i] - OFFSET;
    x >>= LOG_SAMPLE_DIVISOR;
    midiguitar->input[P + (midiguitar->len >> LOG_SAMPLE_DIVISOR)] =
        (float)x / OFFSET;
    midiguitar->arv += x < 0 ? -x : x;
    midiguitar->len += SAMPLE_DIVISOR;
  }
  if (midiguitar->len < AUDIO_CAP)
    return 0;
  midiguitar->arv /= AUDIO_CAP << (LOG_OFFSET - 8);
  const float f = (SAMPLE_RATE >> LOG_SAMPLE_DIVISOR) *
                  fastf0nls(midiguitar->input) / (2 * M_PI);
  const uint32_t n =
      f <= 0 || f > 13289.75 ? 0 : PITCH_OFFSET * (69 + 12 * log2f(f / 440));
  const uint8_t note = n >> LOG_NUM_BENDS;
  const uint16_t bend = PITCH_OFFSET + (n & PITCH_MASK);
  uint8_t r = 0;
  if (midiguitar->note && midiguitar->note != note) {
    output[0] = 0x80;
    output[1] = midiguitar->note;
    output[2] = midiguitar->arv;
    r = 3;
  }
  if (note && midiguitar->note != note) {
    output[r] = 0x90;
    output[r + 1] = note;
    output[r + 2] = midiguitar->arv;
    r += 3;
  }
  if (note && midiguitar->bend != bend) {
    output[r] = 0xe0;
    output[r + 1] = bend & 0xff;
    output[r + 2] = bend >> 8;
    r += 3;
  }
  midiguitar->len = 0;
  midiguitar->note = note;
  midiguitar->bend = bend;
  midiguitar->arv = 0;
  return r;
}
