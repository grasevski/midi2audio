#include "fastf0nls.h"
#include "arm_math.h"
#include "lookup.h"
#include <stdbool.h>

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
static void compute(const float x[SAMPLE_SIZE],
                    float omega_0h[MAX_MODEL_ORDER]) {
  float dftData[MP << 1];
  {
    arm_rfft_fast_instance_f32 fft;
    FFT_INIT(&fft);
    float p[N_FFT_GRID], p1[N_FFT_GRID << 1];
    arm_copy_f32(x, p, SAMPLE_SIZE);
    arm_fill_f32(0, p + SAMPLE_SIZE, N_FFT_GRID - SAMPLE_SIZE);
    arm_rfft_fast_f32(&fft, p, p1, 0);
    arm_cmplx_mult_cmplx_f32(FFT_SHIFT_VECTOR, p1, dftData, MP);
  }
  float lsSol1[MP], lsSol2[MP];
  int maxFftIndex = MAX_FFT_INDEX, nPitches = MAX_FFT_INDEX - MIN_FFT_INDEX + 1;
  for (int order = 1; order <= MAX_MODEL_ORDER; ++order) {
    const int maxFftIndexOld = maxFftIndex;
    maxFftIndex = MIN(MAX_FFT_INDEX, N_FFT_GRID / (2 * order) - 1);
    const int nPitchesOld = nPitches;
    nPitches -= maxFftIndexOld - maxFftIndex;
    const int maxPitchGrid = MIN_FFT_INDEX + nPitches - 1;
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
static float compute_obj(float omega, const float x[SAMPLE_SIZE], int order,
                         float ac[order], float as[order]) {
  float tmp = omega * -0.5 * (SAMPLE_SIZE - 1), t[2 * (order + 1)];
  t[0] = SAMPLE_SIZE;
  for (int i = 1; i <= 2 * order + 1; ++i)
    t[i] = 0.5 * arm_sin_f32(0.5 * omega * SAMPLE_SIZE * i) /
           arm_sin_f32(0.5 * omega * i);
  float cn[SAMPLE_SIZE], sn[SAMPLE_SIZE], bc[order], bs[order];
  arm_fill_f32(0, bc, order);
  arm_fill_f32(0, bs, order);
  for (int i = 0; i < SAMPLE_SIZE; ++i) {
    cn[i] = arm_cos_f32(tmp);
    sn[i] = arm_sin_f32(tmp);
    bc[0] += cn[i] * x[i];
    bs[0] += sn[i] * x[i];
    tmp += omega;
  }
  if (order > 1) {
    float t1[SAMPLE_SIZE], t2[SAMPLE_SIZE];
    float *cnm2 = t1, *snm2 = t2;
    for (int i = 0; i < SAMPLE_SIZE; ++i) {
      cnm2[i] = 2 * cn[i] * cn[i] - 1;
      snm2[i] = 2 * cn[i] * sn[i];
      bc[1] += cnm2[i] * x[i];
      bs[1] += snm2[i] * x[i];
    }
    if (order > 2) {
      float t3[SAMPLE_SIZE], t4[SAMPLE_SIZE];
      float *cnm1 = t3, *snm1 = t4;
      arm_copy_f32(cnm2, cnm1, SAMPLE_SIZE);
      arm_copy_f32(snm2, snm1, SAMPLE_SIZE);
      arm_copy_f32(cn, cnm2, SAMPLE_SIZE);
      arm_copy_f32(sn, snm2, SAMPLE_SIZE);
      for (int l = 3; l <= order; ++l) {
        for (int i = 0; i < SAMPLE_SIZE; ++i) {
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
                                 const float x[SAMPLE_SIZE]) {
  const int r = 2, rho = 1, v = 1;
  const float delta = r * 1.5, u = ((float)SAMPLE_SIZE) / r;
  float energy, lnBF[MAX_MODEL_ORDER + 1];
  arm_power_f32(x, SAMPLE_SIZE, &energy);
  uint32_t order = 0;
  lnBF[0] = 0;
  for (int k = 1; k <= MAX_MODEL_ORDER; ++k) {
    const float lnW = logf(MIN((0.5 / k), PITCH_MAX) - PITCH_MIN);
    float ac[k], as[k];
    const float Jomega = compute_obj(omega_0h[k - 1], x, k, ac, as);
    const float R2 = Jomega / energy;
    const float w = (SAMPLE_SIZE - 2 * k - delta) / r;
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
      const float sigma2 = energy / SAMPLE_SIZE - eS * Jomega / SAMPLE_SIZE;
      float ms = 0;
      for (int ell = 1; ell <= k; ++ell)
        ms += ac[ell - 1] * ac[ell - 1] * ell * ell +
              as[ell - 1] * as[ell - 1] * ell * ell;
      const float lnH =
          logf(fabs(-gh * SAMPLE_SIZE * (SAMPLE_SIZE * SAMPLE_SIZE - 1) /
                    (r * r * 6 * (1 + gh) * sigma2))) +
          logf(ms);
      const float lnBFg =
          logf(1 + gh) * -k +
          logf((energy / SAMPLE_SIZE) / sigma2) * (((float)SAMPLE_SIZE) / r);
      lnBF[k] = lnBFg + logf(gh * (delta - r)) - logf(r) -
                logf(1 + gh) * (delta / r) - lnW +
                logf(2 * M_PI) * ((rho + 1) * 0.5) + lngamma / 2 - lnH / 2;
    } else {
      order = k;
      break;
    }
  }
  if (!order) {
    arm_max_f32(lnBF, MAX_MODEL_ORDER + 1, &energy, &order);
  }
  return order;
}

float fastf0nls(const float x[SAMPLE_SIZE]) {
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
