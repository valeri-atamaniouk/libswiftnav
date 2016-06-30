/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/track.h>

#include <string.h>
#include <math.h>

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** Multiplier for checking out-of bounds NSR */
#define CN0_SNV_NSR_MIN_MULTIPLIER (1e-16f)
/** Maximum supported NSR value (1/NSR_MIN_MULTIPLIER)*/
#define CN0_SNV_NSR_MIN            (1e16f)

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Signal-to-Noise Variance method \f$ C / N_0 \f$ estimator.
 *
 * The method uses the function for SNR computation:
 *
 * \f[
 *    \frac{C}{N_0}(n) = \frac{P_d}{P_tot-P_d}
 * \f]
 * where
 * \f[
 *    P_d(n) = (\frac{1}{2}(|I(n)|+|I(n-1)|))^2
 * \f]
 * \f[
 *    P_tot(n) = \frac{1}{2}(I(n)^2 + I(n-1)^2 + Q(n)^2 + Q(n-1)^2)
 * \f]
 *
 * \param s     The estimator state struct to initialize.
 * \param bw    The loop noise bandwidth in Hz.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 * \param f_s   Input sampling frequency in Hz.
 * \param f_i   Loop integration frequency in Hz.
 * \param alpha IIR coefficient for averaging sum approximation. The coefficient
 *              is related to moving average filter approximation.
 *
 * \return None
 */
void cn0_est_snv_init(cn0_est_state_t *s,
                      float bw,
                      float cn0_0,
                      float f_s,
                      float f_i,
                      float alpha)
{
  memset(s, 0, sizeof(*s));

  /* Normalize by sampling frequency and integration period */
  s->log_bw = 10.f*log10f(bw * f_i / f_s);
  s->alpha = alpha;
  s->snv.I_sum = 0.f;
  s->snv.P_tot = 0.f;
  s->snv.cn0_db = cn0_0;
  s->snv.cnt = 20;
}

/**
 * Computes \f$ C / N_0 \f$ with Signal-to-Noise Variance method.
 *
 * \param s Initialized estimator object.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_snv_update(cn0_est_state_t *s, float I, float Q)
{

  if (s->snv.cnt > 1) {
    s->snv.I_sum += fabsf(I);
    s->snv.P_tot += I * I + Q * Q;
    s->snv.cnt--;
  } else if (s->snv.cnt == 1) {
    s->snv.cnt = 0;
    s->snv.I_sum *= 0.05f;
    s->snv.P_tot *= 0.05f;
  } else {
    float I_sum = s->snv.I_sum * (1 - s->alpha) + s->alpha * fabsf(I);
    float P_tot = s->snv.P_tot * (1 - s->alpha) + s->alpha * (I * I + Q * Q);

    s->snv.I_sum = I_sum;
    s->snv.P_tot = P_tot;

    float P_s = I_sum * I_sum;
    float P_n = P_tot - P_s;

    float nsr;
    float nsr_db;

    /* Ensure the NSR is within the limit */
    if (P_s < P_n * CN0_SNV_NSR_MIN_MULTIPLIER)
      nsr = CN0_SNV_NSR_MIN;
    else
      nsr = P_n / P_s;

    nsr_db = 10.f * log10f(nsr);
    /* Compute and store updated CN0 */
    s->snv.cn0_db = s->log_bw - nsr_db;
  }

  return s->snv.cn0_db;
}

/** \} */
