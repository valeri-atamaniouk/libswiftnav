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
#define CN0_BL_NSR_MIN_MULTIPLIER (1e-6f)
/** Maximum supported NSR value (1/NSR_MIN_MULTIPLIER)*/
#define CN0_BL_NSR_MIN            (1e6f)
/** Maximum estimator output value */
#define CN0_BL_DB_MAX             (100.f)
/** Minimum estimator output value */
#define CN0_BL_DB_MIN             (0.f)

static float limit_cn0(float cn0)
{
  if (cn0 > CN0_BL_DB_MAX)
    return CN0_BL_DB_MAX;
  else if (cn0 < CN0_BL_DB_MIN)
    return CN0_BL_DB_MIN;
  else
    return cn0;
}

static float compute_nsr(float P_s, float P_n)
{
  /* Ensure the NSR is within the limit */
  if (P_s < P_n * CN0_BL_NSR_MIN_MULTIPLIER)
    return CN0_BL_NSR_MIN;
  else
    return P_n / P_s;
}

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Beaulieu's method \f$ C / N_0 \f$ estimator.
 *
 * The method uses the function for SNR computation:
 *
 * \f[
 *    \frac{C}{N_0}(n) = \frac{1}{2}(\frac{P_n(n)}{P_s(n)})
 * \f]
 * where
 * \f[
 *    P_n(n) = (|Q(n)|-|Q(n-1)|)^2
 * \f]
 * for original BL method.
 * \f[
 *    P_n(n) = (|I(n)|-|I(n-1)|)^2
 * \f]
 * for modified BL method.
 * \f[
 *    P_s(n) = \frac{1}{2}(I(n)^2 + I(n-1)^2)
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
 *
 * \sa cn0_est_bl_update
 * \sa cn0_est_bl_update_q
 */
void cn0_est_bl_init(cn0_est_state_t *s,
                     float bw,
                     float cn0_0,
                     float f_s,
                     float f_i,
                     float alpha)
{
  memset(s, 0, sizeof(*s));

  /* Normalize by sampling frequency and integration period */
  s->log_bw = 10.f * log10f(bw * f_i / f_s);
  s->alpha = alpha;
  s->bl.Q_prev_abs = -1.f;
  s->bl.nsr = powf(10.f, 0.1f * (s->log_bw - cn0_0));
}

/**
 * Computes \f$ C / N_0 \f$ with Beaulieu's method.
 *
 * \param s Initialized estimator object.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_bl_update(cn0_est_state_t *s, float I, float Q)
{
  /* Compute values for this iteration */
  float I_abs = fabsf(I);
  (void)Q;
  float I_prev_abs = s->bl.I_prev_abs;
  s->bl.I_prev_abs = I_abs;

  if (I_prev_abs < 0.f) {
    /* This is the first iteration, just update the prev state. */
  } else {
    float P_n;    /* Noise power */
    float P_s;    /* Signal power */
    float nsr;    /* Noise to signal ratio */

    P_n = I_abs - I_prev_abs;
    P_n *= P_n;
    P_s = 0.5f * (I * I + I_prev_abs * I_prev_abs);
    nsr = compute_nsr(P_s, P_n);
    s->bl.nsr = s->bl.nsr * (1.f - s->alpha) + nsr * s->alpha;
  }

  float res = limit_cn0(s->log_bw - 10.f * log10f(s->bl.nsr));

  return res;
}

/**
 * Computes \f$ C / N_0 \f$ with Beaulieu's method.
 *
 * \param s Initialized estimator object.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_bl_update_q(cn0_est_state_t *s, float I, float Q)
{
  /* Compute values for this iteration */
  float I_abs = fabsf(I);
  float Q_abs = fabsf(Q);
  float I_prev_abs = s->bl.I_prev_abs;
  float Q_prev_abs = s->bl.Q_prev_abs;
  s->bl.I_prev_abs = I_abs;
  s->bl.Q_prev_abs = Q_abs;

  if (I_prev_abs < 0.f) {
    /* This is the first iteration, just update the prev state. */
  } else {
    float P_n;    /* Noise power */
    float P_s;    /* Signal power */
    float nsr;    /* Noise to signal ratio */

    P_n = Q_abs - Q_prev_abs;
    P_n *= P_n;
    P_s = 0.5f * (I * I + I_prev_abs * I_prev_abs);
    nsr = compute_nsr(P_s, P_n);
    s->bl.nsr = s->bl.nsr * (1.f - s->alpha) + nsr * s->alpha;

  }

  float res = limit_cn0(s->log_bw - 10.f * log10f(s->bl.nsr));

  return res;
}

/** \} */
