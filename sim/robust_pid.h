/*
 * robust_pid.h
 *
 *  Created on: Feb 12, 2024
 *      Author: Joe Corteo
 *      Website: https://www.mujoseph.com/
 *      GitHub: https://github.com/mujoseph
 */

#ifndef ROBUST_PID_H_
#define ROBUST_PID_H_

//////////////////////////////////////////////////////////////////////////////////////////////
//  INCLUDE
//////////////////////////////////////////////////////////////////////////////////////////////

#include "platform_types.h"


#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
//  DEFINES
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
//  TYPES
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef int8_t
typedef signed char int8_t;
#endif

typedef float robust_pid_data_t;

typedef struct _robust_pid_cfg
{
    robust_pid_data_t              T;
    robust_pid_data_t              LPF_3dB_Hz;
    robust_pid_data_t              Kp;
    robust_pid_data_t              Ki;
    robust_pid_data_t              Kd;

}robust_pid_cfg_t;

typedef struct _robust_pid_branch_err
{
    robust_pid_data_t              P_err;  
    robust_pid_data_t              I_err;   
    robust_pid_data_t              D_err;   

}robust_pid_branch_err_t;

// Run-time variables
typedef struct _robust_pid_rt
{
    robust_pid_branch_err_t        branch_err;
    robust_pid_data_t              u[2];    // Past outputs
    robust_pid_data_t              e[2];    // Past errors
    robust_pid_data_t              y[2];    // Past measurements

}robust_pid_rt_t;

typedef struct _robust_pid_iir_coeff
{
    robust_pid_data_t              c[10];       // PID coefficients
    robust_pid_data_t              d[3];        // PD coefficients
}robust_pid_iir_coeff_t;

typedef struct _robust_pid
{
    robust_pid_cfg_t                 cfg;            // Context configuration structure
    robust_pid_rt_t                  rt;             // Run-time variables
    robust_pid_iir_coeff_t           coeff;          // IIR Coefficients

}robust_pid_t, *ROBUST_PID_CTX;

//////////////////////////////////////////////////////////////////////////////////////////////
//  API
//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  Initialize a controller context from a configuration.
 *
 * @details A NULL @p ctx or @p pCfg, or a non-positive sample period or
 *          derivative-filter cutoff, returns false and leaves @p ctx unchanged.
 *          On success, zeroes @p ctx, copies @p pCfg, and precomputes the
 *          discrete IIR coefficients. The copy is independent of the caller's
 *          struct afterward.
 *
 * @param[in,out] ctx   Context to initialize. Untouched when this returns false.
 * @param[in]     pCfg  Configuration to copy. T and LPF_3dB_Hz must be > 0.
 *
 * @return true if initialization is successful, false otherwise.
 */
extern bool robust_pid_initCtx( ROBUST_PID_CTX ctx, robust_pid_cfg_t *pCfg );

/**
 * @brief  Run one control step. Invoke every robust_pid_cfg_t.T seconds.
 *
 * @details Error is e = r - y. Which gains are non-zero selects the law:
 *          PID (Kp, Ki, and Kd all non-zero), PD (Kp and Kd non-zero, Ki == 0),
 *          or PI (Kp and Ki non-zero, Kd == 0). Any other combination,
 *          including proportional-only, returns 0. PID and PD apply the IIR
 *          coefficients from init. PI applies trapezoidal integration (T/2)
 *          and does not use those coefficients. Gains are applied here.
 *          Only the branch terms of the selected law are written.
 *
 * @param[in,out] ctx  Controller context. The e, y, and u delay lines advance
 *                     on every call, including when the output is forced to 0.
 * @param[in]     r    Setpoint.
 * @param[in]     y    Process measurement.
 *
 * @return Controller output for this step. The same value is stored in the
 *         output delay line. 0 when the gain combination is not PID, PD, or PI.
 */
extern robust_pid_data_t robust_pid_run( ROBUST_PID_CTX ctx, robust_pid_data_t r, robust_pid_data_t y );

/**
 * @brief  Clear history so the next step starts from rest.
 *
 * @details Zeroes the branch terms and the e, y, and u delay lines.
 *          Configuration and IIR coefficients are kept.
 *
 * @param[in,out] ctx  Controller context.
 */
extern void robust_pid_reset( ROBUST_PID_CTX ctx );

/**
 * @brief  Replace Kp, Ki, and Kd, then clear history.
 *
 * @details Gains are applied in the difference equation, so the IIR
 *          coefficients are left as they are. History is cleared so the next
 *          step does not mix old samples with the new gains. Ki == 0 selects
 *          PD, Kd == 0 selects PI, and all three non-zero selects PID. Any
 *          other combination, including Kp == 0, makes robust_pid_run() return 0.
 *
 * @param[in,out] ctx  Controller context.
 * @param[in]     Kp   Proportional gain. Must be non-zero for a recognized law.
 * @param[in]     Ki   Integral gain. Exactly 0 selects the PD law.
 * @param[in]     Kd   Derivative gain. Exactly 0 selects the PI law.
 */
extern void robust_pid_updateGains( ROBUST_PID_CTX ctx, robust_pid_data_t Kp, robust_pid_data_t Ki, robust_pid_data_t Kd );

/**
 * @brief  Set a new derivative-filter cutoff and rebuild the IIR coefficients.
 *
 * @details History is cleared first so samples taken at the old cutoff are
 *          not run through the new coefficients. Gains are left unchanged
 *          because they are applied in the difference equation, not stored
 *          in the coefficients.
 *
 * @param[in,out] ctx         Controller context.
 * @param[in]     LPF_3dB_Hz  Derivative low-pass cutoff, in Hertz. Must be > 0.
 *
 * @return true if update is successful, false otherwise.
 */
extern bool robust_pid_updateLPFCutoffFrequency( ROBUST_PID_CTX ctx, robust_pid_data_t LPF_3dB_Hz );

/**
 * @brief  Read the controller output stored from the last step.
 *
 * @param[in] ctx  Controller context.
 *
 * @return u[n-1], the output of the last robust_pid_run() call.
 *         0 if no step has run since init or reset.
 */
extern robust_pid_data_t robust_pid_get_output( ROBUST_PID_CTX ctx );

#ifdef __cplusplus
}
#endif

#endif // #ifndef ROBUST_PID_H_
