/*
 * robust_pid.c
 *
 *  Created on: Feb 12, 2024
 *      Author: Joe Corteo
 *      Website: https://www.mujoseph.com/
 *      GitHub: https://github.com/mujoseph
 */

//////////////////////////////////////////////////////////////////////////////////////////////
//  INCLUDE
//////////////////////////////////////////////////////////////////////////////////////////////

#include "robust_pid.h"
#include <string.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////////////////////////
//  LOCAL FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////

static inline void update_rt( ROBUST_PID_CTX ctx, robust_pid_data_t y, robust_pid_data_t err, robust_pid_data_t out );
static void compute_iir_coeff( ROBUST_PID_CTX ctx );

//////////////////////////////////////////////////////////////////////////////////////////////
//  CONSTANTS
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif // #ifndef M_PI


// Error indices
#define ENM1_IDX            0   // e[n-1]
#define ENM2_IDX            1   // e[n-2]

// Measurement indices
#define YNM1_IDX            0   // y[n-1]
#define YNM2_IDX            1   // y[n-2]

// Output indices
#define UNM1_IDX            0   // u[n-1]
#define UNM2_IDX            1   // u[n-2]

//////////////////////////////////////////////////////////////////////////////////////////////
//  MACROS
//////////////////////////////////////////////////////////////////////////////////////////////

#define POW2(X)             (X)*(X)

#define T()                 (ctx->cfg.T)

//////////////////////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS
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
bool robust_pid_initCtx( ROBUST_PID_CTX ctx, robust_pid_cfg_t *pCfg )
{
    if( ctx == NULL || pCfg == NULL )
    {
        return false;
    }
    if( pCfg->T <= 0 || pCfg->LPF_3dB_Hz <= 0 )
    {
        return false;
    }

    // Initialize context
    memset(ctx,0,sizeof(robust_pid_t));
    ctx->cfg = *pCfg;

    // Generate IIR coefficients for PID Controller
    compute_iir_coeff( ctx );
    return true;

}// robust_pid_initCtx

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
robust_pid_data_t robust_pid_run( ROBUST_PID_CTX ctx, robust_pid_data_t r, robust_pid_data_t y )
{
    robust_pid_data_t out = 0;
    robust_pid_data_t err = r - y;  // Compute error

    // PID
    if( ctx->cfg.Kp != 0 && ctx->cfg.Ki != 0 && ctx->cfg.Kd != 0 )
    {
        // P error = Kp*( e[n] - c1*e[n-1] - c2*e[n-2] )
        ctx->rt.branch_err.P_err = ctx->cfg.Kp*( err - ctx->coeff.c[0]*ctx->rt.e[0] - ctx->coeff.c[1]*ctx->rt.e[1] );

        // I_err = Ki*( c3*e[n] + c4*e[n-1] + c5*e[n-2] );
        ctx->rt.branch_err.I_err = ctx->cfg.Ki*( ctx->coeff.c[2]*err + ctx->coeff.c[3]*ctx->rt.e[0] + ctx->coeff.c[4]*ctx->rt.e[1] );

        // D_err = Kd*( c6*e[n] - c7*e[n-1] + c8*e[n-2] );
        ctx->rt.branch_err.D_err = ctx->cfg.Kd*( ctx->coeff.c[5]*err - ctx->coeff.c[6]*ctx->rt.e[0] + ctx->coeff.c[7]*ctx->rt.e[1] );

        // Compute output
        // out = P_err + I_err + D_err + c9*u[n-1] + c10*u[n-2];
        out = ctx->rt.branch_err.P_err
              + ctx->rt.branch_err.I_err
              + ctx->rt.branch_err.D_err
              + ctx->coeff.c[8]*ctx->rt.u[0]
              + ctx->coeff.c[9]*ctx->rt.u[1];
    }
    // PD
    else if(  ctx->cfg.Kp != 0 && ctx->cfg.Ki == 0 && ctx->cfg.Kd != 0 )
    {
        // P_err = Kp*(en + d1*enm1);
        ctx->rt.branch_err.P_err = ctx->cfg.Kp*(err + ctx->coeff.d[0]*ctx->rt.e[0]);

        // D_err = Kd*d2*(e[n]-e[n-1]);
        ctx->rt.branch_err.D_err = ctx->cfg.Kd*ctx->coeff.d[1]*( err - ctx->rt.e[0] );
        
        // Compute output
        // out = P_err + D_err + d3*u[n-1];
        out = ctx->rt.branch_err.P_err
              + ctx->rt.branch_err.D_err
              + ctx->coeff.d[2]*ctx->rt.u[0];
    }
    // PI
    else if(  ctx->cfg.Kp != 0 && ctx->cfg.Ki != 0 && ctx->cfg.Kd == 0 )
    {
        //P_err = Kp*(e[n]-e[n-1]);
        ctx->rt.branch_err.P_err = ctx->cfg.Kp*(err - ctx->rt.e[0]);

        // e1 = T/2;
        // I_err = Ki*e1*(e[n]+e[n-1]);
        ctx->rt.branch_err.I_err = ctx->cfg.Ki*(T()*0.5f)*( err + ctx->rt.e[0] );

        // Compute output
        // out = P_err + I_err + u[n-1];
        out = ctx->rt.branch_err.P_err
              + ctx->rt.branch_err.I_err
              + ctx->rt.u[0];
    }
    // Invalid topology
    else
    {
        out = 0;
    }

    // Update run-time variables
    update_rt( ctx, y, err, out );
    
    return out;

}// robust_pid_run

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
bool robust_pid_updateLPFCutoffFrequency( ROBUST_PID_CTX ctx, robust_pid_data_t LPF_3dB_Hz )
{
    if( ctx == NULL || LPF_3dB_Hz <= 0 )
    {
        return false;
    }
    robust_pid_reset( ctx );
    ctx->cfg.LPF_3dB_Hz = LPF_3dB_Hz;
    compute_iir_coeff( ctx );
    return true;
}// robust_pid_updateLPFCutoffFrequency

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
 * @param[in]     Kp   Proportional gain.
 * @param[in]     Ki   Integral gain. 
 * @param[in]     Kd   Derivative gain.
 */
void robust_pid_updateGains( ROBUST_PID_CTX ctx, robust_pid_data_t Kp, robust_pid_data_t Ki, robust_pid_data_t Kd )
{
    robust_pid_reset( ctx );
    ctx->cfg.Kp = Kp;
    ctx->cfg.Ki = Ki;
    ctx->cfg.Kd = Kd;

}// robust_pid_updateGains

/**
 * @brief  Clear history so the next step starts from rest.
 *
 * @details Zeroes the branch terms and the e, y, and u delay lines.
 *          Configuration and IIR coefficients are kept.
 *
 * @param[in,out] ctx  Controller context.
 */
void robust_pid_reset( ROBUST_PID_CTX ctx )
{
    memset(&ctx->rt.branch_err, 0, sizeof(ctx->rt.branch_err));
    memset(&ctx->rt.e, 0, sizeof(ctx->rt.e));
    memset(&ctx->rt.y, 0, sizeof(ctx->rt.y));
    memset(&ctx->rt.u, 0, sizeof(ctx->rt.u));

}// robust_pid_reset

/**
 * @brief  Read the controller output stored from the last step.
 *
 * @param[in] ctx  Controller context.
 *
 * @return u[n-1], the output of the last robust_pid_run() call.
 *         0 if no step has run since init or reset.
 */
robust_pid_data_t robust_pid_get_output( ROBUST_PID_CTX ctx )
{
    return ctx->rt.u[UNM1_IDX];
}// robust_pid_get_output

//////////////////////////////////////////////////////////////////////////////////////////////
//  STATIC FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  Advance the measurement, error, and output delay lines by one sample.
 *
 * @details Moves each [n-1] sample into [n-2], then stores the new measurement,
 *          error, and controller output. The stored output is the sample the
 *          next filtered step feeds back through u[n-1] and u[n-2].
 *
 * @param[in,out] ctx  Controller context.
 * @param[in]     y    Measurement for this step.
 * @param[in]     err  Tracking error for this step.
 * @param[in]     out  Controller output for this step.
 */
static inline void update_rt( ROBUST_PID_CTX ctx, robust_pid_data_t y, robust_pid_data_t err, robust_pid_data_t out )
{
    // Cycle Measurements
    ctx->rt.y[YNM2_IDX] = ctx->rt.y[YNM1_IDX];
    ctx->rt.y[YNM1_IDX] = y;

    // Cycle errors
    ctx->rt.e[ENM2_IDX] = ctx->rt.e[ENM1_IDX];
    ctx->rt.e[ENM1_IDX] = err;

    // Cycle outputs
    ctx->rt.u[UNM2_IDX] = ctx->rt.u[UNM1_IDX];
    ctx->rt.u[UNM1_IDX] = out;

}// update_rt

/**
 * @brief  Precompute the discrete coefficients for the filtered PID and PD laws.
 *
 * @details tau = 1/(2*pi*fc) is the derivative-filter time constant. c[0..9]
 *          are the PID coefficients and d[0..2] the PD coefficients, both from
 *          T and tau. Gains stay outside the coefficients and are applied in
 *          robust_pid_run(), so a gain change does not require this rebuild.
 *
 * @param[in,out] ctx  Controller context. Reads cfg.T and cfg.LPF_3dB_Hz;
 *                     writes coeff.
 */
static void compute_iir_coeff( ROBUST_PID_CTX ctx )
{
    robust_pid_data_t tau = 1.0f/(2.0f*(robust_pid_data_t)M_PI*ctx->cfg.LPF_3dB_Hz);
    robust_pid_data_t sigma1 = (2.0f*T() ) - 4.0f*tau;
    robust_pid_data_t sigma2 = (2.0f*T() ) + 4.0f*tau;

    // Compute PID coefficients
    ctx->coeff.c[0] = (8.0f*tau)/sigma2;                               // C1
    ctx->coeff.c[1] = sigma1/sigma2;                                   // C2
    ctx->coeff.c[2] = ( POW2( T() ) + 2.0f*tau*T() )/sigma2;           // C3
    ctx->coeff.c[3] = (2.0f* POW2( T() ) )/sigma2;                     // C4
    ctx->coeff.c[4] = ( POW2( T() ) - 2.0f*tau*T() )/sigma2;           // C5
    ctx->coeff.c[5] = 4.0f/sigma2;                                     // C6
    ctx->coeff.c[6] = 8.0f/sigma2;                                     // C7
    ctx->coeff.c[7] = 4.0f/sigma2;                                     // C8
    ctx->coeff.c[8] = (8.0f*tau)/sigma2;                               // C9
    ctx->coeff.c[9] = sigma1/sigma2;                                   // C10

    // Compute PD coefficients
    ctx->coeff.d[0] = ( T() - 2.0f*tau)/( T() + 2.0f*tau );            // D1
    ctx->coeff.d[1] = 2.0f/( T()+2.0f*tau);                            // D2
    ctx->coeff.d[2] = (2.0f*tau - T() )/( 2.0f*tau + T() );            // D3

} // compute_iir_coeff

