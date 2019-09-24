//
// File: force_interaction.cpp
//
// Code generated for Simulink model 'force_interaction'.
//
// Model version                  : 1.192
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Sun Jun  9 15:50:41 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "force_interaction.h"

// Model step function
void ForceInteraction::step(real_T (&arg_force_base_)[6], real_T (&arg_delta_x)
  [3], real_T (&arg_delta_rpy)[3])
{
  // local block i/o variables
  real_T rtb_Sum[3];
  real_T rtb_Sum_h[3];
  real_T rtb_RateLimiter3;
  real_T rtb_RateLimiter2;
  real_T rtb_RateLimiter1;
  real_T rateLimiterRate;
  real_T rtb_FilterCoefficient;
  real_T rtb_FilterCoefficient_n;
  real_T rtb_Saturation_idx_0;
  real_T rtb_FilterCoefficient_idx_0;
  real_T rtb_Saturation_idx_1;
  real_T rtb_FilterCoefficient_idx_1;
  real_T rtb_Saturation1_idx_0;
  real_T rtb_FilterCoefficient_n_idx_0;
  real_T rtb_Saturation1_idx_1;
  real_T rtb_FilterCoefficient_n_idx_1;
  real_T u0;
  real_T u0_0;

  // Switch: '<S1>/Switch' incorporates:
  //   Constant: '<Root>/Constant'
  //   Constant: '<Root>/Constant1'
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S1>/u_GTE_up'
  //   RelationalOperator: '<S1>/u_GT_lo'
  //   Switch: '<S1>/Switch1'

  if (arg_force_base_[0] >= 5.0) {
    rateLimiterRate = 5.0;
  } else if (arg_force_base_[0] > -5.0) {
    // Switch: '<S1>/Switch1'
    rateLimiterRate = arg_force_base_[0];
  } else {
    rateLimiterRate = -5.0;
  }

  // Sum: '<S1>/Diff' incorporates:
  //   Inport: '<Root>/force_base_'

  u0 = arg_force_base_[0] - rateLimiterRate;

  // Saturate: '<Root>/Saturation'
  if (u0 > 40.0) {
    u0 = 40.0;
  } else {
    if (u0 < -40.0) {
      u0 = -40.0;
    }
  }

  // Gain: '<S3>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S3>/Filter'
  //   Gain: '<S3>/Derivative Gain'
  //   Sum: '<S3>/SumD'

  rtb_FilterCoefficient = (0.0 * u0 - force_interaction_DW.Filter_DSTATE[0]) *
    5.0;

  // Sum: '<S3>/Sum' incorporates:
  //   DiscreteIntegrator: '<S3>/Integrator'
  //   Gain: '<S3>/Proportional Gain'

  rtb_Sum[0] = (100.0 * u0 + force_interaction_DW.Integrator_DSTATE[0]) +
    rtb_FilterCoefficient;

  // Saturate: '<Root>/Saturation'
  rtb_Saturation_idx_0 = u0;

  // Gain: '<S3>/Filter Coefficient'
  rtb_FilterCoefficient_idx_0 = rtb_FilterCoefficient;

  // Switch: '<S1>/Switch' incorporates:
  //   Constant: '<Root>/Constant'
  //   Constant: '<Root>/Constant1'
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S1>/u_GTE_up'
  //   RelationalOperator: '<S1>/u_GT_lo'
  //   Switch: '<S1>/Switch1'

  if (arg_force_base_[1] >= 5.0) {
    rateLimiterRate = 5.0;
  } else if (arg_force_base_[1] > -5.0) {
    // Switch: '<S1>/Switch1'
    rateLimiterRate = arg_force_base_[1];
  } else {
    rateLimiterRate = -5.0;
  }

  // Sum: '<S1>/Diff' incorporates:
  //   Inport: '<Root>/force_base_'

  u0 = arg_force_base_[1] - rateLimiterRate;

  // Saturate: '<Root>/Saturation'
  if (u0 > 40.0) {
    u0 = 40.0;
  } else {
    if (u0 < -40.0) {
      u0 = -40.0;
    }
  }

  // Gain: '<S3>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S3>/Filter'
  //   Gain: '<S3>/Derivative Gain'
  //   Sum: '<S3>/SumD'

  rtb_FilterCoefficient = (0.0 * u0 - force_interaction_DW.Filter_DSTATE[1]) *
    5.0;

  // Sum: '<S3>/Sum' incorporates:
  //   DiscreteIntegrator: '<S3>/Integrator'
  //   Gain: '<S3>/Proportional Gain'

  rtb_Sum[1] = (100.0 * u0 + force_interaction_DW.Integrator_DSTATE[1]) +
    rtb_FilterCoefficient;

  // Saturate: '<Root>/Saturation'
  rtb_Saturation_idx_1 = u0;

  // Gain: '<S3>/Filter Coefficient'
  rtb_FilterCoefficient_idx_1 = rtb_FilterCoefficient;

  // Switch: '<S1>/Switch' incorporates:
  //   Constant: '<Root>/Constant'
  //   Constant: '<Root>/Constant1'
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S1>/u_GTE_up'
  //   RelationalOperator: '<S1>/u_GT_lo'
  //   Switch: '<S1>/Switch1'

  if (arg_force_base_[2] >= 5.0) {
    rateLimiterRate = 5.0;
  } else if (arg_force_base_[2] > -5.0) {
    // Switch: '<S1>/Switch1'
    rateLimiterRate = arg_force_base_[2];
  } else {
    rateLimiterRate = -5.0;
  }

  // Sum: '<S1>/Diff' incorporates:
  //   Inport: '<Root>/force_base_'

  u0 = arg_force_base_[2] - rateLimiterRate;

  // Saturate: '<Root>/Saturation'
  if (u0 > 40.0) {
    u0 = 40.0;
  } else {
    if (u0 < -40.0) {
      u0 = -40.0;
    }
  }

  // Gain: '<S3>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S3>/Filter'
  //   Gain: '<S3>/Derivative Gain'
  //   Sum: '<S3>/SumD'

  rtb_FilterCoefficient = (0.0 * u0 - force_interaction_DW.Filter_DSTATE[2]) *
    5.0;

  // Sum: '<S3>/Sum' incorporates:
  //   DiscreteIntegrator: '<S3>/Integrator'
  //   Gain: '<S3>/Proportional Gain'

  rtb_Sum[2] = (100.0 * u0 + force_interaction_DW.Integrator_DSTATE[2]) +
    rtb_FilterCoefficient;

  // DiscreteStateSpace: '<S7>/Internal'
  {
    rtb_RateLimiter3 = 0.0600918954997801*force_interaction_DW.Internal_DSTATE;
    rtb_RateLimiter3 += 0.0018814114433874374*rtb_Sum[0];
  }

  // RateLimiter: '<S5>/Rate Limiter'
  rateLimiterRate = rtb_RateLimiter3 - force_interaction_DW.PrevY;
  if (rateLimiterRate > 0.008) {
    rtb_RateLimiter3 = force_interaction_DW.PrevY + 0.008;
  } else {
    if (rateLimiterRate < -0.008) {
      rtb_RateLimiter3 = force_interaction_DW.PrevY + -0.008;
    }
  }

  force_interaction_DW.PrevY = rtb_RateLimiter3;

  // End of RateLimiter: '<S5>/Rate Limiter'

  // DiscreteStateSpace: '<S8>/Internal'
  {
    rtb_RateLimiter2 = 0.0600918954997801*force_interaction_DW.Internal_DSTATE_m;
    rtb_RateLimiter2 += 0.0018814114433874374*rtb_Sum[1];
  }

  // RateLimiter: '<S5>/Rate Limiter3'
  rateLimiterRate = rtb_RateLimiter2 - force_interaction_DW.PrevY_m;
  if (rateLimiterRate > 0.008) {
    rtb_RateLimiter2 = force_interaction_DW.PrevY_m + 0.008;
  } else {
    if (rateLimiterRate < -0.008) {
      rtb_RateLimiter2 = force_interaction_DW.PrevY_m + -0.008;
    }
  }

  force_interaction_DW.PrevY_m = rtb_RateLimiter2;

  // End of RateLimiter: '<S5>/Rate Limiter3'

  // DiscreteStateSpace: '<S9>/Internal'
  {
    rtb_RateLimiter1 = 0.0600918954997801*
      force_interaction_DW.Internal_DSTATE_mt;
    rtb_RateLimiter1 += 0.0018814114433874374*rtb_Sum[2];
  }

  // RateLimiter: '<S5>/Rate Limiter1'
  rateLimiterRate = rtb_RateLimiter1 - force_interaction_DW.PrevY_b;
  if (rateLimiterRate > 0.008) {
    rtb_RateLimiter1 = force_interaction_DW.PrevY_b + 0.008;
  } else {
    if (rateLimiterRate < -0.008) {
      rtb_RateLimiter1 = force_interaction_DW.PrevY_b + -0.008;
    }
  }

  force_interaction_DW.PrevY_b = rtb_RateLimiter1;

  // End of RateLimiter: '<S5>/Rate Limiter1'

  // Outport: '<Root>/delta_x'
  arg_delta_x[0] = rtb_RateLimiter3;
  arg_delta_x[1] = rtb_RateLimiter2;
  arg_delta_x[2] = rtb_RateLimiter1;

  // Switch: '<S2>/Switch' incorporates:
  //   Constant: '<Root>/Constant2'
  //   Constant: '<Root>/Constant3'
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S2>/u_GTE_up'
  //   RelationalOperator: '<S2>/u_GT_lo'
  //   Saturate: '<Root>/Saturation1'
  //   Sum: '<S2>/Diff'
  //   Switch: '<S2>/Switch1'

  if (arg_force_base_[3] >= 0.3) {
    rateLimiterRate = 0.3;
  } else if (arg_force_base_[3] > -0.3) {
    rateLimiterRate = arg_force_base_[3];
  } else {
    rateLimiterRate = -0.3;
  }

  // Sum: '<S2>/Diff' incorporates:
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S2>/u_GTE_up'
  //   RelationalOperator: '<S2>/u_GT_lo'
  //   Saturate: '<Root>/Saturation1'
  //   Switch: '<S2>/Switch1'

  u0_0 = arg_force_base_[3] - rateLimiterRate;

  // Saturate: '<Root>/Saturation1'
  if (u0_0 > 3.0) {
    u0_0 = 3.0;
  } else {
    if (u0_0 < -3.0) {
      u0_0 = -3.0;
    }
  }

  // Gain: '<S4>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S4>/Filter'
  //   Gain: '<S4>/Derivative Gain'
  //   Sum: '<S4>/SumD'

  rtb_FilterCoefficient_n = (0.0 * u0_0 - force_interaction_DW.Filter_DSTATE_b[0])
    * 5.0;

  // Sum: '<S4>/Sum' incorporates:
  //   DiscreteIntegrator: '<S4>/Integrator'
  //   Gain: '<S4>/Proportional Gain'

  rtb_Sum_h[0] = (0.0 * u0_0 + force_interaction_DW.Integrator_DSTATE_h[0]) +
    rtb_FilterCoefficient_n;

  // Saturate: '<Root>/Saturation1'
  rtb_Saturation1_idx_0 = u0_0;

  // Gain: '<S4>/Filter Coefficient'
  rtb_FilterCoefficient_n_idx_0 = rtb_FilterCoefficient_n;

  // Switch: '<S2>/Switch' incorporates:
  //   Constant: '<Root>/Constant2'
  //   Constant: '<Root>/Constant3'
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S2>/u_GTE_up'
  //   RelationalOperator: '<S2>/u_GT_lo'
  //   Saturate: '<Root>/Saturation1'
  //   Sum: '<S2>/Diff'
  //   Switch: '<S2>/Switch1'

  if (arg_force_base_[4] >= 0.3) {
    rateLimiterRate = 0.3;
  } else if (arg_force_base_[4] > -0.3) {
    rateLimiterRate = arg_force_base_[4];
  } else {
    rateLimiterRate = -0.3;
  }

  // Sum: '<S2>/Diff' incorporates:
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S2>/u_GTE_up'
  //   RelationalOperator: '<S2>/u_GT_lo'
  //   Saturate: '<Root>/Saturation1'
  //   Switch: '<S2>/Switch1'

  u0_0 = arg_force_base_[4] - rateLimiterRate;

  // Saturate: '<Root>/Saturation1'
  if (u0_0 > 3.0) {
    u0_0 = 3.0;
  } else {
    if (u0_0 < -3.0) {
      u0_0 = -3.0;
    }
  }

  // Gain: '<S4>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S4>/Filter'
  //   Gain: '<S4>/Derivative Gain'
  //   Sum: '<S4>/SumD'

  rtb_FilterCoefficient_n = (0.0 * u0_0 - force_interaction_DW.Filter_DSTATE_b[1])
    * 5.0;

  // Sum: '<S4>/Sum' incorporates:
  //   DiscreteIntegrator: '<S4>/Integrator'
  //   Gain: '<S4>/Proportional Gain'

  rtb_Sum_h[1] = (0.0 * u0_0 + force_interaction_DW.Integrator_DSTATE_h[1]) +
    rtb_FilterCoefficient_n;

  // Saturate: '<Root>/Saturation1'
  rtb_Saturation1_idx_1 = u0_0;

  // Gain: '<S4>/Filter Coefficient'
  rtb_FilterCoefficient_n_idx_1 = rtb_FilterCoefficient_n;

  // Switch: '<S2>/Switch' incorporates:
  //   Constant: '<Root>/Constant2'
  //   Constant: '<Root>/Constant3'
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S2>/u_GTE_up'
  //   RelationalOperator: '<S2>/u_GT_lo'
  //   Saturate: '<Root>/Saturation1'
  //   Sum: '<S2>/Diff'
  //   Switch: '<S2>/Switch1'

  if (arg_force_base_[5] >= 0.3) {
    rateLimiterRate = 0.3;
  } else if (arg_force_base_[5] > -0.3) {
    rateLimiterRate = arg_force_base_[5];
  } else {
    rateLimiterRate = -0.3;
  }

  // Sum: '<S2>/Diff' incorporates:
  //   Inport: '<Root>/force_base_'
  //   RelationalOperator: '<S2>/u_GTE_up'
  //   RelationalOperator: '<S2>/u_GT_lo'
  //   Saturate: '<Root>/Saturation1'
  //   Switch: '<S2>/Switch1'

  u0_0 = arg_force_base_[5] - rateLimiterRate;

  // Saturate: '<Root>/Saturation1'
  if (u0_0 > 3.0) {
    u0_0 = 3.0;
  } else {
    if (u0_0 < -3.0) {
      u0_0 = -3.0;
    }
  }

  // Gain: '<S4>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S4>/Filter'
  //   Gain: '<S4>/Derivative Gain'
  //   Sum: '<S4>/SumD'

  rtb_FilterCoefficient_n = (0.0 * u0_0 - force_interaction_DW.Filter_DSTATE_b[2])
    * 5.0;

  // Sum: '<S4>/Sum' incorporates:
  //   DiscreteIntegrator: '<S4>/Integrator'
  //   Gain: '<S4>/Proportional Gain'

  rtb_Sum_h[2] = (0.0 * u0_0 + force_interaction_DW.Integrator_DSTATE_h[2]) +
    rtb_FilterCoefficient_n;

  // DiscreteStateSpace: '<S19>/Internal'
  {
    rtb_RateLimiter1 = 0.0600918954997801*force_interaction_DW.Internal_DSTATE_a;
    rtb_RateLimiter1 += 0.0018814114433874374*rtb_Sum_h[0];
  }

  // RateLimiter: '<S6>/Rate Limiter1'
  rateLimiterRate = rtb_RateLimiter1 - force_interaction_DW.PrevY_me;
  if (rateLimiterRate > 0.008) {
    rtb_RateLimiter1 = force_interaction_DW.PrevY_me + 0.008;
  } else {
    if (rateLimiterRate < -0.008) {
      rtb_RateLimiter1 = force_interaction_DW.PrevY_me + -0.008;
    }
  }

  force_interaction_DW.PrevY_me = rtb_RateLimiter1;

  // End of RateLimiter: '<S6>/Rate Limiter1'

  // DiscreteStateSpace: '<S20>/Internal'
  {
    rtb_RateLimiter2 = 0.0600918954997801*force_interaction_DW.Internal_DSTATE_p;
    rtb_RateLimiter2 += 0.0018814114433874374*rtb_Sum_h[1];
  }

  // RateLimiter: '<S6>/Rate Limiter2'
  rateLimiterRate = rtb_RateLimiter2 - force_interaction_DW.PrevY_m0;
  if (rateLimiterRate > 0.008) {
    rtb_RateLimiter2 = force_interaction_DW.PrevY_m0 + 0.008;
  } else {
    if (rateLimiterRate < -0.008) {
      rtb_RateLimiter2 = force_interaction_DW.PrevY_m0 + -0.008;
    }
  }

  force_interaction_DW.PrevY_m0 = rtb_RateLimiter2;

  // End of RateLimiter: '<S6>/Rate Limiter2'

  // DiscreteStateSpace: '<S21>/Internal'
  {
    rtb_RateLimiter3 = 0.0600918954997801*
      force_interaction_DW.Internal_DSTATE_mg;
    rtb_RateLimiter3 += 0.0018814114433874374*rtb_Sum_h[2];
  }

  // RateLimiter: '<S6>/Rate Limiter3'
  rateLimiterRate = rtb_RateLimiter3 - force_interaction_DW.PrevY_c;
  if (rateLimiterRate > 0.008) {
    rtb_RateLimiter3 = force_interaction_DW.PrevY_c + 0.008;
  } else {
    if (rateLimiterRate < -0.008) {
      rtb_RateLimiter3 = force_interaction_DW.PrevY_c + -0.008;
    }
  }

  force_interaction_DW.PrevY_c = rtb_RateLimiter3;

  // End of RateLimiter: '<S6>/Rate Limiter3'

  // Outport: '<Root>/delta_rpy'
  arg_delta_rpy[0] = rtb_RateLimiter1;
  arg_delta_rpy[1] = rtb_RateLimiter2;
  arg_delta_rpy[2] = rtb_RateLimiter3;

  // Update for DiscreteIntegrator: '<S3>/Integrator' incorporates:
  //   Gain: '<S3>/Integral Gain'

  force_interaction_DW.Integrator_DSTATE[0] += 0.0 * rtb_Saturation_idx_0 *
    0.002;

  // Update for DiscreteIntegrator: '<S3>/Filter'
  force_interaction_DW.Filter_DSTATE[0] += 0.002 * rtb_FilterCoefficient_idx_0;

  // Update for DiscreteIntegrator: '<S3>/Integrator' incorporates:
  //   Gain: '<S3>/Integral Gain'

  force_interaction_DW.Integrator_DSTATE[1] += 0.0 * rtb_Saturation_idx_1 *
    0.002;

  // Update for DiscreteIntegrator: '<S3>/Filter'
  force_interaction_DW.Filter_DSTATE[1] += 0.002 * rtb_FilterCoefficient_idx_1;

  // Update for DiscreteIntegrator: '<S3>/Integrator' incorporates:
  //   Gain: '<S3>/Integral Gain'

  force_interaction_DW.Integrator_DSTATE[2] += 0.0 * u0 * 0.002;

  // Update for DiscreteIntegrator: '<S3>/Filter'
  force_interaction_DW.Filter_DSTATE[2] += 0.002 * rtb_FilterCoefficient;

  // Update for DiscreteStateSpace: '<S7>/Internal'
  {
    real_T xnew[1];
    xnew[0] = 0.99623717711322513*force_interaction_DW.Internal_DSTATE;
    xnew[0] += 0.0625*rtb_Sum[0];
    (void) memcpy(&force_interaction_DW.Internal_DSTATE, xnew,
                  sizeof(real_T)*1);
  }

  // Update for DiscreteStateSpace: '<S8>/Internal'
  {
    real_T xnew[1];
    xnew[0] = 0.99623717711322513*force_interaction_DW.Internal_DSTATE_m;
    xnew[0] += 0.0625*rtb_Sum[1];
    (void) memcpy(&force_interaction_DW.Internal_DSTATE_m, xnew,
                  sizeof(real_T)*1);
  }

  // Update for DiscreteStateSpace: '<S9>/Internal'
  {
    real_T xnew[1];
    xnew[0] = 0.99623717711322513*force_interaction_DW.Internal_DSTATE_mt;
    xnew[0] += 0.0625*rtb_Sum[2];
    (void) memcpy(&force_interaction_DW.Internal_DSTATE_mt, xnew,
                  sizeof(real_T)*1);
  }

  // Update for DiscreteIntegrator: '<S4>/Integrator' incorporates:
  //   Gain: '<S4>/Integral Gain'

  force_interaction_DW.Integrator_DSTATE_h[0] += 0.0 * rtb_Saturation1_idx_0 *
    0.002;

  // Update for DiscreteIntegrator: '<S4>/Filter'
  force_interaction_DW.Filter_DSTATE_b[0] += 0.002 *
    rtb_FilterCoefficient_n_idx_0;

  // Update for DiscreteIntegrator: '<S4>/Integrator' incorporates:
  //   Gain: '<S4>/Integral Gain'

  force_interaction_DW.Integrator_DSTATE_h[1] += 0.0 * rtb_Saturation1_idx_1 *
    0.002;

  // Update for DiscreteIntegrator: '<S4>/Filter'
  force_interaction_DW.Filter_DSTATE_b[1] += 0.002 *
    rtb_FilterCoefficient_n_idx_1;

  // Update for DiscreteIntegrator: '<S4>/Integrator' incorporates:
  //   Gain: '<S4>/Integral Gain'

  force_interaction_DW.Integrator_DSTATE_h[2] += 0.0 * u0_0 * 0.002;

  // Update for DiscreteIntegrator: '<S4>/Filter'
  force_interaction_DW.Filter_DSTATE_b[2] += 0.002 * rtb_FilterCoefficient_n;

  // Update for DiscreteStateSpace: '<S19>/Internal'
  {
    real_T xnew[1];
    xnew[0] = 0.99623717711322513*force_interaction_DW.Internal_DSTATE_a;
    xnew[0] += 0.0625*rtb_Sum_h[0];
    (void) memcpy(&force_interaction_DW.Internal_DSTATE_a, xnew,
                  sizeof(real_T)*1);
  }

  // Update for DiscreteStateSpace: '<S20>/Internal'
  {
    real_T xnew[1];
    xnew[0] = 0.99623717711322513*force_interaction_DW.Internal_DSTATE_p;
    xnew[0] += 0.0625*rtb_Sum_h[1];
    (void) memcpy(&force_interaction_DW.Internal_DSTATE_p, xnew,
                  sizeof(real_T)*1);
  }

  // Update for DiscreteStateSpace: '<S21>/Internal'
  {
    real_T xnew[1];
    xnew[0] = 0.99623717711322513*force_interaction_DW.Internal_DSTATE_mg;
    xnew[0] += 0.0625*rtb_Sum_h[2];
    (void) memcpy(&force_interaction_DW.Internal_DSTATE_mg, xnew,
                  sizeof(real_T)*1);
  }
}

// Model initialize function
void ForceInteraction::initialize()
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus((&force_interaction_M), (NULL));

  // states (dwork)
  (void) memset((void *)&force_interaction_DW, 0,
                sizeof(DW_force_interaction_T));

  // InitializeConditions for RateLimiter: '<S5>/Rate Limiter'
  force_interaction_DW.PrevY = 0.0;

  // InitializeConditions for RateLimiter: '<S5>/Rate Limiter3'
  force_interaction_DW.PrevY_m = 0.0;

  // InitializeConditions for RateLimiter: '<S5>/Rate Limiter1'
  force_interaction_DW.PrevY_b = 0.0;

  // InitializeConditions for RateLimiter: '<S6>/Rate Limiter1'
  force_interaction_DW.PrevY_me = 0.0;

  // InitializeConditions for RateLimiter: '<S6>/Rate Limiter2'
  force_interaction_DW.PrevY_m0 = 0.0;

  // InitializeConditions for RateLimiter: '<S6>/Rate Limiter3'
  force_interaction_DW.PrevY_c = 0.0;
}

// Model terminate function
void ForceInteraction::terminate()
{
  // (no terminate code required)
}

// Constructor
ForceInteraction::ForceInteraction()
{
}

// Destructor
ForceInteraction::~ForceInteraction()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_force_interaction_T * ForceInteraction::getRTM()
{
  return (&force_interaction_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
