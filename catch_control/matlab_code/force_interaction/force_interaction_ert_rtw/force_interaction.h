//
// File: force_interaction.h
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
#ifndef RTW_HEADER_force_interaction_h_
#define RTW_HEADER_force_interaction_h_
#include <string.h>
#include <stddef.h>
#ifndef force_interaction_COMMON_INCLUDES_
# define force_interaction_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // force_interaction_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM_force_interaction_T RT_MODEL_force_interaction_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T Integrator_DSTATE[3];         // '<S3>/Integrator'
  real_T Filter_DSTATE[3];             // '<S3>/Filter'
  real_T Internal_DSTATE;              // '<S7>/Internal'
  real_T Internal_DSTATE_m;            // '<S8>/Internal'
  real_T Internal_DSTATE_mt;           // '<S9>/Internal'
  real_T Integrator_DSTATE_h[3];       // '<S4>/Integrator'
  real_T Filter_DSTATE_b[3];           // '<S4>/Filter'
  real_T Internal_DSTATE_a;            // '<S19>/Internal'
  real_T Internal_DSTATE_p;            // '<S20>/Internal'
  real_T Internal_DSTATE_mg;           // '<S21>/Internal'
  real_T PrevY;                        // '<S5>/Rate Limiter'
  real_T PrevY_m;                      // '<S5>/Rate Limiter3'
  real_T PrevY_b;                      // '<S5>/Rate Limiter1'
  real_T PrevY_me;                     // '<S6>/Rate Limiter1'
  real_T PrevY_m0;                     // '<S6>/Rate Limiter2'
  real_T PrevY_c;                      // '<S6>/Rate Limiter3'
} DW_force_interaction_T;

// Real-time Model Data Structure
struct tag_RTM_force_interaction_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model force_interaction
class ForceInteraction {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step(real_T (&arg_force_base_)[6], real_T (&arg_delta_x)[3], real_T
            (&arg_delta_rpy)[3]);

  // model terminate function
  void terminate();

  // Constructor
  ForceInteraction();

  // Destructor
  ~ForceInteraction();

  // Real-Time Model get method
  RT_MODEL_force_interaction_T * getRTM();

  // private data and function members
 private:
  // Block states
  DW_force_interaction_T force_interaction_DW;

  // Real-Time Model
  RT_MODEL_force_interaction_T force_interaction_M;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'force_interaction'
//  '<S1>'   : 'force_interaction/Dead Zone Dynamic'
//  '<S2>'   : 'force_interaction/Dead Zone Dynamic1'
//  '<S3>'   : 'force_interaction/Discrete PID Controller'
//  '<S4>'   : 'force_interaction/Discrete PID Controller1'
//  '<S5>'   : 'force_interaction/filter'
//  '<S6>'   : 'force_interaction/filter2'
//  '<S7>'   : 'force_interaction/filter/LTI System1'
//  '<S8>'   : 'force_interaction/filter/LTI System2'
//  '<S9>'   : 'force_interaction/filter/LTI System3'
//  '<S10>'  : 'force_interaction/filter/LTI System1/IO Delay'
//  '<S11>'  : 'force_interaction/filter/LTI System1/Input Delay'
//  '<S12>'  : 'force_interaction/filter/LTI System1/Output Delay'
//  '<S13>'  : 'force_interaction/filter/LTI System2/IO Delay'
//  '<S14>'  : 'force_interaction/filter/LTI System2/Input Delay'
//  '<S15>'  : 'force_interaction/filter/LTI System2/Output Delay'
//  '<S16>'  : 'force_interaction/filter/LTI System3/IO Delay'
//  '<S17>'  : 'force_interaction/filter/LTI System3/Input Delay'
//  '<S18>'  : 'force_interaction/filter/LTI System3/Output Delay'
//  '<S19>'  : 'force_interaction/filter2/LTI System'
//  '<S20>'  : 'force_interaction/filter2/LTI System1'
//  '<S21>'  : 'force_interaction/filter2/LTI System2'
//  '<S22>'  : 'force_interaction/filter2/LTI System/IO Delay'
//  '<S23>'  : 'force_interaction/filter2/LTI System/Input Delay'
//  '<S24>'  : 'force_interaction/filter2/LTI System/Output Delay'
//  '<S25>'  : 'force_interaction/filter2/LTI System1/IO Delay'
//  '<S26>'  : 'force_interaction/filter2/LTI System1/Input Delay'
//  '<S27>'  : 'force_interaction/filter2/LTI System1/Output Delay'
//  '<S28>'  : 'force_interaction/filter2/LTI System2/IO Delay'
//  '<S29>'  : 'force_interaction/filter2/LTI System2/Input Delay'
//  '<S30>'  : 'force_interaction/filter2/LTI System2/Output Delay'

#endif                                 // RTW_HEADER_force_interaction_h_

//
// File trailer for generated code.
//
// [EOF]
//
