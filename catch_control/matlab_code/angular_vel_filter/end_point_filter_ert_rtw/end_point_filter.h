//
// File: end_point_filter.h
//
// Code generated for Simulink model 'end_point_filter'.
//
// Model version                  : 1.142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Sun Jun  9 11:45:23 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_end_point_filter_h_
#define RTW_HEADER_end_point_filter_h_
#include <stddef.h>
#include <string.h>
#ifndef end_point_filter_COMMON_INCLUDES_
# define end_point_filter_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // end_point_filter_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM_end_point_filter_T RT_MODEL_end_point_filter_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T PrevY;                        // '<S1>/Rate Limiter'
  real_T PrevY_p;                      // '<S1>/Rate Limiter1'
  real_T PrevY_o;                      // '<S1>/Rate Limiter2'
  int32_T MedianFilter_bSEnd[2];       // '<S1>/Median Filter'
  int32_T MedianFilter_bSPreEdg[2];    // '<S1>/Median Filter'
  int32_T MedianFilter_bSPstEdg[2];    // '<S1>/Median Filter'
  int32_T MedianFilter_bSStart[2];     // '<S1>/Median Filter'
  int32_T MedianFilter_inSEnd[2];      // '<S1>/Median Filter'
  int32_T MedianFilter_inSStart[2];    // '<S1>/Median Filter'
  int32_T MedianFilter_mLoc[10];       // '<S1>/Median Filter'
  int32_T MedianFilter_mWidth[10];     // '<S1>/Median Filter'
  int32_T MedianFilter_oSPreEdg[2];    // '<S1>/Median Filter'
  int32_T MedianFilter_oSPstEdg[2];    // '<S1>/Median Filter'
  int32_T MedianFilter_oSStart[2];     // '<S1>/Median Filter'
  int32_T MedianFilter_oSecEnd[2];     // '<S1>/Median Filter'
  int32_T MedianFilter_sCnt[2];        // '<S1>/Median Filter'
  int32_T MedianFilter1_bSEnd[2];      // '<S1>/Median Filter1'
  int32_T MedianFilter1_bSPreEdg[2];   // '<S1>/Median Filter1'
  int32_T MedianFilter1_bSPstEdg[2];   // '<S1>/Median Filter1'
  int32_T MedianFilter1_bSStart[2];    // '<S1>/Median Filter1'
  int32_T MedianFilter1_inSEnd[2];     // '<S1>/Median Filter1'
  int32_T MedianFilter1_inSStart[2];   // '<S1>/Median Filter1'
  int32_T MedianFilter1_mLoc[10];      // '<S1>/Median Filter1'
  int32_T MedianFilter1_mWidth[10];    // '<S1>/Median Filter1'
  int32_T MedianFilter1_oSPreEdg[2];   // '<S1>/Median Filter1'
  int32_T MedianFilter1_oSPstEdg[2];   // '<S1>/Median Filter1'
  int32_T MedianFilter1_oSStart[2];    // '<S1>/Median Filter1'
  int32_T MedianFilter1_oSecEnd[2];    // '<S1>/Median Filter1'
  int32_T MedianFilter1_sCnt[2];       // '<S1>/Median Filter1'
  int32_T MedianFilter2_bSEnd[2];      // '<S1>/Median Filter2'
  int32_T MedianFilter2_bSPreEdg[2];   // '<S1>/Median Filter2'
  int32_T MedianFilter2_bSPstEdg[2];   // '<S1>/Median Filter2'
  int32_T MedianFilter2_bSStart[2];    // '<S1>/Median Filter2'
  int32_T MedianFilter2_inSEnd[2];     // '<S1>/Median Filter2'
  int32_T MedianFilter2_inSStart[2];   // '<S1>/Median Filter2'
  int32_T MedianFilter2_mLoc[10];      // '<S1>/Median Filter2'
  int32_T MedianFilter2_mWidth[10];    // '<S1>/Median Filter2'
  int32_T MedianFilter2_oSPreEdg[2];   // '<S1>/Median Filter2'
  int32_T MedianFilter2_oSPstEdg[2];   // '<S1>/Median Filter2'
  int32_T MedianFilter2_oSStart[2];    // '<S1>/Median Filter2'
  int32_T MedianFilter2_oSecEnd[2];    // '<S1>/Median Filter2'
  int32_T MedianFilter2_sCnt[2];       // '<S1>/Median Filter2'
  boolean_T MedianFilter_isHgtUpd[2];  // '<S1>/Median Filter'
  boolean_T MedianFilter1_isHgtUpd[2]; // '<S1>/Median Filter1'
  boolean_T MedianFilter2_isHgtUpd[2]; // '<S1>/Median Filter2'
} DW_end_point_filter_T;

// Real-time Model Data Structure
struct tag_RTM_end_point_filter_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model end_point_filter
class end_poing_filter {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step(real_T (&arg_end_point)[3], real_T (&arg_end_point_filtered)[3]);

  // model terminate function
  void terminate();

  // Constructor
  end_poing_filter();

  // Destructor
  ~end_poing_filter();

  // Real-Time Model get method
  RT_MODEL_end_point_filter_T * getRTM();

  // private data and function members
 private:
  // Block states
  DW_end_point_filter_T end_point_filter_DW;

  // Real-Time Model
  RT_MODEL_end_point_filter_T end_point_filter_M;
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
//  '<Root>' : 'end_point_filter'
//  '<S1>'   : 'end_point_filter/LTI filter '

#endif                                 // RTW_HEADER_end_point_filter_h_

//
// File trailer for generated code.
//
// [EOF]
//
