#ifndef __c3_torqueBalancing2012b_h__
#define __c3_torqueBalancing2012b_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_struct_KJR2itYvhBuAkuR6dKZHUC_tag
#define struct_struct_KJR2itYvhBuAkuR6dKZHUC_tag

struct struct_KJR2itYvhBuAkuR6dKZHUC_tag
{
  real_T thresholdContactOn;
  real_T thresholdContactOff;
};

#endif                                 /*struct_struct_KJR2itYvhBuAkuR6dKZHUC_tag*/

#ifndef typedef_c3_struct_KJR2itYvhBuAkuR6dKZHUC
#define typedef_c3_struct_KJR2itYvhBuAkuR6dKZHUC

typedef struct struct_KJR2itYvhBuAkuR6dKZHUC_tag
  c3_struct_KJR2itYvhBuAkuR6dKZHUC;

#endif                                 /*typedef_c3_struct_KJR2itYvhBuAkuR6dKZHUC*/

#ifndef struct_struct_DnBdbfPNxiIjhNOyZMmfsE_tag
#define struct_struct_DnBdbfPNxiIjhNOyZMmfsE_tag

struct struct_DnBdbfPNxiIjhNOyZMmfsE_tag
{
  real_T threshold;
  real_T states[39];
};

#endif                                 /*struct_struct_DnBdbfPNxiIjhNOyZMmfsE_tag*/

#ifndef typedef_c3_struct_DnBdbfPNxiIjhNOyZMmfsE
#define typedef_c3_struct_DnBdbfPNxiIjhNOyZMmfsE

typedef struct struct_DnBdbfPNxiIjhNOyZMmfsE_tag
  c3_struct_DnBdbfPNxiIjhNOyZMmfsE;

#endif                                 /*typedef_c3_struct_DnBdbfPNxiIjhNOyZMmfsE*/

#ifndef struct_struct_0U0wBk2LiR1OqsMsUngxdD_tag
#define struct_struct_0U0wBk2LiR1OqsMsUngxdD_tag

struct struct_0U0wBk2LiR1OqsMsUngxdD_tag
{
  real_T thresholdNotInContact;
  real_T thresholdInContact;
  real_T pauseTimeLastPostureL;
  real_T pauseTimeLastPostureR;
  real_T states[299];
  real_T pointsL[192];
  real_T pointsR[192];
  real_T standUpPositions[72];
};

#endif                                 /*struct_struct_0U0wBk2LiR1OqsMsUngxdD_tag*/

#ifndef typedef_c3_struct_0U0wBk2LiR1OqsMsUngxdD
#define typedef_c3_struct_0U0wBk2LiR1OqsMsUngxdD

typedef struct struct_0U0wBk2LiR1OqsMsUngxdD_tag
  c3_struct_0U0wBk2LiR1OqsMsUngxdD;

#endif                                 /*typedef_c3_struct_0U0wBk2LiR1OqsMsUngxdD*/

#ifndef struct_struct_9LpOi5JXaV67jTuay8hWaH_tag
#define struct_struct_9LpOi5JXaV67jTuay8hWaH_tag

struct struct_9LpOi5JXaV67jTuay8hWaH_tag
{
  real_T standUpDeltaCoM[24];
};

#endif                                 /*struct_struct_9LpOi5JXaV67jTuay8hWaH_tag*/

#ifndef typedef_c3_struct_9LpOi5JXaV67jTuay8hWaH
#define typedef_c3_struct_9LpOi5JXaV67jTuay8hWaH

typedef struct struct_9LpOi5JXaV67jTuay8hWaH_tag
  c3_struct_9LpOi5JXaV67jTuay8hWaH;

#endif                                 /*typedef_c3_struct_9LpOi5JXaV67jTuay8hWaH*/

#ifndef struct_struct_rUGQ0INmvPpaxIctEGl5sE_tag
#define struct_struct_rUGQ0INmvPpaxIctEGl5sE_tag

struct struct_rUGQ0INmvPpaxIctEGl5sE_tag
{
  boolean_T skipYoga;
  boolean_T demoOnlyRightFoot;
  boolean_T yogaAlsoOnRightFoot;
  boolean_T yogaInLoop;
  c3_struct_DnBdbfPNxiIjhNOyZMmfsE com;
  c3_struct_KJR2itYvhBuAkuR6dKZHUC wrench;
  c3_struct_0U0wBk2LiR1OqsMsUngxdD joints;
  real_T stateAt0;
  real_T DT;
  real_T waitingTimeAfterYoga;
  real_T jointsSmoothingTimes[13];
  real_T tBalancing;
  boolean_T alsoSitDown;
  real_T jointsAndCoMSmoothingTimes[8];
  c3_struct_9LpOi5JXaV67jTuay8hWaH CoM;
  real_T LwrenchThreshold[8];
  real_T RwrenchThreshold[8];
  real_T RArmThreshold[8];
  real_T LArmThreshold[8];
};

#endif                                 /*struct_struct_rUGQ0INmvPpaxIctEGl5sE_tag*/

#ifndef typedef_c3_struct_rUGQ0INmvPpaxIctEGl5sE
#define typedef_c3_struct_rUGQ0INmvPpaxIctEGl5sE

typedef struct struct_rUGQ0INmvPpaxIctEGl5sE_tag
  c3_struct_rUGQ0INmvPpaxIctEGl5sE;

#endif                                 /*typedef_c3_struct_rUGQ0INmvPpaxIctEGl5sE*/

#ifndef typedef_SFc3_torqueBalancing2012bInstanceStruct
#define typedef_SFc3_torqueBalancing2012bInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_torqueBalancing2012b;
  c3_struct_rUGQ0INmvPpaxIctEGl5sE c3_sm;
} SFc3_torqueBalancing2012bInstanceStruct;

#endif                                 /*typedef_SFc3_torqueBalancing2012bInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_torqueBalancing2012b_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c3_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
extern void c3_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
