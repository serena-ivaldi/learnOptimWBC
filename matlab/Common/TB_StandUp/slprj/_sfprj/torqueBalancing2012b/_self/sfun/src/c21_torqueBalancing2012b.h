#ifndef __c21_torqueBalancing2012b_h__
#define __c21_torqueBalancing2012b_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_SFc21_torqueBalancing2012bInstanceStruct
#define typedef_SFc21_torqueBalancing2012bInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c21_sfEvent;
  boolean_T c21_isStable;
  boolean_T c21_doneDoubleBufferReInit;
  uint8_T c21_is_active_c21_torqueBalancing2012b;
} SFc21_torqueBalancing2012bInstanceStruct;

#endif                                 /*typedef_SFc21_torqueBalancing2012bInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c21_torqueBalancing2012b_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c21_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
extern void c21_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
