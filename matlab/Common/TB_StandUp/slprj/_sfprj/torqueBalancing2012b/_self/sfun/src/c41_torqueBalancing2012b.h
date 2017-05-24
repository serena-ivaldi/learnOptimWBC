#ifndef __c41_torqueBalancing2012b_h__
#define __c41_torqueBalancing2012b_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_SFc41_torqueBalancing2012bInstanceStruct
#define typedef_SFc41_torqueBalancing2012bInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c41_sfEvent;
  boolean_T c41_isStable;
  boolean_T c41_doneDoubleBufferReInit;
  uint8_T c41_is_active_c41_torqueBalancing2012b;
} SFc41_torqueBalancing2012bInstanceStruct;

#endif                                 /*typedef_SFc41_torqueBalancing2012bInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c41_torqueBalancing2012b_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c41_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
extern void c41_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
