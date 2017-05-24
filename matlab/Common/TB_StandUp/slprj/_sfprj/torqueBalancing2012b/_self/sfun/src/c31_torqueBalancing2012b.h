#ifndef __c31_torqueBalancing2012b_h__
#define __c31_torqueBalancing2012b_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_c31_ResolvedFunctionInfo
#define typedef_c31_ResolvedFunctionInfo

typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c31_ResolvedFunctionInfo;

#endif                                 /*typedef_c31_ResolvedFunctionInfo*/

#ifndef typedef_SFc31_torqueBalancing2012bInstanceStruct
#define typedef_SFc31_torqueBalancing2012bInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c31_sfEvent;
  boolean_T c31_isStable;
  boolean_T c31_doneDoubleBufferReInit;
  uint8_T c31_is_active_c31_torqueBalancing2012b;
} SFc31_torqueBalancing2012bInstanceStruct;

#endif                                 /*typedef_SFc31_torqueBalancing2012bInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c31_torqueBalancing2012b_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c31_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
extern void c31_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
