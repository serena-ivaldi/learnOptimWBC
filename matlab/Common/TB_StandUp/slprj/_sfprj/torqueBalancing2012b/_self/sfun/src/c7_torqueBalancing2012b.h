#ifndef __c7_torqueBalancing2012b_h__
#define __c7_torqueBalancing2012b_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_struct_1ZGMVR6bgCMpDdXTSGnu6G_tag
#define struct_struct_1ZGMVR6bgCMpDdXTSGnu6G_tag

struct struct_1ZGMVR6bgCMpDdXTSGnu6G_tag
{
  real_T pinvTol;
  real_T pinvDamp;
  real_T pinvDampVb;
  real_T HessianQP;
  real_T impedances;
  real_T dampings;
  real_T norm_tolerance;
};

#endif                                 /*struct_struct_1ZGMVR6bgCMpDdXTSGnu6G_tag*/

#ifndef typedef_c7_struct_1ZGMVR6bgCMpDdXTSGnu6G
#define typedef_c7_struct_1ZGMVR6bgCMpDdXTSGnu6G

typedef struct struct_1ZGMVR6bgCMpDdXTSGnu6G_tag
  c7_struct_1ZGMVR6bgCMpDdXTSGnu6G;

#endif                                 /*typedef_c7_struct_1ZGMVR6bgCMpDdXTSGnu6G*/

#ifndef typedef_c7_ResolvedFunctionInfo
#define typedef_c7_ResolvedFunctionInfo

typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c7_ResolvedFunctionInfo;

#endif                                 /*typedef_c7_ResolvedFunctionInfo*/

#ifndef typedef_SFc7_torqueBalancing2012bInstanceStruct
#define typedef_SFc7_torqueBalancing2012bInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c7_sfEvent;
  boolean_T c7_isStable;
  boolean_T c7_doneDoubleBufferReInit;
  uint8_T c7_is_active_c7_torqueBalancing2012b;
  c7_struct_1ZGMVR6bgCMpDdXTSGnu6G c7_reg;
} SFc7_torqueBalancing2012bInstanceStruct;

#endif                                 /*typedef_SFc7_torqueBalancing2012bInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c7_torqueBalancing2012b_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c7_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
extern void c7_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
