#ifndef __c5_torqueBalancing2012b_h__
#define __c5_torqueBalancing2012b_h__

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

#ifndef typedef_c5_struct_1ZGMVR6bgCMpDdXTSGnu6G
#define typedef_c5_struct_1ZGMVR6bgCMpDdXTSGnu6G

typedef struct struct_1ZGMVR6bgCMpDdXTSGnu6G_tag
  c5_struct_1ZGMVR6bgCMpDdXTSGnu6G;

#endif                                 /*typedef_c5_struct_1ZGMVR6bgCMpDdXTSGnu6G*/

#ifndef typedef_c5_ResolvedFunctionInfo
#define typedef_c5_ResolvedFunctionInfo

typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c5_ResolvedFunctionInfo;

#endif                                 /*typedef_c5_ResolvedFunctionInfo*/

#ifndef typedef_SFc5_torqueBalancing2012bInstanceStruct
#define typedef_SFc5_torqueBalancing2012bInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c5_sfEvent;
  boolean_T c5_isStable;
  boolean_T c5_doneDoubleBufferReInit;
  uint8_T c5_is_active_c5_torqueBalancing2012b;
  c5_struct_1ZGMVR6bgCMpDdXTSGnu6G c5_reg;
} SFc5_torqueBalancing2012bInstanceStruct;

#endif                                 /*typedef_SFc5_torqueBalancing2012bInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c5_torqueBalancing2012b_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c5_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
extern void c5_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
