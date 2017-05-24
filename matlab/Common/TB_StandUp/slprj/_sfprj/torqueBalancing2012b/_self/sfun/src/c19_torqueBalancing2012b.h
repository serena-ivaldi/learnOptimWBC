#ifndef __c19_torqueBalancing2012b_h__
#define __c19_torqueBalancing2012b_h__

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

#ifndef typedef_c19_struct_1ZGMVR6bgCMpDdXTSGnu6G
#define typedef_c19_struct_1ZGMVR6bgCMpDdXTSGnu6G

typedef struct struct_1ZGMVR6bgCMpDdXTSGnu6G_tag
  c19_struct_1ZGMVR6bgCMpDdXTSGnu6G;

#endif                                 /*typedef_c19_struct_1ZGMVR6bgCMpDdXTSGnu6G*/

#ifndef typedef_c19_ResolvedFunctionInfo
#define typedef_c19_ResolvedFunctionInfo

typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c19_ResolvedFunctionInfo;

#endif                                 /*typedef_c19_ResolvedFunctionInfo*/

#ifndef typedef_SFc19_torqueBalancing2012bInstanceStruct
#define typedef_SFc19_torqueBalancing2012bInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c19_sfEvent;
  boolean_T c19_isStable;
  boolean_T c19_doneDoubleBufferReInit;
  uint8_T c19_is_active_c19_torqueBalancing2012b;
  c19_struct_1ZGMVR6bgCMpDdXTSGnu6G c19_reg;
} SFc19_torqueBalancing2012bInstanceStruct;

#endif                                 /*typedef_SFc19_torqueBalancing2012bInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c19_torqueBalancing2012b_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c19_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
extern void c19_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
