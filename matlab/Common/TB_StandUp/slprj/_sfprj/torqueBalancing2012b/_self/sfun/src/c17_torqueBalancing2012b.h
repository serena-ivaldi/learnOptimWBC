#ifndef __c17_torqueBalancing2012b_h__
#define __c17_torqueBalancing2012b_h__

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

#ifndef typedef_c17_struct_1ZGMVR6bgCMpDdXTSGnu6G
#define typedef_c17_struct_1ZGMVR6bgCMpDdXTSGnu6G

typedef struct struct_1ZGMVR6bgCMpDdXTSGnu6G_tag
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G;

#endif                                 /*typedef_c17_struct_1ZGMVR6bgCMpDdXTSGnu6G*/

#ifndef typedef_c17_ResolvedFunctionInfo
#define typedef_c17_ResolvedFunctionInfo

typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c17_ResolvedFunctionInfo;

#endif                                 /*typedef_c17_ResolvedFunctionInfo*/

#ifndef struct_struct_kzTB0QQWoOlMoMhgKf6sK_tag
#define struct_struct_kzTB0QQWoOlMoMhgKf6sK_tag

struct struct_kzTB0QQWoOlMoMhgKf6sK_tag
{
  real_T qTildeMax;
  real_T SmoothingTimeImp;
  real_T SmoothingTimeGainScheduling;
  real_T PCOM[9];
  real_T ICOM[9];
  real_T DCOM[9];
  real_T PAngularMomentum;
  real_T DAngularMomentum;
  real_T integral[23];
  real_T impedances[23];
  real_T dampings[23];
  real_T increasingRatesImp[23];
  real_T footSize[4];
  real_T legSize[4];
};

#endif                                 /*struct_struct_kzTB0QQWoOlMoMhgKf6sK_tag*/

#ifndef typedef_c17_struct_kzTB0QQWoOlMoMhgKf6sK
#define typedef_c17_struct_kzTB0QQWoOlMoMhgKf6sK

typedef struct struct_kzTB0QQWoOlMoMhgKf6sK_tag c17_struct_kzTB0QQWoOlMoMhgKf6sK;

#endif                                 /*typedef_c17_struct_kzTB0QQWoOlMoMhgKf6sK*/

#ifndef typedef_SFc17_torqueBalancing2012bInstanceStruct
#define typedef_SFc17_torqueBalancing2012bInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c17_sfEvent;
  boolean_T c17_isStable;
  boolean_T c17_doneDoubleBufferReInit;
  uint8_T c17_is_active_c17_torqueBalancing2012b;
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_reg;
  c17_struct_kzTB0QQWoOlMoMhgKf6sK c17_gain;
} SFc17_torqueBalancing2012bInstanceStruct;

#endif                                 /*typedef_SFc17_torqueBalancing2012bInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c17_torqueBalancing2012b_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c17_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
extern void c17_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
