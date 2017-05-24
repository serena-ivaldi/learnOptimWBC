#ifndef __c1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibrary_h__
#define __c1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibrary_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_SFc1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibraryInstanceStruct
#define typedef_SFc1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_ShVT4BxSmtBPnRwC5cMTHH_sfEvent;
  boolean_T c1_ShVT4BxSmtBPnRwC5cMTHH_isStable;
  boolean_T c1_ShVT4BxSmtBPnRwC5cMTHH_doneDoubleBufferReInit;
  uint8_T c1_ShVT4BxSmtBPnRwC5cMTHH_is_active_c1_ShVT4BxSmtBPnRwC5cMTHH_WB;
  real_T c1_ShVT4BxSmtBPnRwC5cMTHH_state[12];
  boolean_T c1_ShVT4BxSmtBPnRwC5cMTHH_state_not_empty;
} SFc1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibraryInstanceStruct;

#endif                                 /*typedef_SFc1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibrary_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibrary_get_check_sum(mxArray *
  plhs[]);
extern void c1_ShVT4BxSmtBPnRwC5cMTHH_WBToolboxLibrary_method_dispatcher
  (SimStruct *S, int_T method, void *data);

#endif
