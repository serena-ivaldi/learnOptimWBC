#ifndef __c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_h__
#define __c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct
#define typedef_SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_sfEvent;
  boolean_T c1_SaIbQGVykbN93G6oZHOX8C_isStable;
  boolean_T c1_SaIbQGVykbN93G6oZHOX8C_doneDoubleBufferReInit;
  uint8_T c1_SaIbQGVykbN93G6oZHOX8C_is_active_c1_SaIbQGVykbN93G6oZHOX8C_WB;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_state[23];
  boolean_T c1_SaIbQGVykbN93G6oZHOX8C_state_not_empty;
} SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct;

#endif                                 /*typedef_SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_get_check_sum(mxArray *
  plhs[]);
extern void c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_method_dispatcher
  (SimStruct *S, int_T method, void *data);

#endif
