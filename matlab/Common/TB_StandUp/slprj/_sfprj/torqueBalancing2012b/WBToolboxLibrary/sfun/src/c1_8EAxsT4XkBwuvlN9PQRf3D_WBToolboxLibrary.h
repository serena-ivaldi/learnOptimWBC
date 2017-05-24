#ifndef __c1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibrary_h__
#define __c1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibrary_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_SFc1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibraryInstanceStruct
#define typedef_SFc1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_8EAxsT4XkBwuvlN9PQRf3D_sfEvent;
  boolean_T c1_8EAxsT4XkBwuvlN9PQRf3D_isStable;
  boolean_T c1_8EAxsT4XkBwuvlN9PQRf3D_doneDoubleBufferReInit;
  uint8_T c1_8EAxsT4XkBwuvlN9PQRf3D_is_active_c1_8EAxsT4XkBwuvlN9PQRf3D_WB;
  real_T c1_8EAxsT4XkBwuvlN9PQRf3D_state[16];
  boolean_T c1_8EAxsT4XkBwuvlN9PQRf3D_state_not_empty;
} SFc1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibraryInstanceStruct;

#endif                                 /*typedef_SFc1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibrary_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibrary_get_check_sum(mxArray *
  plhs[]);
extern void c1_8EAxsT4XkBwuvlN9PQRf3D_WBToolboxLibrary_method_dispatcher
  (SimStruct *S, int_T method, void *data);

#endif
