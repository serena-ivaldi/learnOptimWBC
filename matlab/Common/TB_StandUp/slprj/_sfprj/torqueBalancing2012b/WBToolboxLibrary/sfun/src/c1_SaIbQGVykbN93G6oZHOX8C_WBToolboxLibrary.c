/* Include files */

#include <stddef.h>
#include "blas.h"
#include "WBToolboxLibrary_sfun.h"
#include "c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "WBToolboxLibrary_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_SaIbQGVykbN93G6oZHOX8C_debug_family_names[6] = { "nargin",
  "nargout", "s", "unused", "s0", "state" };

/* Function Declarations */
static void initialize_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void initialize_params_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void enable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void disable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void c1_SaIbQGVykbN93G6oZHOX8C_update_debugger_state_c1_SaIbQGVykbN93
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void set_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_st);
static void finalize_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void initSimStructsc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void registerMessagesc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T
  c1_SaIbQGVykbN93G6oZHOX8C_machineNumber, uint32_T
  c1_SaIbQGVykbN93G6oZHOX8C_chartNumber);
static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData);
static void c1_SaIbQGVykbN93G6oZHOX8C_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_b_state, const char_T
   *c1_SaIbQGVykbN93G6oZHOX8C_identifier, real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23]);
static void c1_SaIbQGVykbN93G6oZHOX8C_b_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId, real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23]);
static void c1_SaIbQGVykbN93G6oZHOX8C_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData, const char_T
  *c1_SaIbQGVykbN93G6oZHOX8C_varName, void *c1_SaIbQGVykbN93G6oZHOX8C_outData);
static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData);
static void c1_SaIbQGVykbN93G6oZHOX8C_c_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_s0, const char_T
   *c1_SaIbQGVykbN93G6oZHOX8C_identifier, real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23]);
static void c1_SaIbQGVykbN93G6oZHOX8C_d_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId, real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23]);
static void c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData, const char_T
  *c1_SaIbQGVykbN93G6oZHOX8C_varName, void *c1_SaIbQGVykbN93G6oZHOX8C_outData);
static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_c_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData);
static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_d_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData);
static real_T c1_SaIbQGVykbN93G6oZHOX8C_e_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId);
static void c1_SaIbQGVykbN93G6oZHOX8C_c_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData, const char_T
  *c1_SaIbQGVykbN93G6oZHOX8C_varName, void *c1_SaIbQGVykbN93G6oZHOX8C_outData);
static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_e_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData);
static int32_T c1_SaIbQGVykbN93G6oZHOX8C_f_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId);
static void c1_SaIbQGVykbN93G6oZHOX8C_d_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData, const char_T
  *c1_SaIbQGVykbN93G6oZHOX8C_varName, void *c1_SaIbQGVykbN93G6oZHOX8C_outData);
static uint8_T c1_SaIbQGVykbN93G6oZHOX8C_g_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray
   *c1_SaIbQGVykbN93G6oZHOX8C_b_is_active_c1_SaIbQGVykbN93G6oZHOX8C_, const
   char_T *c1_SaIbQGVykbN93G6oZHOX8C_identifier);
static uint8_T c1_SaIbQGVykbN93G6oZHOX8C_h_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId);
static void init_dsm_address_info
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
  chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state_not_empty = FALSE;
  chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_is_active_c1_SaIbQGVykbN93G6oZHOX8C_WB
    = 0U;
}

static void initialize_params_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
}

static void enable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_SaIbQGVykbN93G6oZHOX8C_update_debugger_state_c1_SaIbQGVykbN93
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_st;
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i0;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_u[23];
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_b_y = NULL;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i1;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_b_u[23];
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_c_y = NULL;
  uint8_T c1_SaIbQGVykbN93G6oZHOX8C_hoistedGlobal;
  uint8_T c1_SaIbQGVykbN93G6oZHOX8C_c_u;
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_d_y = NULL;
  real_T (*c1_SaIbQGVykbN93G6oZHOX8C_s0)[23];
  c1_SaIbQGVykbN93G6oZHOX8C_s0 = (real_T (*)[23])ssGetOutputPortSignal
    (chartInstance->S, 1);
  c1_SaIbQGVykbN93G6oZHOX8C_st = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_st = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_y, sf_mex_createcellarray(3), FALSE);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i0 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i0 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i0++) {
    c1_SaIbQGVykbN93G6oZHOX8C_u[c1_SaIbQGVykbN93G6oZHOX8C_i0] =
      (*c1_SaIbQGVykbN93G6oZHOX8C_s0)[c1_SaIbQGVykbN93G6oZHOX8C_i0];
  }

  c1_SaIbQGVykbN93G6oZHOX8C_b_y = NULL;
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_b_y, sf_mex_create("y",
    c1_SaIbQGVykbN93G6oZHOX8C_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_setcell(c1_SaIbQGVykbN93G6oZHOX8C_y, 0, c1_SaIbQGVykbN93G6oZHOX8C_b_y);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i1 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i1 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i1++) {
    c1_SaIbQGVykbN93G6oZHOX8C_b_u[c1_SaIbQGVykbN93G6oZHOX8C_i1] =
      chartInstance->
      c1_SaIbQGVykbN93G6oZHOX8C_state[c1_SaIbQGVykbN93G6oZHOX8C_i1];
  }

  c1_SaIbQGVykbN93G6oZHOX8C_c_y = NULL;
  if (!chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state_not_empty) {
    sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_c_y, sf_mex_create("y", NULL, 0, 0U,
      1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_c_y, sf_mex_create("y",
      c1_SaIbQGVykbN93G6oZHOX8C_b_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  }

  sf_mex_setcell(c1_SaIbQGVykbN93G6oZHOX8C_y, 1, c1_SaIbQGVykbN93G6oZHOX8C_c_y);
  c1_SaIbQGVykbN93G6oZHOX8C_hoistedGlobal =
    chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_is_active_c1_SaIbQGVykbN93G6oZHOX8C_WB;
  c1_SaIbQGVykbN93G6oZHOX8C_c_u = c1_SaIbQGVykbN93G6oZHOX8C_hoistedGlobal;
  c1_SaIbQGVykbN93G6oZHOX8C_d_y = NULL;
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_d_y, sf_mex_create("y",
    &c1_SaIbQGVykbN93G6oZHOX8C_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_SaIbQGVykbN93G6oZHOX8C_y, 2, c1_SaIbQGVykbN93G6oZHOX8C_d_y);
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_st, c1_SaIbQGVykbN93G6oZHOX8C_y,
                FALSE);
  return c1_SaIbQGVykbN93G6oZHOX8C_st;
}

static void set_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_st)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_dv0[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i2;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_dv1[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i3;
  real_T (*c1_SaIbQGVykbN93G6oZHOX8C_s0)[23];
  c1_SaIbQGVykbN93G6oZHOX8C_s0 = (real_T (*)[23])ssGetOutputPortSignal
    (chartInstance->S, 1);
  chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_doneDoubleBufferReInit = TRUE;
  c1_SaIbQGVykbN93G6oZHOX8C_u = sf_mex_dup(c1_SaIbQGVykbN93G6oZHOX8C_st);
  c1_SaIbQGVykbN93G6oZHOX8C_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_SaIbQGVykbN93G6oZHOX8C_u, 0)), "s0",
    c1_SaIbQGVykbN93G6oZHOX8C_dv0);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i2 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i2 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i2++) {
    (*c1_SaIbQGVykbN93G6oZHOX8C_s0)[c1_SaIbQGVykbN93G6oZHOX8C_i2] =
      c1_SaIbQGVykbN93G6oZHOX8C_dv0[c1_SaIbQGVykbN93G6oZHOX8C_i2];
  }

  c1_SaIbQGVykbN93G6oZHOX8C_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_SaIbQGVykbN93G6oZHOX8C_u, 1)), "state",
    c1_SaIbQGVykbN93G6oZHOX8C_dv1);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i3 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i3 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i3++) {
    chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state[c1_SaIbQGVykbN93G6oZHOX8C_i3]
      = c1_SaIbQGVykbN93G6oZHOX8C_dv1[c1_SaIbQGVykbN93G6oZHOX8C_i3];
  }

  chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_is_active_c1_SaIbQGVykbN93G6oZHOX8C_WB
    = c1_SaIbQGVykbN93G6oZHOX8C_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_SaIbQGVykbN93G6oZHOX8C_u, 2)),
    "is_active_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary");
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_u);
  c1_SaIbQGVykbN93G6oZHOX8C_update_debugger_state_c1_SaIbQGVykbN93(chartInstance);
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_st);
}

static void finalize_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
}

static void sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i4;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i5;
  boolean_T c1_SaIbQGVykbN93G6oZHOX8C_hoistedGlobal;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i6;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_s[23];
  boolean_T c1_SaIbQGVykbN93G6oZHOX8C_unused;
  uint32_T c1_SaIbQGVykbN93G6oZHOX8C_debug_family_var_map[6];
  real_T c1_SaIbQGVykbN93G6oZHOX8C_nargin = 2.0;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_nargout = 1.0;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_s0[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i7;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i8;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i9;
  boolean_T *c1_SaIbQGVykbN93G6oZHOX8C_b_unused;
  real_T (*c1_SaIbQGVykbN93G6oZHOX8C_b_s0)[23];
  real_T (*c1_SaIbQGVykbN93G6oZHOX8C_b_s)[23];
  c1_SaIbQGVykbN93G6oZHOX8C_b_unused = (boolean_T *)ssGetInputPortSignal
    (chartInstance->S, 1);
  c1_SaIbQGVykbN93G6oZHOX8C_b_s0 = (real_T (*)[23])ssGetOutputPortSignal
    (chartInstance->S, 1);
  c1_SaIbQGVykbN93G6oZHOX8C_b_s = (real_T (*)[23])ssGetInputPortSignal
    (chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U,
               chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i4 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i4 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i4++) {
    _SFD_DATA_RANGE_CHECK((*c1_SaIbQGVykbN93G6oZHOX8C_b_s)
                          [c1_SaIbQGVykbN93G6oZHOX8C_i4], 0U);
  }

  for (c1_SaIbQGVykbN93G6oZHOX8C_i5 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i5 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i5++) {
    _SFD_DATA_RANGE_CHECK((*c1_SaIbQGVykbN93G6oZHOX8C_b_s0)
                          [c1_SaIbQGVykbN93G6oZHOX8C_i5], 1U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)*c1_SaIbQGVykbN93G6oZHOX8C_b_unused, 2U);
  chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U,
               chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent);
  c1_SaIbQGVykbN93G6oZHOX8C_hoistedGlobal = *c1_SaIbQGVykbN93G6oZHOX8C_b_unused;
  for (c1_SaIbQGVykbN93G6oZHOX8C_i6 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i6 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i6++) {
    c1_SaIbQGVykbN93G6oZHOX8C_s[c1_SaIbQGVykbN93G6oZHOX8C_i6] =
      (*c1_SaIbQGVykbN93G6oZHOX8C_b_s)[c1_SaIbQGVykbN93G6oZHOX8C_i6];
  }

  c1_SaIbQGVykbN93G6oZHOX8C_unused = c1_SaIbQGVykbN93G6oZHOX8C_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U,
    c1_SaIbQGVykbN93G6oZHOX8C_debug_family_names,
    c1_SaIbQGVykbN93G6oZHOX8C_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_SaIbQGVykbN93G6oZHOX8C_nargin, 0U,
    c1_SaIbQGVykbN93G6oZHOX8C_d_sf_marshallOut,
    c1_SaIbQGVykbN93G6oZHOX8C_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_SaIbQGVykbN93G6oZHOX8C_nargout, 1U,
    c1_SaIbQGVykbN93G6oZHOX8C_d_sf_marshallOut,
    c1_SaIbQGVykbN93G6oZHOX8C_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_SaIbQGVykbN93G6oZHOX8C_s, 2U,
    c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_SaIbQGVykbN93G6oZHOX8C_unused, 3U,
    c1_SaIbQGVykbN93G6oZHOX8C_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_SaIbQGVykbN93G6oZHOX8C_s0, 4U,
    c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallOut,
    c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE
    (chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state, 5U,
     c1_SaIbQGVykbN93G6oZHOX8C_sf_marshallOut,
     c1_SaIbQGVykbN93G6oZHOX8C_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent, 2);
  _SFD_EML_CALL(0U, chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent, 5);
  if (CV_EML_IF(0, 1, 0,
                !chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent, 6);
    for (c1_SaIbQGVykbN93G6oZHOX8C_i7 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i7 < 23;
         c1_SaIbQGVykbN93G6oZHOX8C_i7++) {
      chartInstance->
        c1_SaIbQGVykbN93G6oZHOX8C_state[c1_SaIbQGVykbN93G6oZHOX8C_i7] =
        c1_SaIbQGVykbN93G6oZHOX8C_s[c1_SaIbQGVykbN93G6oZHOX8C_i7];
    }

    chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent, 9);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i8 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i8 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i8++) {
    c1_SaIbQGVykbN93G6oZHOX8C_s0[c1_SaIbQGVykbN93G6oZHOX8C_i8] =
      chartInstance->
      c1_SaIbQGVykbN93G6oZHOX8C_state[c1_SaIbQGVykbN93G6oZHOX8C_i8];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent, -9);
  _SFD_SYMBOL_SCOPE_POP();
  for (c1_SaIbQGVykbN93G6oZHOX8C_i9 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i9 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i9++) {
    (*c1_SaIbQGVykbN93G6oZHOX8C_b_s0)[c1_SaIbQGVykbN93G6oZHOX8C_i9] =
      c1_SaIbQGVykbN93G6oZHOX8C_s0[c1_SaIbQGVykbN93G6oZHOX8C_i9];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U,
               chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_WBToolboxLibraryMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
}

static void registerMessagesc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T
  c1_SaIbQGVykbN93G6oZHOX8C_machineNumber, uint32_T
  c1_SaIbQGVykbN93G6oZHOX8C_chartNumber)
{
}

static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i10;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_b_inData[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i11;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_u[23];
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  for (c1_SaIbQGVykbN93G6oZHOX8C_i10 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i10 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i10++) {
    c1_SaIbQGVykbN93G6oZHOX8C_b_inData[c1_SaIbQGVykbN93G6oZHOX8C_i10] =
      (*(real_T (*)[23])c1_SaIbQGVykbN93G6oZHOX8C_inData)
      [c1_SaIbQGVykbN93G6oZHOX8C_i10];
  }

  for (c1_SaIbQGVykbN93G6oZHOX8C_i11 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i11 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i11++) {
    c1_SaIbQGVykbN93G6oZHOX8C_u[c1_SaIbQGVykbN93G6oZHOX8C_i11] =
      c1_SaIbQGVykbN93G6oZHOX8C_b_inData[c1_SaIbQGVykbN93G6oZHOX8C_i11];
  }

  c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  if (!chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state_not_empty) {
    sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_y, sf_mex_create("y", NULL, 0, 0U,
      1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_y, sf_mex_create("y",
      c1_SaIbQGVykbN93G6oZHOX8C_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  }

  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData,
                c1_SaIbQGVykbN93G6oZHOX8C_y, FALSE);
  return c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData;
}

static void c1_SaIbQGVykbN93G6oZHOX8C_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_b_state, const char_T
   *c1_SaIbQGVykbN93G6oZHOX8C_identifier, real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23])
{
  emlrtMsgIdentifier c1_SaIbQGVykbN93G6oZHOX8C_thisId;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fIdentifier =
    c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fParent = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_SaIbQGVykbN93G6oZHOX8C_b_state), &c1_SaIbQGVykbN93G6oZHOX8C_thisId,
    c1_SaIbQGVykbN93G6oZHOX8C_y);
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_b_state);
}

static void c1_SaIbQGVykbN93G6oZHOX8C_b_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId, real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23])
{
  real_T c1_SaIbQGVykbN93G6oZHOX8C_dv2[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i12;
  if (mxIsEmpty(c1_SaIbQGVykbN93G6oZHOX8C_u)) {
    chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state_not_empty = FALSE;
  } else {
    chartInstance->c1_SaIbQGVykbN93G6oZHOX8C_state_not_empty = TRUE;
    sf_mex_import(c1_SaIbQGVykbN93G6oZHOX8C_parentId, sf_mex_dup
                  (c1_SaIbQGVykbN93G6oZHOX8C_u), c1_SaIbQGVykbN93G6oZHOX8C_dv2,
                  1, 0, 0U, 1, 0U, 1, 23);
    for (c1_SaIbQGVykbN93G6oZHOX8C_i12 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i12 < 23;
         c1_SaIbQGVykbN93G6oZHOX8C_i12++) {
      c1_SaIbQGVykbN93G6oZHOX8C_y[c1_SaIbQGVykbN93G6oZHOX8C_i12] =
        c1_SaIbQGVykbN93G6oZHOX8C_dv2[c1_SaIbQGVykbN93G6oZHOX8C_i12];
    }
  }

  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_u);
}

static void c1_SaIbQGVykbN93G6oZHOX8C_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData, const char_T
  *c1_SaIbQGVykbN93G6oZHOX8C_varName, void *c1_SaIbQGVykbN93G6oZHOX8C_outData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_b_state;
  const char_T *c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  emlrtMsgIdentifier c1_SaIbQGVykbN93G6oZHOX8C_thisId;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i13;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_b_state = sf_mex_dup
    (c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData);
  c1_SaIbQGVykbN93G6oZHOX8C_identifier = c1_SaIbQGVykbN93G6oZHOX8C_varName;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fIdentifier =
    c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fParent = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_SaIbQGVykbN93G6oZHOX8C_b_state), &c1_SaIbQGVykbN93G6oZHOX8C_thisId,
    c1_SaIbQGVykbN93G6oZHOX8C_y);
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_b_state);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i13 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i13 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i13++) {
    (*(real_T (*)[23])c1_SaIbQGVykbN93G6oZHOX8C_outData)
      [c1_SaIbQGVykbN93G6oZHOX8C_i13] =
      c1_SaIbQGVykbN93G6oZHOX8C_y[c1_SaIbQGVykbN93G6oZHOX8C_i13];
  }

  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData);
}

static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i14;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_b_inData[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i15;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_u[23];
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  for (c1_SaIbQGVykbN93G6oZHOX8C_i14 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i14 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i14++) {
    c1_SaIbQGVykbN93G6oZHOX8C_b_inData[c1_SaIbQGVykbN93G6oZHOX8C_i14] =
      (*(real_T (*)[23])c1_SaIbQGVykbN93G6oZHOX8C_inData)
      [c1_SaIbQGVykbN93G6oZHOX8C_i14];
  }

  for (c1_SaIbQGVykbN93G6oZHOX8C_i15 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i15 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i15++) {
    c1_SaIbQGVykbN93G6oZHOX8C_u[c1_SaIbQGVykbN93G6oZHOX8C_i15] =
      c1_SaIbQGVykbN93G6oZHOX8C_b_inData[c1_SaIbQGVykbN93G6oZHOX8C_i15];
  }

  c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_y, sf_mex_create("y",
    c1_SaIbQGVykbN93G6oZHOX8C_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData,
                c1_SaIbQGVykbN93G6oZHOX8C_y, FALSE);
  return c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData;
}

static void c1_SaIbQGVykbN93G6oZHOX8C_c_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_s0, const char_T
   *c1_SaIbQGVykbN93G6oZHOX8C_identifier, real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23])
{
  emlrtMsgIdentifier c1_SaIbQGVykbN93G6oZHOX8C_thisId;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fIdentifier =
    c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fParent = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_SaIbQGVykbN93G6oZHOX8C_s0), &c1_SaIbQGVykbN93G6oZHOX8C_thisId,
    c1_SaIbQGVykbN93G6oZHOX8C_y);
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_s0);
}

static void c1_SaIbQGVykbN93G6oZHOX8C_d_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId, real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23])
{
  real_T c1_SaIbQGVykbN93G6oZHOX8C_dv3[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i16;
  sf_mex_import(c1_SaIbQGVykbN93G6oZHOX8C_parentId, sf_mex_dup
                (c1_SaIbQGVykbN93G6oZHOX8C_u), c1_SaIbQGVykbN93G6oZHOX8C_dv3, 1,
                0, 0U, 1, 0U, 1, 23);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i16 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i16 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i16++) {
    c1_SaIbQGVykbN93G6oZHOX8C_y[c1_SaIbQGVykbN93G6oZHOX8C_i16] =
      c1_SaIbQGVykbN93G6oZHOX8C_dv3[c1_SaIbQGVykbN93G6oZHOX8C_i16];
  }

  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_u);
}

static void c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData, const char_T
  *c1_SaIbQGVykbN93G6oZHOX8C_varName, void *c1_SaIbQGVykbN93G6oZHOX8C_outData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_s0;
  const char_T *c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  emlrtMsgIdentifier c1_SaIbQGVykbN93G6oZHOX8C_thisId;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_y[23];
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i17;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_s0 = sf_mex_dup
    (c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData);
  c1_SaIbQGVykbN93G6oZHOX8C_identifier = c1_SaIbQGVykbN93G6oZHOX8C_varName;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fIdentifier =
    c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fParent = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_SaIbQGVykbN93G6oZHOX8C_s0), &c1_SaIbQGVykbN93G6oZHOX8C_thisId,
    c1_SaIbQGVykbN93G6oZHOX8C_y);
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_s0);
  for (c1_SaIbQGVykbN93G6oZHOX8C_i17 = 0; c1_SaIbQGVykbN93G6oZHOX8C_i17 < 23;
       c1_SaIbQGVykbN93G6oZHOX8C_i17++) {
    (*(real_T (*)[23])c1_SaIbQGVykbN93G6oZHOX8C_outData)
      [c1_SaIbQGVykbN93G6oZHOX8C_i17] =
      c1_SaIbQGVykbN93G6oZHOX8C_y[c1_SaIbQGVykbN93G6oZHOX8C_i17];
  }

  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData);
}

static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_c_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  boolean_T c1_SaIbQGVykbN93G6oZHOX8C_u;
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_u = *(boolean_T *)c1_SaIbQGVykbN93G6oZHOX8C_inData;
  c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_y, sf_mex_create("y",
    &c1_SaIbQGVykbN93G6oZHOX8C_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData,
                c1_SaIbQGVykbN93G6oZHOX8C_y, FALSE);
  return c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData;
}

static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_d_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_u;
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_u = *(real_T *)c1_SaIbQGVykbN93G6oZHOX8C_inData;
  c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_y, sf_mex_create("y",
    &c1_SaIbQGVykbN93G6oZHOX8C_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData,
                c1_SaIbQGVykbN93G6oZHOX8C_y, FALSE);
  return c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData;
}

static real_T c1_SaIbQGVykbN93G6oZHOX8C_e_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId)
{
  real_T c1_SaIbQGVykbN93G6oZHOX8C_y;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_d0;
  sf_mex_import(c1_SaIbQGVykbN93G6oZHOX8C_parentId, sf_mex_dup
                (c1_SaIbQGVykbN93G6oZHOX8C_u), &c1_SaIbQGVykbN93G6oZHOX8C_d0, 1,
                0, 0U, 0, 0U, 0);
  c1_SaIbQGVykbN93G6oZHOX8C_y = c1_SaIbQGVykbN93G6oZHOX8C_d0;
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_u);
  return c1_SaIbQGVykbN93G6oZHOX8C_y;
}

static void c1_SaIbQGVykbN93G6oZHOX8C_c_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData, const char_T
  *c1_SaIbQGVykbN93G6oZHOX8C_varName, void *c1_SaIbQGVykbN93G6oZHOX8C_outData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_nargout;
  const char_T *c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  emlrtMsgIdentifier c1_SaIbQGVykbN93G6oZHOX8C_thisId;
  real_T c1_SaIbQGVykbN93G6oZHOX8C_y;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_nargout = sf_mex_dup
    (c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData);
  c1_SaIbQGVykbN93G6oZHOX8C_identifier = c1_SaIbQGVykbN93G6oZHOX8C_varName;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fIdentifier =
    c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fParent = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_y = c1_SaIbQGVykbN93G6oZHOX8C_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(c1_SaIbQGVykbN93G6oZHOX8C_nargout),
     &c1_SaIbQGVykbN93G6oZHOX8C_thisId);
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_nargout);
  *(real_T *)c1_SaIbQGVykbN93G6oZHOX8C_outData = c1_SaIbQGVykbN93G6oZHOX8C_y;
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData);
}

const mxArray
  *sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_nameCaptureInfo = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_nameCaptureInfo, sf_mex_create(
    "nameCaptureInfo", NULL, 0, 0U, 1U, 0U, 2, 0, 1), FALSE);
  return c1_SaIbQGVykbN93G6oZHOX8C_nameCaptureInfo;
}

static const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_e_sf_marshallOut(void
  *chartInstanceVoid, void *c1_SaIbQGVykbN93G6oZHOX8C_inData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_u;
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_u = *(int32_T *)c1_SaIbQGVykbN93G6oZHOX8C_inData;
  c1_SaIbQGVykbN93G6oZHOX8C_y = NULL;
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_y, sf_mex_create("y",
    &c1_SaIbQGVykbN93G6oZHOX8C_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData,
                c1_SaIbQGVykbN93G6oZHOX8C_y, FALSE);
  return c1_SaIbQGVykbN93G6oZHOX8C_mxArrayOutData;
}

static int32_T c1_SaIbQGVykbN93G6oZHOX8C_f_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId)
{
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_y;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_i18;
  sf_mex_import(c1_SaIbQGVykbN93G6oZHOX8C_parentId, sf_mex_dup
                (c1_SaIbQGVykbN93G6oZHOX8C_u), &c1_SaIbQGVykbN93G6oZHOX8C_i18, 1,
                6, 0U, 0, 0U, 0);
  c1_SaIbQGVykbN93G6oZHOX8C_y = c1_SaIbQGVykbN93G6oZHOX8C_i18;
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_u);
  return c1_SaIbQGVykbN93G6oZHOX8C_y;
}

static void c1_SaIbQGVykbN93G6oZHOX8C_d_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData, const char_T
  *c1_SaIbQGVykbN93G6oZHOX8C_varName, void *c1_SaIbQGVykbN93G6oZHOX8C_outData)
{
  const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_b_sfEvent;
  const char_T *c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  emlrtMsgIdentifier c1_SaIbQGVykbN93G6oZHOX8C_thisId;
  int32_T c1_SaIbQGVykbN93G6oZHOX8C_y;
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    chartInstanceVoid;
  c1_SaIbQGVykbN93G6oZHOX8C_b_sfEvent = sf_mex_dup
    (c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData);
  c1_SaIbQGVykbN93G6oZHOX8C_identifier = c1_SaIbQGVykbN93G6oZHOX8C_varName;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fIdentifier =
    c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fParent = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_y = c1_SaIbQGVykbN93G6oZHOX8C_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(c1_SaIbQGVykbN93G6oZHOX8C_b_sfEvent),
     &c1_SaIbQGVykbN93G6oZHOX8C_thisId);
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_b_sfEvent);
  *(int32_T *)c1_SaIbQGVykbN93G6oZHOX8C_outData = c1_SaIbQGVykbN93G6oZHOX8C_y;
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_mxArrayInData);
}

static uint8_T c1_SaIbQGVykbN93G6oZHOX8C_g_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray
   *c1_SaIbQGVykbN93G6oZHOX8C_b_is_active_c1_SaIbQGVykbN93G6oZHOX8C_, const
   char_T *c1_SaIbQGVykbN93G6oZHOX8C_identifier)
{
  uint8_T c1_SaIbQGVykbN93G6oZHOX8C_y;
  emlrtMsgIdentifier c1_SaIbQGVykbN93G6oZHOX8C_thisId;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fIdentifier =
    c1_SaIbQGVykbN93G6oZHOX8C_identifier;
  c1_SaIbQGVykbN93G6oZHOX8C_thisId.fParent = NULL;
  c1_SaIbQGVykbN93G6oZHOX8C_y = c1_SaIbQGVykbN93G6oZHOX8C_h_emlrt_marshallIn
    (chartInstance, sf_mex_dup
     (c1_SaIbQGVykbN93G6oZHOX8C_b_is_active_c1_SaIbQGVykbN93G6oZHOX8C_),
     &c1_SaIbQGVykbN93G6oZHOX8C_thisId);
  sf_mex_destroy
    (&c1_SaIbQGVykbN93G6oZHOX8C_b_is_active_c1_SaIbQGVykbN93G6oZHOX8C_);
  return c1_SaIbQGVykbN93G6oZHOX8C_y;
}

static uint8_T c1_SaIbQGVykbN93G6oZHOX8C_h_emlrt_marshallIn
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance,
   const mxArray *c1_SaIbQGVykbN93G6oZHOX8C_u, const emlrtMsgIdentifier
   *c1_SaIbQGVykbN93G6oZHOX8C_parentId)
{
  uint8_T c1_SaIbQGVykbN93G6oZHOX8C_y;
  uint8_T c1_SaIbQGVykbN93G6oZHOX8C_u0;
  sf_mex_import(c1_SaIbQGVykbN93G6oZHOX8C_parentId, sf_mex_dup
                (c1_SaIbQGVykbN93G6oZHOX8C_u), &c1_SaIbQGVykbN93G6oZHOX8C_u0, 1,
                3, 0U, 0, 0U, 0);
  c1_SaIbQGVykbN93G6oZHOX8C_y = c1_SaIbQGVykbN93G6oZHOX8C_u0;
  sf_mex_destroy(&c1_SaIbQGVykbN93G6oZHOX8C_u);
  return c1_SaIbQGVykbN93G6oZHOX8C_y;
}

static void init_dsm_address_info
  (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2475230468U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(68617237U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(83058364U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1512886126U);
}

mxArray *sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_get_autoinheritance_info
  (void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("vhNe8nid1i2OA7agMFKZn");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_third_party_uses_info
  (void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray
  *sf_get_sim_state_info_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[8],T\"s0\",},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[35 40],M[0],}}},{M[8],M[0],T\"is_active_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
    chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _WBToolboxLibraryMachineNumber_,
           1,
           1,
           1,
           3,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_WBToolboxLibraryMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_WBToolboxLibraryMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _WBToolboxLibraryMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"s");
          _SFD_SET_DATA_PROPS(1,2,0,1,"s0");
          _SFD_SET_DATA_PROPS(2,1,1,0,"unused");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,101);
        _SFD_CV_INIT_EML_IF(0,1,0,52,69,-1,88);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallOut,
            (MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallOut,
            (MexInFcnForType)c1_SaIbQGVykbN93G6oZHOX8C_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_SaIbQGVykbN93G6oZHOX8C_c_sf_marshallOut,
          (MexInFcnForType)NULL);

        {
          boolean_T *c1_SaIbQGVykbN93G6oZHOX8C_unused;
          real_T (*c1_SaIbQGVykbN93G6oZHOX8C_s)[23];
          real_T (*c1_SaIbQGVykbN93G6oZHOX8C_s0)[23];
          c1_SaIbQGVykbN93G6oZHOX8C_unused = (boolean_T *)ssGetInputPortSignal
            (chartInstance->S, 1);
          c1_SaIbQGVykbN93G6oZHOX8C_s0 = (real_T (*)[23])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c1_SaIbQGVykbN93G6oZHOX8C_s = (real_T (*)[23])ssGetInputPortSignal
            (chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c1_SaIbQGVykbN93G6oZHOX8C_s);
          _SFD_SET_DATA_VALUE_PTR(1U, *c1_SaIbQGVykbN93G6oZHOX8C_s0);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_SaIbQGVykbN93G6oZHOX8C_unused);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _WBToolboxLibraryMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "SaIbQGVykbN93G6oZHOX8C";
}

static void sf_opaque_initialize_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(void
  *chartInstanceVar)
{
  chart_debug_initialization
    (((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
      chartInstanceVar)->S,0);
  initialize_params_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
    ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
     chartInstanceVar);
  initialize_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
    ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_enable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(void
  *chartInstanceVar)
{
  enable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
    ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_disable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(void
  *chartInstanceVar)
{
  disable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
    ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_gateway_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(void
  *chartInstanceVar)
{
  sf_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
    ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
     chartInstanceVar);
}

extern const mxArray*
  sf_internal_get_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
    ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
     chartInfo->chartInstance);        /* raw sim ctx */
  prhs[3] = (mxArray*)
    sf_get_sim_state_info_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SimStruct* S, const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*)
    sf_get_sim_state_info_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
    ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
     chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray*
  sf_opaque_get_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(S);
}

static void sf_opaque_set_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(S, st);
}

static void sf_opaque_terminate_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(void *
  chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_WBToolboxLibrary_optimization_info();
    }

    finalize_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
      ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
       chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
    ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
     chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
      ((SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct*)
       (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary
  (SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_WBToolboxLibrary_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2475230468U));
  ssSetChecksum1(S,(68617237U));
  ssSetChecksum2(S,(83058364U));
  ssSetChecksum3(S,(1512886126U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(SimStruct *S)
{
  SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct *)
    utMalloc(sizeof(SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW =
    mdlRTW_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.mdlStart =
    mdlStart_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_method_dispatcher(SimStruct *S,
  int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_SaIbQGVykbN93G6oZHOX8C_WBToolboxLibrary_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
