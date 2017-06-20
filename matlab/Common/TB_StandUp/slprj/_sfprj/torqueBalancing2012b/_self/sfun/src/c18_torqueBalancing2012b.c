/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c18_torqueBalancing2012b.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "torqueBalancing2012b_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c18_debug_family_names[8] = { "res", "nargin", "nargout",
  "umin", "umax", "u", "tol", "inRange" };

/* Function Declarations */
static void initialize_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static void c18_update_debugger_state_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c18_st);
static void finalize_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c18_torqueBalancing2012b(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c18_machineNumber, uint32_T
  c18_chartNumber);
static const mxArray *c18_sf_marshallOut(void *chartInstanceVoid, void
  *c18_inData);
static real_T c18_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_inRange, const char_T *c18_identifier);
static real_T c18_b_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_u, const emlrtMsgIdentifier *c18_parentId);
static void c18_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c18_mxArrayInData, const char_T *c18_varName, void *c18_outData);
static const mxArray *c18_b_sf_marshallOut(void *chartInstanceVoid, void
  *c18_inData);
static const mxArray *c18_c_sf_marshallOut(void *chartInstanceVoid, void
  *c18_inData);
static void c18_c_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_u, const emlrtMsgIdentifier *c18_parentId,
  boolean_T c18_y[23]);
static void c18_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c18_mxArrayInData, const char_T *c18_varName, void *c18_outData);
static const mxArray *c18_d_sf_marshallOut(void *chartInstanceVoid, void
  *c18_inData);
static int32_T c18_d_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_u, const emlrtMsgIdentifier *c18_parentId);
static void c18_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c18_mxArrayInData, const char_T *c18_varName, void *c18_outData);
static uint8_T c18_e_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_b_is_active_c18_torqueBalancing2012b, const
  char_T *c18_identifier);
static uint8_T c18_f_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_u, const emlrtMsgIdentifier *c18_parentId);
static void init_dsm_address_info(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c18_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c18_is_active_c18_torqueBalancing2012b = 0U;
}

static void initialize_params_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void enable_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c18_update_debugger_state_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c18_st;
  const mxArray *c18_y = NULL;
  real_T c18_hoistedGlobal;
  real_T c18_u;
  const mxArray *c18_b_y = NULL;
  uint8_T c18_b_hoistedGlobal;
  uint8_T c18_b_u;
  const mxArray *c18_c_y = NULL;
  real_T *c18_inRange;
  c18_inRange = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c18_st = NULL;
  c18_st = NULL;
  c18_y = NULL;
  sf_mex_assign(&c18_y, sf_mex_createcellarray(2), FALSE);
  c18_hoistedGlobal = *c18_inRange;
  c18_u = c18_hoistedGlobal;
  c18_b_y = NULL;
  sf_mex_assign(&c18_b_y, sf_mex_create("y", &c18_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c18_y, 0, c18_b_y);
  c18_b_hoistedGlobal = chartInstance->c18_is_active_c18_torqueBalancing2012b;
  c18_b_u = c18_b_hoistedGlobal;
  c18_c_y = NULL;
  sf_mex_assign(&c18_c_y, sf_mex_create("y", &c18_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c18_y, 1, c18_c_y);
  sf_mex_assign(&c18_st, c18_y, FALSE);
  return c18_st;
}

static void set_sim_state_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c18_st)
{
  const mxArray *c18_u;
  real_T *c18_inRange;
  c18_inRange = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c18_doneDoubleBufferReInit = TRUE;
  c18_u = sf_mex_dup(c18_st);
  *c18_inRange = c18_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c18_u, 0)), "inRange");
  chartInstance->c18_is_active_c18_torqueBalancing2012b = c18_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c18_u, 1)),
     "is_active_c18_torqueBalancing2012b");
  sf_mex_destroy(&c18_u);
  c18_update_debugger_state_c18_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c18_st);
}

static void finalize_c18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c18_torqueBalancing2012b(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c18_i0;
  int32_T c18_i1;
  int32_T c18_i2;
  real_T c18_hoistedGlobal;
  int32_T c18_i3;
  real_T c18_umin[23];
  int32_T c18_i4;
  real_T c18_umax[23];
  int32_T c18_i5;
  real_T c18_u[23];
  real_T c18_tol;
  uint32_T c18_debug_family_var_map[8];
  boolean_T c18_res[23];
  real_T c18_b_res;
  real_T c18_nargin = 4.0;
  real_T c18_nargout = 1.0;
  real_T c18_inRange;
  int32_T c18_i6;
  boolean_T c18_x[23];
  int32_T c18_i7;
  boolean_T c18_bv0[23];
  int32_T c18_i8;
  int32_T c18_i9;
  int32_T c18_k;
  int32_T c18_b_k;
  int32_T c18_i10;
  int32_T c18_i11;
  int32_T c18_i12;
  boolean_T c18_b_u[23];
  const mxArray *c18_y = NULL;
  real_T *c18_b_inRange;
  real_T *c18_b_tol;
  real_T (*c18_c_u)[23];
  real_T (*c18_b_umax)[23];
  real_T (*c18_b_umin)[23];
  c18_b_tol = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c18_b_inRange = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c18_c_u = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 2);
  c18_b_umax = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 1);
  c18_b_umin = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 16U, chartInstance->c18_sfEvent);
  for (c18_i0 = 0; c18_i0 < 23; c18_i0++) {
    _SFD_DATA_RANGE_CHECK((*c18_b_umin)[c18_i0], 0U);
  }

  for (c18_i1 = 0; c18_i1 < 23; c18_i1++) {
    _SFD_DATA_RANGE_CHECK((*c18_b_umax)[c18_i1], 1U);
  }

  for (c18_i2 = 0; c18_i2 < 23; c18_i2++) {
    _SFD_DATA_RANGE_CHECK((*c18_c_u)[c18_i2], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c18_b_inRange, 3U);
  _SFD_DATA_RANGE_CHECK(*c18_b_tol, 4U);
  chartInstance->c18_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 16U, chartInstance->c18_sfEvent);
  c18_hoistedGlobal = *c18_b_tol;
  for (c18_i3 = 0; c18_i3 < 23; c18_i3++) {
    c18_umin[c18_i3] = (*c18_b_umin)[c18_i3];
  }

  for (c18_i4 = 0; c18_i4 < 23; c18_i4++) {
    c18_umax[c18_i4] = (*c18_b_umax)[c18_i4];
  }

  for (c18_i5 = 0; c18_i5 < 23; c18_i5++) {
    c18_u[c18_i5] = (*c18_c_u)[c18_i5];
  }

  c18_tol = c18_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 9U, c18_debug_family_names,
    c18_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c18_res, MAX_uint32_T,
    c18_c_sf_marshallOut, c18_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c18_b_res, MAX_uint32_T,
    c18_sf_marshallOut, c18_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c18_nargin, 1U, c18_sf_marshallOut,
    c18_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c18_nargout, 2U, c18_sf_marshallOut,
    c18_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c18_umin, 3U, c18_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c18_umax, 4U, c18_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c18_u, 5U, c18_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c18_tol, 6U, c18_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c18_inRange, 7U, c18_sf_marshallOut,
    c18_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c18_sfEvent, 4);
  for (c18_i6 = 0; c18_i6 < 23; c18_i6++) {
    c18_x[c18_i6] = (c18_u[c18_i6] < c18_umin[c18_i6] + c18_tol);
  }

  for (c18_i7 = 0; c18_i7 < 23; c18_i7++) {
    c18_bv0[c18_i7] = (c18_u[c18_i7] > c18_umax[c18_i7] - c18_tol);
  }

  for (c18_i8 = 0; c18_i8 < 23; c18_i8++) {
    c18_res[c18_i8] = (c18_x[c18_i8] || c18_bv0[c18_i8]);
  }

  _SFD_SYMBOL_SWITCH(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c18_sfEvent, 5);
  for (c18_i9 = 0; c18_i9 < 23; c18_i9++) {
    c18_x[c18_i9] = c18_res[c18_i9];
  }

  c18_b_res = (real_T)c18_x[0];
  for (c18_k = 2; c18_k < 24; c18_k++) {
    c18_b_k = c18_k;
    c18_b_res += (real_T)c18_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c18_b_k), 1, 23, 1, 0) - 1];
  }

  _SFD_SYMBOL_SWITCH(0U, 1U);
  _SFD_EML_CALL(0U, chartInstance->c18_sfEvent, 6);
  if (CV_EML_IF(0, 1, 0, c18_b_res == 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c18_sfEvent, 7);
    c18_inRange = 1.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c18_sfEvent, 9);
    c18_inRange = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c18_sfEvent, 10);
    for (c18_i10 = 0; c18_i10 < 23; c18_i10++) {
      c18_x[c18_i10] = (c18_u[c18_i10] < c18_umin[c18_i10] + c18_tol);
    }

    for (c18_i11 = 0; c18_i11 < 23; c18_i11++) {
      c18_bv0[c18_i11] = (c18_u[c18_i11] > c18_umax[c18_i11] - c18_tol);
    }

    sf_mex_printf("%s =\\n", "ans");
    for (c18_i12 = 0; c18_i12 < 23; c18_i12++) {
      c18_b_u[c18_i12] = (c18_x[c18_i12] || c18_bv0[c18_i12]);
    }

    c18_y = NULL;
    sf_mex_assign(&c18_y, sf_mex_create("y", c18_b_u, 11, 0U, 1U, 0U, 1, 23),
                  FALSE);
    sf_mex_call_debug("disp", 0U, 1U, 14, c18_y);
  }

  _SFD_EML_CALL(0U, chartInstance->c18_sfEvent, -10);
  _SFD_SYMBOL_SCOPE_POP();
  *c18_b_inRange = c18_inRange;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 16U, chartInstance->c18_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc18_torqueBalancing2012b
  (SFc18_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c18_machineNumber, uint32_T
  c18_chartNumber)
{
}

static const mxArray *c18_sf_marshallOut(void *chartInstanceVoid, void
  *c18_inData)
{
  const mxArray *c18_mxArrayOutData = NULL;
  real_T c18_u;
  const mxArray *c18_y = NULL;
  SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c18_mxArrayOutData = NULL;
  c18_u = *(real_T *)c18_inData;
  c18_y = NULL;
  sf_mex_assign(&c18_y, sf_mex_create("y", &c18_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c18_mxArrayOutData, c18_y, FALSE);
  return c18_mxArrayOutData;
}

static real_T c18_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_inRange, const char_T *c18_identifier)
{
  real_T c18_y;
  emlrtMsgIdentifier c18_thisId;
  c18_thisId.fIdentifier = c18_identifier;
  c18_thisId.fParent = NULL;
  c18_y = c18_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c18_inRange),
    &c18_thisId);
  sf_mex_destroy(&c18_inRange);
  return c18_y;
}

static real_T c18_b_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_u, const emlrtMsgIdentifier *c18_parentId)
{
  real_T c18_y;
  real_T c18_d0;
  sf_mex_import(c18_parentId, sf_mex_dup(c18_u), &c18_d0, 1, 0, 0U, 0, 0U, 0);
  c18_y = c18_d0;
  sf_mex_destroy(&c18_u);
  return c18_y;
}

static void c18_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c18_mxArrayInData, const char_T *c18_varName, void *c18_outData)
{
  const mxArray *c18_inRange;
  const char_T *c18_identifier;
  emlrtMsgIdentifier c18_thisId;
  real_T c18_y;
  SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c18_inRange = sf_mex_dup(c18_mxArrayInData);
  c18_identifier = c18_varName;
  c18_thisId.fIdentifier = c18_identifier;
  c18_thisId.fParent = NULL;
  c18_y = c18_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c18_inRange),
    &c18_thisId);
  sf_mex_destroy(&c18_inRange);
  *(real_T *)c18_outData = c18_y;
  sf_mex_destroy(&c18_mxArrayInData);
}

static const mxArray *c18_b_sf_marshallOut(void *chartInstanceVoid, void
  *c18_inData)
{
  const mxArray *c18_mxArrayOutData = NULL;
  int32_T c18_i13;
  real_T c18_b_inData[23];
  int32_T c18_i14;
  real_T c18_u[23];
  const mxArray *c18_y = NULL;
  SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c18_mxArrayOutData = NULL;
  for (c18_i13 = 0; c18_i13 < 23; c18_i13++) {
    c18_b_inData[c18_i13] = (*(real_T (*)[23])c18_inData)[c18_i13];
  }

  for (c18_i14 = 0; c18_i14 < 23; c18_i14++) {
    c18_u[c18_i14] = c18_b_inData[c18_i14];
  }

  c18_y = NULL;
  sf_mex_assign(&c18_y, sf_mex_create("y", c18_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c18_mxArrayOutData, c18_y, FALSE);
  return c18_mxArrayOutData;
}

static const mxArray *c18_c_sf_marshallOut(void *chartInstanceVoid, void
  *c18_inData)
{
  const mxArray *c18_mxArrayOutData = NULL;
  int32_T c18_i15;
  boolean_T c18_b_inData[23];
  int32_T c18_i16;
  boolean_T c18_u[23];
  const mxArray *c18_y = NULL;
  SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c18_mxArrayOutData = NULL;
  for (c18_i15 = 0; c18_i15 < 23; c18_i15++) {
    c18_b_inData[c18_i15] = (*(boolean_T (*)[23])c18_inData)[c18_i15];
  }

  for (c18_i16 = 0; c18_i16 < 23; c18_i16++) {
    c18_u[c18_i16] = c18_b_inData[c18_i16];
  }

  c18_y = NULL;
  sf_mex_assign(&c18_y, sf_mex_create("y", c18_u, 11, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c18_mxArrayOutData, c18_y, FALSE);
  return c18_mxArrayOutData;
}

static void c18_c_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_u, const emlrtMsgIdentifier *c18_parentId,
  boolean_T c18_y[23])
{
  boolean_T c18_bv1[23];
  int32_T c18_i17;
  sf_mex_import(c18_parentId, sf_mex_dup(c18_u), c18_bv1, 1, 11, 0U, 1, 0U, 1,
                23);
  for (c18_i17 = 0; c18_i17 < 23; c18_i17++) {
    c18_y[c18_i17] = c18_bv1[c18_i17];
  }

  sf_mex_destroy(&c18_u);
}

static void c18_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c18_mxArrayInData, const char_T *c18_varName, void *c18_outData)
{
  const mxArray *c18_res;
  const char_T *c18_identifier;
  emlrtMsgIdentifier c18_thisId;
  boolean_T c18_y[23];
  int32_T c18_i18;
  SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c18_res = sf_mex_dup(c18_mxArrayInData);
  c18_identifier = c18_varName;
  c18_thisId.fIdentifier = c18_identifier;
  c18_thisId.fParent = NULL;
  c18_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c18_res), &c18_thisId, c18_y);
  sf_mex_destroy(&c18_res);
  for (c18_i18 = 0; c18_i18 < 23; c18_i18++) {
    (*(boolean_T (*)[23])c18_outData)[c18_i18] = c18_y[c18_i18];
  }

  sf_mex_destroy(&c18_mxArrayInData);
}

const mxArray *sf_c18_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c18_nameCaptureInfo;
  c18_ResolvedFunctionInfo c18_info[8];
  c18_ResolvedFunctionInfo (*c18_b_info)[8];
  const mxArray *c18_m0 = NULL;
  int32_T c18_i19;
  c18_ResolvedFunctionInfo *c18_r0;
  c18_nameCaptureInfo = NULL;
  c18_nameCaptureInfo = NULL;
  c18_b_info = (c18_ResolvedFunctionInfo (*)[8])c18_info;
  (*c18_b_info)[0].context = "";
  (*c18_b_info)[0].name = "sum";
  (*c18_b_info)[0].dominantType = "logical";
  (*c18_b_info)[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c18_b_info)[0].fileTimeLo = 1314736612U;
  (*c18_b_info)[0].fileTimeHi = 0U;
  (*c18_b_info)[0].mFileTimeLo = 0U;
  (*c18_b_info)[0].mFileTimeHi = 0U;
  (*c18_b_info)[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c18_b_info)[1].name = "isequal";
  (*c18_b_info)[1].dominantType = "double";
  (*c18_b_info)[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  (*c18_b_info)[1].fileTimeLo = 1286818758U;
  (*c18_b_info)[1].fileTimeHi = 0U;
  (*c18_b_info)[1].mFileTimeLo = 0U;
  (*c18_b_info)[1].mFileTimeHi = 0U;
  (*c18_b_info)[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  (*c18_b_info)[2].name = "eml_isequal_core";
  (*c18_b_info)[2].dominantType = "double";
  (*c18_b_info)[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  (*c18_b_info)[2].fileTimeLo = 1286818786U;
  (*c18_b_info)[2].fileTimeHi = 0U;
  (*c18_b_info)[2].mFileTimeLo = 0U;
  (*c18_b_info)[2].mFileTimeHi = 0U;
  (*c18_b_info)[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c18_b_info)[3].name = "eml_const_nonsingleton_dim";
  (*c18_b_info)[3].dominantType = "logical";
  (*c18_b_info)[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  (*c18_b_info)[3].fileTimeLo = 1286818696U;
  (*c18_b_info)[3].fileTimeHi = 0U;
  (*c18_b_info)[3].mFileTimeLo = 0U;
  (*c18_b_info)[3].mFileTimeHi = 0U;
  (*c18_b_info)[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c18_b_info)[4].name = "eml_scalar_eg";
  (*c18_b_info)[4].dominantType = "double";
  (*c18_b_info)[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  (*c18_b_info)[4].fileTimeLo = 1286818796U;
  (*c18_b_info)[4].fileTimeHi = 0U;
  (*c18_b_info)[4].mFileTimeLo = 0U;
  (*c18_b_info)[4].mFileTimeHi = 0U;
  (*c18_b_info)[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c18_b_info)[5].name = "eml_index_class";
  (*c18_b_info)[5].dominantType = "";
  (*c18_b_info)[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c18_b_info)[5].fileTimeLo = 1323170578U;
  (*c18_b_info)[5].fileTimeHi = 0U;
  (*c18_b_info)[5].mFileTimeLo = 0U;
  (*c18_b_info)[5].mFileTimeHi = 0U;
  (*c18_b_info)[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  (*c18_b_info)[6].name = "eml_int_forloop_overflow_check";
  (*c18_b_info)[6].dominantType = "";
  (*c18_b_info)[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  (*c18_b_info)[6].fileTimeLo = 1346510340U;
  (*c18_b_info)[6].fileTimeHi = 0U;
  (*c18_b_info)[6].mFileTimeLo = 0U;
  (*c18_b_info)[6].mFileTimeHi = 0U;
  (*c18_b_info)[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  (*c18_b_info)[7].name = "intmax";
  (*c18_b_info)[7].dominantType = "char";
  (*c18_b_info)[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  (*c18_b_info)[7].fileTimeLo = 1311255316U;
  (*c18_b_info)[7].fileTimeHi = 0U;
  (*c18_b_info)[7].mFileTimeLo = 0U;
  (*c18_b_info)[7].mFileTimeHi = 0U;
  sf_mex_assign(&c18_m0, sf_mex_createstruct("nameCaptureInfo", 1, 8), FALSE);
  for (c18_i19 = 0; c18_i19 < 8; c18_i19++) {
    c18_r0 = &c18_info[c18_i19];
    sf_mex_addfield(c18_m0, sf_mex_create("nameCaptureInfo", c18_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c18_r0->context)), "context", "nameCaptureInfo",
                    c18_i19);
    sf_mex_addfield(c18_m0, sf_mex_create("nameCaptureInfo", c18_r0->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c18_r0->name)), "name", "nameCaptureInfo",
                    c18_i19);
    sf_mex_addfield(c18_m0, sf_mex_create("nameCaptureInfo",
      c18_r0->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c18_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c18_i19);
    sf_mex_addfield(c18_m0, sf_mex_create("nameCaptureInfo", c18_r0->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c18_r0->resolved)), "resolved",
                    "nameCaptureInfo", c18_i19);
    sf_mex_addfield(c18_m0, sf_mex_create("nameCaptureInfo", &c18_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c18_i19);
    sf_mex_addfield(c18_m0, sf_mex_create("nameCaptureInfo", &c18_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c18_i19);
    sf_mex_addfield(c18_m0, sf_mex_create("nameCaptureInfo",
      &c18_r0->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c18_i19);
    sf_mex_addfield(c18_m0, sf_mex_create("nameCaptureInfo",
      &c18_r0->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c18_i19);
  }

  sf_mex_assign(&c18_nameCaptureInfo, c18_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c18_nameCaptureInfo);
  return c18_nameCaptureInfo;
}

static const mxArray *c18_d_sf_marshallOut(void *chartInstanceVoid, void
  *c18_inData)
{
  const mxArray *c18_mxArrayOutData = NULL;
  int32_T c18_u;
  const mxArray *c18_y = NULL;
  SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c18_mxArrayOutData = NULL;
  c18_u = *(int32_T *)c18_inData;
  c18_y = NULL;
  sf_mex_assign(&c18_y, sf_mex_create("y", &c18_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c18_mxArrayOutData, c18_y, FALSE);
  return c18_mxArrayOutData;
}

static int32_T c18_d_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_u, const emlrtMsgIdentifier *c18_parentId)
{
  int32_T c18_y;
  int32_T c18_i20;
  sf_mex_import(c18_parentId, sf_mex_dup(c18_u), &c18_i20, 1, 6, 0U, 0, 0U, 0);
  c18_y = c18_i20;
  sf_mex_destroy(&c18_u);
  return c18_y;
}

static void c18_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c18_mxArrayInData, const char_T *c18_varName, void *c18_outData)
{
  const mxArray *c18_b_sfEvent;
  const char_T *c18_identifier;
  emlrtMsgIdentifier c18_thisId;
  int32_T c18_y;
  SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c18_b_sfEvent = sf_mex_dup(c18_mxArrayInData);
  c18_identifier = c18_varName;
  c18_thisId.fIdentifier = c18_identifier;
  c18_thisId.fParent = NULL;
  c18_y = c18_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c18_b_sfEvent),
    &c18_thisId);
  sf_mex_destroy(&c18_b_sfEvent);
  *(int32_T *)c18_outData = c18_y;
  sf_mex_destroy(&c18_mxArrayInData);
}

static uint8_T c18_e_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_b_is_active_c18_torqueBalancing2012b, const
  char_T *c18_identifier)
{
  uint8_T c18_y;
  emlrtMsgIdentifier c18_thisId;
  c18_thisId.fIdentifier = c18_identifier;
  c18_thisId.fParent = NULL;
  c18_y = c18_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c18_b_is_active_c18_torqueBalancing2012b), &c18_thisId);
  sf_mex_destroy(&c18_b_is_active_c18_torqueBalancing2012b);
  return c18_y;
}

static uint8_T c18_f_emlrt_marshallIn(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c18_u, const emlrtMsgIdentifier *c18_parentId)
{
  uint8_T c18_y;
  uint8_T c18_u0;
  sf_mex_import(c18_parentId, sf_mex_dup(c18_u), &c18_u0, 1, 3, 0U, 0, 0U, 0);
  c18_y = c18_u0;
  sf_mex_destroy(&c18_u);
  return c18_y;
}

static void init_dsm_address_info(SFc18_torqueBalancing2012bInstanceStruct
  *chartInstance)
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

void sf_c18_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1593199028U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2747650279U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1260703810U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1269108278U);
}

mxArray *sf_c18_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("OXq7VKrcGTj6ZpLjjYcL4G");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
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
      pr[0] = (double)(1);
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

mxArray *sf_c18_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c18_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[7],T\"inRange\",},{M[8],M[0],T\"is_active_c18_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c18_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           18,
           1,
           1,
           5,
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
          init_script_number_translation(_torqueBalancing2012bMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_torqueBalancing2012bMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _torqueBalancing2012bMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"umin");
          _SFD_SET_DATA_PROPS(1,1,1,0,"umax");
          _SFD_SET_DATA_PROPS(2,1,1,0,"u");
          _SFD_SET_DATA_PROPS(3,2,0,1,"inRange");
          _SFD_SET_DATA_PROPS(4,1,1,0,"tol");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,204);
        _SFD_CV_INIT_EML_IF(0,1,0,116,125,143,204);
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
            1.0,0,0,(MexFcnForType)c18_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c18_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c18_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c18_sf_marshallOut,(MexInFcnForType)c18_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c18_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c18_inRange;
          real_T *c18_tol;
          real_T (*c18_umin)[23];
          real_T (*c18_umax)[23];
          real_T (*c18_u)[23];
          c18_tol = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c18_inRange = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c18_u = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 2);
          c18_umax = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 1);
          c18_umin = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c18_umin);
          _SFD_SET_DATA_VALUE_PTR(1U, *c18_umax);
          _SFD_SET_DATA_VALUE_PTR(2U, *c18_u);
          _SFD_SET_DATA_VALUE_PTR(3U, c18_inRange);
          _SFD_SET_DATA_VALUE_PTR(4U, c18_tol);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _torqueBalancing2012bMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "ceCZAQV4KdwJlTetl0KEwH";
}

static void sf_opaque_initialize_c18_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc18_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c18_torqueBalancing2012b
    ((SFc18_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c18_torqueBalancing2012b((SFc18_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c18_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c18_torqueBalancing2012b((SFc18_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c18_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c18_torqueBalancing2012b((SFc18_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c18_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c18_torqueBalancing2012b((SFc18_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c18_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c18_torqueBalancing2012b
    ((SFc18_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c18_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c18_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c18_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c18_torqueBalancing2012b
    ((SFc18_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c18_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c18_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c18_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c18_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c18_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc18_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c18_torqueBalancing2012b((SFc18_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc18_torqueBalancing2012b
    ((SFc18_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c18_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c18_torqueBalancing2012b
      ((SFc18_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c18_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      18);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,18,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,18,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,18);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,18,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,18,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,18);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(736224043U));
  ssSetChecksum1(S,(2275710665U));
  ssSetChecksum2(S,(2557776686U));
  ssSetChecksum3(S,(196935047U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c18_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c18_torqueBalancing2012b(SimStruct *S)
{
  SFc18_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc18_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc18_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc18_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c18_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c18_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c18_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c18_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c18_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c18_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c18_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c18_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c18_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c18_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c18_torqueBalancing2012b;
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

void c18_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c18_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c18_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c18_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c18_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
