/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c25_torqueBalancing2012b.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "torqueBalancing2012b_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c25_debug_family_names[4] = { "nargin", "nargout", "R",
  "rollPitchYaw" };

static const char * c25_b_debug_family_names[4] = { "nargin", "nargout", "R",
  "rollPitchYaw" };

/* Function Declarations */
static void initialize_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static void c25_update_debugger_state_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c25_st);
static void finalize_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c25_torqueBalancing2012b(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c25_machineNumber, uint32_T
  c25_chartNumber);
static const mxArray *c25_sf_marshallOut(void *chartInstanceVoid, void
  *c25_inData);
static void c25_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_rollPitchYaw, const char_T *c25_identifier,
  real_T c25_y[3]);
static void c25_b_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId,
  real_T c25_y[3]);
static void c25_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c25_mxArrayInData, const char_T *c25_varName, void *c25_outData);
static const mxArray *c25_b_sf_marshallOut(void *chartInstanceVoid, void
  *c25_inData);
static const mxArray *c25_c_sf_marshallOut(void *chartInstanceVoid, void
  *c25_inData);
static real_T c25_c_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId);
static void c25_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c25_mxArrayInData, const char_T *c25_varName, void *c25_outData);
static void c25_d_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId,
  real_T c25_y[9]);
static void c25_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c25_mxArrayInData, const char_T *c25_varName, void *c25_outData);
static void c25_info_helper(c25_ResolvedFunctionInfo c25_info[13]);
static void c25_eml_error(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance);
static real_T c25_atan2(SFc25_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c25_y, real_T c25_x);
static const mxArray *c25_d_sf_marshallOut(void *chartInstanceVoid, void
  *c25_inData);
static int32_T c25_e_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId);
static void c25_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c25_mxArrayInData, const char_T *c25_varName, void *c25_outData);
static uint8_T c25_f_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_b_is_active_c25_torqueBalancing2012b, const
  char_T *c25_identifier);
static uint8_T c25_g_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId);
static void init_dsm_address_info(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c25_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c25_is_active_c25_torqueBalancing2012b = 0U;
}

static void initialize_params_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void enable_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c25_update_debugger_state_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c25_st;
  const mxArray *c25_y = NULL;
  int32_T c25_i0;
  real_T c25_u[3];
  const mxArray *c25_b_y = NULL;
  uint8_T c25_hoistedGlobal;
  uint8_T c25_b_u;
  const mxArray *c25_c_y = NULL;
  real_T (*c25_rollPitchYaw)[3];
  c25_rollPitchYaw = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c25_st = NULL;
  c25_st = NULL;
  c25_y = NULL;
  sf_mex_assign(&c25_y, sf_mex_createcellarray(2), FALSE);
  for (c25_i0 = 0; c25_i0 < 3; c25_i0++) {
    c25_u[c25_i0] = (*c25_rollPitchYaw)[c25_i0];
  }

  c25_b_y = NULL;
  sf_mex_assign(&c25_b_y, sf_mex_create("y", c25_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c25_y, 0, c25_b_y);
  c25_hoistedGlobal = chartInstance->c25_is_active_c25_torqueBalancing2012b;
  c25_b_u = c25_hoistedGlobal;
  c25_c_y = NULL;
  sf_mex_assign(&c25_c_y, sf_mex_create("y", &c25_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c25_y, 1, c25_c_y);
  sf_mex_assign(&c25_st, c25_y, FALSE);
  return c25_st;
}

static void set_sim_state_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c25_st)
{
  const mxArray *c25_u;
  real_T c25_dv0[3];
  int32_T c25_i1;
  real_T (*c25_rollPitchYaw)[3];
  c25_rollPitchYaw = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c25_doneDoubleBufferReInit = TRUE;
  c25_u = sf_mex_dup(c25_st);
  c25_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c25_u, 0)),
                       "rollPitchYaw", c25_dv0);
  for (c25_i1 = 0; c25_i1 < 3; c25_i1++) {
    (*c25_rollPitchYaw)[c25_i1] = c25_dv0[c25_i1];
  }

  chartInstance->c25_is_active_c25_torqueBalancing2012b = c25_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c25_u, 1)),
     "is_active_c25_torqueBalancing2012b");
  sf_mex_destroy(&c25_u);
  c25_update_debugger_state_c25_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c25_st);
}

static void finalize_c25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c25_torqueBalancing2012b(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c25_i2;
  int32_T c25_i3;
  int32_T c25_i4;
  real_T c25_R[9];
  uint32_T c25_debug_family_var_map[4];
  real_T c25_nargin = 1.0;
  real_T c25_nargout = 1.0;
  real_T c25_rollPitchYaw[3];
  int32_T c25_i5;
  real_T c25_b_R[9];
  real_T c25_b_nargin = 1.0;
  real_T c25_b_nargout = 1.0;
  int32_T c25_i6;
  real_T c25_x;
  real_T c25_b_x;
  int32_T c25_i7;
  real_T c25_a[3];
  int32_T c25_i8;
  int32_T c25_i9;
  int32_T c25_i10;
  real_T (*c25_b_rollPitchYaw)[3];
  real_T (*c25_c_R)[9];
  boolean_T guard1 = FALSE;
  c25_b_rollPitchYaw = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c25_c_R = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 24U, chartInstance->c25_sfEvent);
  for (c25_i2 = 0; c25_i2 < 9; c25_i2++) {
    _SFD_DATA_RANGE_CHECK((*c25_c_R)[c25_i2], 0U);
  }

  for (c25_i3 = 0; c25_i3 < 3; c25_i3++) {
    _SFD_DATA_RANGE_CHECK((*c25_b_rollPitchYaw)[c25_i3], 1U);
  }

  chartInstance->c25_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 24U, chartInstance->c25_sfEvent);
  for (c25_i4 = 0; c25_i4 < 9; c25_i4++) {
    c25_R[c25_i4] = (*c25_c_R)[c25_i4];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c25_debug_family_names,
    c25_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c25_nargin, 0U, c25_c_sf_marshallOut,
    c25_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c25_nargout, 1U, c25_c_sf_marshallOut,
    c25_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c25_R, 2U, c25_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c25_rollPitchYaw, 3U, c25_sf_marshallOut,
    c25_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c25_sfEvent, 4);
  for (c25_i5 = 0; c25_i5 < 9; c25_i5++) {
    c25_b_R[c25_i5] = c25_R[c25_i5];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c25_b_debug_family_names,
    c25_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c25_b_nargin, 0U, c25_c_sf_marshallOut,
    c25_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c25_b_nargout, 1U, c25_c_sf_marshallOut,
    c25_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c25_b_R, 2U, c25_b_sf_marshallOut,
    c25_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c25_rollPitchYaw, 3U, c25_sf_marshallOut,
    c25_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 4);
  for (c25_i6 = 0; c25_i6 < 3; c25_i6++) {
    c25_rollPitchYaw[c25_i6] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 5);
  if (CV_SCRIPT_IF(0, 0, c25_b_R[2] < 1.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 6);
    if (CV_SCRIPT_IF(0, 1, c25_b_R[2] > -1.0)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 7);
      c25_x = -c25_b_R[2];
      c25_b_x = c25_x;
      guard1 = FALSE;
      if (c25_b_x < -1.0) {
        guard1 = TRUE;
      } else {
        if (1.0 < c25_b_x) {
          guard1 = TRUE;
        }
      }

      if (guard1 == TRUE) {
        c25_eml_error(chartInstance);
      }

      c25_b_x = muDoubleScalarAsin(c25_b_x);
      c25_rollPitchYaw[1] = c25_b_x;
      _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 8);
      c25_rollPitchYaw[2] = c25_atan2(chartInstance, c25_b_R[1], c25_b_R[0]);
      _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 9);
      c25_rollPitchYaw[0] = c25_atan2(chartInstance, c25_b_R[5], c25_b_R[8]);
    } else {
      _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 11);
      c25_rollPitchYaw[2] = -c25_atan2(chartInstance, -c25_b_R[7], c25_b_R[4]);
      _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 12);
      c25_rollPitchYaw[0] = 0.0;
    }
  } else {
    _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 15);
    c25_rollPitchYaw[2] = c25_atan2(chartInstance, -c25_b_R[7], c25_b_R[4]);
    _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, 16);
    c25_rollPitchYaw[0] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c25_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c25_sfEvent, 6);
  for (c25_i7 = 0; c25_i7 < 3; c25_i7++) {
    c25_a[c25_i7] = c25_rollPitchYaw[c25_i7];
  }

  for (c25_i8 = 0; c25_i8 < 3; c25_i8++) {
    c25_a[c25_i8] *= 180.0;
  }

  for (c25_i9 = 0; c25_i9 < 3; c25_i9++) {
    c25_rollPitchYaw[c25_i9] = c25_a[c25_i9] / 3.1415926535897931;
  }

  _SFD_EML_CALL(0U, chartInstance->c25_sfEvent, -6);
  _SFD_SYMBOL_SCOPE_POP();
  for (c25_i10 = 0; c25_i10 < 3; c25_i10++) {
    (*c25_b_rollPitchYaw)[c25_i10] = c25_rollPitchYaw[c25_i10];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 24U, chartInstance->c25_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc25_torqueBalancing2012b
  (SFc25_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c25_machineNumber, uint32_T
  c25_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c25_chartNumber, 0U, sf_debug_get_script_id(
    "/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m"));
}

static const mxArray *c25_sf_marshallOut(void *chartInstanceVoid, void
  *c25_inData)
{
  const mxArray *c25_mxArrayOutData = NULL;
  int32_T c25_i11;
  real_T c25_b_inData[3];
  int32_T c25_i12;
  real_T c25_u[3];
  const mxArray *c25_y = NULL;
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c25_mxArrayOutData = NULL;
  for (c25_i11 = 0; c25_i11 < 3; c25_i11++) {
    c25_b_inData[c25_i11] = (*(real_T (*)[3])c25_inData)[c25_i11];
  }

  for (c25_i12 = 0; c25_i12 < 3; c25_i12++) {
    c25_u[c25_i12] = c25_b_inData[c25_i12];
  }

  c25_y = NULL;
  sf_mex_assign(&c25_y, sf_mex_create("y", c25_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c25_mxArrayOutData, c25_y, FALSE);
  return c25_mxArrayOutData;
}

static void c25_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_rollPitchYaw, const char_T *c25_identifier,
  real_T c25_y[3])
{
  emlrtMsgIdentifier c25_thisId;
  c25_thisId.fIdentifier = c25_identifier;
  c25_thisId.fParent = NULL;
  c25_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c25_rollPitchYaw),
    &c25_thisId, c25_y);
  sf_mex_destroy(&c25_rollPitchYaw);
}

static void c25_b_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId,
  real_T c25_y[3])
{
  real_T c25_dv1[3];
  int32_T c25_i13;
  sf_mex_import(c25_parentId, sf_mex_dup(c25_u), c25_dv1, 1, 0, 0U, 1, 0U, 1, 3);
  for (c25_i13 = 0; c25_i13 < 3; c25_i13++) {
    c25_y[c25_i13] = c25_dv1[c25_i13];
  }

  sf_mex_destroy(&c25_u);
}

static void c25_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c25_mxArrayInData, const char_T *c25_varName, void *c25_outData)
{
  const mxArray *c25_rollPitchYaw;
  const char_T *c25_identifier;
  emlrtMsgIdentifier c25_thisId;
  real_T c25_y[3];
  int32_T c25_i14;
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c25_rollPitchYaw = sf_mex_dup(c25_mxArrayInData);
  c25_identifier = c25_varName;
  c25_thisId.fIdentifier = c25_identifier;
  c25_thisId.fParent = NULL;
  c25_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c25_rollPitchYaw),
    &c25_thisId, c25_y);
  sf_mex_destroy(&c25_rollPitchYaw);
  for (c25_i14 = 0; c25_i14 < 3; c25_i14++) {
    (*(real_T (*)[3])c25_outData)[c25_i14] = c25_y[c25_i14];
  }

  sf_mex_destroy(&c25_mxArrayInData);
}

static const mxArray *c25_b_sf_marshallOut(void *chartInstanceVoid, void
  *c25_inData)
{
  const mxArray *c25_mxArrayOutData = NULL;
  int32_T c25_i15;
  int32_T c25_i16;
  int32_T c25_i17;
  real_T c25_b_inData[9];
  int32_T c25_i18;
  int32_T c25_i19;
  int32_T c25_i20;
  real_T c25_u[9];
  const mxArray *c25_y = NULL;
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c25_mxArrayOutData = NULL;
  c25_i15 = 0;
  for (c25_i16 = 0; c25_i16 < 3; c25_i16++) {
    for (c25_i17 = 0; c25_i17 < 3; c25_i17++) {
      c25_b_inData[c25_i17 + c25_i15] = (*(real_T (*)[9])c25_inData)[c25_i17 +
        c25_i15];
    }

    c25_i15 += 3;
  }

  c25_i18 = 0;
  for (c25_i19 = 0; c25_i19 < 3; c25_i19++) {
    for (c25_i20 = 0; c25_i20 < 3; c25_i20++) {
      c25_u[c25_i20 + c25_i18] = c25_b_inData[c25_i20 + c25_i18];
    }

    c25_i18 += 3;
  }

  c25_y = NULL;
  sf_mex_assign(&c25_y, sf_mex_create("y", c25_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c25_mxArrayOutData, c25_y, FALSE);
  return c25_mxArrayOutData;
}

static const mxArray *c25_c_sf_marshallOut(void *chartInstanceVoid, void
  *c25_inData)
{
  const mxArray *c25_mxArrayOutData = NULL;
  real_T c25_u;
  const mxArray *c25_y = NULL;
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c25_mxArrayOutData = NULL;
  c25_u = *(real_T *)c25_inData;
  c25_y = NULL;
  sf_mex_assign(&c25_y, sf_mex_create("y", &c25_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c25_mxArrayOutData, c25_y, FALSE);
  return c25_mxArrayOutData;
}

static real_T c25_c_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId)
{
  real_T c25_y;
  real_T c25_d0;
  sf_mex_import(c25_parentId, sf_mex_dup(c25_u), &c25_d0, 1, 0, 0U, 0, 0U, 0);
  c25_y = c25_d0;
  sf_mex_destroy(&c25_u);
  return c25_y;
}

static void c25_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c25_mxArrayInData, const char_T *c25_varName, void *c25_outData)
{
  const mxArray *c25_nargout;
  const char_T *c25_identifier;
  emlrtMsgIdentifier c25_thisId;
  real_T c25_y;
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c25_nargout = sf_mex_dup(c25_mxArrayInData);
  c25_identifier = c25_varName;
  c25_thisId.fIdentifier = c25_identifier;
  c25_thisId.fParent = NULL;
  c25_y = c25_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c25_nargout),
    &c25_thisId);
  sf_mex_destroy(&c25_nargout);
  *(real_T *)c25_outData = c25_y;
  sf_mex_destroy(&c25_mxArrayInData);
}

static void c25_d_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId,
  real_T c25_y[9])
{
  real_T c25_dv2[9];
  int32_T c25_i21;
  sf_mex_import(c25_parentId, sf_mex_dup(c25_u), c25_dv2, 1, 0, 0U, 1, 0U, 2, 3,
                3);
  for (c25_i21 = 0; c25_i21 < 9; c25_i21++) {
    c25_y[c25_i21] = c25_dv2[c25_i21];
  }

  sf_mex_destroy(&c25_u);
}

static void c25_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c25_mxArrayInData, const char_T *c25_varName, void *c25_outData)
{
  const mxArray *c25_R;
  const char_T *c25_identifier;
  emlrtMsgIdentifier c25_thisId;
  real_T c25_y[9];
  int32_T c25_i22;
  int32_T c25_i23;
  int32_T c25_i24;
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c25_R = sf_mex_dup(c25_mxArrayInData);
  c25_identifier = c25_varName;
  c25_thisId.fIdentifier = c25_identifier;
  c25_thisId.fParent = NULL;
  c25_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c25_R), &c25_thisId, c25_y);
  sf_mex_destroy(&c25_R);
  c25_i22 = 0;
  for (c25_i23 = 0; c25_i23 < 3; c25_i23++) {
    for (c25_i24 = 0; c25_i24 < 3; c25_i24++) {
      (*(real_T (*)[9])c25_outData)[c25_i24 + c25_i22] = c25_y[c25_i24 + c25_i22];
    }

    c25_i22 += 3;
  }

  sf_mex_destroy(&c25_mxArrayInData);
}

const mxArray *sf_c25_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c25_nameCaptureInfo;
  c25_ResolvedFunctionInfo c25_info[13];
  const mxArray *c25_m0 = NULL;
  int32_T c25_i25;
  c25_ResolvedFunctionInfo *c25_r0;
  c25_nameCaptureInfo = NULL;
  c25_nameCaptureInfo = NULL;
  c25_info_helper(c25_info);
  sf_mex_assign(&c25_m0, sf_mex_createstruct("nameCaptureInfo", 1, 13), FALSE);
  for (c25_i25 = 0; c25_i25 < 13; c25_i25++) {
    c25_r0 = &c25_info[c25_i25];
    sf_mex_addfield(c25_m0, sf_mex_create("nameCaptureInfo", c25_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c25_r0->context)), "context", "nameCaptureInfo",
                    c25_i25);
    sf_mex_addfield(c25_m0, sf_mex_create("nameCaptureInfo", c25_r0->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c25_r0->name)), "name", "nameCaptureInfo",
                    c25_i25);
    sf_mex_addfield(c25_m0, sf_mex_create("nameCaptureInfo",
      c25_r0->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c25_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c25_i25);
    sf_mex_addfield(c25_m0, sf_mex_create("nameCaptureInfo", c25_r0->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c25_r0->resolved)), "resolved",
                    "nameCaptureInfo", c25_i25);
    sf_mex_addfield(c25_m0, sf_mex_create("nameCaptureInfo", &c25_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c25_i25);
    sf_mex_addfield(c25_m0, sf_mex_create("nameCaptureInfo", &c25_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c25_i25);
    sf_mex_addfield(c25_m0, sf_mex_create("nameCaptureInfo",
      &c25_r0->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c25_i25);
    sf_mex_addfield(c25_m0, sf_mex_create("nameCaptureInfo",
      &c25_r0->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c25_i25);
  }

  sf_mex_assign(&c25_nameCaptureInfo, c25_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c25_nameCaptureInfo);
  return c25_nameCaptureInfo;
}

static void c25_info_helper(c25_ResolvedFunctionInfo c25_info[13])
{
  c25_info[0].context = "";
  c25_info[0].name = "rollPitchYawFromRotation";
  c25_info[0].dominantType = "double";
  c25_info[0].resolved =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c25_info[0].fileTimeLo = 1495096797U;
  c25_info[0].fileTimeHi = 0U;
  c25_info[0].mFileTimeLo = 0U;
  c25_info[0].mFileTimeHi = 0U;
  c25_info[1].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c25_info[1].name = "asin";
  c25_info[1].dominantType = "double";
  c25_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c25_info[1].fileTimeLo = 1343830370U;
  c25_info[1].fileTimeHi = 0U;
  c25_info[1].mFileTimeLo = 0U;
  c25_info[1].mFileTimeHi = 0U;
  c25_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c25_info[2].name = "eml_error";
  c25_info[2].dominantType = "char";
  c25_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c25_info[2].fileTimeLo = 1343830358U;
  c25_info[2].fileTimeHi = 0U;
  c25_info[2].mFileTimeLo = 0U;
  c25_info[2].mFileTimeHi = 0U;
  c25_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c25_info[3].name = "eml_scalar_asin";
  c25_info[3].dominantType = "double";
  c25_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_asin.m";
  c25_info[3].fileTimeLo = 1343830376U;
  c25_info[3].fileTimeHi = 0U;
  c25_info[3].mFileTimeLo = 0U;
  c25_info[3].mFileTimeHi = 0U;
  c25_info[4].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c25_info[4].name = "atan2";
  c25_info[4].dominantType = "double";
  c25_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c25_info[4].fileTimeLo = 1343830372U;
  c25_info[4].fileTimeHi = 0U;
  c25_info[4].mFileTimeLo = 0U;
  c25_info[4].mFileTimeHi = 0U;
  c25_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c25_info[5].name = "eml_scalar_eg";
  c25_info[5].dominantType = "double";
  c25_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c25_info[5].fileTimeLo = 1286818796U;
  c25_info[5].fileTimeHi = 0U;
  c25_info[5].mFileTimeLo = 0U;
  c25_info[5].mFileTimeHi = 0U;
  c25_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c25_info[6].name = "eml_scalexp_alloc";
  c25_info[6].dominantType = "double";
  c25_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c25_info[6].fileTimeLo = 1352424860U;
  c25_info[6].fileTimeHi = 0U;
  c25_info[6].mFileTimeLo = 0U;
  c25_info[6].mFileTimeHi = 0U;
  c25_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c25_info[7].name = "eml_scalar_atan2";
  c25_info[7].dominantType = "double";
  c25_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m";
  c25_info[7].fileTimeLo = 1286818720U;
  c25_info[7].fileTimeHi = 0U;
  c25_info[7].mFileTimeLo = 0U;
  c25_info[7].mFileTimeHi = 0U;
  c25_info[8].context = "";
  c25_info[8].name = "mtimes";
  c25_info[8].dominantType = "double";
  c25_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c25_info[8].fileTimeLo = 1289519692U;
  c25_info[8].fileTimeHi = 0U;
  c25_info[8].mFileTimeLo = 0U;
  c25_info[8].mFileTimeHi = 0U;
  c25_info[9].context = "";
  c25_info[9].name = "mrdivide";
  c25_info[9].dominantType = "double";
  c25_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c25_info[9].fileTimeLo = 1357951548U;
  c25_info[9].fileTimeHi = 0U;
  c25_info[9].mFileTimeLo = 1319729966U;
  c25_info[9].mFileTimeHi = 0U;
  c25_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c25_info[10].name = "rdivide";
  c25_info[10].dominantType = "double";
  c25_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c25_info[10].fileTimeLo = 1346510388U;
  c25_info[10].fileTimeHi = 0U;
  c25_info[10].mFileTimeLo = 0U;
  c25_info[10].mFileTimeHi = 0U;
  c25_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c25_info[11].name = "eml_scalexp_compatible";
  c25_info[11].dominantType = "double";
  c25_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c25_info[11].fileTimeLo = 1286818796U;
  c25_info[11].fileTimeHi = 0U;
  c25_info[11].mFileTimeLo = 0U;
  c25_info[11].mFileTimeHi = 0U;
  c25_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c25_info[12].name = "eml_div";
  c25_info[12].dominantType = "double";
  c25_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c25_info[12].fileTimeLo = 1313347810U;
  c25_info[12].fileTimeHi = 0U;
  c25_info[12].mFileTimeLo = 0U;
  c25_info[12].mFileTimeHi = 0U;
}

static void c25_eml_error(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c25_i26;
  static char_T c25_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c25_u[30];
  const mxArray *c25_y = NULL;
  int32_T c25_i27;
  static char_T c25_cv1[4] = { 'a', 's', 'i', 'n' };

  char_T c25_b_u[4];
  const mxArray *c25_b_y = NULL;
  for (c25_i26 = 0; c25_i26 < 30; c25_i26++) {
    c25_u[c25_i26] = c25_cv0[c25_i26];
  }

  c25_y = NULL;
  sf_mex_assign(&c25_y, sf_mex_create("y", c25_u, 10, 0U, 1U, 0U, 2, 1, 30),
                FALSE);
  for (c25_i27 = 0; c25_i27 < 4; c25_i27++) {
    c25_b_u[c25_i27] = c25_cv1[c25_i27];
  }

  c25_b_y = NULL;
  sf_mex_assign(&c25_b_y, sf_mex_create("y", c25_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c25_y, 14, c25_b_y));
}

static real_T c25_atan2(SFc25_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c25_y, real_T c25_x)
{
  real_T c25_b_y;
  real_T c25_b_x;
  c25_b_y = c25_y;
  c25_b_x = c25_x;
  return muDoubleScalarAtan2(c25_b_y, c25_b_x);
}

static const mxArray *c25_d_sf_marshallOut(void *chartInstanceVoid, void
  *c25_inData)
{
  const mxArray *c25_mxArrayOutData = NULL;
  int32_T c25_u;
  const mxArray *c25_y = NULL;
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c25_mxArrayOutData = NULL;
  c25_u = *(int32_T *)c25_inData;
  c25_y = NULL;
  sf_mex_assign(&c25_y, sf_mex_create("y", &c25_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c25_mxArrayOutData, c25_y, FALSE);
  return c25_mxArrayOutData;
}

static int32_T c25_e_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId)
{
  int32_T c25_y;
  int32_T c25_i28;
  sf_mex_import(c25_parentId, sf_mex_dup(c25_u), &c25_i28, 1, 6, 0U, 0, 0U, 0);
  c25_y = c25_i28;
  sf_mex_destroy(&c25_u);
  return c25_y;
}

static void c25_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c25_mxArrayInData, const char_T *c25_varName, void *c25_outData)
{
  const mxArray *c25_b_sfEvent;
  const char_T *c25_identifier;
  emlrtMsgIdentifier c25_thisId;
  int32_T c25_y;
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c25_b_sfEvent = sf_mex_dup(c25_mxArrayInData);
  c25_identifier = c25_varName;
  c25_thisId.fIdentifier = c25_identifier;
  c25_thisId.fParent = NULL;
  c25_y = c25_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c25_b_sfEvent),
    &c25_thisId);
  sf_mex_destroy(&c25_b_sfEvent);
  *(int32_T *)c25_outData = c25_y;
  sf_mex_destroy(&c25_mxArrayInData);
}

static uint8_T c25_f_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_b_is_active_c25_torqueBalancing2012b, const
  char_T *c25_identifier)
{
  uint8_T c25_y;
  emlrtMsgIdentifier c25_thisId;
  c25_thisId.fIdentifier = c25_identifier;
  c25_thisId.fParent = NULL;
  c25_y = c25_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c25_b_is_active_c25_torqueBalancing2012b), &c25_thisId);
  sf_mex_destroy(&c25_b_is_active_c25_torqueBalancing2012b);
  return c25_y;
}

static uint8_T c25_g_emlrt_marshallIn(SFc25_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c25_u, const emlrtMsgIdentifier *c25_parentId)
{
  uint8_T c25_y;
  uint8_T c25_u0;
  sf_mex_import(c25_parentId, sf_mex_dup(c25_u), &c25_u0, 1, 3, 0U, 0, 0U, 0);
  c25_y = c25_u0;
  sf_mex_destroy(&c25_u);
  return c25_y;
}

static void init_dsm_address_info(SFc25_torqueBalancing2012bInstanceStruct
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

void sf_c25_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2810076537U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1244322991U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1086178426U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2796252950U);
}

mxArray *sf_c25_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("9B1P5X3qhYr65sYSis5Tw");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
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
      pr[0] = (double)(3);
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

mxArray *sf_c25_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c25_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[7],T\"rollPitchYaw\",},{M[8],M[0],T\"is_active_c25_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c25_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           25,
           1,
           1,
           2,
           0,
           0,
           0,
           0,
           1,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"R");
          _SFD_SET_DATA_PROPS(1,2,0,1,"rollPitchYaw");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,147);
        _SFD_CV_INIT_SCRIPT(0,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"rollPitchYawFromRotation",11,-1,510);
        _SFD_CV_INIT_SCRIPT_IF(0,0,157,173,432,510);
        _SFD_CV_INIT_SCRIPT_IF(0,1,179,195,341,431);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c25_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c25_sf_marshallOut,(MexInFcnForType)
            c25_sf_marshallIn);
        }

        {
          real_T (*c25_R)[9];
          real_T (*c25_rollPitchYaw)[3];
          c25_rollPitchYaw = (real_T (*)[3])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c25_R = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c25_R);
          _SFD_SET_DATA_VALUE_PTR(1U, *c25_rollPitchYaw);
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
  return "UFHwbqPs0saPLmVxuEEcbF";
}

static void sf_opaque_initialize_c25_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc25_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c25_torqueBalancing2012b
    ((SFc25_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c25_torqueBalancing2012b((SFc25_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c25_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c25_torqueBalancing2012b((SFc25_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c25_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c25_torqueBalancing2012b((SFc25_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c25_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c25_torqueBalancing2012b((SFc25_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c25_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c25_torqueBalancing2012b
    ((SFc25_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c25_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c25_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c25_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c25_torqueBalancing2012b
    ((SFc25_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c25_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c25_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c25_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c25_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c25_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc25_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c25_torqueBalancing2012b((SFc25_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc25_torqueBalancing2012b
    ((SFc25_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c25_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c25_torqueBalancing2012b
      ((SFc25_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c25_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      25);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,25,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,25,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,25);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,25,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,25,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 1; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,25);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1762239779U));
  ssSetChecksum1(S,(1753739080U));
  ssSetChecksum2(S,(1150659335U));
  ssSetChecksum3(S,(460100820U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c25_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c25_torqueBalancing2012b(SimStruct *S)
{
  SFc25_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc25_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc25_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc25_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c25_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c25_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c25_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c25_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c25_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c25_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c25_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c25_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c25_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c25_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c25_torqueBalancing2012b;
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

void c25_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c25_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c25_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c25_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c25_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
