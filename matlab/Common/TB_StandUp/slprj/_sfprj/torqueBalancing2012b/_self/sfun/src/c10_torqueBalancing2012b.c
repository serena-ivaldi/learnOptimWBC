/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c10_torqueBalancing2012b.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "torqueBalancing2012b_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c10_debug_family_names[9] = { "Lcop", "Rcop", "nargin",
  "nargout", "fz_left", "fz_right", "Lcontact", "Rcontact", "zmp" };

/* Function Declarations */
static void initialize_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void c10_update_debugger_state_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c10_st);
static void finalize_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c10_torqueBalancing2012b(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c10_chartstep_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void initSimStructsc10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c10_machineNumber, uint32_T
  c10_chartNumber);
static const mxArray *c10_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static void c10_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_zmp, const char_T *c10_identifier, real_T
  c10_y[2]);
static void c10_b_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[2]);
static void c10_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static const mxArray *c10_b_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static const mxArray *c10_c_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static const mxArray *c10_d_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static real_T c10_c_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static void c10_info_helper(c10_ResolvedFunctionInfo c10_info[13]);
static void c10_eml_scalar_eg(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c10_e_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static int32_T c10_d_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static uint8_T c10_e_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_b_is_active_c10_torqueBalancing2012b, const
  char_T *c10_identifier);
static uint8_T c10_f_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void init_dsm_address_info(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c10_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c10_is_active_c10_torqueBalancing2012b = 0U;
}

static void initialize_params_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void enable_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c10_update_debugger_state_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c10_st;
  const mxArray *c10_y = NULL;
  int32_T c10_i0;
  real_T c10_u[2];
  const mxArray *c10_b_y = NULL;
  uint8_T c10_hoistedGlobal;
  uint8_T c10_b_u;
  const mxArray *c10_c_y = NULL;
  real_T (*c10_zmp)[2];
  c10_zmp = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c10_st = NULL;
  c10_st = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_createcellarray(2), FALSE);
  for (c10_i0 = 0; c10_i0 < 2; c10_i0++) {
    c10_u[c10_i0] = (*c10_zmp)[c10_i0];
  }

  c10_b_y = NULL;
  sf_mex_assign(&c10_b_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_setcell(c10_y, 0, c10_b_y);
  c10_hoistedGlobal = chartInstance->c10_is_active_c10_torqueBalancing2012b;
  c10_b_u = c10_hoistedGlobal;
  c10_c_y = NULL;
  sf_mex_assign(&c10_c_y, sf_mex_create("y", &c10_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c10_y, 1, c10_c_y);
  sf_mex_assign(&c10_st, c10_y, FALSE);
  return c10_st;
}

static void set_sim_state_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c10_st)
{
  const mxArray *c10_u;
  real_T c10_dv0[2];
  int32_T c10_i1;
  real_T (*c10_zmp)[2];
  c10_zmp = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c10_doneDoubleBufferReInit = TRUE;
  c10_u = sf_mex_dup(c10_st);
  c10_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 0)),
                       "zmp", c10_dv0);
  for (c10_i1 = 0; c10_i1 < 2; c10_i1++) {
    (*c10_zmp)[c10_i1] = c10_dv0[c10_i1];
  }

  chartInstance->c10_is_active_c10_torqueBalancing2012b = c10_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c10_u, 1)),
     "is_active_c10_torqueBalancing2012b");
  sf_mex_destroy(&c10_u);
  c10_update_debugger_state_c10_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c10_st);
}

static void finalize_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c10_torqueBalancing2012b(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c10_i2;
  int32_T c10_i3;
  int32_T c10_i4;
  int32_T c10_i5;
  int32_T c10_i6;
  real_T *c10_fz_left;
  real_T *c10_fz_right;
  real_T (*c10_Rcontact)[16];
  real_T (*c10_Lcontact)[16];
  real_T (*c10_Rcop)[2];
  real_T (*c10_zmp)[2];
  real_T (*c10_Lcop)[2];
  c10_Rcontact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 5);
  c10_Lcontact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 4);
  c10_Rcop = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 3);
  c10_zmp = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c10_Lcop = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c10_fz_right = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c10_fz_left = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 8U, chartInstance->c10_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c10_fz_left, 0U);
  _SFD_DATA_RANGE_CHECK(*c10_fz_right, 1U);
  for (c10_i2 = 0; c10_i2 < 2; c10_i2++) {
    _SFD_DATA_RANGE_CHECK((*c10_Lcop)[c10_i2], 2U);
  }

  for (c10_i3 = 0; c10_i3 < 2; c10_i3++) {
    _SFD_DATA_RANGE_CHECK((*c10_zmp)[c10_i3], 3U);
  }

  for (c10_i4 = 0; c10_i4 < 2; c10_i4++) {
    _SFD_DATA_RANGE_CHECK((*c10_Rcop)[c10_i4], 4U);
  }

  for (c10_i5 = 0; c10_i5 < 16; c10_i5++) {
    _SFD_DATA_RANGE_CHECK((*c10_Lcontact)[c10_i5], 5U);
  }

  for (c10_i6 = 0; c10_i6 < 16; c10_i6++) {
    _SFD_DATA_RANGE_CHECK((*c10_Rcontact)[c10_i6], 6U);
  }

  chartInstance->c10_sfEvent = CALL_EVENT;
  c10_chartstep_c10_torqueBalancing2012b(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c10_chartstep_c10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
  real_T c10_hoistedGlobal;
  real_T c10_b_hoistedGlobal;
  real_T c10_fz_left;
  real_T c10_fz_right;
  int32_T c10_i7;
  real_T c10_Lcop[2];
  int32_T c10_i8;
  real_T c10_Rcop[2];
  int32_T c10_i9;
  real_T c10_Lcontact[16];
  int32_T c10_i10;
  real_T c10_Rcontact[16];
  uint32_T c10_debug_family_var_map[9];
  real_T c10_b_Lcop[2];
  real_T c10_b_Rcop[2];
  real_T c10_nargin = 6.0;
  real_T c10_nargout = 1.0;
  real_T c10_zmp[2];
  int32_T c10_i11;
  int32_T c10_i12;
  int32_T c10_i13;
  int32_T c10_i14;
  real_T c10_a[4];
  int32_T c10_i15;
  real_T c10_b[2];
  int32_T c10_i16;
  real_T c10_y[2];
  int32_T c10_i17;
  int32_T c10_i18;
  int32_T c10_i19;
  int32_T c10_i20;
  int32_T c10_i21;
  int32_T c10_i22;
  int32_T c10_i23;
  int32_T c10_i24;
  int32_T c10_i25;
  int32_T c10_i26;
  int32_T c10_i27;
  int32_T c10_i28;
  real_T c10_b_a;
  int32_T c10_i29;
  int32_T c10_i30;
  real_T c10_c_a;
  int32_T c10_i31;
  int32_T c10_i32;
  int32_T c10_i33;
  real_T c10_B;
  real_T c10_b_y;
  real_T c10_c_y;
  int32_T c10_i34;
  int32_T c10_i35;
  real_T *c10_b_fz_left;
  real_T *c10_b_fz_right;
  real_T (*c10_b_zmp)[2];
  real_T (*c10_b_Rcontact)[16];
  real_T (*c10_b_Lcontact)[16];
  real_T (*c10_c_Rcop)[2];
  real_T (*c10_c_Lcop)[2];
  c10_b_Rcontact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 5);
  c10_b_Lcontact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 4);
  c10_c_Rcop = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 3);
  c10_b_zmp = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c10_c_Lcop = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c10_b_fz_right = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c10_b_fz_left = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 8U, chartInstance->c10_sfEvent);
  c10_hoistedGlobal = *c10_b_fz_left;
  c10_b_hoistedGlobal = *c10_b_fz_right;
  c10_fz_left = c10_hoistedGlobal;
  c10_fz_right = c10_b_hoistedGlobal;
  for (c10_i7 = 0; c10_i7 < 2; c10_i7++) {
    c10_Lcop[c10_i7] = (*c10_c_Lcop)[c10_i7];
  }

  for (c10_i8 = 0; c10_i8 < 2; c10_i8++) {
    c10_Rcop[c10_i8] = (*c10_c_Rcop)[c10_i8];
  }

  for (c10_i9 = 0; c10_i9 < 16; c10_i9++) {
    c10_Lcontact[c10_i9] = (*c10_b_Lcontact)[c10_i9];
  }

  for (c10_i10 = 0; c10_i10 < 16; c10_i10++) {
    c10_Rcontact[c10_i10] = (*c10_b_Rcontact)[c10_i10];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 11U, c10_debug_family_names,
    c10_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_b_Lcop, MAX_uint32_T,
    c10_sf_marshallOut, c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_b_Rcop, MAX_uint32_T,
    c10_sf_marshallOut, c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargin, 2U, c10_d_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargout, 3U, c10_d_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c10_fz_left, 4U, c10_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c10_fz_right, 5U, c10_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_Lcop, 0U, c10_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_Rcop, 1U, c10_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_Lcontact, 6U, c10_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_Rcontact, 7U, c10_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_zmp, 8U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 4);
  c10_i11 = 0;
  c10_i12 = 0;
  for (c10_i13 = 0; c10_i13 < 2; c10_i13++) {
    for (c10_i14 = 0; c10_i14 < 2; c10_i14++) {
      c10_a[c10_i14 + c10_i11] = c10_Lcontact[c10_i14 + c10_i12];
    }

    c10_i11 += 2;
    c10_i12 += 4;
  }

  for (c10_i15 = 0; c10_i15 < 2; c10_i15++) {
    c10_b[c10_i15] = c10_Lcop[c10_i15];
  }

  c10_eml_scalar_eg(chartInstance);
  c10_eml_scalar_eg(chartInstance);
  for (c10_i16 = 0; c10_i16 < 2; c10_i16++) {
    c10_y[c10_i16] = 0.0;
    c10_i17 = 0;
    for (c10_i18 = 0; c10_i18 < 2; c10_i18++) {
      c10_y[c10_i16] += c10_a[c10_i17 + c10_i16] * c10_b[c10_i18];
      c10_i17 += 2;
    }
  }

  for (c10_i19 = 0; c10_i19 < 2; c10_i19++) {
    c10_b_Lcop[c10_i19] = c10_y[c10_i19] + c10_Lcontact[c10_i19 + 12];
  }

  _SFD_SYMBOL_SWITCH(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 5);
  c10_i20 = 0;
  c10_i21 = 0;
  for (c10_i22 = 0; c10_i22 < 2; c10_i22++) {
    for (c10_i23 = 0; c10_i23 < 2; c10_i23++) {
      c10_a[c10_i23 + c10_i20] = c10_Rcontact[c10_i23 + c10_i21];
    }

    c10_i20 += 2;
    c10_i21 += 4;
  }

  for (c10_i24 = 0; c10_i24 < 2; c10_i24++) {
    c10_b[c10_i24] = c10_Rcop[c10_i24];
  }

  c10_eml_scalar_eg(chartInstance);
  c10_eml_scalar_eg(chartInstance);
  for (c10_i25 = 0; c10_i25 < 2; c10_i25++) {
    c10_y[c10_i25] = 0.0;
    c10_i26 = 0;
    for (c10_i27 = 0; c10_i27 < 2; c10_i27++) {
      c10_y[c10_i25] += c10_a[c10_i26 + c10_i25] * c10_b[c10_i27];
      c10_i26 += 2;
    }
  }

  for (c10_i28 = 0; c10_i28 < 2; c10_i28++) {
    c10_b_Rcop[c10_i28] = c10_y[c10_i28] + c10_Rcontact[c10_i28 + 12];
  }

  _SFD_SYMBOL_SWITCH(1U, 1U);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 9);
  c10_b_a = c10_fz_left;
  for (c10_i29 = 0; c10_i29 < 2; c10_i29++) {
    c10_b[c10_i29] = c10_b_Lcop[c10_i29];
  }

  for (c10_i30 = 0; c10_i30 < 2; c10_i30++) {
    c10_b[c10_i30] *= c10_b_a;
  }

  c10_c_a = c10_fz_right;
  for (c10_i31 = 0; c10_i31 < 2; c10_i31++) {
    c10_y[c10_i31] = c10_b_Rcop[c10_i31];
  }

  for (c10_i32 = 0; c10_i32 < 2; c10_i32++) {
    c10_y[c10_i32] *= c10_c_a;
  }

  for (c10_i33 = 0; c10_i33 < 2; c10_i33++) {
    c10_b[c10_i33] += c10_y[c10_i33];
  }

  c10_B = c10_fz_left + c10_fz_right;
  c10_b_y = c10_B;
  c10_c_y = c10_b_y;
  for (c10_i34 = 0; c10_i34 < 2; c10_i34++) {
    c10_zmp[c10_i34] = c10_b[c10_i34] / c10_c_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, -9);
  _SFD_SYMBOL_SCOPE_POP();
  for (c10_i35 = 0; c10_i35 < 2; c10_i35++) {
    (*c10_b_zmp)[c10_i35] = c10_zmp[c10_i35];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 8U, chartInstance->c10_sfEvent);
}

static void initSimStructsc10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc10_torqueBalancing2012b
  (SFc10_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c10_machineNumber, uint32_T
  c10_chartNumber)
{
}

static const mxArray *c10_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i36;
  real_T c10_b_inData[2];
  int32_T c10_i37;
  real_T c10_u[2];
  const mxArray *c10_y = NULL;
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  for (c10_i36 = 0; c10_i36 < 2; c10_i36++) {
    c10_b_inData[c10_i36] = (*(real_T (*)[2])c10_inData)[c10_i36];
  }

  for (c10_i37 = 0; c10_i37 < 2; c10_i37++) {
    c10_u[c10_i37] = c10_b_inData[c10_i37];
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, FALSE);
  return c10_mxArrayOutData;
}

static void c10_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_zmp, const char_T *c10_identifier, real_T
  c10_y[2])
{
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_zmp), &c10_thisId, c10_y);
  sf_mex_destroy(&c10_zmp);
}

static void c10_b_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId,
  real_T c10_y[2])
{
  real_T c10_dv1[2];
  int32_T c10_i38;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_dv1, 1, 0, 0U, 1, 0U, 1, 2);
  for (c10_i38 = 0; c10_i38 < 2; c10_i38++) {
    c10_y[c10_i38] = c10_dv1[c10_i38];
  }

  sf_mex_destroy(&c10_u);
}

static void c10_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_zmp;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y[2];
  int32_T c10_i39;
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c10_zmp = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_zmp), &c10_thisId, c10_y);
  sf_mex_destroy(&c10_zmp);
  for (c10_i39 = 0; c10_i39 < 2; c10_i39++) {
    (*(real_T (*)[2])c10_outData)[c10_i39] = c10_y[c10_i39];
  }

  sf_mex_destroy(&c10_mxArrayInData);
}

static const mxArray *c10_b_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i40;
  int32_T c10_i41;
  int32_T c10_i42;
  real_T c10_b_inData[16];
  int32_T c10_i43;
  int32_T c10_i44;
  int32_T c10_i45;
  real_T c10_u[16];
  const mxArray *c10_y = NULL;
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_i40 = 0;
  for (c10_i41 = 0; c10_i41 < 4; c10_i41++) {
    for (c10_i42 = 0; c10_i42 < 4; c10_i42++) {
      c10_b_inData[c10_i42 + c10_i40] = (*(real_T (*)[16])c10_inData)[c10_i42 +
        c10_i40];
    }

    c10_i40 += 4;
  }

  c10_i43 = 0;
  for (c10_i44 = 0; c10_i44 < 4; c10_i44++) {
    for (c10_i45 = 0; c10_i45 < 4; c10_i45++) {
      c10_u[c10_i45 + c10_i43] = c10_b_inData[c10_i45 + c10_i43];
    }

    c10_i43 += 4;
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, FALSE);
  return c10_mxArrayOutData;
}

static const mxArray *c10_c_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i46;
  real_T c10_b_inData[2];
  int32_T c10_i47;
  real_T c10_u[2];
  const mxArray *c10_y = NULL;
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  for (c10_i46 = 0; c10_i46 < 2; c10_i46++) {
    c10_b_inData[c10_i46] = (*(real_T (*)[2])c10_inData)[c10_i46];
  }

  for (c10_i47 = 0; c10_i47 < 2; c10_i47++) {
    c10_u[c10_i47] = c10_b_inData[c10_i47];
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 2, 2, 1), FALSE);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, FALSE);
  return c10_mxArrayOutData;
}

static const mxArray *c10_d_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  real_T c10_u;
  const mxArray *c10_y = NULL;
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_u = *(real_T *)c10_inData;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, FALSE);
  return c10_mxArrayOutData;
}

static real_T c10_c_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  real_T c10_y;
  real_T c10_d0;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_d0, 1, 0, 0U, 0, 0U, 0);
  c10_y = c10_d0;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_nargout;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y;
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c10_nargout = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_nargout),
    &c10_thisId);
  sf_mex_destroy(&c10_nargout);
  *(real_T *)c10_outData = c10_y;
  sf_mex_destroy(&c10_mxArrayInData);
}

const mxArray *sf_c10_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c10_nameCaptureInfo;
  c10_ResolvedFunctionInfo c10_info[13];
  const mxArray *c10_m0 = NULL;
  int32_T c10_i48;
  c10_ResolvedFunctionInfo *c10_r0;
  c10_nameCaptureInfo = NULL;
  c10_nameCaptureInfo = NULL;
  c10_info_helper(c10_info);
  sf_mex_assign(&c10_m0, sf_mex_createstruct("nameCaptureInfo", 1, 13), FALSE);
  for (c10_i48 = 0; c10_i48 < 13; c10_i48++) {
    c10_r0 = &c10_info[c10_i48];
    sf_mex_addfield(c10_m0, sf_mex_create("nameCaptureInfo", c10_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c10_r0->context)), "context", "nameCaptureInfo",
                    c10_i48);
    sf_mex_addfield(c10_m0, sf_mex_create("nameCaptureInfo", c10_r0->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c10_r0->name)), "name", "nameCaptureInfo",
                    c10_i48);
    sf_mex_addfield(c10_m0, sf_mex_create("nameCaptureInfo",
      c10_r0->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c10_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c10_i48);
    sf_mex_addfield(c10_m0, sf_mex_create("nameCaptureInfo", c10_r0->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c10_r0->resolved)), "resolved",
                    "nameCaptureInfo", c10_i48);
    sf_mex_addfield(c10_m0, sf_mex_create("nameCaptureInfo", &c10_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c10_i48);
    sf_mex_addfield(c10_m0, sf_mex_create("nameCaptureInfo", &c10_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c10_i48);
    sf_mex_addfield(c10_m0, sf_mex_create("nameCaptureInfo",
      &c10_r0->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c10_i48);
    sf_mex_addfield(c10_m0, sf_mex_create("nameCaptureInfo",
      &c10_r0->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c10_i48);
  }

  sf_mex_assign(&c10_nameCaptureInfo, c10_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c10_nameCaptureInfo);
  return c10_nameCaptureInfo;
}

static void c10_info_helper(c10_ResolvedFunctionInfo c10_info[13])
{
  c10_info[0].context = "";
  c10_info[0].name = "mtimes";
  c10_info[0].dominantType = "double";
  c10_info[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c10_info[0].fileTimeLo = 1289519692U;
  c10_info[0].fileTimeHi = 0U;
  c10_info[0].mFileTimeLo = 0U;
  c10_info[0].mFileTimeHi = 0U;
  c10_info[1].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c10_info[1].name = "eml_index_class";
  c10_info[1].dominantType = "";
  c10_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c10_info[1].fileTimeLo = 1323170578U;
  c10_info[1].fileTimeHi = 0U;
  c10_info[1].mFileTimeLo = 0U;
  c10_info[1].mFileTimeHi = 0U;
  c10_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c10_info[2].name = "eml_scalar_eg";
  c10_info[2].dominantType = "double";
  c10_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c10_info[2].fileTimeLo = 1286818796U;
  c10_info[2].fileTimeHi = 0U;
  c10_info[2].mFileTimeLo = 0U;
  c10_info[2].mFileTimeHi = 0U;
  c10_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c10_info[3].name = "eml_xgemm";
  c10_info[3].dominantType = "char";
  c10_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c10_info[3].fileTimeLo = 1299076772U;
  c10_info[3].fileTimeHi = 0U;
  c10_info[3].mFileTimeLo = 0U;
  c10_info[3].mFileTimeHi = 0U;
  c10_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c10_info[4].name = "eml_blas_inline";
  c10_info[4].dominantType = "";
  c10_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c10_info[4].fileTimeLo = 1299076768U;
  c10_info[4].fileTimeHi = 0U;
  c10_info[4].mFileTimeLo = 0U;
  c10_info[4].mFileTimeHi = 0U;
  c10_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c10_info[5].name = "mtimes";
  c10_info[5].dominantType = "double";
  c10_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c10_info[5].fileTimeLo = 1289519692U;
  c10_info[5].fileTimeHi = 0U;
  c10_info[5].mFileTimeLo = 0U;
  c10_info[5].mFileTimeHi = 0U;
  c10_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c10_info[6].name = "eml_index_class";
  c10_info[6].dominantType = "";
  c10_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c10_info[6].fileTimeLo = 1323170578U;
  c10_info[6].fileTimeHi = 0U;
  c10_info[6].mFileTimeLo = 0U;
  c10_info[6].mFileTimeHi = 0U;
  c10_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c10_info[7].name = "eml_scalar_eg";
  c10_info[7].dominantType = "double";
  c10_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c10_info[7].fileTimeLo = 1286818796U;
  c10_info[7].fileTimeHi = 0U;
  c10_info[7].mFileTimeLo = 0U;
  c10_info[7].mFileTimeHi = 0U;
  c10_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c10_info[8].name = "eml_refblas_xgemm";
  c10_info[8].dominantType = "char";
  c10_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c10_info[8].fileTimeLo = 1299076774U;
  c10_info[8].fileTimeHi = 0U;
  c10_info[8].mFileTimeLo = 0U;
  c10_info[8].mFileTimeHi = 0U;
  c10_info[9].context = "";
  c10_info[9].name = "mrdivide";
  c10_info[9].dominantType = "double";
  c10_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c10_info[9].fileTimeLo = 1357951548U;
  c10_info[9].fileTimeHi = 0U;
  c10_info[9].mFileTimeLo = 1319729966U;
  c10_info[9].mFileTimeHi = 0U;
  c10_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c10_info[10].name = "rdivide";
  c10_info[10].dominantType = "double";
  c10_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c10_info[10].fileTimeLo = 1346510388U;
  c10_info[10].fileTimeHi = 0U;
  c10_info[10].mFileTimeLo = 0U;
  c10_info[10].mFileTimeHi = 0U;
  c10_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c10_info[11].name = "eml_scalexp_compatible";
  c10_info[11].dominantType = "double";
  c10_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c10_info[11].fileTimeLo = 1286818796U;
  c10_info[11].fileTimeHi = 0U;
  c10_info[11].mFileTimeLo = 0U;
  c10_info[11].mFileTimeHi = 0U;
  c10_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c10_info[12].name = "eml_div";
  c10_info[12].dominantType = "double";
  c10_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c10_info[12].fileTimeLo = 1313347810U;
  c10_info[12].fileTimeHi = 0U;
  c10_info[12].mFileTimeLo = 0U;
  c10_info[12].mFileTimeHi = 0U;
}

static void c10_eml_scalar_eg(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static const mxArray *c10_e_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_u;
  const mxArray *c10_y = NULL;
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_u = *(int32_T *)c10_inData;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, FALSE);
  return c10_mxArrayOutData;
}

static int32_T c10_d_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  int32_T c10_y;
  int32_T c10_i49;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_i49, 1, 6, 0U, 0, 0U, 0);
  c10_y = c10_i49;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_b_sfEvent;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  int32_T c10_y;
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c10_b_sfEvent = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_sfEvent),
    &c10_thisId);
  sf_mex_destroy(&c10_b_sfEvent);
  *(int32_T *)c10_outData = c10_y;
  sf_mex_destroy(&c10_mxArrayInData);
}

static uint8_T c10_e_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_b_is_active_c10_torqueBalancing2012b, const
  char_T *c10_identifier)
{
  uint8_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c10_b_is_active_c10_torqueBalancing2012b), &c10_thisId);
  sf_mex_destroy(&c10_b_is_active_c10_torqueBalancing2012b);
  return c10_y;
}

static uint8_T c10_f_emlrt_marshallIn(SFc10_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  uint8_T c10_y;
  uint8_T c10_u0;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_u0, 1, 3, 0U, 0, 0U, 0);
  c10_y = c10_u0;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void init_dsm_address_info(SFc10_torqueBalancing2012bInstanceStruct
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

void sf_c10_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3682668841U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3541030052U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3330767569U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(151866303U);
}

mxArray *sf_c10_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("UvPxi1ZdnPR7AHUR623VMC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
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
      pr[0] = (double)(2);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
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
      pr[0] = (double)(2);
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

mxArray *sf_c10_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c10_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"zmp\",},{M[8],M[0],T\"is_active_c10_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c10_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           10,
           1,
           1,
           7,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"fz_left");
          _SFD_SET_DATA_PROPS(1,1,1,0,"fz_right");
          _SFD_SET_DATA_PROPS(2,1,1,0,"Lcop");
          _SFD_SET_DATA_PROPS(3,2,0,1,"zmp");
          _SFD_SET_DATA_PROPS(4,1,1,0,"Rcop");
          _SFD_SET_DATA_PROPS(5,1,1,0,"Lcontact");
          _SFD_SET_DATA_PROPS(6,1,1,0,"Rcontact");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,267);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_d_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_d_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)
            c10_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c10_fz_left;
          real_T *c10_fz_right;
          real_T (*c10_Lcop)[2];
          real_T (*c10_zmp)[2];
          real_T (*c10_Rcop)[2];
          real_T (*c10_Lcontact)[16];
          real_T (*c10_Rcontact)[16];
          c10_Rcontact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            5);
          c10_Lcontact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            4);
          c10_Rcop = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 3);
          c10_zmp = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
          c10_Lcop = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
          c10_fz_right = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c10_fz_left = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c10_fz_left);
          _SFD_SET_DATA_VALUE_PTR(1U, c10_fz_right);
          _SFD_SET_DATA_VALUE_PTR(2U, *c10_Lcop);
          _SFD_SET_DATA_VALUE_PTR(3U, *c10_zmp);
          _SFD_SET_DATA_VALUE_PTR(4U, *c10_Rcop);
          _SFD_SET_DATA_VALUE_PTR(5U, *c10_Lcontact);
          _SFD_SET_DATA_VALUE_PTR(6U, *c10_Rcontact);
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
  return "n8EtlPdLXBAK10lkieUrlB";
}

static void sf_opaque_initialize_c10_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc10_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c10_torqueBalancing2012b
    ((SFc10_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c10_torqueBalancing2012b((SFc10_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c10_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c10_torqueBalancing2012b((SFc10_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c10_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c10_torqueBalancing2012b((SFc10_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c10_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c10_torqueBalancing2012b((SFc10_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c10_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c10_torqueBalancing2012b
    ((SFc10_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c10_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c10_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c10_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c10_torqueBalancing2012b
    ((SFc10_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c10_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c10_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c10_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c10_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c10_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc10_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c10_torqueBalancing2012b((SFc10_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc10_torqueBalancing2012b
    ((SFc10_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c10_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c10_torqueBalancing2012b
      ((SFc10_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c10_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      10);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,10,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,10,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,10);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,10,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,10,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,10);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1222420032U));
  ssSetChecksum1(S,(3114174418U));
  ssSetChecksum2(S,(74068730U));
  ssSetChecksum3(S,(2669218505U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c10_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c10_torqueBalancing2012b(SimStruct *S)
{
  SFc10_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc10_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc10_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc10_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c10_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c10_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c10_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c10_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c10_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c10_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c10_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c10_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c10_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c10_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c10_torqueBalancing2012b;
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

void c10_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c10_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c10_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c10_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c10_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
