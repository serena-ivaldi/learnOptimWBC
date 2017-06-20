/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c4_torqueBalancing2012b.h"
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
static const char * c4_debug_family_names[9] = { "pinvJb", "nargin", "nargout",
  "qjErr", "JL", "JR", "reg", "activeFeetConstraints", "vbEquivalent" };

/* Function Declarations */
static void initialize_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c4_st);
static void finalize_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c4_torqueBalancing2012b(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c4_chartstep_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void initSimStructsc4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static void c4_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_vbEquivalent, const char_T *c4_identifier,
  real_T c4_y[6]);
static void c4_b_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[6]);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static c4_struct_1ZGMVR6bgCMpDdXTSGnu6G c4_c_emlrt_marshallIn
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c4_u,
   const emlrtMsgIdentifier *c4_parentId);
static real_T c4_d_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_g_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_e_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[36]);
static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[125]);
static void c4_b_info_helper(c4_ResolvedFunctionInfo c4_info[125]);
static void c4_eml_scalar_eg(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c4_mldivide(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c4_A[36], real_T c4_B[36], real_T c4_Y[36]);
static void c4_realmin(SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void c4_eps(SFc4_torqueBalancing2012bInstanceStruct *chartInstance);
static void c4_eml_matlab_zgetrf(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_b_A[36], int32_T c4_ipiv[6],
  int32_T *c4_info);
static void c4_check_forloop_overflow_error
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c4_overflow);
static void c4_eml_xger(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c4_m, int32_T c4_n, real_T c4_alpha1, int32_T c4_ix0, int32_T c4_iy0,
  real_T c4_A[36], int32_T c4_ia0, real_T c4_b_A[36]);
static void c4_eml_warning(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c4_eml_xtrsm(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c4_A[36], real_T c4_B[36], real_T c4_b_B[36]);
static void c4_below_threshold(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c4_b_eml_scalar_eg(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c4_b_eml_xtrsm(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_B[36], real_T c4_b_B[36]);
static void c4_c_eml_scalar_eg(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c4_eml_xgemm(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c4_A[36], real_T c4_B[138], real_T c4_C[138], real_T c4_b_C[138]);
static void c4_d_eml_scalar_eg(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c4_h_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_f_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_g_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_torqueBalancing2012b, const
  char_T *c4_identifier);
static uint8_T c4_h_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_b_eml_matlab_zgetrf(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], int32_T c4_ipiv[6], int32_T *c4_info);
static void c4_b_eml_xger(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c4_m, int32_T c4_n, real_T c4_alpha1, int32_T c4_ix0, int32_T c4_iy0,
  real_T c4_A[36], int32_T c4_ia0);
static void c4_c_eml_xtrsm(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_B[36]);
static void c4_d_eml_xtrsm(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_B[36]);
static void c4_b_eml_xgemm(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_B[138], real_T c4_C[138]);
static void init_dsm_address_info(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c4_is_active_c4_torqueBalancing2012b = 0U;
}

static void initialize_params_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c4_m0 = NULL;
  const mxArray *c4_mxField;
  c4_struct_1ZGMVR6bgCMpDdXTSGnu6G c4_r0;
  sf_set_error_prefix_string(
    "Error evaluating data 'reg' in the parent workspace.\n");
  c4_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c4_mxField = sf_mex_getfield(c4_m0, "pinvTol", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c4_mxField), &c4_r0.pinvTol, 1, 0, 0U, 0,
                      0U, 0);
  c4_mxField = sf_mex_getfield(c4_m0, "pinvDamp", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c4_mxField), &c4_r0.pinvDamp, 1, 0, 0U,
                      0, 0U, 0);
  c4_mxField = sf_mex_getfield(c4_m0, "pinvDampVb", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c4_mxField), &c4_r0.pinvDampVb, 1, 0, 0U,
                      0, 0U, 0);
  c4_mxField = sf_mex_getfield(c4_m0, "HessianQP", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c4_mxField), &c4_r0.HessianQP, 1, 0, 0U,
                      0, 0U, 0);
  c4_mxField = sf_mex_getfield(c4_m0, "impedances", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c4_mxField), &c4_r0.impedances, 1, 0, 0U,
                      0, 0U, 0);
  c4_mxField = sf_mex_getfield(c4_m0, "dampings", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c4_mxField), &c4_r0.dampings, 1, 0, 0U,
                      0, 0U, 0);
  c4_mxField = sf_mex_getfield(c4_m0, "norm_tolerance", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c4_mxField), &c4_r0.norm_tolerance, 1, 0,
                      0U, 0, 0U, 0);
  sf_mex_destroy(&c4_m0);
  chartInstance->c4_reg = c4_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c4_update_debugger_state_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c4_st;
  const mxArray *c4_y = NULL;
  int32_T c4_i0;
  real_T c4_u[6];
  const mxArray *c4_b_y = NULL;
  uint8_T c4_hoistedGlobal;
  uint8_T c4_b_u;
  const mxArray *c4_c_y = NULL;
  real_T (*c4_vbEquivalent)[6];
  c4_vbEquivalent = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_st = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellarray(2), FALSE);
  for (c4_i0 = 0; c4_i0 < 6; c4_i0++) {
    c4_u[c4_i0] = (*c4_vbEquivalent)[c4_i0];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  c4_hoistedGlobal = chartInstance->c4_is_active_c4_torqueBalancing2012b;
  c4_b_u = c4_hoistedGlobal;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  sf_mex_assign(&c4_st, c4_y, FALSE);
  return c4_st;
}

static void set_sim_state_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T c4_dv0[6];
  int32_T c4_i1;
  real_T (*c4_vbEquivalent)[6];
  c4_vbEquivalent = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c4_doneDoubleBufferReInit = TRUE;
  c4_u = sf_mex_dup(c4_st);
  c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 0)),
                      "vbEquivalent", c4_dv0);
  for (c4_i1 = 0; c4_i1 < 6; c4_i1++) {
    (*c4_vbEquivalent)[c4_i1] = c4_dv0[c4_i1];
  }

  chartInstance->c4_is_active_c4_torqueBalancing2012b = c4_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
     "is_active_c4_torqueBalancing2012b");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c4_torqueBalancing2012b(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c4_i2;
  int32_T c4_i3;
  int32_T c4_i4;
  int32_T c4_i5;
  int32_T c4_i6;
  real_T (*c4_activeFeetConstraints)[2];
  real_T (*c4_vbEquivalent)[6];
  real_T (*c4_JR)[174];
  real_T (*c4_JL)[174];
  real_T (*c4_qjErr)[23];
  c4_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
    (chartInstance->S, 3);
  c4_vbEquivalent = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_JR = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 2);
  c4_JL = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c4_qjErr = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c4_sfEvent);
  for (c4_i2 = 0; c4_i2 < 23; c4_i2++) {
    _SFD_DATA_RANGE_CHECK((*c4_qjErr)[c4_i2], 0U);
  }

  for (c4_i3 = 0; c4_i3 < 174; c4_i3++) {
    _SFD_DATA_RANGE_CHECK((*c4_JL)[c4_i3], 1U);
  }

  for (c4_i4 = 0; c4_i4 < 174; c4_i4++) {
    _SFD_DATA_RANGE_CHECK((*c4_JR)[c4_i4], 2U);
  }

  for (c4_i5 = 0; c4_i5 < 6; c4_i5++) {
    _SFD_DATA_RANGE_CHECK((*c4_vbEquivalent)[c4_i5], 3U);
  }

  for (c4_i6 = 0; c4_i6 < 2; c4_i6++) {
    _SFD_DATA_RANGE_CHECK((*c4_activeFeetConstraints)[c4_i6], 5U);
  }

  chartInstance->c4_sfEvent = CALL_EVENT;
  c4_chartstep_c4_torqueBalancing2012b(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c4_chartstep_c4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
  int32_T c4_i7;
  real_T c4_qjErr[23];
  int32_T c4_i8;
  real_T c4_JL[174];
  int32_T c4_i9;
  real_T c4_JR[174];
  c4_struct_1ZGMVR6bgCMpDdXTSGnu6G c4_b_reg;
  int32_T c4_i10;
  real_T c4_activeFeetConstraints[2];
  uint32_T c4_debug_family_var_map[9];
  real_T c4_pinvJb[36];
  real_T c4_nargin = 5.0;
  real_T c4_nargout = 1.0;
  real_T c4_vbEquivalent[6];
  int32_T c4_i11;
  int32_T c4_i12;
  int32_T c4_i13;
  int32_T c4_i14;
  real_T c4_a[36];
  int32_T c4_i15;
  int32_T c4_i16;
  int32_T c4_i17;
  real_T c4_b[36];
  int32_T c4_i18;
  int32_T c4_i19;
  int32_T c4_i20;
  real_T c4_y[36];
  int32_T c4_i21;
  int32_T c4_i22;
  real_T c4_b_a;
  int32_T c4_i23;
  static real_T c4_b_b[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c4_i24;
  real_T c4_b_y[36];
  int32_T c4_i25;
  int32_T c4_i26;
  int32_T c4_i27;
  int32_T c4_i28;
  real_T c4_b_JL[36];
  real_T c4_dv1[36];
  int32_T c4_i29;
  int32_T c4_i30;
  int32_T c4_i31;
  int32_T c4_i32;
  int32_T c4_i33;
  real_T c4_c_b[138];
  int32_T c4_i34;
  real_T c4_c_y[138];
  int32_T c4_i35;
  real_T c4_c_a[36];
  int32_T c4_i36;
  real_T c4_d_b[138];
  int32_T c4_i37;
  real_T c4_e_b[23];
  int32_T c4_i38;
  int32_T c4_i39;
  int32_T c4_i40;
  real_T c4_C[6];
  int32_T c4_i41;
  int32_T c4_i42;
  int32_T c4_i43;
  int32_T c4_i44;
  int32_T c4_i45;
  int32_T c4_i46;
  int32_T c4_i47;
  int32_T c4_i48;
  int32_T c4_i49;
  int32_T c4_i50;
  int32_T c4_i51;
  int32_T c4_i52;
  int32_T c4_i53;
  int32_T c4_i54;
  int32_T c4_i55;
  int32_T c4_i56;
  int32_T c4_i57;
  int32_T c4_i58;
  real_T c4_d_a;
  int32_T c4_i59;
  int32_T c4_i60;
  real_T c4_d_y[36];
  int32_T c4_i61;
  int32_T c4_i62;
  int32_T c4_i63;
  int32_T c4_i64;
  real_T c4_b_JR[36];
  real_T c4_dv2[36];
  int32_T c4_i65;
  int32_T c4_i66;
  int32_T c4_i67;
  int32_T c4_i68;
  int32_T c4_i69;
  int32_T c4_i70;
  int32_T c4_i71;
  real_T c4_e_a[36];
  int32_T c4_i72;
  real_T c4_f_b[138];
  int32_T c4_i73;
  int32_T c4_i74;
  int32_T c4_i75;
  int32_T c4_i76;
  int32_T c4_i77;
  int32_T c4_i78;
  int32_T c4_i79;
  int32_T c4_i80;
  int32_T c4_i81;
  int32_T c4_i82;
  int32_T c4_i83;
  real_T (*c4_b_vbEquivalent)[6];
  real_T (*c4_b_activeFeetConstraints)[2];
  real_T (*c4_c_JR)[174];
  real_T (*c4_c_JL)[174];
  real_T (*c4_b_qjErr)[23];
  c4_b_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
    (chartInstance->S, 3);
  c4_b_vbEquivalent = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_c_JR = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 2);
  c4_c_JL = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c4_b_qjErr = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c4_sfEvent);
  for (c4_i7 = 0; c4_i7 < 23; c4_i7++) {
    c4_qjErr[c4_i7] = (*c4_b_qjErr)[c4_i7];
  }

  for (c4_i8 = 0; c4_i8 < 174; c4_i8++) {
    c4_JL[c4_i8] = (*c4_c_JL)[c4_i8];
  }

  for (c4_i9 = 0; c4_i9 < 174; c4_i9++) {
    c4_JR[c4_i9] = (*c4_c_JR)[c4_i9];
  }

  c4_b_reg = chartInstance->c4_reg;
  for (c4_i10 = 0; c4_i10 < 2; c4_i10++) {
    c4_activeFeetConstraints[c4_i10] = (*c4_b_activeFeetConstraints)[c4_i10];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c4_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_pinvJb, 0U, c4_g_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 1U, c4_f_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 2U, c4_f_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_qjErr, 3U, c4_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_JL, 4U, c4_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_JR, 5U, c4_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_b_reg, 6U, c4_c_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_activeFeetConstraints, 7U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_vbEquivalent, 8U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 3);
  if (CV_EML_IF(0, 1, 0, c4_activeFeetConstraints[0] == 1.0)) {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 5);
    c4_i11 = 0;
    for (c4_i12 = 0; c4_i12 < 6; c4_i12++) {
      c4_i13 = 0;
      for (c4_i14 = 0; c4_i14 < 6; c4_i14++) {
        c4_a[c4_i14 + c4_i11] = c4_JL[c4_i13 + c4_i12];
        c4_i13 += 6;
      }

      c4_i11 += 6;
    }

    c4_i15 = 0;
    for (c4_i16 = 0; c4_i16 < 6; c4_i16++) {
      for (c4_i17 = 0; c4_i17 < 6; c4_i17++) {
        c4_b[c4_i17 + c4_i15] = c4_JL[c4_i17 + c4_i15];
      }

      c4_i15 += 6;
    }

    c4_eml_scalar_eg(chartInstance);
    c4_eml_scalar_eg(chartInstance);
    for (c4_i18 = 0; c4_i18 < 6; c4_i18++) {
      c4_i19 = 0;
      for (c4_i20 = 0; c4_i20 < 6; c4_i20++) {
        c4_y[c4_i19 + c4_i18] = 0.0;
        c4_i21 = 0;
        for (c4_i22 = 0; c4_i22 < 6; c4_i22++) {
          c4_y[c4_i19 + c4_i18] += c4_a[c4_i21 + c4_i18] * c4_b[c4_i22 + c4_i19];
          c4_i21 += 6;
        }

        c4_i19 += 6;
      }
    }

    c4_b_a = c4_b_reg.pinvDampVb;
    for (c4_i23 = 0; c4_i23 < 36; c4_i23++) {
      c4_a[c4_i23] = c4_b_a * c4_b_b[c4_i23];
    }

    for (c4_i24 = 0; c4_i24 < 36; c4_i24++) {
      c4_b_y[c4_i24] = c4_y[c4_i24] + c4_a[c4_i24];
    }

    c4_i25 = 0;
    for (c4_i26 = 0; c4_i26 < 6; c4_i26++) {
      c4_i27 = 0;
      for (c4_i28 = 0; c4_i28 < 6; c4_i28++) {
        c4_b_JL[c4_i28 + c4_i25] = c4_JL[c4_i27 + c4_i26];
        c4_i27 += 6;
      }

      c4_i25 += 6;
    }

    c4_mldivide(chartInstance, c4_b_y, c4_b_JL, c4_dv1);
    for (c4_i29 = 0; c4_i29 < 36; c4_i29++) {
      c4_pinvJb[c4_i29] = c4_dv1[c4_i29];
    }

    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 6);
    for (c4_i30 = 0; c4_i30 < 36; c4_i30++) {
      c4_a[c4_i30] = -c4_pinvJb[c4_i30];
    }

    c4_i31 = 0;
    for (c4_i32 = 0; c4_i32 < 23; c4_i32++) {
      for (c4_i33 = 0; c4_i33 < 6; c4_i33++) {
        c4_c_b[c4_i33 + c4_i31] = c4_JL[(c4_i33 + c4_i31) + 36];
      }

      c4_i31 += 6;
    }

    c4_c_eml_scalar_eg(chartInstance);
    c4_c_eml_scalar_eg(chartInstance);
    for (c4_i34 = 0; c4_i34 < 138; c4_i34++) {
      c4_c_y[c4_i34] = 0.0;
    }

    for (c4_i35 = 0; c4_i35 < 36; c4_i35++) {
      c4_c_a[c4_i35] = c4_a[c4_i35];
    }

    for (c4_i36 = 0; c4_i36 < 138; c4_i36++) {
      c4_d_b[c4_i36] = c4_c_b[c4_i36];
    }

    c4_b_eml_xgemm(chartInstance, c4_c_a, c4_d_b, c4_c_y);
    for (c4_i37 = 0; c4_i37 < 23; c4_i37++) {
      c4_e_b[c4_i37] = c4_qjErr[c4_i37];
    }

    c4_d_eml_scalar_eg(chartInstance);
    c4_d_eml_scalar_eg(chartInstance);
    for (c4_i38 = 0; c4_i38 < 6; c4_i38++) {
      c4_vbEquivalent[c4_i38] = 0.0;
    }

    for (c4_i39 = 0; c4_i39 < 6; c4_i39++) {
      c4_vbEquivalent[c4_i39] = 0.0;
    }

    for (c4_i40 = 0; c4_i40 < 6; c4_i40++) {
      c4_C[c4_i40] = c4_vbEquivalent[c4_i40];
    }

    for (c4_i41 = 0; c4_i41 < 6; c4_i41++) {
      c4_vbEquivalent[c4_i41] = c4_C[c4_i41];
    }

    for (c4_i42 = 0; c4_i42 < 6; c4_i42++) {
      c4_C[c4_i42] = c4_vbEquivalent[c4_i42];
    }

    for (c4_i43 = 0; c4_i43 < 6; c4_i43++) {
      c4_vbEquivalent[c4_i43] = c4_C[c4_i43];
    }

    for (c4_i44 = 0; c4_i44 < 6; c4_i44++) {
      c4_vbEquivalent[c4_i44] = 0.0;
      c4_i45 = 0;
      for (c4_i46 = 0; c4_i46 < 23; c4_i46++) {
        c4_vbEquivalent[c4_i44] += c4_c_y[c4_i45 + c4_i44] * c4_e_b[c4_i46];
        c4_i45 += 6;
      }
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 9);
    c4_i47 = 0;
    for (c4_i48 = 0; c4_i48 < 6; c4_i48++) {
      c4_i49 = 0;
      for (c4_i50 = 0; c4_i50 < 6; c4_i50++) {
        c4_a[c4_i50 + c4_i47] = c4_JR[c4_i49 + c4_i48];
        c4_i49 += 6;
      }

      c4_i47 += 6;
    }

    c4_i51 = 0;
    for (c4_i52 = 0; c4_i52 < 6; c4_i52++) {
      for (c4_i53 = 0; c4_i53 < 6; c4_i53++) {
        c4_b[c4_i53 + c4_i51] = c4_JR[c4_i53 + c4_i51];
      }

      c4_i51 += 6;
    }

    c4_eml_scalar_eg(chartInstance);
    c4_eml_scalar_eg(chartInstance);
    for (c4_i54 = 0; c4_i54 < 6; c4_i54++) {
      c4_i55 = 0;
      for (c4_i56 = 0; c4_i56 < 6; c4_i56++) {
        c4_y[c4_i55 + c4_i54] = 0.0;
        c4_i57 = 0;
        for (c4_i58 = 0; c4_i58 < 6; c4_i58++) {
          c4_y[c4_i55 + c4_i54] += c4_a[c4_i57 + c4_i54] * c4_b[c4_i58 + c4_i55];
          c4_i57 += 6;
        }

        c4_i55 += 6;
      }
    }

    c4_d_a = c4_b_reg.pinvDampVb;
    for (c4_i59 = 0; c4_i59 < 36; c4_i59++) {
      c4_a[c4_i59] = c4_d_a * c4_b_b[c4_i59];
    }

    for (c4_i60 = 0; c4_i60 < 36; c4_i60++) {
      c4_d_y[c4_i60] = c4_y[c4_i60] + c4_a[c4_i60];
    }

    c4_i61 = 0;
    for (c4_i62 = 0; c4_i62 < 6; c4_i62++) {
      c4_i63 = 0;
      for (c4_i64 = 0; c4_i64 < 6; c4_i64++) {
        c4_b_JR[c4_i64 + c4_i61] = c4_JR[c4_i63 + c4_i62];
        c4_i63 += 6;
      }

      c4_i61 += 6;
    }

    c4_mldivide(chartInstance, c4_d_y, c4_b_JR, c4_dv2);
    for (c4_i65 = 0; c4_i65 < 36; c4_i65++) {
      c4_pinvJb[c4_i65] = c4_dv2[c4_i65];
    }

    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 10);
    for (c4_i66 = 0; c4_i66 < 36; c4_i66++) {
      c4_a[c4_i66] = -c4_pinvJb[c4_i66];
    }

    c4_i67 = 0;
    for (c4_i68 = 0; c4_i68 < 23; c4_i68++) {
      for (c4_i69 = 0; c4_i69 < 6; c4_i69++) {
        c4_c_b[c4_i69 + c4_i67] = c4_JR[(c4_i69 + c4_i67) + 36];
      }

      c4_i67 += 6;
    }

    c4_c_eml_scalar_eg(chartInstance);
    c4_c_eml_scalar_eg(chartInstance);
    for (c4_i70 = 0; c4_i70 < 138; c4_i70++) {
      c4_c_y[c4_i70] = 0.0;
    }

    for (c4_i71 = 0; c4_i71 < 36; c4_i71++) {
      c4_e_a[c4_i71] = c4_a[c4_i71];
    }

    for (c4_i72 = 0; c4_i72 < 138; c4_i72++) {
      c4_f_b[c4_i72] = c4_c_b[c4_i72];
    }

    c4_b_eml_xgemm(chartInstance, c4_e_a, c4_f_b, c4_c_y);
    for (c4_i73 = 0; c4_i73 < 23; c4_i73++) {
      c4_e_b[c4_i73] = c4_qjErr[c4_i73];
    }

    c4_d_eml_scalar_eg(chartInstance);
    c4_d_eml_scalar_eg(chartInstance);
    for (c4_i74 = 0; c4_i74 < 6; c4_i74++) {
      c4_vbEquivalent[c4_i74] = 0.0;
    }

    for (c4_i75 = 0; c4_i75 < 6; c4_i75++) {
      c4_vbEquivalent[c4_i75] = 0.0;
    }

    for (c4_i76 = 0; c4_i76 < 6; c4_i76++) {
      c4_C[c4_i76] = c4_vbEquivalent[c4_i76];
    }

    for (c4_i77 = 0; c4_i77 < 6; c4_i77++) {
      c4_vbEquivalent[c4_i77] = c4_C[c4_i77];
    }

    for (c4_i78 = 0; c4_i78 < 6; c4_i78++) {
      c4_C[c4_i78] = c4_vbEquivalent[c4_i78];
    }

    for (c4_i79 = 0; c4_i79 < 6; c4_i79++) {
      c4_vbEquivalent[c4_i79] = c4_C[c4_i79];
    }

    for (c4_i80 = 0; c4_i80 < 6; c4_i80++) {
      c4_vbEquivalent[c4_i80] = 0.0;
      c4_i81 = 0;
      for (c4_i82 = 0; c4_i82 < 23; c4_i82++) {
        c4_vbEquivalent[c4_i80] += c4_c_y[c4_i81 + c4_i80] * c4_e_b[c4_i82];
        c4_i81 += 6;
      }
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -10);
  _SFD_SYMBOL_SCOPE_POP();
  for (c4_i83 = 0; c4_i83 < 6; c4_i83++) {
    (*c4_b_vbEquivalent)[c4_i83] = c4_vbEquivalent[c4_i83];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c4_sfEvent);
}

static void initSimStructsc4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc4_torqueBalancing2012b
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber)
{
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i84;
  real_T c4_b_inData[6];
  int32_T c4_i85;
  real_T c4_u[6];
  const mxArray *c4_y = NULL;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i84 = 0; c4_i84 < 6; c4_i84++) {
    c4_b_inData[c4_i84] = (*(real_T (*)[6])c4_inData)[c4_i84];
  }

  for (c4_i85 = 0; c4_i85 < 6; c4_i85++) {
    c4_u[c4_i85] = c4_b_inData[c4_i85];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static void c4_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_vbEquivalent, const char_T *c4_identifier,
  real_T c4_y[6])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_vbEquivalent), &c4_thisId,
                        c4_y);
  sf_mex_destroy(&c4_vbEquivalent);
}

static void c4_b_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[6])
{
  real_T c4_dv3[6];
  int32_T c4_i86;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv3, 1, 0, 0U, 1, 0U, 1, 6);
  for (c4_i86 = 0; c4_i86 < 6; c4_i86++) {
    c4_y[c4_i86] = c4_dv3[c4_i86];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_vbEquivalent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[6];
  int32_T c4_i87;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_vbEquivalent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_vbEquivalent), &c4_thisId,
                        c4_y);
  sf_mex_destroy(&c4_vbEquivalent);
  for (c4_i87 = 0; c4_i87 < 6; c4_i87++) {
    (*(real_T (*)[6])c4_outData)[c4_i87] = c4_y[c4_i87];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i88;
  real_T c4_b_inData[2];
  int32_T c4_i89;
  real_T c4_u[2];
  const mxArray *c4_y = NULL;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i88 = 0; c4_i88 < 2; c4_i88++) {
    c4_b_inData[c4_i88] = (*(real_T (*)[2])c4_inData)[c4_i88];
  }

  for (c4_i89 = 0; c4_i89 < 2; c4_i89++) {
    c4_u[c4_i89] = c4_b_inData[c4_i89];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  c4_struct_1ZGMVR6bgCMpDdXTSGnu6G c4_u;
  const mxArray *c4_y = NULL;
  real_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  real_T c4_c_u;
  const mxArray *c4_c_y = NULL;
  real_T c4_d_u;
  const mxArray *c4_d_y = NULL;
  real_T c4_e_u;
  const mxArray *c4_e_y = NULL;
  real_T c4_f_u;
  const mxArray *c4_f_y = NULL;
  real_T c4_g_u;
  const mxArray *c4_g_y = NULL;
  real_T c4_h_u;
  const mxArray *c4_h_y = NULL;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(c4_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c4_b_u = c4_u.pinvTol;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c4_y, c4_b_y, "pinvTol", "pinvTol", 0);
  c4_c_u = c4_u.pinvDamp;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c4_y, c4_c_y, "pinvDamp", "pinvDamp", 0);
  c4_d_u = c4_u.pinvDampVb;
  c4_d_y = NULL;
  sf_mex_assign(&c4_d_y, sf_mex_create("y", &c4_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c4_y, c4_d_y, "pinvDampVb", "pinvDampVb", 0);
  c4_e_u = c4_u.HessianQP;
  c4_e_y = NULL;
  sf_mex_assign(&c4_e_y, sf_mex_create("y", &c4_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c4_y, c4_e_y, "HessianQP", "HessianQP", 0);
  c4_f_u = c4_u.impedances;
  c4_f_y = NULL;
  sf_mex_assign(&c4_f_y, sf_mex_create("y", &c4_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c4_y, c4_f_y, "impedances", "impedances", 0);
  c4_g_u = c4_u.dampings;
  c4_g_y = NULL;
  sf_mex_assign(&c4_g_y, sf_mex_create("y", &c4_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c4_y, c4_g_y, "dampings", "dampings", 0);
  c4_h_u = c4_u.norm_tolerance;
  c4_h_y = NULL;
  sf_mex_assign(&c4_h_y, sf_mex_create("y", &c4_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c4_y, c4_h_y, "norm_tolerance", "norm_tolerance", 0);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static c4_struct_1ZGMVR6bgCMpDdXTSGnu6G c4_c_emlrt_marshallIn
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c4_u,
   const emlrtMsgIdentifier *c4_parentId)
{
  c4_struct_1ZGMVR6bgCMpDdXTSGnu6G c4_y;
  emlrtMsgIdentifier c4_thisId;
  static const char * c4_fieldNames[7] = { "pinvTol", "pinvDamp", "pinvDampVb",
    "HessianQP", "impedances", "dampings", "norm_tolerance" };

  c4_thisId.fParent = c4_parentId;
  sf_mex_check_struct(c4_parentId, c4_u, 7, c4_fieldNames, 0U, 0);
  c4_thisId.fIdentifier = "pinvTol";
  c4_y.pinvTol = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
                                        (c4_u, "pinvTol", "pinvTol", 0)),
    &c4_thisId);
  c4_thisId.fIdentifier = "pinvDamp";
  c4_y.pinvDamp = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c4_u, "pinvDamp", "pinvDamp", 0)), &c4_thisId);
  c4_thisId.fIdentifier = "pinvDampVb";
  c4_y.pinvDampVb = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c4_u, "pinvDampVb", "pinvDampVb", 0)), &c4_thisId);
  c4_thisId.fIdentifier = "HessianQP";
  c4_y.HessianQP = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c4_u, "HessianQP", "HessianQP", 0)), &c4_thisId);
  c4_thisId.fIdentifier = "impedances";
  c4_y.impedances = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c4_u, "impedances", "impedances", 0)), &c4_thisId);
  c4_thisId.fIdentifier = "dampings";
  c4_y.dampings = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c4_u, "dampings", "dampings", 0)), &c4_thisId);
  c4_thisId.fIdentifier = "norm_tolerance";
  c4_y.norm_tolerance = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c4_u, "norm_tolerance", "norm_tolerance", 0)), &c4_thisId);
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static real_T c4_d_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_y;
  real_T c4_d0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d0, 1, 0, 0U, 0, 0U, 0);
  c4_y = c4_d0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_reg;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  c4_struct_1ZGMVR6bgCMpDdXTSGnu6G c4_y;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_b_reg = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_reg), &c4_thisId);
  sf_mex_destroy(&c4_b_reg);
  *(c4_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i90;
  int32_T c4_i91;
  int32_T c4_i92;
  real_T c4_b_inData[174];
  int32_T c4_i93;
  int32_T c4_i94;
  int32_T c4_i95;
  real_T c4_u[174];
  const mxArray *c4_y = NULL;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i90 = 0;
  for (c4_i91 = 0; c4_i91 < 29; c4_i91++) {
    for (c4_i92 = 0; c4_i92 < 6; c4_i92++) {
      c4_b_inData[c4_i92 + c4_i90] = (*(real_T (*)[174])c4_inData)[c4_i92 +
        c4_i90];
    }

    c4_i90 += 6;
  }

  c4_i93 = 0;
  for (c4_i94 = 0; c4_i94 < 29; c4_i94++) {
    for (c4_i95 = 0; c4_i95 < 6; c4_i95++) {
      c4_u[c4_i95 + c4_i93] = c4_b_inData[c4_i95 + c4_i93];
    }

    c4_i93 += 6;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 6, 29), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i96;
  real_T c4_b_inData[23];
  int32_T c4_i97;
  real_T c4_u[23];
  const mxArray *c4_y = NULL;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i96 = 0; c4_i96 < 23; c4_i96++) {
    c4_b_inData[c4_i96] = (*(real_T (*)[23])c4_inData)[c4_i96];
  }

  for (c4_i97 = 0; c4_i97 < 23; c4_i97++) {
    c4_u[c4_i97] = c4_b_inData[c4_i97];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_nargout;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_nargout = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_nargout), &c4_thisId);
  sf_mex_destroy(&c4_nargout);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_g_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i98;
  int32_T c4_i99;
  int32_T c4_i100;
  real_T c4_b_inData[36];
  int32_T c4_i101;
  int32_T c4_i102;
  int32_T c4_i103;
  real_T c4_u[36];
  const mxArray *c4_y = NULL;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i98 = 0;
  for (c4_i99 = 0; c4_i99 < 6; c4_i99++) {
    for (c4_i100 = 0; c4_i100 < 6; c4_i100++) {
      c4_b_inData[c4_i100 + c4_i98] = (*(real_T (*)[36])c4_inData)[c4_i100 +
        c4_i98];
    }

    c4_i98 += 6;
  }

  c4_i101 = 0;
  for (c4_i102 = 0; c4_i102 < 6; c4_i102++) {
    for (c4_i103 = 0; c4_i103 < 6; c4_i103++) {
      c4_u[c4_i103 + c4_i101] = c4_b_inData[c4_i103 + c4_i101];
    }

    c4_i101 += 6;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 6, 6), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static void c4_e_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[36])
{
  real_T c4_dv4[36];
  int32_T c4_i104;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv4, 1, 0, 0U, 1, 0U, 2, 6, 6);
  for (c4_i104 = 0; c4_i104 < 36; c4_i104++) {
    c4_y[c4_i104] = c4_dv4[c4_i104];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_pinvJb;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[36];
  int32_T c4_i105;
  int32_T c4_i106;
  int32_T c4_i107;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_pinvJb = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_pinvJb), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_pinvJb);
  c4_i105 = 0;
  for (c4_i106 = 0; c4_i106 < 6; c4_i106++) {
    for (c4_i107 = 0; c4_i107 < 6; c4_i107++) {
      (*(real_T (*)[36])c4_outData)[c4_i107 + c4_i105] = c4_y[c4_i107 + c4_i105];
    }

    c4_i105 += 6;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray *sf_c4_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo;
  c4_ResolvedFunctionInfo c4_info[125];
  const mxArray *c4_m1 = NULL;
  int32_T c4_i108;
  c4_ResolvedFunctionInfo *c4_r1;
  c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  c4_info_helper(c4_info);
  c4_b_info_helper(c4_info);
  sf_mex_assign(&c4_m1, sf_mex_createstruct("nameCaptureInfo", 1, 125), FALSE);
  for (c4_i108 = 0; c4_i108 < 125; c4_i108++) {
    c4_r1 = &c4_info[c4_i108];
    sf_mex_addfield(c4_m1, sf_mex_create("nameCaptureInfo", c4_r1->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r1->context)), "context", "nameCaptureInfo",
                    c4_i108);
    sf_mex_addfield(c4_m1, sf_mex_create("nameCaptureInfo", c4_r1->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c4_r1->name)), "name", "nameCaptureInfo", c4_i108);
    sf_mex_addfield(c4_m1, sf_mex_create("nameCaptureInfo", c4_r1->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c4_r1->dominantType)), "dominantType",
                    "nameCaptureInfo", c4_i108);
    sf_mex_addfield(c4_m1, sf_mex_create("nameCaptureInfo", c4_r1->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r1->resolved)), "resolved", "nameCaptureInfo",
                    c4_i108);
    sf_mex_addfield(c4_m1, sf_mex_create("nameCaptureInfo", &c4_r1->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c4_i108);
    sf_mex_addfield(c4_m1, sf_mex_create("nameCaptureInfo", &c4_r1->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c4_i108);
    sf_mex_addfield(c4_m1, sf_mex_create("nameCaptureInfo", &c4_r1->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c4_i108);
    sf_mex_addfield(c4_m1, sf_mex_create("nameCaptureInfo", &c4_r1->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c4_i108);
  }

  sf_mex_assign(&c4_nameCaptureInfo, c4_m1, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c4_nameCaptureInfo);
  return c4_nameCaptureInfo;
}

static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[125])
{
  c4_info[0].context = "";
  c4_info[0].name = "mtimes";
  c4_info[0].dominantType = "double";
  c4_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[0].fileTimeLo = 1289519692U;
  c4_info[0].fileTimeHi = 0U;
  c4_info[0].mFileTimeLo = 0U;
  c4_info[0].mFileTimeHi = 0U;
  c4_info[1].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[1].name = "eml_index_class";
  c4_info[1].dominantType = "";
  c4_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[1].fileTimeLo = 1323170578U;
  c4_info[1].fileTimeHi = 0U;
  c4_info[1].mFileTimeLo = 0U;
  c4_info[1].mFileTimeHi = 0U;
  c4_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[2].name = "eml_scalar_eg";
  c4_info[2].dominantType = "double";
  c4_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[2].fileTimeLo = 1286818796U;
  c4_info[2].fileTimeHi = 0U;
  c4_info[2].mFileTimeLo = 0U;
  c4_info[2].mFileTimeHi = 0U;
  c4_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[3].name = "eml_xgemm";
  c4_info[3].dominantType = "char";
  c4_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c4_info[3].fileTimeLo = 1299076772U;
  c4_info[3].fileTimeHi = 0U;
  c4_info[3].mFileTimeLo = 0U;
  c4_info[3].mFileTimeHi = 0U;
  c4_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c4_info[4].name = "eml_blas_inline";
  c4_info[4].dominantType = "";
  c4_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[4].fileTimeLo = 1299076768U;
  c4_info[4].fileTimeHi = 0U;
  c4_info[4].mFileTimeLo = 0U;
  c4_info[4].mFileTimeHi = 0U;
  c4_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c4_info[5].name = "mtimes";
  c4_info[5].dominantType = "double";
  c4_info[5].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[5].fileTimeLo = 1289519692U;
  c4_info[5].fileTimeHi = 0U;
  c4_info[5].mFileTimeLo = 0U;
  c4_info[5].mFileTimeHi = 0U;
  c4_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c4_info[6].name = "eml_index_class";
  c4_info[6].dominantType = "";
  c4_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[6].fileTimeLo = 1323170578U;
  c4_info[6].fileTimeHi = 0U;
  c4_info[6].mFileTimeLo = 0U;
  c4_info[6].mFileTimeHi = 0U;
  c4_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c4_info[7].name = "eml_scalar_eg";
  c4_info[7].dominantType = "double";
  c4_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[7].fileTimeLo = 1286818796U;
  c4_info[7].fileTimeHi = 0U;
  c4_info[7].mFileTimeLo = 0U;
  c4_info[7].mFileTimeHi = 0U;
  c4_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c4_info[8].name = "eml_refblas_xgemm";
  c4_info[8].dominantType = "char";
  c4_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c4_info[8].fileTimeLo = 1299076774U;
  c4_info[8].fileTimeHi = 0U;
  c4_info[8].mFileTimeLo = 0U;
  c4_info[8].mFileTimeHi = 0U;
  c4_info[9].context = "";
  c4_info[9].name = "eye";
  c4_info[9].dominantType = "double";
  c4_info[9].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c4_info[9].fileTimeLo = 1286818688U;
  c4_info[9].fileTimeHi = 0U;
  c4_info[9].mFileTimeLo = 0U;
  c4_info[9].mFileTimeHi = 0U;
  c4_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c4_info[10].name = "eml_assert_valid_size_arg";
  c4_info[10].dominantType = "double";
  c4_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c4_info[10].fileTimeLo = 1286818694U;
  c4_info[10].fileTimeHi = 0U;
  c4_info[10].mFileTimeLo = 0U;
  c4_info[10].mFileTimeHi = 0U;
  c4_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c4_info[11].name = "isinf";
  c4_info[11].dominantType = "double";
  c4_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c4_info[11].fileTimeLo = 1286818760U;
  c4_info[11].fileTimeHi = 0U;
  c4_info[11].mFileTimeLo = 0U;
  c4_info[11].mFileTimeHi = 0U;
  c4_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c4_info[12].name = "mtimes";
  c4_info[12].dominantType = "double";
  c4_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[12].fileTimeLo = 1289519692U;
  c4_info[12].fileTimeHi = 0U;
  c4_info[12].mFileTimeLo = 0U;
  c4_info[12].mFileTimeHi = 0U;
  c4_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c4_info[13].name = "eml_index_class";
  c4_info[13].dominantType = "";
  c4_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[13].fileTimeLo = 1323170578U;
  c4_info[13].fileTimeHi = 0U;
  c4_info[13].mFileTimeLo = 0U;
  c4_info[13].mFileTimeHi = 0U;
  c4_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c4_info[14].name = "intmax";
  c4_info[14].dominantType = "char";
  c4_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c4_info[14].fileTimeLo = 1311255316U;
  c4_info[14].fileTimeHi = 0U;
  c4_info[14].mFileTimeLo = 0U;
  c4_info[14].mFileTimeHi = 0U;
  c4_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c4_info[15].name = "eml_is_float_class";
  c4_info[15].dominantType = "char";
  c4_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c4_info[15].fileTimeLo = 1286818782U;
  c4_info[15].fileTimeHi = 0U;
  c4_info[15].mFileTimeLo = 0U;
  c4_info[15].mFileTimeHi = 0U;
  c4_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c4_info[16].name = "min";
  c4_info[16].dominantType = "double";
  c4_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[16].fileTimeLo = 1311255318U;
  c4_info[16].fileTimeHi = 0U;
  c4_info[16].mFileTimeLo = 0U;
  c4_info[16].mFileTimeHi = 0U;
  c4_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[17].name = "eml_min_or_max";
  c4_info[17].dominantType = "char";
  c4_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c4_info[17].fileTimeLo = 1334071490U;
  c4_info[17].fileTimeHi = 0U;
  c4_info[17].mFileTimeLo = 0U;
  c4_info[17].mFileTimeHi = 0U;
  c4_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[18].name = "eml_scalar_eg";
  c4_info[18].dominantType = "double";
  c4_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[18].fileTimeLo = 1286818796U;
  c4_info[18].fileTimeHi = 0U;
  c4_info[18].mFileTimeLo = 0U;
  c4_info[18].mFileTimeHi = 0U;
  c4_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[19].name = "eml_scalexp_alloc";
  c4_info[19].dominantType = "double";
  c4_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[19].fileTimeLo = 1352424860U;
  c4_info[19].fileTimeHi = 0U;
  c4_info[19].mFileTimeLo = 0U;
  c4_info[19].mFileTimeHi = 0U;
  c4_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[20].name = "eml_index_class";
  c4_info[20].dominantType = "";
  c4_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[20].fileTimeLo = 1323170578U;
  c4_info[20].fileTimeHi = 0U;
  c4_info[20].mFileTimeLo = 0U;
  c4_info[20].mFileTimeHi = 0U;
  c4_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c4_info[21].name = "eml_scalar_eg";
  c4_info[21].dominantType = "double";
  c4_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[21].fileTimeLo = 1286818796U;
  c4_info[21].fileTimeHi = 0U;
  c4_info[21].mFileTimeLo = 0U;
  c4_info[21].mFileTimeHi = 0U;
  c4_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c4_info[22].name = "eml_index_class";
  c4_info[22].dominantType = "";
  c4_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[22].fileTimeLo = 1323170578U;
  c4_info[22].fileTimeHi = 0U;
  c4_info[22].mFileTimeLo = 0U;
  c4_info[22].mFileTimeHi = 0U;
  c4_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c4_info[23].name = "eml_int_forloop_overflow_check";
  c4_info[23].dominantType = "";
  c4_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[23].fileTimeLo = 1346510340U;
  c4_info[23].fileTimeHi = 0U;
  c4_info[23].mFileTimeLo = 0U;
  c4_info[23].mFileTimeHi = 0U;
  c4_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c4_info[24].name = "intmax";
  c4_info[24].dominantType = "char";
  c4_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c4_info[24].fileTimeLo = 1311255316U;
  c4_info[24].fileTimeHi = 0U;
  c4_info[24].mFileTimeLo = 0U;
  c4_info[24].mFileTimeHi = 0U;
  c4_info[25].context = "";
  c4_info[25].name = "mldivide";
  c4_info[25].dominantType = "double";
  c4_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c4_info[25].fileTimeLo = 1357951548U;
  c4_info[25].fileTimeHi = 0U;
  c4_info[25].mFileTimeLo = 1319729966U;
  c4_info[25].mFileTimeHi = 0U;
  c4_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c4_info[26].name = "eml_lusolve";
  c4_info[26].dominantType = "double";
  c4_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c4_info[26].fileTimeLo = 1309451196U;
  c4_info[26].fileTimeHi = 0U;
  c4_info[26].mFileTimeLo = 0U;
  c4_info[26].mFileTimeHi = 0U;
  c4_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c4_info[27].name = "eml_index_class";
  c4_info[27].dominantType = "";
  c4_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[27].fileTimeLo = 1323170578U;
  c4_info[27].fileTimeHi = 0U;
  c4_info[27].mFileTimeLo = 0U;
  c4_info[27].mFileTimeHi = 0U;
  c4_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c4_info[28].name = "eml_index_class";
  c4_info[28].dominantType = "";
  c4_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[28].fileTimeLo = 1323170578U;
  c4_info[28].fileTimeHi = 0U;
  c4_info[28].mFileTimeLo = 0U;
  c4_info[28].mFileTimeHi = 0U;
  c4_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c4_info[29].name = "eml_xgetrf";
  c4_info[29].dominantType = "double";
  c4_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c4_info[29].fileTimeLo = 1286818806U;
  c4_info[29].fileTimeHi = 0U;
  c4_info[29].mFileTimeLo = 0U;
  c4_info[29].mFileTimeHi = 0U;
  c4_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c4_info[30].name = "eml_lapack_xgetrf";
  c4_info[30].dominantType = "double";
  c4_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c4_info[30].fileTimeLo = 1286818810U;
  c4_info[30].fileTimeHi = 0U;
  c4_info[30].mFileTimeLo = 0U;
  c4_info[30].mFileTimeHi = 0U;
  c4_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c4_info[31].name = "eml_matlab_zgetrf";
  c4_info[31].dominantType = "double";
  c4_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[31].fileTimeLo = 1302688994U;
  c4_info[31].fileTimeHi = 0U;
  c4_info[31].mFileTimeLo = 0U;
  c4_info[31].mFileTimeHi = 0U;
  c4_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[32].name = "realmin";
  c4_info[32].dominantType = "char";
  c4_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c4_info[32].fileTimeLo = 1307651242U;
  c4_info[32].fileTimeHi = 0U;
  c4_info[32].mFileTimeLo = 0U;
  c4_info[32].mFileTimeHi = 0U;
  c4_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c4_info[33].name = "eml_realmin";
  c4_info[33].dominantType = "char";
  c4_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c4_info[33].fileTimeLo = 1307651244U;
  c4_info[33].fileTimeHi = 0U;
  c4_info[33].mFileTimeLo = 0U;
  c4_info[33].mFileTimeHi = 0U;
  c4_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c4_info[34].name = "eml_float_model";
  c4_info[34].dominantType = "char";
  c4_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c4_info[34].fileTimeLo = 1326727996U;
  c4_info[34].fileTimeHi = 0U;
  c4_info[34].mFileTimeLo = 0U;
  c4_info[34].mFileTimeHi = 0U;
  c4_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[35].name = "eps";
  c4_info[35].dominantType = "char";
  c4_info[35].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[35].fileTimeLo = 1326727996U;
  c4_info[35].fileTimeHi = 0U;
  c4_info[35].mFileTimeLo = 0U;
  c4_info[35].mFileTimeHi = 0U;
  c4_info[36].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[36].name = "eml_is_float_class";
  c4_info[36].dominantType = "char";
  c4_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c4_info[36].fileTimeLo = 1286818782U;
  c4_info[36].fileTimeHi = 0U;
  c4_info[36].mFileTimeLo = 0U;
  c4_info[36].mFileTimeHi = 0U;
  c4_info[37].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[37].name = "eml_eps";
  c4_info[37].dominantType = "char";
  c4_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c4_info[37].fileTimeLo = 1326727996U;
  c4_info[37].fileTimeHi = 0U;
  c4_info[37].mFileTimeLo = 0U;
  c4_info[37].mFileTimeHi = 0U;
  c4_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c4_info[38].name = "eml_float_model";
  c4_info[38].dominantType = "char";
  c4_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c4_info[38].fileTimeLo = 1326727996U;
  c4_info[38].fileTimeHi = 0U;
  c4_info[38].mFileTimeLo = 0U;
  c4_info[38].mFileTimeHi = 0U;
  c4_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[39].name = "min";
  c4_info[39].dominantType = "coder.internal.indexInt";
  c4_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[39].fileTimeLo = 1311255318U;
  c4_info[39].fileTimeHi = 0U;
  c4_info[39].mFileTimeLo = 0U;
  c4_info[39].mFileTimeHi = 0U;
  c4_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[40].name = "eml_scalar_eg";
  c4_info[40].dominantType = "coder.internal.indexInt";
  c4_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[40].fileTimeLo = 1286818796U;
  c4_info[40].fileTimeHi = 0U;
  c4_info[40].mFileTimeLo = 0U;
  c4_info[40].mFileTimeHi = 0U;
  c4_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[41].name = "eml_scalexp_alloc";
  c4_info[41].dominantType = "coder.internal.indexInt";
  c4_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[41].fileTimeLo = 1352424860U;
  c4_info[41].fileTimeHi = 0U;
  c4_info[41].mFileTimeLo = 0U;
  c4_info[41].mFileTimeHi = 0U;
  c4_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c4_info[42].name = "eml_scalar_eg";
  c4_info[42].dominantType = "coder.internal.indexInt";
  c4_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[42].fileTimeLo = 1286818796U;
  c4_info[42].fileTimeHi = 0U;
  c4_info[42].mFileTimeLo = 0U;
  c4_info[42].mFileTimeHi = 0U;
  c4_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[43].name = "colon";
  c4_info[43].dominantType = "double";
  c4_info[43].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c4_info[43].fileTimeLo = 1348191928U;
  c4_info[43].fileTimeHi = 0U;
  c4_info[43].mFileTimeLo = 0U;
  c4_info[43].mFileTimeHi = 0U;
  c4_info[44].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c4_info[44].name = "colon";
  c4_info[44].dominantType = "double";
  c4_info[44].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c4_info[44].fileTimeLo = 1348191928U;
  c4_info[44].fileTimeHi = 0U;
  c4_info[44].mFileTimeLo = 0U;
  c4_info[44].mFileTimeHi = 0U;
  c4_info[45].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c4_info[45].name = "floor";
  c4_info[45].dominantType = "double";
  c4_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c4_info[45].fileTimeLo = 1343830380U;
  c4_info[45].fileTimeHi = 0U;
  c4_info[45].mFileTimeLo = 0U;
  c4_info[45].mFileTimeHi = 0U;
  c4_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c4_info[46].name = "eml_scalar_floor";
  c4_info[46].dominantType = "double";
  c4_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c4_info[46].fileTimeLo = 1286818726U;
  c4_info[46].fileTimeHi = 0U;
  c4_info[46].mFileTimeLo = 0U;
  c4_info[46].mFileTimeHi = 0U;
  c4_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c4_info[47].name = "intmin";
  c4_info[47].dominantType = "char";
  c4_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c4_info[47].fileTimeLo = 1311255318U;
  c4_info[47].fileTimeHi = 0U;
  c4_info[47].mFileTimeLo = 0U;
  c4_info[47].mFileTimeHi = 0U;
  c4_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c4_info[48].name = "intmax";
  c4_info[48].dominantType = "char";
  c4_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c4_info[48].fileTimeLo = 1311255316U;
  c4_info[48].fileTimeHi = 0U;
  c4_info[48].mFileTimeLo = 0U;
  c4_info[48].mFileTimeHi = 0U;
  c4_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c4_info[49].name = "intmin";
  c4_info[49].dominantType = "char";
  c4_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c4_info[49].fileTimeLo = 1311255318U;
  c4_info[49].fileTimeHi = 0U;
  c4_info[49].mFileTimeLo = 0U;
  c4_info[49].mFileTimeHi = 0U;
  c4_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c4_info[50].name = "intmax";
  c4_info[50].dominantType = "char";
  c4_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c4_info[50].fileTimeLo = 1311255316U;
  c4_info[50].fileTimeHi = 0U;
  c4_info[50].mFileTimeLo = 0U;
  c4_info[50].mFileTimeHi = 0U;
  c4_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c4_info[51].name = "eml_isa_uint";
  c4_info[51].dominantType = "coder.internal.indexInt";
  c4_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c4_info[51].fileTimeLo = 1286818784U;
  c4_info[51].fileTimeHi = 0U;
  c4_info[51].mFileTimeLo = 0U;
  c4_info[51].mFileTimeHi = 0U;
  c4_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c4_info[52].name = "eml_unsigned_class";
  c4_info[52].dominantType = "char";
  c4_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c4_info[52].fileTimeLo = 1323170580U;
  c4_info[52].fileTimeHi = 0U;
  c4_info[52].mFileTimeLo = 0U;
  c4_info[52].mFileTimeHi = 0U;
  c4_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c4_info[53].name = "eml_index_class";
  c4_info[53].dominantType = "";
  c4_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[53].fileTimeLo = 1323170578U;
  c4_info[53].fileTimeHi = 0U;
  c4_info[53].mFileTimeLo = 0U;
  c4_info[53].mFileTimeHi = 0U;
  c4_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c4_info[54].name = "eml_index_class";
  c4_info[54].dominantType = "";
  c4_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[54].fileTimeLo = 1323170578U;
  c4_info[54].fileTimeHi = 0U;
  c4_info[54].mFileTimeLo = 0U;
  c4_info[54].mFileTimeHi = 0U;
  c4_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c4_info[55].name = "intmax";
  c4_info[55].dominantType = "char";
  c4_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c4_info[55].fileTimeLo = 1311255316U;
  c4_info[55].fileTimeHi = 0U;
  c4_info[55].mFileTimeLo = 0U;
  c4_info[55].mFileTimeHi = 0U;
  c4_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c4_info[56].name = "eml_isa_uint";
  c4_info[56].dominantType = "coder.internal.indexInt";
  c4_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c4_info[56].fileTimeLo = 1286818784U;
  c4_info[56].fileTimeHi = 0U;
  c4_info[56].mFileTimeLo = 0U;
  c4_info[56].mFileTimeHi = 0U;
  c4_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c4_info[57].name = "eml_index_plus";
  c4_info[57].dominantType = "double";
  c4_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[57].fileTimeLo = 1286818778U;
  c4_info[57].fileTimeHi = 0U;
  c4_info[57].mFileTimeLo = 0U;
  c4_info[57].mFileTimeHi = 0U;
  c4_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[58].name = "eml_index_class";
  c4_info[58].dominantType = "";
  c4_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[58].fileTimeLo = 1323170578U;
  c4_info[58].fileTimeHi = 0U;
  c4_info[58].mFileTimeLo = 0U;
  c4_info[58].mFileTimeHi = 0U;
  c4_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c4_info[59].name = "eml_int_forloop_overflow_check";
  c4_info[59].dominantType = "";
  c4_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[59].fileTimeLo = 1346510340U;
  c4_info[59].fileTimeHi = 0U;
  c4_info[59].mFileTimeLo = 0U;
  c4_info[59].mFileTimeHi = 0U;
  c4_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[60].name = "eml_index_class";
  c4_info[60].dominantType = "";
  c4_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[60].fileTimeLo = 1323170578U;
  c4_info[60].fileTimeHi = 0U;
  c4_info[60].mFileTimeLo = 0U;
  c4_info[60].mFileTimeHi = 0U;
  c4_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[61].name = "eml_index_plus";
  c4_info[61].dominantType = "double";
  c4_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[61].fileTimeLo = 1286818778U;
  c4_info[61].fileTimeHi = 0U;
  c4_info[61].mFileTimeLo = 0U;
  c4_info[61].mFileTimeHi = 0U;
  c4_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[62].name = "eml_int_forloop_overflow_check";
  c4_info[62].dominantType = "";
  c4_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[62].fileTimeLo = 1346510340U;
  c4_info[62].fileTimeHi = 0U;
  c4_info[62].mFileTimeLo = 0U;
  c4_info[62].mFileTimeHi = 0U;
  c4_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[63].name = "eml_index_minus";
  c4_info[63].dominantType = "double";
  c4_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[63].fileTimeLo = 1286818778U;
  c4_info[63].fileTimeHi = 0U;
  c4_info[63].mFileTimeLo = 0U;
  c4_info[63].mFileTimeHi = 0U;
}

static void c4_b_info_helper(c4_ResolvedFunctionInfo c4_info[125])
{
  c4_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[64].name = "eml_index_class";
  c4_info[64].dominantType = "";
  c4_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[64].fileTimeLo = 1323170578U;
  c4_info[64].fileTimeHi = 0U;
  c4_info[64].mFileTimeLo = 0U;
  c4_info[64].mFileTimeHi = 0U;
  c4_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[65].name = "eml_index_minus";
  c4_info[65].dominantType = "coder.internal.indexInt";
  c4_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[65].fileTimeLo = 1286818778U;
  c4_info[65].fileTimeHi = 0U;
  c4_info[65].mFileTimeLo = 0U;
  c4_info[65].mFileTimeHi = 0U;
  c4_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[66].name = "eml_index_times";
  c4_info[66].dominantType = "coder.internal.indexInt";
  c4_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c4_info[66].fileTimeLo = 1286818780U;
  c4_info[66].fileTimeHi = 0U;
  c4_info[66].mFileTimeLo = 0U;
  c4_info[66].mFileTimeHi = 0U;
  c4_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c4_info[67].name = "eml_index_class";
  c4_info[67].dominantType = "";
  c4_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[67].fileTimeLo = 1323170578U;
  c4_info[67].fileTimeHi = 0U;
  c4_info[67].mFileTimeLo = 0U;
  c4_info[67].mFileTimeHi = 0U;
  c4_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[68].name = "eml_index_plus";
  c4_info[68].dominantType = "coder.internal.indexInt";
  c4_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[68].fileTimeLo = 1286818778U;
  c4_info[68].fileTimeHi = 0U;
  c4_info[68].mFileTimeLo = 0U;
  c4_info[68].mFileTimeHi = 0U;
  c4_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[69].name = "eml_ixamax";
  c4_info[69].dominantType = "double";
  c4_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c4_info[69].fileTimeLo = 1299076770U;
  c4_info[69].fileTimeHi = 0U;
  c4_info[69].mFileTimeLo = 0U;
  c4_info[69].mFileTimeHi = 0U;
  c4_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c4_info[70].name = "eml_blas_inline";
  c4_info[70].dominantType = "";
  c4_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[70].fileTimeLo = 1299076768U;
  c4_info[70].fileTimeHi = 0U;
  c4_info[70].mFileTimeLo = 0U;
  c4_info[70].mFileTimeHi = 0U;
  c4_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c4_info[71].name = "length";
  c4_info[71].dominantType = "double";
  c4_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c4_info[71].fileTimeLo = 1303146206U;
  c4_info[71].fileTimeHi = 0U;
  c4_info[71].mFileTimeLo = 0U;
  c4_info[71].mFileTimeHi = 0U;
  c4_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c4_info[72].name = "eml_index_class";
  c4_info[72].dominantType = "";
  c4_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[72].fileTimeLo = 1323170578U;
  c4_info[72].fileTimeHi = 0U;
  c4_info[72].mFileTimeLo = 0U;
  c4_info[72].mFileTimeHi = 0U;
  c4_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c4_info[73].name = "eml_index_class";
  c4_info[73].dominantType = "";
  c4_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[73].fileTimeLo = 1323170578U;
  c4_info[73].fileTimeHi = 0U;
  c4_info[73].mFileTimeLo = 0U;
  c4_info[73].mFileTimeHi = 0U;
  c4_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c4_info[74].name = "eml_refblas_ixamax";
  c4_info[74].dominantType = "double";
  c4_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c4_info[74].fileTimeLo = 1299076770U;
  c4_info[74].fileTimeHi = 0U;
  c4_info[74].mFileTimeLo = 0U;
  c4_info[74].mFileTimeHi = 0U;
  c4_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c4_info[75].name = "eml_index_class";
  c4_info[75].dominantType = "";
  c4_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[75].fileTimeLo = 1323170578U;
  c4_info[75].fileTimeHi = 0U;
  c4_info[75].mFileTimeLo = 0U;
  c4_info[75].mFileTimeHi = 0U;
  c4_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c4_info[76].name = "eml_xcabs1";
  c4_info[76].dominantType = "double";
  c4_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c4_info[76].fileTimeLo = 1286818706U;
  c4_info[76].fileTimeHi = 0U;
  c4_info[76].mFileTimeLo = 0U;
  c4_info[76].mFileTimeHi = 0U;
  c4_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c4_info[77].name = "abs";
  c4_info[77].dominantType = "double";
  c4_info[77].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[77].fileTimeLo = 1343830366U;
  c4_info[77].fileTimeHi = 0U;
  c4_info[77].mFileTimeLo = 0U;
  c4_info[77].mFileTimeHi = 0U;
  c4_info[78].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[78].name = "eml_scalar_abs";
  c4_info[78].dominantType = "double";
  c4_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c4_info[78].fileTimeLo = 1286818712U;
  c4_info[78].fileTimeHi = 0U;
  c4_info[78].mFileTimeLo = 0U;
  c4_info[78].mFileTimeHi = 0U;
  c4_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c4_info[79].name = "eml_int_forloop_overflow_check";
  c4_info[79].dominantType = "";
  c4_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[79].fileTimeLo = 1346510340U;
  c4_info[79].fileTimeHi = 0U;
  c4_info[79].mFileTimeLo = 0U;
  c4_info[79].mFileTimeHi = 0U;
  c4_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c4_info[80].name = "eml_index_plus";
  c4_info[80].dominantType = "coder.internal.indexInt";
  c4_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[80].fileTimeLo = 1286818778U;
  c4_info[80].fileTimeHi = 0U;
  c4_info[80].mFileTimeLo = 0U;
  c4_info[80].mFileTimeHi = 0U;
  c4_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[81].name = "eml_xswap";
  c4_info[81].dominantType = "double";
  c4_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c4_info[81].fileTimeLo = 1299076778U;
  c4_info[81].fileTimeHi = 0U;
  c4_info[81].mFileTimeLo = 0U;
  c4_info[81].mFileTimeHi = 0U;
  c4_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c4_info[82].name = "eml_blas_inline";
  c4_info[82].dominantType = "";
  c4_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[82].fileTimeLo = 1299076768U;
  c4_info[82].fileTimeHi = 0U;
  c4_info[82].mFileTimeLo = 0U;
  c4_info[82].mFileTimeHi = 0U;
  c4_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c4_info[83].name = "eml_index_class";
  c4_info[83].dominantType = "";
  c4_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[83].fileTimeLo = 1323170578U;
  c4_info[83].fileTimeHi = 0U;
  c4_info[83].mFileTimeLo = 0U;
  c4_info[83].mFileTimeHi = 0U;
  c4_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c4_info[84].name = "eml_refblas_xswap";
  c4_info[84].dominantType = "double";
  c4_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[84].fileTimeLo = 1299076786U;
  c4_info[84].fileTimeHi = 0U;
  c4_info[84].mFileTimeLo = 0U;
  c4_info[84].mFileTimeHi = 0U;
  c4_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[85].name = "eml_index_class";
  c4_info[85].dominantType = "";
  c4_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[85].fileTimeLo = 1323170578U;
  c4_info[85].fileTimeHi = 0U;
  c4_info[85].mFileTimeLo = 0U;
  c4_info[85].mFileTimeHi = 0U;
  c4_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[86].name = "abs";
  c4_info[86].dominantType = "coder.internal.indexInt";
  c4_info[86].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[86].fileTimeLo = 1343830366U;
  c4_info[86].fileTimeHi = 0U;
  c4_info[86].mFileTimeLo = 0U;
  c4_info[86].mFileTimeHi = 0U;
  c4_info[87].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[87].name = "eml_scalar_abs";
  c4_info[87].dominantType = "coder.internal.indexInt";
  c4_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c4_info[87].fileTimeLo = 1286818712U;
  c4_info[87].fileTimeHi = 0U;
  c4_info[87].mFileTimeLo = 0U;
  c4_info[87].mFileTimeHi = 0U;
  c4_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[88].name = "eml_int_forloop_overflow_check";
  c4_info[88].dominantType = "";
  c4_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[88].fileTimeLo = 1346510340U;
  c4_info[88].fileTimeHi = 0U;
  c4_info[88].mFileTimeLo = 0U;
  c4_info[88].mFileTimeHi = 0U;
  c4_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[89].name = "eml_index_plus";
  c4_info[89].dominantType = "coder.internal.indexInt";
  c4_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[89].fileTimeLo = 1286818778U;
  c4_info[89].fileTimeHi = 0U;
  c4_info[89].mFileTimeLo = 0U;
  c4_info[89].mFileTimeHi = 0U;
  c4_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[90].name = "eml_div";
  c4_info[90].dominantType = "double";
  c4_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c4_info[90].fileTimeLo = 1313347810U;
  c4_info[90].fileTimeHi = 0U;
  c4_info[90].mFileTimeLo = 0U;
  c4_info[90].mFileTimeHi = 0U;
  c4_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c4_info[91].name = "eml_xgeru";
  c4_info[91].dominantType = "double";
  c4_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c4_info[91].fileTimeLo = 1299076774U;
  c4_info[91].fileTimeHi = 0U;
  c4_info[91].mFileTimeLo = 0U;
  c4_info[91].mFileTimeHi = 0U;
  c4_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c4_info[92].name = "eml_blas_inline";
  c4_info[92].dominantType = "";
  c4_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[92].fileTimeLo = 1299076768U;
  c4_info[92].fileTimeHi = 0U;
  c4_info[92].mFileTimeLo = 0U;
  c4_info[92].mFileTimeHi = 0U;
  c4_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c4_info[93].name = "eml_xger";
  c4_info[93].dominantType = "double";
  c4_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c4_info[93].fileTimeLo = 1299076774U;
  c4_info[93].fileTimeHi = 0U;
  c4_info[93].mFileTimeLo = 0U;
  c4_info[93].mFileTimeHi = 0U;
  c4_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c4_info[94].name = "eml_blas_inline";
  c4_info[94].dominantType = "";
  c4_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[94].fileTimeLo = 1299076768U;
  c4_info[94].fileTimeHi = 0U;
  c4_info[94].mFileTimeLo = 0U;
  c4_info[94].mFileTimeHi = 0U;
  c4_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c4_info[95].name = "intmax";
  c4_info[95].dominantType = "char";
  c4_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c4_info[95].fileTimeLo = 1311255316U;
  c4_info[95].fileTimeHi = 0U;
  c4_info[95].mFileTimeLo = 0U;
  c4_info[95].mFileTimeHi = 0U;
  c4_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c4_info[96].name = "min";
  c4_info[96].dominantType = "double";
  c4_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[96].fileTimeLo = 1311255318U;
  c4_info[96].fileTimeHi = 0U;
  c4_info[96].mFileTimeLo = 0U;
  c4_info[96].mFileTimeHi = 0U;
  c4_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c4_info[97].name = "mtimes";
  c4_info[97].dominantType = "double";
  c4_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[97].fileTimeLo = 1289519692U;
  c4_info[97].fileTimeHi = 0U;
  c4_info[97].mFileTimeLo = 0U;
  c4_info[97].mFileTimeHi = 0U;
  c4_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c4_info[98].name = "eml_index_class";
  c4_info[98].dominantType = "";
  c4_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[98].fileTimeLo = 1323170578U;
  c4_info[98].fileTimeHi = 0U;
  c4_info[98].mFileTimeLo = 0U;
  c4_info[98].mFileTimeHi = 0U;
  c4_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c4_info[99].name = "eml_refblas_xger";
  c4_info[99].dominantType = "double";
  c4_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c4_info[99].fileTimeLo = 1299076776U;
  c4_info[99].fileTimeHi = 0U;
  c4_info[99].mFileTimeLo = 0U;
  c4_info[99].mFileTimeHi = 0U;
  c4_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c4_info[100].name = "eml_refblas_xgerx";
  c4_info[100].dominantType = "char";
  c4_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c4_info[100].fileTimeLo = 1299076778U;
  c4_info[100].fileTimeHi = 0U;
  c4_info[100].mFileTimeLo = 0U;
  c4_info[100].mFileTimeHi = 0U;
  c4_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c4_info[101].name = "eml_index_class";
  c4_info[101].dominantType = "";
  c4_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[101].fileTimeLo = 1323170578U;
  c4_info[101].fileTimeHi = 0U;
  c4_info[101].mFileTimeLo = 0U;
  c4_info[101].mFileTimeHi = 0U;
  c4_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c4_info[102].name = "abs";
  c4_info[102].dominantType = "coder.internal.indexInt";
  c4_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[102].fileTimeLo = 1343830366U;
  c4_info[102].fileTimeHi = 0U;
  c4_info[102].mFileTimeLo = 0U;
  c4_info[102].mFileTimeHi = 0U;
  c4_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c4_info[103].name = "eml_index_minus";
  c4_info[103].dominantType = "double";
  c4_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[103].fileTimeLo = 1286818778U;
  c4_info[103].fileTimeHi = 0U;
  c4_info[103].mFileTimeLo = 0U;
  c4_info[103].mFileTimeHi = 0U;
  c4_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c4_info[104].name = "eml_int_forloop_overflow_check";
  c4_info[104].dominantType = "";
  c4_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[104].fileTimeLo = 1346510340U;
  c4_info[104].fileTimeHi = 0U;
  c4_info[104].mFileTimeLo = 0U;
  c4_info[104].mFileTimeHi = 0U;
  c4_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c4_info[105].name = "eml_index_plus";
  c4_info[105].dominantType = "double";
  c4_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[105].fileTimeLo = 1286818778U;
  c4_info[105].fileTimeHi = 0U;
  c4_info[105].mFileTimeLo = 0U;
  c4_info[105].mFileTimeHi = 0U;
  c4_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c4_info[106].name = "eml_index_plus";
  c4_info[106].dominantType = "coder.internal.indexInt";
  c4_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[106].fileTimeLo = 1286818778U;
  c4_info[106].fileTimeHi = 0U;
  c4_info[106].mFileTimeLo = 0U;
  c4_info[106].mFileTimeHi = 0U;
  c4_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c4_info[107].name = "eml_warning";
  c4_info[107].dominantType = "char";
  c4_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c4_info[107].fileTimeLo = 1286818802U;
  c4_info[107].fileTimeHi = 0U;
  c4_info[107].mFileTimeLo = 0U;
  c4_info[107].mFileTimeHi = 0U;
  c4_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c4_info[108].name = "eml_scalar_eg";
  c4_info[108].dominantType = "double";
  c4_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[108].fileTimeLo = 1286818796U;
  c4_info[108].fileTimeHi = 0U;
  c4_info[108].mFileTimeLo = 0U;
  c4_info[108].mFileTimeHi = 0U;
  c4_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c4_info[109].name = "eml_int_forloop_overflow_check";
  c4_info[109].dominantType = "";
  c4_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[109].fileTimeLo = 1346510340U;
  c4_info[109].fileTimeHi = 0U;
  c4_info[109].mFileTimeLo = 0U;
  c4_info[109].mFileTimeHi = 0U;
  c4_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c4_info[110].name = "eml_xtrsm";
  c4_info[110].dominantType = "char";
  c4_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c4_info[110].fileTimeLo = 1299076778U;
  c4_info[110].fileTimeHi = 0U;
  c4_info[110].mFileTimeLo = 0U;
  c4_info[110].mFileTimeHi = 0U;
  c4_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c4_info[111].name = "eml_blas_inline";
  c4_info[111].dominantType = "";
  c4_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[111].fileTimeLo = 1299076768U;
  c4_info[111].fileTimeHi = 0U;
  c4_info[111].mFileTimeLo = 0U;
  c4_info[111].mFileTimeHi = 0U;
  c4_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c4_info[112].name = "mtimes";
  c4_info[112].dominantType = "double";
  c4_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[112].fileTimeLo = 1289519692U;
  c4_info[112].fileTimeHi = 0U;
  c4_info[112].mFileTimeLo = 0U;
  c4_info[112].mFileTimeHi = 0U;
  c4_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c4_info[113].name = "eml_index_class";
  c4_info[113].dominantType = "";
  c4_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[113].fileTimeLo = 1323170578U;
  c4_info[113].fileTimeHi = 0U;
  c4_info[113].mFileTimeLo = 0U;
  c4_info[113].mFileTimeHi = 0U;
  c4_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c4_info[114].name = "eml_scalar_eg";
  c4_info[114].dominantType = "double";
  c4_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[114].fileTimeLo = 1286818796U;
  c4_info[114].fileTimeHi = 0U;
  c4_info[114].mFileTimeLo = 0U;
  c4_info[114].mFileTimeHi = 0U;
  c4_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c4_info[115].name = "eml_refblas_xtrsm";
  c4_info[115].dominantType = "char";
  c4_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[115].fileTimeLo = 1299076786U;
  c4_info[115].fileTimeHi = 0U;
  c4_info[115].mFileTimeLo = 0U;
  c4_info[115].mFileTimeHi = 0U;
  c4_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[116].name = "eml_scalar_eg";
  c4_info[116].dominantType = "double";
  c4_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[116].fileTimeLo = 1286818796U;
  c4_info[116].fileTimeHi = 0U;
  c4_info[116].mFileTimeLo = 0U;
  c4_info[116].mFileTimeHi = 0U;
  c4_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[117].name = "eml_index_minus";
  c4_info[117].dominantType = "double";
  c4_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[117].fileTimeLo = 1286818778U;
  c4_info[117].fileTimeHi = 0U;
  c4_info[117].mFileTimeLo = 0U;
  c4_info[117].mFileTimeHi = 0U;
  c4_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[118].name = "eml_index_class";
  c4_info[118].dominantType = "";
  c4_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[118].fileTimeLo = 1323170578U;
  c4_info[118].fileTimeHi = 0U;
  c4_info[118].mFileTimeLo = 0U;
  c4_info[118].mFileTimeHi = 0U;
  c4_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[119].name = "eml_int_forloop_overflow_check";
  c4_info[119].dominantType = "";
  c4_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[119].fileTimeLo = 1346510340U;
  c4_info[119].fileTimeHi = 0U;
  c4_info[119].mFileTimeLo = 0U;
  c4_info[119].mFileTimeHi = 0U;
  c4_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[120].name = "eml_index_times";
  c4_info[120].dominantType = "coder.internal.indexInt";
  c4_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c4_info[120].fileTimeLo = 1286818780U;
  c4_info[120].fileTimeHi = 0U;
  c4_info[120].mFileTimeLo = 0U;
  c4_info[120].mFileTimeHi = 0U;
  c4_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[121].name = "eml_index_plus";
  c4_info[121].dominantType = "coder.internal.indexInt";
  c4_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[121].fileTimeLo = 1286818778U;
  c4_info[121].fileTimeHi = 0U;
  c4_info[121].mFileTimeLo = 0U;
  c4_info[121].mFileTimeHi = 0U;
  c4_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[122].name = "eml_index_plus";
  c4_info[122].dominantType = "double";
  c4_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[122].fileTimeLo = 1286818778U;
  c4_info[122].fileTimeHi = 0U;
  c4_info[122].mFileTimeLo = 0U;
  c4_info[122].mFileTimeHi = 0U;
  c4_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c4_info[123].name = "intmin";
  c4_info[123].dominantType = "char";
  c4_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c4_info[123].fileTimeLo = 1311255318U;
  c4_info[123].fileTimeHi = 0U;
  c4_info[123].mFileTimeLo = 0U;
  c4_info[123].mFileTimeHi = 0U;
  c4_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c4_info[124].name = "eml_div";
  c4_info[124].dominantType = "double";
  c4_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c4_info[124].fileTimeLo = 1313347810U;
  c4_info[124].fileTimeHi = 0U;
  c4_info[124].mFileTimeLo = 0U;
  c4_info[124].mFileTimeHi = 0U;
}

static void c4_eml_scalar_eg(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c4_mldivide(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c4_A[36], real_T c4_B[36], real_T c4_Y[36])
{
  int32_T c4_i109;
  real_T c4_b_A[36];
  int32_T c4_info;
  int32_T c4_ipiv[6];
  int32_T c4_b_info;
  int32_T c4_c_info;
  int32_T c4_d_info;
  int32_T c4_i110;
  int32_T c4_i;
  int32_T c4_b_i;
  int32_T c4_ip;
  int32_T c4_j;
  int32_T c4_b_j;
  real_T c4_temp;
  int32_T c4_i111;
  real_T c4_c_A[36];
  int32_T c4_i112;
  real_T c4_d_A[36];
  for (c4_i109 = 0; c4_i109 < 36; c4_i109++) {
    c4_b_A[c4_i109] = c4_A[c4_i109];
  }

  c4_b_eml_matlab_zgetrf(chartInstance, c4_b_A, c4_ipiv, &c4_info);
  c4_b_info = c4_info;
  c4_c_info = c4_b_info;
  c4_d_info = c4_c_info;
  if (c4_d_info > 0) {
    c4_eml_warning(chartInstance);
  }

  c4_eml_scalar_eg(chartInstance);
  for (c4_i110 = 0; c4_i110 < 36; c4_i110++) {
    c4_Y[c4_i110] = c4_B[c4_i110];
  }

  for (c4_i = 1; c4_i < 7; c4_i++) {
    c4_b_i = c4_i;
    if (c4_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_i), 1, 6, 1, 0) - 1] != c4_b_i) {
      c4_ip = c4_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c4_b_i), 1, 6, 1, 0) - 1];
      for (c4_j = 1; c4_j < 7; c4_j++) {
        c4_b_j = c4_j;
        c4_temp = c4_Y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_b_i), 1, 6, 1, 0) + 6 *
                        (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_b_j), 1, 6, 2, 0) - 1)) - 1];
        c4_Y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c4_b_i), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_j), 1, 6, 2, 0)
               - 1)) - 1] = c4_Y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_ip), 1, 6, 1, 0) + 6 *
          (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_j), 1, 6, 2, 0) - 1)) - 1];
        c4_Y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c4_ip), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_j), 1, 6, 2, 0)
               - 1)) - 1] = c4_temp;
      }
    }
  }

  for (c4_i111 = 0; c4_i111 < 36; c4_i111++) {
    c4_c_A[c4_i111] = c4_b_A[c4_i111];
  }

  c4_c_eml_xtrsm(chartInstance, c4_c_A, c4_Y);
  for (c4_i112 = 0; c4_i112 < 36; c4_i112++) {
    c4_d_A[c4_i112] = c4_b_A[c4_i112];
  }

  c4_d_eml_xtrsm(chartInstance, c4_d_A, c4_Y);
}

static void c4_realmin(SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c4_eps(SFc4_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c4_eml_matlab_zgetrf(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_b_A[36], int32_T c4_ipiv[6],
  int32_T *c4_info)
{
  int32_T c4_i113;
  for (c4_i113 = 0; c4_i113 < 36; c4_i113++) {
    c4_b_A[c4_i113] = c4_A[c4_i113];
  }

  c4_b_eml_matlab_zgetrf(chartInstance, c4_b_A, c4_ipiv, c4_info);
}

static void c4_check_forloop_overflow_error
  (SFc4_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c4_overflow)
{
  int32_T c4_i114;
  static char_T c4_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c4_u[34];
  const mxArray *c4_y = NULL;
  int32_T c4_i115;
  static char_T c4_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c4_b_u[23];
  const mxArray *c4_b_y = NULL;
  if (!c4_overflow) {
  } else {
    for (c4_i114 = 0; c4_i114 < 34; c4_i114++) {
      c4_u[c4_i114] = c4_cv0[c4_i114];
    }

    c4_y = NULL;
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c4_i115 = 0; c4_i115 < 23; c4_i115++) {
      c4_b_u[c4_i115] = c4_cv1[c4_i115];
    }

    c4_b_y = NULL;
    sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c4_y, 14, c4_b_y));
  }
}

static void c4_eml_xger(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c4_m, int32_T c4_n, real_T c4_alpha1, int32_T c4_ix0, int32_T c4_iy0,
  real_T c4_A[36], int32_T c4_ia0, real_T c4_b_A[36])
{
  int32_T c4_i116;
  for (c4_i116 = 0; c4_i116 < 36; c4_i116++) {
    c4_b_A[c4_i116] = c4_A[c4_i116];
  }

  c4_b_eml_xger(chartInstance, c4_m, c4_n, c4_alpha1, c4_ix0, c4_iy0, c4_b_A,
                c4_ia0);
}

static void c4_eml_warning(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c4_i117;
  static char_T c4_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c4_u[27];
  const mxArray *c4_y = NULL;
  for (c4_i117 = 0; c4_i117 < 27; c4_i117++) {
    c4_u[c4_i117] = c4_varargin_1[c4_i117];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c4_y));
}

static void c4_eml_xtrsm(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c4_A[36], real_T c4_B[36], real_T c4_b_B[36])
{
  int32_T c4_i118;
  int32_T c4_i119;
  real_T c4_b_A[36];
  for (c4_i118 = 0; c4_i118 < 36; c4_i118++) {
    c4_b_B[c4_i118] = c4_B[c4_i118];
  }

  for (c4_i119 = 0; c4_i119 < 36; c4_i119++) {
    c4_b_A[c4_i119] = c4_A[c4_i119];
  }

  c4_c_eml_xtrsm(chartInstance, c4_b_A, c4_b_B);
}

static void c4_below_threshold(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c4_b_eml_scalar_eg(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c4_b_eml_xtrsm(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_B[36], real_T c4_b_B[36])
{
  int32_T c4_i120;
  int32_T c4_i121;
  real_T c4_b_A[36];
  for (c4_i120 = 0; c4_i120 < 36; c4_i120++) {
    c4_b_B[c4_i120] = c4_B[c4_i120];
  }

  for (c4_i121 = 0; c4_i121 < 36; c4_i121++) {
    c4_b_A[c4_i121] = c4_A[c4_i121];
  }

  c4_d_eml_xtrsm(chartInstance, c4_b_A, c4_b_B);
}

static void c4_c_eml_scalar_eg(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c4_eml_xgemm(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c4_A[36], real_T c4_B[138], real_T c4_C[138], real_T c4_b_C[138])
{
  int32_T c4_i122;
  int32_T c4_i123;
  real_T c4_b_A[36];
  int32_T c4_i124;
  real_T c4_b_B[138];
  for (c4_i122 = 0; c4_i122 < 138; c4_i122++) {
    c4_b_C[c4_i122] = c4_C[c4_i122];
  }

  for (c4_i123 = 0; c4_i123 < 36; c4_i123++) {
    c4_b_A[c4_i123] = c4_A[c4_i123];
  }

  for (c4_i124 = 0; c4_i124 < 138; c4_i124++) {
    c4_b_B[c4_i124] = c4_B[c4_i124];
  }

  c4_b_eml_xgemm(chartInstance, c4_b_A, c4_b_B, c4_b_C);
}

static void c4_d_eml_scalar_eg(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static const mxArray *c4_h_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static int32_T c4_f_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i125;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i125, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i125;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_sfEvent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y;
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c4_b_sfEvent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_g_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_torqueBalancing2012b, const
  char_T *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_torqueBalancing2012b), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_torqueBalancing2012b);
  return c4_y;
}

static uint8_T c4_h_emlrt_marshallIn(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_b_eml_matlab_zgetrf(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], int32_T c4_ipiv[6], int32_T *c4_info)
{
  int32_T c4_i126;
  int32_T c4_j;
  int32_T c4_b_j;
  int32_T c4_a;
  int32_T c4_jm1;
  int32_T c4_b;
  int32_T c4_mmj;
  int32_T c4_b_a;
  int32_T c4_c;
  int32_T c4_b_b;
  int32_T c4_jj;
  int32_T c4_c_a;
  int32_T c4_jp1j;
  int32_T c4_d_a;
  int32_T c4_b_c;
  int32_T c4_n;
  int32_T c4_ix0;
  int32_T c4_b_n;
  int32_T c4_b_ix0;
  int32_T c4_c_n;
  int32_T c4_c_ix0;
  int32_T c4_idxmax;
  int32_T c4_ix;
  real_T c4_x;
  real_T c4_b_x;
  real_T c4_c_x;
  real_T c4_y;
  real_T c4_d_x;
  real_T c4_e_x;
  real_T c4_b_y;
  real_T c4_smax;
  int32_T c4_d_n;
  int32_T c4_c_b;
  int32_T c4_d_b;
  boolean_T c4_overflow;
  int32_T c4_k;
  int32_T c4_b_k;
  int32_T c4_e_a;
  real_T c4_f_x;
  real_T c4_g_x;
  real_T c4_h_x;
  real_T c4_c_y;
  real_T c4_i_x;
  real_T c4_j_x;
  real_T c4_d_y;
  real_T c4_s;
  int32_T c4_f_a;
  int32_T c4_jpiv_offset;
  int32_T c4_g_a;
  int32_T c4_e_b;
  int32_T c4_jpiv;
  int32_T c4_h_a;
  int32_T c4_f_b;
  int32_T c4_c_c;
  int32_T c4_g_b;
  int32_T c4_jrow;
  int32_T c4_i_a;
  int32_T c4_h_b;
  int32_T c4_jprow;
  int32_T c4_d_ix0;
  int32_T c4_iy0;
  int32_T c4_e_ix0;
  int32_T c4_b_iy0;
  int32_T c4_f_ix0;
  int32_T c4_c_iy0;
  int32_T c4_b_ix;
  int32_T c4_iy;
  int32_T c4_c_k;
  real_T c4_temp;
  int32_T c4_j_a;
  int32_T c4_k_a;
  int32_T c4_b_jp1j;
  int32_T c4_l_a;
  int32_T c4_d_c;
  int32_T c4_m_a;
  int32_T c4_i_b;
  int32_T c4_i127;
  int32_T c4_n_a;
  int32_T c4_j_b;
  int32_T c4_o_a;
  int32_T c4_k_b;
  boolean_T c4_b_overflow;
  int32_T c4_i;
  int32_T c4_b_i;
  real_T c4_k_x;
  real_T c4_e_y;
  real_T c4_z;
  int32_T c4_l_b;
  int32_T c4_e_c;
  int32_T c4_p_a;
  int32_T c4_f_c;
  int32_T c4_q_a;
  int32_T c4_g_c;
  int32_T c4_m;
  int32_T c4_e_n;
  int32_T c4_g_ix0;
  int32_T c4_d_iy0;
  int32_T c4_ia0;
  real_T c4_d1;
  c4_realmin(chartInstance);
  c4_eps(chartInstance);
  for (c4_i126 = 0; c4_i126 < 6; c4_i126++) {
    c4_ipiv[c4_i126] = 1 + c4_i126;
  }

  *c4_info = 0;
  for (c4_j = 1; c4_j < 6; c4_j++) {
    c4_b_j = c4_j;
    c4_a = c4_b_j - 1;
    c4_jm1 = c4_a;
    c4_b = c4_b_j;
    c4_mmj = 6 - c4_b;
    c4_b_a = c4_jm1;
    c4_c = c4_b_a * 7;
    c4_b_b = c4_c + 1;
    c4_jj = c4_b_b;
    c4_c_a = c4_jj + 1;
    c4_jp1j = c4_c_a;
    c4_d_a = c4_mmj;
    c4_b_c = c4_d_a;
    c4_n = c4_b_c + 1;
    c4_ix0 = c4_jj;
    c4_b_n = c4_n;
    c4_b_ix0 = c4_ix0;
    c4_c_n = c4_b_n;
    c4_c_ix0 = c4_b_ix0;
    if (c4_c_n < 1) {
      c4_idxmax = 0;
    } else {
      c4_idxmax = 1;
      if (c4_c_n > 1) {
        c4_ix = c4_c_ix0;
        c4_x = c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c4_ix), 1, 36, 1, 0) - 1];
        c4_b_x = c4_x;
        c4_c_x = c4_b_x;
        c4_y = muDoubleScalarAbs(c4_c_x);
        c4_d_x = 0.0;
        c4_e_x = c4_d_x;
        c4_b_y = muDoubleScalarAbs(c4_e_x);
        c4_smax = c4_y + c4_b_y;
        c4_d_n = c4_c_n;
        c4_c_b = c4_d_n;
        c4_d_b = c4_c_b;
        if (2 > c4_d_b) {
          c4_overflow = FALSE;
        } else {
          c4_overflow = (c4_d_b > 2147483646);
        }

        if (c4_overflow) {
          c4_check_forloop_overflow_error(chartInstance, c4_overflow);
        }

        for (c4_k = 2; c4_k <= c4_d_n; c4_k++) {
          c4_b_k = c4_k;
          c4_e_a = c4_ix + 1;
          c4_ix = c4_e_a;
          c4_f_x = c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_ix), 1, 36, 1, 0) - 1];
          c4_g_x = c4_f_x;
          c4_h_x = c4_g_x;
          c4_c_y = muDoubleScalarAbs(c4_h_x);
          c4_i_x = 0.0;
          c4_j_x = c4_i_x;
          c4_d_y = muDoubleScalarAbs(c4_j_x);
          c4_s = c4_c_y + c4_d_y;
          if (c4_s > c4_smax) {
            c4_idxmax = c4_b_k;
            c4_smax = c4_s;
          }
        }
      }
    }

    c4_f_a = c4_idxmax - 1;
    c4_jpiv_offset = c4_f_a;
    c4_g_a = c4_jj;
    c4_e_b = c4_jpiv_offset;
    c4_jpiv = c4_g_a + c4_e_b;
    if (c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_jpiv), 1, 36, 1, 0) - 1] != 0.0) {
      if (c4_jpiv_offset != 0) {
        c4_h_a = c4_b_j;
        c4_f_b = c4_jpiv_offset;
        c4_c_c = c4_h_a + c4_f_b;
        c4_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_j), 1, 6, 1, 0) - 1] = c4_c_c;
        c4_g_b = c4_jm1 + 1;
        c4_jrow = c4_g_b;
        c4_i_a = c4_jrow;
        c4_h_b = c4_jpiv_offset;
        c4_jprow = c4_i_a + c4_h_b;
        c4_d_ix0 = c4_jrow;
        c4_iy0 = c4_jprow;
        c4_e_ix0 = c4_d_ix0;
        c4_b_iy0 = c4_iy0;
        c4_f_ix0 = c4_e_ix0;
        c4_c_iy0 = c4_b_iy0;
        c4_b_ix = c4_f_ix0;
        c4_iy = c4_c_iy0;
        for (c4_c_k = 1; c4_c_k < 7; c4_c_k++) {
          c4_temp = c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_ix), 1, 36, 1, 0) - 1];
          c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_ix), 1, 36, 1, 0) - 1] =
            c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_iy), 1, 36, 1, 0) - 1];
          c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_iy), 1, 36, 1, 0) - 1] = c4_temp;
          c4_j_a = c4_b_ix + 6;
          c4_b_ix = c4_j_a;
          c4_k_a = c4_iy + 6;
          c4_iy = c4_k_a;
        }
      }

      c4_b_jp1j = c4_jp1j;
      c4_l_a = c4_mmj;
      c4_d_c = c4_l_a;
      c4_m_a = c4_jp1j;
      c4_i_b = c4_d_c - 1;
      c4_i127 = c4_m_a + c4_i_b;
      c4_n_a = c4_b_jp1j;
      c4_j_b = c4_i127;
      c4_o_a = c4_n_a;
      c4_k_b = c4_j_b;
      if (c4_o_a > c4_k_b) {
        c4_b_overflow = FALSE;
      } else {
        c4_b_overflow = (c4_k_b > 2147483646);
      }

      if (c4_b_overflow) {
        c4_check_forloop_overflow_error(chartInstance, c4_b_overflow);
      }

      for (c4_i = c4_b_jp1j; c4_i <= c4_i127; c4_i++) {
        c4_b_i = c4_i;
        c4_k_x = c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_b_i), 1, 36, 1, 0) - 1];
        c4_e_y = c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_jj), 1, 36, 1, 0) - 1];
        c4_z = c4_k_x / c4_e_y;
        c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_i), 1, 36, 1, 0) - 1] = c4_z;
      }
    } else {
      *c4_info = c4_b_j;
    }

    c4_l_b = c4_b_j;
    c4_e_c = 6 - c4_l_b;
    c4_p_a = c4_jj;
    c4_f_c = c4_p_a;
    c4_q_a = c4_jj;
    c4_g_c = c4_q_a;
    c4_m = c4_mmj;
    c4_e_n = c4_e_c;
    c4_g_ix0 = c4_jp1j;
    c4_d_iy0 = c4_f_c + 6;
    c4_ia0 = c4_g_c + 7;
    c4_d1 = -1.0;
    c4_b_eml_xger(chartInstance, c4_m, c4_e_n, c4_d1, c4_g_ix0, c4_d_iy0, c4_A,
                  c4_ia0);
  }

  if (*c4_info == 0) {
    if (!(c4_A[35] != 0.0)) {
      *c4_info = 6;
    }
  }
}

static void c4_b_eml_xger(SFc4_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c4_m, int32_T c4_n, real_T c4_alpha1, int32_T c4_ix0, int32_T c4_iy0,
  real_T c4_A[36], int32_T c4_ia0)
{
  int32_T c4_b_m;
  int32_T c4_b_n;
  real_T c4_b_alpha1;
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_b_ia0;
  int32_T c4_c_m;
  int32_T c4_c_n;
  real_T c4_c_alpha1;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_c_ia0;
  int32_T c4_d_m;
  int32_T c4_d_n;
  real_T c4_d_alpha1;
  int32_T c4_d_ix0;
  int32_T c4_d_iy0;
  int32_T c4_d_ia0;
  int32_T c4_ixstart;
  int32_T c4_a;
  int32_T c4_jA;
  int32_T c4_jy;
  int32_T c4_e_n;
  int32_T c4_b;
  int32_T c4_b_b;
  boolean_T c4_overflow;
  int32_T c4_j;
  real_T c4_yjy;
  real_T c4_temp;
  int32_T c4_ix;
  int32_T c4_c_b;
  int32_T c4_i128;
  int32_T c4_b_a;
  int32_T c4_d_b;
  int32_T c4_i129;
  int32_T c4_c_a;
  int32_T c4_e_b;
  int32_T c4_d_a;
  int32_T c4_f_b;
  boolean_T c4_b_overflow;
  int32_T c4_ijA;
  int32_T c4_b_ijA;
  int32_T c4_e_a;
  int32_T c4_f_a;
  int32_T c4_g_a;
  c4_b_m = c4_m;
  c4_b_n = c4_n;
  c4_b_alpha1 = c4_alpha1;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_b_ia0 = c4_ia0;
  c4_c_m = c4_b_m;
  c4_c_n = c4_b_n;
  c4_c_alpha1 = c4_b_alpha1;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  c4_c_ia0 = c4_b_ia0;
  c4_d_m = c4_c_m;
  c4_d_n = c4_c_n;
  c4_d_alpha1 = c4_c_alpha1;
  c4_d_ix0 = c4_c_ix0;
  c4_d_iy0 = c4_c_iy0;
  c4_d_ia0 = c4_c_ia0;
  if (c4_d_alpha1 == 0.0) {
  } else {
    c4_ixstart = c4_d_ix0;
    c4_a = c4_d_ia0 - 1;
    c4_jA = c4_a;
    c4_jy = c4_d_iy0;
    c4_e_n = c4_d_n;
    c4_b = c4_e_n;
    c4_b_b = c4_b;
    if (1 > c4_b_b) {
      c4_overflow = FALSE;
    } else {
      c4_overflow = (c4_b_b > 2147483646);
    }

    if (c4_overflow) {
      c4_check_forloop_overflow_error(chartInstance, c4_overflow);
    }

    for (c4_j = 1; c4_j <= c4_e_n; c4_j++) {
      c4_yjy = c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c4_jy), 1, 36, 1, 0) - 1];
      if (c4_yjy != 0.0) {
        c4_temp = c4_yjy * c4_d_alpha1;
        c4_ix = c4_ixstart;
        c4_c_b = c4_jA + 1;
        c4_i128 = c4_c_b;
        c4_b_a = c4_d_m;
        c4_d_b = c4_jA;
        c4_i129 = c4_b_a + c4_d_b;
        c4_c_a = c4_i128;
        c4_e_b = c4_i129;
        c4_d_a = c4_c_a;
        c4_f_b = c4_e_b;
        if (c4_d_a > c4_f_b) {
          c4_b_overflow = FALSE;
        } else {
          c4_b_overflow = (c4_f_b > 2147483646);
        }

        if (c4_b_overflow) {
          c4_check_forloop_overflow_error(chartInstance, c4_b_overflow);
        }

        for (c4_ijA = c4_i128; c4_ijA <= c4_i129; c4_ijA++) {
          c4_b_ijA = c4_ijA;
          c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_ijA), 1, 36, 1, 0) - 1] =
            c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_ijA), 1, 36, 1, 0) - 1] +
            c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_ix), 1, 36, 1, 0) - 1] * c4_temp;
          c4_e_a = c4_ix + 1;
          c4_ix = c4_e_a;
        }
      }

      c4_f_a = c4_jy + 6;
      c4_jy = c4_f_a;
      c4_g_a = c4_jA + 6;
      c4_jA = c4_g_a;
    }
  }
}

static void c4_c_eml_xtrsm(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_B[36])
{
  int32_T c4_j;
  int32_T c4_b_j;
  int32_T c4_a;
  int32_T c4_c;
  int32_T c4_b;
  int32_T c4_b_c;
  int32_T c4_b_b;
  int32_T c4_jBcol;
  int32_T c4_k;
  int32_T c4_b_k;
  int32_T c4_b_a;
  int32_T c4_c_c;
  int32_T c4_c_b;
  int32_T c4_d_c;
  int32_T c4_d_b;
  int32_T c4_kAcol;
  int32_T c4_c_a;
  int32_T c4_e_b;
  int32_T c4_e_c;
  int32_T c4_d_a;
  int32_T c4_i130;
  boolean_T c4_overflow;
  int32_T c4_i;
  int32_T c4_b_i;
  int32_T c4_e_a;
  int32_T c4_f_b;
  int32_T c4_f_c;
  int32_T c4_f_a;
  int32_T c4_g_b;
  int32_T c4_g_c;
  int32_T c4_g_a;
  int32_T c4_h_b;
  int32_T c4_h_c;
  int32_T c4_h_a;
  int32_T c4_i_b;
  int32_T c4_i_c;
  c4_below_threshold(chartInstance);
  c4_b_eml_scalar_eg(chartInstance);
  for (c4_j = 1; c4_j < 7; c4_j++) {
    c4_b_j = c4_j;
    c4_a = c4_b_j;
    c4_c = c4_a;
    c4_b = c4_c - 1;
    c4_b_c = 6 * c4_b;
    c4_b_b = c4_b_c;
    c4_jBcol = c4_b_b;
    for (c4_k = 1; c4_k < 7; c4_k++) {
      c4_b_k = c4_k;
      c4_b_a = c4_b_k;
      c4_c_c = c4_b_a;
      c4_c_b = c4_c_c - 1;
      c4_d_c = 6 * c4_c_b;
      c4_d_b = c4_d_c;
      c4_kAcol = c4_d_b;
      c4_c_a = c4_b_k;
      c4_e_b = c4_jBcol;
      c4_e_c = c4_c_a + c4_e_b;
      if (c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_e_c), 1, 36, 1, 0) - 1] != 0.0) {
        c4_d_a = c4_b_k;
        c4_i130 = c4_d_a;
        c4_overflow = FALSE;
        if (c4_overflow) {
          c4_check_forloop_overflow_error(chartInstance, c4_overflow);
        }

        for (c4_i = c4_i130 + 1; c4_i < 7; c4_i++) {
          c4_b_i = c4_i;
          c4_e_a = c4_b_i;
          c4_f_b = c4_jBcol;
          c4_f_c = c4_e_a + c4_f_b;
          c4_f_a = c4_b_i;
          c4_g_b = c4_jBcol;
          c4_g_c = c4_f_a + c4_g_b;
          c4_g_a = c4_b_k;
          c4_h_b = c4_jBcol;
          c4_h_c = c4_g_a + c4_h_b;
          c4_h_a = c4_b_i;
          c4_i_b = c4_kAcol;
          c4_i_c = c4_h_a + c4_i_b;
          c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_f_c), 1, 36, 1, 0) - 1] =
            c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_g_c), 1, 36, 1, 0) - 1] -
            c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_h_c), 1, 36, 1, 0) - 1] *
            c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_i_c), 1, 36, 1, 0) - 1];
        }
      }
    }
  }
}

static void c4_d_eml_xtrsm(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_B[36])
{
  int32_T c4_j;
  int32_T c4_b_j;
  int32_T c4_a;
  int32_T c4_c;
  int32_T c4_b;
  int32_T c4_b_c;
  int32_T c4_b_b;
  int32_T c4_jBcol;
  int32_T c4_k;
  int32_T c4_b_k;
  int32_T c4_b_a;
  int32_T c4_c_c;
  int32_T c4_c_b;
  int32_T c4_d_c;
  int32_T c4_d_b;
  int32_T c4_kAcol;
  int32_T c4_c_a;
  int32_T c4_e_b;
  int32_T c4_e_c;
  int32_T c4_d_a;
  int32_T c4_f_b;
  int32_T c4_f_c;
  int32_T c4_e_a;
  int32_T c4_g_b;
  int32_T c4_g_c;
  int32_T c4_f_a;
  int32_T c4_h_b;
  int32_T c4_h_c;
  real_T c4_x;
  real_T c4_y;
  real_T c4_z;
  int32_T c4_g_a;
  int32_T c4_i131;
  int32_T c4_i_b;
  int32_T c4_j_b;
  boolean_T c4_overflow;
  int32_T c4_i;
  int32_T c4_b_i;
  int32_T c4_h_a;
  int32_T c4_k_b;
  int32_T c4_i_c;
  int32_T c4_i_a;
  int32_T c4_l_b;
  int32_T c4_j_c;
  int32_T c4_j_a;
  int32_T c4_m_b;
  int32_T c4_k_c;
  int32_T c4_k_a;
  int32_T c4_n_b;
  int32_T c4_l_c;
  c4_below_threshold(chartInstance);
  c4_b_eml_scalar_eg(chartInstance);
  for (c4_j = 1; c4_j < 7; c4_j++) {
    c4_b_j = c4_j;
    c4_a = c4_b_j;
    c4_c = c4_a;
    c4_b = c4_c - 1;
    c4_b_c = 6 * c4_b;
    c4_b_b = c4_b_c;
    c4_jBcol = c4_b_b;
    for (c4_k = 6; c4_k > 0; c4_k--) {
      c4_b_k = c4_k;
      c4_b_a = c4_b_k;
      c4_c_c = c4_b_a;
      c4_c_b = c4_c_c - 1;
      c4_d_c = 6 * c4_c_b;
      c4_d_b = c4_d_c;
      c4_kAcol = c4_d_b;
      c4_c_a = c4_b_k;
      c4_e_b = c4_jBcol;
      c4_e_c = c4_c_a + c4_e_b;
      if (c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_e_c), 1, 36, 1, 0) - 1] != 0.0) {
        c4_d_a = c4_b_k;
        c4_f_b = c4_jBcol;
        c4_f_c = c4_d_a + c4_f_b;
        c4_e_a = c4_b_k;
        c4_g_b = c4_jBcol;
        c4_g_c = c4_e_a + c4_g_b;
        c4_f_a = c4_b_k;
        c4_h_b = c4_kAcol;
        c4_h_c = c4_f_a + c4_h_b;
        c4_x = c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c4_g_c), 1, 36, 1, 0) - 1];
        c4_y = c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c4_h_c), 1, 36, 1, 0) - 1];
        c4_z = c4_x / c4_y;
        c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_f_c), 1, 36, 1, 0) - 1] = c4_z;
        c4_g_a = c4_b_k - 1;
        c4_i131 = c4_g_a;
        c4_i_b = c4_i131;
        c4_j_b = c4_i_b;
        if (1 > c4_j_b) {
          c4_overflow = FALSE;
        } else {
          c4_overflow = (c4_j_b > 2147483646);
        }

        if (c4_overflow) {
          c4_check_forloop_overflow_error(chartInstance, c4_overflow);
        }

        for (c4_i = 1; c4_i <= c4_i131; c4_i++) {
          c4_b_i = c4_i;
          c4_h_a = c4_b_i;
          c4_k_b = c4_jBcol;
          c4_i_c = c4_h_a + c4_k_b;
          c4_i_a = c4_b_i;
          c4_l_b = c4_jBcol;
          c4_j_c = c4_i_a + c4_l_b;
          c4_j_a = c4_b_k;
          c4_m_b = c4_jBcol;
          c4_k_c = c4_j_a + c4_m_b;
          c4_k_a = c4_b_i;
          c4_n_b = c4_kAcol;
          c4_l_c = c4_k_a + c4_n_b;
          c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_i_c), 1, 36, 1, 0) - 1] =
            c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_j_c), 1, 36, 1, 0) - 1] -
            c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_k_c), 1, 36, 1, 0) - 1] *
            c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_l_c), 1, 36, 1, 0) - 1];
        }
      }
    }
  }
}

static void c4_b_eml_xgemm(SFc4_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c4_A[36], real_T c4_B[138], real_T c4_C[138])
{
  real_T c4_alpha1;
  real_T c4_beta1;
  char_T c4_TRANSB;
  char_T c4_TRANSA;
  ptrdiff_t c4_m_t;
  ptrdiff_t c4_n_t;
  ptrdiff_t c4_k_t;
  ptrdiff_t c4_lda_t;
  ptrdiff_t c4_ldb_t;
  ptrdiff_t c4_ldc_t;
  double * c4_alpha1_t;
  double * c4_Aia0_t;
  double * c4_Bib0_t;
  double * c4_beta1_t;
  double * c4_Cic0_t;
  c4_alpha1 = 1.0;
  c4_beta1 = 0.0;
  c4_TRANSB = 'N';
  c4_TRANSA = 'N';
  c4_m_t = (ptrdiff_t)(6);
  c4_n_t = (ptrdiff_t)(23);
  c4_k_t = (ptrdiff_t)(6);
  c4_lda_t = (ptrdiff_t)(6);
  c4_ldb_t = (ptrdiff_t)(6);
  c4_ldc_t = (ptrdiff_t)(6);
  c4_alpha1_t = (double *)(&c4_alpha1);
  c4_Aia0_t = (double *)(&c4_A[0]);
  c4_Bib0_t = (double *)(&c4_B[0]);
  c4_beta1_t = (double *)(&c4_beta1);
  c4_Cic0_t = (double *)(&c4_C[0]);
  dgemm(&c4_TRANSA, &c4_TRANSB, &c4_m_t, &c4_n_t, &c4_k_t, c4_alpha1_t,
        c4_Aia0_t, &c4_lda_t, c4_Bib0_t, &c4_ldb_t, c4_beta1_t, c4_Cic0_t,
        &c4_ldc_t);
}

static void init_dsm_address_info(SFc4_torqueBalancing2012bInstanceStruct
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

void sf_c4_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4226989685U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3866395182U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(436714063U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3891265924U);
}

mxArray *sf_c4_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("zcwXeLXPiG6Bjb6oS6nqFB");
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
      pr[0] = (double)(6);
      pr[1] = (double)(29);
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
      pr[0] = (double)(6);
      pr[1] = (double)(29);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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

mxArray *sf_c4_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c4_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[7],T\"vbEquivalent\",},{M[8],M[0],T\"is_active_c4_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           4,
           1,
           1,
           6,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"qjErr");
          _SFD_SET_DATA_PROPS(1,1,1,0,"JL");
          _SFD_SET_DATA_PROPS(2,1,1,0,"JR");
          _SFD_SET_DATA_PROPS(3,2,0,1,"vbEquivalent");
          _SFD_SET_DATA_PROPS(4,10,0,0,"reg");
          _SFD_SET_DATA_PROPS(5,1,1,0,"activeFeetConstraints");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",1,-1,429);
        _SFD_CV_INIT_EML_IF(0,1,0,97,129,271,420);
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
            1.0,0,0,(MexFcnForType)c4_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)
            c4_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(4,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)c4_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c4_qjErr)[23];
          real_T (*c4_JL)[174];
          real_T (*c4_JR)[174];
          real_T (*c4_vbEquivalent)[6];
          real_T (*c4_activeFeetConstraints)[2];
          c4_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
            (chartInstance->S, 3);
          c4_vbEquivalent = (real_T (*)[6])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c4_JR = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 2);
          c4_JL = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
          c4_qjErr = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c4_qjErr);
          _SFD_SET_DATA_VALUE_PTR(1U, *c4_JL);
          _SFD_SET_DATA_VALUE_PTR(2U, *c4_JR);
          _SFD_SET_DATA_VALUE_PTR(3U, *c4_vbEquivalent);
          _SFD_SET_DATA_VALUE_PTR(4U, &chartInstance->c4_reg);
          _SFD_SET_DATA_VALUE_PTR(5U, *c4_activeFeetConstraints);
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
  return "vZCkkmTRf2hKhmiWECyBEF";
}

static void sf_opaque_initialize_c4_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c4_torqueBalancing2012b
    ((SFc4_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c4_torqueBalancing2012b((SFc4_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c4_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c4_torqueBalancing2012b((SFc4_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c4_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c4_torqueBalancing2012b((SFc4_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c4_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c4_torqueBalancing2012b((SFc4_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c4_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c4_torqueBalancing2012b
    ((SFc4_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c4_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c4_torqueBalancing2012b((SFc4_torqueBalancing2012bInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c4_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c4_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c4_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c4_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c4_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_torqueBalancing2012bInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c4_torqueBalancing2012b((SFc4_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_torqueBalancing2012b((SFc4_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_torqueBalancing2012b
      ((SFc4_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_torqueBalancing2012b(SimStruct *S)
{
  /* Actual parameters from chart:
     reg
   */
  const char_T *rtParamNames[] = { "reg" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0],
    sf_get_param_data_type_id(S,0));
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,4,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,4,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,4);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(574935929U));
  ssSetChecksum1(S,(459421711U));
  ssSetChecksum2(S,(367408130U));
  ssSetChecksum3(S,(674380260U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c4_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_torqueBalancing2012b(SimStruct *S)
{
  SFc4_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc4_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc4_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c4_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c4_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c4_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c4_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c4_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c4_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c4_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c4_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c4_torqueBalancing2012b;
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

void c4_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void *
  data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
