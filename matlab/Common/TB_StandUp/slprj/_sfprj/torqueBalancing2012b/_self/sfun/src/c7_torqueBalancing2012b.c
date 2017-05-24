/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c7_torqueBalancing2012b.h"
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
static const char * c7_debug_family_names[10] = { "Jc", "pinvJb", "nargin",
  "nargout", "JcLeftFoot", "JcRightFoot", "activeFeetConstraints", "qD", "reg",
  "y" };

/* Function Declarations */
static void initialize_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void c7_update_debugger_state_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c7_st);
static void finalize_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c7_torqueBalancing2012b(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c7_chartstep_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void initSimStructsc7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber);
static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData);
static void c7_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_y, const char_T *c7_identifier, real_T
  c7_b_y[6]);
static void c7_b_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[6]);
static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static c7_struct_1ZGMVR6bgCMpDdXTSGnu6G c7_c_emlrt_marshallIn
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c7_u,
   const emlrtMsgIdentifier *c7_parentId);
static real_T c7_d_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static const mxArray *c7_f_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_g_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_e_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[72]);
static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_h_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_f_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[348]);
static void c7_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static void c7_info_helper(c7_ResolvedFunctionInfo c7_info[125]);
static void c7_b_info_helper(c7_ResolvedFunctionInfo c7_info[125]);
static void c7_eml_scalar_eg(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c7_mldivide(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c7_A[36], real_T c7_B[72], real_T c7_Y[72]);
static void c7_realmin(SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void c7_eps(SFc7_torqueBalancing2012bInstanceStruct *chartInstance);
static void c7_eml_matlab_zgetrf(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], real_T c7_b_A[36], int32_T c7_ipiv[6],
  int32_T *c7_info);
static void c7_check_forloop_overflow_error
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c7_overflow);
static void c7_eml_xger(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_alpha1, int32_T c7_ix0, int32_T c7_iy0,
  real_T c7_A[36], int32_T c7_ia0, real_T c7_b_A[36]);
static void c7_eml_warning(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c7_eml_xtrsm(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c7_A[36], real_T c7_B[72], real_T c7_b_B[72]);
static void c7_below_threshold(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c7_b_eml_scalar_eg(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c7_b_eml_xtrsm(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], real_T c7_B[72], real_T c7_b_B[72]);
static void c7_c_eml_scalar_eg(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c7_eml_xgemm(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c7_A[72], real_T c7_B[276], real_T c7_C[138], real_T c7_b_C[138]);
static void c7_d_eml_scalar_eg(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c7_i_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static int32_T c7_g_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static uint8_T c7_h_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_b_is_active_c7_torqueBalancing2012b, const
  char_T *c7_identifier);
static uint8_T c7_i_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_b_eml_matlab_zgetrf(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], int32_T c7_ipiv[6], int32_T *c7_info);
static void c7_b_eml_xger(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_alpha1, int32_T c7_ix0, int32_T c7_iy0,
  real_T c7_A[36], int32_T c7_ia0);
static void c7_c_eml_xtrsm(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], real_T c7_B[72]);
static void c7_d_eml_xtrsm(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], real_T c7_B[72]);
static void c7_b_eml_xgemm(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[72], real_T c7_B[276], real_T c7_C[138]);
static void init_dsm_address_info(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c7_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c7_is_active_c7_torqueBalancing2012b = 0U;
}

static void initialize_params_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c7_m0 = NULL;
  const mxArray *c7_mxField;
  c7_struct_1ZGMVR6bgCMpDdXTSGnu6G c7_r0;
  sf_set_error_prefix_string(
    "Error evaluating data 'reg' in the parent workspace.\n");
  c7_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c7_mxField = sf_mex_getfield(c7_m0, "pinvTol", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c7_mxField), &c7_r0.pinvTol, 1, 0, 0U, 0,
                      0U, 0);
  c7_mxField = sf_mex_getfield(c7_m0, "pinvDamp", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c7_mxField), &c7_r0.pinvDamp, 1, 0, 0U,
                      0, 0U, 0);
  c7_mxField = sf_mex_getfield(c7_m0, "pinvDampVb", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c7_mxField), &c7_r0.pinvDampVb, 1, 0, 0U,
                      0, 0U, 0);
  c7_mxField = sf_mex_getfield(c7_m0, "HessianQP", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c7_mxField), &c7_r0.HessianQP, 1, 0, 0U,
                      0, 0U, 0);
  c7_mxField = sf_mex_getfield(c7_m0, "impedances", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c7_mxField), &c7_r0.impedances, 1, 0, 0U,
                      0, 0U, 0);
  c7_mxField = sf_mex_getfield(c7_m0, "dampings", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c7_mxField), &c7_r0.dampings, 1, 0, 0U,
                      0, 0U, 0);
  c7_mxField = sf_mex_getfield(c7_m0, "norm_tolerance", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c7_mxField), &c7_r0.norm_tolerance, 1, 0,
                      0U, 0, 0U, 0);
  sf_mex_destroy(&c7_m0);
  chartInstance->c7_reg = c7_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c7_update_debugger_state_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c7_st;
  const mxArray *c7_y = NULL;
  int32_T c7_i0;
  real_T c7_u[6];
  const mxArray *c7_b_y = NULL;
  uint8_T c7_hoistedGlobal;
  uint8_T c7_b_u;
  const mxArray *c7_c_y = NULL;
  real_T (*c7_d_y)[6];
  c7_d_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_st = NULL;
  c7_st = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_createcellarray(2), FALSE);
  for (c7_i0 = 0; c7_i0 < 6; c7_i0++) {
    c7_u[c7_i0] = (*c7_d_y)[c7_i0];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_setcell(c7_y, 0, c7_b_y);
  c7_hoistedGlobal = chartInstance->c7_is_active_c7_torqueBalancing2012b;
  c7_b_u = c7_hoistedGlobal;
  c7_c_y = NULL;
  sf_mex_assign(&c7_c_y, sf_mex_create("y", &c7_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c7_y, 1, c7_c_y);
  sf_mex_assign(&c7_st, c7_y, FALSE);
  return c7_st;
}

static void set_sim_state_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c7_st)
{
  const mxArray *c7_u;
  real_T c7_dv0[6];
  int32_T c7_i1;
  real_T (*c7_y)[6];
  c7_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c7_doneDoubleBufferReInit = TRUE;
  c7_u = sf_mex_dup(c7_st);
  c7_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 0)), "y",
                      c7_dv0);
  for (c7_i1 = 0; c7_i1 < 6; c7_i1++) {
    (*c7_y)[c7_i1] = c7_dv0[c7_i1];
  }

  chartInstance->c7_is_active_c7_torqueBalancing2012b = c7_h_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 1)),
     "is_active_c7_torqueBalancing2012b");
  sf_mex_destroy(&c7_u);
  c7_update_debugger_state_c7_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c7_st);
}

static void finalize_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c7_torqueBalancing2012b(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c7_i2;
  int32_T c7_i3;
  int32_T c7_i4;
  int32_T c7_i5;
  int32_T c7_i6;
  real_T (*c7_qD)[23];
  real_T (*c7_y)[6];
  real_T (*c7_activeFeetConstraints)[2];
  real_T (*c7_JcRightFoot)[174];
  real_T (*c7_JcLeftFoot)[174];
  c7_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c7_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
    (chartInstance->S, 2);
  c7_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c7_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 6U, chartInstance->c7_sfEvent);
  for (c7_i2 = 0; c7_i2 < 174; c7_i2++) {
    _SFD_DATA_RANGE_CHECK((*c7_JcLeftFoot)[c7_i2], 0U);
  }

  for (c7_i3 = 0; c7_i3 < 174; c7_i3++) {
    _SFD_DATA_RANGE_CHECK((*c7_JcRightFoot)[c7_i3], 1U);
  }

  for (c7_i4 = 0; c7_i4 < 2; c7_i4++) {
    _SFD_DATA_RANGE_CHECK((*c7_activeFeetConstraints)[c7_i4], 2U);
  }

  for (c7_i5 = 0; c7_i5 < 6; c7_i5++) {
    _SFD_DATA_RANGE_CHECK((*c7_y)[c7_i5], 3U);
  }

  for (c7_i6 = 0; c7_i6 < 23; c7_i6++) {
    _SFD_DATA_RANGE_CHECK((*c7_qD)[c7_i6], 4U);
  }

  chartInstance->c7_sfEvent = CALL_EVENT;
  c7_chartstep_c7_torqueBalancing2012b(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c7_chartstep_c7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
  int32_T c7_i7;
  real_T c7_JcLeftFoot[174];
  int32_T c7_i8;
  real_T c7_JcRightFoot[174];
  int32_T c7_i9;
  real_T c7_activeFeetConstraints[2];
  int32_T c7_i10;
  real_T c7_qD[23];
  c7_struct_1ZGMVR6bgCMpDdXTSGnu6G c7_b_reg;
  uint32_T c7_debug_family_var_map[10];
  real_T c7_Jc[348];
  real_T c7_pinvJb[72];
  real_T c7_nargin = 5.0;
  real_T c7_nargout = 1.0;
  real_T c7_y[6];
  real_T c7_a;
  int32_T c7_i11;
  real_T c7_b[174];
  int32_T c7_i12;
  real_T c7_b_a;
  int32_T c7_i13;
  real_T c7_b_b[174];
  int32_T c7_i14;
  int32_T c7_i15;
  int32_T c7_i16;
  int32_T c7_i17;
  int32_T c7_i18;
  int32_T c7_i19;
  int32_T c7_i20;
  int32_T c7_i21;
  int32_T c7_i22;
  int32_T c7_i23;
  int32_T c7_i24;
  int32_T c7_i25;
  int32_T c7_i26;
  real_T c7_c_a[72];
  int32_T c7_i27;
  int32_T c7_i28;
  int32_T c7_i29;
  real_T c7_c_b[72];
  int32_T c7_i30;
  int32_T c7_i31;
  int32_T c7_i32;
  int32_T c7_i33;
  real_T c7_b_y[36];
  int32_T c7_i34;
  int32_T c7_i35;
  real_T c7_d_a;
  int32_T c7_i36;
  static real_T c7_d_b[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c7_c_y[36];
  int32_T c7_i37;
  real_T c7_d_y[36];
  int32_T c7_i38;
  int32_T c7_i39;
  int32_T c7_i40;
  int32_T c7_i41;
  real_T c7_b_Jc[72];
  real_T c7_dv1[72];
  int32_T c7_i42;
  int32_T c7_i43;
  int32_T c7_i44;
  int32_T c7_i45;
  int32_T c7_i46;
  real_T c7_e_b[276];
  int32_T c7_i47;
  real_T c7_e_y[138];
  int32_T c7_i48;
  real_T c7_e_a[72];
  int32_T c7_i49;
  real_T c7_f_b[276];
  int32_T c7_i50;
  real_T c7_g_b[23];
  int32_T c7_i51;
  int32_T c7_i52;
  int32_T c7_i53;
  real_T c7_C[6];
  int32_T c7_i54;
  int32_T c7_i55;
  int32_T c7_i56;
  int32_T c7_i57;
  int32_T c7_i58;
  int32_T c7_i59;
  int32_T c7_i60;
  real_T (*c7_f_y)[6];
  real_T (*c7_b_qD)[23];
  real_T (*c7_b_activeFeetConstraints)[2];
  real_T (*c7_b_JcRightFoot)[174];
  real_T (*c7_b_JcLeftFoot)[174];
  c7_b_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c7_f_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_b_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
    (chartInstance->S, 2);
  c7_b_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c7_b_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 6U, chartInstance->c7_sfEvent);
  for (c7_i7 = 0; c7_i7 < 174; c7_i7++) {
    c7_JcLeftFoot[c7_i7] = (*c7_b_JcLeftFoot)[c7_i7];
  }

  for (c7_i8 = 0; c7_i8 < 174; c7_i8++) {
    c7_JcRightFoot[c7_i8] = (*c7_b_JcRightFoot)[c7_i8];
  }

  for (c7_i9 = 0; c7_i9 < 2; c7_i9++) {
    c7_activeFeetConstraints[c7_i9] = (*c7_b_activeFeetConstraints)[c7_i9];
  }

  for (c7_i10 = 0; c7_i10 < 23; c7_i10++) {
    c7_qD[c7_i10] = (*c7_b_qD)[c7_i10];
  }

  c7_b_reg = chartInstance->c7_reg;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c7_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_Jc, 0U, c7_h_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_pinvJb, 1U, c7_g_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 2U, c7_f_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 3U, c7_f_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_JcLeftFoot, 4U, c7_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_JcRightFoot, 5U, c7_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_activeFeetConstraints, 6U, c7_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_qD, 7U, c7_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_reg, 8U, c7_b_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_y, 9U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  c7_a = c7_activeFeetConstraints[0];
  for (c7_i11 = 0; c7_i11 < 174; c7_i11++) {
    c7_b[c7_i11] = c7_JcLeftFoot[c7_i11];
  }

  for (c7_i12 = 0; c7_i12 < 174; c7_i12++) {
    c7_b[c7_i12] *= c7_a;
  }

  c7_b_a = c7_activeFeetConstraints[1];
  for (c7_i13 = 0; c7_i13 < 174; c7_i13++) {
    c7_b_b[c7_i13] = c7_JcRightFoot[c7_i13];
  }

  for (c7_i14 = 0; c7_i14 < 174; c7_i14++) {
    c7_b_b[c7_i14] *= c7_b_a;
  }

  c7_i15 = 0;
  c7_i16 = 0;
  for (c7_i17 = 0; c7_i17 < 29; c7_i17++) {
    for (c7_i18 = 0; c7_i18 < 6; c7_i18++) {
      c7_Jc[c7_i18 + c7_i15] = c7_b[c7_i18 + c7_i16];
    }

    c7_i15 += 12;
    c7_i16 += 6;
  }

  c7_i19 = 0;
  c7_i20 = 0;
  for (c7_i21 = 0; c7_i21 < 29; c7_i21++) {
    for (c7_i22 = 0; c7_i22 < 6; c7_i22++) {
      c7_Jc[(c7_i22 + c7_i19) + 6] = c7_b_b[c7_i22 + c7_i20];
    }

    c7_i19 += 12;
    c7_i20 += 6;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 7);
  c7_i23 = 0;
  for (c7_i24 = 0; c7_i24 < 12; c7_i24++) {
    c7_i25 = 0;
    for (c7_i26 = 0; c7_i26 < 6; c7_i26++) {
      c7_c_a[c7_i26 + c7_i23] = c7_Jc[c7_i25 + c7_i24];
      c7_i25 += 12;
    }

    c7_i23 += 6;
  }

  c7_i27 = 0;
  for (c7_i28 = 0; c7_i28 < 6; c7_i28++) {
    for (c7_i29 = 0; c7_i29 < 12; c7_i29++) {
      c7_c_b[c7_i29 + c7_i27] = c7_Jc[c7_i29 + c7_i27];
    }

    c7_i27 += 12;
  }

  c7_eml_scalar_eg(chartInstance);
  c7_eml_scalar_eg(chartInstance);
  for (c7_i30 = 0; c7_i30 < 6; c7_i30++) {
    c7_i31 = 0;
    c7_i32 = 0;
    for (c7_i33 = 0; c7_i33 < 6; c7_i33++) {
      c7_b_y[c7_i31 + c7_i30] = 0.0;
      c7_i34 = 0;
      for (c7_i35 = 0; c7_i35 < 12; c7_i35++) {
        c7_b_y[c7_i31 + c7_i30] += c7_c_a[c7_i34 + c7_i30] * c7_c_b[c7_i35 +
          c7_i32];
        c7_i34 += 6;
      }

      c7_i31 += 6;
      c7_i32 += 12;
    }
  }

  c7_d_a = c7_b_reg.pinvDampVb;
  for (c7_i36 = 0; c7_i36 < 36; c7_i36++) {
    c7_c_y[c7_i36] = c7_d_a * c7_d_b[c7_i36];
  }

  for (c7_i37 = 0; c7_i37 < 36; c7_i37++) {
    c7_d_y[c7_i37] = c7_b_y[c7_i37] + c7_c_y[c7_i37];
  }

  c7_i38 = 0;
  for (c7_i39 = 0; c7_i39 < 12; c7_i39++) {
    c7_i40 = 0;
    for (c7_i41 = 0; c7_i41 < 6; c7_i41++) {
      c7_b_Jc[c7_i41 + c7_i38] = c7_Jc[c7_i40 + c7_i39];
      c7_i40 += 12;
    }

    c7_i38 += 6;
  }

  c7_mldivide(chartInstance, c7_d_y, c7_b_Jc, c7_dv1);
  for (c7_i42 = 0; c7_i42 < 72; c7_i42++) {
    c7_pinvJb[c7_i42] = c7_dv1[c7_i42];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 11);
  for (c7_i43 = 0; c7_i43 < 72; c7_i43++) {
    c7_c_a[c7_i43] = -c7_pinvJb[c7_i43];
  }

  c7_i44 = 0;
  for (c7_i45 = 0; c7_i45 < 23; c7_i45++) {
    for (c7_i46 = 0; c7_i46 < 12; c7_i46++) {
      c7_e_b[c7_i46 + c7_i44] = c7_Jc[(c7_i46 + c7_i44) + 72];
    }

    c7_i44 += 12;
  }

  c7_c_eml_scalar_eg(chartInstance);
  c7_c_eml_scalar_eg(chartInstance);
  for (c7_i47 = 0; c7_i47 < 138; c7_i47++) {
    c7_e_y[c7_i47] = 0.0;
  }

  for (c7_i48 = 0; c7_i48 < 72; c7_i48++) {
    c7_e_a[c7_i48] = c7_c_a[c7_i48];
  }

  for (c7_i49 = 0; c7_i49 < 276; c7_i49++) {
    c7_f_b[c7_i49] = c7_e_b[c7_i49];
  }

  c7_b_eml_xgemm(chartInstance, c7_e_a, c7_f_b, c7_e_y);
  for (c7_i50 = 0; c7_i50 < 23; c7_i50++) {
    c7_g_b[c7_i50] = c7_qD[c7_i50];
  }

  c7_d_eml_scalar_eg(chartInstance);
  c7_d_eml_scalar_eg(chartInstance);
  for (c7_i51 = 0; c7_i51 < 6; c7_i51++) {
    c7_y[c7_i51] = 0.0;
  }

  for (c7_i52 = 0; c7_i52 < 6; c7_i52++) {
    c7_y[c7_i52] = 0.0;
  }

  for (c7_i53 = 0; c7_i53 < 6; c7_i53++) {
    c7_C[c7_i53] = c7_y[c7_i53];
  }

  for (c7_i54 = 0; c7_i54 < 6; c7_i54++) {
    c7_y[c7_i54] = c7_C[c7_i54];
  }

  for (c7_i55 = 0; c7_i55 < 6; c7_i55++) {
    c7_C[c7_i55] = c7_y[c7_i55];
  }

  for (c7_i56 = 0; c7_i56 < 6; c7_i56++) {
    c7_y[c7_i56] = c7_C[c7_i56];
  }

  for (c7_i57 = 0; c7_i57 < 6; c7_i57++) {
    c7_y[c7_i57] = 0.0;
    c7_i58 = 0;
    for (c7_i59 = 0; c7_i59 < 23; c7_i59++) {
      c7_y[c7_i57] += c7_e_y[c7_i58 + c7_i57] * c7_g_b[c7_i59];
      c7_i58 += 6;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -11);
  _SFD_SYMBOL_SCOPE_POP();
  for (c7_i60 = 0; c7_i60 < 6; c7_i60++) {
    (*c7_f_y)[c7_i60] = c7_y[c7_i60];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 6U, chartInstance->c7_sfEvent);
}

static void initSimStructsc7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc7_torqueBalancing2012b
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber)
{
}

static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i61;
  real_T c7_b_inData[6];
  int32_T c7_i62;
  real_T c7_u[6];
  const mxArray *c7_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i61 = 0; c7_i61 < 6; c7_i61++) {
    c7_b_inData[c7_i61] = (*(real_T (*)[6])c7_inData)[c7_i61];
  }

  for (c7_i62 = 0; c7_i62 < 6; c7_i62++) {
    c7_u[c7_i62] = c7_b_inData[c7_i62];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_y, const char_T *c7_identifier, real_T
  c7_b_y[6])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_y), &c7_thisId, c7_b_y);
  sf_mex_destroy(&c7_y);
}

static void c7_b_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[6])
{
  real_T c7_dv2[6];
  int32_T c7_i63;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv2, 1, 0, 0U, 1, 0U, 1, 6);
  for (c7_i63 = 0; c7_i63 < 6; c7_i63++) {
    c7_y[c7_i63] = c7_dv2[c7_i63];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_y;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_b_y[6];
  int32_T c7_i64;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_y = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_y), &c7_thisId, c7_b_y);
  sf_mex_destroy(&c7_y);
  for (c7_i64 = 0; c7_i64 < 6; c7_i64++) {
    (*(real_T (*)[6])c7_outData)[c7_i64] = c7_b_y[c7_i64];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  c7_struct_1ZGMVR6bgCMpDdXTSGnu6G c7_u;
  const mxArray *c7_y = NULL;
  real_T c7_b_u;
  const mxArray *c7_b_y = NULL;
  real_T c7_c_u;
  const mxArray *c7_c_y = NULL;
  real_T c7_d_u;
  const mxArray *c7_d_y = NULL;
  real_T c7_e_u;
  const mxArray *c7_e_y = NULL;
  real_T c7_f_u;
  const mxArray *c7_f_y = NULL;
  real_T c7_g_u;
  const mxArray *c7_g_y = NULL;
  real_T c7_h_u;
  const mxArray *c7_h_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(c7_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c7_b_u = c7_u.pinvTol;
  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", &c7_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c7_y, c7_b_y, "pinvTol", "pinvTol", 0);
  c7_c_u = c7_u.pinvDamp;
  c7_c_y = NULL;
  sf_mex_assign(&c7_c_y, sf_mex_create("y", &c7_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c7_y, c7_c_y, "pinvDamp", "pinvDamp", 0);
  c7_d_u = c7_u.pinvDampVb;
  c7_d_y = NULL;
  sf_mex_assign(&c7_d_y, sf_mex_create("y", &c7_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c7_y, c7_d_y, "pinvDampVb", "pinvDampVb", 0);
  c7_e_u = c7_u.HessianQP;
  c7_e_y = NULL;
  sf_mex_assign(&c7_e_y, sf_mex_create("y", &c7_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c7_y, c7_e_y, "HessianQP", "HessianQP", 0);
  c7_f_u = c7_u.impedances;
  c7_f_y = NULL;
  sf_mex_assign(&c7_f_y, sf_mex_create("y", &c7_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c7_y, c7_f_y, "impedances", "impedances", 0);
  c7_g_u = c7_u.dampings;
  c7_g_y = NULL;
  sf_mex_assign(&c7_g_y, sf_mex_create("y", &c7_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c7_y, c7_g_y, "dampings", "dampings", 0);
  c7_h_u = c7_u.norm_tolerance;
  c7_h_y = NULL;
  sf_mex_assign(&c7_h_y, sf_mex_create("y", &c7_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c7_y, c7_h_y, "norm_tolerance", "norm_tolerance", 0);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static c7_struct_1ZGMVR6bgCMpDdXTSGnu6G c7_c_emlrt_marshallIn
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c7_u,
   const emlrtMsgIdentifier *c7_parentId)
{
  c7_struct_1ZGMVR6bgCMpDdXTSGnu6G c7_y;
  emlrtMsgIdentifier c7_thisId;
  static const char * c7_fieldNames[7] = { "pinvTol", "pinvDamp", "pinvDampVb",
    "HessianQP", "impedances", "dampings", "norm_tolerance" };

  c7_thisId.fParent = c7_parentId;
  sf_mex_check_struct(c7_parentId, c7_u, 7, c7_fieldNames, 0U, 0);
  c7_thisId.fIdentifier = "pinvTol";
  c7_y.pinvTol = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
                                        (c7_u, "pinvTol", "pinvTol", 0)),
    &c7_thisId);
  c7_thisId.fIdentifier = "pinvDamp";
  c7_y.pinvDamp = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c7_u, "pinvDamp", "pinvDamp", 0)), &c7_thisId);
  c7_thisId.fIdentifier = "pinvDampVb";
  c7_y.pinvDampVb = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c7_u, "pinvDampVb", "pinvDampVb", 0)), &c7_thisId);
  c7_thisId.fIdentifier = "HessianQP";
  c7_y.HessianQP = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c7_u, "HessianQP", "HessianQP", 0)), &c7_thisId);
  c7_thisId.fIdentifier = "impedances";
  c7_y.impedances = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c7_u, "impedances", "impedances", 0)), &c7_thisId);
  c7_thisId.fIdentifier = "dampings";
  c7_y.dampings = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c7_u, "dampings", "dampings", 0)), &c7_thisId);
  c7_thisId.fIdentifier = "norm_tolerance";
  c7_y.norm_tolerance = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c7_u, "norm_tolerance", "norm_tolerance", 0)), &c7_thisId);
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static real_T c7_d_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  real_T c7_y;
  real_T c7_d0;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_d0, 1, 0, 0U, 0, 0U, 0);
  c7_y = c7_d0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_reg;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  c7_struct_1ZGMVR6bgCMpDdXTSGnu6G c7_y;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_b_reg = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_reg), &c7_thisId);
  sf_mex_destroy(&c7_b_reg);
  *(c7_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i65;
  real_T c7_b_inData[23];
  int32_T c7_i66;
  real_T c7_u[23];
  const mxArray *c7_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i65 = 0; c7_i65 < 23; c7_i65++) {
    c7_b_inData[c7_i65] = (*(real_T (*)[23])c7_inData)[c7_i65];
  }

  for (c7_i66 = 0; c7_i66 < 23; c7_i66++) {
    c7_u[c7_i66] = c7_b_inData[c7_i66];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i67;
  real_T c7_b_inData[2];
  int32_T c7_i68;
  real_T c7_u[2];
  const mxArray *c7_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i67 = 0; c7_i67 < 2; c7_i67++) {
    c7_b_inData[c7_i67] = (*(real_T (*)[2])c7_inData)[c7_i67];
  }

  for (c7_i68 = 0; c7_i68 < 2; c7_i68++) {
    c7_u[c7_i68] = c7_b_inData[c7_i68];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i69;
  int32_T c7_i70;
  int32_T c7_i71;
  real_T c7_b_inData[174];
  int32_T c7_i72;
  int32_T c7_i73;
  int32_T c7_i74;
  real_T c7_u[174];
  const mxArray *c7_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_i69 = 0;
  for (c7_i70 = 0; c7_i70 < 29; c7_i70++) {
    for (c7_i71 = 0; c7_i71 < 6; c7_i71++) {
      c7_b_inData[c7_i71 + c7_i69] = (*(real_T (*)[174])c7_inData)[c7_i71 +
        c7_i69];
    }

    c7_i69 += 6;
  }

  c7_i72 = 0;
  for (c7_i73 = 0; c7_i73 < 29; c7_i73++) {
    for (c7_i74 = 0; c7_i74 < 6; c7_i74++) {
      c7_u[c7_i74 + c7_i72] = c7_b_inData[c7_i74 + c7_i72];
    }

    c7_i72 += 6;
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 6, 29), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static const mxArray *c7_f_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  real_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(real_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_nargout;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_nargout = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_nargout), &c7_thisId);
  sf_mex_destroy(&c7_nargout);
  *(real_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_g_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i75;
  int32_T c7_i76;
  int32_T c7_i77;
  real_T c7_b_inData[72];
  int32_T c7_i78;
  int32_T c7_i79;
  int32_T c7_i80;
  real_T c7_u[72];
  const mxArray *c7_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_i75 = 0;
  for (c7_i76 = 0; c7_i76 < 12; c7_i76++) {
    for (c7_i77 = 0; c7_i77 < 6; c7_i77++) {
      c7_b_inData[c7_i77 + c7_i75] = (*(real_T (*)[72])c7_inData)[c7_i77 +
        c7_i75];
    }

    c7_i75 += 6;
  }

  c7_i78 = 0;
  for (c7_i79 = 0; c7_i79 < 12; c7_i79++) {
    for (c7_i80 = 0; c7_i80 < 6; c7_i80++) {
      c7_u[c7_i80 + c7_i78] = c7_b_inData[c7_i80 + c7_i78];
    }

    c7_i78 += 6;
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_e_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[72])
{
  real_T c7_dv3[72];
  int32_T c7_i81;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv3, 1, 0, 0U, 1, 0U, 2, 6, 12);
  for (c7_i81 = 0; c7_i81 < 72; c7_i81++) {
    c7_y[c7_i81] = c7_dv3[c7_i81];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_pinvJb;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[72];
  int32_T c7_i82;
  int32_T c7_i83;
  int32_T c7_i84;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_pinvJb = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_pinvJb), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_pinvJb);
  c7_i82 = 0;
  for (c7_i83 = 0; c7_i83 < 12; c7_i83++) {
    for (c7_i84 = 0; c7_i84 < 6; c7_i84++) {
      (*(real_T (*)[72])c7_outData)[c7_i84 + c7_i82] = c7_y[c7_i84 + c7_i82];
    }

    c7_i82 += 6;
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_h_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i85;
  int32_T c7_i86;
  int32_T c7_i87;
  real_T c7_b_inData[348];
  int32_T c7_i88;
  int32_T c7_i89;
  int32_T c7_i90;
  real_T c7_u[348];
  const mxArray *c7_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_i85 = 0;
  for (c7_i86 = 0; c7_i86 < 29; c7_i86++) {
    for (c7_i87 = 0; c7_i87 < 12; c7_i87++) {
      c7_b_inData[c7_i87 + c7_i85] = (*(real_T (*)[348])c7_inData)[c7_i87 +
        c7_i85];
    }

    c7_i85 += 12;
  }

  c7_i88 = 0;
  for (c7_i89 = 0; c7_i89 < 29; c7_i89++) {
    for (c7_i90 = 0; c7_i90 < 12; c7_i90++) {
      c7_u[c7_i90 + c7_i88] = c7_b_inData[c7_i90 + c7_i88];
    }

    c7_i88 += 12;
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 12, 29), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_f_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[348])
{
  real_T c7_dv4[348];
  int32_T c7_i91;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv4, 1, 0, 0U, 1, 0U, 2, 12,
                29);
  for (c7_i91 = 0; c7_i91 < 348; c7_i91++) {
    c7_y[c7_i91] = c7_dv4[c7_i91];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_Jc;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[348];
  int32_T c7_i92;
  int32_T c7_i93;
  int32_T c7_i94;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_Jc = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_Jc), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_Jc);
  c7_i92 = 0;
  for (c7_i93 = 0; c7_i93 < 29; c7_i93++) {
    for (c7_i94 = 0; c7_i94 < 12; c7_i94++) {
      (*(real_T (*)[348])c7_outData)[c7_i94 + c7_i92] = c7_y[c7_i94 + c7_i92];
    }

    c7_i92 += 12;
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

const mxArray *sf_c7_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c7_nameCaptureInfo;
  c7_ResolvedFunctionInfo c7_info[125];
  const mxArray *c7_m1 = NULL;
  int32_T c7_i95;
  c7_ResolvedFunctionInfo *c7_r1;
  c7_nameCaptureInfo = NULL;
  c7_nameCaptureInfo = NULL;
  c7_info_helper(c7_info);
  c7_b_info_helper(c7_info);
  sf_mex_assign(&c7_m1, sf_mex_createstruct("nameCaptureInfo", 1, 125), FALSE);
  for (c7_i95 = 0; c7_i95 < 125; c7_i95++) {
    c7_r1 = &c7_info[c7_i95];
    sf_mex_addfield(c7_m1, sf_mex_create("nameCaptureInfo", c7_r1->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c7_r1->context)), "context", "nameCaptureInfo",
                    c7_i95);
    sf_mex_addfield(c7_m1, sf_mex_create("nameCaptureInfo", c7_r1->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c7_r1->name)), "name", "nameCaptureInfo", c7_i95);
    sf_mex_addfield(c7_m1, sf_mex_create("nameCaptureInfo", c7_r1->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c7_r1->dominantType)), "dominantType",
                    "nameCaptureInfo", c7_i95);
    sf_mex_addfield(c7_m1, sf_mex_create("nameCaptureInfo", c7_r1->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c7_r1->resolved)), "resolved", "nameCaptureInfo",
                    c7_i95);
    sf_mex_addfield(c7_m1, sf_mex_create("nameCaptureInfo", &c7_r1->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c7_i95);
    sf_mex_addfield(c7_m1, sf_mex_create("nameCaptureInfo", &c7_r1->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c7_i95);
    sf_mex_addfield(c7_m1, sf_mex_create("nameCaptureInfo", &c7_r1->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c7_i95);
    sf_mex_addfield(c7_m1, sf_mex_create("nameCaptureInfo", &c7_r1->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c7_i95);
  }

  sf_mex_assign(&c7_nameCaptureInfo, c7_m1, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c7_nameCaptureInfo);
  return c7_nameCaptureInfo;
}

static void c7_info_helper(c7_ResolvedFunctionInfo c7_info[125])
{
  c7_info[0].context = "";
  c7_info[0].name = "mtimes";
  c7_info[0].dominantType = "double";
  c7_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[0].fileTimeLo = 1289519692U;
  c7_info[0].fileTimeHi = 0U;
  c7_info[0].mFileTimeLo = 0U;
  c7_info[0].mFileTimeHi = 0U;
  c7_info[1].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[1].name = "eml_index_class";
  c7_info[1].dominantType = "";
  c7_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[1].fileTimeLo = 1323170578U;
  c7_info[1].fileTimeHi = 0U;
  c7_info[1].mFileTimeLo = 0U;
  c7_info[1].mFileTimeHi = 0U;
  c7_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[2].name = "eml_scalar_eg";
  c7_info[2].dominantType = "double";
  c7_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[2].fileTimeLo = 1286818796U;
  c7_info[2].fileTimeHi = 0U;
  c7_info[2].mFileTimeLo = 0U;
  c7_info[2].mFileTimeHi = 0U;
  c7_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[3].name = "eml_xgemm";
  c7_info[3].dominantType = "char";
  c7_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c7_info[3].fileTimeLo = 1299076772U;
  c7_info[3].fileTimeHi = 0U;
  c7_info[3].mFileTimeLo = 0U;
  c7_info[3].mFileTimeHi = 0U;
  c7_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c7_info[4].name = "eml_blas_inline";
  c7_info[4].dominantType = "";
  c7_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c7_info[4].fileTimeLo = 1299076768U;
  c7_info[4].fileTimeHi = 0U;
  c7_info[4].mFileTimeLo = 0U;
  c7_info[4].mFileTimeHi = 0U;
  c7_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c7_info[5].name = "mtimes";
  c7_info[5].dominantType = "double";
  c7_info[5].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[5].fileTimeLo = 1289519692U;
  c7_info[5].fileTimeHi = 0U;
  c7_info[5].mFileTimeLo = 0U;
  c7_info[5].mFileTimeHi = 0U;
  c7_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c7_info[6].name = "eml_index_class";
  c7_info[6].dominantType = "";
  c7_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[6].fileTimeLo = 1323170578U;
  c7_info[6].fileTimeHi = 0U;
  c7_info[6].mFileTimeLo = 0U;
  c7_info[6].mFileTimeHi = 0U;
  c7_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c7_info[7].name = "eml_scalar_eg";
  c7_info[7].dominantType = "double";
  c7_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[7].fileTimeLo = 1286818796U;
  c7_info[7].fileTimeHi = 0U;
  c7_info[7].mFileTimeLo = 0U;
  c7_info[7].mFileTimeHi = 0U;
  c7_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c7_info[8].name = "eml_refblas_xgemm";
  c7_info[8].dominantType = "char";
  c7_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c7_info[8].fileTimeLo = 1299076774U;
  c7_info[8].fileTimeHi = 0U;
  c7_info[8].mFileTimeLo = 0U;
  c7_info[8].mFileTimeHi = 0U;
  c7_info[9].context = "";
  c7_info[9].name = "eye";
  c7_info[9].dominantType = "double";
  c7_info[9].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c7_info[9].fileTimeLo = 1286818688U;
  c7_info[9].fileTimeHi = 0U;
  c7_info[9].mFileTimeLo = 0U;
  c7_info[9].mFileTimeHi = 0U;
  c7_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c7_info[10].name = "eml_assert_valid_size_arg";
  c7_info[10].dominantType = "double";
  c7_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c7_info[10].fileTimeLo = 1286818694U;
  c7_info[10].fileTimeHi = 0U;
  c7_info[10].mFileTimeLo = 0U;
  c7_info[10].mFileTimeHi = 0U;
  c7_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c7_info[11].name = "isinf";
  c7_info[11].dominantType = "double";
  c7_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c7_info[11].fileTimeLo = 1286818760U;
  c7_info[11].fileTimeHi = 0U;
  c7_info[11].mFileTimeLo = 0U;
  c7_info[11].mFileTimeHi = 0U;
  c7_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c7_info[12].name = "mtimes";
  c7_info[12].dominantType = "double";
  c7_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[12].fileTimeLo = 1289519692U;
  c7_info[12].fileTimeHi = 0U;
  c7_info[12].mFileTimeLo = 0U;
  c7_info[12].mFileTimeHi = 0U;
  c7_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c7_info[13].name = "eml_index_class";
  c7_info[13].dominantType = "";
  c7_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[13].fileTimeLo = 1323170578U;
  c7_info[13].fileTimeHi = 0U;
  c7_info[13].mFileTimeLo = 0U;
  c7_info[13].mFileTimeHi = 0U;
  c7_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c7_info[14].name = "intmax";
  c7_info[14].dominantType = "char";
  c7_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c7_info[14].fileTimeLo = 1311255316U;
  c7_info[14].fileTimeHi = 0U;
  c7_info[14].mFileTimeLo = 0U;
  c7_info[14].mFileTimeHi = 0U;
  c7_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c7_info[15].name = "eml_is_float_class";
  c7_info[15].dominantType = "char";
  c7_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c7_info[15].fileTimeLo = 1286818782U;
  c7_info[15].fileTimeHi = 0U;
  c7_info[15].mFileTimeLo = 0U;
  c7_info[15].mFileTimeHi = 0U;
  c7_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c7_info[16].name = "min";
  c7_info[16].dominantType = "double";
  c7_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c7_info[16].fileTimeLo = 1311255318U;
  c7_info[16].fileTimeHi = 0U;
  c7_info[16].mFileTimeLo = 0U;
  c7_info[16].mFileTimeHi = 0U;
  c7_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c7_info[17].name = "eml_min_or_max";
  c7_info[17].dominantType = "char";
  c7_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c7_info[17].fileTimeLo = 1334071490U;
  c7_info[17].fileTimeHi = 0U;
  c7_info[17].mFileTimeLo = 0U;
  c7_info[17].mFileTimeHi = 0U;
  c7_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c7_info[18].name = "eml_scalar_eg";
  c7_info[18].dominantType = "double";
  c7_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[18].fileTimeLo = 1286818796U;
  c7_info[18].fileTimeHi = 0U;
  c7_info[18].mFileTimeLo = 0U;
  c7_info[18].mFileTimeHi = 0U;
  c7_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c7_info[19].name = "eml_scalexp_alloc";
  c7_info[19].dominantType = "double";
  c7_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c7_info[19].fileTimeLo = 1352424860U;
  c7_info[19].fileTimeHi = 0U;
  c7_info[19].mFileTimeLo = 0U;
  c7_info[19].mFileTimeHi = 0U;
  c7_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c7_info[20].name = "eml_index_class";
  c7_info[20].dominantType = "";
  c7_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[20].fileTimeLo = 1323170578U;
  c7_info[20].fileTimeHi = 0U;
  c7_info[20].mFileTimeLo = 0U;
  c7_info[20].mFileTimeHi = 0U;
  c7_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c7_info[21].name = "eml_scalar_eg";
  c7_info[21].dominantType = "double";
  c7_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[21].fileTimeLo = 1286818796U;
  c7_info[21].fileTimeHi = 0U;
  c7_info[21].mFileTimeLo = 0U;
  c7_info[21].mFileTimeHi = 0U;
  c7_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c7_info[22].name = "eml_index_class";
  c7_info[22].dominantType = "";
  c7_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[22].fileTimeLo = 1323170578U;
  c7_info[22].fileTimeHi = 0U;
  c7_info[22].mFileTimeLo = 0U;
  c7_info[22].mFileTimeHi = 0U;
  c7_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c7_info[23].name = "eml_int_forloop_overflow_check";
  c7_info[23].dominantType = "";
  c7_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[23].fileTimeLo = 1346510340U;
  c7_info[23].fileTimeHi = 0U;
  c7_info[23].mFileTimeLo = 0U;
  c7_info[23].mFileTimeHi = 0U;
  c7_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c7_info[24].name = "intmax";
  c7_info[24].dominantType = "char";
  c7_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c7_info[24].fileTimeLo = 1311255316U;
  c7_info[24].fileTimeHi = 0U;
  c7_info[24].mFileTimeLo = 0U;
  c7_info[24].mFileTimeHi = 0U;
  c7_info[25].context = "";
  c7_info[25].name = "mldivide";
  c7_info[25].dominantType = "double";
  c7_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c7_info[25].fileTimeLo = 1357951548U;
  c7_info[25].fileTimeHi = 0U;
  c7_info[25].mFileTimeLo = 1319729966U;
  c7_info[25].mFileTimeHi = 0U;
  c7_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c7_info[26].name = "eml_lusolve";
  c7_info[26].dominantType = "double";
  c7_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c7_info[26].fileTimeLo = 1309451196U;
  c7_info[26].fileTimeHi = 0U;
  c7_info[26].mFileTimeLo = 0U;
  c7_info[26].mFileTimeHi = 0U;
  c7_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c7_info[27].name = "eml_index_class";
  c7_info[27].dominantType = "";
  c7_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[27].fileTimeLo = 1323170578U;
  c7_info[27].fileTimeHi = 0U;
  c7_info[27].mFileTimeLo = 0U;
  c7_info[27].mFileTimeHi = 0U;
  c7_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c7_info[28].name = "eml_index_class";
  c7_info[28].dominantType = "";
  c7_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[28].fileTimeLo = 1323170578U;
  c7_info[28].fileTimeHi = 0U;
  c7_info[28].mFileTimeLo = 0U;
  c7_info[28].mFileTimeHi = 0U;
  c7_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c7_info[29].name = "eml_xgetrf";
  c7_info[29].dominantType = "double";
  c7_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c7_info[29].fileTimeLo = 1286818806U;
  c7_info[29].fileTimeHi = 0U;
  c7_info[29].mFileTimeLo = 0U;
  c7_info[29].mFileTimeHi = 0U;
  c7_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c7_info[30].name = "eml_lapack_xgetrf";
  c7_info[30].dominantType = "double";
  c7_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c7_info[30].fileTimeLo = 1286818810U;
  c7_info[30].fileTimeHi = 0U;
  c7_info[30].mFileTimeLo = 0U;
  c7_info[30].mFileTimeHi = 0U;
  c7_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c7_info[31].name = "eml_matlab_zgetrf";
  c7_info[31].dominantType = "double";
  c7_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[31].fileTimeLo = 1302688994U;
  c7_info[31].fileTimeHi = 0U;
  c7_info[31].mFileTimeLo = 0U;
  c7_info[31].mFileTimeHi = 0U;
  c7_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[32].name = "realmin";
  c7_info[32].dominantType = "char";
  c7_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c7_info[32].fileTimeLo = 1307651242U;
  c7_info[32].fileTimeHi = 0U;
  c7_info[32].mFileTimeLo = 0U;
  c7_info[32].mFileTimeHi = 0U;
  c7_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c7_info[33].name = "eml_realmin";
  c7_info[33].dominantType = "char";
  c7_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c7_info[33].fileTimeLo = 1307651244U;
  c7_info[33].fileTimeHi = 0U;
  c7_info[33].mFileTimeLo = 0U;
  c7_info[33].mFileTimeHi = 0U;
  c7_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c7_info[34].name = "eml_float_model";
  c7_info[34].dominantType = "char";
  c7_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c7_info[34].fileTimeLo = 1326727996U;
  c7_info[34].fileTimeHi = 0U;
  c7_info[34].mFileTimeLo = 0U;
  c7_info[34].mFileTimeHi = 0U;
  c7_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[35].name = "eps";
  c7_info[35].dominantType = "char";
  c7_info[35].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c7_info[35].fileTimeLo = 1326727996U;
  c7_info[35].fileTimeHi = 0U;
  c7_info[35].mFileTimeLo = 0U;
  c7_info[35].mFileTimeHi = 0U;
  c7_info[36].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c7_info[36].name = "eml_is_float_class";
  c7_info[36].dominantType = "char";
  c7_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c7_info[36].fileTimeLo = 1286818782U;
  c7_info[36].fileTimeHi = 0U;
  c7_info[36].mFileTimeLo = 0U;
  c7_info[36].mFileTimeHi = 0U;
  c7_info[37].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c7_info[37].name = "eml_eps";
  c7_info[37].dominantType = "char";
  c7_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c7_info[37].fileTimeLo = 1326727996U;
  c7_info[37].fileTimeHi = 0U;
  c7_info[37].mFileTimeLo = 0U;
  c7_info[37].mFileTimeHi = 0U;
  c7_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c7_info[38].name = "eml_float_model";
  c7_info[38].dominantType = "char";
  c7_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c7_info[38].fileTimeLo = 1326727996U;
  c7_info[38].fileTimeHi = 0U;
  c7_info[38].mFileTimeLo = 0U;
  c7_info[38].mFileTimeHi = 0U;
  c7_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[39].name = "min";
  c7_info[39].dominantType = "coder.internal.indexInt";
  c7_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c7_info[39].fileTimeLo = 1311255318U;
  c7_info[39].fileTimeHi = 0U;
  c7_info[39].mFileTimeLo = 0U;
  c7_info[39].mFileTimeHi = 0U;
  c7_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c7_info[40].name = "eml_scalar_eg";
  c7_info[40].dominantType = "coder.internal.indexInt";
  c7_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[40].fileTimeLo = 1286818796U;
  c7_info[40].fileTimeHi = 0U;
  c7_info[40].mFileTimeLo = 0U;
  c7_info[40].mFileTimeHi = 0U;
  c7_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c7_info[41].name = "eml_scalexp_alloc";
  c7_info[41].dominantType = "coder.internal.indexInt";
  c7_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c7_info[41].fileTimeLo = 1352424860U;
  c7_info[41].fileTimeHi = 0U;
  c7_info[41].mFileTimeLo = 0U;
  c7_info[41].mFileTimeHi = 0U;
  c7_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c7_info[42].name = "eml_scalar_eg";
  c7_info[42].dominantType = "coder.internal.indexInt";
  c7_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[42].fileTimeLo = 1286818796U;
  c7_info[42].fileTimeHi = 0U;
  c7_info[42].mFileTimeLo = 0U;
  c7_info[42].mFileTimeHi = 0U;
  c7_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[43].name = "colon";
  c7_info[43].dominantType = "double";
  c7_info[43].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c7_info[43].fileTimeLo = 1348191928U;
  c7_info[43].fileTimeHi = 0U;
  c7_info[43].mFileTimeLo = 0U;
  c7_info[43].mFileTimeHi = 0U;
  c7_info[44].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c7_info[44].name = "colon";
  c7_info[44].dominantType = "double";
  c7_info[44].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c7_info[44].fileTimeLo = 1348191928U;
  c7_info[44].fileTimeHi = 0U;
  c7_info[44].mFileTimeLo = 0U;
  c7_info[44].mFileTimeHi = 0U;
  c7_info[45].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c7_info[45].name = "floor";
  c7_info[45].dominantType = "double";
  c7_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c7_info[45].fileTimeLo = 1343830380U;
  c7_info[45].fileTimeHi = 0U;
  c7_info[45].mFileTimeLo = 0U;
  c7_info[45].mFileTimeHi = 0U;
  c7_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c7_info[46].name = "eml_scalar_floor";
  c7_info[46].dominantType = "double";
  c7_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c7_info[46].fileTimeLo = 1286818726U;
  c7_info[46].fileTimeHi = 0U;
  c7_info[46].mFileTimeLo = 0U;
  c7_info[46].mFileTimeHi = 0U;
  c7_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c7_info[47].name = "intmin";
  c7_info[47].dominantType = "char";
  c7_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c7_info[47].fileTimeLo = 1311255318U;
  c7_info[47].fileTimeHi = 0U;
  c7_info[47].mFileTimeLo = 0U;
  c7_info[47].mFileTimeHi = 0U;
  c7_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c7_info[48].name = "intmax";
  c7_info[48].dominantType = "char";
  c7_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c7_info[48].fileTimeLo = 1311255316U;
  c7_info[48].fileTimeHi = 0U;
  c7_info[48].mFileTimeLo = 0U;
  c7_info[48].mFileTimeHi = 0U;
  c7_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c7_info[49].name = "intmin";
  c7_info[49].dominantType = "char";
  c7_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c7_info[49].fileTimeLo = 1311255318U;
  c7_info[49].fileTimeHi = 0U;
  c7_info[49].mFileTimeLo = 0U;
  c7_info[49].mFileTimeHi = 0U;
  c7_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c7_info[50].name = "intmax";
  c7_info[50].dominantType = "char";
  c7_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c7_info[50].fileTimeLo = 1311255316U;
  c7_info[50].fileTimeHi = 0U;
  c7_info[50].mFileTimeLo = 0U;
  c7_info[50].mFileTimeHi = 0U;
  c7_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c7_info[51].name = "eml_isa_uint";
  c7_info[51].dominantType = "coder.internal.indexInt";
  c7_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c7_info[51].fileTimeLo = 1286818784U;
  c7_info[51].fileTimeHi = 0U;
  c7_info[51].mFileTimeLo = 0U;
  c7_info[51].mFileTimeHi = 0U;
  c7_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c7_info[52].name = "eml_unsigned_class";
  c7_info[52].dominantType = "char";
  c7_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c7_info[52].fileTimeLo = 1323170580U;
  c7_info[52].fileTimeHi = 0U;
  c7_info[52].mFileTimeLo = 0U;
  c7_info[52].mFileTimeHi = 0U;
  c7_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c7_info[53].name = "eml_index_class";
  c7_info[53].dominantType = "";
  c7_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[53].fileTimeLo = 1323170578U;
  c7_info[53].fileTimeHi = 0U;
  c7_info[53].mFileTimeLo = 0U;
  c7_info[53].mFileTimeHi = 0U;
  c7_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c7_info[54].name = "eml_index_class";
  c7_info[54].dominantType = "";
  c7_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[54].fileTimeLo = 1323170578U;
  c7_info[54].fileTimeHi = 0U;
  c7_info[54].mFileTimeLo = 0U;
  c7_info[54].mFileTimeHi = 0U;
  c7_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c7_info[55].name = "intmax";
  c7_info[55].dominantType = "char";
  c7_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c7_info[55].fileTimeLo = 1311255316U;
  c7_info[55].fileTimeHi = 0U;
  c7_info[55].mFileTimeLo = 0U;
  c7_info[55].mFileTimeHi = 0U;
  c7_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c7_info[56].name = "eml_isa_uint";
  c7_info[56].dominantType = "coder.internal.indexInt";
  c7_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c7_info[56].fileTimeLo = 1286818784U;
  c7_info[56].fileTimeHi = 0U;
  c7_info[56].mFileTimeLo = 0U;
  c7_info[56].mFileTimeHi = 0U;
  c7_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c7_info[57].name = "eml_index_plus";
  c7_info[57].dominantType = "double";
  c7_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[57].fileTimeLo = 1286818778U;
  c7_info[57].fileTimeHi = 0U;
  c7_info[57].mFileTimeLo = 0U;
  c7_info[57].mFileTimeHi = 0U;
  c7_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[58].name = "eml_index_class";
  c7_info[58].dominantType = "";
  c7_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[58].fileTimeLo = 1323170578U;
  c7_info[58].fileTimeHi = 0U;
  c7_info[58].mFileTimeLo = 0U;
  c7_info[58].mFileTimeHi = 0U;
  c7_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c7_info[59].name = "eml_int_forloop_overflow_check";
  c7_info[59].dominantType = "";
  c7_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[59].fileTimeLo = 1346510340U;
  c7_info[59].fileTimeHi = 0U;
  c7_info[59].mFileTimeLo = 0U;
  c7_info[59].mFileTimeHi = 0U;
  c7_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[60].name = "eml_index_class";
  c7_info[60].dominantType = "";
  c7_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[60].fileTimeLo = 1323170578U;
  c7_info[60].fileTimeHi = 0U;
  c7_info[60].mFileTimeLo = 0U;
  c7_info[60].mFileTimeHi = 0U;
  c7_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[61].name = "eml_index_plus";
  c7_info[61].dominantType = "double";
  c7_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[61].fileTimeLo = 1286818778U;
  c7_info[61].fileTimeHi = 0U;
  c7_info[61].mFileTimeLo = 0U;
  c7_info[61].mFileTimeHi = 0U;
  c7_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[62].name = "eml_int_forloop_overflow_check";
  c7_info[62].dominantType = "";
  c7_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[62].fileTimeLo = 1346510340U;
  c7_info[62].fileTimeHi = 0U;
  c7_info[62].mFileTimeLo = 0U;
  c7_info[62].mFileTimeHi = 0U;
  c7_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[63].name = "eml_index_minus";
  c7_info[63].dominantType = "double";
  c7_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c7_info[63].fileTimeLo = 1286818778U;
  c7_info[63].fileTimeHi = 0U;
  c7_info[63].mFileTimeLo = 0U;
  c7_info[63].mFileTimeHi = 0U;
}

static void c7_b_info_helper(c7_ResolvedFunctionInfo c7_info[125])
{
  c7_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c7_info[64].name = "eml_index_class";
  c7_info[64].dominantType = "";
  c7_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[64].fileTimeLo = 1323170578U;
  c7_info[64].fileTimeHi = 0U;
  c7_info[64].mFileTimeLo = 0U;
  c7_info[64].mFileTimeHi = 0U;
  c7_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[65].name = "eml_index_minus";
  c7_info[65].dominantType = "coder.internal.indexInt";
  c7_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c7_info[65].fileTimeLo = 1286818778U;
  c7_info[65].fileTimeHi = 0U;
  c7_info[65].mFileTimeLo = 0U;
  c7_info[65].mFileTimeHi = 0U;
  c7_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[66].name = "eml_index_times";
  c7_info[66].dominantType = "coder.internal.indexInt";
  c7_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c7_info[66].fileTimeLo = 1286818780U;
  c7_info[66].fileTimeHi = 0U;
  c7_info[66].mFileTimeLo = 0U;
  c7_info[66].mFileTimeHi = 0U;
  c7_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c7_info[67].name = "eml_index_class";
  c7_info[67].dominantType = "";
  c7_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[67].fileTimeLo = 1323170578U;
  c7_info[67].fileTimeHi = 0U;
  c7_info[67].mFileTimeLo = 0U;
  c7_info[67].mFileTimeHi = 0U;
  c7_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[68].name = "eml_index_plus";
  c7_info[68].dominantType = "coder.internal.indexInt";
  c7_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[68].fileTimeLo = 1286818778U;
  c7_info[68].fileTimeHi = 0U;
  c7_info[68].mFileTimeLo = 0U;
  c7_info[68].mFileTimeHi = 0U;
  c7_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[69].name = "eml_ixamax";
  c7_info[69].dominantType = "double";
  c7_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c7_info[69].fileTimeLo = 1299076770U;
  c7_info[69].fileTimeHi = 0U;
  c7_info[69].mFileTimeLo = 0U;
  c7_info[69].mFileTimeHi = 0U;
  c7_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c7_info[70].name = "eml_blas_inline";
  c7_info[70].dominantType = "";
  c7_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c7_info[70].fileTimeLo = 1299076768U;
  c7_info[70].fileTimeHi = 0U;
  c7_info[70].mFileTimeLo = 0U;
  c7_info[70].mFileTimeHi = 0U;
  c7_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c7_info[71].name = "length";
  c7_info[71].dominantType = "double";
  c7_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c7_info[71].fileTimeLo = 1303146206U;
  c7_info[71].fileTimeHi = 0U;
  c7_info[71].mFileTimeLo = 0U;
  c7_info[71].mFileTimeHi = 0U;
  c7_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c7_info[72].name = "eml_index_class";
  c7_info[72].dominantType = "";
  c7_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[72].fileTimeLo = 1323170578U;
  c7_info[72].fileTimeHi = 0U;
  c7_info[72].mFileTimeLo = 0U;
  c7_info[72].mFileTimeHi = 0U;
  c7_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c7_info[73].name = "eml_index_class";
  c7_info[73].dominantType = "";
  c7_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[73].fileTimeLo = 1323170578U;
  c7_info[73].fileTimeHi = 0U;
  c7_info[73].mFileTimeLo = 0U;
  c7_info[73].mFileTimeHi = 0U;
  c7_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c7_info[74].name = "eml_refblas_ixamax";
  c7_info[74].dominantType = "double";
  c7_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c7_info[74].fileTimeLo = 1299076770U;
  c7_info[74].fileTimeHi = 0U;
  c7_info[74].mFileTimeLo = 0U;
  c7_info[74].mFileTimeHi = 0U;
  c7_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c7_info[75].name = "eml_index_class";
  c7_info[75].dominantType = "";
  c7_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[75].fileTimeLo = 1323170578U;
  c7_info[75].fileTimeHi = 0U;
  c7_info[75].mFileTimeLo = 0U;
  c7_info[75].mFileTimeHi = 0U;
  c7_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c7_info[76].name = "eml_xcabs1";
  c7_info[76].dominantType = "double";
  c7_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c7_info[76].fileTimeLo = 1286818706U;
  c7_info[76].fileTimeHi = 0U;
  c7_info[76].mFileTimeLo = 0U;
  c7_info[76].mFileTimeHi = 0U;
  c7_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c7_info[77].name = "abs";
  c7_info[77].dominantType = "double";
  c7_info[77].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c7_info[77].fileTimeLo = 1343830366U;
  c7_info[77].fileTimeHi = 0U;
  c7_info[77].mFileTimeLo = 0U;
  c7_info[77].mFileTimeHi = 0U;
  c7_info[78].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c7_info[78].name = "eml_scalar_abs";
  c7_info[78].dominantType = "double";
  c7_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c7_info[78].fileTimeLo = 1286818712U;
  c7_info[78].fileTimeHi = 0U;
  c7_info[78].mFileTimeLo = 0U;
  c7_info[78].mFileTimeHi = 0U;
  c7_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c7_info[79].name = "eml_int_forloop_overflow_check";
  c7_info[79].dominantType = "";
  c7_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[79].fileTimeLo = 1346510340U;
  c7_info[79].fileTimeHi = 0U;
  c7_info[79].mFileTimeLo = 0U;
  c7_info[79].mFileTimeHi = 0U;
  c7_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c7_info[80].name = "eml_index_plus";
  c7_info[80].dominantType = "coder.internal.indexInt";
  c7_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[80].fileTimeLo = 1286818778U;
  c7_info[80].fileTimeHi = 0U;
  c7_info[80].mFileTimeLo = 0U;
  c7_info[80].mFileTimeHi = 0U;
  c7_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[81].name = "eml_xswap";
  c7_info[81].dominantType = "double";
  c7_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c7_info[81].fileTimeLo = 1299076778U;
  c7_info[81].fileTimeHi = 0U;
  c7_info[81].mFileTimeLo = 0U;
  c7_info[81].mFileTimeHi = 0U;
  c7_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c7_info[82].name = "eml_blas_inline";
  c7_info[82].dominantType = "";
  c7_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c7_info[82].fileTimeLo = 1299076768U;
  c7_info[82].fileTimeHi = 0U;
  c7_info[82].mFileTimeLo = 0U;
  c7_info[82].mFileTimeHi = 0U;
  c7_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c7_info[83].name = "eml_index_class";
  c7_info[83].dominantType = "";
  c7_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[83].fileTimeLo = 1323170578U;
  c7_info[83].fileTimeHi = 0U;
  c7_info[83].mFileTimeLo = 0U;
  c7_info[83].mFileTimeHi = 0U;
  c7_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c7_info[84].name = "eml_refblas_xswap";
  c7_info[84].dominantType = "double";
  c7_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c7_info[84].fileTimeLo = 1299076786U;
  c7_info[84].fileTimeHi = 0U;
  c7_info[84].mFileTimeLo = 0U;
  c7_info[84].mFileTimeHi = 0U;
  c7_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c7_info[85].name = "eml_index_class";
  c7_info[85].dominantType = "";
  c7_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[85].fileTimeLo = 1323170578U;
  c7_info[85].fileTimeHi = 0U;
  c7_info[85].mFileTimeLo = 0U;
  c7_info[85].mFileTimeHi = 0U;
  c7_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c7_info[86].name = "abs";
  c7_info[86].dominantType = "coder.internal.indexInt";
  c7_info[86].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c7_info[86].fileTimeLo = 1343830366U;
  c7_info[86].fileTimeHi = 0U;
  c7_info[86].mFileTimeLo = 0U;
  c7_info[86].mFileTimeHi = 0U;
  c7_info[87].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c7_info[87].name = "eml_scalar_abs";
  c7_info[87].dominantType = "coder.internal.indexInt";
  c7_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c7_info[87].fileTimeLo = 1286818712U;
  c7_info[87].fileTimeHi = 0U;
  c7_info[87].mFileTimeLo = 0U;
  c7_info[87].mFileTimeHi = 0U;
  c7_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c7_info[88].name = "eml_int_forloop_overflow_check";
  c7_info[88].dominantType = "";
  c7_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[88].fileTimeLo = 1346510340U;
  c7_info[88].fileTimeHi = 0U;
  c7_info[88].mFileTimeLo = 0U;
  c7_info[88].mFileTimeHi = 0U;
  c7_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c7_info[89].name = "eml_index_plus";
  c7_info[89].dominantType = "coder.internal.indexInt";
  c7_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[89].fileTimeLo = 1286818778U;
  c7_info[89].fileTimeHi = 0U;
  c7_info[89].mFileTimeLo = 0U;
  c7_info[89].mFileTimeHi = 0U;
  c7_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[90].name = "eml_div";
  c7_info[90].dominantType = "double";
  c7_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c7_info[90].fileTimeLo = 1313347810U;
  c7_info[90].fileTimeHi = 0U;
  c7_info[90].mFileTimeLo = 0U;
  c7_info[90].mFileTimeHi = 0U;
  c7_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c7_info[91].name = "eml_xgeru";
  c7_info[91].dominantType = "double";
  c7_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c7_info[91].fileTimeLo = 1299076774U;
  c7_info[91].fileTimeHi = 0U;
  c7_info[91].mFileTimeLo = 0U;
  c7_info[91].mFileTimeHi = 0U;
  c7_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c7_info[92].name = "eml_blas_inline";
  c7_info[92].dominantType = "";
  c7_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c7_info[92].fileTimeLo = 1299076768U;
  c7_info[92].fileTimeHi = 0U;
  c7_info[92].mFileTimeLo = 0U;
  c7_info[92].mFileTimeHi = 0U;
  c7_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c7_info[93].name = "eml_xger";
  c7_info[93].dominantType = "double";
  c7_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c7_info[93].fileTimeLo = 1299076774U;
  c7_info[93].fileTimeHi = 0U;
  c7_info[93].mFileTimeLo = 0U;
  c7_info[93].mFileTimeHi = 0U;
  c7_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c7_info[94].name = "eml_blas_inline";
  c7_info[94].dominantType = "";
  c7_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c7_info[94].fileTimeLo = 1299076768U;
  c7_info[94].fileTimeHi = 0U;
  c7_info[94].mFileTimeLo = 0U;
  c7_info[94].mFileTimeHi = 0U;
  c7_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c7_info[95].name = "intmax";
  c7_info[95].dominantType = "char";
  c7_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c7_info[95].fileTimeLo = 1311255316U;
  c7_info[95].fileTimeHi = 0U;
  c7_info[95].mFileTimeLo = 0U;
  c7_info[95].mFileTimeHi = 0U;
  c7_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c7_info[96].name = "min";
  c7_info[96].dominantType = "double";
  c7_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c7_info[96].fileTimeLo = 1311255318U;
  c7_info[96].fileTimeHi = 0U;
  c7_info[96].mFileTimeLo = 0U;
  c7_info[96].mFileTimeHi = 0U;
  c7_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c7_info[97].name = "mtimes";
  c7_info[97].dominantType = "double";
  c7_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[97].fileTimeLo = 1289519692U;
  c7_info[97].fileTimeHi = 0U;
  c7_info[97].mFileTimeLo = 0U;
  c7_info[97].mFileTimeHi = 0U;
  c7_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c7_info[98].name = "eml_index_class";
  c7_info[98].dominantType = "";
  c7_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[98].fileTimeLo = 1323170578U;
  c7_info[98].fileTimeHi = 0U;
  c7_info[98].mFileTimeLo = 0U;
  c7_info[98].mFileTimeHi = 0U;
  c7_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c7_info[99].name = "eml_refblas_xger";
  c7_info[99].dominantType = "double";
  c7_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c7_info[99].fileTimeLo = 1299076776U;
  c7_info[99].fileTimeHi = 0U;
  c7_info[99].mFileTimeLo = 0U;
  c7_info[99].mFileTimeHi = 0U;
  c7_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c7_info[100].name = "eml_refblas_xgerx";
  c7_info[100].dominantType = "char";
  c7_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c7_info[100].fileTimeLo = 1299076778U;
  c7_info[100].fileTimeHi = 0U;
  c7_info[100].mFileTimeLo = 0U;
  c7_info[100].mFileTimeHi = 0U;
  c7_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c7_info[101].name = "eml_index_class";
  c7_info[101].dominantType = "";
  c7_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[101].fileTimeLo = 1323170578U;
  c7_info[101].fileTimeHi = 0U;
  c7_info[101].mFileTimeLo = 0U;
  c7_info[101].mFileTimeHi = 0U;
  c7_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c7_info[102].name = "abs";
  c7_info[102].dominantType = "coder.internal.indexInt";
  c7_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c7_info[102].fileTimeLo = 1343830366U;
  c7_info[102].fileTimeHi = 0U;
  c7_info[102].mFileTimeLo = 0U;
  c7_info[102].mFileTimeHi = 0U;
  c7_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c7_info[103].name = "eml_index_minus";
  c7_info[103].dominantType = "double";
  c7_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c7_info[103].fileTimeLo = 1286818778U;
  c7_info[103].fileTimeHi = 0U;
  c7_info[103].mFileTimeLo = 0U;
  c7_info[103].mFileTimeHi = 0U;
  c7_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c7_info[104].name = "eml_int_forloop_overflow_check";
  c7_info[104].dominantType = "";
  c7_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[104].fileTimeLo = 1346510340U;
  c7_info[104].fileTimeHi = 0U;
  c7_info[104].mFileTimeLo = 0U;
  c7_info[104].mFileTimeHi = 0U;
  c7_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c7_info[105].name = "eml_index_plus";
  c7_info[105].dominantType = "double";
  c7_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[105].fileTimeLo = 1286818778U;
  c7_info[105].fileTimeHi = 0U;
  c7_info[105].mFileTimeLo = 0U;
  c7_info[105].mFileTimeHi = 0U;
  c7_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c7_info[106].name = "eml_index_plus";
  c7_info[106].dominantType = "coder.internal.indexInt";
  c7_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[106].fileTimeLo = 1286818778U;
  c7_info[106].fileTimeHi = 0U;
  c7_info[106].mFileTimeLo = 0U;
  c7_info[106].mFileTimeHi = 0U;
  c7_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c7_info[107].name = "eml_warning";
  c7_info[107].dominantType = "char";
  c7_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c7_info[107].fileTimeLo = 1286818802U;
  c7_info[107].fileTimeHi = 0U;
  c7_info[107].mFileTimeLo = 0U;
  c7_info[107].mFileTimeHi = 0U;
  c7_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c7_info[108].name = "eml_scalar_eg";
  c7_info[108].dominantType = "double";
  c7_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[108].fileTimeLo = 1286818796U;
  c7_info[108].fileTimeHi = 0U;
  c7_info[108].mFileTimeLo = 0U;
  c7_info[108].mFileTimeHi = 0U;
  c7_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c7_info[109].name = "eml_int_forloop_overflow_check";
  c7_info[109].dominantType = "";
  c7_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[109].fileTimeLo = 1346510340U;
  c7_info[109].fileTimeHi = 0U;
  c7_info[109].mFileTimeLo = 0U;
  c7_info[109].mFileTimeHi = 0U;
  c7_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c7_info[110].name = "eml_xtrsm";
  c7_info[110].dominantType = "char";
  c7_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c7_info[110].fileTimeLo = 1299076778U;
  c7_info[110].fileTimeHi = 0U;
  c7_info[110].mFileTimeLo = 0U;
  c7_info[110].mFileTimeHi = 0U;
  c7_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c7_info[111].name = "eml_blas_inline";
  c7_info[111].dominantType = "";
  c7_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c7_info[111].fileTimeLo = 1299076768U;
  c7_info[111].fileTimeHi = 0U;
  c7_info[111].mFileTimeLo = 0U;
  c7_info[111].mFileTimeHi = 0U;
  c7_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c7_info[112].name = "mtimes";
  c7_info[112].dominantType = "double";
  c7_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[112].fileTimeLo = 1289519692U;
  c7_info[112].fileTimeHi = 0U;
  c7_info[112].mFileTimeLo = 0U;
  c7_info[112].mFileTimeHi = 0U;
  c7_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c7_info[113].name = "eml_index_class";
  c7_info[113].dominantType = "";
  c7_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[113].fileTimeLo = 1323170578U;
  c7_info[113].fileTimeHi = 0U;
  c7_info[113].mFileTimeLo = 0U;
  c7_info[113].mFileTimeHi = 0U;
  c7_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c7_info[114].name = "eml_scalar_eg";
  c7_info[114].dominantType = "double";
  c7_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[114].fileTimeLo = 1286818796U;
  c7_info[114].fileTimeHi = 0U;
  c7_info[114].mFileTimeLo = 0U;
  c7_info[114].mFileTimeHi = 0U;
  c7_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c7_info[115].name = "eml_refblas_xtrsm";
  c7_info[115].dominantType = "char";
  c7_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[115].fileTimeLo = 1299076786U;
  c7_info[115].fileTimeHi = 0U;
  c7_info[115].mFileTimeLo = 0U;
  c7_info[115].mFileTimeHi = 0U;
  c7_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[116].name = "eml_scalar_eg";
  c7_info[116].dominantType = "double";
  c7_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[116].fileTimeLo = 1286818796U;
  c7_info[116].fileTimeHi = 0U;
  c7_info[116].mFileTimeLo = 0U;
  c7_info[116].mFileTimeHi = 0U;
  c7_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[117].name = "eml_index_minus";
  c7_info[117].dominantType = "double";
  c7_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c7_info[117].fileTimeLo = 1286818778U;
  c7_info[117].fileTimeHi = 0U;
  c7_info[117].mFileTimeLo = 0U;
  c7_info[117].mFileTimeHi = 0U;
  c7_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[118].name = "eml_index_class";
  c7_info[118].dominantType = "";
  c7_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[118].fileTimeLo = 1323170578U;
  c7_info[118].fileTimeHi = 0U;
  c7_info[118].mFileTimeLo = 0U;
  c7_info[118].mFileTimeHi = 0U;
  c7_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[119].name = "eml_int_forloop_overflow_check";
  c7_info[119].dominantType = "";
  c7_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[119].fileTimeLo = 1346510340U;
  c7_info[119].fileTimeHi = 0U;
  c7_info[119].mFileTimeLo = 0U;
  c7_info[119].mFileTimeHi = 0U;
  c7_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[120].name = "eml_index_times";
  c7_info[120].dominantType = "coder.internal.indexInt";
  c7_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c7_info[120].fileTimeLo = 1286818780U;
  c7_info[120].fileTimeHi = 0U;
  c7_info[120].mFileTimeLo = 0U;
  c7_info[120].mFileTimeHi = 0U;
  c7_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[121].name = "eml_index_plus";
  c7_info[121].dominantType = "coder.internal.indexInt";
  c7_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[121].fileTimeLo = 1286818778U;
  c7_info[121].fileTimeHi = 0U;
  c7_info[121].mFileTimeLo = 0U;
  c7_info[121].mFileTimeHi = 0U;
  c7_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[122].name = "eml_index_plus";
  c7_info[122].dominantType = "double";
  c7_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[122].fileTimeLo = 1286818778U;
  c7_info[122].fileTimeHi = 0U;
  c7_info[122].mFileTimeLo = 0U;
  c7_info[122].mFileTimeHi = 0U;
  c7_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c7_info[123].name = "intmin";
  c7_info[123].dominantType = "char";
  c7_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c7_info[123].fileTimeLo = 1311255318U;
  c7_info[123].fileTimeHi = 0U;
  c7_info[123].mFileTimeLo = 0U;
  c7_info[123].mFileTimeHi = 0U;
  c7_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c7_info[124].name = "eml_div";
  c7_info[124].dominantType = "double";
  c7_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c7_info[124].fileTimeLo = 1313347810U;
  c7_info[124].fileTimeHi = 0U;
  c7_info[124].mFileTimeLo = 0U;
  c7_info[124].mFileTimeHi = 0U;
}

static void c7_eml_scalar_eg(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c7_mldivide(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c7_A[36], real_T c7_B[72], real_T c7_Y[72])
{
  int32_T c7_i96;
  real_T c7_b_A[36];
  int32_T c7_info;
  int32_T c7_ipiv[6];
  int32_T c7_b_info;
  int32_T c7_c_info;
  int32_T c7_d_info;
  int32_T c7_i97;
  int32_T c7_i;
  int32_T c7_b_i;
  int32_T c7_ip;
  int32_T c7_j;
  int32_T c7_b_j;
  real_T c7_temp;
  int32_T c7_i98;
  real_T c7_c_A[36];
  int32_T c7_i99;
  real_T c7_d_A[36];
  for (c7_i96 = 0; c7_i96 < 36; c7_i96++) {
    c7_b_A[c7_i96] = c7_A[c7_i96];
  }

  c7_b_eml_matlab_zgetrf(chartInstance, c7_b_A, c7_ipiv, &c7_info);
  c7_b_info = c7_info;
  c7_c_info = c7_b_info;
  c7_d_info = c7_c_info;
  if (c7_d_info > 0) {
    c7_eml_warning(chartInstance);
  }

  for (c7_i97 = 0; c7_i97 < 72; c7_i97++) {
    c7_Y[c7_i97] = c7_B[c7_i97];
  }

  for (c7_i = 1; c7_i < 7; c7_i++) {
    c7_b_i = c7_i;
    if (c7_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c7_b_i), 1, 6, 1, 0) - 1] != c7_b_i) {
      c7_ip = c7_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c7_b_i), 1, 6, 1, 0) - 1];
      for (c7_j = 1; c7_j < 13; c7_j++) {
        c7_b_j = c7_j;
        c7_temp = c7_Y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c7_b_i), 1, 6, 1, 0) + 6 *
                        (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c7_b_j), 1, 12, 2, 0) - 1)) - 1];
        c7_Y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c7_b_i), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c7_b_j), 1, 12, 2, 0)
               - 1)) - 1] = c7_Y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c7_ip), 1, 6, 1, 0) + 6 *
          (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c7_b_j), 1, 12, 2, 0) - 1)) - 1];
        c7_Y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c7_ip), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c7_b_j), 1, 12, 2, 0)
               - 1)) - 1] = c7_temp;
      }
    }
  }

  for (c7_i98 = 0; c7_i98 < 36; c7_i98++) {
    c7_c_A[c7_i98] = c7_b_A[c7_i98];
  }

  c7_c_eml_xtrsm(chartInstance, c7_c_A, c7_Y);
  for (c7_i99 = 0; c7_i99 < 36; c7_i99++) {
    c7_d_A[c7_i99] = c7_b_A[c7_i99];
  }

  c7_d_eml_xtrsm(chartInstance, c7_d_A, c7_Y);
}

static void c7_realmin(SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c7_eps(SFc7_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c7_eml_matlab_zgetrf(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], real_T c7_b_A[36], int32_T c7_ipiv[6],
  int32_T *c7_info)
{
  int32_T c7_i100;
  for (c7_i100 = 0; c7_i100 < 36; c7_i100++) {
    c7_b_A[c7_i100] = c7_A[c7_i100];
  }

  c7_b_eml_matlab_zgetrf(chartInstance, c7_b_A, c7_ipiv, c7_info);
}

static void c7_check_forloop_overflow_error
  (SFc7_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c7_overflow)
{
  int32_T c7_i101;
  static char_T c7_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c7_u[34];
  const mxArray *c7_y = NULL;
  int32_T c7_i102;
  static char_T c7_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c7_b_u[23];
  const mxArray *c7_b_y = NULL;
  if (!c7_overflow) {
  } else {
    for (c7_i101 = 0; c7_i101 < 34; c7_i101++) {
      c7_u[c7_i101] = c7_cv0[c7_i101];
    }

    c7_y = NULL;
    sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c7_i102 = 0; c7_i102 < 23; c7_i102++) {
      c7_b_u[c7_i102] = c7_cv1[c7_i102];
    }

    c7_b_y = NULL;
    sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c7_y, 14, c7_b_y));
  }
}

static void c7_eml_xger(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_alpha1, int32_T c7_ix0, int32_T c7_iy0,
  real_T c7_A[36], int32_T c7_ia0, real_T c7_b_A[36])
{
  int32_T c7_i103;
  for (c7_i103 = 0; c7_i103 < 36; c7_i103++) {
    c7_b_A[c7_i103] = c7_A[c7_i103];
  }

  c7_b_eml_xger(chartInstance, c7_m, c7_n, c7_alpha1, c7_ix0, c7_iy0, c7_b_A,
                c7_ia0);
}

static void c7_eml_warning(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c7_i104;
  static char_T c7_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c7_u[27];
  const mxArray *c7_y = NULL;
  for (c7_i104 = 0; c7_i104 < 27; c7_i104++) {
    c7_u[c7_i104] = c7_varargin_1[c7_i104];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c7_y));
}

static void c7_eml_xtrsm(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c7_A[36], real_T c7_B[72], real_T c7_b_B[72])
{
  int32_T c7_i105;
  int32_T c7_i106;
  real_T c7_b_A[36];
  for (c7_i105 = 0; c7_i105 < 72; c7_i105++) {
    c7_b_B[c7_i105] = c7_B[c7_i105];
  }

  for (c7_i106 = 0; c7_i106 < 36; c7_i106++) {
    c7_b_A[c7_i106] = c7_A[c7_i106];
  }

  c7_c_eml_xtrsm(chartInstance, c7_b_A, c7_b_B);
}

static void c7_below_threshold(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c7_b_eml_scalar_eg(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c7_b_eml_xtrsm(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], real_T c7_B[72], real_T c7_b_B[72])
{
  int32_T c7_i107;
  int32_T c7_i108;
  real_T c7_b_A[36];
  for (c7_i107 = 0; c7_i107 < 72; c7_i107++) {
    c7_b_B[c7_i107] = c7_B[c7_i107];
  }

  for (c7_i108 = 0; c7_i108 < 36; c7_i108++) {
    c7_b_A[c7_i108] = c7_A[c7_i108];
  }

  c7_d_eml_xtrsm(chartInstance, c7_b_A, c7_b_B);
}

static void c7_c_eml_scalar_eg(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c7_eml_xgemm(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c7_A[72], real_T c7_B[276], real_T c7_C[138], real_T c7_b_C[138])
{
  int32_T c7_i109;
  int32_T c7_i110;
  real_T c7_b_A[72];
  int32_T c7_i111;
  real_T c7_b_B[276];
  for (c7_i109 = 0; c7_i109 < 138; c7_i109++) {
    c7_b_C[c7_i109] = c7_C[c7_i109];
  }

  for (c7_i110 = 0; c7_i110 < 72; c7_i110++) {
    c7_b_A[c7_i110] = c7_A[c7_i110];
  }

  for (c7_i111 = 0; c7_i111 < 276; c7_i111++) {
    c7_b_B[c7_i111] = c7_B[c7_i111];
  }

  c7_b_eml_xgemm(chartInstance, c7_b_A, c7_b_B, c7_b_C);
}

static void c7_d_eml_scalar_eg(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static const mxArray *c7_i_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(int32_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static int32_T c7_g_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  int32_T c7_y;
  int32_T c7_i112;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_i112, 1, 6, 0U, 0, 0U, 0);
  c7_y = c7_i112;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_sfEvent;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  int32_T c7_y;
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c7_b_sfEvent = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_sfEvent),
    &c7_thisId);
  sf_mex_destroy(&c7_b_sfEvent);
  *(int32_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static uint8_T c7_h_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_b_is_active_c7_torqueBalancing2012b, const
  char_T *c7_identifier)
{
  uint8_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c7_b_is_active_c7_torqueBalancing2012b), &c7_thisId);
  sf_mex_destroy(&c7_b_is_active_c7_torqueBalancing2012b);
  return c7_y;
}

static uint8_T c7_i_emlrt_marshallIn(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  uint8_T c7_y;
  uint8_T c7_u0;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_u0, 1, 3, 0U, 0, 0U, 0);
  c7_y = c7_u0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_b_eml_matlab_zgetrf(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], int32_T c7_ipiv[6], int32_T *c7_info)
{
  int32_T c7_i113;
  int32_T c7_j;
  int32_T c7_b_j;
  int32_T c7_a;
  int32_T c7_jm1;
  int32_T c7_b;
  int32_T c7_mmj;
  int32_T c7_b_a;
  int32_T c7_c;
  int32_T c7_b_b;
  int32_T c7_jj;
  int32_T c7_c_a;
  int32_T c7_jp1j;
  int32_T c7_d_a;
  int32_T c7_b_c;
  int32_T c7_n;
  int32_T c7_ix0;
  int32_T c7_b_n;
  int32_T c7_b_ix0;
  int32_T c7_c_n;
  int32_T c7_c_ix0;
  int32_T c7_idxmax;
  int32_T c7_ix;
  real_T c7_x;
  real_T c7_b_x;
  real_T c7_c_x;
  real_T c7_y;
  real_T c7_d_x;
  real_T c7_e_x;
  real_T c7_b_y;
  real_T c7_smax;
  int32_T c7_d_n;
  int32_T c7_c_b;
  int32_T c7_d_b;
  boolean_T c7_overflow;
  int32_T c7_k;
  int32_T c7_b_k;
  int32_T c7_e_a;
  real_T c7_f_x;
  real_T c7_g_x;
  real_T c7_h_x;
  real_T c7_c_y;
  real_T c7_i_x;
  real_T c7_j_x;
  real_T c7_d_y;
  real_T c7_s;
  int32_T c7_f_a;
  int32_T c7_jpiv_offset;
  int32_T c7_g_a;
  int32_T c7_e_b;
  int32_T c7_jpiv;
  int32_T c7_h_a;
  int32_T c7_f_b;
  int32_T c7_c_c;
  int32_T c7_g_b;
  int32_T c7_jrow;
  int32_T c7_i_a;
  int32_T c7_h_b;
  int32_T c7_jprow;
  int32_T c7_d_ix0;
  int32_T c7_iy0;
  int32_T c7_e_ix0;
  int32_T c7_b_iy0;
  int32_T c7_f_ix0;
  int32_T c7_c_iy0;
  int32_T c7_b_ix;
  int32_T c7_iy;
  int32_T c7_c_k;
  real_T c7_temp;
  int32_T c7_j_a;
  int32_T c7_k_a;
  int32_T c7_b_jp1j;
  int32_T c7_l_a;
  int32_T c7_d_c;
  int32_T c7_m_a;
  int32_T c7_i_b;
  int32_T c7_i114;
  int32_T c7_n_a;
  int32_T c7_j_b;
  int32_T c7_o_a;
  int32_T c7_k_b;
  boolean_T c7_b_overflow;
  int32_T c7_i;
  int32_T c7_b_i;
  real_T c7_k_x;
  real_T c7_e_y;
  real_T c7_z;
  int32_T c7_l_b;
  int32_T c7_e_c;
  int32_T c7_p_a;
  int32_T c7_f_c;
  int32_T c7_q_a;
  int32_T c7_g_c;
  int32_T c7_m;
  int32_T c7_e_n;
  int32_T c7_g_ix0;
  int32_T c7_d_iy0;
  int32_T c7_ia0;
  real_T c7_d1;
  c7_realmin(chartInstance);
  c7_eps(chartInstance);
  for (c7_i113 = 0; c7_i113 < 6; c7_i113++) {
    c7_ipiv[c7_i113] = 1 + c7_i113;
  }

  *c7_info = 0;
  for (c7_j = 1; c7_j < 6; c7_j++) {
    c7_b_j = c7_j;
    c7_a = c7_b_j - 1;
    c7_jm1 = c7_a;
    c7_b = c7_b_j;
    c7_mmj = 6 - c7_b;
    c7_b_a = c7_jm1;
    c7_c = c7_b_a * 7;
    c7_b_b = c7_c + 1;
    c7_jj = c7_b_b;
    c7_c_a = c7_jj + 1;
    c7_jp1j = c7_c_a;
    c7_d_a = c7_mmj;
    c7_b_c = c7_d_a;
    c7_n = c7_b_c + 1;
    c7_ix0 = c7_jj;
    c7_b_n = c7_n;
    c7_b_ix0 = c7_ix0;
    c7_c_n = c7_b_n;
    c7_c_ix0 = c7_b_ix0;
    if (c7_c_n < 1) {
      c7_idxmax = 0;
    } else {
      c7_idxmax = 1;
      if (c7_c_n > 1) {
        c7_ix = c7_c_ix0;
        c7_x = c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c7_ix), 1, 36, 1, 0) - 1];
        c7_b_x = c7_x;
        c7_c_x = c7_b_x;
        c7_y = muDoubleScalarAbs(c7_c_x);
        c7_d_x = 0.0;
        c7_e_x = c7_d_x;
        c7_b_y = muDoubleScalarAbs(c7_e_x);
        c7_smax = c7_y + c7_b_y;
        c7_d_n = c7_c_n;
        c7_c_b = c7_d_n;
        c7_d_b = c7_c_b;
        if (2 > c7_d_b) {
          c7_overflow = FALSE;
        } else {
          c7_overflow = (c7_d_b > 2147483646);
        }

        if (c7_overflow) {
          c7_check_forloop_overflow_error(chartInstance, c7_overflow);
        }

        for (c7_k = 2; c7_k <= c7_d_n; c7_k++) {
          c7_b_k = c7_k;
          c7_e_a = c7_ix + 1;
          c7_ix = c7_e_a;
          c7_f_x = c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c7_ix), 1, 36, 1, 0) - 1];
          c7_g_x = c7_f_x;
          c7_h_x = c7_g_x;
          c7_c_y = muDoubleScalarAbs(c7_h_x);
          c7_i_x = 0.0;
          c7_j_x = c7_i_x;
          c7_d_y = muDoubleScalarAbs(c7_j_x);
          c7_s = c7_c_y + c7_d_y;
          if (c7_s > c7_smax) {
            c7_idxmax = c7_b_k;
            c7_smax = c7_s;
          }
        }
      }
    }

    c7_f_a = c7_idxmax - 1;
    c7_jpiv_offset = c7_f_a;
    c7_g_a = c7_jj;
    c7_e_b = c7_jpiv_offset;
    c7_jpiv = c7_g_a + c7_e_b;
    if (c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c7_jpiv), 1, 36, 1, 0) - 1] != 0.0) {
      if (c7_jpiv_offset != 0) {
        c7_h_a = c7_b_j;
        c7_f_b = c7_jpiv_offset;
        c7_c_c = c7_h_a + c7_f_b;
        c7_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c7_b_j), 1, 6, 1, 0) - 1] = c7_c_c;
        c7_g_b = c7_jm1 + 1;
        c7_jrow = c7_g_b;
        c7_i_a = c7_jrow;
        c7_h_b = c7_jpiv_offset;
        c7_jprow = c7_i_a + c7_h_b;
        c7_d_ix0 = c7_jrow;
        c7_iy0 = c7_jprow;
        c7_e_ix0 = c7_d_ix0;
        c7_b_iy0 = c7_iy0;
        c7_f_ix0 = c7_e_ix0;
        c7_c_iy0 = c7_b_iy0;
        c7_b_ix = c7_f_ix0;
        c7_iy = c7_c_iy0;
        for (c7_c_k = 1; c7_c_k < 7; c7_c_k++) {
          c7_temp = c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c7_b_ix), 1, 36, 1, 0) - 1];
          c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_b_ix), 1, 36, 1, 0) - 1] =
            c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_iy), 1, 36, 1, 0) - 1];
          c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_iy), 1, 36, 1, 0) - 1] = c7_temp;
          c7_j_a = c7_b_ix + 6;
          c7_b_ix = c7_j_a;
          c7_k_a = c7_iy + 6;
          c7_iy = c7_k_a;
        }
      }

      c7_b_jp1j = c7_jp1j;
      c7_l_a = c7_mmj;
      c7_d_c = c7_l_a;
      c7_m_a = c7_jp1j;
      c7_i_b = c7_d_c - 1;
      c7_i114 = c7_m_a + c7_i_b;
      c7_n_a = c7_b_jp1j;
      c7_j_b = c7_i114;
      c7_o_a = c7_n_a;
      c7_k_b = c7_j_b;
      if (c7_o_a > c7_k_b) {
        c7_b_overflow = FALSE;
      } else {
        c7_b_overflow = (c7_k_b > 2147483646);
      }

      if (c7_b_overflow) {
        c7_check_forloop_overflow_error(chartInstance, c7_b_overflow);
      }

      for (c7_i = c7_b_jp1j; c7_i <= c7_i114; c7_i++) {
        c7_b_i = c7_i;
        c7_k_x = c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c7_b_i), 1, 36, 1, 0) - 1];
        c7_e_y = c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c7_jj), 1, 36, 1, 0) - 1];
        c7_z = c7_k_x / c7_e_y;
        c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c7_b_i), 1, 36, 1, 0) - 1] = c7_z;
      }
    } else {
      *c7_info = c7_b_j;
    }

    c7_l_b = c7_b_j;
    c7_e_c = 6 - c7_l_b;
    c7_p_a = c7_jj;
    c7_f_c = c7_p_a;
    c7_q_a = c7_jj;
    c7_g_c = c7_q_a;
    c7_m = c7_mmj;
    c7_e_n = c7_e_c;
    c7_g_ix0 = c7_jp1j;
    c7_d_iy0 = c7_f_c + 6;
    c7_ia0 = c7_g_c + 7;
    c7_d1 = -1.0;
    c7_b_eml_xger(chartInstance, c7_m, c7_e_n, c7_d1, c7_g_ix0, c7_d_iy0, c7_A,
                  c7_ia0);
  }

  if (*c7_info == 0) {
    if (!(c7_A[35] != 0.0)) {
      *c7_info = 6;
    }
  }
}

static void c7_b_eml_xger(SFc7_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_alpha1, int32_T c7_ix0, int32_T c7_iy0,
  real_T c7_A[36], int32_T c7_ia0)
{
  int32_T c7_b_m;
  int32_T c7_b_n;
  real_T c7_b_alpha1;
  int32_T c7_b_ix0;
  int32_T c7_b_iy0;
  int32_T c7_b_ia0;
  int32_T c7_c_m;
  int32_T c7_c_n;
  real_T c7_c_alpha1;
  int32_T c7_c_ix0;
  int32_T c7_c_iy0;
  int32_T c7_c_ia0;
  int32_T c7_d_m;
  int32_T c7_d_n;
  real_T c7_d_alpha1;
  int32_T c7_d_ix0;
  int32_T c7_d_iy0;
  int32_T c7_d_ia0;
  int32_T c7_ixstart;
  int32_T c7_a;
  int32_T c7_jA;
  int32_T c7_jy;
  int32_T c7_e_n;
  int32_T c7_b;
  int32_T c7_b_b;
  boolean_T c7_overflow;
  int32_T c7_j;
  real_T c7_yjy;
  real_T c7_temp;
  int32_T c7_ix;
  int32_T c7_c_b;
  int32_T c7_i115;
  int32_T c7_b_a;
  int32_T c7_d_b;
  int32_T c7_i116;
  int32_T c7_c_a;
  int32_T c7_e_b;
  int32_T c7_d_a;
  int32_T c7_f_b;
  boolean_T c7_b_overflow;
  int32_T c7_ijA;
  int32_T c7_b_ijA;
  int32_T c7_e_a;
  int32_T c7_f_a;
  int32_T c7_g_a;
  c7_b_m = c7_m;
  c7_b_n = c7_n;
  c7_b_alpha1 = c7_alpha1;
  c7_b_ix0 = c7_ix0;
  c7_b_iy0 = c7_iy0;
  c7_b_ia0 = c7_ia0;
  c7_c_m = c7_b_m;
  c7_c_n = c7_b_n;
  c7_c_alpha1 = c7_b_alpha1;
  c7_c_ix0 = c7_b_ix0;
  c7_c_iy0 = c7_b_iy0;
  c7_c_ia0 = c7_b_ia0;
  c7_d_m = c7_c_m;
  c7_d_n = c7_c_n;
  c7_d_alpha1 = c7_c_alpha1;
  c7_d_ix0 = c7_c_ix0;
  c7_d_iy0 = c7_c_iy0;
  c7_d_ia0 = c7_c_ia0;
  if (c7_d_alpha1 == 0.0) {
  } else {
    c7_ixstart = c7_d_ix0;
    c7_a = c7_d_ia0 - 1;
    c7_jA = c7_a;
    c7_jy = c7_d_iy0;
    c7_e_n = c7_d_n;
    c7_b = c7_e_n;
    c7_b_b = c7_b;
    if (1 > c7_b_b) {
      c7_overflow = FALSE;
    } else {
      c7_overflow = (c7_b_b > 2147483646);
    }

    if (c7_overflow) {
      c7_check_forloop_overflow_error(chartInstance, c7_overflow);
    }

    for (c7_j = 1; c7_j <= c7_e_n; c7_j++) {
      c7_yjy = c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c7_jy), 1, 36, 1, 0) - 1];
      if (c7_yjy != 0.0) {
        c7_temp = c7_yjy * c7_d_alpha1;
        c7_ix = c7_ixstart;
        c7_c_b = c7_jA + 1;
        c7_i115 = c7_c_b;
        c7_b_a = c7_d_m;
        c7_d_b = c7_jA;
        c7_i116 = c7_b_a + c7_d_b;
        c7_c_a = c7_i115;
        c7_e_b = c7_i116;
        c7_d_a = c7_c_a;
        c7_f_b = c7_e_b;
        if (c7_d_a > c7_f_b) {
          c7_b_overflow = FALSE;
        } else {
          c7_b_overflow = (c7_f_b > 2147483646);
        }

        if (c7_b_overflow) {
          c7_check_forloop_overflow_error(chartInstance, c7_b_overflow);
        }

        for (c7_ijA = c7_i115; c7_ijA <= c7_i116; c7_ijA++) {
          c7_b_ijA = c7_ijA;
          c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_b_ijA), 1, 36, 1, 0) - 1] =
            c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_b_ijA), 1, 36, 1, 0) - 1] +
            c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_ix), 1, 36, 1, 0) - 1] * c7_temp;
          c7_e_a = c7_ix + 1;
          c7_ix = c7_e_a;
        }
      }

      c7_f_a = c7_jy + 6;
      c7_jy = c7_f_a;
      c7_g_a = c7_jA + 6;
      c7_jA = c7_g_a;
    }
  }
}

static void c7_c_eml_xtrsm(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], real_T c7_B[72])
{
  int32_T c7_j;
  int32_T c7_b_j;
  int32_T c7_a;
  int32_T c7_c;
  int32_T c7_b;
  int32_T c7_b_c;
  int32_T c7_b_b;
  int32_T c7_jBcol;
  int32_T c7_k;
  int32_T c7_b_k;
  int32_T c7_b_a;
  int32_T c7_c_c;
  int32_T c7_c_b;
  int32_T c7_d_c;
  int32_T c7_d_b;
  int32_T c7_kAcol;
  int32_T c7_c_a;
  int32_T c7_e_b;
  int32_T c7_e_c;
  int32_T c7_d_a;
  int32_T c7_i117;
  boolean_T c7_overflow;
  int32_T c7_i;
  int32_T c7_b_i;
  int32_T c7_e_a;
  int32_T c7_f_b;
  int32_T c7_f_c;
  int32_T c7_f_a;
  int32_T c7_g_b;
  int32_T c7_g_c;
  int32_T c7_g_a;
  int32_T c7_h_b;
  int32_T c7_h_c;
  int32_T c7_h_a;
  int32_T c7_i_b;
  int32_T c7_i_c;
  c7_below_threshold(chartInstance);
  c7_b_eml_scalar_eg(chartInstance);
  for (c7_j = 1; c7_j < 13; c7_j++) {
    c7_b_j = c7_j;
    c7_a = c7_b_j;
    c7_c = c7_a;
    c7_b = c7_c - 1;
    c7_b_c = 6 * c7_b;
    c7_b_b = c7_b_c;
    c7_jBcol = c7_b_b;
    for (c7_k = 1; c7_k < 7; c7_k++) {
      c7_b_k = c7_k;
      c7_b_a = c7_b_k;
      c7_c_c = c7_b_a;
      c7_c_b = c7_c_c - 1;
      c7_d_c = 6 * c7_c_b;
      c7_d_b = c7_d_c;
      c7_kAcol = c7_d_b;
      c7_c_a = c7_b_k;
      c7_e_b = c7_jBcol;
      c7_e_c = c7_c_a + c7_e_b;
      if (c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_e_c), 1, 72, 1, 0) - 1] != 0.0) {
        c7_d_a = c7_b_k;
        c7_i117 = c7_d_a;
        c7_overflow = FALSE;
        if (c7_overflow) {
          c7_check_forloop_overflow_error(chartInstance, c7_overflow);
        }

        for (c7_i = c7_i117 + 1; c7_i < 7; c7_i++) {
          c7_b_i = c7_i;
          c7_e_a = c7_b_i;
          c7_f_b = c7_jBcol;
          c7_f_c = c7_e_a + c7_f_b;
          c7_f_a = c7_b_i;
          c7_g_b = c7_jBcol;
          c7_g_c = c7_f_a + c7_g_b;
          c7_g_a = c7_b_k;
          c7_h_b = c7_jBcol;
          c7_h_c = c7_g_a + c7_h_b;
          c7_h_a = c7_b_i;
          c7_i_b = c7_kAcol;
          c7_i_c = c7_h_a + c7_i_b;
          c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_f_c), 1, 72, 1, 0) - 1] =
            c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_g_c), 1, 72, 1, 0) - 1] -
            c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_h_c), 1, 72, 1, 0) - 1] *
            c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_i_c), 1, 36, 1, 0) - 1];
        }
      }
    }
  }
}

static void c7_d_eml_xtrsm(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[36], real_T c7_B[72])
{
  int32_T c7_j;
  int32_T c7_b_j;
  int32_T c7_a;
  int32_T c7_c;
  int32_T c7_b;
  int32_T c7_b_c;
  int32_T c7_b_b;
  int32_T c7_jBcol;
  int32_T c7_k;
  int32_T c7_b_k;
  int32_T c7_b_a;
  int32_T c7_c_c;
  int32_T c7_c_b;
  int32_T c7_d_c;
  int32_T c7_d_b;
  int32_T c7_kAcol;
  int32_T c7_c_a;
  int32_T c7_e_b;
  int32_T c7_e_c;
  int32_T c7_d_a;
  int32_T c7_f_b;
  int32_T c7_f_c;
  int32_T c7_e_a;
  int32_T c7_g_b;
  int32_T c7_g_c;
  int32_T c7_f_a;
  int32_T c7_h_b;
  int32_T c7_h_c;
  real_T c7_x;
  real_T c7_y;
  real_T c7_z;
  int32_T c7_g_a;
  int32_T c7_i118;
  int32_T c7_i_b;
  int32_T c7_j_b;
  boolean_T c7_overflow;
  int32_T c7_i;
  int32_T c7_b_i;
  int32_T c7_h_a;
  int32_T c7_k_b;
  int32_T c7_i_c;
  int32_T c7_i_a;
  int32_T c7_l_b;
  int32_T c7_j_c;
  int32_T c7_j_a;
  int32_T c7_m_b;
  int32_T c7_k_c;
  int32_T c7_k_a;
  int32_T c7_n_b;
  int32_T c7_l_c;
  c7_below_threshold(chartInstance);
  c7_b_eml_scalar_eg(chartInstance);
  for (c7_j = 1; c7_j < 13; c7_j++) {
    c7_b_j = c7_j;
    c7_a = c7_b_j;
    c7_c = c7_a;
    c7_b = c7_c - 1;
    c7_b_c = 6 * c7_b;
    c7_b_b = c7_b_c;
    c7_jBcol = c7_b_b;
    for (c7_k = 6; c7_k > 0; c7_k--) {
      c7_b_k = c7_k;
      c7_b_a = c7_b_k;
      c7_c_c = c7_b_a;
      c7_c_b = c7_c_c - 1;
      c7_d_c = 6 * c7_c_b;
      c7_d_b = c7_d_c;
      c7_kAcol = c7_d_b;
      c7_c_a = c7_b_k;
      c7_e_b = c7_jBcol;
      c7_e_c = c7_c_a + c7_e_b;
      if (c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_e_c), 1, 72, 1, 0) - 1] != 0.0) {
        c7_d_a = c7_b_k;
        c7_f_b = c7_jBcol;
        c7_f_c = c7_d_a + c7_f_b;
        c7_e_a = c7_b_k;
        c7_g_b = c7_jBcol;
        c7_g_c = c7_e_a + c7_g_b;
        c7_f_a = c7_b_k;
        c7_h_b = c7_kAcol;
        c7_h_c = c7_f_a + c7_h_b;
        c7_x = c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c7_g_c), 1, 72, 1, 0) - 1];
        c7_y = c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c7_h_c), 1, 36, 1, 0) - 1];
        c7_z = c7_x / c7_y;
        c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c7_f_c), 1, 72, 1, 0) - 1] = c7_z;
        c7_g_a = c7_b_k - 1;
        c7_i118 = c7_g_a;
        c7_i_b = c7_i118;
        c7_j_b = c7_i_b;
        if (1 > c7_j_b) {
          c7_overflow = FALSE;
        } else {
          c7_overflow = (c7_j_b > 2147483646);
        }

        if (c7_overflow) {
          c7_check_forloop_overflow_error(chartInstance, c7_overflow);
        }

        for (c7_i = 1; c7_i <= c7_i118; c7_i++) {
          c7_b_i = c7_i;
          c7_h_a = c7_b_i;
          c7_k_b = c7_jBcol;
          c7_i_c = c7_h_a + c7_k_b;
          c7_i_a = c7_b_i;
          c7_l_b = c7_jBcol;
          c7_j_c = c7_i_a + c7_l_b;
          c7_j_a = c7_b_k;
          c7_m_b = c7_jBcol;
          c7_k_c = c7_j_a + c7_m_b;
          c7_k_a = c7_b_i;
          c7_n_b = c7_kAcol;
          c7_l_c = c7_k_a + c7_n_b;
          c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_i_c), 1, 72, 1, 0) - 1] =
            c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_j_c), 1, 72, 1, 0) - 1] -
            c7_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_k_c), 1, 72, 1, 0) - 1] *
            c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_l_c), 1, 36, 1, 0) - 1];
        }
      }
    }
  }
}

static void c7_b_eml_xgemm(SFc7_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c7_A[72], real_T c7_B[276], real_T c7_C[138])
{
  real_T c7_alpha1;
  real_T c7_beta1;
  char_T c7_TRANSB;
  char_T c7_TRANSA;
  ptrdiff_t c7_m_t;
  ptrdiff_t c7_n_t;
  ptrdiff_t c7_k_t;
  ptrdiff_t c7_lda_t;
  ptrdiff_t c7_ldb_t;
  ptrdiff_t c7_ldc_t;
  double * c7_alpha1_t;
  double * c7_Aia0_t;
  double * c7_Bib0_t;
  double * c7_beta1_t;
  double * c7_Cic0_t;
  c7_alpha1 = 1.0;
  c7_beta1 = 0.0;
  c7_TRANSB = 'N';
  c7_TRANSA = 'N';
  c7_m_t = (ptrdiff_t)(6);
  c7_n_t = (ptrdiff_t)(23);
  c7_k_t = (ptrdiff_t)(12);
  c7_lda_t = (ptrdiff_t)(6);
  c7_ldb_t = (ptrdiff_t)(12);
  c7_ldc_t = (ptrdiff_t)(6);
  c7_alpha1_t = (double *)(&c7_alpha1);
  c7_Aia0_t = (double *)(&c7_A[0]);
  c7_Bib0_t = (double *)(&c7_B[0]);
  c7_beta1_t = (double *)(&c7_beta1);
  c7_Cic0_t = (double *)(&c7_C[0]);
  dgemm(&c7_TRANSA, &c7_TRANSB, &c7_m_t, &c7_n_t, &c7_k_t, c7_alpha1_t,
        c7_Aia0_t, &c7_lda_t, c7_Bib0_t, &c7_ldb_t, c7_beta1_t, c7_Cic0_t,
        &c7_ldc_t);
}

static void init_dsm_address_info(SFc7_torqueBalancing2012bInstanceStruct
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

void sf_c7_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2609311878U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1939696709U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1409159861U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1516891732U);
}

mxArray *sf_c7_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("aGfeCGcwK6wHUSURFtHUY");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(29);
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
      pr[0] = (double)(23);
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

mxArray *sf_c7_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c7_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c7_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c7_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           7,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"JcLeftFoot");
          _SFD_SET_DATA_PROPS(1,1,1,0,"JcRightFoot");
          _SFD_SET_DATA_PROPS(2,1,1,0,"activeFeetConstraints");
          _SFD_SET_DATA_PROPS(3,2,0,1,"y");
          _SFD_SET_DATA_PROPS(4,1,1,0,"qD");
          _SFD_SET_DATA_PROPS(5,10,0,0,"reg");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,300);
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
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)
            c7_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_b_sf_marshallOut,(MexInFcnForType)c7_b_sf_marshallIn);

        {
          real_T (*c7_JcLeftFoot)[174];
          real_T (*c7_JcRightFoot)[174];
          real_T (*c7_activeFeetConstraints)[2];
          real_T (*c7_y)[6];
          real_T (*c7_qD)[23];
          c7_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
          c7_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c7_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
            (chartInstance->S, 2);
          c7_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal
            (chartInstance->S, 1);
          c7_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c7_JcLeftFoot);
          _SFD_SET_DATA_VALUE_PTR(1U, *c7_JcRightFoot);
          _SFD_SET_DATA_VALUE_PTR(2U, *c7_activeFeetConstraints);
          _SFD_SET_DATA_VALUE_PTR(3U, *c7_y);
          _SFD_SET_DATA_VALUE_PTR(4U, *c7_qD);
          _SFD_SET_DATA_VALUE_PTR(5U, &chartInstance->c7_reg);
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
  return "q9hz9H0nLOBpguDFDiv7VE";
}

static void sf_opaque_initialize_c7_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc7_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c7_torqueBalancing2012b
    ((SFc7_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c7_torqueBalancing2012b((SFc7_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c7_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c7_torqueBalancing2012b((SFc7_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c7_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c7_torqueBalancing2012b((SFc7_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c7_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c7_torqueBalancing2012b((SFc7_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c7_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c7_torqueBalancing2012b
    ((SFc7_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c7_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c7_torqueBalancing2012b((SFc7_torqueBalancing2012bInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c7_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c7_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c7_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c7_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c7_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc7_torqueBalancing2012bInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c7_torqueBalancing2012b((SFc7_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc7_torqueBalancing2012b((SFc7_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c7_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c7_torqueBalancing2012b
      ((SFc7_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c7_torqueBalancing2012b(SimStruct *S)
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
      7);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,7,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,7,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,7);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,7,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,7,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,7);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3654278216U));
  ssSetChecksum1(S,(2040900627U));
  ssSetChecksum2(S,(1151640658U));
  ssSetChecksum3(S,(545537918U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c7_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c7_torqueBalancing2012b(SimStruct *S)
{
  SFc7_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc7_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc7_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc7_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c7_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c7_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c7_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c7_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c7_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c7_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c7_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c7_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c7_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c7_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c7_torqueBalancing2012b;
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

void c7_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void *
  data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c7_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c7_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c7_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c7_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
