/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c19_torqueBalancing2012b.h"
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
static const char * c19_debug_family_names[10] = { "Jc", "pinvJb", "nargin",
  "nargout", "JcLeftFoot", "JcRightFoot", "activeFeetConstraints", "qD", "reg",
  "y" };

static const char * c19_b_debug_family_names[5] = { "nargin", "nargout", "A",
  "regDamp", "pinvDampA" };

/* Function Declarations */
static void initialize_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void c19_update_debugger_state_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c19_st);
static void finalize_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c19_torqueBalancing2012b(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c19_chartstep_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void initSimStructsc19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void c19_pinvDamped(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_regDamp, real_T c19_pinvDampA[72]);
static void init_script_number_translation(uint32_T c19_machineNumber, uint32_T
  c19_chartNumber);
static const mxArray *c19_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_y, const char_T *c19_identifier, real_T
  c19_b_y[6]);
static void c19_b_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[6]);
static void c19_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_b_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static c19_struct_1ZGMVR6bgCMpDdXTSGnu6G c19_c_emlrt_marshallIn
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c19_u,
   const emlrtMsgIdentifier *c19_parentId);
static real_T c19_d_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_c_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static const mxArray *c19_d_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static const mxArray *c19_e_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static const mxArray *c19_f_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_g_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_e_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[72]);
static void c19_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_h_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_f_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[348]);
static void c19_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static const mxArray *c19_i_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static void c19_g_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[72]);
static void c19_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static void c19_info_helper(c19_ResolvedFunctionInfo c19_info[128]);
static void c19_b_info_helper(c19_ResolvedFunctionInfo c19_info[128]);
static void c19_eml_scalar_eg(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c19_eml_xgemm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_B[72], real_T c19_C[144], real_T
  c19_b_C[144]);
static void c19_mrdivide(SFc19_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c19_A[72], real_T c19_B[144], real_T c19_y[72]);
static void c19_realmin(SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void c19_eps(SFc19_torqueBalancing2012bInstanceStruct *chartInstance);
static void c19_eml_matlab_zgetrf(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_b_A[144], int32_T c19_ipiv[12],
  int32_T *c19_info);
static void c19_check_forloop_overflow_error
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c19_overflow);
static void c19_eml_xger(SFc19_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c19_m, int32_T c19_n, real_T c19_alpha1, int32_T c19_ix0, int32_T
  c19_iy0, real_T c19_A[144], int32_T c19_ia0, real_T c19_b_A[144]);
static void c19_eml_warning(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c19_eml_xtrsm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_B[72], real_T c19_b_B[72]);
static void c19_below_threshold(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c19_b_eml_xtrsm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_B[72], real_T c19_b_B[72]);
static void c19_b_eml_scalar_eg(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c19_b_eml_xgemm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_B[276], real_T c19_C[138], real_T
  c19_b_C[138]);
static void c19_c_eml_scalar_eg(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c19_j_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData);
static int32_T c19_h_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData);
static uint8_T c19_i_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_b_is_active_c19_torqueBalancing2012b, const
  char_T *c19_identifier);
static uint8_T c19_j_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId);
static void c19_c_eml_xgemm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_B[72], real_T c19_C[144]);
static void c19_b_eml_matlab_zgetrf(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], int32_T c19_ipiv[12], int32_T *c19_info);
static void c19_b_eml_xger(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c19_m, int32_T c19_n, real_T c19_alpha1, int32_T
  c19_ix0, int32_T c19_iy0, real_T c19_A[144], int32_T c19_ia0);
static void c19_c_eml_xtrsm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_B[72]);
static void c19_d_eml_xtrsm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_B[72]);
static void c19_d_eml_xgemm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_B[276], real_T c19_C[138]);
static void init_dsm_address_info(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c19_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c19_is_active_c19_torqueBalancing2012b = 0U;
}

static void initialize_params_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c19_m0 = NULL;
  const mxArray *c19_mxField;
  c19_struct_1ZGMVR6bgCMpDdXTSGnu6G c19_r0;
  sf_set_error_prefix_string(
    "Error evaluating data 'reg' in the parent workspace.\n");
  c19_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c19_mxField = sf_mex_getfield(c19_m0, "pinvTol", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c19_mxField), &c19_r0.pinvTol, 1, 0, 0U,
                      0, 0U, 0);
  c19_mxField = sf_mex_getfield(c19_m0, "pinvDamp", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c19_mxField), &c19_r0.pinvDamp, 1, 0, 0U,
                      0, 0U, 0);
  c19_mxField = sf_mex_getfield(c19_m0, "pinvDampVb", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c19_mxField), &c19_r0.pinvDampVb, 1, 0,
                      0U, 0, 0U, 0);
  c19_mxField = sf_mex_getfield(c19_m0, "HessianQP", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c19_mxField), &c19_r0.HessianQP, 1, 0,
                      0U, 0, 0U, 0);
  c19_mxField = sf_mex_getfield(c19_m0, "impedances", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c19_mxField), &c19_r0.impedances, 1, 0,
                      0U, 0, 0U, 0);
  c19_mxField = sf_mex_getfield(c19_m0, "dampings", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c19_mxField), &c19_r0.dampings, 1, 0, 0U,
                      0, 0U, 0);
  c19_mxField = sf_mex_getfield(c19_m0, "norm_tolerance", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c19_mxField), &c19_r0.norm_tolerance, 1,
                      0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c19_m0);
  chartInstance->c19_reg = c19_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c19_update_debugger_state_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c19_st;
  const mxArray *c19_y = NULL;
  int32_T c19_i0;
  real_T c19_u[6];
  const mxArray *c19_b_y = NULL;
  uint8_T c19_hoistedGlobal;
  uint8_T c19_b_u;
  const mxArray *c19_c_y = NULL;
  real_T (*c19_d_y)[6];
  c19_d_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c19_st = NULL;
  c19_st = NULL;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_createcellarray(2), FALSE);
  for (c19_i0 = 0; c19_i0 < 6; c19_i0++) {
    c19_u[c19_i0] = (*c19_d_y)[c19_i0];
  }

  c19_b_y = NULL;
  sf_mex_assign(&c19_b_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_setcell(c19_y, 0, c19_b_y);
  c19_hoistedGlobal = chartInstance->c19_is_active_c19_torqueBalancing2012b;
  c19_b_u = c19_hoistedGlobal;
  c19_c_y = NULL;
  sf_mex_assign(&c19_c_y, sf_mex_create("y", &c19_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c19_y, 1, c19_c_y);
  sf_mex_assign(&c19_st, c19_y, FALSE);
  return c19_st;
}

static void set_sim_state_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c19_st)
{
  const mxArray *c19_u;
  real_T c19_dv0[6];
  int32_T c19_i1;
  real_T (*c19_y)[6];
  c19_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c19_doneDoubleBufferReInit = TRUE;
  c19_u = sf_mex_dup(c19_st);
  c19_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c19_u, 0)), "y",
                       c19_dv0);
  for (c19_i1 = 0; c19_i1 < 6; c19_i1++) {
    (*c19_y)[c19_i1] = c19_dv0[c19_i1];
  }

  chartInstance->c19_is_active_c19_torqueBalancing2012b = c19_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c19_u, 1)),
     "is_active_c19_torqueBalancing2012b");
  sf_mex_destroy(&c19_u);
  c19_update_debugger_state_c19_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c19_st);
}

static void finalize_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c19_torqueBalancing2012b(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c19_i2;
  int32_T c19_i3;
  int32_T c19_i4;
  int32_T c19_i5;
  int32_T c19_i6;
  real_T (*c19_qD)[23];
  real_T (*c19_y)[6];
  real_T (*c19_activeFeetConstraints)[2];
  real_T (*c19_JcRightFoot)[174];
  real_T (*c19_JcLeftFoot)[174];
  c19_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c19_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c19_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
    (chartInstance->S, 2);
  c19_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c19_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
  for (c19_i2 = 0; c19_i2 < 174; c19_i2++) {
    _SFD_DATA_RANGE_CHECK((*c19_JcLeftFoot)[c19_i2], 0U);
  }

  for (c19_i3 = 0; c19_i3 < 174; c19_i3++) {
    _SFD_DATA_RANGE_CHECK((*c19_JcRightFoot)[c19_i3], 1U);
  }

  for (c19_i4 = 0; c19_i4 < 2; c19_i4++) {
    _SFD_DATA_RANGE_CHECK((*c19_activeFeetConstraints)[c19_i4], 2U);
  }

  for (c19_i5 = 0; c19_i5 < 6; c19_i5++) {
    _SFD_DATA_RANGE_CHECK((*c19_y)[c19_i5], 3U);
  }

  for (c19_i6 = 0; c19_i6 < 23; c19_i6++) {
    _SFD_DATA_RANGE_CHECK((*c19_qD)[c19_i6], 4U);
  }

  chartInstance->c19_sfEvent = CALL_EVENT;
  c19_chartstep_c19_torqueBalancing2012b(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c19_chartstep_c19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
  int32_T c19_i7;
  real_T c19_JcLeftFoot[174];
  int32_T c19_i8;
  real_T c19_JcRightFoot[174];
  int32_T c19_i9;
  real_T c19_activeFeetConstraints[2];
  int32_T c19_i10;
  real_T c19_qD[23];
  c19_struct_1ZGMVR6bgCMpDdXTSGnu6G c19_b_reg;
  uint32_T c19_debug_family_var_map[10];
  real_T c19_Jc[348];
  real_T c19_pinvJb[72];
  real_T c19_nargin = 5.0;
  real_T c19_nargout = 1.0;
  real_T c19_y[6];
  real_T c19_a;
  int32_T c19_i11;
  real_T c19_b[174];
  int32_T c19_i12;
  real_T c19_b_a;
  int32_T c19_i13;
  real_T c19_b_b[174];
  int32_T c19_i14;
  int32_T c19_i15;
  int32_T c19_i16;
  int32_T c19_i17;
  int32_T c19_i18;
  int32_T c19_i19;
  int32_T c19_i20;
  int32_T c19_i21;
  int32_T c19_i22;
  int32_T c19_i23;
  int32_T c19_i24;
  int32_T c19_i25;
  real_T c19_b_Jc[72];
  real_T c19_dv1[72];
  int32_T c19_i26;
  int32_T c19_i27;
  real_T c19_c_a[72];
  int32_T c19_i28;
  int32_T c19_i29;
  int32_T c19_i30;
  real_T c19_c_b[276];
  int32_T c19_i31;
  real_T c19_b_y[138];
  int32_T c19_i32;
  real_T c19_d_a[72];
  int32_T c19_i33;
  real_T c19_d_b[276];
  int32_T c19_i34;
  real_T c19_e_b[23];
  int32_T c19_i35;
  int32_T c19_i36;
  int32_T c19_i37;
  real_T c19_C[6];
  int32_T c19_i38;
  int32_T c19_i39;
  int32_T c19_i40;
  int32_T c19_i41;
  int32_T c19_i42;
  int32_T c19_i43;
  int32_T c19_i44;
  real_T (*c19_c_y)[6];
  real_T (*c19_b_qD)[23];
  real_T (*c19_b_activeFeetConstraints)[2];
  real_T (*c19_b_JcRightFoot)[174];
  real_T (*c19_b_JcLeftFoot)[174];
  c19_b_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c19_c_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c19_b_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
    (chartInstance->S, 2);
  c19_b_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c19_b_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
  for (c19_i7 = 0; c19_i7 < 174; c19_i7++) {
    c19_JcLeftFoot[c19_i7] = (*c19_b_JcLeftFoot)[c19_i7];
  }

  for (c19_i8 = 0; c19_i8 < 174; c19_i8++) {
    c19_JcRightFoot[c19_i8] = (*c19_b_JcRightFoot)[c19_i8];
  }

  for (c19_i9 = 0; c19_i9 < 2; c19_i9++) {
    c19_activeFeetConstraints[c19_i9] = (*c19_b_activeFeetConstraints)[c19_i9];
  }

  for (c19_i10 = 0; c19_i10 < 23; c19_i10++) {
    c19_qD[c19_i10] = (*c19_b_qD)[c19_i10];
  }

  c19_b_reg = chartInstance->c19_reg;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c19_debug_family_names,
    c19_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_Jc, 0U, c19_h_sf_marshallOut,
    c19_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_pinvJb, 1U, c19_g_sf_marshallOut,
    c19_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_nargin, 2U, c19_f_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_nargout, 3U, c19_f_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c19_JcLeftFoot, 4U, c19_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c19_JcRightFoot, 5U, c19_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c19_activeFeetConstraints, 6U, c19_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c19_qD, 7U, c19_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_b_reg, 8U, c19_b_sf_marshallOut,
    c19_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_y, 9U, c19_sf_marshallOut,
    c19_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 4);
  c19_a = c19_activeFeetConstraints[0];
  for (c19_i11 = 0; c19_i11 < 174; c19_i11++) {
    c19_b[c19_i11] = c19_JcLeftFoot[c19_i11];
  }

  for (c19_i12 = 0; c19_i12 < 174; c19_i12++) {
    c19_b[c19_i12] *= c19_a;
  }

  c19_b_a = c19_activeFeetConstraints[1];
  for (c19_i13 = 0; c19_i13 < 174; c19_i13++) {
    c19_b_b[c19_i13] = c19_JcRightFoot[c19_i13];
  }

  for (c19_i14 = 0; c19_i14 < 174; c19_i14++) {
    c19_b_b[c19_i14] *= c19_b_a;
  }

  c19_i15 = 0;
  c19_i16 = 0;
  for (c19_i17 = 0; c19_i17 < 29; c19_i17++) {
    for (c19_i18 = 0; c19_i18 < 6; c19_i18++) {
      c19_Jc[c19_i18 + c19_i15] = c19_b[c19_i18 + c19_i16];
    }

    c19_i15 += 12;
    c19_i16 += 6;
  }

  c19_i19 = 0;
  c19_i20 = 0;
  for (c19_i21 = 0; c19_i21 < 29; c19_i21++) {
    for (c19_i22 = 0; c19_i22 < 6; c19_i22++) {
      c19_Jc[(c19_i22 + c19_i19) + 6] = c19_b_b[c19_i22 + c19_i20];
    }

    c19_i19 += 12;
    c19_i20 += 6;
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 7);
  c19_i23 = 0;
  for (c19_i24 = 0; c19_i24 < 6; c19_i24++) {
    for (c19_i25 = 0; c19_i25 < 12; c19_i25++) {
      c19_b_Jc[c19_i25 + c19_i23] = c19_Jc[c19_i25 + c19_i23];
    }

    c19_i23 += 12;
  }

  c19_pinvDamped(chartInstance, c19_b_Jc, c19_b_reg.pinvDampVb, c19_dv1);
  for (c19_i26 = 0; c19_i26 < 72; c19_i26++) {
    c19_pinvJb[c19_i26] = c19_dv1[c19_i26];
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, 11);
  for (c19_i27 = 0; c19_i27 < 72; c19_i27++) {
    c19_c_a[c19_i27] = -c19_pinvJb[c19_i27];
  }

  c19_i28 = 0;
  for (c19_i29 = 0; c19_i29 < 23; c19_i29++) {
    for (c19_i30 = 0; c19_i30 < 12; c19_i30++) {
      c19_c_b[c19_i30 + c19_i28] = c19_Jc[(c19_i30 + c19_i28) + 72];
    }

    c19_i28 += 12;
  }

  c19_b_eml_scalar_eg(chartInstance);
  c19_b_eml_scalar_eg(chartInstance);
  for (c19_i31 = 0; c19_i31 < 138; c19_i31++) {
    c19_b_y[c19_i31] = 0.0;
  }

  for (c19_i32 = 0; c19_i32 < 72; c19_i32++) {
    c19_d_a[c19_i32] = c19_c_a[c19_i32];
  }

  for (c19_i33 = 0; c19_i33 < 276; c19_i33++) {
    c19_d_b[c19_i33] = c19_c_b[c19_i33];
  }

  c19_d_eml_xgemm(chartInstance, c19_d_a, c19_d_b, c19_b_y);
  for (c19_i34 = 0; c19_i34 < 23; c19_i34++) {
    c19_e_b[c19_i34] = c19_qD[c19_i34];
  }

  c19_c_eml_scalar_eg(chartInstance);
  c19_c_eml_scalar_eg(chartInstance);
  for (c19_i35 = 0; c19_i35 < 6; c19_i35++) {
    c19_y[c19_i35] = 0.0;
  }

  for (c19_i36 = 0; c19_i36 < 6; c19_i36++) {
    c19_y[c19_i36] = 0.0;
  }

  for (c19_i37 = 0; c19_i37 < 6; c19_i37++) {
    c19_C[c19_i37] = c19_y[c19_i37];
  }

  for (c19_i38 = 0; c19_i38 < 6; c19_i38++) {
    c19_y[c19_i38] = c19_C[c19_i38];
  }

  for (c19_i39 = 0; c19_i39 < 6; c19_i39++) {
    c19_C[c19_i39] = c19_y[c19_i39];
  }

  for (c19_i40 = 0; c19_i40 < 6; c19_i40++) {
    c19_y[c19_i40] = c19_C[c19_i40];
  }

  for (c19_i41 = 0; c19_i41 < 6; c19_i41++) {
    c19_y[c19_i41] = 0.0;
    c19_i42 = 0;
    for (c19_i43 = 0; c19_i43 < 23; c19_i43++) {
      c19_y[c19_i41] += c19_b_y[c19_i42 + c19_i41] * c19_e_b[c19_i43];
      c19_i42 += 6;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c19_sfEvent, -11);
  _SFD_SYMBOL_SCOPE_POP();
  for (c19_i44 = 0; c19_i44 < 6; c19_i44++) {
    (*c19_c_y)[c19_i44] = c19_y[c19_i44];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 18U, chartInstance->c19_sfEvent);
}

static void initSimStructsc19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc19_torqueBalancing2012b
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c19_pinvDamped(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_regDamp, real_T c19_pinvDampA[72])
{
  uint32_T c19_debug_family_var_map[5];
  real_T c19_nargin = 2.0;
  real_T c19_nargout = 1.0;
  int32_T c19_i45;
  real_T c19_a[72];
  int32_T c19_i46;
  int32_T c19_i47;
  int32_T c19_i48;
  int32_T c19_i49;
  real_T c19_b[72];
  int32_T c19_i50;
  real_T c19_y[144];
  int32_T c19_i51;
  real_T c19_b_a[72];
  int32_T c19_i52;
  real_T c19_b_b[72];
  real_T c19_c_a;
  int32_T c19_i53;
  static real_T c19_c_b[144] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c19_b_y[144];
  int32_T c19_i54;
  int32_T c19_i55;
  int32_T c19_i56;
  int32_T c19_i57;
  real_T c19_b_A[72];
  int32_T c19_i58;
  real_T c19_c_y[144];
  real_T c19_dv2[72];
  int32_T c19_i59;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c19_b_debug_family_names,
    c19_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_nargin, 0U, c19_f_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_nargout, 1U, c19_f_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_A, 2U, c19_i_sf_marshallOut,
    c19_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c19_regDamp, 3U, c19_f_sf_marshallOut,
    c19_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c19_pinvDampA, 4U, c19_g_sf_marshallOut,
    c19_d_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c19_sfEvent, 10);
  for (c19_i45 = 0; c19_i45 < 72; c19_i45++) {
    c19_a[c19_i45] = c19_A[c19_i45];
  }

  c19_i46 = 0;
  for (c19_i47 = 0; c19_i47 < 12; c19_i47++) {
    c19_i48 = 0;
    for (c19_i49 = 0; c19_i49 < 6; c19_i49++) {
      c19_b[c19_i49 + c19_i46] = c19_A[c19_i48 + c19_i47];
      c19_i48 += 12;
    }

    c19_i46 += 6;
  }

  c19_eml_scalar_eg(chartInstance);
  c19_eml_scalar_eg(chartInstance);
  for (c19_i50 = 0; c19_i50 < 144; c19_i50++) {
    c19_y[c19_i50] = 0.0;
  }

  for (c19_i51 = 0; c19_i51 < 72; c19_i51++) {
    c19_b_a[c19_i51] = c19_a[c19_i51];
  }

  for (c19_i52 = 0; c19_i52 < 72; c19_i52++) {
    c19_b_b[c19_i52] = c19_b[c19_i52];
  }

  c19_c_eml_xgemm(chartInstance, c19_b_a, c19_b_b, c19_y);
  c19_c_a = c19_regDamp;
  for (c19_i53 = 0; c19_i53 < 144; c19_i53++) {
    c19_b_y[c19_i53] = c19_c_a * c19_c_b[c19_i53];
  }

  c19_i54 = 0;
  for (c19_i55 = 0; c19_i55 < 12; c19_i55++) {
    c19_i56 = 0;
    for (c19_i57 = 0; c19_i57 < 6; c19_i57++) {
      c19_b_A[c19_i57 + c19_i54] = c19_A[c19_i56 + c19_i55];
      c19_i56 += 12;
    }

    c19_i54 += 6;
  }

  for (c19_i58 = 0; c19_i58 < 144; c19_i58++) {
    c19_c_y[c19_i58] = c19_y[c19_i58] + c19_b_y[c19_i58];
  }

  c19_mrdivide(chartInstance, c19_b_A, c19_c_y, c19_dv2);
  for (c19_i59 = 0; c19_i59 < 72; c19_i59++) {
    c19_pinvDampA[c19_i59] = c19_dv2[c19_i59];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c19_sfEvent, -10);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c19_machineNumber, uint32_T
  c19_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c19_chartNumber, 0U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m"));
}

static const mxArray *c19_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i60;
  real_T c19_b_inData[6];
  int32_T c19_i61;
  real_T c19_u[6];
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  for (c19_i60 = 0; c19_i60 < 6; c19_i60++) {
    c19_b_inData[c19_i60] = (*(real_T (*)[6])c19_inData)[c19_i60];
  }

  for (c19_i61 = 0; c19_i61 < 6; c19_i61++) {
    c19_u[c19_i61] = c19_b_inData[c19_i61];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static void c19_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_y, const char_T *c19_identifier, real_T
  c19_b_y[6])
{
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_y), &c19_thisId, c19_b_y);
  sf_mex_destroy(&c19_y);
}

static void c19_b_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[6])
{
  real_T c19_dv3[6];
  int32_T c19_i62;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv3, 1, 0, 0U, 1, 0U, 1, 6);
  for (c19_i62 = 0; c19_i62 < 6; c19_i62++) {
    c19_y[c19_i62] = c19_dv3[c19_i62];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_y;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_b_y[6];
  int32_T c19_i63;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_y = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_y), &c19_thisId, c19_b_y);
  sf_mex_destroy(&c19_y);
  for (c19_i63 = 0; c19_i63 < 6; c19_i63++) {
    (*(real_T (*)[6])c19_outData)[c19_i63] = c19_b_y[c19_i63];
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_b_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  c19_struct_1ZGMVR6bgCMpDdXTSGnu6G c19_u;
  const mxArray *c19_y = NULL;
  real_T c19_b_u;
  const mxArray *c19_b_y = NULL;
  real_T c19_c_u;
  const mxArray *c19_c_y = NULL;
  real_T c19_d_u;
  const mxArray *c19_d_y = NULL;
  real_T c19_e_u;
  const mxArray *c19_e_y = NULL;
  real_T c19_f_u;
  const mxArray *c19_f_y = NULL;
  real_T c19_g_u;
  const mxArray *c19_g_y = NULL;
  real_T c19_h_u;
  const mxArray *c19_h_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(c19_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c19_inData;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c19_b_u = c19_u.pinvTol;
  c19_b_y = NULL;
  sf_mex_assign(&c19_b_y, sf_mex_create("y", &c19_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c19_y, c19_b_y, "pinvTol", "pinvTol", 0);
  c19_c_u = c19_u.pinvDamp;
  c19_c_y = NULL;
  sf_mex_assign(&c19_c_y, sf_mex_create("y", &c19_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c19_y, c19_c_y, "pinvDamp", "pinvDamp", 0);
  c19_d_u = c19_u.pinvDampVb;
  c19_d_y = NULL;
  sf_mex_assign(&c19_d_y, sf_mex_create("y", &c19_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c19_y, c19_d_y, "pinvDampVb", "pinvDampVb", 0);
  c19_e_u = c19_u.HessianQP;
  c19_e_y = NULL;
  sf_mex_assign(&c19_e_y, sf_mex_create("y", &c19_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c19_y, c19_e_y, "HessianQP", "HessianQP", 0);
  c19_f_u = c19_u.impedances;
  c19_f_y = NULL;
  sf_mex_assign(&c19_f_y, sf_mex_create("y", &c19_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c19_y, c19_f_y, "impedances", "impedances", 0);
  c19_g_u = c19_u.dampings;
  c19_g_y = NULL;
  sf_mex_assign(&c19_g_y, sf_mex_create("y", &c19_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c19_y, c19_g_y, "dampings", "dampings", 0);
  c19_h_u = c19_u.norm_tolerance;
  c19_h_y = NULL;
  sf_mex_assign(&c19_h_y, sf_mex_create("y", &c19_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c19_y, c19_h_y, "norm_tolerance", "norm_tolerance", 0);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static c19_struct_1ZGMVR6bgCMpDdXTSGnu6G c19_c_emlrt_marshallIn
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c19_u,
   const emlrtMsgIdentifier *c19_parentId)
{
  c19_struct_1ZGMVR6bgCMpDdXTSGnu6G c19_y;
  emlrtMsgIdentifier c19_thisId;
  static const char * c19_fieldNames[7] = { "pinvTol", "pinvDamp", "pinvDampVb",
    "HessianQP", "impedances", "dampings", "norm_tolerance" };

  c19_thisId.fParent = c19_parentId;
  sf_mex_check_struct(c19_parentId, c19_u, 7, c19_fieldNames, 0U, 0);
  c19_thisId.fIdentifier = "pinvTol";
  c19_y.pinvTol = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c19_u, "pinvTol", "pinvTol", 0)), &c19_thisId);
  c19_thisId.fIdentifier = "pinvDamp";
  c19_y.pinvDamp = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c19_u, "pinvDamp", "pinvDamp", 0)), &c19_thisId);
  c19_thisId.fIdentifier = "pinvDampVb";
  c19_y.pinvDampVb = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c19_u, "pinvDampVb", "pinvDampVb", 0)), &c19_thisId);
  c19_thisId.fIdentifier = "HessianQP";
  c19_y.HessianQP = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c19_u, "HessianQP", "HessianQP", 0)), &c19_thisId);
  c19_thisId.fIdentifier = "impedances";
  c19_y.impedances = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c19_u, "impedances", "impedances", 0)), &c19_thisId);
  c19_thisId.fIdentifier = "dampings";
  c19_y.dampings = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c19_u, "dampings", "dampings", 0)), &c19_thisId);
  c19_thisId.fIdentifier = "norm_tolerance";
  c19_y.norm_tolerance = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c19_u, "norm_tolerance", "norm_tolerance", 0)), &c19_thisId);
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static real_T c19_d_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  real_T c19_y;
  real_T c19_d0;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_d0, 1, 0, 0U, 0, 0U, 0);
  c19_y = c19_d0;
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_reg;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  c19_struct_1ZGMVR6bgCMpDdXTSGnu6G c19_y;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_b_reg = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_reg),
    &c19_thisId);
  sf_mex_destroy(&c19_b_reg);
  *(c19_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_c_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i64;
  real_T c19_b_inData[23];
  int32_T c19_i65;
  real_T c19_u[23];
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  for (c19_i64 = 0; c19_i64 < 23; c19_i64++) {
    c19_b_inData[c19_i64] = (*(real_T (*)[23])c19_inData)[c19_i64];
  }

  for (c19_i65 = 0; c19_i65 < 23; c19_i65++) {
    c19_u[c19_i65] = c19_b_inData[c19_i65];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static const mxArray *c19_d_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i66;
  real_T c19_b_inData[2];
  int32_T c19_i67;
  real_T c19_u[2];
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  for (c19_i66 = 0; c19_i66 < 2; c19_i66++) {
    c19_b_inData[c19_i66] = (*(real_T (*)[2])c19_inData)[c19_i66];
  }

  for (c19_i67 = 0; c19_i67 < 2; c19_i67++) {
    c19_u[c19_i67] = c19_b_inData[c19_i67];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static const mxArray *c19_e_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i68;
  int32_T c19_i69;
  int32_T c19_i70;
  real_T c19_b_inData[174];
  int32_T c19_i71;
  int32_T c19_i72;
  int32_T c19_i73;
  real_T c19_u[174];
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i68 = 0;
  for (c19_i69 = 0; c19_i69 < 29; c19_i69++) {
    for (c19_i70 = 0; c19_i70 < 6; c19_i70++) {
      c19_b_inData[c19_i70 + c19_i68] = (*(real_T (*)[174])c19_inData)[c19_i70 +
        c19_i68];
    }

    c19_i68 += 6;
  }

  c19_i71 = 0;
  for (c19_i72 = 0; c19_i72 < 29; c19_i72++) {
    for (c19_i73 = 0; c19_i73 < 6; c19_i73++) {
      c19_u[c19_i73 + c19_i71] = c19_b_inData[c19_i73 + c19_i71];
    }

    c19_i71 += 6;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 6, 29),
                FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static const mxArray *c19_f_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  real_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(real_T *)c19_inData;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static void c19_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_nargout;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_nargout = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_nargout),
    &c19_thisId);
  sf_mex_destroy(&c19_nargout);
  *(real_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_g_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i74;
  int32_T c19_i75;
  int32_T c19_i76;
  real_T c19_b_inData[72];
  int32_T c19_i77;
  int32_T c19_i78;
  int32_T c19_i79;
  real_T c19_u[72];
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i74 = 0;
  for (c19_i75 = 0; c19_i75 < 12; c19_i75++) {
    for (c19_i76 = 0; c19_i76 < 6; c19_i76++) {
      c19_b_inData[c19_i76 + c19_i74] = (*(real_T (*)[72])c19_inData)[c19_i76 +
        c19_i74];
    }

    c19_i74 += 6;
  }

  c19_i77 = 0;
  for (c19_i78 = 0; c19_i78 < 12; c19_i78++) {
    for (c19_i79 = 0; c19_i79 < 6; c19_i79++) {
      c19_u[c19_i79 + c19_i77] = c19_b_inData[c19_i79 + c19_i77];
    }

    c19_i77 += 6;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 6, 12),
                FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static void c19_e_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[72])
{
  real_T c19_dv4[72];
  int32_T c19_i80;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv4, 1, 0, 0U, 1, 0U, 2, 6,
                12);
  for (c19_i80 = 0; c19_i80 < 72; c19_i80++) {
    c19_y[c19_i80] = c19_dv4[c19_i80];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_pinvJb;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[72];
  int32_T c19_i81;
  int32_T c19_i82;
  int32_T c19_i83;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_pinvJb = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_pinvJb), &c19_thisId,
    c19_y);
  sf_mex_destroy(&c19_pinvJb);
  c19_i81 = 0;
  for (c19_i82 = 0; c19_i82 < 12; c19_i82++) {
    for (c19_i83 = 0; c19_i83 < 6; c19_i83++) {
      (*(real_T (*)[72])c19_outData)[c19_i83 + c19_i81] = c19_y[c19_i83 +
        c19_i81];
    }

    c19_i81 += 6;
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_h_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i84;
  int32_T c19_i85;
  int32_T c19_i86;
  real_T c19_b_inData[348];
  int32_T c19_i87;
  int32_T c19_i88;
  int32_T c19_i89;
  real_T c19_u[348];
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i84 = 0;
  for (c19_i85 = 0; c19_i85 < 29; c19_i85++) {
    for (c19_i86 = 0; c19_i86 < 12; c19_i86++) {
      c19_b_inData[c19_i86 + c19_i84] = (*(real_T (*)[348])c19_inData)[c19_i86 +
        c19_i84];
    }

    c19_i84 += 12;
  }

  c19_i87 = 0;
  for (c19_i88 = 0; c19_i88 < 29; c19_i88++) {
    for (c19_i89 = 0; c19_i89 < 12; c19_i89++) {
      c19_u[c19_i89 + c19_i87] = c19_b_inData[c19_i89 + c19_i87];
    }

    c19_i87 += 12;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 12, 29),
                FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static void c19_f_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[348])
{
  real_T c19_dv5[348];
  int32_T c19_i90;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv5, 1, 0, 0U, 1, 0U, 2, 12,
                29);
  for (c19_i90 = 0; c19_i90 < 348; c19_i90++) {
    c19_y[c19_i90] = c19_dv5[c19_i90];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_Jc;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[348];
  int32_T c19_i91;
  int32_T c19_i92;
  int32_T c19_i93;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_Jc = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_Jc), &c19_thisId, c19_y);
  sf_mex_destroy(&c19_Jc);
  c19_i91 = 0;
  for (c19_i92 = 0; c19_i92 < 29; c19_i92++) {
    for (c19_i93 = 0; c19_i93 < 12; c19_i93++) {
      (*(real_T (*)[348])c19_outData)[c19_i93 + c19_i91] = c19_y[c19_i93 +
        c19_i91];
    }

    c19_i91 += 12;
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

static const mxArray *c19_i_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_i94;
  int32_T c19_i95;
  int32_T c19_i96;
  real_T c19_b_inData[72];
  int32_T c19_i97;
  int32_T c19_i98;
  int32_T c19_i99;
  real_T c19_u[72];
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_i94 = 0;
  for (c19_i95 = 0; c19_i95 < 6; c19_i95++) {
    for (c19_i96 = 0; c19_i96 < 12; c19_i96++) {
      c19_b_inData[c19_i96 + c19_i94] = (*(real_T (*)[72])c19_inData)[c19_i96 +
        c19_i94];
    }

    c19_i94 += 12;
  }

  c19_i97 = 0;
  for (c19_i98 = 0; c19_i98 < 6; c19_i98++) {
    for (c19_i99 = 0; c19_i99 < 12; c19_i99++) {
      c19_u[c19_i99 + c19_i97] = c19_b_inData[c19_i99 + c19_i97];
    }

    c19_i97 += 12;
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 0, 0U, 1U, 0U, 2, 12, 6),
                FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static void c19_g_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId,
  real_T c19_y[72])
{
  real_T c19_dv6[72];
  int32_T c19_i100;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), c19_dv6, 1, 0, 0U, 1, 0U, 2, 12,
                6);
  for (c19_i100 = 0; c19_i100 < 72; c19_i100++) {
    c19_y[c19_i100] = c19_dv6[c19_i100];
  }

  sf_mex_destroy(&c19_u);
}

static void c19_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_A;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  real_T c19_y[72];
  int32_T c19_i101;
  int32_T c19_i102;
  int32_T c19_i103;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_A = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_A), &c19_thisId, c19_y);
  sf_mex_destroy(&c19_A);
  c19_i101 = 0;
  for (c19_i102 = 0; c19_i102 < 6; c19_i102++) {
    for (c19_i103 = 0; c19_i103 < 12; c19_i103++) {
      (*(real_T (*)[72])c19_outData)[c19_i103 + c19_i101] = c19_y[c19_i103 +
        c19_i101];
    }

    c19_i101 += 12;
  }

  sf_mex_destroy(&c19_mxArrayInData);
}

const mxArray *sf_c19_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c19_nameCaptureInfo;
  c19_ResolvedFunctionInfo c19_info[128];
  const mxArray *c19_m1 = NULL;
  int32_T c19_i104;
  c19_ResolvedFunctionInfo *c19_r1;
  c19_nameCaptureInfo = NULL;
  c19_nameCaptureInfo = NULL;
  c19_info_helper(c19_info);
  c19_b_info_helper(c19_info);
  sf_mex_assign(&c19_m1, sf_mex_createstruct("nameCaptureInfo", 1, 128), FALSE);
  for (c19_i104 = 0; c19_i104 < 128; c19_i104++) {
    c19_r1 = &c19_info[c19_i104];
    sf_mex_addfield(c19_m1, sf_mex_create("nameCaptureInfo", c19_r1->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c19_r1->context)), "context", "nameCaptureInfo",
                    c19_i104);
    sf_mex_addfield(c19_m1, sf_mex_create("nameCaptureInfo", c19_r1->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c19_r1->name)), "name", "nameCaptureInfo",
                    c19_i104);
    sf_mex_addfield(c19_m1, sf_mex_create("nameCaptureInfo",
      c19_r1->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c19_r1->dominantType)),
                    "dominantType", "nameCaptureInfo", c19_i104);
    sf_mex_addfield(c19_m1, sf_mex_create("nameCaptureInfo", c19_r1->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c19_r1->resolved)), "resolved",
                    "nameCaptureInfo", c19_i104);
    sf_mex_addfield(c19_m1, sf_mex_create("nameCaptureInfo", &c19_r1->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c19_i104);
    sf_mex_addfield(c19_m1, sf_mex_create("nameCaptureInfo", &c19_r1->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c19_i104);
    sf_mex_addfield(c19_m1, sf_mex_create("nameCaptureInfo",
      &c19_r1->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c19_i104);
    sf_mex_addfield(c19_m1, sf_mex_create("nameCaptureInfo",
      &c19_r1->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c19_i104);
  }

  sf_mex_assign(&c19_nameCaptureInfo, c19_m1, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c19_nameCaptureInfo);
  return c19_nameCaptureInfo;
}

static void c19_info_helper(c19_ResolvedFunctionInfo c19_info[128])
{
  c19_info[0].context = "";
  c19_info[0].name = "mtimes";
  c19_info[0].dominantType = "double";
  c19_info[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[0].fileTimeLo = 1289519692U;
  c19_info[0].fileTimeHi = 0U;
  c19_info[0].mFileTimeLo = 0U;
  c19_info[0].mFileTimeHi = 0U;
  c19_info[1].context = "";
  c19_info[1].name = "pinvDamped";
  c19_info[1].dominantType = "double";
  c19_info[1].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c19_info[1].fileTimeLo = 1495631764U;
  c19_info[1].fileTimeHi = 0U;
  c19_info[1].mFileTimeLo = 0U;
  c19_info[1].mFileTimeHi = 0U;
  c19_info[2].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c19_info[2].name = "mtimes";
  c19_info[2].dominantType = "double";
  c19_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[2].fileTimeLo = 1289519692U;
  c19_info[2].fileTimeHi = 0U;
  c19_info[2].mFileTimeLo = 0U;
  c19_info[2].mFileTimeHi = 0U;
  c19_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[3].name = "eml_index_class";
  c19_info[3].dominantType = "";
  c19_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[3].fileTimeLo = 1323170578U;
  c19_info[3].fileTimeHi = 0U;
  c19_info[3].mFileTimeLo = 0U;
  c19_info[3].mFileTimeHi = 0U;
  c19_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[4].name = "eml_scalar_eg";
  c19_info[4].dominantType = "double";
  c19_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[4].fileTimeLo = 1286818796U;
  c19_info[4].fileTimeHi = 0U;
  c19_info[4].mFileTimeLo = 0U;
  c19_info[4].mFileTimeHi = 0U;
  c19_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[5].name = "eml_xgemm";
  c19_info[5].dominantType = "char";
  c19_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c19_info[5].fileTimeLo = 1299076772U;
  c19_info[5].fileTimeHi = 0U;
  c19_info[5].mFileTimeLo = 0U;
  c19_info[5].mFileTimeHi = 0U;
  c19_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c19_info[6].name = "eml_blas_inline";
  c19_info[6].dominantType = "";
  c19_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c19_info[6].fileTimeLo = 1299076768U;
  c19_info[6].fileTimeHi = 0U;
  c19_info[6].mFileTimeLo = 0U;
  c19_info[6].mFileTimeHi = 0U;
  c19_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c19_info[7].name = "mtimes";
  c19_info[7].dominantType = "double";
  c19_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[7].fileTimeLo = 1289519692U;
  c19_info[7].fileTimeHi = 0U;
  c19_info[7].mFileTimeLo = 0U;
  c19_info[7].mFileTimeHi = 0U;
  c19_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c19_info[8].name = "eml_index_class";
  c19_info[8].dominantType = "";
  c19_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[8].fileTimeLo = 1323170578U;
  c19_info[8].fileTimeHi = 0U;
  c19_info[8].mFileTimeLo = 0U;
  c19_info[8].mFileTimeHi = 0U;
  c19_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c19_info[9].name = "eml_scalar_eg";
  c19_info[9].dominantType = "double";
  c19_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[9].fileTimeLo = 1286818796U;
  c19_info[9].fileTimeHi = 0U;
  c19_info[9].mFileTimeLo = 0U;
  c19_info[9].mFileTimeHi = 0U;
  c19_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c19_info[10].name = "eml_refblas_xgemm";
  c19_info[10].dominantType = "char";
  c19_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c19_info[10].fileTimeLo = 1299076774U;
  c19_info[10].fileTimeHi = 0U;
  c19_info[10].mFileTimeLo = 0U;
  c19_info[10].mFileTimeHi = 0U;
  c19_info[11].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c19_info[11].name = "eye";
  c19_info[11].dominantType = "double";
  c19_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c19_info[11].fileTimeLo = 1286818688U;
  c19_info[11].fileTimeHi = 0U;
  c19_info[11].mFileTimeLo = 0U;
  c19_info[11].mFileTimeHi = 0U;
  c19_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c19_info[12].name = "eml_assert_valid_size_arg";
  c19_info[12].dominantType = "double";
  c19_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c19_info[12].fileTimeLo = 1286818694U;
  c19_info[12].fileTimeHi = 0U;
  c19_info[12].mFileTimeLo = 0U;
  c19_info[12].mFileTimeHi = 0U;
  c19_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c19_info[13].name = "isinf";
  c19_info[13].dominantType = "double";
  c19_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c19_info[13].fileTimeLo = 1286818760U;
  c19_info[13].fileTimeHi = 0U;
  c19_info[13].mFileTimeLo = 0U;
  c19_info[13].mFileTimeHi = 0U;
  c19_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c19_info[14].name = "mtimes";
  c19_info[14].dominantType = "double";
  c19_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[14].fileTimeLo = 1289519692U;
  c19_info[14].fileTimeHi = 0U;
  c19_info[14].mFileTimeLo = 0U;
  c19_info[14].mFileTimeHi = 0U;
  c19_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c19_info[15].name = "eml_index_class";
  c19_info[15].dominantType = "";
  c19_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[15].fileTimeLo = 1323170578U;
  c19_info[15].fileTimeHi = 0U;
  c19_info[15].mFileTimeLo = 0U;
  c19_info[15].mFileTimeHi = 0U;
  c19_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c19_info[16].name = "intmax";
  c19_info[16].dominantType = "char";
  c19_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c19_info[16].fileTimeLo = 1311255316U;
  c19_info[16].fileTimeHi = 0U;
  c19_info[16].mFileTimeLo = 0U;
  c19_info[16].mFileTimeHi = 0U;
  c19_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c19_info[17].name = "eml_is_float_class";
  c19_info[17].dominantType = "char";
  c19_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c19_info[17].fileTimeLo = 1286818782U;
  c19_info[17].fileTimeHi = 0U;
  c19_info[17].mFileTimeLo = 0U;
  c19_info[17].mFileTimeHi = 0U;
  c19_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c19_info[18].name = "min";
  c19_info[18].dominantType = "double";
  c19_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c19_info[18].fileTimeLo = 1311255318U;
  c19_info[18].fileTimeHi = 0U;
  c19_info[18].mFileTimeLo = 0U;
  c19_info[18].mFileTimeHi = 0U;
  c19_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c19_info[19].name = "eml_min_or_max";
  c19_info[19].dominantType = "char";
  c19_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c19_info[19].fileTimeLo = 1334071490U;
  c19_info[19].fileTimeHi = 0U;
  c19_info[19].mFileTimeLo = 0U;
  c19_info[19].mFileTimeHi = 0U;
  c19_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c19_info[20].name = "eml_scalar_eg";
  c19_info[20].dominantType = "double";
  c19_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[20].fileTimeLo = 1286818796U;
  c19_info[20].fileTimeHi = 0U;
  c19_info[20].mFileTimeLo = 0U;
  c19_info[20].mFileTimeHi = 0U;
  c19_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c19_info[21].name = "eml_scalexp_alloc";
  c19_info[21].dominantType = "double";
  c19_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c19_info[21].fileTimeLo = 1352424860U;
  c19_info[21].fileTimeHi = 0U;
  c19_info[21].mFileTimeLo = 0U;
  c19_info[21].mFileTimeHi = 0U;
  c19_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c19_info[22].name = "eml_index_class";
  c19_info[22].dominantType = "";
  c19_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[22].fileTimeLo = 1323170578U;
  c19_info[22].fileTimeHi = 0U;
  c19_info[22].mFileTimeLo = 0U;
  c19_info[22].mFileTimeHi = 0U;
  c19_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c19_info[23].name = "eml_scalar_eg";
  c19_info[23].dominantType = "double";
  c19_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[23].fileTimeLo = 1286818796U;
  c19_info[23].fileTimeHi = 0U;
  c19_info[23].mFileTimeLo = 0U;
  c19_info[23].mFileTimeHi = 0U;
  c19_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c19_info[24].name = "eml_index_class";
  c19_info[24].dominantType = "";
  c19_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[24].fileTimeLo = 1323170578U;
  c19_info[24].fileTimeHi = 0U;
  c19_info[24].mFileTimeLo = 0U;
  c19_info[24].mFileTimeHi = 0U;
  c19_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c19_info[25].name = "eml_int_forloop_overflow_check";
  c19_info[25].dominantType = "";
  c19_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c19_info[25].fileTimeLo = 1346510340U;
  c19_info[25].fileTimeHi = 0U;
  c19_info[25].mFileTimeLo = 0U;
  c19_info[25].mFileTimeHi = 0U;
  c19_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c19_info[26].name = "intmax";
  c19_info[26].dominantType = "char";
  c19_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c19_info[26].fileTimeLo = 1311255316U;
  c19_info[26].fileTimeHi = 0U;
  c19_info[26].mFileTimeLo = 0U;
  c19_info[26].mFileTimeHi = 0U;
  c19_info[27].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c19_info[27].name = "mrdivide";
  c19_info[27].dominantType = "double";
  c19_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c19_info[27].fileTimeLo = 1357951548U;
  c19_info[27].fileTimeHi = 0U;
  c19_info[27].mFileTimeLo = 1319729966U;
  c19_info[27].mFileTimeHi = 0U;
  c19_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c19_info[28].name = "mldivide";
  c19_info[28].dominantType = "double";
  c19_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c19_info[28].fileTimeLo = 1357951548U;
  c19_info[28].fileTimeHi = 0U;
  c19_info[28].mFileTimeLo = 1319729966U;
  c19_info[28].mFileTimeHi = 0U;
  c19_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c19_info[29].name = "eml_lusolve";
  c19_info[29].dominantType = "double";
  c19_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c19_info[29].fileTimeLo = 1309451196U;
  c19_info[29].fileTimeHi = 0U;
  c19_info[29].mFileTimeLo = 0U;
  c19_info[29].mFileTimeHi = 0U;
  c19_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c19_info[30].name = "eml_index_class";
  c19_info[30].dominantType = "";
  c19_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[30].fileTimeLo = 1323170578U;
  c19_info[30].fileTimeHi = 0U;
  c19_info[30].mFileTimeLo = 0U;
  c19_info[30].mFileTimeHi = 0U;
  c19_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c19_info[31].name = "eml_index_class";
  c19_info[31].dominantType = "";
  c19_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[31].fileTimeLo = 1323170578U;
  c19_info[31].fileTimeHi = 0U;
  c19_info[31].mFileTimeLo = 0U;
  c19_info[31].mFileTimeHi = 0U;
  c19_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c19_info[32].name = "eml_xgetrf";
  c19_info[32].dominantType = "double";
  c19_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c19_info[32].fileTimeLo = 1286818806U;
  c19_info[32].fileTimeHi = 0U;
  c19_info[32].mFileTimeLo = 0U;
  c19_info[32].mFileTimeHi = 0U;
  c19_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c19_info[33].name = "eml_lapack_xgetrf";
  c19_info[33].dominantType = "double";
  c19_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c19_info[33].fileTimeLo = 1286818810U;
  c19_info[33].fileTimeHi = 0U;
  c19_info[33].mFileTimeLo = 0U;
  c19_info[33].mFileTimeHi = 0U;
  c19_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c19_info[34].name = "eml_matlab_zgetrf";
  c19_info[34].dominantType = "double";
  c19_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[34].fileTimeLo = 1302688994U;
  c19_info[34].fileTimeHi = 0U;
  c19_info[34].mFileTimeLo = 0U;
  c19_info[34].mFileTimeHi = 0U;
  c19_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[35].name = "realmin";
  c19_info[35].dominantType = "char";
  c19_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c19_info[35].fileTimeLo = 1307651242U;
  c19_info[35].fileTimeHi = 0U;
  c19_info[35].mFileTimeLo = 0U;
  c19_info[35].mFileTimeHi = 0U;
  c19_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c19_info[36].name = "eml_realmin";
  c19_info[36].dominantType = "char";
  c19_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c19_info[36].fileTimeLo = 1307651244U;
  c19_info[36].fileTimeHi = 0U;
  c19_info[36].mFileTimeLo = 0U;
  c19_info[36].mFileTimeHi = 0U;
  c19_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c19_info[37].name = "eml_float_model";
  c19_info[37].dominantType = "char";
  c19_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c19_info[37].fileTimeLo = 1326727996U;
  c19_info[37].fileTimeHi = 0U;
  c19_info[37].mFileTimeLo = 0U;
  c19_info[37].mFileTimeHi = 0U;
  c19_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[38].name = "eps";
  c19_info[38].dominantType = "char";
  c19_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c19_info[38].fileTimeLo = 1326727996U;
  c19_info[38].fileTimeHi = 0U;
  c19_info[38].mFileTimeLo = 0U;
  c19_info[38].mFileTimeHi = 0U;
  c19_info[39].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c19_info[39].name = "eml_is_float_class";
  c19_info[39].dominantType = "char";
  c19_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c19_info[39].fileTimeLo = 1286818782U;
  c19_info[39].fileTimeHi = 0U;
  c19_info[39].mFileTimeLo = 0U;
  c19_info[39].mFileTimeHi = 0U;
  c19_info[40].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c19_info[40].name = "eml_eps";
  c19_info[40].dominantType = "char";
  c19_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c19_info[40].fileTimeLo = 1326727996U;
  c19_info[40].fileTimeHi = 0U;
  c19_info[40].mFileTimeLo = 0U;
  c19_info[40].mFileTimeHi = 0U;
  c19_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c19_info[41].name = "eml_float_model";
  c19_info[41].dominantType = "char";
  c19_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c19_info[41].fileTimeLo = 1326727996U;
  c19_info[41].fileTimeHi = 0U;
  c19_info[41].mFileTimeLo = 0U;
  c19_info[41].mFileTimeHi = 0U;
  c19_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[42].name = "min";
  c19_info[42].dominantType = "coder.internal.indexInt";
  c19_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c19_info[42].fileTimeLo = 1311255318U;
  c19_info[42].fileTimeHi = 0U;
  c19_info[42].mFileTimeLo = 0U;
  c19_info[42].mFileTimeHi = 0U;
  c19_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c19_info[43].name = "eml_scalar_eg";
  c19_info[43].dominantType = "coder.internal.indexInt";
  c19_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[43].fileTimeLo = 1286818796U;
  c19_info[43].fileTimeHi = 0U;
  c19_info[43].mFileTimeLo = 0U;
  c19_info[43].mFileTimeHi = 0U;
  c19_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c19_info[44].name = "eml_scalexp_alloc";
  c19_info[44].dominantType = "coder.internal.indexInt";
  c19_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c19_info[44].fileTimeLo = 1352424860U;
  c19_info[44].fileTimeHi = 0U;
  c19_info[44].mFileTimeLo = 0U;
  c19_info[44].mFileTimeHi = 0U;
  c19_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c19_info[45].name = "eml_scalar_eg";
  c19_info[45].dominantType = "coder.internal.indexInt";
  c19_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[45].fileTimeLo = 1286818796U;
  c19_info[45].fileTimeHi = 0U;
  c19_info[45].mFileTimeLo = 0U;
  c19_info[45].mFileTimeHi = 0U;
  c19_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[46].name = "colon";
  c19_info[46].dominantType = "double";
  c19_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c19_info[46].fileTimeLo = 1348191928U;
  c19_info[46].fileTimeHi = 0U;
  c19_info[46].mFileTimeLo = 0U;
  c19_info[46].mFileTimeHi = 0U;
  c19_info[47].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c19_info[47].name = "colon";
  c19_info[47].dominantType = "double";
  c19_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c19_info[47].fileTimeLo = 1348191928U;
  c19_info[47].fileTimeHi = 0U;
  c19_info[47].mFileTimeLo = 0U;
  c19_info[47].mFileTimeHi = 0U;
  c19_info[48].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c19_info[48].name = "floor";
  c19_info[48].dominantType = "double";
  c19_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c19_info[48].fileTimeLo = 1343830380U;
  c19_info[48].fileTimeHi = 0U;
  c19_info[48].mFileTimeLo = 0U;
  c19_info[48].mFileTimeHi = 0U;
  c19_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c19_info[49].name = "eml_scalar_floor";
  c19_info[49].dominantType = "double";
  c19_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c19_info[49].fileTimeLo = 1286818726U;
  c19_info[49].fileTimeHi = 0U;
  c19_info[49].mFileTimeLo = 0U;
  c19_info[49].mFileTimeHi = 0U;
  c19_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c19_info[50].name = "intmin";
  c19_info[50].dominantType = "char";
  c19_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c19_info[50].fileTimeLo = 1311255318U;
  c19_info[50].fileTimeHi = 0U;
  c19_info[50].mFileTimeLo = 0U;
  c19_info[50].mFileTimeHi = 0U;
  c19_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c19_info[51].name = "intmax";
  c19_info[51].dominantType = "char";
  c19_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c19_info[51].fileTimeLo = 1311255316U;
  c19_info[51].fileTimeHi = 0U;
  c19_info[51].mFileTimeLo = 0U;
  c19_info[51].mFileTimeHi = 0U;
  c19_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c19_info[52].name = "intmin";
  c19_info[52].dominantType = "char";
  c19_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c19_info[52].fileTimeLo = 1311255318U;
  c19_info[52].fileTimeHi = 0U;
  c19_info[52].mFileTimeLo = 0U;
  c19_info[52].mFileTimeHi = 0U;
  c19_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c19_info[53].name = "intmax";
  c19_info[53].dominantType = "char";
  c19_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c19_info[53].fileTimeLo = 1311255316U;
  c19_info[53].fileTimeHi = 0U;
  c19_info[53].mFileTimeLo = 0U;
  c19_info[53].mFileTimeHi = 0U;
  c19_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c19_info[54].name = "eml_isa_uint";
  c19_info[54].dominantType = "coder.internal.indexInt";
  c19_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c19_info[54].fileTimeLo = 1286818784U;
  c19_info[54].fileTimeHi = 0U;
  c19_info[54].mFileTimeLo = 0U;
  c19_info[54].mFileTimeHi = 0U;
  c19_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c19_info[55].name = "eml_unsigned_class";
  c19_info[55].dominantType = "char";
  c19_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c19_info[55].fileTimeLo = 1323170580U;
  c19_info[55].fileTimeHi = 0U;
  c19_info[55].mFileTimeLo = 0U;
  c19_info[55].mFileTimeHi = 0U;
  c19_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c19_info[56].name = "eml_index_class";
  c19_info[56].dominantType = "";
  c19_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[56].fileTimeLo = 1323170578U;
  c19_info[56].fileTimeHi = 0U;
  c19_info[56].mFileTimeLo = 0U;
  c19_info[56].mFileTimeHi = 0U;
  c19_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c19_info[57].name = "eml_index_class";
  c19_info[57].dominantType = "";
  c19_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[57].fileTimeLo = 1323170578U;
  c19_info[57].fileTimeHi = 0U;
  c19_info[57].mFileTimeLo = 0U;
  c19_info[57].mFileTimeHi = 0U;
  c19_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c19_info[58].name = "intmax";
  c19_info[58].dominantType = "char";
  c19_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c19_info[58].fileTimeLo = 1311255316U;
  c19_info[58].fileTimeHi = 0U;
  c19_info[58].mFileTimeLo = 0U;
  c19_info[58].mFileTimeHi = 0U;
  c19_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c19_info[59].name = "eml_isa_uint";
  c19_info[59].dominantType = "coder.internal.indexInt";
  c19_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c19_info[59].fileTimeLo = 1286818784U;
  c19_info[59].fileTimeHi = 0U;
  c19_info[59].mFileTimeLo = 0U;
  c19_info[59].mFileTimeHi = 0U;
  c19_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c19_info[60].name = "eml_index_plus";
  c19_info[60].dominantType = "double";
  c19_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[60].fileTimeLo = 1286818778U;
  c19_info[60].fileTimeHi = 0U;
  c19_info[60].mFileTimeLo = 0U;
  c19_info[60].mFileTimeHi = 0U;
  c19_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[61].name = "eml_index_class";
  c19_info[61].dominantType = "";
  c19_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[61].fileTimeLo = 1323170578U;
  c19_info[61].fileTimeHi = 0U;
  c19_info[61].mFileTimeLo = 0U;
  c19_info[61].mFileTimeHi = 0U;
  c19_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c19_info[62].name = "eml_int_forloop_overflow_check";
  c19_info[62].dominantType = "";
  c19_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c19_info[62].fileTimeLo = 1346510340U;
  c19_info[62].fileTimeHi = 0U;
  c19_info[62].mFileTimeLo = 0U;
  c19_info[62].mFileTimeHi = 0U;
  c19_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[63].name = "eml_index_class";
  c19_info[63].dominantType = "";
  c19_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[63].fileTimeLo = 1323170578U;
  c19_info[63].fileTimeHi = 0U;
  c19_info[63].mFileTimeLo = 0U;
  c19_info[63].mFileTimeHi = 0U;
}

static void c19_b_info_helper(c19_ResolvedFunctionInfo c19_info[128])
{
  c19_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[64].name = "eml_index_plus";
  c19_info[64].dominantType = "double";
  c19_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[64].fileTimeLo = 1286818778U;
  c19_info[64].fileTimeHi = 0U;
  c19_info[64].mFileTimeLo = 0U;
  c19_info[64].mFileTimeHi = 0U;
  c19_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[65].name = "eml_int_forloop_overflow_check";
  c19_info[65].dominantType = "";
  c19_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c19_info[65].fileTimeLo = 1346510340U;
  c19_info[65].fileTimeHi = 0U;
  c19_info[65].mFileTimeLo = 0U;
  c19_info[65].mFileTimeHi = 0U;
  c19_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[66].name = "eml_index_minus";
  c19_info[66].dominantType = "double";
  c19_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c19_info[66].fileTimeLo = 1286818778U;
  c19_info[66].fileTimeHi = 0U;
  c19_info[66].mFileTimeLo = 0U;
  c19_info[66].mFileTimeHi = 0U;
  c19_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c19_info[67].name = "eml_index_class";
  c19_info[67].dominantType = "";
  c19_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[67].fileTimeLo = 1323170578U;
  c19_info[67].fileTimeHi = 0U;
  c19_info[67].mFileTimeLo = 0U;
  c19_info[67].mFileTimeHi = 0U;
  c19_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[68].name = "eml_index_minus";
  c19_info[68].dominantType = "coder.internal.indexInt";
  c19_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c19_info[68].fileTimeLo = 1286818778U;
  c19_info[68].fileTimeHi = 0U;
  c19_info[68].mFileTimeLo = 0U;
  c19_info[68].mFileTimeHi = 0U;
  c19_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[69].name = "eml_index_times";
  c19_info[69].dominantType = "coder.internal.indexInt";
  c19_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c19_info[69].fileTimeLo = 1286818780U;
  c19_info[69].fileTimeHi = 0U;
  c19_info[69].mFileTimeLo = 0U;
  c19_info[69].mFileTimeHi = 0U;
  c19_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c19_info[70].name = "eml_index_class";
  c19_info[70].dominantType = "";
  c19_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[70].fileTimeLo = 1323170578U;
  c19_info[70].fileTimeHi = 0U;
  c19_info[70].mFileTimeLo = 0U;
  c19_info[70].mFileTimeHi = 0U;
  c19_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[71].name = "eml_index_plus";
  c19_info[71].dominantType = "coder.internal.indexInt";
  c19_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[71].fileTimeLo = 1286818778U;
  c19_info[71].fileTimeHi = 0U;
  c19_info[71].mFileTimeLo = 0U;
  c19_info[71].mFileTimeHi = 0U;
  c19_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[72].name = "eml_ixamax";
  c19_info[72].dominantType = "double";
  c19_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c19_info[72].fileTimeLo = 1299076770U;
  c19_info[72].fileTimeHi = 0U;
  c19_info[72].mFileTimeLo = 0U;
  c19_info[72].mFileTimeHi = 0U;
  c19_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c19_info[73].name = "eml_blas_inline";
  c19_info[73].dominantType = "";
  c19_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c19_info[73].fileTimeLo = 1299076768U;
  c19_info[73].fileTimeHi = 0U;
  c19_info[73].mFileTimeLo = 0U;
  c19_info[73].mFileTimeHi = 0U;
  c19_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c19_info[74].name = "length";
  c19_info[74].dominantType = "double";
  c19_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c19_info[74].fileTimeLo = 1303146206U;
  c19_info[74].fileTimeHi = 0U;
  c19_info[74].mFileTimeLo = 0U;
  c19_info[74].mFileTimeHi = 0U;
  c19_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c19_info[75].name = "eml_index_class";
  c19_info[75].dominantType = "";
  c19_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[75].fileTimeLo = 1323170578U;
  c19_info[75].fileTimeHi = 0U;
  c19_info[75].mFileTimeLo = 0U;
  c19_info[75].mFileTimeHi = 0U;
  c19_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c19_info[76].name = "eml_index_class";
  c19_info[76].dominantType = "";
  c19_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[76].fileTimeLo = 1323170578U;
  c19_info[76].fileTimeHi = 0U;
  c19_info[76].mFileTimeLo = 0U;
  c19_info[76].mFileTimeHi = 0U;
  c19_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c19_info[77].name = "eml_refblas_ixamax";
  c19_info[77].dominantType = "double";
  c19_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c19_info[77].fileTimeLo = 1299076770U;
  c19_info[77].fileTimeHi = 0U;
  c19_info[77].mFileTimeLo = 0U;
  c19_info[77].mFileTimeHi = 0U;
  c19_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c19_info[78].name = "eml_index_class";
  c19_info[78].dominantType = "";
  c19_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[78].fileTimeLo = 1323170578U;
  c19_info[78].fileTimeHi = 0U;
  c19_info[78].mFileTimeLo = 0U;
  c19_info[78].mFileTimeHi = 0U;
  c19_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c19_info[79].name = "eml_xcabs1";
  c19_info[79].dominantType = "double";
  c19_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c19_info[79].fileTimeLo = 1286818706U;
  c19_info[79].fileTimeHi = 0U;
  c19_info[79].mFileTimeLo = 0U;
  c19_info[79].mFileTimeHi = 0U;
  c19_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c19_info[80].name = "abs";
  c19_info[80].dominantType = "double";
  c19_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c19_info[80].fileTimeLo = 1343830366U;
  c19_info[80].fileTimeHi = 0U;
  c19_info[80].mFileTimeLo = 0U;
  c19_info[80].mFileTimeHi = 0U;
  c19_info[81].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c19_info[81].name = "eml_scalar_abs";
  c19_info[81].dominantType = "double";
  c19_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c19_info[81].fileTimeLo = 1286818712U;
  c19_info[81].fileTimeHi = 0U;
  c19_info[81].mFileTimeLo = 0U;
  c19_info[81].mFileTimeHi = 0U;
  c19_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c19_info[82].name = "eml_int_forloop_overflow_check";
  c19_info[82].dominantType = "";
  c19_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c19_info[82].fileTimeLo = 1346510340U;
  c19_info[82].fileTimeHi = 0U;
  c19_info[82].mFileTimeLo = 0U;
  c19_info[82].mFileTimeHi = 0U;
  c19_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c19_info[83].name = "eml_index_plus";
  c19_info[83].dominantType = "coder.internal.indexInt";
  c19_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[83].fileTimeLo = 1286818778U;
  c19_info[83].fileTimeHi = 0U;
  c19_info[83].mFileTimeLo = 0U;
  c19_info[83].mFileTimeHi = 0U;
  c19_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[84].name = "eml_xswap";
  c19_info[84].dominantType = "double";
  c19_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c19_info[84].fileTimeLo = 1299076778U;
  c19_info[84].fileTimeHi = 0U;
  c19_info[84].mFileTimeLo = 0U;
  c19_info[84].mFileTimeHi = 0U;
  c19_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c19_info[85].name = "eml_blas_inline";
  c19_info[85].dominantType = "";
  c19_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c19_info[85].fileTimeLo = 1299076768U;
  c19_info[85].fileTimeHi = 0U;
  c19_info[85].mFileTimeLo = 0U;
  c19_info[85].mFileTimeHi = 0U;
  c19_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c19_info[86].name = "eml_index_class";
  c19_info[86].dominantType = "";
  c19_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[86].fileTimeLo = 1323170578U;
  c19_info[86].fileTimeHi = 0U;
  c19_info[86].mFileTimeLo = 0U;
  c19_info[86].mFileTimeHi = 0U;
  c19_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c19_info[87].name = "eml_refblas_xswap";
  c19_info[87].dominantType = "double";
  c19_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c19_info[87].fileTimeLo = 1299076786U;
  c19_info[87].fileTimeHi = 0U;
  c19_info[87].mFileTimeLo = 0U;
  c19_info[87].mFileTimeHi = 0U;
  c19_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c19_info[88].name = "eml_index_class";
  c19_info[88].dominantType = "";
  c19_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[88].fileTimeLo = 1323170578U;
  c19_info[88].fileTimeHi = 0U;
  c19_info[88].mFileTimeLo = 0U;
  c19_info[88].mFileTimeHi = 0U;
  c19_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c19_info[89].name = "abs";
  c19_info[89].dominantType = "coder.internal.indexInt";
  c19_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c19_info[89].fileTimeLo = 1343830366U;
  c19_info[89].fileTimeHi = 0U;
  c19_info[89].mFileTimeLo = 0U;
  c19_info[89].mFileTimeHi = 0U;
  c19_info[90].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c19_info[90].name = "eml_scalar_abs";
  c19_info[90].dominantType = "coder.internal.indexInt";
  c19_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c19_info[90].fileTimeLo = 1286818712U;
  c19_info[90].fileTimeHi = 0U;
  c19_info[90].mFileTimeLo = 0U;
  c19_info[90].mFileTimeHi = 0U;
  c19_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c19_info[91].name = "eml_int_forloop_overflow_check";
  c19_info[91].dominantType = "";
  c19_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c19_info[91].fileTimeLo = 1346510340U;
  c19_info[91].fileTimeHi = 0U;
  c19_info[91].mFileTimeLo = 0U;
  c19_info[91].mFileTimeHi = 0U;
  c19_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c19_info[92].name = "eml_index_plus";
  c19_info[92].dominantType = "coder.internal.indexInt";
  c19_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[92].fileTimeLo = 1286818778U;
  c19_info[92].fileTimeHi = 0U;
  c19_info[92].mFileTimeLo = 0U;
  c19_info[92].mFileTimeHi = 0U;
  c19_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[93].name = "eml_div";
  c19_info[93].dominantType = "double";
  c19_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c19_info[93].fileTimeLo = 1313347810U;
  c19_info[93].fileTimeHi = 0U;
  c19_info[93].mFileTimeLo = 0U;
  c19_info[93].mFileTimeHi = 0U;
  c19_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c19_info[94].name = "eml_xgeru";
  c19_info[94].dominantType = "double";
  c19_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c19_info[94].fileTimeLo = 1299076774U;
  c19_info[94].fileTimeHi = 0U;
  c19_info[94].mFileTimeLo = 0U;
  c19_info[94].mFileTimeHi = 0U;
  c19_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c19_info[95].name = "eml_blas_inline";
  c19_info[95].dominantType = "";
  c19_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c19_info[95].fileTimeLo = 1299076768U;
  c19_info[95].fileTimeHi = 0U;
  c19_info[95].mFileTimeLo = 0U;
  c19_info[95].mFileTimeHi = 0U;
  c19_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c19_info[96].name = "eml_xger";
  c19_info[96].dominantType = "double";
  c19_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c19_info[96].fileTimeLo = 1299076774U;
  c19_info[96].fileTimeHi = 0U;
  c19_info[96].mFileTimeLo = 0U;
  c19_info[96].mFileTimeHi = 0U;
  c19_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c19_info[97].name = "eml_blas_inline";
  c19_info[97].dominantType = "";
  c19_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c19_info[97].fileTimeLo = 1299076768U;
  c19_info[97].fileTimeHi = 0U;
  c19_info[97].mFileTimeLo = 0U;
  c19_info[97].mFileTimeHi = 0U;
  c19_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c19_info[98].name = "intmax";
  c19_info[98].dominantType = "char";
  c19_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c19_info[98].fileTimeLo = 1311255316U;
  c19_info[98].fileTimeHi = 0U;
  c19_info[98].mFileTimeLo = 0U;
  c19_info[98].mFileTimeHi = 0U;
  c19_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c19_info[99].name = "min";
  c19_info[99].dominantType = "double";
  c19_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c19_info[99].fileTimeLo = 1311255318U;
  c19_info[99].fileTimeHi = 0U;
  c19_info[99].mFileTimeLo = 0U;
  c19_info[99].mFileTimeHi = 0U;
  c19_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c19_info[100].name = "mtimes";
  c19_info[100].dominantType = "double";
  c19_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[100].fileTimeLo = 1289519692U;
  c19_info[100].fileTimeHi = 0U;
  c19_info[100].mFileTimeLo = 0U;
  c19_info[100].mFileTimeHi = 0U;
  c19_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c19_info[101].name = "eml_index_class";
  c19_info[101].dominantType = "";
  c19_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[101].fileTimeLo = 1323170578U;
  c19_info[101].fileTimeHi = 0U;
  c19_info[101].mFileTimeLo = 0U;
  c19_info[101].mFileTimeHi = 0U;
  c19_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c19_info[102].name = "eml_refblas_xger";
  c19_info[102].dominantType = "double";
  c19_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c19_info[102].fileTimeLo = 1299076776U;
  c19_info[102].fileTimeHi = 0U;
  c19_info[102].mFileTimeLo = 0U;
  c19_info[102].mFileTimeHi = 0U;
  c19_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c19_info[103].name = "eml_refblas_xgerx";
  c19_info[103].dominantType = "char";
  c19_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c19_info[103].fileTimeLo = 1299076778U;
  c19_info[103].fileTimeHi = 0U;
  c19_info[103].mFileTimeLo = 0U;
  c19_info[103].mFileTimeHi = 0U;
  c19_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c19_info[104].name = "eml_index_class";
  c19_info[104].dominantType = "";
  c19_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[104].fileTimeLo = 1323170578U;
  c19_info[104].fileTimeHi = 0U;
  c19_info[104].mFileTimeLo = 0U;
  c19_info[104].mFileTimeHi = 0U;
  c19_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c19_info[105].name = "abs";
  c19_info[105].dominantType = "coder.internal.indexInt";
  c19_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c19_info[105].fileTimeLo = 1343830366U;
  c19_info[105].fileTimeHi = 0U;
  c19_info[105].mFileTimeLo = 0U;
  c19_info[105].mFileTimeHi = 0U;
  c19_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c19_info[106].name = "eml_index_minus";
  c19_info[106].dominantType = "double";
  c19_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c19_info[106].fileTimeLo = 1286818778U;
  c19_info[106].fileTimeHi = 0U;
  c19_info[106].mFileTimeLo = 0U;
  c19_info[106].mFileTimeHi = 0U;
  c19_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c19_info[107].name = "eml_int_forloop_overflow_check";
  c19_info[107].dominantType = "";
  c19_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c19_info[107].fileTimeLo = 1346510340U;
  c19_info[107].fileTimeHi = 0U;
  c19_info[107].mFileTimeLo = 0U;
  c19_info[107].mFileTimeHi = 0U;
  c19_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c19_info[108].name = "eml_index_plus";
  c19_info[108].dominantType = "double";
  c19_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[108].fileTimeLo = 1286818778U;
  c19_info[108].fileTimeHi = 0U;
  c19_info[108].mFileTimeLo = 0U;
  c19_info[108].mFileTimeHi = 0U;
  c19_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c19_info[109].name = "eml_index_plus";
  c19_info[109].dominantType = "coder.internal.indexInt";
  c19_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[109].fileTimeLo = 1286818778U;
  c19_info[109].fileTimeHi = 0U;
  c19_info[109].mFileTimeLo = 0U;
  c19_info[109].mFileTimeHi = 0U;
  c19_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c19_info[110].name = "eml_warning";
  c19_info[110].dominantType = "char";
  c19_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c19_info[110].fileTimeLo = 1286818802U;
  c19_info[110].fileTimeHi = 0U;
  c19_info[110].mFileTimeLo = 0U;
  c19_info[110].mFileTimeHi = 0U;
  c19_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c19_info[111].name = "eml_scalar_eg";
  c19_info[111].dominantType = "double";
  c19_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[111].fileTimeLo = 1286818796U;
  c19_info[111].fileTimeHi = 0U;
  c19_info[111].mFileTimeLo = 0U;
  c19_info[111].mFileTimeHi = 0U;
  c19_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c19_info[112].name = "eml_int_forloop_overflow_check";
  c19_info[112].dominantType = "";
  c19_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c19_info[112].fileTimeLo = 1346510340U;
  c19_info[112].fileTimeHi = 0U;
  c19_info[112].mFileTimeLo = 0U;
  c19_info[112].mFileTimeHi = 0U;
  c19_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c19_info[113].name = "eml_xtrsm";
  c19_info[113].dominantType = "char";
  c19_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c19_info[113].fileTimeLo = 1299076778U;
  c19_info[113].fileTimeHi = 0U;
  c19_info[113].mFileTimeLo = 0U;
  c19_info[113].mFileTimeHi = 0U;
  c19_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c19_info[114].name = "eml_blas_inline";
  c19_info[114].dominantType = "";
  c19_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c19_info[114].fileTimeLo = 1299076768U;
  c19_info[114].fileTimeHi = 0U;
  c19_info[114].mFileTimeLo = 0U;
  c19_info[114].mFileTimeHi = 0U;
  c19_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c19_info[115].name = "mtimes";
  c19_info[115].dominantType = "double";
  c19_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c19_info[115].fileTimeLo = 1289519692U;
  c19_info[115].fileTimeHi = 0U;
  c19_info[115].mFileTimeLo = 0U;
  c19_info[115].mFileTimeHi = 0U;
  c19_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c19_info[116].name = "eml_index_class";
  c19_info[116].dominantType = "";
  c19_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[116].fileTimeLo = 1323170578U;
  c19_info[116].fileTimeHi = 0U;
  c19_info[116].mFileTimeLo = 0U;
  c19_info[116].mFileTimeHi = 0U;
  c19_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c19_info[117].name = "eml_scalar_eg";
  c19_info[117].dominantType = "double";
  c19_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[117].fileTimeLo = 1286818796U;
  c19_info[117].fileTimeHi = 0U;
  c19_info[117].mFileTimeLo = 0U;
  c19_info[117].mFileTimeHi = 0U;
  c19_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c19_info[118].name = "eml_refblas_xtrsm";
  c19_info[118].dominantType = "char";
  c19_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[118].fileTimeLo = 1299076786U;
  c19_info[118].fileTimeHi = 0U;
  c19_info[118].mFileTimeLo = 0U;
  c19_info[118].mFileTimeHi = 0U;
  c19_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[119].name = "eml_scalar_eg";
  c19_info[119].dominantType = "double";
  c19_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c19_info[119].fileTimeLo = 1286818796U;
  c19_info[119].fileTimeHi = 0U;
  c19_info[119].mFileTimeLo = 0U;
  c19_info[119].mFileTimeHi = 0U;
  c19_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[120].name = "eml_index_minus";
  c19_info[120].dominantType = "double";
  c19_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c19_info[120].fileTimeLo = 1286818778U;
  c19_info[120].fileTimeHi = 0U;
  c19_info[120].mFileTimeLo = 0U;
  c19_info[120].mFileTimeHi = 0U;
  c19_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[121].name = "eml_index_class";
  c19_info[121].dominantType = "";
  c19_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c19_info[121].fileTimeLo = 1323170578U;
  c19_info[121].fileTimeHi = 0U;
  c19_info[121].mFileTimeLo = 0U;
  c19_info[121].mFileTimeHi = 0U;
  c19_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[122].name = "eml_int_forloop_overflow_check";
  c19_info[122].dominantType = "";
  c19_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c19_info[122].fileTimeLo = 1346510340U;
  c19_info[122].fileTimeHi = 0U;
  c19_info[122].mFileTimeLo = 0U;
  c19_info[122].mFileTimeHi = 0U;
  c19_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[123].name = "eml_index_times";
  c19_info[123].dominantType = "coder.internal.indexInt";
  c19_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c19_info[123].fileTimeLo = 1286818780U;
  c19_info[123].fileTimeHi = 0U;
  c19_info[123].mFileTimeLo = 0U;
  c19_info[123].mFileTimeHi = 0U;
  c19_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[124].name = "eml_index_plus";
  c19_info[124].dominantType = "coder.internal.indexInt";
  c19_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[124].fileTimeLo = 1286818778U;
  c19_info[124].fileTimeHi = 0U;
  c19_info[124].mFileTimeLo = 0U;
  c19_info[124].mFileTimeHi = 0U;
  c19_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[125].name = "eml_index_plus";
  c19_info[125].dominantType = "double";
  c19_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c19_info[125].fileTimeLo = 1286818778U;
  c19_info[125].fileTimeHi = 0U;
  c19_info[125].mFileTimeLo = 0U;
  c19_info[125].mFileTimeHi = 0U;
  c19_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c19_info[126].name = "intmin";
  c19_info[126].dominantType = "char";
  c19_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c19_info[126].fileTimeLo = 1311255318U;
  c19_info[126].fileTimeHi = 0U;
  c19_info[126].mFileTimeLo = 0U;
  c19_info[126].mFileTimeHi = 0U;
  c19_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c19_info[127].name = "eml_div";
  c19_info[127].dominantType = "double";
  c19_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c19_info[127].fileTimeLo = 1313347810U;
  c19_info[127].fileTimeHi = 0U;
  c19_info[127].mFileTimeLo = 0U;
  c19_info[127].mFileTimeHi = 0U;
}

static void c19_eml_scalar_eg(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c19_eml_xgemm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_B[72], real_T c19_C[144], real_T
  c19_b_C[144])
{
  int32_T c19_i105;
  int32_T c19_i106;
  real_T c19_b_A[72];
  int32_T c19_i107;
  real_T c19_b_B[72];
  for (c19_i105 = 0; c19_i105 < 144; c19_i105++) {
    c19_b_C[c19_i105] = c19_C[c19_i105];
  }

  for (c19_i106 = 0; c19_i106 < 72; c19_i106++) {
    c19_b_A[c19_i106] = c19_A[c19_i106];
  }

  for (c19_i107 = 0; c19_i107 < 72; c19_i107++) {
    c19_b_B[c19_i107] = c19_B[c19_i107];
  }

  c19_c_eml_xgemm(chartInstance, c19_b_A, c19_b_B, c19_b_C);
}

static void c19_mrdivide(SFc19_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c19_A[72], real_T c19_B[144], real_T c19_y[72])
{
  int32_T c19_i108;
  int32_T c19_i109;
  int32_T c19_i110;
  int32_T c19_i111;
  real_T c19_b_A[144];
  int32_T c19_i112;
  int32_T c19_i113;
  int32_T c19_i114;
  int32_T c19_i115;
  real_T c19_b_B[72];
  int32_T c19_info;
  int32_T c19_ipiv[12];
  int32_T c19_b_info;
  int32_T c19_c_info;
  int32_T c19_d_info;
  int32_T c19_i;
  int32_T c19_b_i;
  int32_T c19_ip;
  int32_T c19_j;
  int32_T c19_b_j;
  real_T c19_temp;
  int32_T c19_i116;
  real_T c19_c_A[144];
  int32_T c19_i117;
  real_T c19_d_A[144];
  int32_T c19_i118;
  int32_T c19_i119;
  int32_T c19_i120;
  int32_T c19_i121;
  c19_i108 = 0;
  for (c19_i109 = 0; c19_i109 < 12; c19_i109++) {
    c19_i110 = 0;
    for (c19_i111 = 0; c19_i111 < 12; c19_i111++) {
      c19_b_A[c19_i111 + c19_i108] = c19_B[c19_i110 + c19_i109];
      c19_i110 += 12;
    }

    c19_i108 += 12;
  }

  c19_i112 = 0;
  for (c19_i113 = 0; c19_i113 < 6; c19_i113++) {
    c19_i114 = 0;
    for (c19_i115 = 0; c19_i115 < 12; c19_i115++) {
      c19_b_B[c19_i115 + c19_i112] = c19_A[c19_i114 + c19_i113];
      c19_i114 += 6;
    }

    c19_i112 += 12;
  }

  c19_b_eml_matlab_zgetrf(chartInstance, c19_b_A, c19_ipiv, &c19_info);
  c19_b_info = c19_info;
  c19_c_info = c19_b_info;
  c19_d_info = c19_c_info;
  if (c19_d_info > 0) {
    c19_eml_warning(chartInstance);
  }

  for (c19_i = 1; c19_i < 13; c19_i++) {
    c19_b_i = c19_i;
    if (c19_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_i), 1, 12, 1, 0) - 1] != c19_b_i) {
      c19_ip = c19_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c19_b_i), 1, 12, 1, 0) - 1];
      for (c19_j = 1; c19_j < 7; c19_j++) {
        c19_b_j = c19_j;
        c19_temp = c19_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_b_i), 1, 12, 1, 0) + 12 *
                            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_b_j), 1, 6, 2, 0) - 1)) - 1];
        c19_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c19_b_i), 1, 12, 1, 0) + 12 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c19_b_j), 1, 6, 2, 0) - 1)) - 1] = c19_b_B
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c19_ip), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c19_b_j), 1, 6, 2, 0)
             - 1)) - 1];
        c19_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c19_ip), 1, 12, 1, 0) + 12 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c19_b_j), 1, 6, 2, 0) - 1)) - 1] = c19_temp;
      }
    }
  }

  for (c19_i116 = 0; c19_i116 < 144; c19_i116++) {
    c19_c_A[c19_i116] = c19_b_A[c19_i116];
  }

  c19_c_eml_xtrsm(chartInstance, c19_c_A, c19_b_B);
  for (c19_i117 = 0; c19_i117 < 144; c19_i117++) {
    c19_d_A[c19_i117] = c19_b_A[c19_i117];
  }

  c19_d_eml_xtrsm(chartInstance, c19_d_A, c19_b_B);
  c19_i118 = 0;
  for (c19_i119 = 0; c19_i119 < 12; c19_i119++) {
    c19_i120 = 0;
    for (c19_i121 = 0; c19_i121 < 6; c19_i121++) {
      c19_y[c19_i121 + c19_i118] = c19_b_B[c19_i120 + c19_i119];
      c19_i120 += 12;
    }

    c19_i118 += 6;
  }
}

static void c19_realmin(SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c19_eps(SFc19_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c19_eml_matlab_zgetrf(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_b_A[144], int32_T c19_ipiv[12],
  int32_T *c19_info)
{
  int32_T c19_i122;
  for (c19_i122 = 0; c19_i122 < 144; c19_i122++) {
    c19_b_A[c19_i122] = c19_A[c19_i122];
  }

  c19_b_eml_matlab_zgetrf(chartInstance, c19_b_A, c19_ipiv, c19_info);
}

static void c19_check_forloop_overflow_error
  (SFc19_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c19_overflow)
{
  int32_T c19_i123;
  static char_T c19_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c19_u[34];
  const mxArray *c19_y = NULL;
  int32_T c19_i124;
  static char_T c19_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c19_b_u[23];
  const mxArray *c19_b_y = NULL;
  if (!c19_overflow) {
  } else {
    for (c19_i123 = 0; c19_i123 < 34; c19_i123++) {
      c19_u[c19_i123] = c19_cv0[c19_i123];
    }

    c19_y = NULL;
    sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c19_i124 = 0; c19_i124 < 23; c19_i124++) {
      c19_b_u[c19_i124] = c19_cv1[c19_i124];
    }

    c19_b_y = NULL;
    sf_mex_assign(&c19_b_y, sf_mex_create("y", c19_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c19_y, 14, c19_b_y));
  }
}

static void c19_eml_xger(SFc19_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c19_m, int32_T c19_n, real_T c19_alpha1, int32_T c19_ix0, int32_T
  c19_iy0, real_T c19_A[144], int32_T c19_ia0, real_T c19_b_A[144])
{
  int32_T c19_i125;
  for (c19_i125 = 0; c19_i125 < 144; c19_i125++) {
    c19_b_A[c19_i125] = c19_A[c19_i125];
  }

  c19_b_eml_xger(chartInstance, c19_m, c19_n, c19_alpha1, c19_ix0, c19_iy0,
                 c19_b_A, c19_ia0);
}

static void c19_eml_warning(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c19_i126;
  static char_T c19_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c19_u[27];
  const mxArray *c19_y = NULL;
  for (c19_i126 = 0; c19_i126 < 27; c19_i126++) {
    c19_u[c19_i126] = c19_varargin_1[c19_i126];
  }

  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", c19_u, 10, 0U, 1U, 0U, 2, 1, 27),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c19_y));
}

static void c19_eml_xtrsm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_B[72], real_T c19_b_B[72])
{
  int32_T c19_i127;
  int32_T c19_i128;
  real_T c19_b_A[144];
  for (c19_i127 = 0; c19_i127 < 72; c19_i127++) {
    c19_b_B[c19_i127] = c19_B[c19_i127];
  }

  for (c19_i128 = 0; c19_i128 < 144; c19_i128++) {
    c19_b_A[c19_i128] = c19_A[c19_i128];
  }

  c19_c_eml_xtrsm(chartInstance, c19_b_A, c19_b_B);
}

static void c19_below_threshold(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c19_b_eml_xtrsm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_B[72], real_T c19_b_B[72])
{
  int32_T c19_i129;
  int32_T c19_i130;
  real_T c19_b_A[144];
  for (c19_i129 = 0; c19_i129 < 72; c19_i129++) {
    c19_b_B[c19_i129] = c19_B[c19_i129];
  }

  for (c19_i130 = 0; c19_i130 < 144; c19_i130++) {
    c19_b_A[c19_i130] = c19_A[c19_i130];
  }

  c19_d_eml_xtrsm(chartInstance, c19_b_A, c19_b_B);
}

static void c19_b_eml_scalar_eg(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c19_b_eml_xgemm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_B[276], real_T c19_C[138], real_T
  c19_b_C[138])
{
  int32_T c19_i131;
  int32_T c19_i132;
  real_T c19_b_A[72];
  int32_T c19_i133;
  real_T c19_b_B[276];
  for (c19_i131 = 0; c19_i131 < 138; c19_i131++) {
    c19_b_C[c19_i131] = c19_C[c19_i131];
  }

  for (c19_i132 = 0; c19_i132 < 72; c19_i132++) {
    c19_b_A[c19_i132] = c19_A[c19_i132];
  }

  for (c19_i133 = 0; c19_i133 < 276; c19_i133++) {
    c19_b_B[c19_i133] = c19_B[c19_i133];
  }

  c19_d_eml_xgemm(chartInstance, c19_b_A, c19_b_B, c19_b_C);
}

static void c19_c_eml_scalar_eg(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static const mxArray *c19_j_sf_marshallOut(void *chartInstanceVoid, void
  *c19_inData)
{
  const mxArray *c19_mxArrayOutData = NULL;
  int32_T c19_u;
  const mxArray *c19_y = NULL;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_mxArrayOutData = NULL;
  c19_u = *(int32_T *)c19_inData;
  c19_y = NULL;
  sf_mex_assign(&c19_y, sf_mex_create("y", &c19_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c19_mxArrayOutData, c19_y, FALSE);
  return c19_mxArrayOutData;
}

static int32_T c19_h_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  int32_T c19_y;
  int32_T c19_i134;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_i134, 1, 6, 0U, 0, 0U, 0);
  c19_y = c19_i134;
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c19_mxArrayInData, const char_T *c19_varName, void *c19_outData)
{
  const mxArray *c19_b_sfEvent;
  const char_T *c19_identifier;
  emlrtMsgIdentifier c19_thisId;
  int32_T c19_y;
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c19_b_sfEvent = sf_mex_dup(c19_mxArrayInData);
  c19_identifier = c19_varName;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c19_b_sfEvent),
    &c19_thisId);
  sf_mex_destroy(&c19_b_sfEvent);
  *(int32_T *)c19_outData = c19_y;
  sf_mex_destroy(&c19_mxArrayInData);
}

static uint8_T c19_i_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_b_is_active_c19_torqueBalancing2012b, const
  char_T *c19_identifier)
{
  uint8_T c19_y;
  emlrtMsgIdentifier c19_thisId;
  c19_thisId.fIdentifier = c19_identifier;
  c19_thisId.fParent = NULL;
  c19_y = c19_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c19_b_is_active_c19_torqueBalancing2012b), &c19_thisId);
  sf_mex_destroy(&c19_b_is_active_c19_torqueBalancing2012b);
  return c19_y;
}

static uint8_T c19_j_emlrt_marshallIn(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c19_u, const emlrtMsgIdentifier *c19_parentId)
{
  uint8_T c19_y;
  uint8_T c19_u0;
  sf_mex_import(c19_parentId, sf_mex_dup(c19_u), &c19_u0, 1, 3, 0U, 0, 0U, 0);
  c19_y = c19_u0;
  sf_mex_destroy(&c19_u);
  return c19_y;
}

static void c19_c_eml_xgemm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_B[72], real_T c19_C[144])
{
  real_T c19_alpha1;
  real_T c19_beta1;
  char_T c19_TRANSB;
  char_T c19_TRANSA;
  ptrdiff_t c19_m_t;
  ptrdiff_t c19_n_t;
  ptrdiff_t c19_k_t;
  ptrdiff_t c19_lda_t;
  ptrdiff_t c19_ldb_t;
  ptrdiff_t c19_ldc_t;
  double * c19_alpha1_t;
  double * c19_Aia0_t;
  double * c19_Bib0_t;
  double * c19_beta1_t;
  double * c19_Cic0_t;
  c19_alpha1 = 1.0;
  c19_beta1 = 0.0;
  c19_TRANSB = 'N';
  c19_TRANSA = 'N';
  c19_m_t = (ptrdiff_t)(12);
  c19_n_t = (ptrdiff_t)(12);
  c19_k_t = (ptrdiff_t)(6);
  c19_lda_t = (ptrdiff_t)(12);
  c19_ldb_t = (ptrdiff_t)(6);
  c19_ldc_t = (ptrdiff_t)(12);
  c19_alpha1_t = (double *)(&c19_alpha1);
  c19_Aia0_t = (double *)(&c19_A[0]);
  c19_Bib0_t = (double *)(&c19_B[0]);
  c19_beta1_t = (double *)(&c19_beta1);
  c19_Cic0_t = (double *)(&c19_C[0]);
  dgemm(&c19_TRANSA, &c19_TRANSB, &c19_m_t, &c19_n_t, &c19_k_t, c19_alpha1_t,
        c19_Aia0_t, &c19_lda_t, c19_Bib0_t, &c19_ldb_t, c19_beta1_t, c19_Cic0_t,
        &c19_ldc_t);
}

static void c19_b_eml_matlab_zgetrf(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], int32_T c19_ipiv[12], int32_T *c19_info)
{
  int32_T c19_i135;
  int32_T c19_j;
  int32_T c19_b_j;
  int32_T c19_a;
  int32_T c19_jm1;
  int32_T c19_b;
  int32_T c19_mmj;
  int32_T c19_b_a;
  int32_T c19_c;
  int32_T c19_b_b;
  int32_T c19_jj;
  int32_T c19_c_a;
  int32_T c19_jp1j;
  int32_T c19_d_a;
  int32_T c19_b_c;
  int32_T c19_n;
  int32_T c19_ix0;
  int32_T c19_b_n;
  int32_T c19_b_ix0;
  int32_T c19_c_n;
  int32_T c19_c_ix0;
  int32_T c19_idxmax;
  int32_T c19_ix;
  real_T c19_x;
  real_T c19_b_x;
  real_T c19_c_x;
  real_T c19_y;
  real_T c19_d_x;
  real_T c19_e_x;
  real_T c19_b_y;
  real_T c19_smax;
  int32_T c19_d_n;
  int32_T c19_c_b;
  int32_T c19_d_b;
  boolean_T c19_overflow;
  int32_T c19_k;
  int32_T c19_b_k;
  int32_T c19_e_a;
  real_T c19_f_x;
  real_T c19_g_x;
  real_T c19_h_x;
  real_T c19_c_y;
  real_T c19_i_x;
  real_T c19_j_x;
  real_T c19_d_y;
  real_T c19_s;
  int32_T c19_f_a;
  int32_T c19_jpiv_offset;
  int32_T c19_g_a;
  int32_T c19_e_b;
  int32_T c19_jpiv;
  int32_T c19_h_a;
  int32_T c19_f_b;
  int32_T c19_c_c;
  int32_T c19_g_b;
  int32_T c19_jrow;
  int32_T c19_i_a;
  int32_T c19_h_b;
  int32_T c19_jprow;
  int32_T c19_d_ix0;
  int32_T c19_iy0;
  int32_T c19_e_ix0;
  int32_T c19_b_iy0;
  int32_T c19_f_ix0;
  int32_T c19_c_iy0;
  int32_T c19_b_ix;
  int32_T c19_iy;
  int32_T c19_c_k;
  real_T c19_temp;
  int32_T c19_j_a;
  int32_T c19_k_a;
  int32_T c19_b_jp1j;
  int32_T c19_l_a;
  int32_T c19_d_c;
  int32_T c19_m_a;
  int32_T c19_i_b;
  int32_T c19_i136;
  int32_T c19_n_a;
  int32_T c19_j_b;
  int32_T c19_o_a;
  int32_T c19_k_b;
  boolean_T c19_b_overflow;
  int32_T c19_i;
  int32_T c19_b_i;
  real_T c19_k_x;
  real_T c19_e_y;
  real_T c19_z;
  int32_T c19_l_b;
  int32_T c19_e_c;
  int32_T c19_p_a;
  int32_T c19_f_c;
  int32_T c19_q_a;
  int32_T c19_g_c;
  int32_T c19_m;
  int32_T c19_e_n;
  int32_T c19_g_ix0;
  int32_T c19_d_iy0;
  int32_T c19_ia0;
  real_T c19_d1;
  c19_realmin(chartInstance);
  c19_eps(chartInstance);
  for (c19_i135 = 0; c19_i135 < 12; c19_i135++) {
    c19_ipiv[c19_i135] = 1 + c19_i135;
  }

  *c19_info = 0;
  for (c19_j = 1; c19_j < 12; c19_j++) {
    c19_b_j = c19_j;
    c19_a = c19_b_j - 1;
    c19_jm1 = c19_a;
    c19_b = c19_b_j;
    c19_mmj = 12 - c19_b;
    c19_b_a = c19_jm1;
    c19_c = c19_b_a * 13;
    c19_b_b = c19_c + 1;
    c19_jj = c19_b_b;
    c19_c_a = c19_jj + 1;
    c19_jp1j = c19_c_a;
    c19_d_a = c19_mmj;
    c19_b_c = c19_d_a;
    c19_n = c19_b_c + 1;
    c19_ix0 = c19_jj;
    c19_b_n = c19_n;
    c19_b_ix0 = c19_ix0;
    c19_c_n = c19_b_n;
    c19_c_ix0 = c19_b_ix0;
    if (c19_c_n < 1) {
      c19_idxmax = 0;
    } else {
      c19_idxmax = 1;
      if (c19_c_n > 1) {
        c19_ix = c19_c_ix0;
        c19_x = c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_ix), 1, 144, 1, 0) - 1];
        c19_b_x = c19_x;
        c19_c_x = c19_b_x;
        c19_y = muDoubleScalarAbs(c19_c_x);
        c19_d_x = 0.0;
        c19_e_x = c19_d_x;
        c19_b_y = muDoubleScalarAbs(c19_e_x);
        c19_smax = c19_y + c19_b_y;
        c19_d_n = c19_c_n;
        c19_c_b = c19_d_n;
        c19_d_b = c19_c_b;
        if (2 > c19_d_b) {
          c19_overflow = FALSE;
        } else {
          c19_overflow = (c19_d_b > 2147483646);
        }

        if (c19_overflow) {
          c19_check_forloop_overflow_error(chartInstance, c19_overflow);
        }

        for (c19_k = 2; c19_k <= c19_d_n; c19_k++) {
          c19_b_k = c19_k;
          c19_e_a = c19_ix + 1;
          c19_ix = c19_e_a;
          c19_f_x = c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_ix), 1, 144, 1, 0) - 1];
          c19_g_x = c19_f_x;
          c19_h_x = c19_g_x;
          c19_c_y = muDoubleScalarAbs(c19_h_x);
          c19_i_x = 0.0;
          c19_j_x = c19_i_x;
          c19_d_y = muDoubleScalarAbs(c19_j_x);
          c19_s = c19_c_y + c19_d_y;
          if (c19_s > c19_smax) {
            c19_idxmax = c19_b_k;
            c19_smax = c19_s;
          }
        }
      }
    }

    c19_f_a = c19_idxmax - 1;
    c19_jpiv_offset = c19_f_a;
    c19_g_a = c19_jj;
    c19_e_b = c19_jpiv_offset;
    c19_jpiv = c19_g_a + c19_e_b;
    if (c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_jpiv), 1, 144, 1, 0) - 1] != 0.0) {
      if (c19_jpiv_offset != 0) {
        c19_h_a = c19_b_j;
        c19_f_b = c19_jpiv_offset;
        c19_c_c = c19_h_a + c19_f_b;
        c19_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_j), 1, 12, 1, 0) - 1] = c19_c_c;
        c19_g_b = c19_jm1 + 1;
        c19_jrow = c19_g_b;
        c19_i_a = c19_jrow;
        c19_h_b = c19_jpiv_offset;
        c19_jprow = c19_i_a + c19_h_b;
        c19_d_ix0 = c19_jrow;
        c19_iy0 = c19_jprow;
        c19_e_ix0 = c19_d_ix0;
        c19_b_iy0 = c19_iy0;
        c19_f_ix0 = c19_e_ix0;
        c19_c_iy0 = c19_b_iy0;
        c19_b_ix = c19_f_ix0;
        c19_iy = c19_c_iy0;
        for (c19_c_k = 1; c19_c_k < 13; c19_c_k++) {
          c19_temp = c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c19_b_ix), 1, 144, 1, 0) - 1];
          c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_ix), 1, 144, 1, 0) - 1] =
            c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_iy), 1, 144, 1, 0) - 1];
          c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_iy), 1, 144, 1, 0) - 1] = c19_temp;
          c19_j_a = c19_b_ix + 12;
          c19_b_ix = c19_j_a;
          c19_k_a = c19_iy + 12;
          c19_iy = c19_k_a;
        }
      }

      c19_b_jp1j = c19_jp1j;
      c19_l_a = c19_mmj;
      c19_d_c = c19_l_a;
      c19_m_a = c19_jp1j;
      c19_i_b = c19_d_c - 1;
      c19_i136 = c19_m_a + c19_i_b;
      c19_n_a = c19_b_jp1j;
      c19_j_b = c19_i136;
      c19_o_a = c19_n_a;
      c19_k_b = c19_j_b;
      if (c19_o_a > c19_k_b) {
        c19_b_overflow = FALSE;
      } else {
        c19_b_overflow = (c19_k_b > 2147483646);
      }

      if (c19_b_overflow) {
        c19_check_forloop_overflow_error(chartInstance, c19_b_overflow);
      }

      for (c19_i = c19_b_jp1j; c19_i <= c19_i136; c19_i++) {
        c19_b_i = c19_i;
        c19_k_x = c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_b_i), 1, 144, 1, 0) - 1];
        c19_e_y = c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c19_jj), 1, 144, 1, 0) - 1];
        c19_z = c19_k_x / c19_e_y;
        c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c19_b_i), 1, 144, 1, 0) - 1] = c19_z;
      }
    } else {
      *c19_info = c19_b_j;
    }

    c19_l_b = c19_b_j;
    c19_e_c = 12 - c19_l_b;
    c19_p_a = c19_jj;
    c19_f_c = c19_p_a;
    c19_q_a = c19_jj;
    c19_g_c = c19_q_a;
    c19_m = c19_mmj;
    c19_e_n = c19_e_c;
    c19_g_ix0 = c19_jp1j;
    c19_d_iy0 = c19_f_c + 12;
    c19_ia0 = c19_g_c + 13;
    c19_d1 = -1.0;
    c19_b_eml_xger(chartInstance, c19_m, c19_e_n, c19_d1, c19_g_ix0, c19_d_iy0,
                   c19_A, c19_ia0);
  }

  if (*c19_info == 0) {
    if (!(c19_A[143] != 0.0)) {
      *c19_info = 12;
    }
  }
}

static void c19_b_eml_xger(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c19_m, int32_T c19_n, real_T c19_alpha1, int32_T
  c19_ix0, int32_T c19_iy0, real_T c19_A[144], int32_T c19_ia0)
{
  int32_T c19_b_m;
  int32_T c19_b_n;
  real_T c19_b_alpha1;
  int32_T c19_b_ix0;
  int32_T c19_b_iy0;
  int32_T c19_b_ia0;
  int32_T c19_c_m;
  int32_T c19_c_n;
  real_T c19_c_alpha1;
  int32_T c19_c_ix0;
  int32_T c19_c_iy0;
  int32_T c19_c_ia0;
  int32_T c19_d_m;
  int32_T c19_d_n;
  real_T c19_d_alpha1;
  int32_T c19_d_ix0;
  int32_T c19_d_iy0;
  int32_T c19_d_ia0;
  int32_T c19_ixstart;
  int32_T c19_a;
  int32_T c19_jA;
  int32_T c19_jy;
  int32_T c19_e_n;
  int32_T c19_b;
  int32_T c19_b_b;
  boolean_T c19_overflow;
  int32_T c19_j;
  real_T c19_yjy;
  real_T c19_temp;
  int32_T c19_ix;
  int32_T c19_c_b;
  int32_T c19_i137;
  int32_T c19_b_a;
  int32_T c19_d_b;
  int32_T c19_i138;
  int32_T c19_c_a;
  int32_T c19_e_b;
  int32_T c19_d_a;
  int32_T c19_f_b;
  boolean_T c19_b_overflow;
  int32_T c19_ijA;
  int32_T c19_b_ijA;
  int32_T c19_e_a;
  int32_T c19_f_a;
  int32_T c19_g_a;
  c19_b_m = c19_m;
  c19_b_n = c19_n;
  c19_b_alpha1 = c19_alpha1;
  c19_b_ix0 = c19_ix0;
  c19_b_iy0 = c19_iy0;
  c19_b_ia0 = c19_ia0;
  c19_c_m = c19_b_m;
  c19_c_n = c19_b_n;
  c19_c_alpha1 = c19_b_alpha1;
  c19_c_ix0 = c19_b_ix0;
  c19_c_iy0 = c19_b_iy0;
  c19_c_ia0 = c19_b_ia0;
  c19_d_m = c19_c_m;
  c19_d_n = c19_c_n;
  c19_d_alpha1 = c19_c_alpha1;
  c19_d_ix0 = c19_c_ix0;
  c19_d_iy0 = c19_c_iy0;
  c19_d_ia0 = c19_c_ia0;
  if (c19_d_alpha1 == 0.0) {
  } else {
    c19_ixstart = c19_d_ix0;
    c19_a = c19_d_ia0 - 1;
    c19_jA = c19_a;
    c19_jy = c19_d_iy0;
    c19_e_n = c19_d_n;
    c19_b = c19_e_n;
    c19_b_b = c19_b;
    if (1 > c19_b_b) {
      c19_overflow = FALSE;
    } else {
      c19_overflow = (c19_b_b > 2147483646);
    }

    if (c19_overflow) {
      c19_check_forloop_overflow_error(chartInstance, c19_overflow);
    }

    for (c19_j = 1; c19_j <= c19_e_n; c19_j++) {
      c19_yjy = c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c19_jy), 1, 144, 1, 0) - 1];
      if (c19_yjy != 0.0) {
        c19_temp = c19_yjy * c19_d_alpha1;
        c19_ix = c19_ixstart;
        c19_c_b = c19_jA + 1;
        c19_i137 = c19_c_b;
        c19_b_a = c19_d_m;
        c19_d_b = c19_jA;
        c19_i138 = c19_b_a + c19_d_b;
        c19_c_a = c19_i137;
        c19_e_b = c19_i138;
        c19_d_a = c19_c_a;
        c19_f_b = c19_e_b;
        if (c19_d_a > c19_f_b) {
          c19_b_overflow = FALSE;
        } else {
          c19_b_overflow = (c19_f_b > 2147483646);
        }

        if (c19_b_overflow) {
          c19_check_forloop_overflow_error(chartInstance, c19_b_overflow);
        }

        for (c19_ijA = c19_i137; c19_ijA <= c19_i138; c19_ijA++) {
          c19_b_ijA = c19_ijA;
          c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_ijA), 1, 144, 1, 0) - 1] =
            c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_b_ijA), 1, 144, 1, 0) - 1] +
            c19_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c19_ix), 1, 144, 1, 0) - 1] * c19_temp;
          c19_e_a = c19_ix + 1;
          c19_ix = c19_e_a;
        }
      }

      c19_f_a = c19_jy + 12;
      c19_jy = c19_f_a;
      c19_g_a = c19_jA + 12;
      c19_jA = c19_g_a;
    }
  }
}

static void c19_c_eml_xtrsm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_B[72])
{
  real_T c19_alpha1;
  char_T c19_DIAGA;
  char_T c19_TRANSA;
  char_T c19_UPLO;
  char_T c19_SIDE;
  ptrdiff_t c19_m_t;
  ptrdiff_t c19_n_t;
  ptrdiff_t c19_lda_t;
  ptrdiff_t c19_ldb_t;
  double * c19_Aia0_t;
  double * c19_Bib0_t;
  double * c19_alpha1_t;
  c19_below_threshold(chartInstance);
  c19_alpha1 = 1.0;
  c19_DIAGA = 'U';
  c19_TRANSA = 'N';
  c19_UPLO = 'L';
  c19_SIDE = 'L';
  c19_m_t = (ptrdiff_t)(12);
  c19_n_t = (ptrdiff_t)(6);
  c19_lda_t = (ptrdiff_t)(12);
  c19_ldb_t = (ptrdiff_t)(12);
  c19_Aia0_t = (double *)(&c19_A[0]);
  c19_Bib0_t = (double *)(&c19_B[0]);
  c19_alpha1_t = (double *)(&c19_alpha1);
  dtrsm(&c19_SIDE, &c19_UPLO, &c19_TRANSA, &c19_DIAGA, &c19_m_t, &c19_n_t,
        c19_alpha1_t, c19_Aia0_t, &c19_lda_t, c19_Bib0_t, &c19_ldb_t);
}

static void c19_d_eml_xtrsm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[144], real_T c19_B[72])
{
  real_T c19_alpha1;
  char_T c19_DIAGA;
  char_T c19_TRANSA;
  char_T c19_UPLO;
  char_T c19_SIDE;
  ptrdiff_t c19_m_t;
  ptrdiff_t c19_n_t;
  ptrdiff_t c19_lda_t;
  ptrdiff_t c19_ldb_t;
  double * c19_Aia0_t;
  double * c19_Bib0_t;
  double * c19_alpha1_t;
  c19_below_threshold(chartInstance);
  c19_alpha1 = 1.0;
  c19_DIAGA = 'N';
  c19_TRANSA = 'N';
  c19_UPLO = 'U';
  c19_SIDE = 'L';
  c19_m_t = (ptrdiff_t)(12);
  c19_n_t = (ptrdiff_t)(6);
  c19_lda_t = (ptrdiff_t)(12);
  c19_ldb_t = (ptrdiff_t)(12);
  c19_Aia0_t = (double *)(&c19_A[0]);
  c19_Bib0_t = (double *)(&c19_B[0]);
  c19_alpha1_t = (double *)(&c19_alpha1);
  dtrsm(&c19_SIDE, &c19_UPLO, &c19_TRANSA, &c19_DIAGA, &c19_m_t, &c19_n_t,
        c19_alpha1_t, c19_Aia0_t, &c19_lda_t, c19_Bib0_t, &c19_ldb_t);
}

static void c19_d_eml_xgemm(SFc19_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c19_A[72], real_T c19_B[276], real_T c19_C[138])
{
  real_T c19_alpha1;
  real_T c19_beta1;
  char_T c19_TRANSB;
  char_T c19_TRANSA;
  ptrdiff_t c19_m_t;
  ptrdiff_t c19_n_t;
  ptrdiff_t c19_k_t;
  ptrdiff_t c19_lda_t;
  ptrdiff_t c19_ldb_t;
  ptrdiff_t c19_ldc_t;
  double * c19_alpha1_t;
  double * c19_Aia0_t;
  double * c19_Bib0_t;
  double * c19_beta1_t;
  double * c19_Cic0_t;
  c19_alpha1 = 1.0;
  c19_beta1 = 0.0;
  c19_TRANSB = 'N';
  c19_TRANSA = 'N';
  c19_m_t = (ptrdiff_t)(6);
  c19_n_t = (ptrdiff_t)(23);
  c19_k_t = (ptrdiff_t)(12);
  c19_lda_t = (ptrdiff_t)(6);
  c19_ldb_t = (ptrdiff_t)(12);
  c19_ldc_t = (ptrdiff_t)(6);
  c19_alpha1_t = (double *)(&c19_alpha1);
  c19_Aia0_t = (double *)(&c19_A[0]);
  c19_Bib0_t = (double *)(&c19_B[0]);
  c19_beta1_t = (double *)(&c19_beta1);
  c19_Cic0_t = (double *)(&c19_C[0]);
  dgemm(&c19_TRANSA, &c19_TRANSB, &c19_m_t, &c19_n_t, &c19_k_t, c19_alpha1_t,
        c19_Aia0_t, &c19_lda_t, c19_Bib0_t, &c19_ldb_t, c19_beta1_t, c19_Cic0_t,
        &c19_ldc_t);
}

static void init_dsm_address_info(SFc19_torqueBalancing2012bInstanceStruct
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

void sf_c19_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3086969071U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3087569955U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1615034229U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3004227463U);
}

mxArray *sf_c19_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("SOj1hYgXUf40sPHtPExqMG");
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

mxArray *sf_c19_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c19_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c19_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c19_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           19,
           1,
           1,
           6,
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,279);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"pinvDamped",0,-1,271);
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
            1.0,0,0,(MexFcnForType)c19_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c19_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c19_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c19_sf_marshallOut,(MexInFcnForType)
            c19_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c19_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c19_b_sf_marshallOut,(MexInFcnForType)
          c19_b_sf_marshallIn);

        {
          real_T (*c19_JcLeftFoot)[174];
          real_T (*c19_JcRightFoot)[174];
          real_T (*c19_activeFeetConstraints)[2];
          real_T (*c19_y)[6];
          real_T (*c19_qD)[23];
          c19_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
          c19_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c19_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
            (chartInstance->S, 2);
          c19_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal
            (chartInstance->S, 1);
          c19_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal
            (chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c19_JcLeftFoot);
          _SFD_SET_DATA_VALUE_PTR(1U, *c19_JcRightFoot);
          _SFD_SET_DATA_VALUE_PTR(2U, *c19_activeFeetConstraints);
          _SFD_SET_DATA_VALUE_PTR(3U, *c19_y);
          _SFD_SET_DATA_VALUE_PTR(4U, *c19_qD);
          _SFD_SET_DATA_VALUE_PTR(5U, &chartInstance->c19_reg);
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
  return "dUrGMBS4hIM1loakG1cU9B";
}

static void sf_opaque_initialize_c19_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc19_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c19_torqueBalancing2012b
    ((SFc19_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c19_torqueBalancing2012b((SFc19_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c19_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c19_torqueBalancing2012b((SFc19_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c19_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c19_torqueBalancing2012b((SFc19_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c19_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c19_torqueBalancing2012b((SFc19_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c19_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c19_torqueBalancing2012b
    ((SFc19_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c19_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c19_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c19_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c19_torqueBalancing2012b
    ((SFc19_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c19_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c19_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c19_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c19_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c19_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc19_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c19_torqueBalancing2012b((SFc19_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc19_torqueBalancing2012b
    ((SFc19_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c19_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c19_torqueBalancing2012b
      ((SFc19_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c19_torqueBalancing2012b(SimStruct *S)
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
      19);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,19,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,19,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,19);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,19,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,19,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,19);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(240957980U));
  ssSetChecksum1(S,(3091098244U));
  ssSetChecksum2(S,(1685212973U));
  ssSetChecksum3(S,(2360109791U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c19_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c19_torqueBalancing2012b(SimStruct *S)
{
  SFc19_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc19_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc19_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc19_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c19_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c19_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c19_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c19_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c19_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c19_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c19_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c19_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c19_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c19_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c19_torqueBalancing2012b;
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

void c19_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c19_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c19_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c19_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c19_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
