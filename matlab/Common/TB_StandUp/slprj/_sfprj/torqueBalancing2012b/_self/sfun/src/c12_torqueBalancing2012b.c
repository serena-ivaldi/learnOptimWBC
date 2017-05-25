/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c12_torqueBalancing2012b.h"
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
static const char * c12_debug_family_names[10] = { "Jc", "pinvJb", "nargin",
  "nargout", "JcLeftFoot", "JcRightFoot", "activeFeetConstraints", "qD", "reg",
  "y" };

static const char * c12_b_debug_family_names[5] = { "nargin", "nargout", "A",
  "regDamp", "pinvDampA" };

/* Function Declarations */
static void initialize_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void c12_update_debugger_state_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c12_st);
static void finalize_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c12_torqueBalancing2012b(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c12_chartstep_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void initSimStructsc12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void c12_pinvDamped(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_regDamp, real_T c12_pinvDampA[72]);
static void init_script_number_translation(uint32_T c12_machineNumber, uint32_T
  c12_chartNumber);
static const mxArray *c12_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static void c12_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_y, const char_T *c12_identifier, real_T
  c12_b_y[6]);
static void c12_b_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId,
  real_T c12_y[6]);
static void c12_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static const mxArray *c12_b_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static c12_struct_1ZGMVR6bgCMpDdXTSGnu6G c12_c_emlrt_marshallIn
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c12_u,
   const emlrtMsgIdentifier *c12_parentId);
static real_T c12_d_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId);
static void c12_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static const mxArray *c12_c_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static const mxArray *c12_d_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static const mxArray *c12_e_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static const mxArray *c12_f_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static void c12_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static const mxArray *c12_g_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static void c12_e_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId,
  real_T c12_y[72]);
static void c12_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static const mxArray *c12_h_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static void c12_f_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId,
  real_T c12_y[348]);
static void c12_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static const mxArray *c12_i_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static void c12_g_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId,
  real_T c12_y[72]);
static void c12_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static void c12_info_helper(c12_ResolvedFunctionInfo c12_info[128]);
static void c12_b_info_helper(c12_ResolvedFunctionInfo c12_info[128]);
static void c12_eml_scalar_eg(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c12_eml_xgemm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_B[72], real_T c12_C[144], real_T
  c12_b_C[144]);
static void c12_mrdivide(SFc12_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c12_A[72], real_T c12_B[144], real_T c12_y[72]);
static void c12_realmin(SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void c12_eps(SFc12_torqueBalancing2012bInstanceStruct *chartInstance);
static void c12_eml_matlab_zgetrf(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_b_A[144], int32_T c12_ipiv[12],
  int32_T *c12_info);
static void c12_check_forloop_overflow_error
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c12_overflow);
static void c12_eml_xger(SFc12_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c12_m, int32_T c12_n, real_T c12_alpha1, int32_T c12_ix0, int32_T
  c12_iy0, real_T c12_A[144], int32_T c12_ia0, real_T c12_b_A[144]);
static void c12_eml_warning(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c12_eml_xtrsm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_B[72], real_T c12_b_B[72]);
static void c12_below_threshold(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c12_b_eml_xtrsm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_B[72], real_T c12_b_B[72]);
static void c12_b_eml_scalar_eg(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c12_b_eml_xgemm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_B[276], real_T c12_C[138], real_T
  c12_b_C[138]);
static void c12_c_eml_scalar_eg(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c12_j_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static int32_T c12_h_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId);
static void c12_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static uint8_T c12_i_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_b_is_active_c12_torqueBalancing2012b, const
  char_T *c12_identifier);
static uint8_T c12_j_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId);
static void c12_c_eml_xgemm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_B[72], real_T c12_C[144]);
static void c12_b_eml_matlab_zgetrf(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], int32_T c12_ipiv[12], int32_T *c12_info);
static void c12_b_eml_xger(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c12_m, int32_T c12_n, real_T c12_alpha1, int32_T
  c12_ix0, int32_T c12_iy0, real_T c12_A[144], int32_T c12_ia0);
static void c12_c_eml_xtrsm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_B[72]);
static void c12_d_eml_xtrsm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_B[72]);
static void c12_d_eml_xgemm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_B[276], real_T c12_C[138]);
static void init_dsm_address_info(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c12_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c12_is_active_c12_torqueBalancing2012b = 0U;
}

static void initialize_params_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c12_m0 = NULL;
  const mxArray *c12_mxField;
  c12_struct_1ZGMVR6bgCMpDdXTSGnu6G c12_r0;
  sf_set_error_prefix_string(
    "Error evaluating data 'reg' in the parent workspace.\n");
  c12_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c12_mxField = sf_mex_getfield(c12_m0, "pinvTol", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c12_mxField), &c12_r0.pinvTol, 1, 0, 0U,
                      0, 0U, 0);
  c12_mxField = sf_mex_getfield(c12_m0, "pinvDamp", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c12_mxField), &c12_r0.pinvDamp, 1, 0, 0U,
                      0, 0U, 0);
  c12_mxField = sf_mex_getfield(c12_m0, "pinvDampVb", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c12_mxField), &c12_r0.pinvDampVb, 1, 0,
                      0U, 0, 0U, 0);
  c12_mxField = sf_mex_getfield(c12_m0, "HessianQP", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c12_mxField), &c12_r0.HessianQP, 1, 0,
                      0U, 0, 0U, 0);
  c12_mxField = sf_mex_getfield(c12_m0, "impedances", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c12_mxField), &c12_r0.impedances, 1, 0,
                      0U, 0, 0U, 0);
  c12_mxField = sf_mex_getfield(c12_m0, "dampings", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c12_mxField), &c12_r0.dampings, 1, 0, 0U,
                      0, 0U, 0);
  c12_mxField = sf_mex_getfield(c12_m0, "norm_tolerance", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c12_mxField), &c12_r0.norm_tolerance, 1,
                      0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c12_m0);
  chartInstance->c12_reg = c12_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c12_update_debugger_state_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c12_st;
  const mxArray *c12_y = NULL;
  int32_T c12_i0;
  real_T c12_u[6];
  const mxArray *c12_b_y = NULL;
  uint8_T c12_hoistedGlobal;
  uint8_T c12_b_u;
  const mxArray *c12_c_y = NULL;
  real_T (*c12_d_y)[6];
  c12_d_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c12_st = NULL;
  c12_st = NULL;
  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_createcellarray(2), FALSE);
  for (c12_i0 = 0; c12_i0 < 6; c12_i0++) {
    c12_u[c12_i0] = (*c12_d_y)[c12_i0];
  }

  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_create("y", c12_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_setcell(c12_y, 0, c12_b_y);
  c12_hoistedGlobal = chartInstance->c12_is_active_c12_torqueBalancing2012b;
  c12_b_u = c12_hoistedGlobal;
  c12_c_y = NULL;
  sf_mex_assign(&c12_c_y, sf_mex_create("y", &c12_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c12_y, 1, c12_c_y);
  sf_mex_assign(&c12_st, c12_y, FALSE);
  return c12_st;
}

static void set_sim_state_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c12_st)
{
  const mxArray *c12_u;
  real_T c12_dv0[6];
  int32_T c12_i1;
  real_T (*c12_y)[6];
  c12_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c12_doneDoubleBufferReInit = TRUE;
  c12_u = sf_mex_dup(c12_st);
  c12_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c12_u, 0)), "y",
                       c12_dv0);
  for (c12_i1 = 0; c12_i1 < 6; c12_i1++) {
    (*c12_y)[c12_i1] = c12_dv0[c12_i1];
  }

  chartInstance->c12_is_active_c12_torqueBalancing2012b = c12_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c12_u, 1)),
     "is_active_c12_torqueBalancing2012b");
  sf_mex_destroy(&c12_u);
  c12_update_debugger_state_c12_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c12_st);
}

static void finalize_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c12_torqueBalancing2012b(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c12_i2;
  int32_T c12_i3;
  int32_T c12_i4;
  int32_T c12_i5;
  int32_T c12_i6;
  real_T (*c12_qD)[23];
  real_T (*c12_y)[6];
  real_T (*c12_activeFeetConstraints)[2];
  real_T (*c12_JcRightFoot)[174];
  real_T (*c12_JcLeftFoot)[174];
  c12_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c12_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c12_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
    (chartInstance->S, 2);
  c12_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c12_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 11U, chartInstance->c12_sfEvent);
  for (c12_i2 = 0; c12_i2 < 174; c12_i2++) {
    _SFD_DATA_RANGE_CHECK((*c12_JcLeftFoot)[c12_i2], 0U);
  }

  for (c12_i3 = 0; c12_i3 < 174; c12_i3++) {
    _SFD_DATA_RANGE_CHECK((*c12_JcRightFoot)[c12_i3], 1U);
  }

  for (c12_i4 = 0; c12_i4 < 2; c12_i4++) {
    _SFD_DATA_RANGE_CHECK((*c12_activeFeetConstraints)[c12_i4], 2U);
  }

  for (c12_i5 = 0; c12_i5 < 6; c12_i5++) {
    _SFD_DATA_RANGE_CHECK((*c12_y)[c12_i5], 3U);
  }

  for (c12_i6 = 0; c12_i6 < 23; c12_i6++) {
    _SFD_DATA_RANGE_CHECK((*c12_qD)[c12_i6], 4U);
  }

  chartInstance->c12_sfEvent = CALL_EVENT;
  c12_chartstep_c12_torqueBalancing2012b(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c12_chartstep_c12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
  int32_T c12_i7;
  real_T c12_JcLeftFoot[174];
  int32_T c12_i8;
  real_T c12_JcRightFoot[174];
  int32_T c12_i9;
  real_T c12_activeFeetConstraints[2];
  int32_T c12_i10;
  real_T c12_qD[23];
  c12_struct_1ZGMVR6bgCMpDdXTSGnu6G c12_b_reg;
  uint32_T c12_debug_family_var_map[10];
  real_T c12_Jc[348];
  real_T c12_pinvJb[72];
  real_T c12_nargin = 5.0;
  real_T c12_nargout = 1.0;
  real_T c12_y[6];
  real_T c12_a;
  int32_T c12_i11;
  real_T c12_b[174];
  int32_T c12_i12;
  real_T c12_b_a;
  int32_T c12_i13;
  real_T c12_b_b[174];
  int32_T c12_i14;
  int32_T c12_i15;
  int32_T c12_i16;
  int32_T c12_i17;
  int32_T c12_i18;
  int32_T c12_i19;
  int32_T c12_i20;
  int32_T c12_i21;
  int32_T c12_i22;
  int32_T c12_i23;
  int32_T c12_i24;
  int32_T c12_i25;
  real_T c12_b_Jc[72];
  real_T c12_dv1[72];
  int32_T c12_i26;
  int32_T c12_i27;
  real_T c12_c_a[72];
  int32_T c12_i28;
  int32_T c12_i29;
  int32_T c12_i30;
  real_T c12_c_b[276];
  int32_T c12_i31;
  real_T c12_b_y[138];
  int32_T c12_i32;
  real_T c12_d_a[72];
  int32_T c12_i33;
  real_T c12_d_b[276];
  int32_T c12_i34;
  real_T c12_e_b[23];
  int32_T c12_i35;
  int32_T c12_i36;
  int32_T c12_i37;
  real_T c12_C[6];
  int32_T c12_i38;
  int32_T c12_i39;
  int32_T c12_i40;
  int32_T c12_i41;
  int32_T c12_i42;
  int32_T c12_i43;
  int32_T c12_i44;
  real_T (*c12_c_y)[6];
  real_T (*c12_b_qD)[23];
  real_T (*c12_b_activeFeetConstraints)[2];
  real_T (*c12_b_JcRightFoot)[174];
  real_T (*c12_b_JcLeftFoot)[174];
  c12_b_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c12_c_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c12_b_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
    (chartInstance->S, 2);
  c12_b_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c12_b_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 11U, chartInstance->c12_sfEvent);
  for (c12_i7 = 0; c12_i7 < 174; c12_i7++) {
    c12_JcLeftFoot[c12_i7] = (*c12_b_JcLeftFoot)[c12_i7];
  }

  for (c12_i8 = 0; c12_i8 < 174; c12_i8++) {
    c12_JcRightFoot[c12_i8] = (*c12_b_JcRightFoot)[c12_i8];
  }

  for (c12_i9 = 0; c12_i9 < 2; c12_i9++) {
    c12_activeFeetConstraints[c12_i9] = (*c12_b_activeFeetConstraints)[c12_i9];
  }

  for (c12_i10 = 0; c12_i10 < 23; c12_i10++) {
    c12_qD[c12_i10] = (*c12_b_qD)[c12_i10];
  }

  c12_b_reg = chartInstance->c12_reg;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c12_debug_family_names,
    c12_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c12_Jc, 0U, c12_h_sf_marshallOut,
    c12_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c12_pinvJb, 1U, c12_g_sf_marshallOut,
    c12_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_nargin, 2U, c12_f_sf_marshallOut,
    c12_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_nargout, 3U, c12_f_sf_marshallOut,
    c12_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c12_JcLeftFoot, 4U, c12_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c12_JcRightFoot, 5U, c12_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c12_activeFeetConstraints, 6U, c12_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c12_qD, 7U, c12_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_b_reg, 8U, c12_b_sf_marshallOut,
    c12_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c12_y, 9U, c12_sf_marshallOut,
    c12_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c12_sfEvent, 4);
  c12_a = c12_activeFeetConstraints[0];
  for (c12_i11 = 0; c12_i11 < 174; c12_i11++) {
    c12_b[c12_i11] = c12_JcLeftFoot[c12_i11];
  }

  for (c12_i12 = 0; c12_i12 < 174; c12_i12++) {
    c12_b[c12_i12] *= c12_a;
  }

  c12_b_a = c12_activeFeetConstraints[1];
  for (c12_i13 = 0; c12_i13 < 174; c12_i13++) {
    c12_b_b[c12_i13] = c12_JcRightFoot[c12_i13];
  }

  for (c12_i14 = 0; c12_i14 < 174; c12_i14++) {
    c12_b_b[c12_i14] *= c12_b_a;
  }

  c12_i15 = 0;
  c12_i16 = 0;
  for (c12_i17 = 0; c12_i17 < 29; c12_i17++) {
    for (c12_i18 = 0; c12_i18 < 6; c12_i18++) {
      c12_Jc[c12_i18 + c12_i15] = c12_b[c12_i18 + c12_i16];
    }

    c12_i15 += 12;
    c12_i16 += 6;
  }

  c12_i19 = 0;
  c12_i20 = 0;
  for (c12_i21 = 0; c12_i21 < 29; c12_i21++) {
    for (c12_i22 = 0; c12_i22 < 6; c12_i22++) {
      c12_Jc[(c12_i22 + c12_i19) + 6] = c12_b_b[c12_i22 + c12_i20];
    }

    c12_i19 += 12;
    c12_i20 += 6;
  }

  _SFD_EML_CALL(0U, chartInstance->c12_sfEvent, 7);
  c12_i23 = 0;
  for (c12_i24 = 0; c12_i24 < 6; c12_i24++) {
    for (c12_i25 = 0; c12_i25 < 12; c12_i25++) {
      c12_b_Jc[c12_i25 + c12_i23] = c12_Jc[c12_i25 + c12_i23];
    }

    c12_i23 += 12;
  }

  c12_pinvDamped(chartInstance, c12_b_Jc, c12_b_reg.pinvDampVb, c12_dv1);
  for (c12_i26 = 0; c12_i26 < 72; c12_i26++) {
    c12_pinvJb[c12_i26] = c12_dv1[c12_i26];
  }

  _SFD_EML_CALL(0U, chartInstance->c12_sfEvent, 11);
  for (c12_i27 = 0; c12_i27 < 72; c12_i27++) {
    c12_c_a[c12_i27] = -c12_pinvJb[c12_i27];
  }

  c12_i28 = 0;
  for (c12_i29 = 0; c12_i29 < 23; c12_i29++) {
    for (c12_i30 = 0; c12_i30 < 12; c12_i30++) {
      c12_c_b[c12_i30 + c12_i28] = c12_Jc[(c12_i30 + c12_i28) + 72];
    }

    c12_i28 += 12;
  }

  c12_b_eml_scalar_eg(chartInstance);
  c12_b_eml_scalar_eg(chartInstance);
  for (c12_i31 = 0; c12_i31 < 138; c12_i31++) {
    c12_b_y[c12_i31] = 0.0;
  }

  for (c12_i32 = 0; c12_i32 < 72; c12_i32++) {
    c12_d_a[c12_i32] = c12_c_a[c12_i32];
  }

  for (c12_i33 = 0; c12_i33 < 276; c12_i33++) {
    c12_d_b[c12_i33] = c12_c_b[c12_i33];
  }

  c12_d_eml_xgemm(chartInstance, c12_d_a, c12_d_b, c12_b_y);
  for (c12_i34 = 0; c12_i34 < 23; c12_i34++) {
    c12_e_b[c12_i34] = c12_qD[c12_i34];
  }

  c12_c_eml_scalar_eg(chartInstance);
  c12_c_eml_scalar_eg(chartInstance);
  for (c12_i35 = 0; c12_i35 < 6; c12_i35++) {
    c12_y[c12_i35] = 0.0;
  }

  for (c12_i36 = 0; c12_i36 < 6; c12_i36++) {
    c12_y[c12_i36] = 0.0;
  }

  for (c12_i37 = 0; c12_i37 < 6; c12_i37++) {
    c12_C[c12_i37] = c12_y[c12_i37];
  }

  for (c12_i38 = 0; c12_i38 < 6; c12_i38++) {
    c12_y[c12_i38] = c12_C[c12_i38];
  }

  for (c12_i39 = 0; c12_i39 < 6; c12_i39++) {
    c12_C[c12_i39] = c12_y[c12_i39];
  }

  for (c12_i40 = 0; c12_i40 < 6; c12_i40++) {
    c12_y[c12_i40] = c12_C[c12_i40];
  }

  for (c12_i41 = 0; c12_i41 < 6; c12_i41++) {
    c12_y[c12_i41] = 0.0;
    c12_i42 = 0;
    for (c12_i43 = 0; c12_i43 < 23; c12_i43++) {
      c12_y[c12_i41] += c12_b_y[c12_i42 + c12_i41] * c12_e_b[c12_i43];
      c12_i42 += 6;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c12_sfEvent, -11);
  _SFD_SYMBOL_SCOPE_POP();
  for (c12_i44 = 0; c12_i44 < 6; c12_i44++) {
    (*c12_c_y)[c12_i44] = c12_y[c12_i44];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 11U, chartInstance->c12_sfEvent);
}

static void initSimStructsc12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc12_torqueBalancing2012b
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c12_pinvDamped(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_regDamp, real_T c12_pinvDampA[72])
{
  uint32_T c12_debug_family_var_map[5];
  real_T c12_nargin = 2.0;
  real_T c12_nargout = 1.0;
  int32_T c12_i45;
  real_T c12_a[72];
  int32_T c12_i46;
  int32_T c12_i47;
  int32_T c12_i48;
  int32_T c12_i49;
  real_T c12_b[72];
  int32_T c12_i50;
  real_T c12_y[144];
  int32_T c12_i51;
  real_T c12_b_a[72];
  int32_T c12_i52;
  real_T c12_b_b[72];
  real_T c12_c_a;
  int32_T c12_i53;
  static real_T c12_c_b[144] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c12_b_y[144];
  int32_T c12_i54;
  int32_T c12_i55;
  int32_T c12_i56;
  int32_T c12_i57;
  real_T c12_b_A[72];
  int32_T c12_i58;
  real_T c12_c_y[144];
  real_T c12_dv2[72];
  int32_T c12_i59;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c12_b_debug_family_names,
    c12_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_nargin, 0U, c12_f_sf_marshallOut,
    c12_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_nargout, 1U, c12_f_sf_marshallOut,
    c12_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c12_A, 2U, c12_i_sf_marshallOut,
    c12_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_regDamp, 3U, c12_f_sf_marshallOut,
    c12_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c12_pinvDampA, 4U, c12_g_sf_marshallOut,
    c12_d_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 10);
  for (c12_i45 = 0; c12_i45 < 72; c12_i45++) {
    c12_a[c12_i45] = c12_A[c12_i45];
  }

  c12_i46 = 0;
  for (c12_i47 = 0; c12_i47 < 12; c12_i47++) {
    c12_i48 = 0;
    for (c12_i49 = 0; c12_i49 < 6; c12_i49++) {
      c12_b[c12_i49 + c12_i46] = c12_A[c12_i48 + c12_i47];
      c12_i48 += 12;
    }

    c12_i46 += 6;
  }

  c12_eml_scalar_eg(chartInstance);
  c12_eml_scalar_eg(chartInstance);
  for (c12_i50 = 0; c12_i50 < 144; c12_i50++) {
    c12_y[c12_i50] = 0.0;
  }

  for (c12_i51 = 0; c12_i51 < 72; c12_i51++) {
    c12_b_a[c12_i51] = c12_a[c12_i51];
  }

  for (c12_i52 = 0; c12_i52 < 72; c12_i52++) {
    c12_b_b[c12_i52] = c12_b[c12_i52];
  }

  c12_c_eml_xgemm(chartInstance, c12_b_a, c12_b_b, c12_y);
  c12_c_a = c12_regDamp;
  for (c12_i53 = 0; c12_i53 < 144; c12_i53++) {
    c12_b_y[c12_i53] = c12_c_a * c12_c_b[c12_i53];
  }

  c12_i54 = 0;
  for (c12_i55 = 0; c12_i55 < 12; c12_i55++) {
    c12_i56 = 0;
    for (c12_i57 = 0; c12_i57 < 6; c12_i57++) {
      c12_b_A[c12_i57 + c12_i54] = c12_A[c12_i56 + c12_i55];
      c12_i56 += 12;
    }

    c12_i54 += 6;
  }

  for (c12_i58 = 0; c12_i58 < 144; c12_i58++) {
    c12_c_y[c12_i58] = c12_y[c12_i58] + c12_b_y[c12_i58];
  }

  c12_mrdivide(chartInstance, c12_b_A, c12_c_y, c12_dv2);
  for (c12_i59 = 0; c12_i59 < 72; c12_i59++) {
    c12_pinvDampA[c12_i59] = c12_dv2[c12_i59];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, -10);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c12_machineNumber, uint32_T
  c12_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c12_chartNumber, 0U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m"));
}

static const mxArray *c12_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_i60;
  real_T c12_b_inData[6];
  int32_T c12_i61;
  real_T c12_u[6];
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  for (c12_i60 = 0; c12_i60 < 6; c12_i60++) {
    c12_b_inData[c12_i60] = (*(real_T (*)[6])c12_inData)[c12_i60];
  }

  for (c12_i61 = 0; c12_i61 < 6; c12_i61++) {
    c12_u[c12_i61] = c12_b_inData[c12_i61];
  }

  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static void c12_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_y, const char_T *c12_identifier, real_T
  c12_b_y[6])
{
  emlrtMsgIdentifier c12_thisId;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_y), &c12_thisId, c12_b_y);
  sf_mex_destroy(&c12_y);
}

static void c12_b_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId,
  real_T c12_y[6])
{
  real_T c12_dv3[6];
  int32_T c12_i62;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_u), c12_dv3, 1, 0, 0U, 1, 0U, 1, 6);
  for (c12_i62 = 0; c12_i62 < 6; c12_i62++) {
    c12_y[c12_i62] = c12_dv3[c12_i62];
  }

  sf_mex_destroy(&c12_u);
}

static void c12_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_y;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  real_T c12_b_y[6];
  int32_T c12_i63;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_y = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_y), &c12_thisId, c12_b_y);
  sf_mex_destroy(&c12_y);
  for (c12_i63 = 0; c12_i63 < 6; c12_i63++) {
    (*(real_T (*)[6])c12_outData)[c12_i63] = c12_b_y[c12_i63];
  }

  sf_mex_destroy(&c12_mxArrayInData);
}

static const mxArray *c12_b_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  c12_struct_1ZGMVR6bgCMpDdXTSGnu6G c12_u;
  const mxArray *c12_y = NULL;
  real_T c12_b_u;
  const mxArray *c12_b_y = NULL;
  real_T c12_c_u;
  const mxArray *c12_c_y = NULL;
  real_T c12_d_u;
  const mxArray *c12_d_y = NULL;
  real_T c12_e_u;
  const mxArray *c12_e_y = NULL;
  real_T c12_f_u;
  const mxArray *c12_f_y = NULL;
  real_T c12_g_u;
  const mxArray *c12_g_y = NULL;
  real_T c12_h_u;
  const mxArray *c12_h_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_u = *(c12_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c12_inData;
  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c12_b_u = c12_u.pinvTol;
  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_create("y", &c12_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c12_y, c12_b_y, "pinvTol", "pinvTol", 0);
  c12_c_u = c12_u.pinvDamp;
  c12_c_y = NULL;
  sf_mex_assign(&c12_c_y, sf_mex_create("y", &c12_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c12_y, c12_c_y, "pinvDamp", "pinvDamp", 0);
  c12_d_u = c12_u.pinvDampVb;
  c12_d_y = NULL;
  sf_mex_assign(&c12_d_y, sf_mex_create("y", &c12_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c12_y, c12_d_y, "pinvDampVb", "pinvDampVb", 0);
  c12_e_u = c12_u.HessianQP;
  c12_e_y = NULL;
  sf_mex_assign(&c12_e_y, sf_mex_create("y", &c12_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c12_y, c12_e_y, "HessianQP", "HessianQP", 0);
  c12_f_u = c12_u.impedances;
  c12_f_y = NULL;
  sf_mex_assign(&c12_f_y, sf_mex_create("y", &c12_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c12_y, c12_f_y, "impedances", "impedances", 0);
  c12_g_u = c12_u.dampings;
  c12_g_y = NULL;
  sf_mex_assign(&c12_g_y, sf_mex_create("y", &c12_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c12_y, c12_g_y, "dampings", "dampings", 0);
  c12_h_u = c12_u.norm_tolerance;
  c12_h_y = NULL;
  sf_mex_assign(&c12_h_y, sf_mex_create("y", &c12_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c12_y, c12_h_y, "norm_tolerance", "norm_tolerance", 0);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static c12_struct_1ZGMVR6bgCMpDdXTSGnu6G c12_c_emlrt_marshallIn
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c12_u,
   const emlrtMsgIdentifier *c12_parentId)
{
  c12_struct_1ZGMVR6bgCMpDdXTSGnu6G c12_y;
  emlrtMsgIdentifier c12_thisId;
  static const char * c12_fieldNames[7] = { "pinvTol", "pinvDamp", "pinvDampVb",
    "HessianQP", "impedances", "dampings", "norm_tolerance" };

  c12_thisId.fParent = c12_parentId;
  sf_mex_check_struct(c12_parentId, c12_u, 7, c12_fieldNames, 0U, 0);
  c12_thisId.fIdentifier = "pinvTol";
  c12_y.pinvTol = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_u, "pinvTol", "pinvTol", 0)), &c12_thisId);
  c12_thisId.fIdentifier = "pinvDamp";
  c12_y.pinvDamp = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_u, "pinvDamp", "pinvDamp", 0)), &c12_thisId);
  c12_thisId.fIdentifier = "pinvDampVb";
  c12_y.pinvDampVb = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_u, "pinvDampVb", "pinvDampVb", 0)), &c12_thisId);
  c12_thisId.fIdentifier = "HessianQP";
  c12_y.HessianQP = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_u, "HessianQP", "HessianQP", 0)), &c12_thisId);
  c12_thisId.fIdentifier = "impedances";
  c12_y.impedances = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_u, "impedances", "impedances", 0)), &c12_thisId);
  c12_thisId.fIdentifier = "dampings";
  c12_y.dampings = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_u, "dampings", "dampings", 0)), &c12_thisId);
  c12_thisId.fIdentifier = "norm_tolerance";
  c12_y.norm_tolerance = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_u, "norm_tolerance", "norm_tolerance", 0)), &c12_thisId);
  sf_mex_destroy(&c12_u);
  return c12_y;
}

static real_T c12_d_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId)
{
  real_T c12_y;
  real_T c12_d0;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_u), &c12_d0, 1, 0, 0U, 0, 0U, 0);
  c12_y = c12_d0;
  sf_mex_destroy(&c12_u);
  return c12_y;
}

static void c12_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_b_reg;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  c12_struct_1ZGMVR6bgCMpDdXTSGnu6G c12_y;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_b_reg = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_y = c12_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_b_reg),
    &c12_thisId);
  sf_mex_destroy(&c12_b_reg);
  *(c12_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c12_outData = c12_y;
  sf_mex_destroy(&c12_mxArrayInData);
}

static const mxArray *c12_c_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_i64;
  real_T c12_b_inData[23];
  int32_T c12_i65;
  real_T c12_u[23];
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  for (c12_i64 = 0; c12_i64 < 23; c12_i64++) {
    c12_b_inData[c12_i64] = (*(real_T (*)[23])c12_inData)[c12_i64];
  }

  for (c12_i65 = 0; c12_i65 < 23; c12_i65++) {
    c12_u[c12_i65] = c12_b_inData[c12_i65];
  }

  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static const mxArray *c12_d_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_i66;
  real_T c12_b_inData[2];
  int32_T c12_i67;
  real_T c12_u[2];
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  for (c12_i66 = 0; c12_i66 < 2; c12_i66++) {
    c12_b_inData[c12_i66] = (*(real_T (*)[2])c12_inData)[c12_i66];
  }

  for (c12_i67 = 0; c12_i67 < 2; c12_i67++) {
    c12_u[c12_i67] = c12_b_inData[c12_i67];
  }

  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static const mxArray *c12_e_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_i68;
  int32_T c12_i69;
  int32_T c12_i70;
  real_T c12_b_inData[174];
  int32_T c12_i71;
  int32_T c12_i72;
  int32_T c12_i73;
  real_T c12_u[174];
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_i68 = 0;
  for (c12_i69 = 0; c12_i69 < 29; c12_i69++) {
    for (c12_i70 = 0; c12_i70 < 6; c12_i70++) {
      c12_b_inData[c12_i70 + c12_i68] = (*(real_T (*)[174])c12_inData)[c12_i70 +
        c12_i68];
    }

    c12_i68 += 6;
  }

  c12_i71 = 0;
  for (c12_i72 = 0; c12_i72 < 29; c12_i72++) {
    for (c12_i73 = 0; c12_i73 < 6; c12_i73++) {
      c12_u[c12_i73 + c12_i71] = c12_b_inData[c12_i73 + c12_i71];
    }

    c12_i71 += 6;
  }

  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 0, 0U, 1U, 0U, 2, 6, 29),
                FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static const mxArray *c12_f_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  real_T c12_u;
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_u = *(real_T *)c12_inData;
  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", &c12_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static void c12_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_nargout;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  real_T c12_y;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_nargout = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_y = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_nargout),
    &c12_thisId);
  sf_mex_destroy(&c12_nargout);
  *(real_T *)c12_outData = c12_y;
  sf_mex_destroy(&c12_mxArrayInData);
}

static const mxArray *c12_g_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_i74;
  int32_T c12_i75;
  int32_T c12_i76;
  real_T c12_b_inData[72];
  int32_T c12_i77;
  int32_T c12_i78;
  int32_T c12_i79;
  real_T c12_u[72];
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_i74 = 0;
  for (c12_i75 = 0; c12_i75 < 12; c12_i75++) {
    for (c12_i76 = 0; c12_i76 < 6; c12_i76++) {
      c12_b_inData[c12_i76 + c12_i74] = (*(real_T (*)[72])c12_inData)[c12_i76 +
        c12_i74];
    }

    c12_i74 += 6;
  }

  c12_i77 = 0;
  for (c12_i78 = 0; c12_i78 < 12; c12_i78++) {
    for (c12_i79 = 0; c12_i79 < 6; c12_i79++) {
      c12_u[c12_i79 + c12_i77] = c12_b_inData[c12_i79 + c12_i77];
    }

    c12_i77 += 6;
  }

  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 0, 0U, 1U, 0U, 2, 6, 12),
                FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static void c12_e_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId,
  real_T c12_y[72])
{
  real_T c12_dv4[72];
  int32_T c12_i80;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_u), c12_dv4, 1, 0, 0U, 1, 0U, 2, 6,
                12);
  for (c12_i80 = 0; c12_i80 < 72; c12_i80++) {
    c12_y[c12_i80] = c12_dv4[c12_i80];
  }

  sf_mex_destroy(&c12_u);
}

static void c12_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_pinvJb;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  real_T c12_y[72];
  int32_T c12_i81;
  int32_T c12_i82;
  int32_T c12_i83;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_pinvJb = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_pinvJb), &c12_thisId,
    c12_y);
  sf_mex_destroy(&c12_pinvJb);
  c12_i81 = 0;
  for (c12_i82 = 0; c12_i82 < 12; c12_i82++) {
    for (c12_i83 = 0; c12_i83 < 6; c12_i83++) {
      (*(real_T (*)[72])c12_outData)[c12_i83 + c12_i81] = c12_y[c12_i83 +
        c12_i81];
    }

    c12_i81 += 6;
  }

  sf_mex_destroy(&c12_mxArrayInData);
}

static const mxArray *c12_h_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_i84;
  int32_T c12_i85;
  int32_T c12_i86;
  real_T c12_b_inData[348];
  int32_T c12_i87;
  int32_T c12_i88;
  int32_T c12_i89;
  real_T c12_u[348];
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_i84 = 0;
  for (c12_i85 = 0; c12_i85 < 29; c12_i85++) {
    for (c12_i86 = 0; c12_i86 < 12; c12_i86++) {
      c12_b_inData[c12_i86 + c12_i84] = (*(real_T (*)[348])c12_inData)[c12_i86 +
        c12_i84];
    }

    c12_i84 += 12;
  }

  c12_i87 = 0;
  for (c12_i88 = 0; c12_i88 < 29; c12_i88++) {
    for (c12_i89 = 0; c12_i89 < 12; c12_i89++) {
      c12_u[c12_i89 + c12_i87] = c12_b_inData[c12_i89 + c12_i87];
    }

    c12_i87 += 12;
  }

  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 0, 0U, 1U, 0U, 2, 12, 29),
                FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static void c12_f_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId,
  real_T c12_y[348])
{
  real_T c12_dv5[348];
  int32_T c12_i90;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_u), c12_dv5, 1, 0, 0U, 1, 0U, 2, 12,
                29);
  for (c12_i90 = 0; c12_i90 < 348; c12_i90++) {
    c12_y[c12_i90] = c12_dv5[c12_i90];
  }

  sf_mex_destroy(&c12_u);
}

static void c12_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_Jc;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  real_T c12_y[348];
  int32_T c12_i91;
  int32_T c12_i92;
  int32_T c12_i93;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_Jc = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_Jc), &c12_thisId, c12_y);
  sf_mex_destroy(&c12_Jc);
  c12_i91 = 0;
  for (c12_i92 = 0; c12_i92 < 29; c12_i92++) {
    for (c12_i93 = 0; c12_i93 < 12; c12_i93++) {
      (*(real_T (*)[348])c12_outData)[c12_i93 + c12_i91] = c12_y[c12_i93 +
        c12_i91];
    }

    c12_i91 += 12;
  }

  sf_mex_destroy(&c12_mxArrayInData);
}

static const mxArray *c12_i_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_i94;
  int32_T c12_i95;
  int32_T c12_i96;
  real_T c12_b_inData[72];
  int32_T c12_i97;
  int32_T c12_i98;
  int32_T c12_i99;
  real_T c12_u[72];
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_i94 = 0;
  for (c12_i95 = 0; c12_i95 < 6; c12_i95++) {
    for (c12_i96 = 0; c12_i96 < 12; c12_i96++) {
      c12_b_inData[c12_i96 + c12_i94] = (*(real_T (*)[72])c12_inData)[c12_i96 +
        c12_i94];
    }

    c12_i94 += 12;
  }

  c12_i97 = 0;
  for (c12_i98 = 0; c12_i98 < 6; c12_i98++) {
    for (c12_i99 = 0; c12_i99 < 12; c12_i99++) {
      c12_u[c12_i99 + c12_i97] = c12_b_inData[c12_i99 + c12_i97];
    }

    c12_i97 += 12;
  }

  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 0, 0U, 1U, 0U, 2, 12, 6),
                FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static void c12_g_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId,
  real_T c12_y[72])
{
  real_T c12_dv6[72];
  int32_T c12_i100;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_u), c12_dv6, 1, 0, 0U, 1, 0U, 2, 12,
                6);
  for (c12_i100 = 0; c12_i100 < 72; c12_i100++) {
    c12_y[c12_i100] = c12_dv6[c12_i100];
  }

  sf_mex_destroy(&c12_u);
}

static void c12_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_A;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  real_T c12_y[72];
  int32_T c12_i101;
  int32_T c12_i102;
  int32_T c12_i103;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_A = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_A), &c12_thisId, c12_y);
  sf_mex_destroy(&c12_A);
  c12_i101 = 0;
  for (c12_i102 = 0; c12_i102 < 6; c12_i102++) {
    for (c12_i103 = 0; c12_i103 < 12; c12_i103++) {
      (*(real_T (*)[72])c12_outData)[c12_i103 + c12_i101] = c12_y[c12_i103 +
        c12_i101];
    }

    c12_i101 += 12;
  }

  sf_mex_destroy(&c12_mxArrayInData);
}

const mxArray *sf_c12_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c12_nameCaptureInfo;
  c12_ResolvedFunctionInfo c12_info[128];
  const mxArray *c12_m1 = NULL;
  int32_T c12_i104;
  c12_ResolvedFunctionInfo *c12_r1;
  c12_nameCaptureInfo = NULL;
  c12_nameCaptureInfo = NULL;
  c12_info_helper(c12_info);
  c12_b_info_helper(c12_info);
  sf_mex_assign(&c12_m1, sf_mex_createstruct("nameCaptureInfo", 1, 128), FALSE);
  for (c12_i104 = 0; c12_i104 < 128; c12_i104++) {
    c12_r1 = &c12_info[c12_i104];
    sf_mex_addfield(c12_m1, sf_mex_create("nameCaptureInfo", c12_r1->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c12_r1->context)), "context", "nameCaptureInfo",
                    c12_i104);
    sf_mex_addfield(c12_m1, sf_mex_create("nameCaptureInfo", c12_r1->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c12_r1->name)), "name", "nameCaptureInfo",
                    c12_i104);
    sf_mex_addfield(c12_m1, sf_mex_create("nameCaptureInfo",
      c12_r1->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c12_r1->dominantType)),
                    "dominantType", "nameCaptureInfo", c12_i104);
    sf_mex_addfield(c12_m1, sf_mex_create("nameCaptureInfo", c12_r1->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c12_r1->resolved)), "resolved",
                    "nameCaptureInfo", c12_i104);
    sf_mex_addfield(c12_m1, sf_mex_create("nameCaptureInfo", &c12_r1->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c12_i104);
    sf_mex_addfield(c12_m1, sf_mex_create("nameCaptureInfo", &c12_r1->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c12_i104);
    sf_mex_addfield(c12_m1, sf_mex_create("nameCaptureInfo",
      &c12_r1->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c12_i104);
    sf_mex_addfield(c12_m1, sf_mex_create("nameCaptureInfo",
      &c12_r1->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c12_i104);
  }

  sf_mex_assign(&c12_nameCaptureInfo, c12_m1, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c12_nameCaptureInfo);
  return c12_nameCaptureInfo;
}

static void c12_info_helper(c12_ResolvedFunctionInfo c12_info[128])
{
  c12_info[0].context = "";
  c12_info[0].name = "mtimes";
  c12_info[0].dominantType = "double";
  c12_info[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[0].fileTimeLo = 1289519692U;
  c12_info[0].fileTimeHi = 0U;
  c12_info[0].mFileTimeLo = 0U;
  c12_info[0].mFileTimeHi = 0U;
  c12_info[1].context = "";
  c12_info[1].name = "pinvDamped";
  c12_info[1].dominantType = "double";
  c12_info[1].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c12_info[1].fileTimeLo = 1495631764U;
  c12_info[1].fileTimeHi = 0U;
  c12_info[1].mFileTimeLo = 0U;
  c12_info[1].mFileTimeHi = 0U;
  c12_info[2].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c12_info[2].name = "mtimes";
  c12_info[2].dominantType = "double";
  c12_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[2].fileTimeLo = 1289519692U;
  c12_info[2].fileTimeHi = 0U;
  c12_info[2].mFileTimeLo = 0U;
  c12_info[2].mFileTimeHi = 0U;
  c12_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[3].name = "eml_index_class";
  c12_info[3].dominantType = "";
  c12_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[3].fileTimeLo = 1323170578U;
  c12_info[3].fileTimeHi = 0U;
  c12_info[3].mFileTimeLo = 0U;
  c12_info[3].mFileTimeHi = 0U;
  c12_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[4].name = "eml_scalar_eg";
  c12_info[4].dominantType = "double";
  c12_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[4].fileTimeLo = 1286818796U;
  c12_info[4].fileTimeHi = 0U;
  c12_info[4].mFileTimeLo = 0U;
  c12_info[4].mFileTimeHi = 0U;
  c12_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[5].name = "eml_xgemm";
  c12_info[5].dominantType = "char";
  c12_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c12_info[5].fileTimeLo = 1299076772U;
  c12_info[5].fileTimeHi = 0U;
  c12_info[5].mFileTimeLo = 0U;
  c12_info[5].mFileTimeHi = 0U;
  c12_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c12_info[6].name = "eml_blas_inline";
  c12_info[6].dominantType = "";
  c12_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c12_info[6].fileTimeLo = 1299076768U;
  c12_info[6].fileTimeHi = 0U;
  c12_info[6].mFileTimeLo = 0U;
  c12_info[6].mFileTimeHi = 0U;
  c12_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c12_info[7].name = "mtimes";
  c12_info[7].dominantType = "double";
  c12_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[7].fileTimeLo = 1289519692U;
  c12_info[7].fileTimeHi = 0U;
  c12_info[7].mFileTimeLo = 0U;
  c12_info[7].mFileTimeHi = 0U;
  c12_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c12_info[8].name = "eml_index_class";
  c12_info[8].dominantType = "";
  c12_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[8].fileTimeLo = 1323170578U;
  c12_info[8].fileTimeHi = 0U;
  c12_info[8].mFileTimeLo = 0U;
  c12_info[8].mFileTimeHi = 0U;
  c12_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c12_info[9].name = "eml_scalar_eg";
  c12_info[9].dominantType = "double";
  c12_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[9].fileTimeLo = 1286818796U;
  c12_info[9].fileTimeHi = 0U;
  c12_info[9].mFileTimeLo = 0U;
  c12_info[9].mFileTimeHi = 0U;
  c12_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c12_info[10].name = "eml_refblas_xgemm";
  c12_info[10].dominantType = "char";
  c12_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c12_info[10].fileTimeLo = 1299076774U;
  c12_info[10].fileTimeHi = 0U;
  c12_info[10].mFileTimeLo = 0U;
  c12_info[10].mFileTimeHi = 0U;
  c12_info[11].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c12_info[11].name = "eye";
  c12_info[11].dominantType = "double";
  c12_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c12_info[11].fileTimeLo = 1286818688U;
  c12_info[11].fileTimeHi = 0U;
  c12_info[11].mFileTimeLo = 0U;
  c12_info[11].mFileTimeHi = 0U;
  c12_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c12_info[12].name = "eml_assert_valid_size_arg";
  c12_info[12].dominantType = "double";
  c12_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c12_info[12].fileTimeLo = 1286818694U;
  c12_info[12].fileTimeHi = 0U;
  c12_info[12].mFileTimeLo = 0U;
  c12_info[12].mFileTimeHi = 0U;
  c12_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c12_info[13].name = "isinf";
  c12_info[13].dominantType = "double";
  c12_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c12_info[13].fileTimeLo = 1286818760U;
  c12_info[13].fileTimeHi = 0U;
  c12_info[13].mFileTimeLo = 0U;
  c12_info[13].mFileTimeHi = 0U;
  c12_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c12_info[14].name = "mtimes";
  c12_info[14].dominantType = "double";
  c12_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[14].fileTimeLo = 1289519692U;
  c12_info[14].fileTimeHi = 0U;
  c12_info[14].mFileTimeLo = 0U;
  c12_info[14].mFileTimeHi = 0U;
  c12_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c12_info[15].name = "eml_index_class";
  c12_info[15].dominantType = "";
  c12_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[15].fileTimeLo = 1323170578U;
  c12_info[15].fileTimeHi = 0U;
  c12_info[15].mFileTimeLo = 0U;
  c12_info[15].mFileTimeHi = 0U;
  c12_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c12_info[16].name = "intmax";
  c12_info[16].dominantType = "char";
  c12_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c12_info[16].fileTimeLo = 1311255316U;
  c12_info[16].fileTimeHi = 0U;
  c12_info[16].mFileTimeLo = 0U;
  c12_info[16].mFileTimeHi = 0U;
  c12_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c12_info[17].name = "eml_is_float_class";
  c12_info[17].dominantType = "char";
  c12_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c12_info[17].fileTimeLo = 1286818782U;
  c12_info[17].fileTimeHi = 0U;
  c12_info[17].mFileTimeLo = 0U;
  c12_info[17].mFileTimeHi = 0U;
  c12_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c12_info[18].name = "min";
  c12_info[18].dominantType = "double";
  c12_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c12_info[18].fileTimeLo = 1311255318U;
  c12_info[18].fileTimeHi = 0U;
  c12_info[18].mFileTimeLo = 0U;
  c12_info[18].mFileTimeHi = 0U;
  c12_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c12_info[19].name = "eml_min_or_max";
  c12_info[19].dominantType = "char";
  c12_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c12_info[19].fileTimeLo = 1334071490U;
  c12_info[19].fileTimeHi = 0U;
  c12_info[19].mFileTimeLo = 0U;
  c12_info[19].mFileTimeHi = 0U;
  c12_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c12_info[20].name = "eml_scalar_eg";
  c12_info[20].dominantType = "double";
  c12_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[20].fileTimeLo = 1286818796U;
  c12_info[20].fileTimeHi = 0U;
  c12_info[20].mFileTimeLo = 0U;
  c12_info[20].mFileTimeHi = 0U;
  c12_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c12_info[21].name = "eml_scalexp_alloc";
  c12_info[21].dominantType = "double";
  c12_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c12_info[21].fileTimeLo = 1352424860U;
  c12_info[21].fileTimeHi = 0U;
  c12_info[21].mFileTimeLo = 0U;
  c12_info[21].mFileTimeHi = 0U;
  c12_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c12_info[22].name = "eml_index_class";
  c12_info[22].dominantType = "";
  c12_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[22].fileTimeLo = 1323170578U;
  c12_info[22].fileTimeHi = 0U;
  c12_info[22].mFileTimeLo = 0U;
  c12_info[22].mFileTimeHi = 0U;
  c12_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c12_info[23].name = "eml_scalar_eg";
  c12_info[23].dominantType = "double";
  c12_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[23].fileTimeLo = 1286818796U;
  c12_info[23].fileTimeHi = 0U;
  c12_info[23].mFileTimeLo = 0U;
  c12_info[23].mFileTimeHi = 0U;
  c12_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c12_info[24].name = "eml_index_class";
  c12_info[24].dominantType = "";
  c12_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[24].fileTimeLo = 1323170578U;
  c12_info[24].fileTimeHi = 0U;
  c12_info[24].mFileTimeLo = 0U;
  c12_info[24].mFileTimeHi = 0U;
  c12_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c12_info[25].name = "eml_int_forloop_overflow_check";
  c12_info[25].dominantType = "";
  c12_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c12_info[25].fileTimeLo = 1346510340U;
  c12_info[25].fileTimeHi = 0U;
  c12_info[25].mFileTimeLo = 0U;
  c12_info[25].mFileTimeHi = 0U;
  c12_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c12_info[26].name = "intmax";
  c12_info[26].dominantType = "char";
  c12_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c12_info[26].fileTimeLo = 1311255316U;
  c12_info[26].fileTimeHi = 0U;
  c12_info[26].mFileTimeLo = 0U;
  c12_info[26].mFileTimeHi = 0U;
  c12_info[27].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c12_info[27].name = "mrdivide";
  c12_info[27].dominantType = "double";
  c12_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c12_info[27].fileTimeLo = 1357951548U;
  c12_info[27].fileTimeHi = 0U;
  c12_info[27].mFileTimeLo = 1319729966U;
  c12_info[27].mFileTimeHi = 0U;
  c12_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c12_info[28].name = "mldivide";
  c12_info[28].dominantType = "double";
  c12_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c12_info[28].fileTimeLo = 1357951548U;
  c12_info[28].fileTimeHi = 0U;
  c12_info[28].mFileTimeLo = 1319729966U;
  c12_info[28].mFileTimeHi = 0U;
  c12_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c12_info[29].name = "eml_lusolve";
  c12_info[29].dominantType = "double";
  c12_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c12_info[29].fileTimeLo = 1309451196U;
  c12_info[29].fileTimeHi = 0U;
  c12_info[29].mFileTimeLo = 0U;
  c12_info[29].mFileTimeHi = 0U;
  c12_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c12_info[30].name = "eml_index_class";
  c12_info[30].dominantType = "";
  c12_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[30].fileTimeLo = 1323170578U;
  c12_info[30].fileTimeHi = 0U;
  c12_info[30].mFileTimeLo = 0U;
  c12_info[30].mFileTimeHi = 0U;
  c12_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c12_info[31].name = "eml_index_class";
  c12_info[31].dominantType = "";
  c12_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[31].fileTimeLo = 1323170578U;
  c12_info[31].fileTimeHi = 0U;
  c12_info[31].mFileTimeLo = 0U;
  c12_info[31].mFileTimeHi = 0U;
  c12_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c12_info[32].name = "eml_xgetrf";
  c12_info[32].dominantType = "double";
  c12_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c12_info[32].fileTimeLo = 1286818806U;
  c12_info[32].fileTimeHi = 0U;
  c12_info[32].mFileTimeLo = 0U;
  c12_info[32].mFileTimeHi = 0U;
  c12_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c12_info[33].name = "eml_lapack_xgetrf";
  c12_info[33].dominantType = "double";
  c12_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c12_info[33].fileTimeLo = 1286818810U;
  c12_info[33].fileTimeHi = 0U;
  c12_info[33].mFileTimeLo = 0U;
  c12_info[33].mFileTimeHi = 0U;
  c12_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c12_info[34].name = "eml_matlab_zgetrf";
  c12_info[34].dominantType = "double";
  c12_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[34].fileTimeLo = 1302688994U;
  c12_info[34].fileTimeHi = 0U;
  c12_info[34].mFileTimeLo = 0U;
  c12_info[34].mFileTimeHi = 0U;
  c12_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[35].name = "realmin";
  c12_info[35].dominantType = "char";
  c12_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c12_info[35].fileTimeLo = 1307651242U;
  c12_info[35].fileTimeHi = 0U;
  c12_info[35].mFileTimeLo = 0U;
  c12_info[35].mFileTimeHi = 0U;
  c12_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c12_info[36].name = "eml_realmin";
  c12_info[36].dominantType = "char";
  c12_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c12_info[36].fileTimeLo = 1307651244U;
  c12_info[36].fileTimeHi = 0U;
  c12_info[36].mFileTimeLo = 0U;
  c12_info[36].mFileTimeHi = 0U;
  c12_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c12_info[37].name = "eml_float_model";
  c12_info[37].dominantType = "char";
  c12_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c12_info[37].fileTimeLo = 1326727996U;
  c12_info[37].fileTimeHi = 0U;
  c12_info[37].mFileTimeLo = 0U;
  c12_info[37].mFileTimeHi = 0U;
  c12_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[38].name = "eps";
  c12_info[38].dominantType = "char";
  c12_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c12_info[38].fileTimeLo = 1326727996U;
  c12_info[38].fileTimeHi = 0U;
  c12_info[38].mFileTimeLo = 0U;
  c12_info[38].mFileTimeHi = 0U;
  c12_info[39].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c12_info[39].name = "eml_is_float_class";
  c12_info[39].dominantType = "char";
  c12_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c12_info[39].fileTimeLo = 1286818782U;
  c12_info[39].fileTimeHi = 0U;
  c12_info[39].mFileTimeLo = 0U;
  c12_info[39].mFileTimeHi = 0U;
  c12_info[40].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c12_info[40].name = "eml_eps";
  c12_info[40].dominantType = "char";
  c12_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c12_info[40].fileTimeLo = 1326727996U;
  c12_info[40].fileTimeHi = 0U;
  c12_info[40].mFileTimeLo = 0U;
  c12_info[40].mFileTimeHi = 0U;
  c12_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c12_info[41].name = "eml_float_model";
  c12_info[41].dominantType = "char";
  c12_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c12_info[41].fileTimeLo = 1326727996U;
  c12_info[41].fileTimeHi = 0U;
  c12_info[41].mFileTimeLo = 0U;
  c12_info[41].mFileTimeHi = 0U;
  c12_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[42].name = "min";
  c12_info[42].dominantType = "coder.internal.indexInt";
  c12_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c12_info[42].fileTimeLo = 1311255318U;
  c12_info[42].fileTimeHi = 0U;
  c12_info[42].mFileTimeLo = 0U;
  c12_info[42].mFileTimeHi = 0U;
  c12_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c12_info[43].name = "eml_scalar_eg";
  c12_info[43].dominantType = "coder.internal.indexInt";
  c12_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[43].fileTimeLo = 1286818796U;
  c12_info[43].fileTimeHi = 0U;
  c12_info[43].mFileTimeLo = 0U;
  c12_info[43].mFileTimeHi = 0U;
  c12_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c12_info[44].name = "eml_scalexp_alloc";
  c12_info[44].dominantType = "coder.internal.indexInt";
  c12_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c12_info[44].fileTimeLo = 1352424860U;
  c12_info[44].fileTimeHi = 0U;
  c12_info[44].mFileTimeLo = 0U;
  c12_info[44].mFileTimeHi = 0U;
  c12_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c12_info[45].name = "eml_scalar_eg";
  c12_info[45].dominantType = "coder.internal.indexInt";
  c12_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[45].fileTimeLo = 1286818796U;
  c12_info[45].fileTimeHi = 0U;
  c12_info[45].mFileTimeLo = 0U;
  c12_info[45].mFileTimeHi = 0U;
  c12_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[46].name = "colon";
  c12_info[46].dominantType = "double";
  c12_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c12_info[46].fileTimeLo = 1348191928U;
  c12_info[46].fileTimeHi = 0U;
  c12_info[46].mFileTimeLo = 0U;
  c12_info[46].mFileTimeHi = 0U;
  c12_info[47].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c12_info[47].name = "colon";
  c12_info[47].dominantType = "double";
  c12_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c12_info[47].fileTimeLo = 1348191928U;
  c12_info[47].fileTimeHi = 0U;
  c12_info[47].mFileTimeLo = 0U;
  c12_info[47].mFileTimeHi = 0U;
  c12_info[48].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c12_info[48].name = "floor";
  c12_info[48].dominantType = "double";
  c12_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c12_info[48].fileTimeLo = 1343830380U;
  c12_info[48].fileTimeHi = 0U;
  c12_info[48].mFileTimeLo = 0U;
  c12_info[48].mFileTimeHi = 0U;
  c12_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c12_info[49].name = "eml_scalar_floor";
  c12_info[49].dominantType = "double";
  c12_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c12_info[49].fileTimeLo = 1286818726U;
  c12_info[49].fileTimeHi = 0U;
  c12_info[49].mFileTimeLo = 0U;
  c12_info[49].mFileTimeHi = 0U;
  c12_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c12_info[50].name = "intmin";
  c12_info[50].dominantType = "char";
  c12_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c12_info[50].fileTimeLo = 1311255318U;
  c12_info[50].fileTimeHi = 0U;
  c12_info[50].mFileTimeLo = 0U;
  c12_info[50].mFileTimeHi = 0U;
  c12_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c12_info[51].name = "intmax";
  c12_info[51].dominantType = "char";
  c12_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c12_info[51].fileTimeLo = 1311255316U;
  c12_info[51].fileTimeHi = 0U;
  c12_info[51].mFileTimeLo = 0U;
  c12_info[51].mFileTimeHi = 0U;
  c12_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c12_info[52].name = "intmin";
  c12_info[52].dominantType = "char";
  c12_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c12_info[52].fileTimeLo = 1311255318U;
  c12_info[52].fileTimeHi = 0U;
  c12_info[52].mFileTimeLo = 0U;
  c12_info[52].mFileTimeHi = 0U;
  c12_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c12_info[53].name = "intmax";
  c12_info[53].dominantType = "char";
  c12_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c12_info[53].fileTimeLo = 1311255316U;
  c12_info[53].fileTimeHi = 0U;
  c12_info[53].mFileTimeLo = 0U;
  c12_info[53].mFileTimeHi = 0U;
  c12_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c12_info[54].name = "eml_isa_uint";
  c12_info[54].dominantType = "coder.internal.indexInt";
  c12_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c12_info[54].fileTimeLo = 1286818784U;
  c12_info[54].fileTimeHi = 0U;
  c12_info[54].mFileTimeLo = 0U;
  c12_info[54].mFileTimeHi = 0U;
  c12_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c12_info[55].name = "eml_unsigned_class";
  c12_info[55].dominantType = "char";
  c12_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c12_info[55].fileTimeLo = 1323170580U;
  c12_info[55].fileTimeHi = 0U;
  c12_info[55].mFileTimeLo = 0U;
  c12_info[55].mFileTimeHi = 0U;
  c12_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c12_info[56].name = "eml_index_class";
  c12_info[56].dominantType = "";
  c12_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[56].fileTimeLo = 1323170578U;
  c12_info[56].fileTimeHi = 0U;
  c12_info[56].mFileTimeLo = 0U;
  c12_info[56].mFileTimeHi = 0U;
  c12_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c12_info[57].name = "eml_index_class";
  c12_info[57].dominantType = "";
  c12_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[57].fileTimeLo = 1323170578U;
  c12_info[57].fileTimeHi = 0U;
  c12_info[57].mFileTimeLo = 0U;
  c12_info[57].mFileTimeHi = 0U;
  c12_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c12_info[58].name = "intmax";
  c12_info[58].dominantType = "char";
  c12_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c12_info[58].fileTimeLo = 1311255316U;
  c12_info[58].fileTimeHi = 0U;
  c12_info[58].mFileTimeLo = 0U;
  c12_info[58].mFileTimeHi = 0U;
  c12_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c12_info[59].name = "eml_isa_uint";
  c12_info[59].dominantType = "coder.internal.indexInt";
  c12_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c12_info[59].fileTimeLo = 1286818784U;
  c12_info[59].fileTimeHi = 0U;
  c12_info[59].mFileTimeLo = 0U;
  c12_info[59].mFileTimeHi = 0U;
  c12_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c12_info[60].name = "eml_index_plus";
  c12_info[60].dominantType = "double";
  c12_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[60].fileTimeLo = 1286818778U;
  c12_info[60].fileTimeHi = 0U;
  c12_info[60].mFileTimeLo = 0U;
  c12_info[60].mFileTimeHi = 0U;
  c12_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[61].name = "eml_index_class";
  c12_info[61].dominantType = "";
  c12_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[61].fileTimeLo = 1323170578U;
  c12_info[61].fileTimeHi = 0U;
  c12_info[61].mFileTimeLo = 0U;
  c12_info[61].mFileTimeHi = 0U;
  c12_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c12_info[62].name = "eml_int_forloop_overflow_check";
  c12_info[62].dominantType = "";
  c12_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c12_info[62].fileTimeLo = 1346510340U;
  c12_info[62].fileTimeHi = 0U;
  c12_info[62].mFileTimeLo = 0U;
  c12_info[62].mFileTimeHi = 0U;
  c12_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[63].name = "eml_index_class";
  c12_info[63].dominantType = "";
  c12_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[63].fileTimeLo = 1323170578U;
  c12_info[63].fileTimeHi = 0U;
  c12_info[63].mFileTimeLo = 0U;
  c12_info[63].mFileTimeHi = 0U;
}

static void c12_b_info_helper(c12_ResolvedFunctionInfo c12_info[128])
{
  c12_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[64].name = "eml_index_plus";
  c12_info[64].dominantType = "double";
  c12_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[64].fileTimeLo = 1286818778U;
  c12_info[64].fileTimeHi = 0U;
  c12_info[64].mFileTimeLo = 0U;
  c12_info[64].mFileTimeHi = 0U;
  c12_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[65].name = "eml_int_forloop_overflow_check";
  c12_info[65].dominantType = "";
  c12_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c12_info[65].fileTimeLo = 1346510340U;
  c12_info[65].fileTimeHi = 0U;
  c12_info[65].mFileTimeLo = 0U;
  c12_info[65].mFileTimeHi = 0U;
  c12_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[66].name = "eml_index_minus";
  c12_info[66].dominantType = "double";
  c12_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c12_info[66].fileTimeLo = 1286818778U;
  c12_info[66].fileTimeHi = 0U;
  c12_info[66].mFileTimeLo = 0U;
  c12_info[66].mFileTimeHi = 0U;
  c12_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c12_info[67].name = "eml_index_class";
  c12_info[67].dominantType = "";
  c12_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[67].fileTimeLo = 1323170578U;
  c12_info[67].fileTimeHi = 0U;
  c12_info[67].mFileTimeLo = 0U;
  c12_info[67].mFileTimeHi = 0U;
  c12_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[68].name = "eml_index_minus";
  c12_info[68].dominantType = "coder.internal.indexInt";
  c12_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c12_info[68].fileTimeLo = 1286818778U;
  c12_info[68].fileTimeHi = 0U;
  c12_info[68].mFileTimeLo = 0U;
  c12_info[68].mFileTimeHi = 0U;
  c12_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[69].name = "eml_index_times";
  c12_info[69].dominantType = "coder.internal.indexInt";
  c12_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c12_info[69].fileTimeLo = 1286818780U;
  c12_info[69].fileTimeHi = 0U;
  c12_info[69].mFileTimeLo = 0U;
  c12_info[69].mFileTimeHi = 0U;
  c12_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c12_info[70].name = "eml_index_class";
  c12_info[70].dominantType = "";
  c12_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[70].fileTimeLo = 1323170578U;
  c12_info[70].fileTimeHi = 0U;
  c12_info[70].mFileTimeLo = 0U;
  c12_info[70].mFileTimeHi = 0U;
  c12_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[71].name = "eml_index_plus";
  c12_info[71].dominantType = "coder.internal.indexInt";
  c12_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[71].fileTimeLo = 1286818778U;
  c12_info[71].fileTimeHi = 0U;
  c12_info[71].mFileTimeLo = 0U;
  c12_info[71].mFileTimeHi = 0U;
  c12_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[72].name = "eml_ixamax";
  c12_info[72].dominantType = "double";
  c12_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c12_info[72].fileTimeLo = 1299076770U;
  c12_info[72].fileTimeHi = 0U;
  c12_info[72].mFileTimeLo = 0U;
  c12_info[72].mFileTimeHi = 0U;
  c12_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c12_info[73].name = "eml_blas_inline";
  c12_info[73].dominantType = "";
  c12_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c12_info[73].fileTimeLo = 1299076768U;
  c12_info[73].fileTimeHi = 0U;
  c12_info[73].mFileTimeLo = 0U;
  c12_info[73].mFileTimeHi = 0U;
  c12_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c12_info[74].name = "length";
  c12_info[74].dominantType = "double";
  c12_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c12_info[74].fileTimeLo = 1303146206U;
  c12_info[74].fileTimeHi = 0U;
  c12_info[74].mFileTimeLo = 0U;
  c12_info[74].mFileTimeHi = 0U;
  c12_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c12_info[75].name = "eml_index_class";
  c12_info[75].dominantType = "";
  c12_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[75].fileTimeLo = 1323170578U;
  c12_info[75].fileTimeHi = 0U;
  c12_info[75].mFileTimeLo = 0U;
  c12_info[75].mFileTimeHi = 0U;
  c12_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c12_info[76].name = "eml_index_class";
  c12_info[76].dominantType = "";
  c12_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[76].fileTimeLo = 1323170578U;
  c12_info[76].fileTimeHi = 0U;
  c12_info[76].mFileTimeLo = 0U;
  c12_info[76].mFileTimeHi = 0U;
  c12_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c12_info[77].name = "eml_refblas_ixamax";
  c12_info[77].dominantType = "double";
  c12_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c12_info[77].fileTimeLo = 1299076770U;
  c12_info[77].fileTimeHi = 0U;
  c12_info[77].mFileTimeLo = 0U;
  c12_info[77].mFileTimeHi = 0U;
  c12_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c12_info[78].name = "eml_index_class";
  c12_info[78].dominantType = "";
  c12_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[78].fileTimeLo = 1323170578U;
  c12_info[78].fileTimeHi = 0U;
  c12_info[78].mFileTimeLo = 0U;
  c12_info[78].mFileTimeHi = 0U;
  c12_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c12_info[79].name = "eml_xcabs1";
  c12_info[79].dominantType = "double";
  c12_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c12_info[79].fileTimeLo = 1286818706U;
  c12_info[79].fileTimeHi = 0U;
  c12_info[79].mFileTimeLo = 0U;
  c12_info[79].mFileTimeHi = 0U;
  c12_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c12_info[80].name = "abs";
  c12_info[80].dominantType = "double";
  c12_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c12_info[80].fileTimeLo = 1343830366U;
  c12_info[80].fileTimeHi = 0U;
  c12_info[80].mFileTimeLo = 0U;
  c12_info[80].mFileTimeHi = 0U;
  c12_info[81].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c12_info[81].name = "eml_scalar_abs";
  c12_info[81].dominantType = "double";
  c12_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c12_info[81].fileTimeLo = 1286818712U;
  c12_info[81].fileTimeHi = 0U;
  c12_info[81].mFileTimeLo = 0U;
  c12_info[81].mFileTimeHi = 0U;
  c12_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c12_info[82].name = "eml_int_forloop_overflow_check";
  c12_info[82].dominantType = "";
  c12_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c12_info[82].fileTimeLo = 1346510340U;
  c12_info[82].fileTimeHi = 0U;
  c12_info[82].mFileTimeLo = 0U;
  c12_info[82].mFileTimeHi = 0U;
  c12_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c12_info[83].name = "eml_index_plus";
  c12_info[83].dominantType = "coder.internal.indexInt";
  c12_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[83].fileTimeLo = 1286818778U;
  c12_info[83].fileTimeHi = 0U;
  c12_info[83].mFileTimeLo = 0U;
  c12_info[83].mFileTimeHi = 0U;
  c12_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[84].name = "eml_xswap";
  c12_info[84].dominantType = "double";
  c12_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c12_info[84].fileTimeLo = 1299076778U;
  c12_info[84].fileTimeHi = 0U;
  c12_info[84].mFileTimeLo = 0U;
  c12_info[84].mFileTimeHi = 0U;
  c12_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c12_info[85].name = "eml_blas_inline";
  c12_info[85].dominantType = "";
  c12_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c12_info[85].fileTimeLo = 1299076768U;
  c12_info[85].fileTimeHi = 0U;
  c12_info[85].mFileTimeLo = 0U;
  c12_info[85].mFileTimeHi = 0U;
  c12_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c12_info[86].name = "eml_index_class";
  c12_info[86].dominantType = "";
  c12_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[86].fileTimeLo = 1323170578U;
  c12_info[86].fileTimeHi = 0U;
  c12_info[86].mFileTimeLo = 0U;
  c12_info[86].mFileTimeHi = 0U;
  c12_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c12_info[87].name = "eml_refblas_xswap";
  c12_info[87].dominantType = "double";
  c12_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c12_info[87].fileTimeLo = 1299076786U;
  c12_info[87].fileTimeHi = 0U;
  c12_info[87].mFileTimeLo = 0U;
  c12_info[87].mFileTimeHi = 0U;
  c12_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c12_info[88].name = "eml_index_class";
  c12_info[88].dominantType = "";
  c12_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[88].fileTimeLo = 1323170578U;
  c12_info[88].fileTimeHi = 0U;
  c12_info[88].mFileTimeLo = 0U;
  c12_info[88].mFileTimeHi = 0U;
  c12_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c12_info[89].name = "abs";
  c12_info[89].dominantType = "coder.internal.indexInt";
  c12_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c12_info[89].fileTimeLo = 1343830366U;
  c12_info[89].fileTimeHi = 0U;
  c12_info[89].mFileTimeLo = 0U;
  c12_info[89].mFileTimeHi = 0U;
  c12_info[90].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c12_info[90].name = "eml_scalar_abs";
  c12_info[90].dominantType = "coder.internal.indexInt";
  c12_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c12_info[90].fileTimeLo = 1286818712U;
  c12_info[90].fileTimeHi = 0U;
  c12_info[90].mFileTimeLo = 0U;
  c12_info[90].mFileTimeHi = 0U;
  c12_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c12_info[91].name = "eml_int_forloop_overflow_check";
  c12_info[91].dominantType = "";
  c12_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c12_info[91].fileTimeLo = 1346510340U;
  c12_info[91].fileTimeHi = 0U;
  c12_info[91].mFileTimeLo = 0U;
  c12_info[91].mFileTimeHi = 0U;
  c12_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c12_info[92].name = "eml_index_plus";
  c12_info[92].dominantType = "coder.internal.indexInt";
  c12_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[92].fileTimeLo = 1286818778U;
  c12_info[92].fileTimeHi = 0U;
  c12_info[92].mFileTimeLo = 0U;
  c12_info[92].mFileTimeHi = 0U;
  c12_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[93].name = "eml_div";
  c12_info[93].dominantType = "double";
  c12_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c12_info[93].fileTimeLo = 1313347810U;
  c12_info[93].fileTimeHi = 0U;
  c12_info[93].mFileTimeLo = 0U;
  c12_info[93].mFileTimeHi = 0U;
  c12_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c12_info[94].name = "eml_xgeru";
  c12_info[94].dominantType = "double";
  c12_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c12_info[94].fileTimeLo = 1299076774U;
  c12_info[94].fileTimeHi = 0U;
  c12_info[94].mFileTimeLo = 0U;
  c12_info[94].mFileTimeHi = 0U;
  c12_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c12_info[95].name = "eml_blas_inline";
  c12_info[95].dominantType = "";
  c12_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c12_info[95].fileTimeLo = 1299076768U;
  c12_info[95].fileTimeHi = 0U;
  c12_info[95].mFileTimeLo = 0U;
  c12_info[95].mFileTimeHi = 0U;
  c12_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c12_info[96].name = "eml_xger";
  c12_info[96].dominantType = "double";
  c12_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c12_info[96].fileTimeLo = 1299076774U;
  c12_info[96].fileTimeHi = 0U;
  c12_info[96].mFileTimeLo = 0U;
  c12_info[96].mFileTimeHi = 0U;
  c12_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c12_info[97].name = "eml_blas_inline";
  c12_info[97].dominantType = "";
  c12_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c12_info[97].fileTimeLo = 1299076768U;
  c12_info[97].fileTimeHi = 0U;
  c12_info[97].mFileTimeLo = 0U;
  c12_info[97].mFileTimeHi = 0U;
  c12_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c12_info[98].name = "intmax";
  c12_info[98].dominantType = "char";
  c12_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c12_info[98].fileTimeLo = 1311255316U;
  c12_info[98].fileTimeHi = 0U;
  c12_info[98].mFileTimeLo = 0U;
  c12_info[98].mFileTimeHi = 0U;
  c12_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c12_info[99].name = "min";
  c12_info[99].dominantType = "double";
  c12_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c12_info[99].fileTimeLo = 1311255318U;
  c12_info[99].fileTimeHi = 0U;
  c12_info[99].mFileTimeLo = 0U;
  c12_info[99].mFileTimeHi = 0U;
  c12_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c12_info[100].name = "mtimes";
  c12_info[100].dominantType = "double";
  c12_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[100].fileTimeLo = 1289519692U;
  c12_info[100].fileTimeHi = 0U;
  c12_info[100].mFileTimeLo = 0U;
  c12_info[100].mFileTimeHi = 0U;
  c12_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c12_info[101].name = "eml_index_class";
  c12_info[101].dominantType = "";
  c12_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[101].fileTimeLo = 1323170578U;
  c12_info[101].fileTimeHi = 0U;
  c12_info[101].mFileTimeLo = 0U;
  c12_info[101].mFileTimeHi = 0U;
  c12_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c12_info[102].name = "eml_refblas_xger";
  c12_info[102].dominantType = "double";
  c12_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c12_info[102].fileTimeLo = 1299076776U;
  c12_info[102].fileTimeHi = 0U;
  c12_info[102].mFileTimeLo = 0U;
  c12_info[102].mFileTimeHi = 0U;
  c12_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c12_info[103].name = "eml_refblas_xgerx";
  c12_info[103].dominantType = "char";
  c12_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c12_info[103].fileTimeLo = 1299076778U;
  c12_info[103].fileTimeHi = 0U;
  c12_info[103].mFileTimeLo = 0U;
  c12_info[103].mFileTimeHi = 0U;
  c12_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c12_info[104].name = "eml_index_class";
  c12_info[104].dominantType = "";
  c12_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[104].fileTimeLo = 1323170578U;
  c12_info[104].fileTimeHi = 0U;
  c12_info[104].mFileTimeLo = 0U;
  c12_info[104].mFileTimeHi = 0U;
  c12_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c12_info[105].name = "abs";
  c12_info[105].dominantType = "coder.internal.indexInt";
  c12_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c12_info[105].fileTimeLo = 1343830366U;
  c12_info[105].fileTimeHi = 0U;
  c12_info[105].mFileTimeLo = 0U;
  c12_info[105].mFileTimeHi = 0U;
  c12_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c12_info[106].name = "eml_index_minus";
  c12_info[106].dominantType = "double";
  c12_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c12_info[106].fileTimeLo = 1286818778U;
  c12_info[106].fileTimeHi = 0U;
  c12_info[106].mFileTimeLo = 0U;
  c12_info[106].mFileTimeHi = 0U;
  c12_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c12_info[107].name = "eml_int_forloop_overflow_check";
  c12_info[107].dominantType = "";
  c12_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c12_info[107].fileTimeLo = 1346510340U;
  c12_info[107].fileTimeHi = 0U;
  c12_info[107].mFileTimeLo = 0U;
  c12_info[107].mFileTimeHi = 0U;
  c12_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c12_info[108].name = "eml_index_plus";
  c12_info[108].dominantType = "double";
  c12_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[108].fileTimeLo = 1286818778U;
  c12_info[108].fileTimeHi = 0U;
  c12_info[108].mFileTimeLo = 0U;
  c12_info[108].mFileTimeHi = 0U;
  c12_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c12_info[109].name = "eml_index_plus";
  c12_info[109].dominantType = "coder.internal.indexInt";
  c12_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[109].fileTimeLo = 1286818778U;
  c12_info[109].fileTimeHi = 0U;
  c12_info[109].mFileTimeLo = 0U;
  c12_info[109].mFileTimeHi = 0U;
  c12_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c12_info[110].name = "eml_warning";
  c12_info[110].dominantType = "char";
  c12_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c12_info[110].fileTimeLo = 1286818802U;
  c12_info[110].fileTimeHi = 0U;
  c12_info[110].mFileTimeLo = 0U;
  c12_info[110].mFileTimeHi = 0U;
  c12_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c12_info[111].name = "eml_scalar_eg";
  c12_info[111].dominantType = "double";
  c12_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[111].fileTimeLo = 1286818796U;
  c12_info[111].fileTimeHi = 0U;
  c12_info[111].mFileTimeLo = 0U;
  c12_info[111].mFileTimeHi = 0U;
  c12_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c12_info[112].name = "eml_int_forloop_overflow_check";
  c12_info[112].dominantType = "";
  c12_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c12_info[112].fileTimeLo = 1346510340U;
  c12_info[112].fileTimeHi = 0U;
  c12_info[112].mFileTimeLo = 0U;
  c12_info[112].mFileTimeHi = 0U;
  c12_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c12_info[113].name = "eml_xtrsm";
  c12_info[113].dominantType = "char";
  c12_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c12_info[113].fileTimeLo = 1299076778U;
  c12_info[113].fileTimeHi = 0U;
  c12_info[113].mFileTimeLo = 0U;
  c12_info[113].mFileTimeHi = 0U;
  c12_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c12_info[114].name = "eml_blas_inline";
  c12_info[114].dominantType = "";
  c12_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c12_info[114].fileTimeLo = 1299076768U;
  c12_info[114].fileTimeHi = 0U;
  c12_info[114].mFileTimeLo = 0U;
  c12_info[114].mFileTimeHi = 0U;
  c12_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c12_info[115].name = "mtimes";
  c12_info[115].dominantType = "double";
  c12_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c12_info[115].fileTimeLo = 1289519692U;
  c12_info[115].fileTimeHi = 0U;
  c12_info[115].mFileTimeLo = 0U;
  c12_info[115].mFileTimeHi = 0U;
  c12_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c12_info[116].name = "eml_index_class";
  c12_info[116].dominantType = "";
  c12_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[116].fileTimeLo = 1323170578U;
  c12_info[116].fileTimeHi = 0U;
  c12_info[116].mFileTimeLo = 0U;
  c12_info[116].mFileTimeHi = 0U;
  c12_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c12_info[117].name = "eml_scalar_eg";
  c12_info[117].dominantType = "double";
  c12_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[117].fileTimeLo = 1286818796U;
  c12_info[117].fileTimeHi = 0U;
  c12_info[117].mFileTimeLo = 0U;
  c12_info[117].mFileTimeHi = 0U;
  c12_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c12_info[118].name = "eml_refblas_xtrsm";
  c12_info[118].dominantType = "char";
  c12_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[118].fileTimeLo = 1299076786U;
  c12_info[118].fileTimeHi = 0U;
  c12_info[118].mFileTimeLo = 0U;
  c12_info[118].mFileTimeHi = 0U;
  c12_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[119].name = "eml_scalar_eg";
  c12_info[119].dominantType = "double";
  c12_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c12_info[119].fileTimeLo = 1286818796U;
  c12_info[119].fileTimeHi = 0U;
  c12_info[119].mFileTimeLo = 0U;
  c12_info[119].mFileTimeHi = 0U;
  c12_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[120].name = "eml_index_minus";
  c12_info[120].dominantType = "double";
  c12_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c12_info[120].fileTimeLo = 1286818778U;
  c12_info[120].fileTimeHi = 0U;
  c12_info[120].mFileTimeLo = 0U;
  c12_info[120].mFileTimeHi = 0U;
  c12_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[121].name = "eml_index_class";
  c12_info[121].dominantType = "";
  c12_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c12_info[121].fileTimeLo = 1323170578U;
  c12_info[121].fileTimeHi = 0U;
  c12_info[121].mFileTimeLo = 0U;
  c12_info[121].mFileTimeHi = 0U;
  c12_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[122].name = "eml_int_forloop_overflow_check";
  c12_info[122].dominantType = "";
  c12_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c12_info[122].fileTimeLo = 1346510340U;
  c12_info[122].fileTimeHi = 0U;
  c12_info[122].mFileTimeLo = 0U;
  c12_info[122].mFileTimeHi = 0U;
  c12_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[123].name = "eml_index_times";
  c12_info[123].dominantType = "coder.internal.indexInt";
  c12_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c12_info[123].fileTimeLo = 1286818780U;
  c12_info[123].fileTimeHi = 0U;
  c12_info[123].mFileTimeLo = 0U;
  c12_info[123].mFileTimeHi = 0U;
  c12_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[124].name = "eml_index_plus";
  c12_info[124].dominantType = "coder.internal.indexInt";
  c12_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[124].fileTimeLo = 1286818778U;
  c12_info[124].fileTimeHi = 0U;
  c12_info[124].mFileTimeLo = 0U;
  c12_info[124].mFileTimeHi = 0U;
  c12_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[125].name = "eml_index_plus";
  c12_info[125].dominantType = "double";
  c12_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c12_info[125].fileTimeLo = 1286818778U;
  c12_info[125].fileTimeHi = 0U;
  c12_info[125].mFileTimeLo = 0U;
  c12_info[125].mFileTimeHi = 0U;
  c12_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c12_info[126].name = "intmin";
  c12_info[126].dominantType = "char";
  c12_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c12_info[126].fileTimeLo = 1311255318U;
  c12_info[126].fileTimeHi = 0U;
  c12_info[126].mFileTimeLo = 0U;
  c12_info[126].mFileTimeHi = 0U;
  c12_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c12_info[127].name = "eml_div";
  c12_info[127].dominantType = "double";
  c12_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c12_info[127].fileTimeLo = 1313347810U;
  c12_info[127].fileTimeHi = 0U;
  c12_info[127].mFileTimeLo = 0U;
  c12_info[127].mFileTimeHi = 0U;
}

static void c12_eml_scalar_eg(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c12_eml_xgemm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_B[72], real_T c12_C[144], real_T
  c12_b_C[144])
{
  int32_T c12_i105;
  int32_T c12_i106;
  real_T c12_b_A[72];
  int32_T c12_i107;
  real_T c12_b_B[72];
  for (c12_i105 = 0; c12_i105 < 144; c12_i105++) {
    c12_b_C[c12_i105] = c12_C[c12_i105];
  }

  for (c12_i106 = 0; c12_i106 < 72; c12_i106++) {
    c12_b_A[c12_i106] = c12_A[c12_i106];
  }

  for (c12_i107 = 0; c12_i107 < 72; c12_i107++) {
    c12_b_B[c12_i107] = c12_B[c12_i107];
  }

  c12_c_eml_xgemm(chartInstance, c12_b_A, c12_b_B, c12_b_C);
}

static void c12_mrdivide(SFc12_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c12_A[72], real_T c12_B[144], real_T c12_y[72])
{
  int32_T c12_i108;
  int32_T c12_i109;
  int32_T c12_i110;
  int32_T c12_i111;
  real_T c12_b_A[144];
  int32_T c12_i112;
  int32_T c12_i113;
  int32_T c12_i114;
  int32_T c12_i115;
  real_T c12_b_B[72];
  int32_T c12_info;
  int32_T c12_ipiv[12];
  int32_T c12_b_info;
  int32_T c12_c_info;
  int32_T c12_d_info;
  int32_T c12_i;
  int32_T c12_b_i;
  int32_T c12_ip;
  int32_T c12_j;
  int32_T c12_b_j;
  real_T c12_temp;
  int32_T c12_i116;
  real_T c12_c_A[144];
  int32_T c12_i117;
  real_T c12_d_A[144];
  int32_T c12_i118;
  int32_T c12_i119;
  int32_T c12_i120;
  int32_T c12_i121;
  c12_i108 = 0;
  for (c12_i109 = 0; c12_i109 < 12; c12_i109++) {
    c12_i110 = 0;
    for (c12_i111 = 0; c12_i111 < 12; c12_i111++) {
      c12_b_A[c12_i111 + c12_i108] = c12_B[c12_i110 + c12_i109];
      c12_i110 += 12;
    }

    c12_i108 += 12;
  }

  c12_i112 = 0;
  for (c12_i113 = 0; c12_i113 < 6; c12_i113++) {
    c12_i114 = 0;
    for (c12_i115 = 0; c12_i115 < 12; c12_i115++) {
      c12_b_B[c12_i115 + c12_i112] = c12_A[c12_i114 + c12_i113];
      c12_i114 += 6;
    }

    c12_i112 += 12;
  }

  c12_b_eml_matlab_zgetrf(chartInstance, c12_b_A, c12_ipiv, &c12_info);
  c12_b_info = c12_info;
  c12_c_info = c12_b_info;
  c12_d_info = c12_c_info;
  if (c12_d_info > 0) {
    c12_eml_warning(chartInstance);
  }

  for (c12_i = 1; c12_i < 13; c12_i++) {
    c12_b_i = c12_i;
    if (c12_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c12_b_i), 1, 12, 1, 0) - 1] != c12_b_i) {
      c12_ip = c12_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c12_b_i), 1, 12, 1, 0) - 1];
      for (c12_j = 1; c12_j < 7; c12_j++) {
        c12_b_j = c12_j;
        c12_temp = c12_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c12_b_i), 1, 12, 1, 0) + 12 *
                            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c12_b_j), 1, 6, 2, 0) - 1)) - 1];
        c12_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c12_b_i), 1, 12, 1, 0) + 12 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c12_b_j), 1, 6, 2, 0) - 1)) - 1] = c12_b_B
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c12_ip), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c12_b_j), 1, 6, 2, 0)
             - 1)) - 1];
        c12_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c12_ip), 1, 12, 1, 0) + 12 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c12_b_j), 1, 6, 2, 0) - 1)) - 1] = c12_temp;
      }
    }
  }

  for (c12_i116 = 0; c12_i116 < 144; c12_i116++) {
    c12_c_A[c12_i116] = c12_b_A[c12_i116];
  }

  c12_c_eml_xtrsm(chartInstance, c12_c_A, c12_b_B);
  for (c12_i117 = 0; c12_i117 < 144; c12_i117++) {
    c12_d_A[c12_i117] = c12_b_A[c12_i117];
  }

  c12_d_eml_xtrsm(chartInstance, c12_d_A, c12_b_B);
  c12_i118 = 0;
  for (c12_i119 = 0; c12_i119 < 12; c12_i119++) {
    c12_i120 = 0;
    for (c12_i121 = 0; c12_i121 < 6; c12_i121++) {
      c12_y[c12_i121 + c12_i118] = c12_b_B[c12_i120 + c12_i119];
      c12_i120 += 12;
    }

    c12_i118 += 6;
  }
}

static void c12_realmin(SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c12_eps(SFc12_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c12_eml_matlab_zgetrf(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_b_A[144], int32_T c12_ipiv[12],
  int32_T *c12_info)
{
  int32_T c12_i122;
  for (c12_i122 = 0; c12_i122 < 144; c12_i122++) {
    c12_b_A[c12_i122] = c12_A[c12_i122];
  }

  c12_b_eml_matlab_zgetrf(chartInstance, c12_b_A, c12_ipiv, c12_info);
}

static void c12_check_forloop_overflow_error
  (SFc12_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c12_overflow)
{
  int32_T c12_i123;
  static char_T c12_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c12_u[34];
  const mxArray *c12_y = NULL;
  int32_T c12_i124;
  static char_T c12_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c12_b_u[23];
  const mxArray *c12_b_y = NULL;
  if (!c12_overflow) {
  } else {
    for (c12_i123 = 0; c12_i123 < 34; c12_i123++) {
      c12_u[c12_i123] = c12_cv0[c12_i123];
    }

    c12_y = NULL;
    sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c12_i124 = 0; c12_i124 < 23; c12_i124++) {
      c12_b_u[c12_i124] = c12_cv1[c12_i124];
    }

    c12_b_y = NULL;
    sf_mex_assign(&c12_b_y, sf_mex_create("y", c12_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c12_y, 14, c12_b_y));
  }
}

static void c12_eml_xger(SFc12_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c12_m, int32_T c12_n, real_T c12_alpha1, int32_T c12_ix0, int32_T
  c12_iy0, real_T c12_A[144], int32_T c12_ia0, real_T c12_b_A[144])
{
  int32_T c12_i125;
  for (c12_i125 = 0; c12_i125 < 144; c12_i125++) {
    c12_b_A[c12_i125] = c12_A[c12_i125];
  }

  c12_b_eml_xger(chartInstance, c12_m, c12_n, c12_alpha1, c12_ix0, c12_iy0,
                 c12_b_A, c12_ia0);
}

static void c12_eml_warning(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c12_i126;
  static char_T c12_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c12_u[27];
  const mxArray *c12_y = NULL;
  for (c12_i126 = 0; c12_i126 < 27; c12_i126++) {
    c12_u[c12_i126] = c12_varargin_1[c12_i126];
  }

  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", c12_u, 10, 0U, 1U, 0U, 2, 1, 27),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c12_y));
}

static void c12_eml_xtrsm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_B[72], real_T c12_b_B[72])
{
  int32_T c12_i127;
  int32_T c12_i128;
  real_T c12_b_A[144];
  for (c12_i127 = 0; c12_i127 < 72; c12_i127++) {
    c12_b_B[c12_i127] = c12_B[c12_i127];
  }

  for (c12_i128 = 0; c12_i128 < 144; c12_i128++) {
    c12_b_A[c12_i128] = c12_A[c12_i128];
  }

  c12_c_eml_xtrsm(chartInstance, c12_b_A, c12_b_B);
}

static void c12_below_threshold(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c12_b_eml_xtrsm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_B[72], real_T c12_b_B[72])
{
  int32_T c12_i129;
  int32_T c12_i130;
  real_T c12_b_A[144];
  for (c12_i129 = 0; c12_i129 < 72; c12_i129++) {
    c12_b_B[c12_i129] = c12_B[c12_i129];
  }

  for (c12_i130 = 0; c12_i130 < 144; c12_i130++) {
    c12_b_A[c12_i130] = c12_A[c12_i130];
  }

  c12_d_eml_xtrsm(chartInstance, c12_b_A, c12_b_B);
}

static void c12_b_eml_scalar_eg(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c12_b_eml_xgemm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_B[276], real_T c12_C[138], real_T
  c12_b_C[138])
{
  int32_T c12_i131;
  int32_T c12_i132;
  real_T c12_b_A[72];
  int32_T c12_i133;
  real_T c12_b_B[276];
  for (c12_i131 = 0; c12_i131 < 138; c12_i131++) {
    c12_b_C[c12_i131] = c12_C[c12_i131];
  }

  for (c12_i132 = 0; c12_i132 < 72; c12_i132++) {
    c12_b_A[c12_i132] = c12_A[c12_i132];
  }

  for (c12_i133 = 0; c12_i133 < 276; c12_i133++) {
    c12_b_B[c12_i133] = c12_B[c12_i133];
  }

  c12_d_eml_xgemm(chartInstance, c12_b_A, c12_b_B, c12_b_C);
}

static void c12_c_eml_scalar_eg(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static const mxArray *c12_j_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_u;
  const mxArray *c12_y = NULL;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_u = *(int32_T *)c12_inData;
  c12_y = NULL;
  sf_mex_assign(&c12_y, sf_mex_create("y", &c12_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c12_mxArrayOutData, c12_y, FALSE);
  return c12_mxArrayOutData;
}

static int32_T c12_h_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId)
{
  int32_T c12_y;
  int32_T c12_i134;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_u), &c12_i134, 1, 6, 0U, 0, 0U, 0);
  c12_y = c12_i134;
  sf_mex_destroy(&c12_u);
  return c12_y;
}

static void c12_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_b_sfEvent;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  int32_T c12_y;
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c12_b_sfEvent = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_y = c12_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_b_sfEvent),
    &c12_thisId);
  sf_mex_destroy(&c12_b_sfEvent);
  *(int32_T *)c12_outData = c12_y;
  sf_mex_destroy(&c12_mxArrayInData);
}

static uint8_T c12_i_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_b_is_active_c12_torqueBalancing2012b, const
  char_T *c12_identifier)
{
  uint8_T c12_y;
  emlrtMsgIdentifier c12_thisId;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_y = c12_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c12_b_is_active_c12_torqueBalancing2012b), &c12_thisId);
  sf_mex_destroy(&c12_b_is_active_c12_torqueBalancing2012b);
  return c12_y;
}

static uint8_T c12_j_emlrt_marshallIn(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c12_u, const emlrtMsgIdentifier *c12_parentId)
{
  uint8_T c12_y;
  uint8_T c12_u0;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_u), &c12_u0, 1, 3, 0U, 0, 0U, 0);
  c12_y = c12_u0;
  sf_mex_destroy(&c12_u);
  return c12_y;
}

static void c12_c_eml_xgemm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_B[72], real_T c12_C[144])
{
  real_T c12_alpha1;
  real_T c12_beta1;
  char_T c12_TRANSB;
  char_T c12_TRANSA;
  ptrdiff_t c12_m_t;
  ptrdiff_t c12_n_t;
  ptrdiff_t c12_k_t;
  ptrdiff_t c12_lda_t;
  ptrdiff_t c12_ldb_t;
  ptrdiff_t c12_ldc_t;
  double * c12_alpha1_t;
  double * c12_Aia0_t;
  double * c12_Bib0_t;
  double * c12_beta1_t;
  double * c12_Cic0_t;
  c12_alpha1 = 1.0;
  c12_beta1 = 0.0;
  c12_TRANSB = 'N';
  c12_TRANSA = 'N';
  c12_m_t = (ptrdiff_t)(12);
  c12_n_t = (ptrdiff_t)(12);
  c12_k_t = (ptrdiff_t)(6);
  c12_lda_t = (ptrdiff_t)(12);
  c12_ldb_t = (ptrdiff_t)(6);
  c12_ldc_t = (ptrdiff_t)(12);
  c12_alpha1_t = (double *)(&c12_alpha1);
  c12_Aia0_t = (double *)(&c12_A[0]);
  c12_Bib0_t = (double *)(&c12_B[0]);
  c12_beta1_t = (double *)(&c12_beta1);
  c12_Cic0_t = (double *)(&c12_C[0]);
  dgemm(&c12_TRANSA, &c12_TRANSB, &c12_m_t, &c12_n_t, &c12_k_t, c12_alpha1_t,
        c12_Aia0_t, &c12_lda_t, c12_Bib0_t, &c12_ldb_t, c12_beta1_t, c12_Cic0_t,
        &c12_ldc_t);
}

static void c12_b_eml_matlab_zgetrf(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], int32_T c12_ipiv[12], int32_T *c12_info)
{
  int32_T c12_i135;
  int32_T c12_j;
  int32_T c12_b_j;
  int32_T c12_a;
  int32_T c12_jm1;
  int32_T c12_b;
  int32_T c12_mmj;
  int32_T c12_b_a;
  int32_T c12_c;
  int32_T c12_b_b;
  int32_T c12_jj;
  int32_T c12_c_a;
  int32_T c12_jp1j;
  int32_T c12_d_a;
  int32_T c12_b_c;
  int32_T c12_n;
  int32_T c12_ix0;
  int32_T c12_b_n;
  int32_T c12_b_ix0;
  int32_T c12_c_n;
  int32_T c12_c_ix0;
  int32_T c12_idxmax;
  int32_T c12_ix;
  real_T c12_x;
  real_T c12_b_x;
  real_T c12_c_x;
  real_T c12_y;
  real_T c12_d_x;
  real_T c12_e_x;
  real_T c12_b_y;
  real_T c12_smax;
  int32_T c12_d_n;
  int32_T c12_c_b;
  int32_T c12_d_b;
  boolean_T c12_overflow;
  int32_T c12_k;
  int32_T c12_b_k;
  int32_T c12_e_a;
  real_T c12_f_x;
  real_T c12_g_x;
  real_T c12_h_x;
  real_T c12_c_y;
  real_T c12_i_x;
  real_T c12_j_x;
  real_T c12_d_y;
  real_T c12_s;
  int32_T c12_f_a;
  int32_T c12_jpiv_offset;
  int32_T c12_g_a;
  int32_T c12_e_b;
  int32_T c12_jpiv;
  int32_T c12_h_a;
  int32_T c12_f_b;
  int32_T c12_c_c;
  int32_T c12_g_b;
  int32_T c12_jrow;
  int32_T c12_i_a;
  int32_T c12_h_b;
  int32_T c12_jprow;
  int32_T c12_d_ix0;
  int32_T c12_iy0;
  int32_T c12_e_ix0;
  int32_T c12_b_iy0;
  int32_T c12_f_ix0;
  int32_T c12_c_iy0;
  int32_T c12_b_ix;
  int32_T c12_iy;
  int32_T c12_c_k;
  real_T c12_temp;
  int32_T c12_j_a;
  int32_T c12_k_a;
  int32_T c12_b_jp1j;
  int32_T c12_l_a;
  int32_T c12_d_c;
  int32_T c12_m_a;
  int32_T c12_i_b;
  int32_T c12_i136;
  int32_T c12_n_a;
  int32_T c12_j_b;
  int32_T c12_o_a;
  int32_T c12_k_b;
  boolean_T c12_b_overflow;
  int32_T c12_i;
  int32_T c12_b_i;
  real_T c12_k_x;
  real_T c12_e_y;
  real_T c12_z;
  int32_T c12_l_b;
  int32_T c12_e_c;
  int32_T c12_p_a;
  int32_T c12_f_c;
  int32_T c12_q_a;
  int32_T c12_g_c;
  int32_T c12_m;
  int32_T c12_e_n;
  int32_T c12_g_ix0;
  int32_T c12_d_iy0;
  int32_T c12_ia0;
  real_T c12_d1;
  c12_realmin(chartInstance);
  c12_eps(chartInstance);
  for (c12_i135 = 0; c12_i135 < 12; c12_i135++) {
    c12_ipiv[c12_i135] = 1 + c12_i135;
  }

  *c12_info = 0;
  for (c12_j = 1; c12_j < 12; c12_j++) {
    c12_b_j = c12_j;
    c12_a = c12_b_j - 1;
    c12_jm1 = c12_a;
    c12_b = c12_b_j;
    c12_mmj = 12 - c12_b;
    c12_b_a = c12_jm1;
    c12_c = c12_b_a * 13;
    c12_b_b = c12_c + 1;
    c12_jj = c12_b_b;
    c12_c_a = c12_jj + 1;
    c12_jp1j = c12_c_a;
    c12_d_a = c12_mmj;
    c12_b_c = c12_d_a;
    c12_n = c12_b_c + 1;
    c12_ix0 = c12_jj;
    c12_b_n = c12_n;
    c12_b_ix0 = c12_ix0;
    c12_c_n = c12_b_n;
    c12_c_ix0 = c12_b_ix0;
    if (c12_c_n < 1) {
      c12_idxmax = 0;
    } else {
      c12_idxmax = 1;
      if (c12_c_n > 1) {
        c12_ix = c12_c_ix0;
        c12_x = c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c12_ix), 1, 144, 1, 0) - 1];
        c12_b_x = c12_x;
        c12_c_x = c12_b_x;
        c12_y = muDoubleScalarAbs(c12_c_x);
        c12_d_x = 0.0;
        c12_e_x = c12_d_x;
        c12_b_y = muDoubleScalarAbs(c12_e_x);
        c12_smax = c12_y + c12_b_y;
        c12_d_n = c12_c_n;
        c12_c_b = c12_d_n;
        c12_d_b = c12_c_b;
        if (2 > c12_d_b) {
          c12_overflow = FALSE;
        } else {
          c12_overflow = (c12_d_b > 2147483646);
        }

        if (c12_overflow) {
          c12_check_forloop_overflow_error(chartInstance, c12_overflow);
        }

        for (c12_k = 2; c12_k <= c12_d_n; c12_k++) {
          c12_b_k = c12_k;
          c12_e_a = c12_ix + 1;
          c12_ix = c12_e_a;
          c12_f_x = c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c12_ix), 1, 144, 1, 0) - 1];
          c12_g_x = c12_f_x;
          c12_h_x = c12_g_x;
          c12_c_y = muDoubleScalarAbs(c12_h_x);
          c12_i_x = 0.0;
          c12_j_x = c12_i_x;
          c12_d_y = muDoubleScalarAbs(c12_j_x);
          c12_s = c12_c_y + c12_d_y;
          if (c12_s > c12_smax) {
            c12_idxmax = c12_b_k;
            c12_smax = c12_s;
          }
        }
      }
    }

    c12_f_a = c12_idxmax - 1;
    c12_jpiv_offset = c12_f_a;
    c12_g_a = c12_jj;
    c12_e_b = c12_jpiv_offset;
    c12_jpiv = c12_g_a + c12_e_b;
    if (c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c12_jpiv), 1, 144, 1, 0) - 1] != 0.0) {
      if (c12_jpiv_offset != 0) {
        c12_h_a = c12_b_j;
        c12_f_b = c12_jpiv_offset;
        c12_c_c = c12_h_a + c12_f_b;
        c12_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c12_b_j), 1, 12, 1, 0) - 1] = c12_c_c;
        c12_g_b = c12_jm1 + 1;
        c12_jrow = c12_g_b;
        c12_i_a = c12_jrow;
        c12_h_b = c12_jpiv_offset;
        c12_jprow = c12_i_a + c12_h_b;
        c12_d_ix0 = c12_jrow;
        c12_iy0 = c12_jprow;
        c12_e_ix0 = c12_d_ix0;
        c12_b_iy0 = c12_iy0;
        c12_f_ix0 = c12_e_ix0;
        c12_c_iy0 = c12_b_iy0;
        c12_b_ix = c12_f_ix0;
        c12_iy = c12_c_iy0;
        for (c12_c_k = 1; c12_c_k < 13; c12_c_k++) {
          c12_temp = c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c12_b_ix), 1, 144, 1, 0) - 1];
          c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c12_b_ix), 1, 144, 1, 0) - 1] =
            c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c12_iy), 1, 144, 1, 0) - 1];
          c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c12_iy), 1, 144, 1, 0) - 1] = c12_temp;
          c12_j_a = c12_b_ix + 12;
          c12_b_ix = c12_j_a;
          c12_k_a = c12_iy + 12;
          c12_iy = c12_k_a;
        }
      }

      c12_b_jp1j = c12_jp1j;
      c12_l_a = c12_mmj;
      c12_d_c = c12_l_a;
      c12_m_a = c12_jp1j;
      c12_i_b = c12_d_c - 1;
      c12_i136 = c12_m_a + c12_i_b;
      c12_n_a = c12_b_jp1j;
      c12_j_b = c12_i136;
      c12_o_a = c12_n_a;
      c12_k_b = c12_j_b;
      if (c12_o_a > c12_k_b) {
        c12_b_overflow = FALSE;
      } else {
        c12_b_overflow = (c12_k_b > 2147483646);
      }

      if (c12_b_overflow) {
        c12_check_forloop_overflow_error(chartInstance, c12_b_overflow);
      }

      for (c12_i = c12_b_jp1j; c12_i <= c12_i136; c12_i++) {
        c12_b_i = c12_i;
        c12_k_x = c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c12_b_i), 1, 144, 1, 0) - 1];
        c12_e_y = c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c12_jj), 1, 144, 1, 0) - 1];
        c12_z = c12_k_x / c12_e_y;
        c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c12_b_i), 1, 144, 1, 0) - 1] = c12_z;
      }
    } else {
      *c12_info = c12_b_j;
    }

    c12_l_b = c12_b_j;
    c12_e_c = 12 - c12_l_b;
    c12_p_a = c12_jj;
    c12_f_c = c12_p_a;
    c12_q_a = c12_jj;
    c12_g_c = c12_q_a;
    c12_m = c12_mmj;
    c12_e_n = c12_e_c;
    c12_g_ix0 = c12_jp1j;
    c12_d_iy0 = c12_f_c + 12;
    c12_ia0 = c12_g_c + 13;
    c12_d1 = -1.0;
    c12_b_eml_xger(chartInstance, c12_m, c12_e_n, c12_d1, c12_g_ix0, c12_d_iy0,
                   c12_A, c12_ia0);
  }

  if (*c12_info == 0) {
    if (!(c12_A[143] != 0.0)) {
      *c12_info = 12;
    }
  }
}

static void c12_b_eml_xger(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c12_m, int32_T c12_n, real_T c12_alpha1, int32_T
  c12_ix0, int32_T c12_iy0, real_T c12_A[144], int32_T c12_ia0)
{
  int32_T c12_b_m;
  int32_T c12_b_n;
  real_T c12_b_alpha1;
  int32_T c12_b_ix0;
  int32_T c12_b_iy0;
  int32_T c12_b_ia0;
  int32_T c12_c_m;
  int32_T c12_c_n;
  real_T c12_c_alpha1;
  int32_T c12_c_ix0;
  int32_T c12_c_iy0;
  int32_T c12_c_ia0;
  int32_T c12_d_m;
  int32_T c12_d_n;
  real_T c12_d_alpha1;
  int32_T c12_d_ix0;
  int32_T c12_d_iy0;
  int32_T c12_d_ia0;
  int32_T c12_ixstart;
  int32_T c12_a;
  int32_T c12_jA;
  int32_T c12_jy;
  int32_T c12_e_n;
  int32_T c12_b;
  int32_T c12_b_b;
  boolean_T c12_overflow;
  int32_T c12_j;
  real_T c12_yjy;
  real_T c12_temp;
  int32_T c12_ix;
  int32_T c12_c_b;
  int32_T c12_i137;
  int32_T c12_b_a;
  int32_T c12_d_b;
  int32_T c12_i138;
  int32_T c12_c_a;
  int32_T c12_e_b;
  int32_T c12_d_a;
  int32_T c12_f_b;
  boolean_T c12_b_overflow;
  int32_T c12_ijA;
  int32_T c12_b_ijA;
  int32_T c12_e_a;
  int32_T c12_f_a;
  int32_T c12_g_a;
  c12_b_m = c12_m;
  c12_b_n = c12_n;
  c12_b_alpha1 = c12_alpha1;
  c12_b_ix0 = c12_ix0;
  c12_b_iy0 = c12_iy0;
  c12_b_ia0 = c12_ia0;
  c12_c_m = c12_b_m;
  c12_c_n = c12_b_n;
  c12_c_alpha1 = c12_b_alpha1;
  c12_c_ix0 = c12_b_ix0;
  c12_c_iy0 = c12_b_iy0;
  c12_c_ia0 = c12_b_ia0;
  c12_d_m = c12_c_m;
  c12_d_n = c12_c_n;
  c12_d_alpha1 = c12_c_alpha1;
  c12_d_ix0 = c12_c_ix0;
  c12_d_iy0 = c12_c_iy0;
  c12_d_ia0 = c12_c_ia0;
  if (c12_d_alpha1 == 0.0) {
  } else {
    c12_ixstart = c12_d_ix0;
    c12_a = c12_d_ia0 - 1;
    c12_jA = c12_a;
    c12_jy = c12_d_iy0;
    c12_e_n = c12_d_n;
    c12_b = c12_e_n;
    c12_b_b = c12_b;
    if (1 > c12_b_b) {
      c12_overflow = FALSE;
    } else {
      c12_overflow = (c12_b_b > 2147483646);
    }

    if (c12_overflow) {
      c12_check_forloop_overflow_error(chartInstance, c12_overflow);
    }

    for (c12_j = 1; c12_j <= c12_e_n; c12_j++) {
      c12_yjy = c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c12_jy), 1, 144, 1, 0) - 1];
      if (c12_yjy != 0.0) {
        c12_temp = c12_yjy * c12_d_alpha1;
        c12_ix = c12_ixstart;
        c12_c_b = c12_jA + 1;
        c12_i137 = c12_c_b;
        c12_b_a = c12_d_m;
        c12_d_b = c12_jA;
        c12_i138 = c12_b_a + c12_d_b;
        c12_c_a = c12_i137;
        c12_e_b = c12_i138;
        c12_d_a = c12_c_a;
        c12_f_b = c12_e_b;
        if (c12_d_a > c12_f_b) {
          c12_b_overflow = FALSE;
        } else {
          c12_b_overflow = (c12_f_b > 2147483646);
        }

        if (c12_b_overflow) {
          c12_check_forloop_overflow_error(chartInstance, c12_b_overflow);
        }

        for (c12_ijA = c12_i137; c12_ijA <= c12_i138; c12_ijA++) {
          c12_b_ijA = c12_ijA;
          c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c12_b_ijA), 1, 144, 1, 0) - 1] =
            c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c12_b_ijA), 1, 144, 1, 0) - 1] +
            c12_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c12_ix), 1, 144, 1, 0) - 1] * c12_temp;
          c12_e_a = c12_ix + 1;
          c12_ix = c12_e_a;
        }
      }

      c12_f_a = c12_jy + 12;
      c12_jy = c12_f_a;
      c12_g_a = c12_jA + 12;
      c12_jA = c12_g_a;
    }
  }
}

static void c12_c_eml_xtrsm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_B[72])
{
  real_T c12_alpha1;
  char_T c12_DIAGA;
  char_T c12_TRANSA;
  char_T c12_UPLO;
  char_T c12_SIDE;
  ptrdiff_t c12_m_t;
  ptrdiff_t c12_n_t;
  ptrdiff_t c12_lda_t;
  ptrdiff_t c12_ldb_t;
  double * c12_Aia0_t;
  double * c12_Bib0_t;
  double * c12_alpha1_t;
  c12_below_threshold(chartInstance);
  c12_alpha1 = 1.0;
  c12_DIAGA = 'U';
  c12_TRANSA = 'N';
  c12_UPLO = 'L';
  c12_SIDE = 'L';
  c12_m_t = (ptrdiff_t)(12);
  c12_n_t = (ptrdiff_t)(6);
  c12_lda_t = (ptrdiff_t)(12);
  c12_ldb_t = (ptrdiff_t)(12);
  c12_Aia0_t = (double *)(&c12_A[0]);
  c12_Bib0_t = (double *)(&c12_B[0]);
  c12_alpha1_t = (double *)(&c12_alpha1);
  dtrsm(&c12_SIDE, &c12_UPLO, &c12_TRANSA, &c12_DIAGA, &c12_m_t, &c12_n_t,
        c12_alpha1_t, c12_Aia0_t, &c12_lda_t, c12_Bib0_t, &c12_ldb_t);
}

static void c12_d_eml_xtrsm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[144], real_T c12_B[72])
{
  real_T c12_alpha1;
  char_T c12_DIAGA;
  char_T c12_TRANSA;
  char_T c12_UPLO;
  char_T c12_SIDE;
  ptrdiff_t c12_m_t;
  ptrdiff_t c12_n_t;
  ptrdiff_t c12_lda_t;
  ptrdiff_t c12_ldb_t;
  double * c12_Aia0_t;
  double * c12_Bib0_t;
  double * c12_alpha1_t;
  c12_below_threshold(chartInstance);
  c12_alpha1 = 1.0;
  c12_DIAGA = 'N';
  c12_TRANSA = 'N';
  c12_UPLO = 'U';
  c12_SIDE = 'L';
  c12_m_t = (ptrdiff_t)(12);
  c12_n_t = (ptrdiff_t)(6);
  c12_lda_t = (ptrdiff_t)(12);
  c12_ldb_t = (ptrdiff_t)(12);
  c12_Aia0_t = (double *)(&c12_A[0]);
  c12_Bib0_t = (double *)(&c12_B[0]);
  c12_alpha1_t = (double *)(&c12_alpha1);
  dtrsm(&c12_SIDE, &c12_UPLO, &c12_TRANSA, &c12_DIAGA, &c12_m_t, &c12_n_t,
        c12_alpha1_t, c12_Aia0_t, &c12_lda_t, c12_Bib0_t, &c12_ldb_t);
}

static void c12_d_eml_xgemm(SFc12_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c12_A[72], real_T c12_B[276], real_T c12_C[138])
{
  real_T c12_alpha1;
  real_T c12_beta1;
  char_T c12_TRANSB;
  char_T c12_TRANSA;
  ptrdiff_t c12_m_t;
  ptrdiff_t c12_n_t;
  ptrdiff_t c12_k_t;
  ptrdiff_t c12_lda_t;
  ptrdiff_t c12_ldb_t;
  ptrdiff_t c12_ldc_t;
  double * c12_alpha1_t;
  double * c12_Aia0_t;
  double * c12_Bib0_t;
  double * c12_beta1_t;
  double * c12_Cic0_t;
  c12_alpha1 = 1.0;
  c12_beta1 = 0.0;
  c12_TRANSB = 'N';
  c12_TRANSA = 'N';
  c12_m_t = (ptrdiff_t)(6);
  c12_n_t = (ptrdiff_t)(23);
  c12_k_t = (ptrdiff_t)(12);
  c12_lda_t = (ptrdiff_t)(6);
  c12_ldb_t = (ptrdiff_t)(12);
  c12_ldc_t = (ptrdiff_t)(6);
  c12_alpha1_t = (double *)(&c12_alpha1);
  c12_Aia0_t = (double *)(&c12_A[0]);
  c12_Bib0_t = (double *)(&c12_B[0]);
  c12_beta1_t = (double *)(&c12_beta1);
  c12_Cic0_t = (double *)(&c12_C[0]);
  dgemm(&c12_TRANSA, &c12_TRANSB, &c12_m_t, &c12_n_t, &c12_k_t, c12_alpha1_t,
        c12_Aia0_t, &c12_lda_t, c12_Bib0_t, &c12_ldb_t, c12_beta1_t, c12_Cic0_t,
        &c12_ldc_t);
}

static void init_dsm_address_info(SFc12_torqueBalancing2012bInstanceStruct
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

void sf_c12_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3086969071U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3087569955U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1615034229U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3004227463U);
}

mxArray *sf_c12_torqueBalancing2012b_get_autoinheritance_info(void)
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

mxArray *sf_c12_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c12_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c12_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c12_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           12,
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
            1.0,0,0,(MexFcnForType)c12_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c12_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c12_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c12_sf_marshallOut,(MexInFcnForType)
            c12_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c12_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c12_b_sf_marshallOut,(MexInFcnForType)
          c12_b_sf_marshallIn);

        {
          real_T (*c12_JcLeftFoot)[174];
          real_T (*c12_JcRightFoot)[174];
          real_T (*c12_activeFeetConstraints)[2];
          real_T (*c12_y)[6];
          real_T (*c12_qD)[23];
          c12_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
          c12_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c12_activeFeetConstraints = (real_T (*)[2])ssGetInputPortSignal
            (chartInstance->S, 2);
          c12_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal
            (chartInstance->S, 1);
          c12_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal
            (chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c12_JcLeftFoot);
          _SFD_SET_DATA_VALUE_PTR(1U, *c12_JcRightFoot);
          _SFD_SET_DATA_VALUE_PTR(2U, *c12_activeFeetConstraints);
          _SFD_SET_DATA_VALUE_PTR(3U, *c12_y);
          _SFD_SET_DATA_VALUE_PTR(4U, *c12_qD);
          _SFD_SET_DATA_VALUE_PTR(5U, &chartInstance->c12_reg);
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

static void sf_opaque_initialize_c12_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc12_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c12_torqueBalancing2012b
    ((SFc12_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c12_torqueBalancing2012b((SFc12_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c12_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c12_torqueBalancing2012b((SFc12_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c12_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c12_torqueBalancing2012b((SFc12_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c12_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c12_torqueBalancing2012b((SFc12_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c12_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c12_torqueBalancing2012b
    ((SFc12_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c12_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c12_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c12_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c12_torqueBalancing2012b
    ((SFc12_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c12_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c12_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c12_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c12_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c12_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc12_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c12_torqueBalancing2012b((SFc12_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc12_torqueBalancing2012b
    ((SFc12_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c12_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c12_torqueBalancing2012b
      ((SFc12_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c12_torqueBalancing2012b(SimStruct *S)
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
      12);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,12,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,12,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,12);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,12,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,12,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,12);
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

static void mdlRTW_c12_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c12_torqueBalancing2012b(SimStruct *S)
{
  SFc12_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc12_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc12_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc12_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c12_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c12_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c12_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c12_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c12_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c12_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c12_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c12_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c12_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c12_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c12_torqueBalancing2012b;
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

void c12_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c12_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c12_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c12_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c12_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
