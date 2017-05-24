/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c3_torqueBalancing2012b.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "torqueBalancing2012b_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c3_debug_family_names[6] = { "nargin", "nargout", "state",
  "sm", "thresholdContactLeft", "thresholdContactRight" };

/* Function Declarations */
static void initialize_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c3_st);
static void finalize_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c3_torqueBalancing2012b(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static real_T c3_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_thresholdContactRight, const char_T
  *c3_identifier);
static real_T c3_b_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_c_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_struct_rUGQ0INmvPpaxIctEGl5sE *c3_y);
static boolean_T c3_d_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_e_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_struct_DnBdbfPNxiIjhNOyZMmfsE *c3_y);
static void c3_f_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[39]);
static c3_struct_KJR2itYvhBuAkuR6dKZHUC c3_g_emlrt_marshallIn
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c3_u,
   const emlrtMsgIdentifier *c3_parentId);
static void c3_h_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_struct_0U0wBk2LiR1OqsMsUngxdD *c3_y);
static void c3_i_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[299]);
static void c3_j_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[192]);
static void c3_k_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[72]);
static void c3_l_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[13]);
static void c3_m_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8]);
static void c3_n_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_struct_9LpOi5JXaV67jTuay8hWaH *c3_y);
static void c3_o_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[24]);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_p_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_q_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_torqueBalancing2012b, const
  char_T *c3_identifier);
static uint8_T c3_r_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void init_dsm_address_info(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c3_is_active_c3_torqueBalancing2012b = 0U;
}

static void initialize_params_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c3_m0 = NULL;
  const mxArray *c3_mxField;
  c3_struct_rUGQ0INmvPpaxIctEGl5sE c3_r0;
  const mxArray *c3_m1 = NULL;
  const mxArray *c3_b_mxField;
  const mxArray *c3_m2 = NULL;
  const mxArray *c3_c_mxField;
  const mxArray *c3_m3 = NULL;
  const mxArray *c3_d_mxField;
  const mxArray *c3_m4 = NULL;
  const mxArray *c3_e_mxField;
  sf_set_error_prefix_string(
    "Error evaluating data 'sm' in the parent workspace.\n");
  c3_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c3_mxField = sf_mex_getfield(c3_m0, "skipYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.skipYoga, 1, 11, 0U,
                      0, 0U, 0);
  c3_mxField = sf_mex_getfield(c3_m0, "demoOnlyRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.demoOnlyRightFoot, 1,
                      11, 0U, 0, 0U, 0);
  c3_mxField = sf_mex_getfield(c3_m0, "yogaAlsoOnRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.yogaAlsoOnRightFoot,
                      1, 11, 0U, 0, 0U, 0);
  c3_mxField = sf_mex_getfield(c3_m0, "yogaInLoop", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.yogaInLoop, 1, 11, 0U,
                      0, 0U, 0);
  c3_mxField = sf_mex_getfield(c3_m0, "com", "sm", 0);
  c3_m1 = sf_mex_dup(c3_mxField);
  c3_b_mxField = sf_mex_getfield(c3_m1, "threshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_b_mxField), &c3_r0.com.threshold, 1, 0,
                      0U, 0, 0U, 0);
  c3_b_mxField = sf_mex_getfield(c3_m1, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_b_mxField), c3_r0.com.states, 1, 0, 0U,
                      1, 0U, 2, 13, 3);
  sf_mex_destroy(&c3_m1);
  c3_mxField = sf_mex_getfield(c3_m0, "wrench", "sm", 0);
  c3_m2 = sf_mex_dup(c3_mxField);
  c3_c_mxField = sf_mex_getfield(c3_m2, "thresholdContactOn", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_c_mxField),
                      &c3_r0.wrench.thresholdContactOn, 1, 0, 0U, 0, 0U, 0);
  c3_c_mxField = sf_mex_getfield(c3_m2, "thresholdContactOff", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_c_mxField),
                      &c3_r0.wrench.thresholdContactOff, 1, 0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c3_m2);
  c3_mxField = sf_mex_getfield(c3_m0, "joints", "sm", 0);
  c3_m3 = sf_mex_dup(c3_mxField);
  c3_d_mxField = sf_mex_getfield(c3_m3, "thresholdNotInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_d_mxField),
                      &c3_r0.joints.thresholdNotInContact, 1, 0, 0U, 0, 0U, 0);
  c3_d_mxField = sf_mex_getfield(c3_m3, "thresholdInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_d_mxField),
                      &c3_r0.joints.thresholdInContact, 1, 0, 0U, 0, 0U, 0);
  c3_d_mxField = sf_mex_getfield(c3_m3, "pauseTimeLastPostureL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_d_mxField),
                      &c3_r0.joints.pauseTimeLastPostureL, 1, 0, 0U, 0, 0U, 0);
  c3_d_mxField = sf_mex_getfield(c3_m3, "pauseTimeLastPostureR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_d_mxField),
                      &c3_r0.joints.pauseTimeLastPostureR, 1, 0, 0U, 0, 0U, 0);
  c3_d_mxField = sf_mex_getfield(c3_m3, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_d_mxField), c3_r0.joints.states, 1, 0,
                      0U, 1, 0U, 2, 13, 23);
  c3_d_mxField = sf_mex_getfield(c3_m3, "pointsL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_d_mxField), c3_r0.joints.pointsL, 1, 0,
                      0U, 1, 0U, 2, 8, 24);
  c3_d_mxField = sf_mex_getfield(c3_m3, "pointsR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_d_mxField), c3_r0.joints.pointsR, 1, 0,
                      0U, 1, 0U, 2, 8, 24);
  c3_d_mxField = sf_mex_getfield(c3_m3, "standUpPositions", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_d_mxField),
                      c3_r0.joints.standUpPositions, 1, 0, 0U, 1, 0U, 2, 8, 9);
  sf_mex_destroy(&c3_m3);
  c3_mxField = sf_mex_getfield(c3_m0, "stateAt0", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.stateAt0, 1, 0, 0U, 0,
                      0U, 0);
  c3_mxField = sf_mex_getfield(c3_m0, "DT", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.DT, 1, 0, 0U, 0, 0U,
                      0);
  c3_mxField = sf_mex_getfield(c3_m0, "waitingTimeAfterYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.waitingTimeAfterYoga,
                      1, 0, 0U, 0, 0U, 0);
  c3_mxField = sf_mex_getfield(c3_m0, "jointsSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), c3_r0.jointsSmoothingTimes,
                      1, 0, 0U, 1, 0U, 2, 13, 1);
  c3_mxField = sf_mex_getfield(c3_m0, "tBalancing", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.tBalancing, 1, 0, 0U,
                      0, 0U, 0);
  c3_mxField = sf_mex_getfield(c3_m0, "alsoSitDown", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), &c3_r0.alsoSitDown, 1, 11,
                      0U, 0, 0U, 0);
  c3_mxField = sf_mex_getfield(c3_m0, "jointsAndCoMSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField),
                      c3_r0.jointsAndCoMSmoothingTimes, 1, 0, 0U, 1, 0U, 2, 8, 1);
  c3_mxField = sf_mex_getfield(c3_m0, "CoM", "sm", 0);
  c3_m4 = sf_mex_dup(c3_mxField);
  c3_e_mxField = sf_mex_getfield(c3_m4, "standUpDeltaCoM", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_e_mxField), c3_r0.CoM.standUpDeltaCoM,
                      1, 0, 0U, 1, 0U, 2, 8, 3);
  sf_mex_destroy(&c3_m4);
  c3_mxField = sf_mex_getfield(c3_m0, "LwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), c3_r0.LwrenchThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  c3_mxField = sf_mex_getfield(c3_m0, "RwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), c3_r0.RwrenchThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  c3_mxField = sf_mex_getfield(c3_m0, "RArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), c3_r0.RArmThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  c3_mxField = sf_mex_getfield(c3_m0, "LArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c3_mxField), c3_r0.LArmThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  sf_mex_destroy(&c3_m0);
  chartInstance->c3_sm = c3_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c3_update_debugger_state_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_b_hoistedGlobal;
  real_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  uint8_T c3_c_hoistedGlobal;
  uint8_T c3_c_u;
  const mxArray *c3_d_y = NULL;
  real_T *c3_thresholdContactLeft;
  real_T *c3_thresholdContactRight;
  c3_thresholdContactRight = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c3_thresholdContactLeft = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellarray(3), FALSE);
  c3_hoistedGlobal = *c3_thresholdContactLeft;
  c3_u = c3_hoistedGlobal;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_b_hoistedGlobal = *c3_thresholdContactRight;
  c3_b_u = c3_b_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_c_hoistedGlobal = chartInstance->c3_is_active_c3_torqueBalancing2012b;
  c3_c_u = c3_c_hoistedGlobal;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 2, c3_d_y);
  sf_mex_assign(&c3_st, c3_y, FALSE);
  return c3_st;
}

static void set_sim_state_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T *c3_thresholdContactLeft;
  real_T *c3_thresholdContactRight;
  c3_thresholdContactRight = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c3_thresholdContactLeft = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = TRUE;
  c3_u = sf_mex_dup(c3_st);
  *c3_thresholdContactLeft = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 0)), "thresholdContactLeft");
  *c3_thresholdContactRight = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 1)), "thresholdContactRight");
  chartInstance->c3_is_active_c3_torqueBalancing2012b = c3_q_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 2)),
     "is_active_c3_torqueBalancing2012b");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c3_torqueBalancing2012b(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  real_T c3_hoistedGlobal;
  real_T c3_state;
  c3_struct_rUGQ0INmvPpaxIctEGl5sE c3_b_sm;
  uint32_T c3_debug_family_var_map[6];
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 2.0;
  real_T c3_thresholdContactLeft;
  real_T c3_thresholdContactRight;
  real_T *c3_b_state;
  real_T *c3_b_thresholdContactLeft;
  real_T *c3_b_thresholdContactRight;
  c3_b_thresholdContactRight = (real_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  c3_b_thresholdContactLeft = (real_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  c3_b_state = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c3_b_state, 0U);
  _SFD_DATA_RANGE_CHECK(*c3_b_thresholdContactLeft, 1U);
  _SFD_DATA_RANGE_CHECK(*c3_b_thresholdContactRight, 2U);
  chartInstance->c3_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *c3_b_state;
  c3_state = c3_hoistedGlobal;
  c3_b_sm = chartInstance->c3_sm;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_state, 2U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_sm, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_thresholdContactLeft, 4U,
    c3_sf_marshallOut, c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_thresholdContactRight, 5U,
    c3_sf_marshallOut, c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_thresholdContactLeft = c3_b_sm.LwrenchThreshold[(int32_T)(real_T)
    _SFD_EML_ARRAY_BOUNDS_CHECK("sm.LwrenchThreshold", (int32_T)
    _SFD_INTEGER_CHECK("state", c3_state), 1, 8, 1, 0) - 1];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_thresholdContactRight = c3_b_sm.RwrenchThreshold[(int32_T)(real_T)
    _SFD_EML_ARRAY_BOUNDS_CHECK("sm.RwrenchThreshold", (int32_T)
    _SFD_INTEGER_CHECK("state", c3_state), 1, 8, 1, 0) - 1];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
  *c3_b_thresholdContactLeft = c3_thresholdContactLeft;
  *c3_b_thresholdContactRight = c3_thresholdContactRight;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc3_torqueBalancing2012b
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber)
{
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc3_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_thresholdContactRight, const char_T
  *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_thresholdContactRight), &c3_thisId);
  sf_mex_destroy(&c3_thresholdContactRight);
  return c3_y;
}

static real_T c3_b_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_thresholdContactRight;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc3_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c3_thresholdContactRight = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_thresholdContactRight), &c3_thisId);
  sf_mex_destroy(&c3_thresholdContactRight);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData;
  c3_struct_rUGQ0INmvPpaxIctEGl5sE c3_u;
  const mxArray *c3_y = NULL;
  boolean_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  boolean_T c3_c_u;
  const mxArray *c3_c_y = NULL;
  boolean_T c3_d_u;
  const mxArray *c3_d_y = NULL;
  boolean_T c3_e_u;
  const mxArray *c3_e_y = NULL;
  c3_struct_DnBdbfPNxiIjhNOyZMmfsE c3_f_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_g_u;
  const mxArray *c3_g_y = NULL;
  int32_T c3_i0;
  real_T c3_h_u[39];
  const mxArray *c3_h_y = NULL;
  c3_struct_KJR2itYvhBuAkuR6dKZHUC c3_i_u;
  const mxArray *c3_i_y = NULL;
  real_T c3_j_u;
  const mxArray *c3_j_y = NULL;
  real_T c3_k_u;
  const mxArray *c3_k_y = NULL;
  c3_struct_0U0wBk2LiR1OqsMsUngxdD c3_l_u;
  const mxArray *c3_l_y = NULL;
  real_T c3_m_u;
  const mxArray *c3_m_y = NULL;
  real_T c3_n_u;
  const mxArray *c3_n_y = NULL;
  real_T c3_o_u;
  const mxArray *c3_o_y = NULL;
  real_T c3_p_u;
  const mxArray *c3_p_y = NULL;
  int32_T c3_i1;
  real_T c3_q_u[299];
  const mxArray *c3_q_y = NULL;
  int32_T c3_i2;
  real_T c3_r_u[192];
  const mxArray *c3_r_y = NULL;
  int32_T c3_i3;
  real_T c3_s_u[192];
  const mxArray *c3_s_y = NULL;
  int32_T c3_i4;
  real_T c3_t_u[72];
  const mxArray *c3_t_y = NULL;
  real_T c3_u_u;
  const mxArray *c3_u_y = NULL;
  real_T c3_v_u;
  const mxArray *c3_v_y = NULL;
  real_T c3_w_u;
  const mxArray *c3_w_y = NULL;
  int32_T c3_i5;
  real_T c3_x_u[13];
  const mxArray *c3_x_y = NULL;
  real_T c3_y_u;
  const mxArray *c3_y_y = NULL;
  boolean_T c3_ab_u;
  const mxArray *c3_ab_y = NULL;
  int32_T c3_i6;
  real_T c3_bb_u[8];
  const mxArray *c3_bb_y = NULL;
  c3_struct_9LpOi5JXaV67jTuay8hWaH c3_cb_u;
  const mxArray *c3_cb_y = NULL;
  int32_T c3_i7;
  real_T c3_db_u[24];
  const mxArray *c3_db_y = NULL;
  int32_T c3_i8;
  real_T c3_eb_u[8];
  const mxArray *c3_eb_y = NULL;
  int32_T c3_i9;
  real_T c3_fb_u[8];
  const mxArray *c3_fb_y = NULL;
  int32_T c3_i10;
  real_T c3_gb_u[8];
  const mxArray *c3_gb_y = NULL;
  int32_T c3_i11;
  real_T c3_hb_u[8];
  const mxArray *c3_hb_y = NULL;
  SFc3_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc3_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_mxArrayOutData = NULL;
  c3_u = *(c3_struct_rUGQ0INmvPpaxIctEGl5sE *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c3_b_u = c3_u.skipYoga;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_b_y, "skipYoga", "skipYoga", 0);
  c3_c_u = c3_u.demoOnlyRightFoot;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_c_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_c_y, "demoOnlyRightFoot", "demoOnlyRightFoot", 0);
  c3_d_u = c3_u.yogaAlsoOnRightFoot;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_d_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_d_y, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot", 0);
  c3_e_u = c3_u.yogaInLoop;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_e_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_e_y, "yogaInLoop", "yogaInLoop", 0);
  c3_f_u = c3_u.com;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c3_g_u = c3_f_u.threshold;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_f_y, c3_g_y, "threshold", "threshold", 0);
  for (c3_i0 = 0; c3_i0 < 39; c3_i0++) {
    c3_h_u[c3_i0] = c3_f_u.states[c3_i0];
  }

  c3_h_y = NULL;
  sf_mex_assign(&c3_h_y, sf_mex_create("y", c3_h_u, 0, 0U, 1U, 0U, 2, 13, 3),
                FALSE);
  sf_mex_addfield(c3_f_y, c3_h_y, "states", "states", 0);
  sf_mex_addfield(c3_y, c3_f_y, "com", "com", 0);
  c3_i_u = c3_u.wrench;
  c3_i_y = NULL;
  sf_mex_assign(&c3_i_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c3_j_u = c3_i_u.thresholdContactOn;
  c3_j_y = NULL;
  sf_mex_assign(&c3_j_y, sf_mex_create("y", &c3_j_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_i_y, c3_j_y, "thresholdContactOn", "thresholdContactOn", 0);
  c3_k_u = c3_i_u.thresholdContactOff;
  c3_k_y = NULL;
  sf_mex_assign(&c3_k_y, sf_mex_create("y", &c3_k_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_i_y, c3_k_y, "thresholdContactOff", "thresholdContactOff",
                  0);
  sf_mex_addfield(c3_y, c3_i_y, "wrench", "wrench", 0);
  c3_l_u = c3_u.joints;
  c3_l_y = NULL;
  sf_mex_assign(&c3_l_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c3_m_u = c3_l_u.thresholdNotInContact;
  c3_m_y = NULL;
  sf_mex_assign(&c3_m_y, sf_mex_create("y", &c3_m_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_l_y, c3_m_y, "thresholdNotInContact",
                  "thresholdNotInContact", 0);
  c3_n_u = c3_l_u.thresholdInContact;
  c3_n_y = NULL;
  sf_mex_assign(&c3_n_y, sf_mex_create("y", &c3_n_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_l_y, c3_n_y, "thresholdInContact", "thresholdInContact", 0);
  c3_o_u = c3_l_u.pauseTimeLastPostureL;
  c3_o_y = NULL;
  sf_mex_assign(&c3_o_y, sf_mex_create("y", &c3_o_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_l_y, c3_o_y, "pauseTimeLastPostureL",
                  "pauseTimeLastPostureL", 0);
  c3_p_u = c3_l_u.pauseTimeLastPostureR;
  c3_p_y = NULL;
  sf_mex_assign(&c3_p_y, sf_mex_create("y", &c3_p_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_l_y, c3_p_y, "pauseTimeLastPostureR",
                  "pauseTimeLastPostureR", 0);
  for (c3_i1 = 0; c3_i1 < 299; c3_i1++) {
    c3_q_u[c3_i1] = c3_l_u.states[c3_i1];
  }

  c3_q_y = NULL;
  sf_mex_assign(&c3_q_y, sf_mex_create("y", c3_q_u, 0, 0U, 1U, 0U, 2, 13, 23),
                FALSE);
  sf_mex_addfield(c3_l_y, c3_q_y, "states", "states", 0);
  for (c3_i2 = 0; c3_i2 < 192; c3_i2++) {
    c3_r_u[c3_i2] = c3_l_u.pointsL[c3_i2];
  }

  c3_r_y = NULL;
  sf_mex_assign(&c3_r_y, sf_mex_create("y", c3_r_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c3_l_y, c3_r_y, "pointsL", "pointsL", 0);
  for (c3_i3 = 0; c3_i3 < 192; c3_i3++) {
    c3_s_u[c3_i3] = c3_l_u.pointsR[c3_i3];
  }

  c3_s_y = NULL;
  sf_mex_assign(&c3_s_y, sf_mex_create("y", c3_s_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c3_l_y, c3_s_y, "pointsR", "pointsR", 0);
  for (c3_i4 = 0; c3_i4 < 72; c3_i4++) {
    c3_t_u[c3_i4] = c3_l_u.standUpPositions[c3_i4];
  }

  c3_t_y = NULL;
  sf_mex_assign(&c3_t_y, sf_mex_create("y", c3_t_u, 0, 0U, 1U, 0U, 2, 8, 9),
                FALSE);
  sf_mex_addfield(c3_l_y, c3_t_y, "standUpPositions", "standUpPositions", 0);
  sf_mex_addfield(c3_y, c3_l_y, "joints", "joints", 0);
  c3_u_u = c3_u.stateAt0;
  c3_u_y = NULL;
  sf_mex_assign(&c3_u_y, sf_mex_create("y", &c3_u_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_u_y, "stateAt0", "stateAt0", 0);
  c3_v_u = c3_u.DT;
  c3_v_y = NULL;
  sf_mex_assign(&c3_v_y, sf_mex_create("y", &c3_v_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_v_y, "DT", "DT", 0);
  c3_w_u = c3_u.waitingTimeAfterYoga;
  c3_w_y = NULL;
  sf_mex_assign(&c3_w_y, sf_mex_create("y", &c3_w_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_w_y, "waitingTimeAfterYoga", "waitingTimeAfterYoga",
                  0);
  for (c3_i5 = 0; c3_i5 < 13; c3_i5++) {
    c3_x_u[c3_i5] = c3_u.jointsSmoothingTimes[c3_i5];
  }

  c3_x_y = NULL;
  sf_mex_assign(&c3_x_y, sf_mex_create("y", c3_x_u, 0, 0U, 1U, 0U, 2, 13, 1),
                FALSE);
  sf_mex_addfield(c3_y, c3_x_y, "jointsSmoothingTimes", "jointsSmoothingTimes",
                  0);
  c3_y_u = c3_u.tBalancing;
  c3_y_y = NULL;
  sf_mex_assign(&c3_y_y, sf_mex_create("y", &c3_y_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_y_y, "tBalancing", "tBalancing", 0);
  c3_ab_u = c3_u.alsoSitDown;
  c3_ab_y = NULL;
  sf_mex_assign(&c3_ab_y, sf_mex_create("y", &c3_ab_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c3_y, c3_ab_y, "alsoSitDown", "alsoSitDown", 0);
  for (c3_i6 = 0; c3_i6 < 8; c3_i6++) {
    c3_bb_u[c3_i6] = c3_u.jointsAndCoMSmoothingTimes[c3_i6];
  }

  c3_bb_y = NULL;
  sf_mex_assign(&c3_bb_y, sf_mex_create("y", c3_bb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c3_y, c3_bb_y, "jointsAndCoMSmoothingTimes",
                  "jointsAndCoMSmoothingTimes", 0);
  c3_cb_u = c3_u.CoM;
  c3_cb_y = NULL;
  sf_mex_assign(&c3_cb_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  for (c3_i7 = 0; c3_i7 < 24; c3_i7++) {
    c3_db_u[c3_i7] = c3_cb_u.standUpDeltaCoM[c3_i7];
  }

  c3_db_y = NULL;
  sf_mex_assign(&c3_db_y, sf_mex_create("y", c3_db_u, 0, 0U, 1U, 0U, 2, 8, 3),
                FALSE);
  sf_mex_addfield(c3_cb_y, c3_db_y, "standUpDeltaCoM", "standUpDeltaCoM", 0);
  sf_mex_addfield(c3_y, c3_cb_y, "CoM", "CoM", 0);
  for (c3_i8 = 0; c3_i8 < 8; c3_i8++) {
    c3_eb_u[c3_i8] = c3_u.LwrenchThreshold[c3_i8];
  }

  c3_eb_y = NULL;
  sf_mex_assign(&c3_eb_y, sf_mex_create("y", c3_eb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c3_y, c3_eb_y, "LwrenchThreshold", "LwrenchThreshold", 0);
  for (c3_i9 = 0; c3_i9 < 8; c3_i9++) {
    c3_fb_u[c3_i9] = c3_u.RwrenchThreshold[c3_i9];
  }

  c3_fb_y = NULL;
  sf_mex_assign(&c3_fb_y, sf_mex_create("y", c3_fb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c3_y, c3_fb_y, "RwrenchThreshold", "RwrenchThreshold", 0);
  for (c3_i10 = 0; c3_i10 < 8; c3_i10++) {
    c3_gb_u[c3_i10] = c3_u.RArmThreshold[c3_i10];
  }

  c3_gb_y = NULL;
  sf_mex_assign(&c3_gb_y, sf_mex_create("y", c3_gb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c3_y, c3_gb_y, "RArmThreshold", "RArmThreshold", 0);
  for (c3_i11 = 0; c3_i11 < 8; c3_i11++) {
    c3_hb_u[c3_i11] = c3_u.LArmThreshold[c3_i11];
  }

  c3_hb_y = NULL;
  sf_mex_assign(&c3_hb_y, sf_mex_create("y", c3_hb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c3_y, c3_hb_y, "LArmThreshold", "LArmThreshold", 0);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static void c3_c_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_struct_rUGQ0INmvPpaxIctEGl5sE *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[19] = { "skipYoga", "demoOnlyRightFoot",
    "yogaAlsoOnRightFoot", "yogaInLoop", "com", "wrench", "joints", "stateAt0",
    "DT", "waitingTimeAfterYoga", "jointsSmoothingTimes", "tBalancing",
    "alsoSitDown", "jointsAndCoMSmoothingTimes", "CoM", "LwrenchThreshold",
    "RwrenchThreshold", "RArmThreshold", "LArmThreshold" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 19, c3_fieldNames, 0U, 0);
  c3_thisId.fIdentifier = "skipYoga";
  c3_y->skipYoga = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "skipYoga", "skipYoga", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "demoOnlyRightFoot";
  c3_y->demoOnlyRightFoot = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "demoOnlyRightFoot", "demoOnlyRightFoot", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "yogaAlsoOnRightFoot";
  c3_y->yogaAlsoOnRightFoot = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "yogaInLoop";
  c3_y->yogaInLoop = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "yogaInLoop", "yogaInLoop", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "com";
  c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "com",
    "com", 0)), &c3_thisId, &c3_y->com);
  c3_thisId.fIdentifier = "wrench";
  c3_y->wrench = c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
                                        (c3_u, "wrench", "wrench", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "joints";
  c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "joints",
    "joints", 0)), &c3_thisId, &c3_y->joints);
  c3_thisId.fIdentifier = "stateAt0";
  c3_y->stateAt0 = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "stateAt0", "stateAt0", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "DT";
  c3_y->DT = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c3_u, "DT", "DT", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "waitingTimeAfterYoga";
  c3_y->waitingTimeAfterYoga = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "waitingTimeAfterYoga", "waitingTimeAfterYoga", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "jointsSmoothingTimes";
  c3_l_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "jointsSmoothingTimes", "jointsSmoothingTimes", 0)), &c3_thisId,
                        c3_y->jointsSmoothingTimes);
  c3_thisId.fIdentifier = "tBalancing";
  c3_y->tBalancing = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "tBalancing", "tBalancing", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "alsoSitDown";
  c3_y->alsoSitDown = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "alsoSitDown", "alsoSitDown", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "jointsAndCoMSmoothingTimes";
  c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "jointsAndCoMSmoothingTimes", "jointsAndCoMSmoothingTimes", 0)), &c3_thisId,
                        c3_y->jointsAndCoMSmoothingTimes);
  c3_thisId.fIdentifier = "CoM";
  c3_n_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "CoM",
    "CoM", 0)), &c3_thisId, &c3_y->CoM);
  c3_thisId.fIdentifier = "LwrenchThreshold";
  c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "LwrenchThreshold", "LwrenchThreshold", 0)), &c3_thisId,
                        c3_y->LwrenchThreshold);
  c3_thisId.fIdentifier = "RwrenchThreshold";
  c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "RwrenchThreshold", "RwrenchThreshold", 0)), &c3_thisId,
                        c3_y->RwrenchThreshold);
  c3_thisId.fIdentifier = "RArmThreshold";
  c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "RArmThreshold", "RArmThreshold", 0)), &c3_thisId, c3_y->RArmThreshold);
  c3_thisId.fIdentifier = "LArmThreshold";
  c3_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "LArmThreshold", "LArmThreshold", 0)), &c3_thisId, c3_y->LArmThreshold);
  sf_mex_destroy(&c3_u);
}

static boolean_T c3_d_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  boolean_T c3_y;
  boolean_T c3_b0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_b0, 1, 11, 0U, 0, 0U, 0);
  c3_y = c3_b0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_e_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_struct_DnBdbfPNxiIjhNOyZMmfsE *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[2] = { "threshold", "states" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 2, c3_fieldNames, 0U, 0);
  c3_thisId.fIdentifier = "threshold";
  c3_y->threshold = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "threshold", "threshold", 0)), &c3_thisId);
  c3_thisId.fIdentifier = "states";
  c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "states",
    "states", 0)), &c3_thisId, c3_y->states);
  sf_mex_destroy(&c3_u);
}

static void c3_f_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[39])
{
  real_T c3_dv0[39];
  int32_T c3_i12;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv0, 1, 0, 0U, 1, 0U, 2, 13, 3);
  for (c3_i12 = 0; c3_i12 < 39; c3_i12++) {
    c3_y[c3_i12] = c3_dv0[c3_i12];
  }

  sf_mex_destroy(&c3_u);
}

static c3_struct_KJR2itYvhBuAkuR6dKZHUC c3_g_emlrt_marshallIn
  (SFc3_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c3_u,
   const emlrtMsgIdentifier *c3_parentId)
{
  c3_struct_KJR2itYvhBuAkuR6dKZHUC c3_y;
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[2] = { "thresholdContactOn",
    "thresholdContactOff" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 2, c3_fieldNames, 0U, 0);
  c3_thisId.fIdentifier = "thresholdContactOn";
  c3_y.thresholdContactOn = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "thresholdContactOn", "thresholdContactOn", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "thresholdContactOff";
  c3_y.thresholdContactOff = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "thresholdContactOff", "thresholdContactOff", 0)),
    &c3_thisId);
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_h_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_struct_0U0wBk2LiR1OqsMsUngxdD *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[8] = { "thresholdNotInContact",
    "thresholdInContact", "pauseTimeLastPostureL", "pauseTimeLastPostureR",
    "states", "pointsL", "pointsR", "standUpPositions" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 8, c3_fieldNames, 0U, 0);
  c3_thisId.fIdentifier = "thresholdNotInContact";
  c3_y->thresholdNotInContact = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "thresholdNotInContact", "thresholdNotInContact", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "thresholdInContact";
  c3_y->thresholdInContact = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "thresholdInContact", "thresholdInContact", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "pauseTimeLastPostureL";
  c3_y->pauseTimeLastPostureL = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "pauseTimeLastPostureL", "pauseTimeLastPostureL", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "pauseTimeLastPostureR";
  c3_y->pauseTimeLastPostureR = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c3_u, "pauseTimeLastPostureR", "pauseTimeLastPostureR", 0)),
    &c3_thisId);
  c3_thisId.fIdentifier = "states";
  c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u, "states",
    "states", 0)), &c3_thisId, c3_y->states);
  c3_thisId.fIdentifier = "pointsL";
  c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "pointsL", "pointsL", 0)), &c3_thisId, c3_y->pointsL);
  c3_thisId.fIdentifier = "pointsR";
  c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "pointsR", "pointsR", 0)), &c3_thisId, c3_y->pointsR);
  c3_thisId.fIdentifier = "standUpPositions";
  c3_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "standUpPositions", "standUpPositions", 0)), &c3_thisId,
                        c3_y->standUpPositions);
  sf_mex_destroy(&c3_u);
}

static void c3_i_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[299])
{
  real_T c3_dv1[299];
  int32_T c3_i13;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv1, 1, 0, 0U, 1, 0U, 2, 13,
                23);
  for (c3_i13 = 0; c3_i13 < 299; c3_i13++) {
    c3_y[c3_i13] = c3_dv1[c3_i13];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_j_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[192])
{
  real_T c3_dv2[192];
  int32_T c3_i14;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv2, 1, 0, 0U, 1, 0U, 2, 8, 24);
  for (c3_i14 = 0; c3_i14 < 192; c3_i14++) {
    c3_y[c3_i14] = c3_dv2[c3_i14];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_k_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[72])
{
  real_T c3_dv3[72];
  int32_T c3_i15;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv3, 1, 0, 0U, 1, 0U, 2, 8, 9);
  for (c3_i15 = 0; c3_i15 < 72; c3_i15++) {
    c3_y[c3_i15] = c3_dv3[c3_i15];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_l_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[13])
{
  real_T c3_dv4[13];
  int32_T c3_i16;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv4, 1, 0, 0U, 1, 0U, 2, 13, 1);
  for (c3_i16 = 0; c3_i16 < 13; c3_i16++) {
    c3_y[c3_i16] = c3_dv4[c3_i16];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_m_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[8])
{
  real_T c3_dv5[8];
  int32_T c3_i17;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv5, 1, 0, 0U, 1, 0U, 2, 8, 1);
  for (c3_i17 = 0; c3_i17 < 8; c3_i17++) {
    c3_y[c3_i17] = c3_dv5[c3_i17];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_n_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  c3_struct_9LpOi5JXaV67jTuay8hWaH *c3_y)
{
  emlrtMsgIdentifier c3_thisId;
  static const char * c3_fieldNames[1] = { "standUpDeltaCoM" };

  c3_thisId.fParent = c3_parentId;
  sf_mex_check_struct(c3_parentId, c3_u, 1, c3_fieldNames, 0U, 0);
  c3_thisId.fIdentifier = "standUpDeltaCoM";
  c3_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c3_u,
    "standUpDeltaCoM", "standUpDeltaCoM", 0)), &c3_thisId, c3_y->standUpDeltaCoM);
  sf_mex_destroy(&c3_u);
}

static void c3_o_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[24])
{
  real_T c3_dv6[24];
  int32_T c3_i18;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv6, 1, 0, 0U, 1, 0U, 2, 8, 3);
  for (c3_i18 = 0; c3_i18 < 24; c3_i18++) {
    c3_y[c3_i18] = c3_dv6[c3_i18];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sm;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  c3_struct_rUGQ0INmvPpaxIctEGl5sE c3_y;
  SFc3_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc3_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c3_b_sm = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sm), &c3_thisId, &c3_y);
  sf_mex_destroy(&c3_b_sm);
  *(c3_struct_rUGQ0INmvPpaxIctEGl5sE *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  sf_mex_assign(&c3_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), FALSE);
  return c3_nameCaptureInfo;
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc3_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static int32_T c3_p_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i19;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i19, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i19;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc3_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_q_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_torqueBalancing2012b, const
  char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_r_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_torqueBalancing2012b), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_torqueBalancing2012b);
  return c3_y;
}

static uint8_T c3_r_emlrt_marshallIn(SFc3_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void init_dsm_address_info(SFc3_torqueBalancing2012bInstanceStruct
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

void sf_c3_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3114988707U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3828861454U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2845401588U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(848464764U);
}

mxArray *sf_c3_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("EX18UBeAltytyKxXaAuhAG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c3_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"thresholdContactLeft\",},{M[1],M[7],T\"thresholdContactRight\",},{M[8],M[0],T\"is_active_c3_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc3_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           3,
           1,
           1,
           4,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"state");
          _SFD_SET_DATA_PROPS(1,2,0,1,"thresholdContactLeft");
          _SFD_SET_DATA_PROPS(2,2,0,1,"thresholdContactRight");
          _SFD_SET_DATA_PROPS(3,10,0,0,"sm");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,187);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)c3_b_sf_marshallIn);

        {
          real_T *c3_state;
          real_T *c3_thresholdContactLeft;
          real_T *c3_thresholdContactRight;
          c3_thresholdContactRight = (real_T *)ssGetOutputPortSignal
            (chartInstance->S, 2);
          c3_thresholdContactLeft = (real_T *)ssGetOutputPortSignal
            (chartInstance->S, 1);
          c3_state = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c3_state);
          _SFD_SET_DATA_VALUE_PTR(1U, c3_thresholdContactLeft);
          _SFD_SET_DATA_VALUE_PTR(2U, c3_thresholdContactRight);
          _SFD_SET_DATA_VALUE_PTR(3U, &chartInstance->c3_sm);
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
  return "rrzRfmA6jg71OCcou75XRB";
}

static void sf_opaque_initialize_c3_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c3_torqueBalancing2012b
    ((SFc3_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c3_torqueBalancing2012b((SFc3_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c3_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c3_torqueBalancing2012b((SFc3_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c3_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c3_torqueBalancing2012b((SFc3_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c3_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c3_torqueBalancing2012b((SFc3_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c3_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_torqueBalancing2012b
    ((SFc3_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c3_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_torqueBalancing2012b((SFc3_torqueBalancing2012bInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c3_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c3_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c3_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c3_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_torqueBalancing2012bInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c3_torqueBalancing2012b((SFc3_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_torqueBalancing2012b((SFc3_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_torqueBalancing2012b
      ((SFc3_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_torqueBalancing2012b(SimStruct *S)
{
  /* Actual parameters from chart:
     sm
   */
  const char_T *rtParamNames[] = { "sm" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0],
    sf_get_param_data_type_id(S,0));
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,3,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 1; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1900389832U));
  ssSetChecksum1(S,(2494693417U));
  ssSetChecksum2(S,(2464810903U));
  ssSetChecksum3(S,(578865007U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_torqueBalancing2012b(SimStruct *S)
{
  SFc3_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc3_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc3_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c3_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c3_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c3_torqueBalancing2012b;
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

void c3_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void *
  data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
