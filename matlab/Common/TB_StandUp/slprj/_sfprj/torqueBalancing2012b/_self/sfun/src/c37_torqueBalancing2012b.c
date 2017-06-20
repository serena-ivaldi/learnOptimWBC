/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c37_torqueBalancing2012b.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "torqueBalancing2012b_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c37_debug_family_names[6] = { "nargin", "nargout", "state",
  "sm", "thresholdArmContactLeft", "thresholdArmContactRight" };

/* Function Declarations */
static void initialize_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static void c37_update_debugger_state_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c37_st);
static void finalize_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c37_torqueBalancing2012b(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c37_machineNumber, uint32_T
  c37_chartNumber);
static const mxArray *c37_sf_marshallOut(void *chartInstanceVoid, void
  *c37_inData);
static real_T c37_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_thresholdArmContactRight, const char_T
  *c37_identifier);
static real_T c37_b_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId);
static void c37_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c37_mxArrayInData, const char_T *c37_varName, void *c37_outData);
static const mxArray *c37_b_sf_marshallOut(void *chartInstanceVoid, void
  *c37_inData);
static void c37_c_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  c37_struct_rUGQ0INmvPpaxIctEGl5sE *c37_y);
static boolean_T c37_d_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId);
static void c37_e_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  c37_struct_DnBdbfPNxiIjhNOyZMmfsE *c37_y);
static void c37_f_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[39]);
static c37_struct_KJR2itYvhBuAkuR6dKZHUC c37_g_emlrt_marshallIn
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c37_u,
   const emlrtMsgIdentifier *c37_parentId);
static void c37_h_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  c37_struct_0U0wBk2LiR1OqsMsUngxdD *c37_y);
static void c37_i_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[299]);
static void c37_j_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[192]);
static void c37_k_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[72]);
static void c37_l_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[13]);
static void c37_m_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[8]);
static void c37_n_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  c37_struct_9LpOi5JXaV67jTuay8hWaH *c37_y);
static void c37_o_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[24]);
static void c37_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c37_mxArrayInData, const char_T *c37_varName, void *c37_outData);
static const mxArray *c37_c_sf_marshallOut(void *chartInstanceVoid, void
  *c37_inData);
static int32_T c37_p_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId);
static void c37_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c37_mxArrayInData, const char_T *c37_varName, void *c37_outData);
static uint8_T c37_q_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_b_is_active_c37_torqueBalancing2012b, const
  char_T *c37_identifier);
static uint8_T c37_r_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId);
static void init_dsm_address_info(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c37_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c37_is_active_c37_torqueBalancing2012b = 0U;
}

static void initialize_params_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c37_m0 = NULL;
  const mxArray *c37_mxField;
  c37_struct_rUGQ0INmvPpaxIctEGl5sE c37_r0;
  const mxArray *c37_m1 = NULL;
  const mxArray *c37_b_mxField;
  const mxArray *c37_m2 = NULL;
  const mxArray *c37_c_mxField;
  const mxArray *c37_m3 = NULL;
  const mxArray *c37_d_mxField;
  const mxArray *c37_m4 = NULL;
  const mxArray *c37_e_mxField;
  sf_set_error_prefix_string(
    "Error evaluating data 'sm' in the parent workspace.\n");
  c37_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c37_mxField = sf_mex_getfield(c37_m0, "skipYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), &c37_r0.skipYoga, 1, 11, 0U,
                      0, 0U, 0);
  c37_mxField = sf_mex_getfield(c37_m0, "demoOnlyRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), &c37_r0.demoOnlyRightFoot,
                      1, 11, 0U, 0, 0U, 0);
  c37_mxField = sf_mex_getfield(c37_m0, "yogaAlsoOnRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), &c37_r0.yogaAlsoOnRightFoot,
                      1, 11, 0U, 0, 0U, 0);
  c37_mxField = sf_mex_getfield(c37_m0, "yogaInLoop", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), &c37_r0.yogaInLoop, 1, 11,
                      0U, 0, 0U, 0);
  c37_mxField = sf_mex_getfield(c37_m0, "com", "sm", 0);
  c37_m1 = sf_mex_dup(c37_mxField);
  c37_b_mxField = sf_mex_getfield(c37_m1, "threshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_b_mxField), &c37_r0.com.threshold, 1,
                      0, 0U, 0, 0U, 0);
  c37_b_mxField = sf_mex_getfield(c37_m1, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_b_mxField), c37_r0.com.states, 1, 0,
                      0U, 1, 0U, 2, 13, 3);
  sf_mex_destroy(&c37_m1);
  c37_mxField = sf_mex_getfield(c37_m0, "wrench", "sm", 0);
  c37_m2 = sf_mex_dup(c37_mxField);
  c37_c_mxField = sf_mex_getfield(c37_m2, "thresholdContactOn", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_c_mxField),
                      &c37_r0.wrench.thresholdContactOn, 1, 0, 0U, 0, 0U, 0);
  c37_c_mxField = sf_mex_getfield(c37_m2, "thresholdContactOff", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_c_mxField),
                      &c37_r0.wrench.thresholdContactOff, 1, 0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c37_m2);
  c37_mxField = sf_mex_getfield(c37_m0, "joints", "sm", 0);
  c37_m3 = sf_mex_dup(c37_mxField);
  c37_d_mxField = sf_mex_getfield(c37_m3, "thresholdNotInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_d_mxField),
                      &c37_r0.joints.thresholdNotInContact, 1, 0, 0U, 0, 0U, 0);
  c37_d_mxField = sf_mex_getfield(c37_m3, "thresholdInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_d_mxField),
                      &c37_r0.joints.thresholdInContact, 1, 0, 0U, 0, 0U, 0);
  c37_d_mxField = sf_mex_getfield(c37_m3, "pauseTimeLastPostureL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_d_mxField),
                      &c37_r0.joints.pauseTimeLastPostureL, 1, 0, 0U, 0, 0U, 0);
  c37_d_mxField = sf_mex_getfield(c37_m3, "pauseTimeLastPostureR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_d_mxField),
                      &c37_r0.joints.pauseTimeLastPostureR, 1, 0, 0U, 0, 0U, 0);
  c37_d_mxField = sf_mex_getfield(c37_m3, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_d_mxField), c37_r0.joints.states, 1,
                      0, 0U, 1, 0U, 2, 13, 23);
  c37_d_mxField = sf_mex_getfield(c37_m3, "pointsL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_d_mxField), c37_r0.joints.pointsL, 1,
                      0, 0U, 1, 0U, 2, 8, 24);
  c37_d_mxField = sf_mex_getfield(c37_m3, "pointsR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_d_mxField), c37_r0.joints.pointsR, 1,
                      0, 0U, 1, 0U, 2, 8, 24);
  c37_d_mxField = sf_mex_getfield(c37_m3, "standUpPositions", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_d_mxField),
                      c37_r0.joints.standUpPositions, 1, 0, 0U, 1, 0U, 2, 8, 9);
  sf_mex_destroy(&c37_m3);
  c37_mxField = sf_mex_getfield(c37_m0, "stateAt0", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), &c37_r0.stateAt0, 1, 0, 0U,
                      0, 0U, 0);
  c37_mxField = sf_mex_getfield(c37_m0, "DT", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), &c37_r0.DT, 1, 0, 0U, 0, 0U,
                      0);
  c37_mxField = sf_mex_getfield(c37_m0, "waitingTimeAfterYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField),
                      &c37_r0.waitingTimeAfterYoga, 1, 0, 0U, 0, 0U, 0);
  c37_mxField = sf_mex_getfield(c37_m0, "jointsSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), c37_r0.jointsSmoothingTimes,
                      1, 0, 0U, 1, 0U, 2, 13, 1);
  c37_mxField = sf_mex_getfield(c37_m0, "tBalancing", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), &c37_r0.tBalancing, 1, 0,
                      0U, 0, 0U, 0);
  c37_mxField = sf_mex_getfield(c37_m0, "alsoSitDown", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), &c37_r0.alsoSitDown, 1, 11,
                      0U, 0, 0U, 0);
  c37_mxField = sf_mex_getfield(c37_m0, "jointsAndCoMSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField),
                      c37_r0.jointsAndCoMSmoothingTimes, 1, 0, 0U, 1, 0U, 2, 8,
                      1);
  c37_mxField = sf_mex_getfield(c37_m0, "CoM", "sm", 0);
  c37_m4 = sf_mex_dup(c37_mxField);
  c37_e_mxField = sf_mex_getfield(c37_m4, "standUpDeltaCoM", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_e_mxField),
                      c37_r0.CoM.standUpDeltaCoM, 1, 0, 0U, 1, 0U, 2, 8, 3);
  sf_mex_destroy(&c37_m4);
  c37_mxField = sf_mex_getfield(c37_m0, "LwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), c37_r0.LwrenchThreshold, 1,
                      0, 0U, 1, 0U, 2, 8, 1);
  c37_mxField = sf_mex_getfield(c37_m0, "RwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), c37_r0.RwrenchThreshold, 1,
                      0, 0U, 1, 0U, 2, 8, 1);
  c37_mxField = sf_mex_getfield(c37_m0, "RArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), c37_r0.RArmThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  c37_mxField = sf_mex_getfield(c37_m0, "LArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c37_mxField), c37_r0.LArmThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  sf_mex_destroy(&c37_m0);
  chartInstance->c37_sm = c37_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c37_update_debugger_state_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c37_st;
  const mxArray *c37_y = NULL;
  real_T c37_hoistedGlobal;
  real_T c37_u;
  const mxArray *c37_b_y = NULL;
  real_T c37_b_hoistedGlobal;
  real_T c37_b_u;
  const mxArray *c37_c_y = NULL;
  uint8_T c37_c_hoistedGlobal;
  uint8_T c37_c_u;
  const mxArray *c37_d_y = NULL;
  real_T *c37_thresholdArmContactLeft;
  real_T *c37_thresholdArmContactRight;
  c37_thresholdArmContactRight = (real_T *)ssGetOutputPortSignal
    (chartInstance->S, 2);
  c37_thresholdArmContactLeft = (real_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  c37_st = NULL;
  c37_st = NULL;
  c37_y = NULL;
  sf_mex_assign(&c37_y, sf_mex_createcellarray(3), FALSE);
  c37_hoistedGlobal = *c37_thresholdArmContactLeft;
  c37_u = c37_hoistedGlobal;
  c37_b_y = NULL;
  sf_mex_assign(&c37_b_y, sf_mex_create("y", &c37_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c37_y, 0, c37_b_y);
  c37_b_hoistedGlobal = *c37_thresholdArmContactRight;
  c37_b_u = c37_b_hoistedGlobal;
  c37_c_y = NULL;
  sf_mex_assign(&c37_c_y, sf_mex_create("y", &c37_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c37_y, 1, c37_c_y);
  c37_c_hoistedGlobal = chartInstance->c37_is_active_c37_torqueBalancing2012b;
  c37_c_u = c37_c_hoistedGlobal;
  c37_d_y = NULL;
  sf_mex_assign(&c37_d_y, sf_mex_create("y", &c37_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c37_y, 2, c37_d_y);
  sf_mex_assign(&c37_st, c37_y, FALSE);
  return c37_st;
}

static void set_sim_state_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c37_st)
{
  const mxArray *c37_u;
  real_T *c37_thresholdArmContactLeft;
  real_T *c37_thresholdArmContactRight;
  c37_thresholdArmContactRight = (real_T *)ssGetOutputPortSignal
    (chartInstance->S, 2);
  c37_thresholdArmContactLeft = (real_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  chartInstance->c37_doneDoubleBufferReInit = TRUE;
  c37_u = sf_mex_dup(c37_st);
  *c37_thresholdArmContactLeft = c37_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c37_u, 0)), "thresholdArmContactLeft");
  *c37_thresholdArmContactRight = c37_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c37_u, 1)), "thresholdArmContactRight");
  chartInstance->c37_is_active_c37_torqueBalancing2012b = c37_q_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c37_u, 2)),
     "is_active_c37_torqueBalancing2012b");
  sf_mex_destroy(&c37_u);
  c37_update_debugger_state_c37_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c37_st);
}

static void finalize_c37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c37_torqueBalancing2012b(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  real_T c37_hoistedGlobal;
  real_T c37_state;
  c37_struct_rUGQ0INmvPpaxIctEGl5sE c37_b_sm;
  uint32_T c37_debug_family_var_map[6];
  real_T c37_nargin = 2.0;
  real_T c37_nargout = 2.0;
  real_T c37_thresholdArmContactLeft;
  real_T c37_thresholdArmContactRight;
  real_T *c37_b_state;
  real_T *c37_b_thresholdArmContactLeft;
  real_T *c37_b_thresholdArmContactRight;
  c37_b_thresholdArmContactRight = (real_T *)ssGetOutputPortSignal
    (chartInstance->S, 2);
  c37_b_thresholdArmContactLeft = (real_T *)ssGetOutputPortSignal
    (chartInstance->S, 1);
  c37_b_state = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 30U, chartInstance->c37_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c37_b_state, 0U);
  _SFD_DATA_RANGE_CHECK(*c37_b_thresholdArmContactLeft, 1U);
  _SFD_DATA_RANGE_CHECK(*c37_b_thresholdArmContactRight, 2U);
  chartInstance->c37_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 30U, chartInstance->c37_sfEvent);
  c37_hoistedGlobal = *c37_b_state;
  c37_state = c37_hoistedGlobal;
  c37_b_sm = chartInstance->c37_sm;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c37_debug_family_names,
    c37_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c37_nargin, 0U, c37_sf_marshallOut,
    c37_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c37_nargout, 1U, c37_sf_marshallOut,
    c37_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c37_state, 2U, c37_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c37_b_sm, 3U, c37_b_sf_marshallOut,
    c37_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c37_thresholdArmContactLeft, 4U,
    c37_sf_marshallOut, c37_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c37_thresholdArmContactRight, 5U,
    c37_sf_marshallOut, c37_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c37_sfEvent, 3);
  c37_thresholdArmContactLeft = c37_b_sm.LArmThreshold[(int32_T)(real_T)
    _SFD_EML_ARRAY_BOUNDS_CHECK("sm.LArmThreshold", (int32_T)_SFD_INTEGER_CHECK(
    "state", c37_state), 1, 8, 1, 0) - 1];
  _SFD_EML_CALL(0U, chartInstance->c37_sfEvent, 4);
  c37_thresholdArmContactRight = c37_b_sm.RArmThreshold[(int32_T)(real_T)
    _SFD_EML_ARRAY_BOUNDS_CHECK("sm.RArmThreshold", (int32_T)_SFD_INTEGER_CHECK(
    "state", c37_state), 1, 8, 1, 0) - 1];
  _SFD_EML_CALL(0U, chartInstance->c37_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
  *c37_b_thresholdArmContactLeft = c37_thresholdArmContactLeft;
  *c37_b_thresholdArmContactRight = c37_thresholdArmContactRight;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 30U, chartInstance->c37_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc37_torqueBalancing2012b
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c37_machineNumber, uint32_T
  c37_chartNumber)
{
}

static const mxArray *c37_sf_marshallOut(void *chartInstanceVoid, void
  *c37_inData)
{
  const mxArray *c37_mxArrayOutData = NULL;
  real_T c37_u;
  const mxArray *c37_y = NULL;
  SFc37_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc37_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c37_mxArrayOutData = NULL;
  c37_u = *(real_T *)c37_inData;
  c37_y = NULL;
  sf_mex_assign(&c37_y, sf_mex_create("y", &c37_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c37_mxArrayOutData, c37_y, FALSE);
  return c37_mxArrayOutData;
}

static real_T c37_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_thresholdArmContactRight, const char_T
  *c37_identifier)
{
  real_T c37_y;
  emlrtMsgIdentifier c37_thisId;
  c37_thisId.fIdentifier = c37_identifier;
  c37_thisId.fParent = NULL;
  c37_y = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c37_thresholdArmContactRight), &c37_thisId);
  sf_mex_destroy(&c37_thresholdArmContactRight);
  return c37_y;
}

static real_T c37_b_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId)
{
  real_T c37_y;
  real_T c37_d0;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), &c37_d0, 1, 0, 0U, 0, 0U, 0);
  c37_y = c37_d0;
  sf_mex_destroy(&c37_u);
  return c37_y;
}

static void c37_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c37_mxArrayInData, const char_T *c37_varName, void *c37_outData)
{
  const mxArray *c37_thresholdArmContactRight;
  const char_T *c37_identifier;
  emlrtMsgIdentifier c37_thisId;
  real_T c37_y;
  SFc37_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc37_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c37_thresholdArmContactRight = sf_mex_dup(c37_mxArrayInData);
  c37_identifier = c37_varName;
  c37_thisId.fIdentifier = c37_identifier;
  c37_thisId.fParent = NULL;
  c37_y = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c37_thresholdArmContactRight), &c37_thisId);
  sf_mex_destroy(&c37_thresholdArmContactRight);
  *(real_T *)c37_outData = c37_y;
  sf_mex_destroy(&c37_mxArrayInData);
}

static const mxArray *c37_b_sf_marshallOut(void *chartInstanceVoid, void
  *c37_inData)
{
  const mxArray *c37_mxArrayOutData;
  c37_struct_rUGQ0INmvPpaxIctEGl5sE c37_u;
  const mxArray *c37_y = NULL;
  boolean_T c37_b_u;
  const mxArray *c37_b_y = NULL;
  boolean_T c37_c_u;
  const mxArray *c37_c_y = NULL;
  boolean_T c37_d_u;
  const mxArray *c37_d_y = NULL;
  boolean_T c37_e_u;
  const mxArray *c37_e_y = NULL;
  c37_struct_DnBdbfPNxiIjhNOyZMmfsE c37_f_u;
  const mxArray *c37_f_y = NULL;
  real_T c37_g_u;
  const mxArray *c37_g_y = NULL;
  int32_T c37_i0;
  real_T c37_h_u[39];
  const mxArray *c37_h_y = NULL;
  c37_struct_KJR2itYvhBuAkuR6dKZHUC c37_i_u;
  const mxArray *c37_i_y = NULL;
  real_T c37_j_u;
  const mxArray *c37_j_y = NULL;
  real_T c37_k_u;
  const mxArray *c37_k_y = NULL;
  c37_struct_0U0wBk2LiR1OqsMsUngxdD c37_l_u;
  const mxArray *c37_l_y = NULL;
  real_T c37_m_u;
  const mxArray *c37_m_y = NULL;
  real_T c37_n_u;
  const mxArray *c37_n_y = NULL;
  real_T c37_o_u;
  const mxArray *c37_o_y = NULL;
  real_T c37_p_u;
  const mxArray *c37_p_y = NULL;
  int32_T c37_i1;
  real_T c37_q_u[299];
  const mxArray *c37_q_y = NULL;
  int32_T c37_i2;
  real_T c37_r_u[192];
  const mxArray *c37_r_y = NULL;
  int32_T c37_i3;
  real_T c37_s_u[192];
  const mxArray *c37_s_y = NULL;
  int32_T c37_i4;
  real_T c37_t_u[72];
  const mxArray *c37_t_y = NULL;
  real_T c37_u_u;
  const mxArray *c37_u_y = NULL;
  real_T c37_v_u;
  const mxArray *c37_v_y = NULL;
  real_T c37_w_u;
  const mxArray *c37_w_y = NULL;
  int32_T c37_i5;
  real_T c37_x_u[13];
  const mxArray *c37_x_y = NULL;
  real_T c37_y_u;
  const mxArray *c37_y_y = NULL;
  boolean_T c37_ab_u;
  const mxArray *c37_ab_y = NULL;
  int32_T c37_i6;
  real_T c37_bb_u[8];
  const mxArray *c37_bb_y = NULL;
  c37_struct_9LpOi5JXaV67jTuay8hWaH c37_cb_u;
  const mxArray *c37_cb_y = NULL;
  int32_T c37_i7;
  real_T c37_db_u[24];
  const mxArray *c37_db_y = NULL;
  int32_T c37_i8;
  real_T c37_eb_u[8];
  const mxArray *c37_eb_y = NULL;
  int32_T c37_i9;
  real_T c37_fb_u[8];
  const mxArray *c37_fb_y = NULL;
  int32_T c37_i10;
  real_T c37_gb_u[8];
  const mxArray *c37_gb_y = NULL;
  int32_T c37_i11;
  real_T c37_hb_u[8];
  const mxArray *c37_hb_y = NULL;
  SFc37_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc37_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c37_mxArrayOutData = NULL;
  c37_mxArrayOutData = NULL;
  c37_u = *(c37_struct_rUGQ0INmvPpaxIctEGl5sE *)c37_inData;
  c37_y = NULL;
  sf_mex_assign(&c37_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c37_b_u = c37_u.skipYoga;
  c37_b_y = NULL;
  sf_mex_assign(&c37_b_y, sf_mex_create("y", &c37_b_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_y, c37_b_y, "skipYoga", "skipYoga", 0);
  c37_c_u = c37_u.demoOnlyRightFoot;
  c37_c_y = NULL;
  sf_mex_assign(&c37_c_y, sf_mex_create("y", &c37_c_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_y, c37_c_y, "demoOnlyRightFoot", "demoOnlyRightFoot", 0);
  c37_d_u = c37_u.yogaAlsoOnRightFoot;
  c37_d_y = NULL;
  sf_mex_assign(&c37_d_y, sf_mex_create("y", &c37_d_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_y, c37_d_y, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot",
                  0);
  c37_e_u = c37_u.yogaInLoop;
  c37_e_y = NULL;
  sf_mex_assign(&c37_e_y, sf_mex_create("y", &c37_e_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_y, c37_e_y, "yogaInLoop", "yogaInLoop", 0);
  c37_f_u = c37_u.com;
  c37_f_y = NULL;
  sf_mex_assign(&c37_f_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c37_g_u = c37_f_u.threshold;
  c37_g_y = NULL;
  sf_mex_assign(&c37_g_y, sf_mex_create("y", &c37_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_f_y, c37_g_y, "threshold", "threshold", 0);
  for (c37_i0 = 0; c37_i0 < 39; c37_i0++) {
    c37_h_u[c37_i0] = c37_f_u.states[c37_i0];
  }

  c37_h_y = NULL;
  sf_mex_assign(&c37_h_y, sf_mex_create("y", c37_h_u, 0, 0U, 1U, 0U, 2, 13, 3),
                FALSE);
  sf_mex_addfield(c37_f_y, c37_h_y, "states", "states", 0);
  sf_mex_addfield(c37_y, c37_f_y, "com", "com", 0);
  c37_i_u = c37_u.wrench;
  c37_i_y = NULL;
  sf_mex_assign(&c37_i_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c37_j_u = c37_i_u.thresholdContactOn;
  c37_j_y = NULL;
  sf_mex_assign(&c37_j_y, sf_mex_create("y", &c37_j_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_i_y, c37_j_y, "thresholdContactOn", "thresholdContactOn",
                  0);
  c37_k_u = c37_i_u.thresholdContactOff;
  c37_k_y = NULL;
  sf_mex_assign(&c37_k_y, sf_mex_create("y", &c37_k_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_i_y, c37_k_y, "thresholdContactOff", "thresholdContactOff",
                  0);
  sf_mex_addfield(c37_y, c37_i_y, "wrench", "wrench", 0);
  c37_l_u = c37_u.joints;
  c37_l_y = NULL;
  sf_mex_assign(&c37_l_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c37_m_u = c37_l_u.thresholdNotInContact;
  c37_m_y = NULL;
  sf_mex_assign(&c37_m_y, sf_mex_create("y", &c37_m_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_l_y, c37_m_y, "thresholdNotInContact",
                  "thresholdNotInContact", 0);
  c37_n_u = c37_l_u.thresholdInContact;
  c37_n_y = NULL;
  sf_mex_assign(&c37_n_y, sf_mex_create("y", &c37_n_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_l_y, c37_n_y, "thresholdInContact", "thresholdInContact",
                  0);
  c37_o_u = c37_l_u.pauseTimeLastPostureL;
  c37_o_y = NULL;
  sf_mex_assign(&c37_o_y, sf_mex_create("y", &c37_o_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_l_y, c37_o_y, "pauseTimeLastPostureL",
                  "pauseTimeLastPostureL", 0);
  c37_p_u = c37_l_u.pauseTimeLastPostureR;
  c37_p_y = NULL;
  sf_mex_assign(&c37_p_y, sf_mex_create("y", &c37_p_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_l_y, c37_p_y, "pauseTimeLastPostureR",
                  "pauseTimeLastPostureR", 0);
  for (c37_i1 = 0; c37_i1 < 299; c37_i1++) {
    c37_q_u[c37_i1] = c37_l_u.states[c37_i1];
  }

  c37_q_y = NULL;
  sf_mex_assign(&c37_q_y, sf_mex_create("y", c37_q_u, 0, 0U, 1U, 0U, 2, 13, 23),
                FALSE);
  sf_mex_addfield(c37_l_y, c37_q_y, "states", "states", 0);
  for (c37_i2 = 0; c37_i2 < 192; c37_i2++) {
    c37_r_u[c37_i2] = c37_l_u.pointsL[c37_i2];
  }

  c37_r_y = NULL;
  sf_mex_assign(&c37_r_y, sf_mex_create("y", c37_r_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c37_l_y, c37_r_y, "pointsL", "pointsL", 0);
  for (c37_i3 = 0; c37_i3 < 192; c37_i3++) {
    c37_s_u[c37_i3] = c37_l_u.pointsR[c37_i3];
  }

  c37_s_y = NULL;
  sf_mex_assign(&c37_s_y, sf_mex_create("y", c37_s_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c37_l_y, c37_s_y, "pointsR", "pointsR", 0);
  for (c37_i4 = 0; c37_i4 < 72; c37_i4++) {
    c37_t_u[c37_i4] = c37_l_u.standUpPositions[c37_i4];
  }

  c37_t_y = NULL;
  sf_mex_assign(&c37_t_y, sf_mex_create("y", c37_t_u, 0, 0U, 1U, 0U, 2, 8, 9),
                FALSE);
  sf_mex_addfield(c37_l_y, c37_t_y, "standUpPositions", "standUpPositions", 0);
  sf_mex_addfield(c37_y, c37_l_y, "joints", "joints", 0);
  c37_u_u = c37_u.stateAt0;
  c37_u_y = NULL;
  sf_mex_assign(&c37_u_y, sf_mex_create("y", &c37_u_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_y, c37_u_y, "stateAt0", "stateAt0", 0);
  c37_v_u = c37_u.DT;
  c37_v_y = NULL;
  sf_mex_assign(&c37_v_y, sf_mex_create("y", &c37_v_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_y, c37_v_y, "DT", "DT", 0);
  c37_w_u = c37_u.waitingTimeAfterYoga;
  c37_w_y = NULL;
  sf_mex_assign(&c37_w_y, sf_mex_create("y", &c37_w_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_y, c37_w_y, "waitingTimeAfterYoga", "waitingTimeAfterYoga",
                  0);
  for (c37_i5 = 0; c37_i5 < 13; c37_i5++) {
    c37_x_u[c37_i5] = c37_u.jointsSmoothingTimes[c37_i5];
  }

  c37_x_y = NULL;
  sf_mex_assign(&c37_x_y, sf_mex_create("y", c37_x_u, 0, 0U, 1U, 0U, 2, 13, 1),
                FALSE);
  sf_mex_addfield(c37_y, c37_x_y, "jointsSmoothingTimes", "jointsSmoothingTimes",
                  0);
  c37_y_u = c37_u.tBalancing;
  c37_y_y = NULL;
  sf_mex_assign(&c37_y_y, sf_mex_create("y", &c37_y_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c37_y, c37_y_y, "tBalancing", "tBalancing", 0);
  c37_ab_u = c37_u.alsoSitDown;
  c37_ab_y = NULL;
  sf_mex_assign(&c37_ab_y, sf_mex_create("y", &c37_ab_u, 11, 0U, 0U, 0U, 0),
                FALSE);
  sf_mex_addfield(c37_y, c37_ab_y, "alsoSitDown", "alsoSitDown", 0);
  for (c37_i6 = 0; c37_i6 < 8; c37_i6++) {
    c37_bb_u[c37_i6] = c37_u.jointsAndCoMSmoothingTimes[c37_i6];
  }

  c37_bb_y = NULL;
  sf_mex_assign(&c37_bb_y, sf_mex_create("y", c37_bb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c37_y, c37_bb_y, "jointsAndCoMSmoothingTimes",
                  "jointsAndCoMSmoothingTimes", 0);
  c37_cb_u = c37_u.CoM;
  c37_cb_y = NULL;
  sf_mex_assign(&c37_cb_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  for (c37_i7 = 0; c37_i7 < 24; c37_i7++) {
    c37_db_u[c37_i7] = c37_cb_u.standUpDeltaCoM[c37_i7];
  }

  c37_db_y = NULL;
  sf_mex_assign(&c37_db_y, sf_mex_create("y", c37_db_u, 0, 0U, 1U, 0U, 2, 8, 3),
                FALSE);
  sf_mex_addfield(c37_cb_y, c37_db_y, "standUpDeltaCoM", "standUpDeltaCoM", 0);
  sf_mex_addfield(c37_y, c37_cb_y, "CoM", "CoM", 0);
  for (c37_i8 = 0; c37_i8 < 8; c37_i8++) {
    c37_eb_u[c37_i8] = c37_u.LwrenchThreshold[c37_i8];
  }

  c37_eb_y = NULL;
  sf_mex_assign(&c37_eb_y, sf_mex_create("y", c37_eb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c37_y, c37_eb_y, "LwrenchThreshold", "LwrenchThreshold", 0);
  for (c37_i9 = 0; c37_i9 < 8; c37_i9++) {
    c37_fb_u[c37_i9] = c37_u.RwrenchThreshold[c37_i9];
  }

  c37_fb_y = NULL;
  sf_mex_assign(&c37_fb_y, sf_mex_create("y", c37_fb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c37_y, c37_fb_y, "RwrenchThreshold", "RwrenchThreshold", 0);
  for (c37_i10 = 0; c37_i10 < 8; c37_i10++) {
    c37_gb_u[c37_i10] = c37_u.RArmThreshold[c37_i10];
  }

  c37_gb_y = NULL;
  sf_mex_assign(&c37_gb_y, sf_mex_create("y", c37_gb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c37_y, c37_gb_y, "RArmThreshold", "RArmThreshold", 0);
  for (c37_i11 = 0; c37_i11 < 8; c37_i11++) {
    c37_hb_u[c37_i11] = c37_u.LArmThreshold[c37_i11];
  }

  c37_hb_y = NULL;
  sf_mex_assign(&c37_hb_y, sf_mex_create("y", c37_hb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c37_y, c37_hb_y, "LArmThreshold", "LArmThreshold", 0);
  sf_mex_assign(&c37_mxArrayOutData, c37_y, FALSE);
  return c37_mxArrayOutData;
}

static void c37_c_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  c37_struct_rUGQ0INmvPpaxIctEGl5sE *c37_y)
{
  emlrtMsgIdentifier c37_thisId;
  static const char * c37_fieldNames[19] = { "skipYoga", "demoOnlyRightFoot",
    "yogaAlsoOnRightFoot", "yogaInLoop", "com", "wrench", "joints", "stateAt0",
    "DT", "waitingTimeAfterYoga", "jointsSmoothingTimes", "tBalancing",
    "alsoSitDown", "jointsAndCoMSmoothingTimes", "CoM", "LwrenchThreshold",
    "RwrenchThreshold", "RArmThreshold", "LArmThreshold" };

  c37_thisId.fParent = c37_parentId;
  sf_mex_check_struct(c37_parentId, c37_u, 19, c37_fieldNames, 0U, 0);
  c37_thisId.fIdentifier = "skipYoga";
  c37_y->skipYoga = c37_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "skipYoga", "skipYoga", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "demoOnlyRightFoot";
  c37_y->demoOnlyRightFoot = c37_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "demoOnlyRightFoot", "demoOnlyRightFoot", 0)),
    &c37_thisId);
  c37_thisId.fIdentifier = "yogaAlsoOnRightFoot";
  c37_y->yogaAlsoOnRightFoot = c37_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot", 0)),
    &c37_thisId);
  c37_thisId.fIdentifier = "yogaInLoop";
  c37_y->yogaInLoop = c37_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "yogaInLoop", "yogaInLoop", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "com";
  c37_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u, "com",
    "com", 0)), &c37_thisId, &c37_y->com);
  c37_thisId.fIdentifier = "wrench";
  c37_y->wrench = c37_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "wrench", "wrench", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "joints";
  c37_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "joints", "joints", 0)), &c37_thisId, &c37_y->joints);
  c37_thisId.fIdentifier = "stateAt0";
  c37_y->stateAt0 = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "stateAt0", "stateAt0", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "DT";
  c37_y->DT = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c37_u, "DT", "DT", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "waitingTimeAfterYoga";
  c37_y->waitingTimeAfterYoga = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "waitingTimeAfterYoga", "waitingTimeAfterYoga", 0)),
    &c37_thisId);
  c37_thisId.fIdentifier = "jointsSmoothingTimes";
  c37_l_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "jointsSmoothingTimes", "jointsSmoothingTimes", 0)), &c37_thisId,
    c37_y->jointsSmoothingTimes);
  c37_thisId.fIdentifier = "tBalancing";
  c37_y->tBalancing = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "tBalancing", "tBalancing", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "alsoSitDown";
  c37_y->alsoSitDown = c37_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "alsoSitDown", "alsoSitDown", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "jointsAndCoMSmoothingTimes";
  c37_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "jointsAndCoMSmoothingTimes", "jointsAndCoMSmoothingTimes", 0)), &c37_thisId,
    c37_y->jointsAndCoMSmoothingTimes);
  c37_thisId.fIdentifier = "CoM";
  c37_n_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u, "CoM",
    "CoM", 0)), &c37_thisId, &c37_y->CoM);
  c37_thisId.fIdentifier = "LwrenchThreshold";
  c37_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "LwrenchThreshold", "LwrenchThreshold", 0)), &c37_thisId,
    c37_y->LwrenchThreshold);
  c37_thisId.fIdentifier = "RwrenchThreshold";
  c37_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "RwrenchThreshold", "RwrenchThreshold", 0)), &c37_thisId,
    c37_y->RwrenchThreshold);
  c37_thisId.fIdentifier = "RArmThreshold";
  c37_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "RArmThreshold", "RArmThreshold", 0)), &c37_thisId, c37_y->RArmThreshold);
  c37_thisId.fIdentifier = "LArmThreshold";
  c37_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "LArmThreshold", "LArmThreshold", 0)), &c37_thisId, c37_y->LArmThreshold);
  sf_mex_destroy(&c37_u);
}

static boolean_T c37_d_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId)
{
  boolean_T c37_y;
  boolean_T c37_b0;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), &c37_b0, 1, 11, 0U, 0, 0U, 0);
  c37_y = c37_b0;
  sf_mex_destroy(&c37_u);
  return c37_y;
}

static void c37_e_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  c37_struct_DnBdbfPNxiIjhNOyZMmfsE *c37_y)
{
  emlrtMsgIdentifier c37_thisId;
  static const char * c37_fieldNames[2] = { "threshold", "states" };

  c37_thisId.fParent = c37_parentId;
  sf_mex_check_struct(c37_parentId, c37_u, 2, c37_fieldNames, 0U, 0);
  c37_thisId.fIdentifier = "threshold";
  c37_y->threshold = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "threshold", "threshold", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "states";
  c37_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "states", "states", 0)), &c37_thisId, c37_y->states);
  sf_mex_destroy(&c37_u);
}

static void c37_f_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[39])
{
  real_T c37_dv0[39];
  int32_T c37_i12;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), c37_dv0, 1, 0, 0U, 1, 0U, 2, 13,
                3);
  for (c37_i12 = 0; c37_i12 < 39; c37_i12++) {
    c37_y[c37_i12] = c37_dv0[c37_i12];
  }

  sf_mex_destroy(&c37_u);
}

static c37_struct_KJR2itYvhBuAkuR6dKZHUC c37_g_emlrt_marshallIn
  (SFc37_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c37_u,
   const emlrtMsgIdentifier *c37_parentId)
{
  c37_struct_KJR2itYvhBuAkuR6dKZHUC c37_y;
  emlrtMsgIdentifier c37_thisId;
  static const char * c37_fieldNames[2] = { "thresholdContactOn",
    "thresholdContactOff" };

  c37_thisId.fParent = c37_parentId;
  sf_mex_check_struct(c37_parentId, c37_u, 2, c37_fieldNames, 0U, 0);
  c37_thisId.fIdentifier = "thresholdContactOn";
  c37_y.thresholdContactOn = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "thresholdContactOn", "thresholdContactOn", 0)),
    &c37_thisId);
  c37_thisId.fIdentifier = "thresholdContactOff";
  c37_y.thresholdContactOff = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "thresholdContactOff", "thresholdContactOff", 0)),
    &c37_thisId);
  sf_mex_destroy(&c37_u);
  return c37_y;
}

static void c37_h_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  c37_struct_0U0wBk2LiR1OqsMsUngxdD *c37_y)
{
  emlrtMsgIdentifier c37_thisId;
  static const char * c37_fieldNames[8] = { "thresholdNotInContact",
    "thresholdInContact", "pauseTimeLastPostureL", "pauseTimeLastPostureR",
    "states", "pointsL", "pointsR", "standUpPositions" };

  c37_thisId.fParent = c37_parentId;
  sf_mex_check_struct(c37_parentId, c37_u, 8, c37_fieldNames, 0U, 0);
  c37_thisId.fIdentifier = "thresholdNotInContact";
  c37_y->thresholdNotInContact = c37_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c37_u, "thresholdNotInContact",
    "thresholdNotInContact", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "thresholdInContact";
  c37_y->thresholdInContact = c37_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c37_u, "thresholdInContact", "thresholdInContact", 0)),
    &c37_thisId);
  c37_thisId.fIdentifier = "pauseTimeLastPostureL";
  c37_y->pauseTimeLastPostureL = c37_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c37_u, "pauseTimeLastPostureL",
    "pauseTimeLastPostureL", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "pauseTimeLastPostureR";
  c37_y->pauseTimeLastPostureR = c37_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c37_u, "pauseTimeLastPostureR",
    "pauseTimeLastPostureR", 0)), &c37_thisId);
  c37_thisId.fIdentifier = "states";
  c37_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "states", "states", 0)), &c37_thisId, c37_y->states);
  c37_thisId.fIdentifier = "pointsL";
  c37_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "pointsL", "pointsL", 0)), &c37_thisId, c37_y->pointsL);
  c37_thisId.fIdentifier = "pointsR";
  c37_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "pointsR", "pointsR", 0)), &c37_thisId, c37_y->pointsR);
  c37_thisId.fIdentifier = "standUpPositions";
  c37_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "standUpPositions", "standUpPositions", 0)), &c37_thisId,
    c37_y->standUpPositions);
  sf_mex_destroy(&c37_u);
}

static void c37_i_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[299])
{
  real_T c37_dv1[299];
  int32_T c37_i13;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), c37_dv1, 1, 0, 0U, 1, 0U, 2, 13,
                23);
  for (c37_i13 = 0; c37_i13 < 299; c37_i13++) {
    c37_y[c37_i13] = c37_dv1[c37_i13];
  }

  sf_mex_destroy(&c37_u);
}

static void c37_j_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[192])
{
  real_T c37_dv2[192];
  int32_T c37_i14;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), c37_dv2, 1, 0, 0U, 1, 0U, 2, 8,
                24);
  for (c37_i14 = 0; c37_i14 < 192; c37_i14++) {
    c37_y[c37_i14] = c37_dv2[c37_i14];
  }

  sf_mex_destroy(&c37_u);
}

static void c37_k_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[72])
{
  real_T c37_dv3[72];
  int32_T c37_i15;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), c37_dv3, 1, 0, 0U, 1, 0U, 2, 8,
                9);
  for (c37_i15 = 0; c37_i15 < 72; c37_i15++) {
    c37_y[c37_i15] = c37_dv3[c37_i15];
  }

  sf_mex_destroy(&c37_u);
}

static void c37_l_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[13])
{
  real_T c37_dv4[13];
  int32_T c37_i16;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), c37_dv4, 1, 0, 0U, 1, 0U, 2, 13,
                1);
  for (c37_i16 = 0; c37_i16 < 13; c37_i16++) {
    c37_y[c37_i16] = c37_dv4[c37_i16];
  }

  sf_mex_destroy(&c37_u);
}

static void c37_m_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[8])
{
  real_T c37_dv5[8];
  int32_T c37_i17;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), c37_dv5, 1, 0, 0U, 1, 0U, 2, 8,
                1);
  for (c37_i17 = 0; c37_i17 < 8; c37_i17++) {
    c37_y[c37_i17] = c37_dv5[c37_i17];
  }

  sf_mex_destroy(&c37_u);
}

static void c37_n_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  c37_struct_9LpOi5JXaV67jTuay8hWaH *c37_y)
{
  emlrtMsgIdentifier c37_thisId;
  static const char * c37_fieldNames[1] = { "standUpDeltaCoM" };

  c37_thisId.fParent = c37_parentId;
  sf_mex_check_struct(c37_parentId, c37_u, 1, c37_fieldNames, 0U, 0);
  c37_thisId.fIdentifier = "standUpDeltaCoM";
  c37_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c37_u,
    "standUpDeltaCoM", "standUpDeltaCoM", 0)), &c37_thisId,
    c37_y->standUpDeltaCoM);
  sf_mex_destroy(&c37_u);
}

static void c37_o_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId,
  real_T c37_y[24])
{
  real_T c37_dv6[24];
  int32_T c37_i18;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), c37_dv6, 1, 0, 0U, 1, 0U, 2, 8,
                3);
  for (c37_i18 = 0; c37_i18 < 24; c37_i18++) {
    c37_y[c37_i18] = c37_dv6[c37_i18];
  }

  sf_mex_destroy(&c37_u);
}

static void c37_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c37_mxArrayInData, const char_T *c37_varName, void *c37_outData)
{
  const mxArray *c37_b_sm;
  const char_T *c37_identifier;
  emlrtMsgIdentifier c37_thisId;
  c37_struct_rUGQ0INmvPpaxIctEGl5sE c37_y;
  SFc37_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc37_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c37_b_sm = sf_mex_dup(c37_mxArrayInData);
  c37_identifier = c37_varName;
  c37_thisId.fIdentifier = c37_identifier;
  c37_thisId.fParent = NULL;
  c37_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c37_b_sm), &c37_thisId,
    &c37_y);
  sf_mex_destroy(&c37_b_sm);
  *(c37_struct_rUGQ0INmvPpaxIctEGl5sE *)c37_outData = c37_y;
  sf_mex_destroy(&c37_mxArrayInData);
}

const mxArray *sf_c37_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c37_nameCaptureInfo = NULL;
  c37_nameCaptureInfo = NULL;
  sf_mex_assign(&c37_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), FALSE);
  return c37_nameCaptureInfo;
}

static const mxArray *c37_c_sf_marshallOut(void *chartInstanceVoid, void
  *c37_inData)
{
  const mxArray *c37_mxArrayOutData = NULL;
  int32_T c37_u;
  const mxArray *c37_y = NULL;
  SFc37_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc37_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c37_mxArrayOutData = NULL;
  c37_u = *(int32_T *)c37_inData;
  c37_y = NULL;
  sf_mex_assign(&c37_y, sf_mex_create("y", &c37_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c37_mxArrayOutData, c37_y, FALSE);
  return c37_mxArrayOutData;
}

static int32_T c37_p_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId)
{
  int32_T c37_y;
  int32_T c37_i19;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), &c37_i19, 1, 6, 0U, 0, 0U, 0);
  c37_y = c37_i19;
  sf_mex_destroy(&c37_u);
  return c37_y;
}

static void c37_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c37_mxArrayInData, const char_T *c37_varName, void *c37_outData)
{
  const mxArray *c37_b_sfEvent;
  const char_T *c37_identifier;
  emlrtMsgIdentifier c37_thisId;
  int32_T c37_y;
  SFc37_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc37_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c37_b_sfEvent = sf_mex_dup(c37_mxArrayInData);
  c37_identifier = c37_varName;
  c37_thisId.fIdentifier = c37_identifier;
  c37_thisId.fParent = NULL;
  c37_y = c37_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c37_b_sfEvent),
    &c37_thisId);
  sf_mex_destroy(&c37_b_sfEvent);
  *(int32_T *)c37_outData = c37_y;
  sf_mex_destroy(&c37_mxArrayInData);
}

static uint8_T c37_q_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_b_is_active_c37_torqueBalancing2012b, const
  char_T *c37_identifier)
{
  uint8_T c37_y;
  emlrtMsgIdentifier c37_thisId;
  c37_thisId.fIdentifier = c37_identifier;
  c37_thisId.fParent = NULL;
  c37_y = c37_r_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c37_b_is_active_c37_torqueBalancing2012b), &c37_thisId);
  sf_mex_destroy(&c37_b_is_active_c37_torqueBalancing2012b);
  return c37_y;
}

static uint8_T c37_r_emlrt_marshallIn(SFc37_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c37_u, const emlrtMsgIdentifier *c37_parentId)
{
  uint8_T c37_y;
  uint8_T c37_u0;
  sf_mex_import(c37_parentId, sf_mex_dup(c37_u), &c37_u0, 1, 3, 0U, 0, 0U, 0);
  c37_y = c37_u0;
  sf_mex_destroy(&c37_u);
  return c37_y;
}

static void init_dsm_address_info(SFc37_torqueBalancing2012bInstanceStruct
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

void sf_c37_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3160629673U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2237194820U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3191434137U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4253061203U);
}

mxArray *sf_c37_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("40WA98L0ERNwXhQSEKm7zG");
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

mxArray *sf_c37_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c37_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"thresholdArmContactLeft\",},{M[1],M[7],T\"thresholdArmContactRight\",},{M[8],M[0],T\"is_active_c37_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c37_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc37_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc37_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           37,
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
          _SFD_SET_DATA_PROPS(1,2,0,1,"thresholdArmContactLeft");
          _SFD_SET_DATA_PROPS(2,2,0,1,"thresholdArmContactRight");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,193);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c37_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c37_sf_marshallOut,(MexInFcnForType)c37_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c37_sf_marshallOut,(MexInFcnForType)c37_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c37_b_sf_marshallOut,(MexInFcnForType)
          c37_b_sf_marshallIn);

        {
          real_T *c37_state;
          real_T *c37_thresholdArmContactLeft;
          real_T *c37_thresholdArmContactRight;
          c37_thresholdArmContactRight = (real_T *)ssGetOutputPortSignal
            (chartInstance->S, 2);
          c37_thresholdArmContactLeft = (real_T *)ssGetOutputPortSignal
            (chartInstance->S, 1);
          c37_state = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c37_state);
          _SFD_SET_DATA_VALUE_PTR(1U, c37_thresholdArmContactLeft);
          _SFD_SET_DATA_VALUE_PTR(2U, c37_thresholdArmContactRight);
          _SFD_SET_DATA_VALUE_PTR(3U, &chartInstance->c37_sm);
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
  return "Kx9wS4Zz1XwTxccw4ZeqtD";
}

static void sf_opaque_initialize_c37_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc37_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c37_torqueBalancing2012b
    ((SFc37_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c37_torqueBalancing2012b((SFc37_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c37_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c37_torqueBalancing2012b((SFc37_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c37_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c37_torqueBalancing2012b((SFc37_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c37_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c37_torqueBalancing2012b((SFc37_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c37_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c37_torqueBalancing2012b
    ((SFc37_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c37_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c37_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c37_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c37_torqueBalancing2012b
    ((SFc37_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c37_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c37_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c37_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c37_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c37_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc37_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c37_torqueBalancing2012b((SFc37_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc37_torqueBalancing2012b
    ((SFc37_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c37_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c37_torqueBalancing2012b
      ((SFc37_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c37_torqueBalancing2012b(SimStruct *S)
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
      37);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,37,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,37,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,37);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,37,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,37,2);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,37);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1826152515U));
  ssSetChecksum1(S,(284593176U));
  ssSetChecksum2(S,(1273468239U));
  ssSetChecksum3(S,(215512194U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c37_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c37_torqueBalancing2012b(SimStruct *S)
{
  SFc37_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc37_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc37_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc37_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c37_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c37_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c37_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c37_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c37_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c37_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c37_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c37_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c37_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c37_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c37_torqueBalancing2012b;
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

void c37_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c37_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c37_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c37_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c37_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
