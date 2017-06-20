/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c6_torqueBalancing2012b.h"
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
static const char * c6_debug_family_names[21] = { "nargin", "nargout",
  "connection", "wrench_rightFoot", "wrench_leftFoot", "CoM_0", "q0", "w_CoM",
  "CoMIn", "qIn", "constraintsIn", "l_sole_H_b", "r_sole_H_b", "sm", "gain",
  "w_H_b", "CoMDes", "qDes", "constraints", "impedances", "currentState" };

static const char * c6_b_debug_family_names[25] = { "CoMError", "nargin",
  "nargout", "connection", "CoM_0", "q0", "w_CoM", "CoMIn", "qIn",
  "constraintsIn", "wrench_rightFoot", "wrench_leftFoot", "l_sole_H_b",
  "r_sole_H_b", "sm", "gain", "CoMDes", "qDes", "constraints", "currentState",
  "impedances", "w_H_b", "state", "fixedLink", "w_H_fixedLink" };

/* Function Declarations */
static void initialize_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void c6_update_debugger_state_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c6_st);
static void finalize_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c6_torqueBalancing2012b(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void c6_stateMachineWalking(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, boolean_T c6_connection, real_T c6_CoM_0[3], real_T c6_q0[23],
  real_T c6_w_CoM[3], real_T c6_CoMIn[3], real_T c6_qIn[23], real_T
  c6_constraintsIn[2], real_T c6_wrench_rightFoot[6], real_T c6_wrench_leftFoot
  [6], real_T c6_l_sole_H_b[16], real_T c6_r_sole_H_b[16],
  c6_struct_rUGQ0INmvPpaxIctEGl5sE *c6_b_sm, c6_struct_kzTB0QQWoOlMoMhgKf6sK
  *c6_b_gain, real_T c6_CoMDes[3], real_T c6_qDes[23], real_T c6_constraints[2],
  real_T *c6_currentState, real_T c6_impedances[23], real_T c6_w_H_b[16]);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static real_T c6_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_currentState, const char_T *c6_identifier);
static real_T c6_b_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_c_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_impedances, const char_T *c6_identifier,
  real_T c6_y[23]);
static void c6_d_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[23]);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_e_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_constraints, const char_T *c6_identifier,
  real_T c6_y[2]);
static void c6_f_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[2]);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_g_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_qDes, const char_T *c6_identifier, real_T
  c6_y[23]);
static void c6_h_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[23]);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_i_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_CoMDes, const char_T *c6_identifier, real_T
  c6_y[3]);
static void c6_j_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3]);
static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_k_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_w_H_b, const char_T *c6_identifier, real_T
  c6_y[16]);
static void c6_l_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[16]);
static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_m_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_kzTB0QQWoOlMoMhgKf6sK *c6_y);
static void c6_n_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[9]);
static void c6_o_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4]);
static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_p_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_rUGQ0INmvPpaxIctEGl5sE *c6_y);
static boolean_T c6_q_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_r_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_DnBdbfPNxiIjhNOyZMmfsE *c6_y);
static void c6_s_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[39]);
static c6_struct_KJR2itYvhBuAkuR6dKZHUC c6_t_emlrt_marshallIn
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c6_u,
   const emlrtMsgIdentifier *c6_parentId);
static void c6_u_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_0U0wBk2LiR1OqsMsUngxdD *c6_y);
static void c6_v_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[299]);
static void c6_w_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[192]);
static void c6_x_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[72]);
static void c6_y_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[13]);
static void c6_ab_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[8]);
static void c6_bb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_9LpOi5JXaV67jTuay8hWaH *c6_y);
static void c6_cb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[24]);
static void c6_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_i_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_j_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_k_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_l_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_db_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_b_w_H_fixedLink, const char_T *c6_identifier,
  real_T c6_y[16]);
static void c6_eb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[16]);
static void c6_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_m_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_fb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_b_fixedLink, const char_T *c6_identifier);
static real_T c6_gb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_n_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_hb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_b_state, const char_T *c6_identifier);
static real_T c6_ib_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_jb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[6]);
static void c6_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[148]);
static void c6_b_info_helper(c6_ResolvedFunctionInfo c6_info[148]);
static void c6_c_info_helper(c6_ResolvedFunctionInfo c6_info[148]);
static void c6_eye(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
                   real_T c6_I[16]);
static void c6_eml_scalar_eg(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c6_realmin(SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void c6_eml_eps(SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static boolean_T c6_any(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c6_x[2]);
static void c6_mrdivide(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c6_A[16], real_T c6_B[16], real_T c6_y[16]);
static void c6_eps(SFc6_torqueBalancing2012bInstanceStruct *chartInstance);
static void c6_eml_matlab_zgetrf(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], real_T c6_b_A[16], int32_T c6_ipiv[4],
  int32_T *c6_info);
static void c6_check_forloop_overflow_error
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c6_overflow);
static void c6_eml_xger(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c6_m, int32_T c6_n, real_T c6_alpha1, int32_T c6_ix0, int32_T c6_iy0,
  real_T c6_A[16], int32_T c6_ia0, real_T c6_b_A[16]);
static void c6_eml_warning(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c6_eml_xtrsm(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c6_A[16], real_T c6_B[16], real_T c6_b_B[16]);
static void c6_below_threshold(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c6_b_eml_scalar_eg(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c6_b_eml_xtrsm(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], real_T c6_B[16], real_T c6_b_B[16]);
static const mxArray *c6_o_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_kb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_lb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_torqueBalancing2012b, const
  char_T *c6_identifier);
static uint8_T c6_mb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_b_eml_matlab_zgetrf(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], int32_T c6_ipiv[4], int32_T *c6_info);
static void c6_b_eml_xger(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c6_m, int32_T c6_n, real_T c6_alpha1, int32_T c6_ix0, int32_T c6_iy0,
  real_T c6_A[16], int32_T c6_ia0);
static void c6_c_eml_xtrsm(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], real_T c6_B[16]);
static void c6_d_eml_xtrsm(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], real_T c6_B[16]);
static void init_dsm_address_info(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c6_state_not_empty = FALSE;
  chartInstance->c6_fixedLink_not_empty = FALSE;
  chartInstance->c6_w_H_fixedLink_not_empty = FALSE;
  chartInstance->c6_is_active_c6_torqueBalancing2012b = 0U;
}

static void initialize_params_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c6_m0 = NULL;
  const mxArray *c6_mxField;
  c6_struct_rUGQ0INmvPpaxIctEGl5sE c6_r0;
  const mxArray *c6_m1 = NULL;
  const mxArray *c6_b_mxField;
  const mxArray *c6_m2 = NULL;
  const mxArray *c6_c_mxField;
  const mxArray *c6_m3 = NULL;
  const mxArray *c6_d_mxField;
  const mxArray *c6_m4 = NULL;
  const mxArray *c6_e_mxField;
  const mxArray *c6_m5 = NULL;
  const mxArray *c6_f_mxField;
  c6_struct_kzTB0QQWoOlMoMhgKf6sK c6_r1;
  sf_set_error_prefix_string(
    "Error evaluating data 'sm' in the parent workspace.\n");
  c6_m0 = sf_mex_get_sfun_param(chartInstance->S, 1, 1);
  c6_mxField = sf_mex_getfield(c6_m0, "skipYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.skipYoga, 1, 11, 0U,
                      0, 0U, 0);
  c6_mxField = sf_mex_getfield(c6_m0, "demoOnlyRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.demoOnlyRightFoot, 1,
                      11, 0U, 0, 0U, 0);
  c6_mxField = sf_mex_getfield(c6_m0, "yogaAlsoOnRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.yogaAlsoOnRightFoot,
                      1, 11, 0U, 0, 0U, 0);
  c6_mxField = sf_mex_getfield(c6_m0, "yogaInLoop", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.yogaInLoop, 1, 11, 0U,
                      0, 0U, 0);
  c6_mxField = sf_mex_getfield(c6_m0, "com", "sm", 0);
  c6_m1 = sf_mex_dup(c6_mxField);
  c6_b_mxField = sf_mex_getfield(c6_m1, "threshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_b_mxField), &c6_r0.com.threshold, 1, 0,
                      0U, 0, 0U, 0);
  c6_b_mxField = sf_mex_getfield(c6_m1, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_b_mxField), c6_r0.com.states, 1, 0, 0U,
                      1, 0U, 2, 13, 3);
  sf_mex_destroy(&c6_m1);
  c6_mxField = sf_mex_getfield(c6_m0, "wrench", "sm", 0);
  c6_m2 = sf_mex_dup(c6_mxField);
  c6_c_mxField = sf_mex_getfield(c6_m2, "thresholdContactOn", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_c_mxField),
                      &c6_r0.wrench.thresholdContactOn, 1, 0, 0U, 0, 0U, 0);
  c6_c_mxField = sf_mex_getfield(c6_m2, "thresholdContactOff", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_c_mxField),
                      &c6_r0.wrench.thresholdContactOff, 1, 0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c6_m2);
  c6_mxField = sf_mex_getfield(c6_m0, "joints", "sm", 0);
  c6_m3 = sf_mex_dup(c6_mxField);
  c6_d_mxField = sf_mex_getfield(c6_m3, "thresholdNotInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_d_mxField),
                      &c6_r0.joints.thresholdNotInContact, 1, 0, 0U, 0, 0U, 0);
  c6_d_mxField = sf_mex_getfield(c6_m3, "thresholdInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_d_mxField),
                      &c6_r0.joints.thresholdInContact, 1, 0, 0U, 0, 0U, 0);
  c6_d_mxField = sf_mex_getfield(c6_m3, "pauseTimeLastPostureL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_d_mxField),
                      &c6_r0.joints.pauseTimeLastPostureL, 1, 0, 0U, 0, 0U, 0);
  c6_d_mxField = sf_mex_getfield(c6_m3, "pauseTimeLastPostureR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_d_mxField),
                      &c6_r0.joints.pauseTimeLastPostureR, 1, 0, 0U, 0, 0U, 0);
  c6_d_mxField = sf_mex_getfield(c6_m3, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_d_mxField), c6_r0.joints.states, 1, 0,
                      0U, 1, 0U, 2, 13, 23);
  c6_d_mxField = sf_mex_getfield(c6_m3, "pointsL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_d_mxField), c6_r0.joints.pointsL, 1, 0,
                      0U, 1, 0U, 2, 8, 24);
  c6_d_mxField = sf_mex_getfield(c6_m3, "pointsR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_d_mxField), c6_r0.joints.pointsR, 1, 0,
                      0U, 1, 0U, 2, 8, 24);
  c6_d_mxField = sf_mex_getfield(c6_m3, "standUpPositions", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_d_mxField),
                      c6_r0.joints.standUpPositions, 1, 0, 0U, 1, 0U, 2, 8, 9);
  sf_mex_destroy(&c6_m3);
  c6_mxField = sf_mex_getfield(c6_m0, "stateAt0", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.stateAt0, 1, 0, 0U, 0,
                      0U, 0);
  c6_mxField = sf_mex_getfield(c6_m0, "DT", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.DT, 1, 0, 0U, 0, 0U,
                      0);
  c6_mxField = sf_mex_getfield(c6_m0, "waitingTimeAfterYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.waitingTimeAfterYoga,
                      1, 0, 0U, 0, 0U, 0);
  c6_mxField = sf_mex_getfield(c6_m0, "jointsSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), c6_r0.jointsSmoothingTimes,
                      1, 0, 0U, 1, 0U, 2, 13, 1);
  c6_mxField = sf_mex_getfield(c6_m0, "tBalancing", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.tBalancing, 1, 0, 0U,
                      0, 0U, 0);
  c6_mxField = sf_mex_getfield(c6_m0, "alsoSitDown", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), &c6_r0.alsoSitDown, 1, 11,
                      0U, 0, 0U, 0);
  c6_mxField = sf_mex_getfield(c6_m0, "jointsAndCoMSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField),
                      c6_r0.jointsAndCoMSmoothingTimes, 1, 0, 0U, 1, 0U, 2, 8, 1);
  c6_mxField = sf_mex_getfield(c6_m0, "CoM", "sm", 0);
  c6_m4 = sf_mex_dup(c6_mxField);
  c6_e_mxField = sf_mex_getfield(c6_m4, "standUpDeltaCoM", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_e_mxField), c6_r0.CoM.standUpDeltaCoM,
                      1, 0, 0U, 1, 0U, 2, 8, 3);
  sf_mex_destroy(&c6_m4);
  c6_mxField = sf_mex_getfield(c6_m0, "LwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), c6_r0.LwrenchThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  c6_mxField = sf_mex_getfield(c6_m0, "RwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), c6_r0.RwrenchThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  c6_mxField = sf_mex_getfield(c6_m0, "RArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), c6_r0.RArmThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  c6_mxField = sf_mex_getfield(c6_m0, "LArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c6_mxField), c6_r0.LArmThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  sf_mex_destroy(&c6_m0);
  chartInstance->c6_sm = c6_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'gain' in the parent workspace.\n");
  c6_m5 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c6_f_mxField = sf_mex_getfield(c6_m5, "qTildeMax", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), &c6_r1.qTildeMax, 1, 0,
                      0U, 0, 0U, 0);
  c6_f_mxField = sf_mex_getfield(c6_m5, "SmoothingTimeImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), &c6_r1.SmoothingTimeImp,
                      1, 0, 0U, 0, 0U, 0);
  c6_f_mxField = sf_mex_getfield(c6_m5, "SmoothingTimeGainScheduling", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField),
                      &c6_r1.SmoothingTimeGainScheduling, 1, 0, 0U, 0, 0U, 0);
  c6_f_mxField = sf_mex_getfield(c6_m5, "PCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.PCOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c6_f_mxField = sf_mex_getfield(c6_m5, "ICOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.ICOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c6_f_mxField = sf_mex_getfield(c6_m5, "DCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.DCOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c6_f_mxField = sf_mex_getfield(c6_m5, "PAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), &c6_r1.PAngularMomentum,
                      1, 0, 0U, 0, 0U, 0);
  c6_f_mxField = sf_mex_getfield(c6_m5, "DAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), &c6_r1.DAngularMomentum,
                      1, 0, 0U, 0, 0U, 0);
  c6_f_mxField = sf_mex_getfield(c6_m5, "integral", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.integral, 1, 0, 0U,
                      1, 0U, 2, 1, 23);
  c6_f_mxField = sf_mex_getfield(c6_m5, "impedances", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.impedances, 1, 0,
                      0U, 1, 0U, 2, 1, 23);
  c6_f_mxField = sf_mex_getfield(c6_m5, "dampings", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.dampings, 1, 0, 0U,
                      1, 0U, 2, 1, 23);
  c6_f_mxField = sf_mex_getfield(c6_m5, "increasingRatesImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.increasingRatesImp,
                      1, 0, 0U, 1, 0U, 2, 1, 23);
  c6_f_mxField = sf_mex_getfield(c6_m5, "footSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.footSize, 1, 0, 0U,
                      1, 0U, 2, 2, 2);
  c6_f_mxField = sf_mex_getfield(c6_m5, "legSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c6_f_mxField), c6_r1.legSize, 1, 0, 0U,
                      1, 0U, 2, 2, 2);
  sf_mex_destroy(&c6_m5);
  chartInstance->c6_gain = c6_r1;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c6_update_debugger_state_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  int32_T c6_i0;
  real_T c6_u[3];
  const mxArray *c6_b_y = NULL;
  int32_T c6_i1;
  real_T c6_b_u[2];
  const mxArray *c6_c_y = NULL;
  real_T c6_hoistedGlobal;
  real_T c6_c_u;
  const mxArray *c6_d_y = NULL;
  int32_T c6_i2;
  real_T c6_d_u[23];
  const mxArray *c6_e_y = NULL;
  int32_T c6_i3;
  real_T c6_e_u[23];
  const mxArray *c6_f_y = NULL;
  int32_T c6_i4;
  real_T c6_f_u[16];
  const mxArray *c6_g_y = NULL;
  real_T c6_b_hoistedGlobal;
  real_T c6_g_u;
  const mxArray *c6_h_y = NULL;
  real_T c6_c_hoistedGlobal;
  real_T c6_h_u;
  const mxArray *c6_i_y = NULL;
  int32_T c6_i5;
  real_T c6_i_u[16];
  const mxArray *c6_j_y = NULL;
  uint8_T c6_d_hoistedGlobal;
  uint8_T c6_j_u;
  const mxArray *c6_k_y = NULL;
  real_T *c6_currentState;
  real_T (*c6_w_H_b)[16];
  real_T (*c6_qDes)[23];
  real_T (*c6_impedances)[23];
  real_T (*c6_constraints)[2];
  real_T (*c6_CoMDes)[3];
  c6_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c6_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 5);
  c6_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 4);
  c6_qDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c6_CoMDes = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellarray(10), FALSE);
  for (c6_i0 = 0; c6_i0 < 3; c6_i0++) {
    c6_u[c6_i0] = (*c6_CoMDes)[c6_i0];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  for (c6_i1 = 0; c6_i1 < 2; c6_i1++) {
    c6_b_u[c6_i1] = (*c6_constraints)[c6_i1];
  }

  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_b_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_setcell(c6_y, 1, c6_c_y);
  c6_hoistedGlobal = *c6_currentState;
  c6_c_u = c6_hoistedGlobal;
  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 2, c6_d_y);
  for (c6_i2 = 0; c6_i2 < 23; c6_i2++) {
    c6_d_u[c6_i2] = (*c6_impedances)[c6_i2];
  }

  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", c6_d_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_setcell(c6_y, 3, c6_e_y);
  for (c6_i3 = 0; c6_i3 < 23; c6_i3++) {
    c6_e_u[c6_i3] = (*c6_qDes)[c6_i3];
  }

  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_create("y", c6_e_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_setcell(c6_y, 4, c6_f_y);
  for (c6_i4 = 0; c6_i4 < 16; c6_i4++) {
    c6_f_u[c6_i4] = (*c6_w_H_b)[c6_i4];
  }

  c6_g_y = NULL;
  sf_mex_assign(&c6_g_y, sf_mex_create("y", c6_f_u, 0, 0U, 1U, 0U, 2, 4, 4),
                FALSE);
  sf_mex_setcell(c6_y, 5, c6_g_y);
  c6_b_hoistedGlobal = chartInstance->c6_fixedLink;
  c6_g_u = c6_b_hoistedGlobal;
  c6_h_y = NULL;
  if (!chartInstance->c6_fixedLink_not_empty) {
    sf_mex_assign(&c6_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c6_h_y, sf_mex_create("y", &c6_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c6_y, 6, c6_h_y);
  c6_c_hoistedGlobal = chartInstance->c6_state;
  c6_h_u = c6_c_hoistedGlobal;
  c6_i_y = NULL;
  if (!chartInstance->c6_state_not_empty) {
    sf_mex_assign(&c6_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c6_i_y, sf_mex_create("y", &c6_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c6_y, 7, c6_i_y);
  for (c6_i5 = 0; c6_i5 < 16; c6_i5++) {
    c6_i_u[c6_i5] = chartInstance->c6_w_H_fixedLink[c6_i5];
  }

  c6_j_y = NULL;
  if (!chartInstance->c6_w_H_fixedLink_not_empty) {
    sf_mex_assign(&c6_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c6_j_y, sf_mex_create("y", c6_i_u, 0, 0U, 1U, 0U, 2, 4, 4),
                  FALSE);
  }

  sf_mex_setcell(c6_y, 8, c6_j_y);
  c6_d_hoistedGlobal = chartInstance->c6_is_active_c6_torqueBalancing2012b;
  c6_j_u = c6_d_hoistedGlobal;
  c6_k_y = NULL;
  sf_mex_assign(&c6_k_y, sf_mex_create("y", &c6_j_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 9, c6_k_y);
  sf_mex_assign(&c6_st, c6_y, FALSE);
  return c6_st;
}

static void set_sim_state_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c6_st)
{
  const mxArray *c6_u;
  real_T c6_dv0[3];
  int32_T c6_i6;
  real_T c6_dv1[2];
  int32_T c6_i7;
  real_T c6_dv2[23];
  int32_T c6_i8;
  real_T c6_dv3[23];
  int32_T c6_i9;
  real_T c6_dv4[16];
  int32_T c6_i10;
  real_T c6_dv5[16];
  int32_T c6_i11;
  real_T *c6_currentState;
  real_T (*c6_CoMDes)[3];
  real_T (*c6_constraints)[2];
  real_T (*c6_qDes)[23];
  real_T (*c6_w_H_b)[16];
  real_T (*c6_impedances)[23];
  c6_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c6_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 5);
  c6_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 4);
  c6_qDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c6_CoMDes = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = TRUE;
  c6_u = sf_mex_dup(c6_st);
  c6_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)),
                        "CoMDes", c6_dv0);
  for (c6_i6 = 0; c6_i6 < 3; c6_i6++) {
    (*c6_CoMDes)[c6_i6] = c6_dv0[c6_i6];
  }

  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 1)),
                        "constraints", c6_dv1);
  for (c6_i7 = 0; c6_i7 < 2; c6_i7++) {
    (*c6_constraints)[c6_i7] = c6_dv1[c6_i7];
  }

  *c6_currentState = c6_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c6_u, 2)), "currentState");
  c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 3)),
                        "impedances", c6_dv2);
  for (c6_i8 = 0; c6_i8 < 23; c6_i8++) {
    (*c6_impedances)[c6_i8] = c6_dv2[c6_i8];
  }

  c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 4)),
                        "qDes", c6_dv3);
  for (c6_i9 = 0; c6_i9 < 23; c6_i9++) {
    (*c6_qDes)[c6_i9] = c6_dv3[c6_i9];
  }

  c6_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 5)),
                        "w_H_b", c6_dv4);
  for (c6_i10 = 0; c6_i10 < 16; c6_i10++) {
    (*c6_w_H_b)[c6_i10] = c6_dv4[c6_i10];
  }

  chartInstance->c6_fixedLink = c6_fb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c6_u, 6)), "fixedLink");
  chartInstance->c6_state = c6_hb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c6_u, 7)), "state");
  c6_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 8)),
    "w_H_fixedLink", c6_dv5);
  for (c6_i11 = 0; c6_i11 < 16; c6_i11++) {
    chartInstance->c6_w_H_fixedLink[c6_i11] = c6_dv5[c6_i11];
  }

  chartInstance->c6_is_active_c6_torqueBalancing2012b = c6_lb_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 9)),
     "is_active_c6_torqueBalancing2012b");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c6_torqueBalancing2012b(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c6_i12;
  int32_T c6_i13;
  int32_T c6_i14;
  int32_T c6_i15;
  int32_T c6_i16;
  int32_T c6_i17;
  int32_T c6_i18;
  int32_T c6_i19;
  int32_T c6_i20;
  int32_T c6_i21;
  int32_T c6_i22;
  int32_T c6_i23;
  int32_T c6_i24;
  int32_T c6_i25;
  int32_T c6_i26;
  boolean_T c6_hoistedGlobal;
  boolean_T c6_connection;
  int32_T c6_i27;
  real_T c6_wrench_rightFoot[6];
  int32_T c6_i28;
  real_T c6_wrench_leftFoot[6];
  int32_T c6_i29;
  real_T c6_CoM_0[3];
  int32_T c6_i30;
  real_T c6_q0[23];
  int32_T c6_i31;
  real_T c6_w_CoM[3];
  int32_T c6_i32;
  real_T c6_CoMIn[3];
  int32_T c6_i33;
  real_T c6_qIn[23];
  int32_T c6_i34;
  real_T c6_constraintsIn[2];
  int32_T c6_i35;
  real_T c6_l_sole_H_b[16];
  int32_T c6_i36;
  real_T c6_r_sole_H_b[16];
  c6_struct_rUGQ0INmvPpaxIctEGl5sE c6_b_sm;
  c6_struct_kzTB0QQWoOlMoMhgKf6sK c6_b_gain;
  uint32_T c6_debug_family_var_map[21];
  real_T c6_nargin = 13.0;
  real_T c6_nargout = 6.0;
  real_T c6_w_H_b[16];
  real_T c6_CoMDes[3];
  real_T c6_qDes[23];
  real_T c6_constraints[2];
  real_T c6_impedances[23];
  real_T c6_currentState;
  int32_T c6_i37;
  real_T c6_b_CoM_0[3];
  int32_T c6_i38;
  real_T c6_b_q0[23];
  int32_T c6_i39;
  real_T c6_b_w_CoM[3];
  int32_T c6_i40;
  real_T c6_b_CoMIn[3];
  int32_T c6_i41;
  real_T c6_b_qIn[23];
  int32_T c6_i42;
  real_T c6_b_constraintsIn[2];
  int32_T c6_i43;
  real_T c6_b_wrench_rightFoot[6];
  int32_T c6_i44;
  real_T c6_b_wrench_leftFoot[6];
  int32_T c6_i45;
  real_T c6_b_l_sole_H_b[16];
  int32_T c6_i46;
  real_T c6_b_r_sole_H_b[16];
  c6_struct_rUGQ0INmvPpaxIctEGl5sE c6_c_sm;
  c6_struct_kzTB0QQWoOlMoMhgKf6sK c6_c_gain;
  real_T c6_b_w_H_b[16];
  real_T c6_b_impedances[23];
  real_T c6_b_currentState;
  real_T c6_b_constraints[2];
  real_T c6_b_qDes[23];
  real_T c6_b_CoMDes[3];
  int32_T c6_i47;
  int32_T c6_i48;
  int32_T c6_i49;
  int32_T c6_i50;
  int32_T c6_i51;
  int32_T c6_i52;
  int32_T c6_i53;
  int32_T c6_i54;
  int32_T c6_i55;
  int32_T c6_i56;
  boolean_T *c6_b_connection;
  real_T *c6_c_currentState;
  real_T (*c6_c_w_H_b)[16];
  real_T (*c6_c_CoMDes)[3];
  real_T (*c6_c_qDes)[23];
  real_T (*c6_c_constraints)[2];
  real_T (*c6_c_impedances)[23];
  real_T (*c6_c_r_sole_H_b)[16];
  real_T (*c6_c_l_sole_H_b)[16];
  real_T (*c6_c_constraintsIn)[2];
  real_T (*c6_c_qIn)[23];
  real_T (*c6_c_CoMIn)[3];
  real_T (*c6_c_w_CoM)[3];
  real_T (*c6_c_q0)[23];
  real_T (*c6_c_CoM_0)[3];
  real_T (*c6_c_wrench_leftFoot)[6];
  real_T (*c6_c_wrench_rightFoot)[6];
  c6_c_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c6_c_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 5);
  c6_c_r_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 10);
  c6_c_l_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 9);
  c6_c_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 4);
  c6_c_qDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c6_c_CoMDes = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_c_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_c_constraintsIn = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 8);
  c6_c_qIn = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 7);
  c6_c_CoMIn = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 6);
  c6_c_w_CoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c6_c_q0 = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 4);
  c6_c_CoM_0 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c6_c_wrench_leftFoot = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
  c6_c_wrench_rightFoot = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
    1);
  c6_b_connection = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c6_sfEvent);
  _SFD_DATA_RANGE_CHECK((real_T)*c6_b_connection, 0U);
  for (c6_i12 = 0; c6_i12 < 6; c6_i12++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_wrench_rightFoot)[c6_i12], 1U);
  }

  for (c6_i13 = 0; c6_i13 < 6; c6_i13++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_wrench_leftFoot)[c6_i13], 2U);
  }

  for (c6_i14 = 0; c6_i14 < 3; c6_i14++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_CoM_0)[c6_i14], 3U);
  }

  for (c6_i15 = 0; c6_i15 < 23; c6_i15++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_q0)[c6_i15], 4U);
  }

  for (c6_i16 = 0; c6_i16 < 3; c6_i16++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_w_CoM)[c6_i16], 5U);
  }

  for (c6_i17 = 0; c6_i17 < 3; c6_i17++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_CoMIn)[c6_i17], 6U);
  }

  for (c6_i18 = 0; c6_i18 < 23; c6_i18++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_qIn)[c6_i18], 7U);
  }

  for (c6_i19 = 0; c6_i19 < 2; c6_i19++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_constraintsIn)[c6_i19], 8U);
  }

  for (c6_i20 = 0; c6_i20 < 16; c6_i20++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_w_H_b)[c6_i20], 9U);
  }

  for (c6_i21 = 0; c6_i21 < 3; c6_i21++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_CoMDes)[c6_i21], 10U);
  }

  for (c6_i22 = 0; c6_i22 < 23; c6_i22++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_qDes)[c6_i22], 11U);
  }

  for (c6_i23 = 0; c6_i23 < 2; c6_i23++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_constraints)[c6_i23], 12U);
  }

  for (c6_i24 = 0; c6_i24 < 16; c6_i24++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_l_sole_H_b)[c6_i24], 13U);
  }

  for (c6_i25 = 0; c6_i25 < 16; c6_i25++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_r_sole_H_b)[c6_i25], 14U);
  }

  for (c6_i26 = 0; c6_i26 < 23; c6_i26++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_impedances)[c6_i26], 16U);
  }

  _SFD_DATA_RANGE_CHECK(*c6_c_currentState, 18U);
  chartInstance->c6_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c6_sfEvent);
  c6_hoistedGlobal = *c6_b_connection;
  c6_connection = c6_hoistedGlobal;
  for (c6_i27 = 0; c6_i27 < 6; c6_i27++) {
    c6_wrench_rightFoot[c6_i27] = (*c6_c_wrench_rightFoot)[c6_i27];
  }

  for (c6_i28 = 0; c6_i28 < 6; c6_i28++) {
    c6_wrench_leftFoot[c6_i28] = (*c6_c_wrench_leftFoot)[c6_i28];
  }

  for (c6_i29 = 0; c6_i29 < 3; c6_i29++) {
    c6_CoM_0[c6_i29] = (*c6_c_CoM_0)[c6_i29];
  }

  for (c6_i30 = 0; c6_i30 < 23; c6_i30++) {
    c6_q0[c6_i30] = (*c6_c_q0)[c6_i30];
  }

  for (c6_i31 = 0; c6_i31 < 3; c6_i31++) {
    c6_w_CoM[c6_i31] = (*c6_c_w_CoM)[c6_i31];
  }

  for (c6_i32 = 0; c6_i32 < 3; c6_i32++) {
    c6_CoMIn[c6_i32] = (*c6_c_CoMIn)[c6_i32];
  }

  for (c6_i33 = 0; c6_i33 < 23; c6_i33++) {
    c6_qIn[c6_i33] = (*c6_c_qIn)[c6_i33];
  }

  for (c6_i34 = 0; c6_i34 < 2; c6_i34++) {
    c6_constraintsIn[c6_i34] = (*c6_c_constraintsIn)[c6_i34];
  }

  for (c6_i35 = 0; c6_i35 < 16; c6_i35++) {
    c6_l_sole_H_b[c6_i35] = (*c6_c_l_sole_H_b)[c6_i35];
  }

  for (c6_i36 = 0; c6_i36 < 16; c6_i36++) {
    c6_r_sole_H_b[c6_i36] = (*c6_c_r_sole_H_b)[c6_i36];
  }

  c6_b_sm = chartInstance->c6_sm;
  c6_b_gain = chartInstance->c6_gain;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 21U, 21U, c6_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 0U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 1U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_connection, 2U, c6_k_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_wrench_rightFoot, 3U, c6_j_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_wrench_leftFoot, 4U, c6_j_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_CoM_0, 5U, c6_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_q0, 6U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_w_CoM, 7U, c6_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_CoMIn, 8U, c6_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_qIn, 9U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_constraintsIn, 10U, c6_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_l_sole_H_b, 11U, c6_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_r_sole_H_b, 12U, c6_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_b_sm, 13U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_b_gain, 14U, c6_g_sf_marshallOut,
    c6_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_w_H_b, 15U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_CoMDes, 16U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_qDes, 17U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_constraints, 18U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_impedances, 19U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_currentState, 20U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 5);
  for (c6_i37 = 0; c6_i37 < 3; c6_i37++) {
    c6_b_CoM_0[c6_i37] = c6_CoM_0[c6_i37];
  }

  for (c6_i38 = 0; c6_i38 < 23; c6_i38++) {
    c6_b_q0[c6_i38] = c6_q0[c6_i38];
  }

  for (c6_i39 = 0; c6_i39 < 3; c6_i39++) {
    c6_b_w_CoM[c6_i39] = c6_w_CoM[c6_i39];
  }

  for (c6_i40 = 0; c6_i40 < 3; c6_i40++) {
    c6_b_CoMIn[c6_i40] = c6_CoMIn[c6_i40];
  }

  for (c6_i41 = 0; c6_i41 < 23; c6_i41++) {
    c6_b_qIn[c6_i41] = c6_qIn[c6_i41];
  }

  for (c6_i42 = 0; c6_i42 < 2; c6_i42++) {
    c6_b_constraintsIn[c6_i42] = c6_constraintsIn[c6_i42];
  }

  for (c6_i43 = 0; c6_i43 < 6; c6_i43++) {
    c6_b_wrench_rightFoot[c6_i43] = c6_wrench_rightFoot[c6_i43];
  }

  for (c6_i44 = 0; c6_i44 < 6; c6_i44++) {
    c6_b_wrench_leftFoot[c6_i44] = c6_wrench_leftFoot[c6_i44];
  }

  for (c6_i45 = 0; c6_i45 < 16; c6_i45++) {
    c6_b_l_sole_H_b[c6_i45] = c6_l_sole_H_b[c6_i45];
  }

  for (c6_i46 = 0; c6_i46 < 16; c6_i46++) {
    c6_b_r_sole_H_b[c6_i46] = c6_r_sole_H_b[c6_i46];
  }

  c6_c_sm = c6_b_sm;
  c6_c_gain = c6_b_gain;
  c6_stateMachineWalking(chartInstance, c6_connection, c6_b_CoM_0, c6_b_q0,
    c6_b_w_CoM, c6_b_CoMIn, c6_b_qIn, c6_b_constraintsIn, c6_b_wrench_rightFoot,
    c6_b_wrench_leftFoot, c6_b_l_sole_H_b, c6_b_r_sole_H_b, &c6_c_sm, &c6_c_gain,
    c6_b_CoMDes, c6_b_qDes, c6_b_constraints, &c6_b_currentState,
    c6_b_impedances, c6_b_w_H_b);
  for (c6_i47 = 0; c6_i47 < 3; c6_i47++) {
    c6_CoMDes[c6_i47] = c6_b_CoMDes[c6_i47];
  }

  for (c6_i48 = 0; c6_i48 < 23; c6_i48++) {
    c6_qDes[c6_i48] = c6_b_qDes[c6_i48];
  }

  for (c6_i49 = 0; c6_i49 < 2; c6_i49++) {
    c6_constraints[c6_i49] = c6_b_constraints[c6_i49];
  }

  c6_currentState = c6_b_currentState;
  for (c6_i50 = 0; c6_i50 < 23; c6_i50++) {
    c6_impedances[c6_i50] = c6_b_impedances[c6_i50];
  }

  for (c6_i51 = 0; c6_i51 < 16; c6_i51++) {
    c6_w_H_b[c6_i51] = c6_b_w_H_b[c6_i51];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
  for (c6_i52 = 0; c6_i52 < 16; c6_i52++) {
    (*c6_c_w_H_b)[c6_i52] = c6_w_H_b[c6_i52];
  }

  for (c6_i53 = 0; c6_i53 < 3; c6_i53++) {
    (*c6_c_CoMDes)[c6_i53] = c6_CoMDes[c6_i53];
  }

  for (c6_i54 = 0; c6_i54 < 23; c6_i54++) {
    (*c6_c_qDes)[c6_i54] = c6_qDes[c6_i54];
  }

  for (c6_i55 = 0; c6_i55 < 2; c6_i55++) {
    (*c6_c_constraints)[c6_i55] = c6_constraints[c6_i55];
  }

  for (c6_i56 = 0; c6_i56 < 23; c6_i56++) {
    (*c6_c_impedances)[c6_i56] = c6_impedances[c6_i56];
  }

  *c6_c_currentState = c6_currentState;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c6_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc6_torqueBalancing2012b
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c6_stateMachineWalking(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, boolean_T c6_connection, real_T c6_CoM_0[3], real_T c6_q0[23],
  real_T c6_w_CoM[3], real_T c6_CoMIn[3], real_T c6_qIn[23], real_T
  c6_constraintsIn[2], real_T c6_wrench_rightFoot[6], real_T c6_wrench_leftFoot
  [6], real_T c6_l_sole_H_b[16], real_T c6_r_sole_H_b[16],
  c6_struct_rUGQ0INmvPpaxIctEGl5sE *c6_b_sm, c6_struct_kzTB0QQWoOlMoMhgKf6sK
  *c6_b_gain, real_T c6_CoMDes[3], real_T c6_qDes[23], real_T c6_constraints[2],
  real_T *c6_currentState, real_T c6_impedances[23], real_T c6_w_H_b[16])
{
  uint32_T c6_debug_family_var_map[25];
  real_T c6_CoMError[3];
  real_T c6_nargin = 13.0;
  real_T c6_nargout = 6.0;
  real_T c6_dv6[16];
  int32_T c6_i57;
  int32_T c6_i58;
  int32_T c6_i59;
  int32_T c6_i60;
  int32_T c6_i61;
  int32_T c6_i62;
  real_T c6_hoistedGlobal[16];
  int32_T c6_i63;
  real_T c6_b[16];
  int32_T c6_i64;
  int32_T c6_i65;
  int32_T c6_i66;
  real_T c6_C[16];
  int32_T c6_i67;
  int32_T c6_i68;
  int32_T c6_i69;
  int32_T c6_i70;
  int32_T c6_i71;
  int32_T c6_i72;
  int32_T c6_i73;
  int32_T c6_i74;
  int32_T c6_i75;
  int32_T c6_i76;
  int32_T c6_i77;
  int32_T c6_i78;
  int32_T c6_i79;
  int32_T c6_i80;
  int32_T c6_i81;
  int32_T c6_i82;
  int32_T c6_i83;
  int32_T c6_i84;
  int32_T c6_i85;
  int32_T c6_i86;
  int32_T c6_i87;
  int32_T c6_i88;
  int32_T c6_i89;
  real_T c6_x[3];
  real_T c6_y;
  real_T c6_scale;
  int32_T c6_k;
  int32_T c6_b_k;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_absxk;
  real_T c6_t;
  int32_T c6_i90;
  int32_T c6_i91;
  int32_T c6_i92;
  int32_T c6_i93;
  int32_T c6_i94;
  real_T c6_b_constraintsIn[2];
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_f_x;
  real_T c6_b_y;
  int32_T c6_i95;
  int32_T c6_i96;
  real_T c6_b_w_H_b[16];
  int32_T c6_i97;
  real_T c6_b_l_sole_H_b[16];
  real_T c6_dv7[16];
  int32_T c6_i98;
  int32_T c6_i99;
  real_T c6_c_constraintsIn[2];
  real_T c6_g_x;
  real_T c6_h_x;
  real_T c6_i_x;
  real_T c6_c_y;
  int32_T c6_i100;
  int32_T c6_i101;
  real_T c6_c_w_H_b[16];
  int32_T c6_i102;
  real_T c6_b_r_sole_H_b[16];
  real_T c6_dv8[16];
  int32_T c6_i103;
  int32_T c6_i104;
  int32_T c6_i105;
  int32_T c6_i106;
  int32_T c6_i107;
  int32_T c6_i108;
  int32_T c6_i109;
  int32_T c6_i110;
  int32_T c6_i111;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  boolean_T guard5 = FALSE;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 25U, 25U, c6_b_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_CoMError, 0U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 1U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 2U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_connection, 3U, c6_k_sf_marshallOut,
    c6_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_CoM_0, 4U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_q0, 5U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_w_CoM, 6U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_CoMIn, 7U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_qIn, 8U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_constraintsIn, 9U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_wrench_rightFoot, 10U,
    c6_j_sf_marshallOut, c6_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_wrench_leftFoot, 11U,
    c6_j_sf_marshallOut, c6_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_l_sole_H_b, 12U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_r_sole_H_b, 13U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_b_sm, 14U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_b_gain, 15U, c6_g_sf_marshallOut,
    c6_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_CoMDes, 16U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_qDes, 17U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_constraints, 18U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_currentState, 19U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_impedances, 20U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_w_H_b, 21U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c6_state, 22U,
    c6_n_sf_marshallOut, c6_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c6_fixedLink, 23U,
    c6_m_sf_marshallOut, c6_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c6_w_H_fixedLink, 24U,
    c6_l_sf_marshallOut, c6_i_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 5);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 6);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 7);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 9);
  guard4 = FALSE;
  guard5 = FALSE;
  if (CV_SCRIPT_COND(0, 0, !chartInstance->c6_state_not_empty)) {
    guard5 = TRUE;
  } else if (CV_SCRIPT_COND(0, 1, !chartInstance->c6_fixedLink_not_empty)) {
    guard5 = TRUE;
  } else if (CV_SCRIPT_COND(0, 2, !chartInstance->c6_w_H_fixedLink_not_empty)) {
    guard4 = TRUE;
  } else {
    CV_SCRIPT_MCDC(0, 0, FALSE);
    CV_SCRIPT_IF(0, 0, FALSE);
  }

  if (guard5 == TRUE) {
    guard4 = TRUE;
  }

  if (guard4 == TRUE) {
    CV_SCRIPT_MCDC(0, 0, TRUE);
    CV_SCRIPT_IF(0, 0, TRUE);
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 10);
    chartInstance->c6_state = c6_b_sm->stateAt0;
    chartInstance->c6_state_not_empty = TRUE;
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 11);
    chartInstance->c6_fixedLink = 1.0;
    chartInstance->c6_fixedLink_not_empty = TRUE;
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 12);
    c6_eye(chartInstance, c6_dv6);
    for (c6_i57 = 0; c6_i57 < 16; c6_i57++) {
      chartInstance->c6_w_H_fixedLink[c6_i57] = c6_dv6[c6_i57];
    }

    chartInstance->c6_w_H_fixedLink_not_empty = TRUE;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 19);
  for (c6_i58 = 0; c6_i58 < 3; c6_i58++) {
    c6_CoMDes[c6_i58] = c6_CoM_0[c6_i58];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 20);
  for (c6_i59 = 0; c6_i59 < 2; c6_i59++) {
    c6_constraints[c6_i59] = 1.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 21);
  for (c6_i60 = 0; c6_i60 < 23; c6_i60++) {
    c6_qDes[c6_i60] = c6_q0[c6_i60];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 22);
  for (c6_i61 = 0; c6_i61 < 23; c6_i61++) {
    c6_impedances[c6_i61] = c6_b_gain->impedances[c6_i61];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 30);
  if (CV_SCRIPT_IF(0, 1, chartInstance->c6_fixedLink == 1.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 31);
    for (c6_i62 = 0; c6_i62 < 16; c6_i62++) {
      c6_hoistedGlobal[c6_i62] = chartInstance->c6_w_H_fixedLink[c6_i62];
    }

    for (c6_i63 = 0; c6_i63 < 16; c6_i63++) {
      c6_b[c6_i63] = c6_l_sole_H_b[c6_i63];
    }

    c6_eml_scalar_eg(chartInstance);
    c6_eml_scalar_eg(chartInstance);
    for (c6_i64 = 0; c6_i64 < 16; c6_i64++) {
      c6_w_H_b[c6_i64] = 0.0;
    }

    for (c6_i65 = 0; c6_i65 < 16; c6_i65++) {
      c6_w_H_b[c6_i65] = 0.0;
    }

    for (c6_i66 = 0; c6_i66 < 16; c6_i66++) {
      c6_C[c6_i66] = c6_w_H_b[c6_i66];
    }

    for (c6_i67 = 0; c6_i67 < 16; c6_i67++) {
      c6_w_H_b[c6_i67] = c6_C[c6_i67];
    }

    for (c6_i68 = 0; c6_i68 < 16; c6_i68++) {
      c6_C[c6_i68] = c6_w_H_b[c6_i68];
    }

    for (c6_i69 = 0; c6_i69 < 16; c6_i69++) {
      c6_w_H_b[c6_i69] = c6_C[c6_i69];
    }

    for (c6_i70 = 0; c6_i70 < 4; c6_i70++) {
      c6_i71 = 0;
      for (c6_i72 = 0; c6_i72 < 4; c6_i72++) {
        c6_w_H_b[c6_i71 + c6_i70] = 0.0;
        c6_i73 = 0;
        for (c6_i74 = 0; c6_i74 < 4; c6_i74++) {
          c6_w_H_b[c6_i71 + c6_i70] += c6_hoistedGlobal[c6_i73 + c6_i70] *
            c6_b[c6_i74 + c6_i71];
          c6_i73 += 4;
        }

        c6_i71 += 4;
      }
    }
  } else {
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 33);
    for (c6_i75 = 0; c6_i75 < 16; c6_i75++) {
      c6_hoistedGlobal[c6_i75] = chartInstance->c6_w_H_fixedLink[c6_i75];
    }

    for (c6_i76 = 0; c6_i76 < 16; c6_i76++) {
      c6_b[c6_i76] = c6_r_sole_H_b[c6_i76];
    }

    c6_eml_scalar_eg(chartInstance);
    c6_eml_scalar_eg(chartInstance);
    for (c6_i77 = 0; c6_i77 < 16; c6_i77++) {
      c6_w_H_b[c6_i77] = 0.0;
    }

    for (c6_i78 = 0; c6_i78 < 16; c6_i78++) {
      c6_w_H_b[c6_i78] = 0.0;
    }

    for (c6_i79 = 0; c6_i79 < 16; c6_i79++) {
      c6_C[c6_i79] = c6_w_H_b[c6_i79];
    }

    for (c6_i80 = 0; c6_i80 < 16; c6_i80++) {
      c6_w_H_b[c6_i80] = c6_C[c6_i80];
    }

    for (c6_i81 = 0; c6_i81 < 16; c6_i81++) {
      c6_C[c6_i81] = c6_w_H_b[c6_i81];
    }

    for (c6_i82 = 0; c6_i82 < 16; c6_i82++) {
      c6_w_H_b[c6_i82] = c6_C[c6_i82];
    }

    for (c6_i83 = 0; c6_i83 < 4; c6_i83++) {
      c6_i84 = 0;
      for (c6_i85 = 0; c6_i85 < 4; c6_i85++) {
        c6_w_H_b[c6_i84 + c6_i83] = 0.0;
        c6_i86 = 0;
        for (c6_i87 = 0; c6_i87 < 4; c6_i87++) {
          c6_w_H_b[c6_i84 + c6_i83] += c6_hoistedGlobal[c6_i86 + c6_i83] *
            c6_b[c6_i87 + c6_i84];
          c6_i86 += 4;
        }

        c6_i84 += 4;
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 36);
  if (CV_SCRIPT_IF(0, 2, chartInstance->c6_state == 1.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 38);
    for (c6_i88 = 0; c6_i88 < 3; c6_i88++) {
      c6_CoMDes[c6_i88] = c6_CoM_0[c6_i88];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 40);
    c6_eml_eps(chartInstance);
    for (c6_i89 = 0; c6_i89 < 3; c6_i89++) {
      c6_x[c6_i89] = c6_CoM_0[c6_i89] - c6_CoMIn[c6_i89];
    }

    c6_y = 0.0;
    c6_realmin(chartInstance);
    c6_scale = 2.2250738585072014E-308;
    for (c6_k = 1; c6_k < 4; c6_k++) {
      c6_b_k = c6_k;
      c6_b_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_b_k), 1, 3, 1, 0) - 1];
      c6_c_x = c6_b_x;
      c6_absxk = muDoubleScalarAbs(c6_c_x);
      if (c6_absxk > c6_scale) {
        c6_t = c6_scale / c6_absxk;
        c6_y = 1.0 + c6_y * c6_t * c6_t;
        c6_scale = c6_absxk;
      } else {
        c6_t = c6_absxk / c6_scale;
        c6_y += c6_t * c6_t;
      }
    }

    c6_y = c6_scale * muDoubleScalarSqrt(c6_y);
    guard3 = FALSE;
    if (CV_SCRIPT_COND(0, 3, c6_y > 2.2204460492503131E-16)) {
      if (CV_SCRIPT_COND(0, 4, c6_connection)) {
        CV_SCRIPT_MCDC(0, 1, TRUE);
        CV_SCRIPT_IF(0, 3, TRUE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 41);
        chartInstance->c6_state = 2.0;
      } else {
        guard3 = TRUE;
      }
    } else {
      guard3 = TRUE;
    }

    if (guard3 == TRUE) {
      CV_SCRIPT_MCDC(0, 1, FALSE);
      CV_SCRIPT_IF(0, 3, FALSE);
    }
  } else {
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 44);
    if (CV_SCRIPT_IF(0, 4, chartInstance->c6_state == 2.0)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 46);
      for (c6_i90 = 0; c6_i90 < 3; c6_i90++) {
        c6_CoMDes[c6_i90] = c6_CoMIn[c6_i90];
      }

      _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 47);
      for (c6_i91 = 0; c6_i91 < 23; c6_i91++) {
        c6_qDes[c6_i91] = c6_qIn[c6_i91];
      }

      _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 48);
      (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
        _SFD_INTEGER_CHECK("state", chartInstance->c6_state), 1, 1, 1, 0);
      for (c6_i92 = 0; c6_i92 < 23; c6_i92++) {
        c6_impedances[c6_i92] = c6_b_gain->impedances[c6_i92];
      }

      _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 49);
      for (c6_i93 = 0; c6_i93 < 3; c6_i93++) {
        c6_CoMError[c6_i93] = c6_CoMDes[c6_i93] - c6_w_CoM[c6_i93];
      }

      _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 51);
      for (c6_i94 = 0; c6_i94 < 2; c6_i94++) {
        c6_b_constraintsIn[c6_i94] = c6_constraintsIn[c6_i94] - (1.0 - (real_T)
          c6_i94);
      }

      guard1 = FALSE;
      if (!CV_SCRIPT_COND(0, 5, c6_any(chartInstance, c6_b_constraintsIn))) {
        c6_d_x = c6_CoMError[1];
        c6_e_x = c6_d_x;
        c6_f_x = c6_e_x;
        c6_b_y = muDoubleScalarAbs(c6_f_x);
        if (CV_SCRIPT_COND(0, 6, c6_b_y < c6_b_sm->com.threshold)) {
          CV_SCRIPT_MCDC(0, 2, TRUE);
          CV_SCRIPT_IF(0, 5, TRUE);
          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 53);
          for (c6_i95 = 0; c6_i95 < 2; c6_i95++) {
            c6_constraints[c6_i95] = 1.0 - (real_T)c6_i95;
          }

          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 54);
          chartInstance->c6_fixedLink = 1.0;
          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 55);
          for (c6_i96 = 0; c6_i96 < 16; c6_i96++) {
            c6_b_w_H_b[c6_i96] = c6_w_H_b[c6_i96];
          }

          for (c6_i97 = 0; c6_i97 < 16; c6_i97++) {
            c6_b_l_sole_H_b[c6_i97] = c6_l_sole_H_b[c6_i97];
          }

          c6_mrdivide(chartInstance, c6_b_w_H_b, c6_b_l_sole_H_b, c6_dv7);
          for (c6_i98 = 0; c6_i98 < 16; c6_i98++) {
            chartInstance->c6_w_H_fixedLink[c6_i98] = c6_dv7[c6_i98];
          }

          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 56);
          chartInstance->c6_state = 3.0;
        } else {
          guard1 = TRUE;
        }
      } else {
        guard1 = TRUE;
      }

      if (guard1 == TRUE) {
        CV_SCRIPT_MCDC(0, 2, FALSE);
        CV_SCRIPT_IF(0, 5, FALSE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 58);
        for (c6_i99 = 0; c6_i99 < 2; c6_i99++) {
          c6_c_constraintsIn[c6_i99] = c6_constraintsIn[c6_i99] - (real_T)c6_i99;
        }

        guard2 = FALSE;
        if (!CV_SCRIPT_COND(0, 7, c6_any(chartInstance, c6_c_constraintsIn))) {
          c6_g_x = c6_CoMError[1];
          c6_h_x = c6_g_x;
          c6_i_x = c6_h_x;
          c6_c_y = muDoubleScalarAbs(c6_i_x);
          if (CV_SCRIPT_COND(0, 8, c6_c_y < c6_b_sm->com.threshold)) {
            CV_SCRIPT_MCDC(0, 3, TRUE);
            CV_SCRIPT_IF(0, 6, TRUE);
            _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 60);
            for (c6_i100 = 0; c6_i100 < 2; c6_i100++) {
              c6_constraints[c6_i100] = (real_T)c6_i100;
            }

            _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 61);
            chartInstance->c6_state = 4.0;
            _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 62);
            chartInstance->c6_fixedLink = 2.0;
            _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 63);
            for (c6_i101 = 0; c6_i101 < 16; c6_i101++) {
              c6_c_w_H_b[c6_i101] = c6_w_H_b[c6_i101];
            }

            for (c6_i102 = 0; c6_i102 < 16; c6_i102++) {
              c6_b_r_sole_H_b[c6_i102] = c6_r_sole_H_b[c6_i102];
            }

            c6_mrdivide(chartInstance, c6_c_w_H_b, c6_b_r_sole_H_b, c6_dv8);
            for (c6_i103 = 0; c6_i103 < 16; c6_i103++) {
              chartInstance->c6_w_H_fixedLink[c6_i103] = c6_dv8[c6_i103];
            }
          } else {
            guard2 = TRUE;
          }
        } else {
          guard2 = TRUE;
        }

        if (guard2 == TRUE) {
          CV_SCRIPT_MCDC(0, 3, FALSE);
          CV_SCRIPT_IF(0, 6, FALSE);
        }
      }
    } else {
      _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 67);
      if (CV_SCRIPT_IF(0, 7, chartInstance->c6_state == 3.0)) {
        _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 69);
        for (c6_i104 = 0; c6_i104 < 2; c6_i104++) {
          c6_constraints[c6_i104] = 1.0 - (real_T)c6_i104;
        }

        _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 71);
        for (c6_i105 = 0; c6_i105 < 3; c6_i105++) {
          c6_CoMDes[c6_i105] = c6_CoMIn[c6_i105];
        }

        _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 72);
        for (c6_i106 = 0; c6_i106 < 23; c6_i106++) {
          c6_qDes[c6_i106] = c6_qIn[c6_i106];
        }

        _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 73);
        (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
          _SFD_INTEGER_CHECK("state", chartInstance->c6_state), 1, 1, 1, 0);
        for (c6_i107 = 0; c6_i107 < 23; c6_i107++) {
          c6_impedances[c6_i107] = c6_b_gain->impedances[c6_i107];
        }

        _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 75);
        if (CV_SCRIPT_IF(0, 8, c6_wrench_rightFoot[2] >
                         c6_b_sm->wrench.thresholdContactOn)) {
          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 76);
          chartInstance->c6_state = 2.0;
        }
      } else {
        _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 79);
        if (CV_SCRIPT_IF(0, 9, chartInstance->c6_state == 4.0)) {
          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 81);
          for (c6_i108 = 0; c6_i108 < 2; c6_i108++) {
            c6_constraints[c6_i108] = (real_T)c6_i108;
          }

          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 83);
          for (c6_i109 = 0; c6_i109 < 3; c6_i109++) {
            c6_CoMDes[c6_i109] = c6_CoMIn[c6_i109];
          }

          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 84);
          for (c6_i110 = 0; c6_i110 < 23; c6_i110++) {
            c6_qDes[c6_i110] = c6_qIn[c6_i110];
          }

          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 85);
          (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
            _SFD_INTEGER_CHECK("state", chartInstance->c6_state), 1, 1, 1, 0);
          for (c6_i111 = 0; c6_i111 < 23; c6_i111++) {
            c6_impedances[c6_i111] = c6_b_gain->impedances[c6_i111];
          }

          _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 87);
          if (CV_SCRIPT_IF(0, 10, c6_wrench_leftFoot[2] >
                           c6_b_sm->wrench.thresholdContactOn)) {
            _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 88);
            chartInstance->c6_state = 2.0;
          }
        }
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 92);
  *c6_currentState = chartInstance->c6_state;
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, -92);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c6_chartNumber, 0U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m"));
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static real_T c6_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_currentState, const char_T *c6_identifier)
{
  real_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_currentState),
    &c6_thisId);
  sf_mex_destroy(&c6_currentState);
  return c6_y;
}

static real_T c6_b_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_currentState;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_currentState = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_currentState),
    &c6_thisId);
  sf_mex_destroy(&c6_currentState);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i112;
  real_T c6_b_inData[23];
  int32_T c6_i113;
  real_T c6_u[23];
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i112 = 0; c6_i112 < 23; c6_i112++) {
    c6_b_inData[c6_i112] = (*(real_T (*)[23])c6_inData)[c6_i112];
  }

  for (c6_i113 = 0; c6_i113 < 23; c6_i113++) {
    c6_u[c6_i113] = c6_b_inData[c6_i113];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 23), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_c_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_impedances, const char_T *c6_identifier,
  real_T c6_y[23])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_impedances), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_impedances);
}

static void c6_d_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[23])
{
  real_T c6_dv9[23];
  int32_T c6_i114;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv9, 1, 0, 0U, 1, 0U, 2, 1, 23);
  for (c6_i114 = 0; c6_i114 < 23; c6_i114++) {
    c6_y[c6_i114] = c6_dv9[c6_i114];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_impedances;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[23];
  int32_T c6_i115;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_impedances = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_impedances), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_impedances);
  for (c6_i115 = 0; c6_i115 < 23; c6_i115++) {
    (*(real_T (*)[23])c6_outData)[c6_i115] = c6_y[c6_i115];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i116;
  real_T c6_b_inData[2];
  int32_T c6_i117;
  real_T c6_u[2];
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i116 = 0; c6_i116 < 2; c6_i116++) {
    c6_b_inData[c6_i116] = (*(real_T (*)[2])c6_inData)[c6_i116];
  }

  for (c6_i117 = 0; c6_i117 < 2; c6_i117++) {
    c6_u[c6_i117] = c6_b_inData[c6_i117];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_e_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_constraints, const char_T *c6_identifier,
  real_T c6_y[2])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_constraints), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_constraints);
}

static void c6_f_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[2])
{
  real_T c6_dv10[2];
  int32_T c6_i118;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv10, 1, 0, 0U, 1, 0U, 1, 2);
  for (c6_i118 = 0; c6_i118 < 2; c6_i118++) {
    c6_y[c6_i118] = c6_dv10[c6_i118];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_constraints;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[2];
  int32_T c6_i119;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_constraints = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_constraints), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_constraints);
  for (c6_i119 = 0; c6_i119 < 2; c6_i119++) {
    (*(real_T (*)[2])c6_outData)[c6_i119] = c6_y[c6_i119];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i120;
  real_T c6_b_inData[23];
  int32_T c6_i121;
  real_T c6_u[23];
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i120 = 0; c6_i120 < 23; c6_i120++) {
    c6_b_inData[c6_i120] = (*(real_T (*)[23])c6_inData)[c6_i120];
  }

  for (c6_i121 = 0; c6_i121 < 23; c6_i121++) {
    c6_u[c6_i121] = c6_b_inData[c6_i121];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_g_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_qDes, const char_T *c6_identifier, real_T
  c6_y[23])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_qDes), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_qDes);
}

static void c6_h_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[23])
{
  real_T c6_dv11[23];
  int32_T c6_i122;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv11, 1, 0, 0U, 1, 0U, 1, 23);
  for (c6_i122 = 0; c6_i122 < 23; c6_i122++) {
    c6_y[c6_i122] = c6_dv11[c6_i122];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_qDes;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[23];
  int32_T c6_i123;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_qDes = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_qDes), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_qDes);
  for (c6_i123 = 0; c6_i123 < 23; c6_i123++) {
    (*(real_T (*)[23])c6_outData)[c6_i123] = c6_y[c6_i123];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i124;
  real_T c6_b_inData[3];
  int32_T c6_i125;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i124 = 0; c6_i124 < 3; c6_i124++) {
    c6_b_inData[c6_i124] = (*(real_T (*)[3])c6_inData)[c6_i124];
  }

  for (c6_i125 = 0; c6_i125 < 3; c6_i125++) {
    c6_u[c6_i125] = c6_b_inData[c6_i125];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_i_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_CoMDes, const char_T *c6_identifier, real_T
  c6_y[3])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_CoMDes), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_CoMDes);
}

static void c6_j_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3])
{
  real_T c6_dv12[3];
  int32_T c6_i126;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv12, 1, 0, 0U, 1, 0U, 1, 3);
  for (c6_i126 = 0; c6_i126 < 3; c6_i126++) {
    c6_y[c6_i126] = c6_dv12[c6_i126];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_CoMDes;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i127;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_CoMDes = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_CoMDes), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_CoMDes);
  for (c6_i127 = 0; c6_i127 < 3; c6_i127++) {
    (*(real_T (*)[3])c6_outData)[c6_i127] = c6_y[c6_i127];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i128;
  int32_T c6_i129;
  int32_T c6_i130;
  real_T c6_b_inData[16];
  int32_T c6_i131;
  int32_T c6_i132;
  int32_T c6_i133;
  real_T c6_u[16];
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i128 = 0;
  for (c6_i129 = 0; c6_i129 < 4; c6_i129++) {
    for (c6_i130 = 0; c6_i130 < 4; c6_i130++) {
      c6_b_inData[c6_i130 + c6_i128] = (*(real_T (*)[16])c6_inData)[c6_i130 +
        c6_i128];
    }

    c6_i128 += 4;
  }

  c6_i131 = 0;
  for (c6_i132 = 0; c6_i132 < 4; c6_i132++) {
    for (c6_i133 = 0; c6_i133 < 4; c6_i133++) {
      c6_u[c6_i133 + c6_i131] = c6_b_inData[c6_i133 + c6_i131];
    }

    c6_i131 += 4;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_k_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_w_H_b, const char_T *c6_identifier, real_T
  c6_y[16])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_w_H_b), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_w_H_b);
}

static void c6_l_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[16])
{
  real_T c6_dv13[16];
  int32_T c6_i134;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv13, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c6_i134 = 0; c6_i134 < 16; c6_i134++) {
    c6_y[c6_i134] = c6_dv13[c6_i134];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_w_H_b;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[16];
  int32_T c6_i135;
  int32_T c6_i136;
  int32_T c6_i137;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_w_H_b = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_w_H_b), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_w_H_b);
  c6_i135 = 0;
  for (c6_i136 = 0; c6_i136 < 4; c6_i136++) {
    for (c6_i137 = 0; c6_i137 < 4; c6_i137++) {
      (*(real_T (*)[16])c6_outData)[c6_i137 + c6_i135] = c6_y[c6_i137 + c6_i135];
    }

    c6_i135 += 4;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  c6_struct_kzTB0QQWoOlMoMhgKf6sK c6_u;
  const mxArray *c6_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_c_y = NULL;
  real_T c6_d_u;
  const mxArray *c6_d_y = NULL;
  int32_T c6_i138;
  real_T c6_e_u[9];
  const mxArray *c6_e_y = NULL;
  int32_T c6_i139;
  real_T c6_f_u[9];
  const mxArray *c6_f_y = NULL;
  int32_T c6_i140;
  real_T c6_g_u[9];
  const mxArray *c6_g_y = NULL;
  real_T c6_h_u;
  const mxArray *c6_h_y = NULL;
  real_T c6_i_u;
  const mxArray *c6_i_y = NULL;
  int32_T c6_i141;
  real_T c6_j_u[23];
  const mxArray *c6_j_y = NULL;
  int32_T c6_i142;
  real_T c6_k_u[23];
  const mxArray *c6_k_y = NULL;
  int32_T c6_i143;
  real_T c6_l_u[23];
  const mxArray *c6_l_y = NULL;
  int32_T c6_i144;
  real_T c6_m_u[23];
  const mxArray *c6_m_y = NULL;
  int32_T c6_i145;
  real_T c6_n_u[4];
  const mxArray *c6_n_y = NULL;
  int32_T c6_i146;
  real_T c6_o_u[4];
  const mxArray *c6_o_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(c6_struct_kzTB0QQWoOlMoMhgKf6sK *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c6_b_u = c6_u.qTildeMax;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_b_y, "qTildeMax", "qTildeMax", 0);
  c6_c_u = c6_u.SmoothingTimeImp;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_c_y, "SmoothingTimeImp", "SmoothingTimeImp", 0);
  c6_d_u = c6_u.SmoothingTimeGainScheduling;
  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_d_y, "SmoothingTimeGainScheduling",
                  "SmoothingTimeGainScheduling", 0);
  for (c6_i138 = 0; c6_i138 < 9; c6_i138++) {
    c6_e_u[c6_i138] = c6_u.PCOM[c6_i138];
  }

  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", c6_e_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c6_y, c6_e_y, "PCOM", "PCOM", 0);
  for (c6_i139 = 0; c6_i139 < 9; c6_i139++) {
    c6_f_u[c6_i139] = c6_u.ICOM[c6_i139];
  }

  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_create("y", c6_f_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c6_y, c6_f_y, "ICOM", "ICOM", 0);
  for (c6_i140 = 0; c6_i140 < 9; c6_i140++) {
    c6_g_u[c6_i140] = c6_u.DCOM[c6_i140];
  }

  c6_g_y = NULL;
  sf_mex_assign(&c6_g_y, sf_mex_create("y", c6_g_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c6_y, c6_g_y, "DCOM", "DCOM", 0);
  c6_h_u = c6_u.PAngularMomentum;
  c6_h_y = NULL;
  sf_mex_assign(&c6_h_y, sf_mex_create("y", &c6_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_h_y, "PAngularMomentum", "PAngularMomentum", 0);
  c6_i_u = c6_u.DAngularMomentum;
  c6_i_y = NULL;
  sf_mex_assign(&c6_i_y, sf_mex_create("y", &c6_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_i_y, "DAngularMomentum", "DAngularMomentum", 0);
  for (c6_i141 = 0; c6_i141 < 23; c6_i141++) {
    c6_j_u[c6_i141] = c6_u.integral[c6_i141];
  }

  c6_j_y = NULL;
  sf_mex_assign(&c6_j_y, sf_mex_create("y", c6_j_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c6_y, c6_j_y, "integral", "integral", 0);
  for (c6_i142 = 0; c6_i142 < 23; c6_i142++) {
    c6_k_u[c6_i142] = c6_u.impedances[c6_i142];
  }

  c6_k_y = NULL;
  sf_mex_assign(&c6_k_y, sf_mex_create("y", c6_k_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c6_y, c6_k_y, "impedances", "impedances", 0);
  for (c6_i143 = 0; c6_i143 < 23; c6_i143++) {
    c6_l_u[c6_i143] = c6_u.dampings[c6_i143];
  }

  c6_l_y = NULL;
  sf_mex_assign(&c6_l_y, sf_mex_create("y", c6_l_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c6_y, c6_l_y, "dampings", "dampings", 0);
  for (c6_i144 = 0; c6_i144 < 23; c6_i144++) {
    c6_m_u[c6_i144] = c6_u.increasingRatesImp[c6_i144];
  }

  c6_m_y = NULL;
  sf_mex_assign(&c6_m_y, sf_mex_create("y", c6_m_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c6_y, c6_m_y, "increasingRatesImp", "increasingRatesImp", 0);
  for (c6_i145 = 0; c6_i145 < 4; c6_i145++) {
    c6_n_u[c6_i145] = c6_u.footSize[c6_i145];
  }

  c6_n_y = NULL;
  sf_mex_assign(&c6_n_y, sf_mex_create("y", c6_n_u, 0, 0U, 1U, 0U, 2, 2, 2),
                FALSE);
  sf_mex_addfield(c6_y, c6_n_y, "footSize", "footSize", 0);
  for (c6_i146 = 0; c6_i146 < 4; c6_i146++) {
    c6_o_u[c6_i146] = c6_u.legSize[c6_i146];
  }

  c6_o_y = NULL;
  sf_mex_assign(&c6_o_y, sf_mex_create("y", c6_o_u, 0, 0U, 1U, 0U, 2, 2, 2),
                FALSE);
  sf_mex_addfield(c6_y, c6_o_y, "legSize", "legSize", 0);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_m_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_kzTB0QQWoOlMoMhgKf6sK *c6_y)
{
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[14] = { "qTildeMax", "SmoothingTimeImp",
    "SmoothingTimeGainScheduling", "PCOM", "ICOM", "DCOM", "PAngularMomentum",
    "DAngularMomentum", "integral", "impedances", "dampings",
    "increasingRatesImp", "footSize", "legSize" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 14, c6_fieldNames, 0U, 0);
  c6_thisId.fIdentifier = "qTildeMax";
  c6_y->qTildeMax = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "qTildeMax", "qTildeMax", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "SmoothingTimeImp";
  c6_y->SmoothingTimeImp = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "SmoothingTimeImp", "SmoothingTimeImp", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "SmoothingTimeGainScheduling";
  c6_y->SmoothingTimeGainScheduling = c6_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c6_u, "SmoothingTimeGainScheduling",
    "SmoothingTimeGainScheduling", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "PCOM";
  c6_n_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "PCOM",
    "PCOM", 0)), &c6_thisId, c6_y->PCOM);
  c6_thisId.fIdentifier = "ICOM";
  c6_n_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "ICOM",
    "ICOM", 0)), &c6_thisId, c6_y->ICOM);
  c6_thisId.fIdentifier = "DCOM";
  c6_n_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "DCOM",
    "DCOM", 0)), &c6_thisId, c6_y->DCOM);
  c6_thisId.fIdentifier = "PAngularMomentum";
  c6_y->PAngularMomentum = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "PAngularMomentum", "PAngularMomentum", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "DAngularMomentum";
  c6_y->DAngularMomentum = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "DAngularMomentum", "DAngularMomentum", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "integral";
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "integral", "integral", 0)), &c6_thisId, c6_y->integral);
  c6_thisId.fIdentifier = "impedances";
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "impedances", "impedances", 0)), &c6_thisId, c6_y->impedances);
  c6_thisId.fIdentifier = "dampings";
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "dampings", "dampings", 0)), &c6_thisId, c6_y->dampings);
  c6_thisId.fIdentifier = "increasingRatesImp";
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "increasingRatesImp", "increasingRatesImp", 0)), &c6_thisId,
                        c6_y->increasingRatesImp);
  c6_thisId.fIdentifier = "footSize";
  c6_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "footSize", "footSize", 0)), &c6_thisId, c6_y->footSize);
  c6_thisId.fIdentifier = "legSize";
  c6_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "legSize", "legSize", 0)), &c6_thisId, c6_y->legSize);
  sf_mex_destroy(&c6_u);
}

static void c6_n_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[9])
{
  real_T c6_dv14[9];
  int32_T c6_i147;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv14, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c6_i147 = 0; c6_i147 < 9; c6_i147++) {
    c6_y[c6_i147] = c6_dv14[c6_i147];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_o_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4])
{
  real_T c6_dv15[4];
  int32_T c6_i148;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv15, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c6_i148 = 0; c6_i148 < 4; c6_i148++) {
    c6_y[c6_i148] = c6_dv15[c6_i148];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_gain;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  c6_struct_kzTB0QQWoOlMoMhgKf6sK c6_y;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_b_gain = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_gain), &c6_thisId, &c6_y);
  sf_mex_destroy(&c6_b_gain);
  *(c6_struct_kzTB0QQWoOlMoMhgKf6sK *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData;
  c6_struct_rUGQ0INmvPpaxIctEGl5sE c6_u;
  const mxArray *c6_y = NULL;
  boolean_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  boolean_T c6_c_u;
  const mxArray *c6_c_y = NULL;
  boolean_T c6_d_u;
  const mxArray *c6_d_y = NULL;
  boolean_T c6_e_u;
  const mxArray *c6_e_y = NULL;
  c6_struct_DnBdbfPNxiIjhNOyZMmfsE c6_f_u;
  const mxArray *c6_f_y = NULL;
  real_T c6_g_u;
  const mxArray *c6_g_y = NULL;
  int32_T c6_i149;
  real_T c6_h_u[39];
  const mxArray *c6_h_y = NULL;
  c6_struct_KJR2itYvhBuAkuR6dKZHUC c6_i_u;
  const mxArray *c6_i_y = NULL;
  real_T c6_j_u;
  const mxArray *c6_j_y = NULL;
  real_T c6_k_u;
  const mxArray *c6_k_y = NULL;
  c6_struct_0U0wBk2LiR1OqsMsUngxdD c6_l_u;
  const mxArray *c6_l_y = NULL;
  real_T c6_m_u;
  const mxArray *c6_m_y = NULL;
  real_T c6_n_u;
  const mxArray *c6_n_y = NULL;
  real_T c6_o_u;
  const mxArray *c6_o_y = NULL;
  real_T c6_p_u;
  const mxArray *c6_p_y = NULL;
  int32_T c6_i150;
  real_T c6_q_u[299];
  const mxArray *c6_q_y = NULL;
  int32_T c6_i151;
  real_T c6_r_u[192];
  const mxArray *c6_r_y = NULL;
  int32_T c6_i152;
  real_T c6_s_u[192];
  const mxArray *c6_s_y = NULL;
  int32_T c6_i153;
  real_T c6_t_u[72];
  const mxArray *c6_t_y = NULL;
  real_T c6_u_u;
  const mxArray *c6_u_y = NULL;
  real_T c6_v_u;
  const mxArray *c6_v_y = NULL;
  real_T c6_w_u;
  const mxArray *c6_w_y = NULL;
  int32_T c6_i154;
  real_T c6_x_u[13];
  const mxArray *c6_x_y = NULL;
  real_T c6_y_u;
  const mxArray *c6_y_y = NULL;
  boolean_T c6_ab_u;
  const mxArray *c6_ab_y = NULL;
  int32_T c6_i155;
  real_T c6_bb_u[8];
  const mxArray *c6_bb_y = NULL;
  c6_struct_9LpOi5JXaV67jTuay8hWaH c6_cb_u;
  const mxArray *c6_cb_y = NULL;
  int32_T c6_i156;
  real_T c6_db_u[24];
  const mxArray *c6_db_y = NULL;
  int32_T c6_i157;
  real_T c6_eb_u[8];
  const mxArray *c6_eb_y = NULL;
  int32_T c6_i158;
  real_T c6_fb_u[8];
  const mxArray *c6_fb_y = NULL;
  int32_T c6_i159;
  real_T c6_gb_u[8];
  const mxArray *c6_gb_y = NULL;
  int32_T c6_i160;
  real_T c6_hb_u[8];
  const mxArray *c6_hb_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_mxArrayOutData = NULL;
  c6_u = *(c6_struct_rUGQ0INmvPpaxIctEGl5sE *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c6_b_u = c6_u.skipYoga;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_b_y, "skipYoga", "skipYoga", 0);
  c6_c_u = c6_u.demoOnlyRightFoot;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_c_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_c_y, "demoOnlyRightFoot", "demoOnlyRightFoot", 0);
  c6_d_u = c6_u.yogaAlsoOnRightFoot;
  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_d_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_d_y, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot", 0);
  c6_e_u = c6_u.yogaInLoop;
  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_e_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_e_y, "yogaInLoop", "yogaInLoop", 0);
  c6_f_u = c6_u.com;
  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c6_g_u = c6_f_u.threshold;
  c6_g_y = NULL;
  sf_mex_assign(&c6_g_y, sf_mex_create("y", &c6_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_f_y, c6_g_y, "threshold", "threshold", 0);
  for (c6_i149 = 0; c6_i149 < 39; c6_i149++) {
    c6_h_u[c6_i149] = c6_f_u.states[c6_i149];
  }

  c6_h_y = NULL;
  sf_mex_assign(&c6_h_y, sf_mex_create("y", c6_h_u, 0, 0U, 1U, 0U, 2, 13, 3),
                FALSE);
  sf_mex_addfield(c6_f_y, c6_h_y, "states", "states", 0);
  sf_mex_addfield(c6_y, c6_f_y, "com", "com", 0);
  c6_i_u = c6_u.wrench;
  c6_i_y = NULL;
  sf_mex_assign(&c6_i_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c6_j_u = c6_i_u.thresholdContactOn;
  c6_j_y = NULL;
  sf_mex_assign(&c6_j_y, sf_mex_create("y", &c6_j_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_i_y, c6_j_y, "thresholdContactOn", "thresholdContactOn", 0);
  c6_k_u = c6_i_u.thresholdContactOff;
  c6_k_y = NULL;
  sf_mex_assign(&c6_k_y, sf_mex_create("y", &c6_k_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_i_y, c6_k_y, "thresholdContactOff", "thresholdContactOff",
                  0);
  sf_mex_addfield(c6_y, c6_i_y, "wrench", "wrench", 0);
  c6_l_u = c6_u.joints;
  c6_l_y = NULL;
  sf_mex_assign(&c6_l_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c6_m_u = c6_l_u.thresholdNotInContact;
  c6_m_y = NULL;
  sf_mex_assign(&c6_m_y, sf_mex_create("y", &c6_m_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_l_y, c6_m_y, "thresholdNotInContact",
                  "thresholdNotInContact", 0);
  c6_n_u = c6_l_u.thresholdInContact;
  c6_n_y = NULL;
  sf_mex_assign(&c6_n_y, sf_mex_create("y", &c6_n_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_l_y, c6_n_y, "thresholdInContact", "thresholdInContact", 0);
  c6_o_u = c6_l_u.pauseTimeLastPostureL;
  c6_o_y = NULL;
  sf_mex_assign(&c6_o_y, sf_mex_create("y", &c6_o_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_l_y, c6_o_y, "pauseTimeLastPostureL",
                  "pauseTimeLastPostureL", 0);
  c6_p_u = c6_l_u.pauseTimeLastPostureR;
  c6_p_y = NULL;
  sf_mex_assign(&c6_p_y, sf_mex_create("y", &c6_p_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_l_y, c6_p_y, "pauseTimeLastPostureR",
                  "pauseTimeLastPostureR", 0);
  for (c6_i150 = 0; c6_i150 < 299; c6_i150++) {
    c6_q_u[c6_i150] = c6_l_u.states[c6_i150];
  }

  c6_q_y = NULL;
  sf_mex_assign(&c6_q_y, sf_mex_create("y", c6_q_u, 0, 0U, 1U, 0U, 2, 13, 23),
                FALSE);
  sf_mex_addfield(c6_l_y, c6_q_y, "states", "states", 0);
  for (c6_i151 = 0; c6_i151 < 192; c6_i151++) {
    c6_r_u[c6_i151] = c6_l_u.pointsL[c6_i151];
  }

  c6_r_y = NULL;
  sf_mex_assign(&c6_r_y, sf_mex_create("y", c6_r_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c6_l_y, c6_r_y, "pointsL", "pointsL", 0);
  for (c6_i152 = 0; c6_i152 < 192; c6_i152++) {
    c6_s_u[c6_i152] = c6_l_u.pointsR[c6_i152];
  }

  c6_s_y = NULL;
  sf_mex_assign(&c6_s_y, sf_mex_create("y", c6_s_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c6_l_y, c6_s_y, "pointsR", "pointsR", 0);
  for (c6_i153 = 0; c6_i153 < 72; c6_i153++) {
    c6_t_u[c6_i153] = c6_l_u.standUpPositions[c6_i153];
  }

  c6_t_y = NULL;
  sf_mex_assign(&c6_t_y, sf_mex_create("y", c6_t_u, 0, 0U, 1U, 0U, 2, 8, 9),
                FALSE);
  sf_mex_addfield(c6_l_y, c6_t_y, "standUpPositions", "standUpPositions", 0);
  sf_mex_addfield(c6_y, c6_l_y, "joints", "joints", 0);
  c6_u_u = c6_u.stateAt0;
  c6_u_y = NULL;
  sf_mex_assign(&c6_u_y, sf_mex_create("y", &c6_u_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_u_y, "stateAt0", "stateAt0", 0);
  c6_v_u = c6_u.DT;
  c6_v_y = NULL;
  sf_mex_assign(&c6_v_y, sf_mex_create("y", &c6_v_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_v_y, "DT", "DT", 0);
  c6_w_u = c6_u.waitingTimeAfterYoga;
  c6_w_y = NULL;
  sf_mex_assign(&c6_w_y, sf_mex_create("y", &c6_w_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_w_y, "waitingTimeAfterYoga", "waitingTimeAfterYoga",
                  0);
  for (c6_i154 = 0; c6_i154 < 13; c6_i154++) {
    c6_x_u[c6_i154] = c6_u.jointsSmoothingTimes[c6_i154];
  }

  c6_x_y = NULL;
  sf_mex_assign(&c6_x_y, sf_mex_create("y", c6_x_u, 0, 0U, 1U, 0U, 2, 13, 1),
                FALSE);
  sf_mex_addfield(c6_y, c6_x_y, "jointsSmoothingTimes", "jointsSmoothingTimes",
                  0);
  c6_y_u = c6_u.tBalancing;
  c6_y_y = NULL;
  sf_mex_assign(&c6_y_y, sf_mex_create("y", &c6_y_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_y_y, "tBalancing", "tBalancing", 0);
  c6_ab_u = c6_u.alsoSitDown;
  c6_ab_y = NULL;
  sf_mex_assign(&c6_ab_y, sf_mex_create("y", &c6_ab_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c6_y, c6_ab_y, "alsoSitDown", "alsoSitDown", 0);
  for (c6_i155 = 0; c6_i155 < 8; c6_i155++) {
    c6_bb_u[c6_i155] = c6_u.jointsAndCoMSmoothingTimes[c6_i155];
  }

  c6_bb_y = NULL;
  sf_mex_assign(&c6_bb_y, sf_mex_create("y", c6_bb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c6_y, c6_bb_y, "jointsAndCoMSmoothingTimes",
                  "jointsAndCoMSmoothingTimes", 0);
  c6_cb_u = c6_u.CoM;
  c6_cb_y = NULL;
  sf_mex_assign(&c6_cb_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  for (c6_i156 = 0; c6_i156 < 24; c6_i156++) {
    c6_db_u[c6_i156] = c6_cb_u.standUpDeltaCoM[c6_i156];
  }

  c6_db_y = NULL;
  sf_mex_assign(&c6_db_y, sf_mex_create("y", c6_db_u, 0, 0U, 1U, 0U, 2, 8, 3),
                FALSE);
  sf_mex_addfield(c6_cb_y, c6_db_y, "standUpDeltaCoM", "standUpDeltaCoM", 0);
  sf_mex_addfield(c6_y, c6_cb_y, "CoM", "CoM", 0);
  for (c6_i157 = 0; c6_i157 < 8; c6_i157++) {
    c6_eb_u[c6_i157] = c6_u.LwrenchThreshold[c6_i157];
  }

  c6_eb_y = NULL;
  sf_mex_assign(&c6_eb_y, sf_mex_create("y", c6_eb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c6_y, c6_eb_y, "LwrenchThreshold", "LwrenchThreshold", 0);
  for (c6_i158 = 0; c6_i158 < 8; c6_i158++) {
    c6_fb_u[c6_i158] = c6_u.RwrenchThreshold[c6_i158];
  }

  c6_fb_y = NULL;
  sf_mex_assign(&c6_fb_y, sf_mex_create("y", c6_fb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c6_y, c6_fb_y, "RwrenchThreshold", "RwrenchThreshold", 0);
  for (c6_i159 = 0; c6_i159 < 8; c6_i159++) {
    c6_gb_u[c6_i159] = c6_u.RArmThreshold[c6_i159];
  }

  c6_gb_y = NULL;
  sf_mex_assign(&c6_gb_y, sf_mex_create("y", c6_gb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c6_y, c6_gb_y, "RArmThreshold", "RArmThreshold", 0);
  for (c6_i160 = 0; c6_i160 < 8; c6_i160++) {
    c6_hb_u[c6_i160] = c6_u.LArmThreshold[c6_i160];
  }

  c6_hb_y = NULL;
  sf_mex_assign(&c6_hb_y, sf_mex_create("y", c6_hb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c6_y, c6_hb_y, "LArmThreshold", "LArmThreshold", 0);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_p_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_rUGQ0INmvPpaxIctEGl5sE *c6_y)
{
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[19] = { "skipYoga", "demoOnlyRightFoot",
    "yogaAlsoOnRightFoot", "yogaInLoop", "com", "wrench", "joints", "stateAt0",
    "DT", "waitingTimeAfterYoga", "jointsSmoothingTimes", "tBalancing",
    "alsoSitDown", "jointsAndCoMSmoothingTimes", "CoM", "LwrenchThreshold",
    "RwrenchThreshold", "RArmThreshold", "LArmThreshold" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 19, c6_fieldNames, 0U, 0);
  c6_thisId.fIdentifier = "skipYoga";
  c6_y->skipYoga = c6_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "skipYoga", "skipYoga", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "demoOnlyRightFoot";
  c6_y->demoOnlyRightFoot = c6_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "demoOnlyRightFoot", "demoOnlyRightFoot", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "yogaAlsoOnRightFoot";
  c6_y->yogaAlsoOnRightFoot = c6_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "yogaInLoop";
  c6_y->yogaInLoop = c6_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "yogaInLoop", "yogaInLoop", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "com";
  c6_r_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "com",
    "com", 0)), &c6_thisId, &c6_y->com);
  c6_thisId.fIdentifier = "wrench";
  c6_y->wrench = c6_t_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
                                        (c6_u, "wrench", "wrench", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "joints";
  c6_u_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "joints",
    "joints", 0)), &c6_thisId, &c6_y->joints);
  c6_thisId.fIdentifier = "stateAt0";
  c6_y->stateAt0 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "stateAt0", "stateAt0", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "DT";
  c6_y->DT = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "DT", "DT", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "waitingTimeAfterYoga";
  c6_y->waitingTimeAfterYoga = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "waitingTimeAfterYoga", "waitingTimeAfterYoga", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "jointsSmoothingTimes";
  c6_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "jointsSmoothingTimes", "jointsSmoothingTimes", 0)), &c6_thisId,
                        c6_y->jointsSmoothingTimes);
  c6_thisId.fIdentifier = "tBalancing";
  c6_y->tBalancing = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "tBalancing", "tBalancing", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "alsoSitDown";
  c6_y->alsoSitDown = c6_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "alsoSitDown", "alsoSitDown", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "jointsAndCoMSmoothingTimes";
  c6_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "jointsAndCoMSmoothingTimes", "jointsAndCoMSmoothingTimes", 0)), &c6_thisId,
    c6_y->jointsAndCoMSmoothingTimes);
  c6_thisId.fIdentifier = "CoM";
  c6_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "CoM",
    "CoM", 0)), &c6_thisId, &c6_y->CoM);
  c6_thisId.fIdentifier = "LwrenchThreshold";
  c6_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "LwrenchThreshold", "LwrenchThreshold", 0)), &c6_thisId,
    c6_y->LwrenchThreshold);
  c6_thisId.fIdentifier = "RwrenchThreshold";
  c6_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "RwrenchThreshold", "RwrenchThreshold", 0)), &c6_thisId,
    c6_y->RwrenchThreshold);
  c6_thisId.fIdentifier = "RArmThreshold";
  c6_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "RArmThreshold", "RArmThreshold", 0)), &c6_thisId, c6_y->RArmThreshold);
  c6_thisId.fIdentifier = "LArmThreshold";
  c6_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "LArmThreshold", "LArmThreshold", 0)), &c6_thisId, c6_y->LArmThreshold);
  sf_mex_destroy(&c6_u);
}

static boolean_T c6_q_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  boolean_T c6_y;
  boolean_T c6_b0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_b0, 1, 11, 0U, 0, 0U, 0);
  c6_y = c6_b0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_r_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_DnBdbfPNxiIjhNOyZMmfsE *c6_y)
{
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[2] = { "threshold", "states" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 2, c6_fieldNames, 0U, 0);
  c6_thisId.fIdentifier = "threshold";
  c6_y->threshold = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "threshold", "threshold", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "states";
  c6_s_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "states",
    "states", 0)), &c6_thisId, c6_y->states);
  sf_mex_destroy(&c6_u);
}

static void c6_s_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[39])
{
  real_T c6_dv16[39];
  int32_T c6_i161;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv16, 1, 0, 0U, 1, 0U, 2, 13,
                3);
  for (c6_i161 = 0; c6_i161 < 39; c6_i161++) {
    c6_y[c6_i161] = c6_dv16[c6_i161];
  }

  sf_mex_destroy(&c6_u);
}

static c6_struct_KJR2itYvhBuAkuR6dKZHUC c6_t_emlrt_marshallIn
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c6_u,
   const emlrtMsgIdentifier *c6_parentId)
{
  c6_struct_KJR2itYvhBuAkuR6dKZHUC c6_y;
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[2] = { "thresholdContactOn",
    "thresholdContactOff" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 2, c6_fieldNames, 0U, 0);
  c6_thisId.fIdentifier = "thresholdContactOn";
  c6_y.thresholdContactOn = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "thresholdContactOn", "thresholdContactOn", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "thresholdContactOff";
  c6_y.thresholdContactOff = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "thresholdContactOff", "thresholdContactOff", 0)),
    &c6_thisId);
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_u_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_0U0wBk2LiR1OqsMsUngxdD *c6_y)
{
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[8] = { "thresholdNotInContact",
    "thresholdInContact", "pauseTimeLastPostureL", "pauseTimeLastPostureR",
    "states", "pointsL", "pointsR", "standUpPositions" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 8, c6_fieldNames, 0U, 0);
  c6_thisId.fIdentifier = "thresholdNotInContact";
  c6_y->thresholdNotInContact = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "thresholdNotInContact", "thresholdNotInContact", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "thresholdInContact";
  c6_y->thresholdInContact = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "thresholdInContact", "thresholdInContact", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "pauseTimeLastPostureL";
  c6_y->pauseTimeLastPostureL = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "pauseTimeLastPostureL", "pauseTimeLastPostureL", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "pauseTimeLastPostureR";
  c6_y->pauseTimeLastPostureR = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c6_u, "pauseTimeLastPostureR", "pauseTimeLastPostureR", 0)),
    &c6_thisId);
  c6_thisId.fIdentifier = "states";
  c6_v_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "states",
    "states", 0)), &c6_thisId, c6_y->states);
  c6_thisId.fIdentifier = "pointsL";
  c6_w_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "pointsL", "pointsL", 0)), &c6_thisId, c6_y->pointsL);
  c6_thisId.fIdentifier = "pointsR";
  c6_w_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "pointsR", "pointsR", 0)), &c6_thisId, c6_y->pointsR);
  c6_thisId.fIdentifier = "standUpPositions";
  c6_x_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "standUpPositions", "standUpPositions", 0)), &c6_thisId,
                        c6_y->standUpPositions);
  sf_mex_destroy(&c6_u);
}

static void c6_v_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[299])
{
  real_T c6_dv17[299];
  int32_T c6_i162;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv17, 1, 0, 0U, 1, 0U, 2, 13,
                23);
  for (c6_i162 = 0; c6_i162 < 299; c6_i162++) {
    c6_y[c6_i162] = c6_dv17[c6_i162];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_w_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[192])
{
  real_T c6_dv18[192];
  int32_T c6_i163;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv18, 1, 0, 0U, 1, 0U, 2, 8,
                24);
  for (c6_i163 = 0; c6_i163 < 192; c6_i163++) {
    c6_y[c6_i163] = c6_dv18[c6_i163];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_x_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[72])
{
  real_T c6_dv19[72];
  int32_T c6_i164;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv19, 1, 0, 0U, 1, 0U, 2, 8, 9);
  for (c6_i164 = 0; c6_i164 < 72; c6_i164++) {
    c6_y[c6_i164] = c6_dv19[c6_i164];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_y_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[13])
{
  real_T c6_dv20[13];
  int32_T c6_i165;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv20, 1, 0, 0U, 1, 0U, 2, 13,
                1);
  for (c6_i165 = 0; c6_i165 < 13; c6_i165++) {
    c6_y[c6_i165] = c6_dv20[c6_i165];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_ab_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[8])
{
  real_T c6_dv21[8];
  int32_T c6_i166;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv21, 1, 0, 0U, 1, 0U, 2, 8, 1);
  for (c6_i166 = 0; c6_i166 < 8; c6_i166++) {
    c6_y[c6_i166] = c6_dv21[c6_i166];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_bb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_struct_9LpOi5JXaV67jTuay8hWaH *c6_y)
{
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[1] = { "standUpDeltaCoM" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 1, c6_fieldNames, 0U, 0);
  c6_thisId.fIdentifier = "standUpDeltaCoM";
  c6_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u,
    "standUpDeltaCoM", "standUpDeltaCoM", 0)), &c6_thisId, c6_y->standUpDeltaCoM);
  sf_mex_destroy(&c6_u);
}

static void c6_cb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[24])
{
  real_T c6_dv22[24];
  int32_T c6_i167;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv22, 1, 0, 0U, 1, 0U, 2, 8, 3);
  for (c6_i167 = 0; c6_i167 < 24; c6_i167++) {
    c6_y[c6_i167] = c6_dv22[c6_i167];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sm;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  c6_struct_rUGQ0INmvPpaxIctEGl5sE c6_y;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_b_sm = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sm), &c6_thisId, &c6_y);
  sf_mex_destroy(&c6_b_sm);
  *(c6_struct_rUGQ0INmvPpaxIctEGl5sE *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_i_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i168;
  real_T c6_b_inData[3];
  int32_T c6_i169;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i168 = 0; c6_i168 < 3; c6_i168++) {
    c6_b_inData[c6_i168] = (*(real_T (*)[3])c6_inData)[c6_i168];
  }

  for (c6_i169 = 0; c6_i169 < 3; c6_i169++) {
    c6_u[c6_i169] = c6_b_inData[c6_i169];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 3, 1), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_j_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i170;
  real_T c6_b_inData[6];
  int32_T c6_i171;
  real_T c6_u[6];
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i170 = 0; c6_i170 < 6; c6_i170++) {
    c6_b_inData[c6_i170] = (*(real_T (*)[6])c6_inData)[c6_i170];
  }

  for (c6_i171 = 0; c6_i171 < 6; c6_i171++) {
    c6_u[c6_i171] = c6_b_inData[c6_i171];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_k_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  boolean_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(boolean_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_l_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i172;
  int32_T c6_i173;
  int32_T c6_i174;
  real_T c6_b_inData[16];
  int32_T c6_i175;
  int32_T c6_i176;
  int32_T c6_i177;
  real_T c6_u[16];
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i172 = 0;
  for (c6_i173 = 0; c6_i173 < 4; c6_i173++) {
    for (c6_i174 = 0; c6_i174 < 4; c6_i174++) {
      c6_b_inData[c6_i174 + c6_i172] = (*(real_T (*)[16])c6_inData)[c6_i174 +
        c6_i172];
    }

    c6_i172 += 4;
  }

  c6_i175 = 0;
  for (c6_i176 = 0; c6_i176 < 4; c6_i176++) {
    for (c6_i177 = 0; c6_i177 < 4; c6_i177++) {
      c6_u[c6_i177 + c6_i175] = c6_b_inData[c6_i177 + c6_i175];
    }

    c6_i175 += 4;
  }

  c6_y = NULL;
  if (!chartInstance->c6_w_H_fixedLink_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_db_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_b_w_H_fixedLink, const char_T *c6_identifier,
  real_T c6_y[16])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_w_H_fixedLink),
    &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_w_H_fixedLink);
}

static void c6_eb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[16])
{
  real_T c6_dv23[16];
  int32_T c6_i178;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_w_H_fixedLink_not_empty = FALSE;
  } else {
    chartInstance->c6_w_H_fixedLink_not_empty = TRUE;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv23, 1, 0, 0U, 1, 0U, 2, 4,
                  4);
    for (c6_i178 = 0; c6_i178 < 16; c6_i178++) {
      c6_y[c6_i178] = c6_dv23[c6_i178];
    }
  }

  sf_mex_destroy(&c6_u);
}

static void c6_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_w_H_fixedLink;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[16];
  int32_T c6_i179;
  int32_T c6_i180;
  int32_T c6_i181;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_b_w_H_fixedLink = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_w_H_fixedLink),
    &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_w_H_fixedLink);
  c6_i179 = 0;
  for (c6_i180 = 0; c6_i180 < 4; c6_i180++) {
    for (c6_i181 = 0; c6_i181 < 4; c6_i181++) {
      (*(real_T (*)[16])c6_outData)[c6_i181 + c6_i179] = c6_y[c6_i181 + c6_i179];
    }

    c6_i179 += 4;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_m_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  if (!chartInstance->c6_fixedLink_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static real_T c6_fb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_b_fixedLink, const char_T *c6_identifier)
{
  real_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_fixedLink),
    &c6_thisId);
  sf_mex_destroy(&c6_b_fixedLink);
  return c6_y;
}

static real_T c6_gb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d1;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_fixedLink_not_empty = FALSE;
  } else {
    chartInstance->c6_fixedLink_not_empty = TRUE;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d1, 1, 0, 0U, 0, 0U, 0);
    c6_y = c6_d1;
  }

  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_fixedLink;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_b_fixedLink = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_fixedLink),
    &c6_thisId);
  sf_mex_destroy(&c6_b_fixedLink);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_n_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  if (!chartInstance->c6_state_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static real_T c6_hb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_b_state, const char_T *c6_identifier)
{
  real_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_state),
    &c6_thisId);
  sf_mex_destroy(&c6_b_state);
  return c6_y;
}

static real_T c6_ib_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d2;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_state_not_empty = FALSE;
  } else {
    chartInstance->c6_state_not_empty = TRUE;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d2, 1, 0, 0U, 0, 0U, 0);
    c6_y = c6_d2;
  }

  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_state;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_b_state = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_state),
    &c6_thisId);
  sf_mex_destroy(&c6_b_state);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static void c6_jb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[6])
{
  real_T c6_dv24[6];
  int32_T c6_i182;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv24, 1, 0, 0U, 1, 0U, 1, 6);
  for (c6_i182 = 0; c6_i182 < 6; c6_i182++) {
    c6_y[c6_i182] = c6_dv24[c6_i182];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_wrench_leftFoot;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[6];
  int32_T c6_i183;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_wrench_leftFoot = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_jb_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_wrench_leftFoot),
    &c6_thisId, c6_y);
  sf_mex_destroy(&c6_wrench_leftFoot);
  for (c6_i183 = 0; c6_i183 < 6; c6_i183++) {
    (*(real_T (*)[6])c6_outData)[c6_i183] = c6_y[c6_i183];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static void c6_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_connection;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  boolean_T c6_y;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_connection = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_connection),
    &c6_thisId);
  sf_mex_destroy(&c6_connection);
  *(boolean_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo;
  c6_ResolvedFunctionInfo c6_info[148];
  const mxArray *c6_m6 = NULL;
  int32_T c6_i184;
  c6_ResolvedFunctionInfo *c6_r2;
  c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  c6_info_helper(c6_info);
  c6_b_info_helper(c6_info);
  c6_c_info_helper(c6_info);
  sf_mex_assign(&c6_m6, sf_mex_createstruct("nameCaptureInfo", 1, 148), FALSE);
  for (c6_i184 = 0; c6_i184 < 148; c6_i184++) {
    c6_r2 = &c6_info[c6_i184];
    sf_mex_addfield(c6_m6, sf_mex_create("nameCaptureInfo", c6_r2->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r2->context)), "context", "nameCaptureInfo",
                    c6_i184);
    sf_mex_addfield(c6_m6, sf_mex_create("nameCaptureInfo", c6_r2->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c6_r2->name)), "name", "nameCaptureInfo", c6_i184);
    sf_mex_addfield(c6_m6, sf_mex_create("nameCaptureInfo", c6_r2->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c6_r2->dominantType)), "dominantType",
                    "nameCaptureInfo", c6_i184);
    sf_mex_addfield(c6_m6, sf_mex_create("nameCaptureInfo", c6_r2->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r2->resolved)), "resolved", "nameCaptureInfo",
                    c6_i184);
    sf_mex_addfield(c6_m6, sf_mex_create("nameCaptureInfo", &c6_r2->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c6_i184);
    sf_mex_addfield(c6_m6, sf_mex_create("nameCaptureInfo", &c6_r2->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c6_i184);
    sf_mex_addfield(c6_m6, sf_mex_create("nameCaptureInfo", &c6_r2->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c6_i184);
    sf_mex_addfield(c6_m6, sf_mex_create("nameCaptureInfo", &c6_r2->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c6_i184);
  }

  sf_mex_assign(&c6_nameCaptureInfo, c6_m6, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[148])
{
  c6_info[0].context = "";
  c6_info[0].name = "stateMachineWalking";
  c6_info[0].dominantType = "struct";
  c6_info[0].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m";
  c6_info[0].fileTimeLo = 1495631764U;
  c6_info[0].fileTimeHi = 0U;
  c6_info[0].mFileTimeLo = 0U;
  c6_info[0].mFileTimeHi = 0U;
  c6_info[1].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m";
  c6_info[1].name = "eye";
  c6_info[1].dominantType = "double";
  c6_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c6_info[1].fileTimeLo = 1286818688U;
  c6_info[1].fileTimeHi = 0U;
  c6_info[1].mFileTimeLo = 0U;
  c6_info[1].mFileTimeHi = 0U;
  c6_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[2].name = "eml_assert_valid_size_arg";
  c6_info[2].dominantType = "double";
  c6_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[2].fileTimeLo = 1286818694U;
  c6_info[2].fileTimeHi = 0U;
  c6_info[2].mFileTimeLo = 0U;
  c6_info[2].mFileTimeHi = 0U;
  c6_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c6_info[3].name = "isinf";
  c6_info[3].dominantType = "double";
  c6_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c6_info[3].fileTimeLo = 1286818760U;
  c6_info[3].fileTimeHi = 0U;
  c6_info[3].mFileTimeLo = 0U;
  c6_info[3].mFileTimeHi = 0U;
  c6_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c6_info[4].name = "mtimes";
  c6_info[4].dominantType = "double";
  c6_info[4].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[4].fileTimeLo = 1289519692U;
  c6_info[4].fileTimeHi = 0U;
  c6_info[4].mFileTimeLo = 0U;
  c6_info[4].mFileTimeHi = 0U;
  c6_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[5].name = "eml_index_class";
  c6_info[5].dominantType = "";
  c6_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[5].fileTimeLo = 1323170578U;
  c6_info[5].fileTimeHi = 0U;
  c6_info[5].mFileTimeLo = 0U;
  c6_info[5].mFileTimeHi = 0U;
  c6_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[6].name = "intmax";
  c6_info[6].dominantType = "char";
  c6_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[6].fileTimeLo = 1311255316U;
  c6_info[6].fileTimeHi = 0U;
  c6_info[6].mFileTimeLo = 0U;
  c6_info[6].mFileTimeHi = 0U;
  c6_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[7].name = "eml_is_float_class";
  c6_info[7].dominantType = "char";
  c6_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[7].fileTimeLo = 1286818782U;
  c6_info[7].fileTimeHi = 0U;
  c6_info[7].mFileTimeLo = 0U;
  c6_info[7].mFileTimeHi = 0U;
  c6_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[8].name = "min";
  c6_info[8].dominantType = "double";
  c6_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[8].fileTimeLo = 1311255318U;
  c6_info[8].fileTimeHi = 0U;
  c6_info[8].mFileTimeLo = 0U;
  c6_info[8].mFileTimeHi = 0U;
  c6_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[9].name = "eml_min_or_max";
  c6_info[9].dominantType = "char";
  c6_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c6_info[9].fileTimeLo = 1334071490U;
  c6_info[9].fileTimeHi = 0U;
  c6_info[9].mFileTimeLo = 0U;
  c6_info[9].mFileTimeHi = 0U;
  c6_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[10].name = "eml_scalar_eg";
  c6_info[10].dominantType = "double";
  c6_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[10].fileTimeLo = 1286818796U;
  c6_info[10].fileTimeHi = 0U;
  c6_info[10].mFileTimeLo = 0U;
  c6_info[10].mFileTimeHi = 0U;
  c6_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[11].name = "eml_scalexp_alloc";
  c6_info[11].dominantType = "double";
  c6_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[11].fileTimeLo = 1352424860U;
  c6_info[11].fileTimeHi = 0U;
  c6_info[11].mFileTimeLo = 0U;
  c6_info[11].mFileTimeHi = 0U;
  c6_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[12].name = "eml_index_class";
  c6_info[12].dominantType = "";
  c6_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[12].fileTimeLo = 1323170578U;
  c6_info[12].fileTimeHi = 0U;
  c6_info[12].mFileTimeLo = 0U;
  c6_info[12].mFileTimeHi = 0U;
  c6_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c6_info[13].name = "eml_scalar_eg";
  c6_info[13].dominantType = "double";
  c6_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[13].fileTimeLo = 1286818796U;
  c6_info[13].fileTimeHi = 0U;
  c6_info[13].mFileTimeLo = 0U;
  c6_info[13].mFileTimeHi = 0U;
  c6_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[14].name = "eml_index_class";
  c6_info[14].dominantType = "";
  c6_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[14].fileTimeLo = 1323170578U;
  c6_info[14].fileTimeHi = 0U;
  c6_info[14].mFileTimeLo = 0U;
  c6_info[14].mFileTimeHi = 0U;
  c6_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[15].name = "eml_int_forloop_overflow_check";
  c6_info[15].dominantType = "";
  c6_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[15].fileTimeLo = 1346510340U;
  c6_info[15].fileTimeHi = 0U;
  c6_info[15].mFileTimeLo = 0U;
  c6_info[15].mFileTimeHi = 0U;
  c6_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c6_info[16].name = "intmax";
  c6_info[16].dominantType = "char";
  c6_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[16].fileTimeLo = 1311255316U;
  c6_info[16].fileTimeHi = 0U;
  c6_info[16].mFileTimeLo = 0U;
  c6_info[16].mFileTimeHi = 0U;
  c6_info[17].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m";
  c6_info[17].name = "mtimes";
  c6_info[17].dominantType = "double";
  c6_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[17].fileTimeLo = 1289519692U;
  c6_info[17].fileTimeHi = 0U;
  c6_info[17].mFileTimeLo = 0U;
  c6_info[17].mFileTimeHi = 0U;
  c6_info[18].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[18].name = "eml_index_class";
  c6_info[18].dominantType = "";
  c6_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[18].fileTimeLo = 1323170578U;
  c6_info[18].fileTimeHi = 0U;
  c6_info[18].mFileTimeLo = 0U;
  c6_info[18].mFileTimeHi = 0U;
  c6_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[19].name = "eml_scalar_eg";
  c6_info[19].dominantType = "double";
  c6_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[19].fileTimeLo = 1286818796U;
  c6_info[19].fileTimeHi = 0U;
  c6_info[19].mFileTimeLo = 0U;
  c6_info[19].mFileTimeHi = 0U;
  c6_info[20].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[20].name = "eml_xgemm";
  c6_info[20].dominantType = "char";
  c6_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c6_info[20].fileTimeLo = 1299076772U;
  c6_info[20].fileTimeHi = 0U;
  c6_info[20].mFileTimeLo = 0U;
  c6_info[20].mFileTimeHi = 0U;
  c6_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c6_info[21].name = "eml_blas_inline";
  c6_info[21].dominantType = "";
  c6_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[21].fileTimeLo = 1299076768U;
  c6_info[21].fileTimeHi = 0U;
  c6_info[21].mFileTimeLo = 0U;
  c6_info[21].mFileTimeHi = 0U;
  c6_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c6_info[22].name = "mtimes";
  c6_info[22].dominantType = "double";
  c6_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[22].fileTimeLo = 1289519692U;
  c6_info[22].fileTimeHi = 0U;
  c6_info[22].mFileTimeLo = 0U;
  c6_info[22].mFileTimeHi = 0U;
  c6_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[23].name = "eml_index_class";
  c6_info[23].dominantType = "";
  c6_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[23].fileTimeLo = 1323170578U;
  c6_info[23].fileTimeHi = 0U;
  c6_info[23].mFileTimeLo = 0U;
  c6_info[23].mFileTimeHi = 0U;
  c6_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[24].name = "eml_scalar_eg";
  c6_info[24].dominantType = "double";
  c6_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[24].fileTimeLo = 1286818796U;
  c6_info[24].fileTimeHi = 0U;
  c6_info[24].mFileTimeLo = 0U;
  c6_info[24].mFileTimeHi = 0U;
  c6_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[25].name = "eml_refblas_xgemm";
  c6_info[25].dominantType = "char";
  c6_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c6_info[25].fileTimeLo = 1299076774U;
  c6_info[25].fileTimeHi = 0U;
  c6_info[25].mFileTimeLo = 0U;
  c6_info[25].mFileTimeHi = 0U;
  c6_info[26].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m";
  c6_info[26].name = "norm";
  c6_info[26].dominantType = "double";
  c6_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c6_info[26].fileTimeLo = 1336522094U;
  c6_info[26].fileTimeHi = 0U;
  c6_info[26].mFileTimeLo = 0U;
  c6_info[26].mFileTimeHi = 0U;
  c6_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c6_info[27].name = "eml_index_class";
  c6_info[27].dominantType = "";
  c6_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[27].fileTimeLo = 1323170578U;
  c6_info[27].fileTimeHi = 0U;
  c6_info[27].mFileTimeLo = 0U;
  c6_info[27].mFileTimeHi = 0U;
  c6_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c6_info[28].name = "eml_xnrm2";
  c6_info[28].dominantType = "double";
  c6_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c6_info[28].fileTimeLo = 1299076776U;
  c6_info[28].fileTimeHi = 0U;
  c6_info[28].mFileTimeLo = 0U;
  c6_info[28].mFileTimeHi = 0U;
  c6_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c6_info[29].name = "eml_blas_inline";
  c6_info[29].dominantType = "";
  c6_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[29].fileTimeLo = 1299076768U;
  c6_info[29].fileTimeHi = 0U;
  c6_info[29].mFileTimeLo = 0U;
  c6_info[29].mFileTimeHi = 0U;
  c6_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c6_info[30].name = "eml_index_class";
  c6_info[30].dominantType = "";
  c6_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[30].fileTimeLo = 1323170578U;
  c6_info[30].fileTimeHi = 0U;
  c6_info[30].mFileTimeLo = 0U;
  c6_info[30].mFileTimeHi = 0U;
  c6_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c6_info[31].name = "eml_refblas_xnrm2";
  c6_info[31].dominantType = "double";
  c6_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c6_info[31].fileTimeLo = 1299076784U;
  c6_info[31].fileTimeHi = 0U;
  c6_info[31].mFileTimeLo = 0U;
  c6_info[31].mFileTimeHi = 0U;
  c6_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c6_info[32].name = "realmin";
  c6_info[32].dominantType = "char";
  c6_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c6_info[32].fileTimeLo = 1307651242U;
  c6_info[32].fileTimeHi = 0U;
  c6_info[32].mFileTimeLo = 0U;
  c6_info[32].mFileTimeHi = 0U;
  c6_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c6_info[33].name = "eml_realmin";
  c6_info[33].dominantType = "char";
  c6_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c6_info[33].fileTimeLo = 1307651244U;
  c6_info[33].fileTimeHi = 0U;
  c6_info[33].mFileTimeLo = 0U;
  c6_info[33].mFileTimeHi = 0U;
  c6_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c6_info[34].name = "eml_float_model";
  c6_info[34].dominantType = "char";
  c6_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[34].fileTimeLo = 1326727996U;
  c6_info[34].fileTimeHi = 0U;
  c6_info[34].mFileTimeLo = 0U;
  c6_info[34].mFileTimeHi = 0U;
  c6_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c6_info[35].name = "eml_index_class";
  c6_info[35].dominantType = "";
  c6_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[35].fileTimeLo = 1323170578U;
  c6_info[35].fileTimeHi = 0U;
  c6_info[35].mFileTimeLo = 0U;
  c6_info[35].mFileTimeHi = 0U;
  c6_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c6_info[36].name = "eml_index_minus";
  c6_info[36].dominantType = "double";
  c6_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[36].fileTimeLo = 1286818778U;
  c6_info[36].fileTimeHi = 0U;
  c6_info[36].mFileTimeLo = 0U;
  c6_info[36].mFileTimeHi = 0U;
  c6_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[37].name = "eml_index_class";
  c6_info[37].dominantType = "";
  c6_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[37].fileTimeLo = 1323170578U;
  c6_info[37].fileTimeHi = 0U;
  c6_info[37].mFileTimeLo = 0U;
  c6_info[37].mFileTimeHi = 0U;
  c6_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c6_info[38].name = "eml_index_times";
  c6_info[38].dominantType = "coder.internal.indexInt";
  c6_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[38].fileTimeLo = 1286818780U;
  c6_info[38].fileTimeHi = 0U;
  c6_info[38].mFileTimeLo = 0U;
  c6_info[38].mFileTimeHi = 0U;
  c6_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[39].name = "eml_index_class";
  c6_info[39].dominantType = "";
  c6_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[39].fileTimeLo = 1323170578U;
  c6_info[39].fileTimeHi = 0U;
  c6_info[39].mFileTimeLo = 0U;
  c6_info[39].mFileTimeHi = 0U;
  c6_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c6_info[40].name = "eml_index_plus";
  c6_info[40].dominantType = "coder.internal.indexInt";
  c6_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[40].fileTimeLo = 1286818778U;
  c6_info[40].fileTimeHi = 0U;
  c6_info[40].mFileTimeLo = 0U;
  c6_info[40].mFileTimeHi = 0U;
  c6_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[41].name = "eml_index_class";
  c6_info[41].dominantType = "";
  c6_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[41].fileTimeLo = 1323170578U;
  c6_info[41].fileTimeHi = 0U;
  c6_info[41].mFileTimeLo = 0U;
  c6_info[41].mFileTimeHi = 0U;
  c6_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c6_info[42].name = "eml_int_forloop_overflow_check";
  c6_info[42].dominantType = "";
  c6_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[42].fileTimeLo = 1346510340U;
  c6_info[42].fileTimeHi = 0U;
  c6_info[42].mFileTimeLo = 0U;
  c6_info[42].mFileTimeHi = 0U;
  c6_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c6_info[43].name = "abs";
  c6_info[43].dominantType = "double";
  c6_info[43].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[43].fileTimeLo = 1343830366U;
  c6_info[43].fileTimeHi = 0U;
  c6_info[43].mFileTimeLo = 0U;
  c6_info[43].mFileTimeHi = 0U;
  c6_info[44].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[44].name = "eml_scalar_abs";
  c6_info[44].dominantType = "double";
  c6_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c6_info[44].fileTimeLo = 1286818712U;
  c6_info[44].fileTimeHi = 0U;
  c6_info[44].mFileTimeLo = 0U;
  c6_info[44].mFileTimeHi = 0U;
  c6_info[45].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m";
  c6_info[45].name = "eps";
  c6_info[45].dominantType = "";
  c6_info[45].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[45].fileTimeLo = 1326727996U;
  c6_info[45].fileTimeHi = 0U;
  c6_info[45].mFileTimeLo = 0U;
  c6_info[45].mFileTimeHi = 0U;
  c6_info[46].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[46].name = "eml_eps";
  c6_info[46].dominantType = "char";
  c6_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[46].fileTimeLo = 1326727996U;
  c6_info[46].fileTimeHi = 0U;
  c6_info[46].mFileTimeLo = 0U;
  c6_info[46].mFileTimeHi = 0U;
  c6_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[47].name = "eml_float_model";
  c6_info[47].dominantType = "char";
  c6_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[47].fileTimeLo = 1326727996U;
  c6_info[47].fileTimeHi = 0U;
  c6_info[47].mFileTimeLo = 0U;
  c6_info[47].mFileTimeHi = 0U;
  c6_info[48].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m";
  c6_info[48].name = "any";
  c6_info[48].dominantType = "double";
  c6_info[48].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m";
  c6_info[48].fileTimeLo = 1286818834U;
  c6_info[48].fileTimeHi = 0U;
  c6_info[48].mFileTimeLo = 0U;
  c6_info[48].mFileTimeHi = 0U;
  c6_info[49].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m";
  c6_info[49].name = "eml_all_or_any";
  c6_info[49].dominantType = "char";
  c6_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_all_or_any.m";
  c6_info[49].fileTimeLo = 1286818694U;
  c6_info[49].fileTimeHi = 0U;
  c6_info[49].mFileTimeLo = 0U;
  c6_info[49].mFileTimeHi = 0U;
  c6_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_all_or_any.m";
  c6_info[50].name = "isequal";
  c6_info[50].dominantType = "double";
  c6_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c6_info[50].fileTimeLo = 1286818758U;
  c6_info[50].fileTimeHi = 0U;
  c6_info[50].mFileTimeLo = 0U;
  c6_info[50].mFileTimeHi = 0U;
  c6_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c6_info[51].name = "eml_isequal_core";
  c6_info[51].dominantType = "double";
  c6_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[51].fileTimeLo = 1286818786U;
  c6_info[51].fileTimeHi = 0U;
  c6_info[51].mFileTimeLo = 0U;
  c6_info[51].mFileTimeHi = 0U;
  c6_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_all_or_any.m";
  c6_info[52].name = "eml_const_nonsingleton_dim";
  c6_info[52].dominantType = "double";
  c6_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c6_info[52].fileTimeLo = 1286818696U;
  c6_info[52].fileTimeHi = 0U;
  c6_info[52].mFileTimeLo = 0U;
  c6_info[52].mFileTimeHi = 0U;
  c6_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_all_or_any.m";
  c6_info[53].name = "isnan";
  c6_info[53].dominantType = "double";
  c6_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c6_info[53].fileTimeLo = 1286818760U;
  c6_info[53].fileTimeHi = 0U;
  c6_info[53].mFileTimeLo = 0U;
  c6_info[53].mFileTimeHi = 0U;
  c6_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c6_info[54].name = "abs";
  c6_info[54].dominantType = "double";
  c6_info[54].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[54].fileTimeLo = 1343830366U;
  c6_info[54].fileTimeHi = 0U;
  c6_info[54].mFileTimeLo = 0U;
  c6_info[54].mFileTimeHi = 0U;
  c6_info[55].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m";
  c6_info[55].name = "mrdivide";
  c6_info[55].dominantType = "double";
  c6_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[55].fileTimeLo = 1357951548U;
  c6_info[55].fileTimeHi = 0U;
  c6_info[55].mFileTimeLo = 1319729966U;
  c6_info[55].mFileTimeHi = 0U;
  c6_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[56].name = "mldivide";
  c6_info[56].dominantType = "double";
  c6_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c6_info[56].fileTimeLo = 1357951548U;
  c6_info[56].fileTimeHi = 0U;
  c6_info[56].mFileTimeLo = 1319729966U;
  c6_info[56].mFileTimeHi = 0U;
  c6_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c6_info[57].name = "eml_lusolve";
  c6_info[57].dominantType = "double";
  c6_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c6_info[57].fileTimeLo = 1309451196U;
  c6_info[57].fileTimeHi = 0U;
  c6_info[57].mFileTimeLo = 0U;
  c6_info[57].mFileTimeHi = 0U;
  c6_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c6_info[58].name = "eml_index_class";
  c6_info[58].dominantType = "";
  c6_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[58].fileTimeLo = 1323170578U;
  c6_info[58].fileTimeHi = 0U;
  c6_info[58].mFileTimeLo = 0U;
  c6_info[58].mFileTimeHi = 0U;
  c6_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c6_info[59].name = "eml_index_class";
  c6_info[59].dominantType = "";
  c6_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[59].fileTimeLo = 1323170578U;
  c6_info[59].fileTimeHi = 0U;
  c6_info[59].mFileTimeLo = 0U;
  c6_info[59].mFileTimeHi = 0U;
  c6_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c6_info[60].name = "eml_xgetrf";
  c6_info[60].dominantType = "double";
  c6_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c6_info[60].fileTimeLo = 1286818806U;
  c6_info[60].fileTimeHi = 0U;
  c6_info[60].mFileTimeLo = 0U;
  c6_info[60].mFileTimeHi = 0U;
  c6_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c6_info[61].name = "eml_lapack_xgetrf";
  c6_info[61].dominantType = "double";
  c6_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c6_info[61].fileTimeLo = 1286818810U;
  c6_info[61].fileTimeHi = 0U;
  c6_info[61].mFileTimeLo = 0U;
  c6_info[61].mFileTimeHi = 0U;
  c6_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c6_info[62].name = "eml_matlab_zgetrf";
  c6_info[62].dominantType = "double";
  c6_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[62].fileTimeLo = 1302688994U;
  c6_info[62].fileTimeHi = 0U;
  c6_info[62].mFileTimeLo = 0U;
  c6_info[62].mFileTimeHi = 0U;
  c6_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[63].name = "realmin";
  c6_info[63].dominantType = "char";
  c6_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c6_info[63].fileTimeLo = 1307651242U;
  c6_info[63].fileTimeHi = 0U;
  c6_info[63].mFileTimeLo = 0U;
  c6_info[63].mFileTimeHi = 0U;
}

static void c6_b_info_helper(c6_ResolvedFunctionInfo c6_info[148])
{
  c6_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[64].name = "eps";
  c6_info[64].dominantType = "char";
  c6_info[64].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[64].fileTimeLo = 1326727996U;
  c6_info[64].fileTimeHi = 0U;
  c6_info[64].mFileTimeLo = 0U;
  c6_info[64].mFileTimeHi = 0U;
  c6_info[65].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[65].name = "eml_is_float_class";
  c6_info[65].dominantType = "char";
  c6_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[65].fileTimeLo = 1286818782U;
  c6_info[65].fileTimeHi = 0U;
  c6_info[65].mFileTimeLo = 0U;
  c6_info[65].mFileTimeHi = 0U;
  c6_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[66].name = "min";
  c6_info[66].dominantType = "coder.internal.indexInt";
  c6_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[66].fileTimeLo = 1311255318U;
  c6_info[66].fileTimeHi = 0U;
  c6_info[66].mFileTimeLo = 0U;
  c6_info[66].mFileTimeHi = 0U;
  c6_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[67].name = "eml_scalar_eg";
  c6_info[67].dominantType = "coder.internal.indexInt";
  c6_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[67].fileTimeLo = 1286818796U;
  c6_info[67].fileTimeHi = 0U;
  c6_info[67].mFileTimeLo = 0U;
  c6_info[67].mFileTimeHi = 0U;
  c6_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[68].name = "eml_scalexp_alloc";
  c6_info[68].dominantType = "coder.internal.indexInt";
  c6_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[68].fileTimeLo = 1352424860U;
  c6_info[68].fileTimeHi = 0U;
  c6_info[68].mFileTimeLo = 0U;
  c6_info[68].mFileTimeHi = 0U;
  c6_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c6_info[69].name = "eml_scalar_eg";
  c6_info[69].dominantType = "coder.internal.indexInt";
  c6_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[69].fileTimeLo = 1286818796U;
  c6_info[69].fileTimeHi = 0U;
  c6_info[69].mFileTimeLo = 0U;
  c6_info[69].mFileTimeHi = 0U;
  c6_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[70].name = "colon";
  c6_info[70].dominantType = "double";
  c6_info[70].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[70].fileTimeLo = 1348191928U;
  c6_info[70].fileTimeHi = 0U;
  c6_info[70].mFileTimeLo = 0U;
  c6_info[70].mFileTimeHi = 0U;
  c6_info[71].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[71].name = "colon";
  c6_info[71].dominantType = "double";
  c6_info[71].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[71].fileTimeLo = 1348191928U;
  c6_info[71].fileTimeHi = 0U;
  c6_info[71].mFileTimeLo = 0U;
  c6_info[71].mFileTimeHi = 0U;
  c6_info[72].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[72].name = "floor";
  c6_info[72].dominantType = "double";
  c6_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[72].fileTimeLo = 1343830380U;
  c6_info[72].fileTimeHi = 0U;
  c6_info[72].mFileTimeLo = 0U;
  c6_info[72].mFileTimeHi = 0U;
  c6_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[73].name = "eml_scalar_floor";
  c6_info[73].dominantType = "double";
  c6_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[73].fileTimeLo = 1286818726U;
  c6_info[73].fileTimeHi = 0U;
  c6_info[73].mFileTimeLo = 0U;
  c6_info[73].mFileTimeHi = 0U;
  c6_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c6_info[74].name = "intmin";
  c6_info[74].dominantType = "char";
  c6_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c6_info[74].fileTimeLo = 1311255318U;
  c6_info[74].fileTimeHi = 0U;
  c6_info[74].mFileTimeLo = 0U;
  c6_info[74].mFileTimeHi = 0U;
  c6_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c6_info[75].name = "intmax";
  c6_info[75].dominantType = "char";
  c6_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[75].fileTimeLo = 1311255316U;
  c6_info[75].fileTimeHi = 0U;
  c6_info[75].mFileTimeLo = 0U;
  c6_info[75].mFileTimeHi = 0U;
  c6_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c6_info[76].name = "intmin";
  c6_info[76].dominantType = "char";
  c6_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c6_info[76].fileTimeLo = 1311255318U;
  c6_info[76].fileTimeHi = 0U;
  c6_info[76].mFileTimeLo = 0U;
  c6_info[76].mFileTimeHi = 0U;
  c6_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c6_info[77].name = "intmax";
  c6_info[77].dominantType = "char";
  c6_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[77].fileTimeLo = 1311255316U;
  c6_info[77].fileTimeHi = 0U;
  c6_info[77].mFileTimeLo = 0U;
  c6_info[77].mFileTimeHi = 0U;
  c6_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c6_info[78].name = "eml_isa_uint";
  c6_info[78].dominantType = "coder.internal.indexInt";
  c6_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c6_info[78].fileTimeLo = 1286818784U;
  c6_info[78].fileTimeHi = 0U;
  c6_info[78].mFileTimeLo = 0U;
  c6_info[78].mFileTimeHi = 0U;
  c6_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[79].name = "eml_unsigned_class";
  c6_info[79].dominantType = "char";
  c6_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c6_info[79].fileTimeLo = 1323170580U;
  c6_info[79].fileTimeHi = 0U;
  c6_info[79].mFileTimeLo = 0U;
  c6_info[79].mFileTimeHi = 0U;
  c6_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c6_info[80].name = "eml_index_class";
  c6_info[80].dominantType = "";
  c6_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[80].fileTimeLo = 1323170578U;
  c6_info[80].fileTimeHi = 0U;
  c6_info[80].mFileTimeLo = 0U;
  c6_info[80].mFileTimeHi = 0U;
  c6_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[81].name = "eml_index_class";
  c6_info[81].dominantType = "";
  c6_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[81].fileTimeLo = 1323170578U;
  c6_info[81].fileTimeHi = 0U;
  c6_info[81].mFileTimeLo = 0U;
  c6_info[81].mFileTimeHi = 0U;
  c6_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[82].name = "intmax";
  c6_info[82].dominantType = "char";
  c6_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[82].fileTimeLo = 1311255316U;
  c6_info[82].fileTimeHi = 0U;
  c6_info[82].mFileTimeLo = 0U;
  c6_info[82].mFileTimeHi = 0U;
  c6_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[83].name = "eml_isa_uint";
  c6_info[83].dominantType = "coder.internal.indexInt";
  c6_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c6_info[83].fileTimeLo = 1286818784U;
  c6_info[83].fileTimeHi = 0U;
  c6_info[83].mFileTimeLo = 0U;
  c6_info[83].mFileTimeHi = 0U;
  c6_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[84].name = "eml_index_plus";
  c6_info[84].dominantType = "double";
  c6_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[84].fileTimeLo = 1286818778U;
  c6_info[84].fileTimeHi = 0U;
  c6_info[84].mFileTimeLo = 0U;
  c6_info[84].mFileTimeHi = 0U;
  c6_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c6_info[85].name = "eml_int_forloop_overflow_check";
  c6_info[85].dominantType = "";
  c6_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[85].fileTimeLo = 1346510340U;
  c6_info[85].fileTimeHi = 0U;
  c6_info[85].mFileTimeLo = 0U;
  c6_info[85].mFileTimeHi = 0U;
  c6_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[86].name = "eml_index_class";
  c6_info[86].dominantType = "";
  c6_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[86].fileTimeLo = 1323170578U;
  c6_info[86].fileTimeHi = 0U;
  c6_info[86].mFileTimeLo = 0U;
  c6_info[86].mFileTimeHi = 0U;
  c6_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[87].name = "eml_index_plus";
  c6_info[87].dominantType = "double";
  c6_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[87].fileTimeLo = 1286818778U;
  c6_info[87].fileTimeHi = 0U;
  c6_info[87].mFileTimeLo = 0U;
  c6_info[87].mFileTimeHi = 0U;
  c6_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[88].name = "eml_int_forloop_overflow_check";
  c6_info[88].dominantType = "";
  c6_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[88].fileTimeLo = 1346510340U;
  c6_info[88].fileTimeHi = 0U;
  c6_info[88].mFileTimeLo = 0U;
  c6_info[88].mFileTimeHi = 0U;
  c6_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[89].name = "eml_index_minus";
  c6_info[89].dominantType = "double";
  c6_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[89].fileTimeLo = 1286818778U;
  c6_info[89].fileTimeHi = 0U;
  c6_info[89].mFileTimeLo = 0U;
  c6_info[89].mFileTimeHi = 0U;
  c6_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[90].name = "eml_index_minus";
  c6_info[90].dominantType = "coder.internal.indexInt";
  c6_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[90].fileTimeLo = 1286818778U;
  c6_info[90].fileTimeHi = 0U;
  c6_info[90].mFileTimeLo = 0U;
  c6_info[90].mFileTimeHi = 0U;
  c6_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[91].name = "eml_index_times";
  c6_info[91].dominantType = "coder.internal.indexInt";
  c6_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[91].fileTimeLo = 1286818780U;
  c6_info[91].fileTimeHi = 0U;
  c6_info[91].mFileTimeLo = 0U;
  c6_info[91].mFileTimeHi = 0U;
  c6_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[92].name = "eml_index_plus";
  c6_info[92].dominantType = "coder.internal.indexInt";
  c6_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[92].fileTimeLo = 1286818778U;
  c6_info[92].fileTimeHi = 0U;
  c6_info[92].mFileTimeLo = 0U;
  c6_info[92].mFileTimeHi = 0U;
  c6_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[93].name = "eml_ixamax";
  c6_info[93].dominantType = "double";
  c6_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c6_info[93].fileTimeLo = 1299076770U;
  c6_info[93].fileTimeHi = 0U;
  c6_info[93].mFileTimeLo = 0U;
  c6_info[93].mFileTimeHi = 0U;
  c6_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c6_info[94].name = "eml_blas_inline";
  c6_info[94].dominantType = "";
  c6_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[94].fileTimeLo = 1299076768U;
  c6_info[94].fileTimeHi = 0U;
  c6_info[94].mFileTimeLo = 0U;
  c6_info[94].mFileTimeHi = 0U;
  c6_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c6_info[95].name = "length";
  c6_info[95].dominantType = "double";
  c6_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[95].fileTimeLo = 1303146206U;
  c6_info[95].fileTimeHi = 0U;
  c6_info[95].mFileTimeLo = 0U;
  c6_info[95].mFileTimeHi = 0U;
  c6_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c6_info[96].name = "eml_index_class";
  c6_info[96].dominantType = "";
  c6_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[96].fileTimeLo = 1323170578U;
  c6_info[96].fileTimeHi = 0U;
  c6_info[96].mFileTimeLo = 0U;
  c6_info[96].mFileTimeHi = 0U;
  c6_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c6_info[97].name = "eml_index_class";
  c6_info[97].dominantType = "";
  c6_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[97].fileTimeLo = 1323170578U;
  c6_info[97].fileTimeHi = 0U;
  c6_info[97].mFileTimeLo = 0U;
  c6_info[97].mFileTimeHi = 0U;
  c6_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c6_info[98].name = "eml_refblas_ixamax";
  c6_info[98].dominantType = "double";
  c6_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[98].fileTimeLo = 1299076770U;
  c6_info[98].fileTimeHi = 0U;
  c6_info[98].mFileTimeLo = 0U;
  c6_info[98].mFileTimeHi = 0U;
  c6_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[99].name = "eml_index_class";
  c6_info[99].dominantType = "";
  c6_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[99].fileTimeLo = 1323170578U;
  c6_info[99].fileTimeHi = 0U;
  c6_info[99].mFileTimeLo = 0U;
  c6_info[99].mFileTimeHi = 0U;
  c6_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[100].name = "eml_xcabs1";
  c6_info[100].dominantType = "double";
  c6_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c6_info[100].fileTimeLo = 1286818706U;
  c6_info[100].fileTimeHi = 0U;
  c6_info[100].mFileTimeLo = 0U;
  c6_info[100].mFileTimeHi = 0U;
  c6_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c6_info[101].name = "abs";
  c6_info[101].dominantType = "double";
  c6_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[101].fileTimeLo = 1343830366U;
  c6_info[101].fileTimeHi = 0U;
  c6_info[101].mFileTimeLo = 0U;
  c6_info[101].mFileTimeHi = 0U;
  c6_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[102].name = "eml_int_forloop_overflow_check";
  c6_info[102].dominantType = "";
  c6_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[102].fileTimeLo = 1346510340U;
  c6_info[102].fileTimeHi = 0U;
  c6_info[102].mFileTimeLo = 0U;
  c6_info[102].mFileTimeHi = 0U;
  c6_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[103].name = "eml_index_plus";
  c6_info[103].dominantType = "coder.internal.indexInt";
  c6_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[103].fileTimeLo = 1286818778U;
  c6_info[103].fileTimeHi = 0U;
  c6_info[103].mFileTimeLo = 0U;
  c6_info[103].mFileTimeHi = 0U;
  c6_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[104].name = "eml_xswap";
  c6_info[104].dominantType = "double";
  c6_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c6_info[104].fileTimeLo = 1299076778U;
  c6_info[104].fileTimeHi = 0U;
  c6_info[104].mFileTimeLo = 0U;
  c6_info[104].mFileTimeHi = 0U;
  c6_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c6_info[105].name = "eml_blas_inline";
  c6_info[105].dominantType = "";
  c6_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[105].fileTimeLo = 1299076768U;
  c6_info[105].fileTimeHi = 0U;
  c6_info[105].mFileTimeLo = 0U;
  c6_info[105].mFileTimeHi = 0U;
  c6_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c6_info[106].name = "eml_index_class";
  c6_info[106].dominantType = "";
  c6_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[106].fileTimeLo = 1323170578U;
  c6_info[106].fileTimeHi = 0U;
  c6_info[106].mFileTimeLo = 0U;
  c6_info[106].mFileTimeHi = 0U;
  c6_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c6_info[107].name = "eml_refblas_xswap";
  c6_info[107].dominantType = "double";
  c6_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[107].fileTimeLo = 1299076786U;
  c6_info[107].fileTimeHi = 0U;
  c6_info[107].mFileTimeLo = 0U;
  c6_info[107].mFileTimeHi = 0U;
  c6_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[108].name = "eml_index_class";
  c6_info[108].dominantType = "";
  c6_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[108].fileTimeLo = 1323170578U;
  c6_info[108].fileTimeHi = 0U;
  c6_info[108].mFileTimeLo = 0U;
  c6_info[108].mFileTimeHi = 0U;
  c6_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[109].name = "abs";
  c6_info[109].dominantType = "coder.internal.indexInt";
  c6_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[109].fileTimeLo = 1343830366U;
  c6_info[109].fileTimeHi = 0U;
  c6_info[109].mFileTimeLo = 0U;
  c6_info[109].mFileTimeHi = 0U;
  c6_info[110].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[110].name = "eml_scalar_abs";
  c6_info[110].dominantType = "coder.internal.indexInt";
  c6_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c6_info[110].fileTimeLo = 1286818712U;
  c6_info[110].fileTimeHi = 0U;
  c6_info[110].mFileTimeLo = 0U;
  c6_info[110].mFileTimeHi = 0U;
  c6_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[111].name = "eml_int_forloop_overflow_check";
  c6_info[111].dominantType = "";
  c6_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[111].fileTimeLo = 1346510340U;
  c6_info[111].fileTimeHi = 0U;
  c6_info[111].mFileTimeLo = 0U;
  c6_info[111].mFileTimeHi = 0U;
  c6_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[112].name = "eml_index_plus";
  c6_info[112].dominantType = "coder.internal.indexInt";
  c6_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[112].fileTimeLo = 1286818778U;
  c6_info[112].fileTimeHi = 0U;
  c6_info[112].mFileTimeLo = 0U;
  c6_info[112].mFileTimeHi = 0U;
  c6_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[113].name = "eml_div";
  c6_info[113].dominantType = "double";
  c6_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[113].fileTimeLo = 1313347810U;
  c6_info[113].fileTimeHi = 0U;
  c6_info[113].mFileTimeLo = 0U;
  c6_info[113].mFileTimeHi = 0U;
  c6_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[114].name = "eml_xgeru";
  c6_info[114].dominantType = "double";
  c6_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c6_info[114].fileTimeLo = 1299076774U;
  c6_info[114].fileTimeHi = 0U;
  c6_info[114].mFileTimeLo = 0U;
  c6_info[114].mFileTimeHi = 0U;
  c6_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c6_info[115].name = "eml_blas_inline";
  c6_info[115].dominantType = "";
  c6_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[115].fileTimeLo = 1299076768U;
  c6_info[115].fileTimeHi = 0U;
  c6_info[115].mFileTimeLo = 0U;
  c6_info[115].mFileTimeHi = 0U;
  c6_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c6_info[116].name = "eml_xger";
  c6_info[116].dominantType = "double";
  c6_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c6_info[116].fileTimeLo = 1299076774U;
  c6_info[116].fileTimeHi = 0U;
  c6_info[116].mFileTimeLo = 0U;
  c6_info[116].mFileTimeHi = 0U;
  c6_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c6_info[117].name = "eml_blas_inline";
  c6_info[117].dominantType = "";
  c6_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[117].fileTimeLo = 1299076768U;
  c6_info[117].fileTimeHi = 0U;
  c6_info[117].mFileTimeLo = 0U;
  c6_info[117].mFileTimeHi = 0U;
  c6_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c6_info[118].name = "intmax";
  c6_info[118].dominantType = "char";
  c6_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[118].fileTimeLo = 1311255316U;
  c6_info[118].fileTimeHi = 0U;
  c6_info[118].mFileTimeLo = 0U;
  c6_info[118].mFileTimeHi = 0U;
  c6_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c6_info[119].name = "min";
  c6_info[119].dominantType = "double";
  c6_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[119].fileTimeLo = 1311255318U;
  c6_info[119].fileTimeHi = 0U;
  c6_info[119].mFileTimeLo = 0U;
  c6_info[119].mFileTimeHi = 0U;
  c6_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c6_info[120].name = "mtimes";
  c6_info[120].dominantType = "double";
  c6_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[120].fileTimeLo = 1289519692U;
  c6_info[120].fileTimeHi = 0U;
  c6_info[120].mFileTimeLo = 0U;
  c6_info[120].mFileTimeHi = 0U;
  c6_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c6_info[121].name = "eml_index_class";
  c6_info[121].dominantType = "";
  c6_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[121].fileTimeLo = 1323170578U;
  c6_info[121].fileTimeHi = 0U;
  c6_info[121].mFileTimeLo = 0U;
  c6_info[121].mFileTimeHi = 0U;
  c6_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c6_info[122].name = "eml_refblas_xger";
  c6_info[122].dominantType = "double";
  c6_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c6_info[122].fileTimeLo = 1299076776U;
  c6_info[122].fileTimeHi = 0U;
  c6_info[122].mFileTimeLo = 0U;
  c6_info[122].mFileTimeHi = 0U;
  c6_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c6_info[123].name = "eml_refblas_xgerx";
  c6_info[123].dominantType = "char";
  c6_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[123].fileTimeLo = 1299076778U;
  c6_info[123].fileTimeHi = 0U;
  c6_info[123].mFileTimeLo = 0U;
  c6_info[123].mFileTimeHi = 0U;
  c6_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[124].name = "eml_index_class";
  c6_info[124].dominantType = "";
  c6_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[124].fileTimeLo = 1323170578U;
  c6_info[124].fileTimeHi = 0U;
  c6_info[124].mFileTimeLo = 0U;
  c6_info[124].mFileTimeHi = 0U;
  c6_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[125].name = "abs";
  c6_info[125].dominantType = "coder.internal.indexInt";
  c6_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[125].fileTimeLo = 1343830366U;
  c6_info[125].fileTimeHi = 0U;
  c6_info[125].mFileTimeLo = 0U;
  c6_info[125].mFileTimeHi = 0U;
  c6_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[126].name = "eml_index_minus";
  c6_info[126].dominantType = "double";
  c6_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[126].fileTimeLo = 1286818778U;
  c6_info[126].fileTimeHi = 0U;
  c6_info[126].mFileTimeLo = 0U;
  c6_info[126].mFileTimeHi = 0U;
  c6_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[127].name = "eml_int_forloop_overflow_check";
  c6_info[127].dominantType = "";
  c6_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[127].fileTimeLo = 1346510340U;
  c6_info[127].fileTimeHi = 0U;
  c6_info[127].mFileTimeLo = 0U;
  c6_info[127].mFileTimeHi = 0U;
}

static void c6_c_info_helper(c6_ResolvedFunctionInfo c6_info[148])
{
  c6_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[128].name = "eml_index_plus";
  c6_info[128].dominantType = "double";
  c6_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[128].fileTimeLo = 1286818778U;
  c6_info[128].fileTimeHi = 0U;
  c6_info[128].mFileTimeLo = 0U;
  c6_info[128].mFileTimeHi = 0U;
  c6_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[129].name = "eml_index_plus";
  c6_info[129].dominantType = "coder.internal.indexInt";
  c6_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[129].fileTimeLo = 1286818778U;
  c6_info[129].fileTimeHi = 0U;
  c6_info[129].mFileTimeLo = 0U;
  c6_info[129].mFileTimeHi = 0U;
  c6_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c6_info[130].name = "eml_warning";
  c6_info[130].dominantType = "char";
  c6_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c6_info[130].fileTimeLo = 1286818802U;
  c6_info[130].fileTimeHi = 0U;
  c6_info[130].mFileTimeLo = 0U;
  c6_info[130].mFileTimeHi = 0U;
  c6_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c6_info[131].name = "eml_scalar_eg";
  c6_info[131].dominantType = "double";
  c6_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[131].fileTimeLo = 1286818796U;
  c6_info[131].fileTimeHi = 0U;
  c6_info[131].mFileTimeLo = 0U;
  c6_info[131].mFileTimeHi = 0U;
  c6_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c6_info[132].name = "eml_int_forloop_overflow_check";
  c6_info[132].dominantType = "";
  c6_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[132].fileTimeLo = 1346510340U;
  c6_info[132].fileTimeHi = 0U;
  c6_info[132].mFileTimeLo = 0U;
  c6_info[132].mFileTimeHi = 0U;
  c6_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c6_info[133].name = "eml_xtrsm";
  c6_info[133].dominantType = "char";
  c6_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c6_info[133].fileTimeLo = 1299076778U;
  c6_info[133].fileTimeHi = 0U;
  c6_info[133].mFileTimeLo = 0U;
  c6_info[133].mFileTimeHi = 0U;
  c6_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c6_info[134].name = "eml_blas_inline";
  c6_info[134].dominantType = "";
  c6_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[134].fileTimeLo = 1299076768U;
  c6_info[134].fileTimeHi = 0U;
  c6_info[134].mFileTimeLo = 0U;
  c6_info[134].mFileTimeHi = 0U;
  c6_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c6_info[135].name = "mtimes";
  c6_info[135].dominantType = "double";
  c6_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[135].fileTimeLo = 1289519692U;
  c6_info[135].fileTimeHi = 0U;
  c6_info[135].mFileTimeLo = 0U;
  c6_info[135].mFileTimeHi = 0U;
  c6_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c6_info[136].name = "eml_index_class";
  c6_info[136].dominantType = "";
  c6_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[136].fileTimeLo = 1323170578U;
  c6_info[136].fileTimeHi = 0U;
  c6_info[136].mFileTimeLo = 0U;
  c6_info[136].mFileTimeHi = 0U;
  c6_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c6_info[137].name = "eml_scalar_eg";
  c6_info[137].dominantType = "double";
  c6_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[137].fileTimeLo = 1286818796U;
  c6_info[137].fileTimeHi = 0U;
  c6_info[137].mFileTimeLo = 0U;
  c6_info[137].mFileTimeHi = 0U;
  c6_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c6_info[138].name = "eml_refblas_xtrsm";
  c6_info[138].dominantType = "char";
  c6_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[138].fileTimeLo = 1299076786U;
  c6_info[138].fileTimeHi = 0U;
  c6_info[138].mFileTimeLo = 0U;
  c6_info[138].mFileTimeHi = 0U;
  c6_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[139].name = "eml_scalar_eg";
  c6_info[139].dominantType = "double";
  c6_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[139].fileTimeLo = 1286818796U;
  c6_info[139].fileTimeHi = 0U;
  c6_info[139].mFileTimeLo = 0U;
  c6_info[139].mFileTimeHi = 0U;
  c6_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[140].name = "eml_index_minus";
  c6_info[140].dominantType = "double";
  c6_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[140].fileTimeLo = 1286818778U;
  c6_info[140].fileTimeHi = 0U;
  c6_info[140].mFileTimeLo = 0U;
  c6_info[140].mFileTimeHi = 0U;
  c6_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[141].name = "eml_index_class";
  c6_info[141].dominantType = "";
  c6_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[141].fileTimeLo = 1323170578U;
  c6_info[141].fileTimeHi = 0U;
  c6_info[141].mFileTimeLo = 0U;
  c6_info[141].mFileTimeHi = 0U;
  c6_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[142].name = "eml_int_forloop_overflow_check";
  c6_info[142].dominantType = "";
  c6_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[142].fileTimeLo = 1346510340U;
  c6_info[142].fileTimeHi = 0U;
  c6_info[142].mFileTimeLo = 0U;
  c6_info[142].mFileTimeHi = 0U;
  c6_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[143].name = "eml_index_times";
  c6_info[143].dominantType = "coder.internal.indexInt";
  c6_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[143].fileTimeLo = 1286818780U;
  c6_info[143].fileTimeHi = 0U;
  c6_info[143].mFileTimeLo = 0U;
  c6_info[143].mFileTimeHi = 0U;
  c6_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[144].name = "eml_index_plus";
  c6_info[144].dominantType = "coder.internal.indexInt";
  c6_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[144].fileTimeLo = 1286818778U;
  c6_info[144].fileTimeHi = 0U;
  c6_info[144].mFileTimeLo = 0U;
  c6_info[144].mFileTimeHi = 0U;
  c6_info[145].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[145].name = "eml_index_plus";
  c6_info[145].dominantType = "double";
  c6_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[145].fileTimeLo = 1286818778U;
  c6_info[145].fileTimeHi = 0U;
  c6_info[145].mFileTimeLo = 0U;
  c6_info[145].mFileTimeHi = 0U;
  c6_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c6_info[146].name = "intmin";
  c6_info[146].dominantType = "char";
  c6_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c6_info[146].fileTimeLo = 1311255318U;
  c6_info[146].fileTimeHi = 0U;
  c6_info[146].mFileTimeLo = 0U;
  c6_info[146].mFileTimeHi = 0U;
  c6_info[147].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[147].name = "eml_div";
  c6_info[147].dominantType = "double";
  c6_info[147].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[147].fileTimeLo = 1313347810U;
  c6_info[147].fileTimeHi = 0U;
  c6_info[147].mFileTimeLo = 0U;
  c6_info[147].mFileTimeHi = 0U;
}

static void c6_eye(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
                   real_T c6_I[16])
{
  int32_T c6_i185;
  int32_T c6_i;
  int32_T c6_b_i;
  for (c6_i185 = 0; c6_i185 < 16; c6_i185++) {
    c6_I[c6_i185] = 0.0;
  }

  for (c6_i = 1; c6_i < 5; c6_i++) {
    c6_b_i = c6_i;
    c6_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 4, 2, 0) - 1) <<
           2)) - 1] = 1.0;
  }
}

static void c6_eml_scalar_eg(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c6_realmin(SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c6_eml_eps(SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static boolean_T c6_any(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c6_x[2])
{
  boolean_T c6_y;
  int32_T c6_k;
  real_T c6_b_k;
  real_T c6_b_x;
  boolean_T c6_b;
  boolean_T c6_b1;
  boolean_T guard1 = FALSE;
  boolean_T exitg1;
  c6_y = FALSE;
  c6_k = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c6_k < 2)) {
    c6_b_k = 1.0 + (real_T)c6_k;
    guard1 = FALSE;
    if (c6_x[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
         _SFD_INTEGER_CHECK("", c6_b_k), 1, 2, 1, 0) - 1] == 0.0) {
      guard1 = TRUE;
    } else {
      c6_b_x = c6_x[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c6_b_k), 1, 2, 1, 0) - 1];
      c6_b = muDoubleScalarIsNaN(c6_b_x);
      if (c6_b) {
        guard1 = TRUE;
      } else {
        c6_b1 = FALSE;
      }
    }

    if (guard1 == TRUE) {
      c6_b1 = TRUE;
    }

    if (!c6_b1) {
      c6_y = TRUE;
      exitg1 = TRUE;
    } else {
      c6_k++;
    }
  }

  return c6_y;
}

static void c6_mrdivide(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c6_A[16], real_T c6_B[16], real_T c6_y[16])
{
  int32_T c6_i186;
  int32_T c6_i187;
  int32_T c6_i188;
  int32_T c6_i189;
  real_T c6_b_A[16];
  int32_T c6_i190;
  int32_T c6_i191;
  int32_T c6_i192;
  int32_T c6_i193;
  real_T c6_b_B[16];
  int32_T c6_info;
  int32_T c6_ipiv[4];
  int32_T c6_b_info;
  int32_T c6_c_info;
  int32_T c6_d_info;
  int32_T c6_i;
  int32_T c6_b_i;
  int32_T c6_ip;
  int32_T c6_j;
  int32_T c6_b_j;
  real_T c6_temp;
  int32_T c6_i194;
  real_T c6_c_A[16];
  int32_T c6_i195;
  real_T c6_d_A[16];
  int32_T c6_i196;
  int32_T c6_i197;
  int32_T c6_i198;
  int32_T c6_i199;
  c6_i186 = 0;
  for (c6_i187 = 0; c6_i187 < 4; c6_i187++) {
    c6_i188 = 0;
    for (c6_i189 = 0; c6_i189 < 4; c6_i189++) {
      c6_b_A[c6_i189 + c6_i186] = c6_B[c6_i188 + c6_i187];
      c6_i188 += 4;
    }

    c6_i186 += 4;
  }

  c6_i190 = 0;
  for (c6_i191 = 0; c6_i191 < 4; c6_i191++) {
    c6_i192 = 0;
    for (c6_i193 = 0; c6_i193 < 4; c6_i193++) {
      c6_b_B[c6_i193 + c6_i190] = c6_A[c6_i192 + c6_i191];
      c6_i192 += 4;
    }

    c6_i190 += 4;
  }

  c6_b_eml_matlab_zgetrf(chartInstance, c6_b_A, c6_ipiv, &c6_info);
  c6_b_info = c6_info;
  c6_c_info = c6_b_info;
  c6_d_info = c6_c_info;
  if (c6_d_info > 0) {
    c6_eml_warning(chartInstance);
  }

  c6_eml_scalar_eg(chartInstance);
  for (c6_i = 1; c6_i < 5; c6_i++) {
    c6_b_i = c6_i;
    if (c6_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_i), 1, 4, 1, 0) - 1] != c6_b_i) {
      c6_ip = c6_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 4, 1, 0) - 1];
      for (c6_j = 1; c6_j < 5; c6_j++) {
        c6_b_j = c6_j;
        c6_temp = c6_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 4, 1, 0) +
                          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c6_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c6_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                   "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 4, 2,
                   0) - 1) << 2)) - 1] = c6_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_ip), 1, 4, 1, 0) +
          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c6_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c6_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
                   (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 4, 2, 0)
                  - 1) << 2)) - 1] = c6_temp;
      }
    }
  }

  for (c6_i194 = 0; c6_i194 < 16; c6_i194++) {
    c6_c_A[c6_i194] = c6_b_A[c6_i194];
  }

  c6_c_eml_xtrsm(chartInstance, c6_c_A, c6_b_B);
  for (c6_i195 = 0; c6_i195 < 16; c6_i195++) {
    c6_d_A[c6_i195] = c6_b_A[c6_i195];
  }

  c6_d_eml_xtrsm(chartInstance, c6_d_A, c6_b_B);
  c6_i196 = 0;
  for (c6_i197 = 0; c6_i197 < 4; c6_i197++) {
    c6_i198 = 0;
    for (c6_i199 = 0; c6_i199 < 4; c6_i199++) {
      c6_y[c6_i199 + c6_i196] = c6_b_B[c6_i198 + c6_i197];
      c6_i198 += 4;
    }

    c6_i196 += 4;
  }
}

static void c6_eps(SFc6_torqueBalancing2012bInstanceStruct *chartInstance)
{
  c6_eml_eps(chartInstance);
}

static void c6_eml_matlab_zgetrf(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], real_T c6_b_A[16], int32_T c6_ipiv[4],
  int32_T *c6_info)
{
  int32_T c6_i200;
  for (c6_i200 = 0; c6_i200 < 16; c6_i200++) {
    c6_b_A[c6_i200] = c6_A[c6_i200];
  }

  c6_b_eml_matlab_zgetrf(chartInstance, c6_b_A, c6_ipiv, c6_info);
}

static void c6_check_forloop_overflow_error
  (SFc6_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c6_overflow)
{
  int32_T c6_i201;
  static char_T c6_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c6_u[34];
  const mxArray *c6_y = NULL;
  int32_T c6_i202;
  static char_T c6_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c6_b_u[23];
  const mxArray *c6_b_y = NULL;
  if (!c6_overflow) {
  } else {
    for (c6_i201 = 0; c6_i201 < 34; c6_i201++) {
      c6_u[c6_i201] = c6_cv0[c6_i201];
    }

    c6_y = NULL;
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c6_i202 = 0; c6_i202 < 23; c6_i202++) {
      c6_b_u[c6_i202] = c6_cv1[c6_i202];
    }

    c6_b_y = NULL;
    sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c6_y, 14, c6_b_y));
  }
}

static void c6_eml_xger(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c6_m, int32_T c6_n, real_T c6_alpha1, int32_T c6_ix0, int32_T c6_iy0,
  real_T c6_A[16], int32_T c6_ia0, real_T c6_b_A[16])
{
  int32_T c6_i203;
  for (c6_i203 = 0; c6_i203 < 16; c6_i203++) {
    c6_b_A[c6_i203] = c6_A[c6_i203];
  }

  c6_b_eml_xger(chartInstance, c6_m, c6_n, c6_alpha1, c6_ix0, c6_iy0, c6_b_A,
                c6_ia0);
}

static void c6_eml_warning(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c6_i204;
  static char_T c6_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c6_u[27];
  const mxArray *c6_y = NULL;
  for (c6_i204 = 0; c6_i204 < 27; c6_i204++) {
    c6_u[c6_i204] = c6_varargin_1[c6_i204];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c6_y));
}

static void c6_eml_xtrsm(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c6_A[16], real_T c6_B[16], real_T c6_b_B[16])
{
  int32_T c6_i205;
  int32_T c6_i206;
  real_T c6_b_A[16];
  for (c6_i205 = 0; c6_i205 < 16; c6_i205++) {
    c6_b_B[c6_i205] = c6_B[c6_i205];
  }

  for (c6_i206 = 0; c6_i206 < 16; c6_i206++) {
    c6_b_A[c6_i206] = c6_A[c6_i206];
  }

  c6_c_eml_xtrsm(chartInstance, c6_b_A, c6_b_B);
}

static void c6_below_threshold(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c6_b_eml_scalar_eg(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c6_b_eml_xtrsm(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], real_T c6_B[16], real_T c6_b_B[16])
{
  int32_T c6_i207;
  int32_T c6_i208;
  real_T c6_b_A[16];
  for (c6_i207 = 0; c6_i207 < 16; c6_i207++) {
    c6_b_B[c6_i207] = c6_B[c6_i207];
  }

  for (c6_i208 = 0; c6_i208 < 16; c6_i208++) {
    c6_b_A[c6_i208] = c6_A[c6_i208];
  }

  c6_d_eml_xtrsm(chartInstance, c6_b_A, c6_b_B);
}

static const mxArray *c6_o_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static int32_T c6_kb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i209;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i209, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i209;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_kb_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_lb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_torqueBalancing2012b, const
  char_T *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_mb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_torqueBalancing2012b), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_torqueBalancing2012b);
  return c6_y;
}

static uint8_T c6_mb_emlrt_marshallIn(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_b_eml_matlab_zgetrf(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], int32_T c6_ipiv[4], int32_T *c6_info)
{
  int32_T c6_i210;
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_a;
  int32_T c6_jm1;
  int32_T c6_b;
  int32_T c6_mmj;
  int32_T c6_b_a;
  int32_T c6_c;
  int32_T c6_b_b;
  int32_T c6_jj;
  int32_T c6_c_a;
  int32_T c6_jp1j;
  int32_T c6_d_a;
  int32_T c6_b_c;
  int32_T c6_n;
  int32_T c6_ix0;
  int32_T c6_b_n;
  int32_T c6_b_ix0;
  int32_T c6_c_n;
  int32_T c6_c_ix0;
  int32_T c6_idxmax;
  int32_T c6_ix;
  real_T c6_x;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_y;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_b_y;
  real_T c6_smax;
  int32_T c6_d_n;
  int32_T c6_c_b;
  int32_T c6_d_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_b_k;
  int32_T c6_e_a;
  real_T c6_f_x;
  real_T c6_g_x;
  real_T c6_h_x;
  real_T c6_c_y;
  real_T c6_i_x;
  real_T c6_j_x;
  real_T c6_d_y;
  real_T c6_s;
  int32_T c6_f_a;
  int32_T c6_jpiv_offset;
  int32_T c6_g_a;
  int32_T c6_e_b;
  int32_T c6_jpiv;
  int32_T c6_h_a;
  int32_T c6_f_b;
  int32_T c6_c_c;
  int32_T c6_g_b;
  int32_T c6_jrow;
  int32_T c6_i_a;
  int32_T c6_h_b;
  int32_T c6_jprow;
  int32_T c6_d_ix0;
  int32_T c6_iy0;
  int32_T c6_e_ix0;
  int32_T c6_b_iy0;
  int32_T c6_f_ix0;
  int32_T c6_c_iy0;
  int32_T c6_b_ix;
  int32_T c6_iy;
  int32_T c6_c_k;
  real_T c6_temp;
  int32_T c6_j_a;
  int32_T c6_k_a;
  int32_T c6_b_jp1j;
  int32_T c6_l_a;
  int32_T c6_d_c;
  int32_T c6_m_a;
  int32_T c6_i_b;
  int32_T c6_i211;
  int32_T c6_n_a;
  int32_T c6_j_b;
  int32_T c6_o_a;
  int32_T c6_k_b;
  boolean_T c6_b_overflow;
  int32_T c6_i;
  int32_T c6_b_i;
  real_T c6_k_x;
  real_T c6_e_y;
  real_T c6_z;
  int32_T c6_l_b;
  int32_T c6_e_c;
  int32_T c6_p_a;
  int32_T c6_f_c;
  int32_T c6_q_a;
  int32_T c6_g_c;
  int32_T c6_m;
  int32_T c6_e_n;
  int32_T c6_g_ix0;
  int32_T c6_d_iy0;
  int32_T c6_ia0;
  real_T c6_d3;
  c6_realmin(chartInstance);
  c6_eps(chartInstance);
  for (c6_i210 = 0; c6_i210 < 4; c6_i210++) {
    c6_ipiv[c6_i210] = 1 + c6_i210;
  }

  *c6_info = 0;
  for (c6_j = 1; c6_j < 4; c6_j++) {
    c6_b_j = c6_j;
    c6_a = c6_b_j - 1;
    c6_jm1 = c6_a;
    c6_b = c6_b_j;
    c6_mmj = 4 - c6_b;
    c6_b_a = c6_jm1;
    c6_c = c6_b_a * 5;
    c6_b_b = c6_c + 1;
    c6_jj = c6_b_b;
    c6_c_a = c6_jj + 1;
    c6_jp1j = c6_c_a;
    c6_d_a = c6_mmj;
    c6_b_c = c6_d_a;
    c6_n = c6_b_c + 1;
    c6_ix0 = c6_jj;
    c6_b_n = c6_n;
    c6_b_ix0 = c6_ix0;
    c6_c_n = c6_b_n;
    c6_c_ix0 = c6_b_ix0;
    if (c6_c_n < 1) {
      c6_idxmax = 0;
    } else {
      c6_idxmax = 1;
      if (c6_c_n > 1) {
        c6_ix = c6_c_ix0;
        c6_x = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_ix), 1, 16, 1, 0) - 1];
        c6_b_x = c6_x;
        c6_c_x = c6_b_x;
        c6_y = muDoubleScalarAbs(c6_c_x);
        c6_d_x = 0.0;
        c6_e_x = c6_d_x;
        c6_b_y = muDoubleScalarAbs(c6_e_x);
        c6_smax = c6_y + c6_b_y;
        c6_d_n = c6_c_n;
        c6_c_b = c6_d_n;
        c6_d_b = c6_c_b;
        if (2 > c6_d_b) {
          c6_overflow = FALSE;
        } else {
          c6_overflow = (c6_d_b > 2147483646);
        }

        if (c6_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_overflow);
        }

        for (c6_k = 2; c6_k <= c6_d_n; c6_k++) {
          c6_b_k = c6_k;
          c6_e_a = c6_ix + 1;
          c6_ix = c6_e_a;
          c6_f_x = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_ix), 1, 16, 1, 0) - 1];
          c6_g_x = c6_f_x;
          c6_h_x = c6_g_x;
          c6_c_y = muDoubleScalarAbs(c6_h_x);
          c6_i_x = 0.0;
          c6_j_x = c6_i_x;
          c6_d_y = muDoubleScalarAbs(c6_j_x);
          c6_s = c6_c_y + c6_d_y;
          if (c6_s > c6_smax) {
            c6_idxmax = c6_b_k;
            c6_smax = c6_s;
          }
        }
      }
    }

    c6_f_a = c6_idxmax - 1;
    c6_jpiv_offset = c6_f_a;
    c6_g_a = c6_jj;
    c6_e_b = c6_jpiv_offset;
    c6_jpiv = c6_g_a + c6_e_b;
    if (c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if (c6_jpiv_offset != 0) {
        c6_h_a = c6_b_j;
        c6_f_b = c6_jpiv_offset;
        c6_c_c = c6_h_a + c6_f_b;
        c6_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_j), 1, 4, 1, 0) - 1] = c6_c_c;
        c6_g_b = c6_jm1 + 1;
        c6_jrow = c6_g_b;
        c6_i_a = c6_jrow;
        c6_h_b = c6_jpiv_offset;
        c6_jprow = c6_i_a + c6_h_b;
        c6_d_ix0 = c6_jrow;
        c6_iy0 = c6_jprow;
        c6_e_ix0 = c6_d_ix0;
        c6_b_iy0 = c6_iy0;
        c6_f_ix0 = c6_e_ix0;
        c6_c_iy0 = c6_b_iy0;
        c6_b_ix = c6_f_ix0;
        c6_iy = c6_c_iy0;
        for (c6_c_k = 1; c6_c_k < 5; c6_c_k++) {
          c6_temp = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_ix), 1, 16, 1, 0) - 1];
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ix), 1, 16, 1, 0) - 1] =
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_iy), 1, 16, 1, 0) - 1];
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_iy), 1, 16, 1, 0) - 1] = c6_temp;
          c6_j_a = c6_b_ix + 4;
          c6_b_ix = c6_j_a;
          c6_k_a = c6_iy + 4;
          c6_iy = c6_k_a;
        }
      }

      c6_b_jp1j = c6_jp1j;
      c6_l_a = c6_mmj;
      c6_d_c = c6_l_a;
      c6_m_a = c6_jp1j;
      c6_i_b = c6_d_c - 1;
      c6_i211 = c6_m_a + c6_i_b;
      c6_n_a = c6_b_jp1j;
      c6_j_b = c6_i211;
      c6_o_a = c6_n_a;
      c6_k_b = c6_j_b;
      if (c6_o_a > c6_k_b) {
        c6_b_overflow = FALSE;
      } else {
        c6_b_overflow = (c6_k_b > 2147483646);
      }

      if (c6_b_overflow) {
        c6_check_forloop_overflow_error(chartInstance, c6_b_overflow);
      }

      for (c6_i = c6_b_jp1j; c6_i <= c6_i211; c6_i++) {
        c6_b_i = c6_i;
        c6_k_x = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 16, 1, 0) - 1];
        c6_e_y = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_jj), 1, 16, 1, 0) - 1];
        c6_z = c6_k_x / c6_e_y;
        c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_i), 1, 16, 1, 0) - 1] = c6_z;
      }
    } else {
      *c6_info = c6_b_j;
    }

    c6_l_b = c6_b_j;
    c6_e_c = 4 - c6_l_b;
    c6_p_a = c6_jj;
    c6_f_c = c6_p_a;
    c6_q_a = c6_jj;
    c6_g_c = c6_q_a;
    c6_m = c6_mmj;
    c6_e_n = c6_e_c;
    c6_g_ix0 = c6_jp1j;
    c6_d_iy0 = c6_f_c + 4;
    c6_ia0 = c6_g_c + 5;
    c6_d3 = -1.0;
    c6_b_eml_xger(chartInstance, c6_m, c6_e_n, c6_d3, c6_g_ix0, c6_d_iy0, c6_A,
                  c6_ia0);
  }

  if (*c6_info == 0) {
    if (!(c6_A[15] != 0.0)) {
      *c6_info = 4;
    }
  }
}

static void c6_b_eml_xger(SFc6_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c6_m, int32_T c6_n, real_T c6_alpha1, int32_T c6_ix0, int32_T c6_iy0,
  real_T c6_A[16], int32_T c6_ia0)
{
  int32_T c6_b_m;
  int32_T c6_b_n;
  real_T c6_b_alpha1;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_b_ia0;
  int32_T c6_c_m;
  int32_T c6_c_n;
  real_T c6_c_alpha1;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_c_ia0;
  int32_T c6_d_m;
  int32_T c6_d_n;
  real_T c6_d_alpha1;
  int32_T c6_d_ix0;
  int32_T c6_d_iy0;
  int32_T c6_d_ia0;
  int32_T c6_ixstart;
  int32_T c6_a;
  int32_T c6_jA;
  int32_T c6_jy;
  int32_T c6_e_n;
  int32_T c6_b;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_j;
  real_T c6_yjy;
  real_T c6_temp;
  int32_T c6_ix;
  int32_T c6_c_b;
  int32_T c6_i212;
  int32_T c6_b_a;
  int32_T c6_d_b;
  int32_T c6_i213;
  int32_T c6_c_a;
  int32_T c6_e_b;
  int32_T c6_d_a;
  int32_T c6_f_b;
  boolean_T c6_b_overflow;
  int32_T c6_ijA;
  int32_T c6_b_ijA;
  int32_T c6_e_a;
  int32_T c6_f_a;
  int32_T c6_g_a;
  c6_b_m = c6_m;
  c6_b_n = c6_n;
  c6_b_alpha1 = c6_alpha1;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_b_ia0 = c6_ia0;
  c6_c_m = c6_b_m;
  c6_c_n = c6_b_n;
  c6_c_alpha1 = c6_b_alpha1;
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  c6_c_ia0 = c6_b_ia0;
  c6_d_m = c6_c_m;
  c6_d_n = c6_c_n;
  c6_d_alpha1 = c6_c_alpha1;
  c6_d_ix0 = c6_c_ix0;
  c6_d_iy0 = c6_c_iy0;
  c6_d_ia0 = c6_c_ia0;
  if (c6_d_alpha1 == 0.0) {
  } else {
    c6_ixstart = c6_d_ix0;
    c6_a = c6_d_ia0 - 1;
    c6_jA = c6_a;
    c6_jy = c6_d_iy0;
    c6_e_n = c6_d_n;
    c6_b = c6_e_n;
    c6_b_b = c6_b;
    if (1 > c6_b_b) {
      c6_overflow = FALSE;
    } else {
      c6_overflow = (c6_b_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_j = 1; c6_j <= c6_e_n; c6_j++) {
      c6_yjy = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_jy), 1, 16, 1, 0) - 1];
      if (c6_yjy != 0.0) {
        c6_temp = c6_yjy * c6_d_alpha1;
        c6_ix = c6_ixstart;
        c6_c_b = c6_jA + 1;
        c6_i212 = c6_c_b;
        c6_b_a = c6_d_m;
        c6_d_b = c6_jA;
        c6_i213 = c6_b_a + c6_d_b;
        c6_c_a = c6_i212;
        c6_e_b = c6_i213;
        c6_d_a = c6_c_a;
        c6_f_b = c6_e_b;
        if (c6_d_a > c6_f_b) {
          c6_b_overflow = FALSE;
        } else {
          c6_b_overflow = (c6_f_b > 2147483646);
        }

        if (c6_b_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_b_overflow);
        }

        for (c6_ijA = c6_i212; c6_ijA <= c6_i213; c6_ijA++) {
          c6_b_ijA = c6_ijA;
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ijA), 1, 16, 1, 0) - 1] =
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ijA), 1, 16, 1, 0) - 1] +
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_ix), 1, 16, 1, 0) - 1] * c6_temp;
          c6_e_a = c6_ix + 1;
          c6_ix = c6_e_a;
        }
      }

      c6_f_a = c6_jy + 4;
      c6_jy = c6_f_a;
      c6_g_a = c6_jA + 4;
      c6_jA = c6_g_a;
    }
  }
}

static void c6_c_eml_xtrsm(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], real_T c6_B[16])
{
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_a;
  int32_T c6_c;
  int32_T c6_b;
  int32_T c6_b_c;
  int32_T c6_b_b;
  int32_T c6_jBcol;
  int32_T c6_k;
  int32_T c6_b_k;
  int32_T c6_b_a;
  int32_T c6_c_c;
  int32_T c6_c_b;
  int32_T c6_d_c;
  int32_T c6_d_b;
  int32_T c6_kAcol;
  int32_T c6_c_a;
  int32_T c6_e_b;
  int32_T c6_e_c;
  int32_T c6_d_a;
  int32_T c6_i214;
  boolean_T c6_overflow;
  int32_T c6_i;
  int32_T c6_b_i;
  int32_T c6_e_a;
  int32_T c6_f_b;
  int32_T c6_f_c;
  int32_T c6_f_a;
  int32_T c6_g_b;
  int32_T c6_g_c;
  int32_T c6_g_a;
  int32_T c6_h_b;
  int32_T c6_h_c;
  int32_T c6_h_a;
  int32_T c6_i_b;
  int32_T c6_i_c;
  c6_below_threshold(chartInstance);
  c6_b_eml_scalar_eg(chartInstance);
  for (c6_j = 1; c6_j < 5; c6_j++) {
    c6_b_j = c6_j;
    c6_a = c6_b_j;
    c6_c = c6_a;
    c6_b = c6_c - 1;
    c6_b_c = c6_b << 2;
    c6_b_b = c6_b_c;
    c6_jBcol = c6_b_b;
    for (c6_k = 1; c6_k < 5; c6_k++) {
      c6_b_k = c6_k;
      c6_b_a = c6_b_k;
      c6_c_c = c6_b_a;
      c6_c_b = c6_c_c - 1;
      c6_d_c = c6_c_b << 2;
      c6_d_b = c6_d_c;
      c6_kAcol = c6_d_b;
      c6_c_a = c6_b_k;
      c6_e_b = c6_jBcol;
      c6_e_c = c6_c_a + c6_e_b;
      if (c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c6_d_a = c6_b_k;
        c6_i214 = c6_d_a;
        c6_overflow = FALSE;
        if (c6_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_overflow);
        }

        for (c6_i = c6_i214 + 1; c6_i < 5; c6_i++) {
          c6_b_i = c6_i;
          c6_e_a = c6_b_i;
          c6_f_b = c6_jBcol;
          c6_f_c = c6_e_a + c6_f_b;
          c6_f_a = c6_b_i;
          c6_g_b = c6_jBcol;
          c6_g_c = c6_f_a + c6_g_b;
          c6_g_a = c6_b_k;
          c6_h_b = c6_jBcol;
          c6_h_c = c6_g_a + c6_h_b;
          c6_h_a = c6_b_i;
          c6_i_b = c6_kAcol;
          c6_i_c = c6_h_a + c6_i_b;
          c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_f_c), 1, 16, 1, 0) - 1] =
            c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_g_c), 1, 16, 1, 0) - 1] -
            c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_h_c), 1, 16, 1, 0) - 1] *
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_i_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void c6_d_eml_xtrsm(SFc6_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c6_A[16], real_T c6_B[16])
{
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_a;
  int32_T c6_c;
  int32_T c6_b;
  int32_T c6_b_c;
  int32_T c6_b_b;
  int32_T c6_jBcol;
  int32_T c6_k;
  int32_T c6_b_k;
  int32_T c6_b_a;
  int32_T c6_c_c;
  int32_T c6_c_b;
  int32_T c6_d_c;
  int32_T c6_d_b;
  int32_T c6_kAcol;
  int32_T c6_c_a;
  int32_T c6_e_b;
  int32_T c6_e_c;
  int32_T c6_d_a;
  int32_T c6_f_b;
  int32_T c6_f_c;
  int32_T c6_e_a;
  int32_T c6_g_b;
  int32_T c6_g_c;
  int32_T c6_f_a;
  int32_T c6_h_b;
  int32_T c6_h_c;
  real_T c6_x;
  real_T c6_y;
  real_T c6_z;
  int32_T c6_g_a;
  int32_T c6_i215;
  int32_T c6_i_b;
  int32_T c6_j_b;
  boolean_T c6_overflow;
  int32_T c6_i;
  int32_T c6_b_i;
  int32_T c6_h_a;
  int32_T c6_k_b;
  int32_T c6_i_c;
  int32_T c6_i_a;
  int32_T c6_l_b;
  int32_T c6_j_c;
  int32_T c6_j_a;
  int32_T c6_m_b;
  int32_T c6_k_c;
  int32_T c6_k_a;
  int32_T c6_n_b;
  int32_T c6_l_c;
  c6_below_threshold(chartInstance);
  c6_b_eml_scalar_eg(chartInstance);
  for (c6_j = 1; c6_j < 5; c6_j++) {
    c6_b_j = c6_j;
    c6_a = c6_b_j;
    c6_c = c6_a;
    c6_b = c6_c - 1;
    c6_b_c = c6_b << 2;
    c6_b_b = c6_b_c;
    c6_jBcol = c6_b_b;
    for (c6_k = 4; c6_k > 0; c6_k--) {
      c6_b_k = c6_k;
      c6_b_a = c6_b_k;
      c6_c_c = c6_b_a;
      c6_c_b = c6_c_c - 1;
      c6_d_c = c6_c_b << 2;
      c6_d_b = c6_d_c;
      c6_kAcol = c6_d_b;
      c6_c_a = c6_b_k;
      c6_e_b = c6_jBcol;
      c6_e_c = c6_c_a + c6_e_b;
      if (c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c6_d_a = c6_b_k;
        c6_f_b = c6_jBcol;
        c6_f_c = c6_d_a + c6_f_b;
        c6_e_a = c6_b_k;
        c6_g_b = c6_jBcol;
        c6_g_c = c6_e_a + c6_g_b;
        c6_f_a = c6_b_k;
        c6_h_b = c6_kAcol;
        c6_h_c = c6_f_a + c6_h_b;
        c6_x = c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_g_c), 1, 16, 1, 0) - 1];
        c6_y = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_h_c), 1, 16, 1, 0) - 1];
        c6_z = c6_x / c6_y;
        c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_f_c), 1, 16, 1, 0) - 1] = c6_z;
        c6_g_a = c6_b_k - 1;
        c6_i215 = c6_g_a;
        c6_i_b = c6_i215;
        c6_j_b = c6_i_b;
        if (1 > c6_j_b) {
          c6_overflow = FALSE;
        } else {
          c6_overflow = (c6_j_b > 2147483646);
        }

        if (c6_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_overflow);
        }

        for (c6_i = 1; c6_i <= c6_i215; c6_i++) {
          c6_b_i = c6_i;
          c6_h_a = c6_b_i;
          c6_k_b = c6_jBcol;
          c6_i_c = c6_h_a + c6_k_b;
          c6_i_a = c6_b_i;
          c6_l_b = c6_jBcol;
          c6_j_c = c6_i_a + c6_l_b;
          c6_j_a = c6_b_k;
          c6_m_b = c6_jBcol;
          c6_k_c = c6_j_a + c6_m_b;
          c6_k_a = c6_b_i;
          c6_n_b = c6_kAcol;
          c6_l_c = c6_k_a + c6_n_b;
          c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_i_c), 1, 16, 1, 0) - 1] =
            c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_j_c), 1, 16, 1, 0) - 1] -
            c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_k_c), 1, 16, 1, 0) - 1] *
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_l_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void init_dsm_address_info(SFc6_torqueBalancing2012bInstanceStruct
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

void sf_c6_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(103355317U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2777125867U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1402386611U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1762082695U);
}

mxArray *sf_c6_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("n94yI7eaWsMM62Yc6nXnKH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,11,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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
      pr[0] = (double)(6);
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
      pr[0] = (double)(3);
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
      pr[0] = (double)(23);
      pr[1] = (double)(1);
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
      pr[0] = (double)(3);
      pr[1] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
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
      pr[0] = (double)(3);
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
      pr[0] = (double)(1);
      pr[1] = (double)(23);
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
      pr[0] = (double)(1);
      pr[1] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c6_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c6_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[52],T\"CoMDes\",},{M[1],M[43],T\"constraints\",},{M[1],M[68],T\"currentState\",},{M[1],M[66],T\"impedances\",},{M[1],M[46],T\"qDes\",},{M[1],M[71],T\"w_H_b\",},{M[4],M[0],T\"fixedLink\",S'l','i','p'{{M1x2[303 312],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[281 286],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m\"}}},{M[4],M[0],T\"w_H_fixedLink\",S'l','i','p'{{M1x2[329 342],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineWalking.m\"}}},{M[8],M[0],T\"is_active_c6_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 10, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           6,
           1,
           1,
           19,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"connection");
          _SFD_SET_DATA_PROPS(1,1,1,0,"wrench_rightFoot");
          _SFD_SET_DATA_PROPS(2,1,1,0,"wrench_leftFoot");
          _SFD_SET_DATA_PROPS(3,1,1,0,"CoM_0");
          _SFD_SET_DATA_PROPS(4,1,1,0,"q0");
          _SFD_SET_DATA_PROPS(5,1,1,0,"w_CoM");
          _SFD_SET_DATA_PROPS(6,1,1,0,"CoMIn");
          _SFD_SET_DATA_PROPS(7,1,1,0,"qIn");
          _SFD_SET_DATA_PROPS(8,1,1,0,"constraintsIn");
          _SFD_SET_DATA_PROPS(9,2,0,1,"w_H_b");
          _SFD_SET_DATA_PROPS(10,2,0,1,"CoMDes");
          _SFD_SET_DATA_PROPS(11,2,0,1,"qDes");
          _SFD_SET_DATA_PROPS(12,2,0,1,"constraints");
          _SFD_SET_DATA_PROPS(13,1,1,0,"l_sole_H_b");
          _SFD_SET_DATA_PROPS(14,1,1,0,"r_sole_H_b");
          _SFD_SET_DATA_PROPS(15,10,0,0,"sm");
          _SFD_SET_DATA_PROPS(16,2,0,1,"impedances");
          _SFD_SET_DATA_PROPS(17,10,0,0,"gain");
          _SFD_SET_DATA_PROPS(18,2,0,1,"currentState");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,439);
        _SFD_CV_INIT_SCRIPT(0,1,11,0,0,0,0,0,9,4);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"stateMachineWalking",0,-1,2641);
        _SFD_CV_INIT_SCRIPT_IF(0,0,353,418,-1,523);
        _SFD_CV_INIT_SCRIPT_IF(0,1,904,921,973,1033);
        _SFD_CV_INIT_SCRIPT_IF(0,2,1043,1056,1232,1249);
        _SFD_CV_INIT_SCRIPT_IF(0,3,1141,1183,-1,1218);
        _SFD_CV_INIT_SCRIPT_IF(0,4,1232,1249,2012,2598);
        _SFD_CV_INIT_SCRIPT_IF(0,5,1409,1500,1693,1788);
        _SFD_CV_INIT_SCRIPT_IF(0,6,1693,1788,-1,1788);
        _SFD_CV_INIT_SCRIPT_IF(0,7,2012,2029,2304,2598);
        _SFD_CV_INIT_SCRIPT_IF(0,8,2202,2255,-1,2290);
        _SFD_CV_INIT_SCRIPT_IF(0,9,2304,2321,-1,2321);
        _SFD_CV_INIT_SCRIPT_IF(0,10,2503,2555,-1,2590);

        {
          static int condStart[] = { 356, 374, 396 };

          static int condEnd[] = { 370, 392, 418 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,0,356,418,3,0,&(condStart[0]),&(condEnd[0]),
            5,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1144, 1173 };

          static int condEnd[] = { 1169, 1183 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,1,1144,1183,2,3,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1413, 1464 };

          static int condEnd[] = { 1440, 1500 };

          static int pfixExpr[] = { 0, -1, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,2,1412,1500,2,5,&(condStart[0]),&(condEnd[0]),
            4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1701, 1752 };

          static int condEnd[] = { 1728, 1788 };

          static int pfixExpr[] = { 0, -1, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,3,1700,1788,2,7,&(condStart[0]),&(condEnd[0]),
            4,&(pfixExpr[0]));
        }

        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_k_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_j_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_j_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_i_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_f_sf_marshallOut,(MexInFcnForType)
            c6_f_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)
            c6_e_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)
            c6_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)
            c6_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(15,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_h_sf_marshallOut,(MexInFcnForType)c6_h_sf_marshallIn);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)
            c6_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(17,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_g_sf_marshallOut,(MexInFcnForType)c6_g_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)c6_sf_marshallIn);

        {
          boolean_T *c6_connection;
          real_T *c6_currentState;
          real_T (*c6_wrench_rightFoot)[6];
          real_T (*c6_wrench_leftFoot)[6];
          real_T (*c6_CoM_0)[3];
          real_T (*c6_q0)[23];
          real_T (*c6_w_CoM)[3];
          real_T (*c6_CoMIn)[3];
          real_T (*c6_qIn)[23];
          real_T (*c6_constraintsIn)[2];
          real_T (*c6_w_H_b)[16];
          real_T (*c6_CoMDes)[3];
          real_T (*c6_qDes)[23];
          real_T (*c6_constraints)[2];
          real_T (*c6_l_sole_H_b)[16];
          real_T (*c6_r_sole_H_b)[16];
          real_T (*c6_impedances)[23];
          c6_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
          c6_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S,
            5);
          c6_r_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            10);
          c6_l_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            9);
          c6_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S,
            4);
          c6_qDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
          c6_CoMDes = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
          c6_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
          c6_constraintsIn = (real_T (*)[2])ssGetInputPortSignal
            (chartInstance->S, 8);
          c6_qIn = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 7);
          c6_CoMIn = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 6);
          c6_w_CoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
          c6_q0 = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 4);
          c6_CoM_0 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
          c6_wrench_leftFoot = (real_T (*)[6])ssGetInputPortSignal
            (chartInstance->S, 2);
          c6_wrench_rightFoot = (real_T (*)[6])ssGetInputPortSignal
            (chartInstance->S, 1);
          c6_connection = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c6_connection);
          _SFD_SET_DATA_VALUE_PTR(1U, *c6_wrench_rightFoot);
          _SFD_SET_DATA_VALUE_PTR(2U, *c6_wrench_leftFoot);
          _SFD_SET_DATA_VALUE_PTR(3U, *c6_CoM_0);
          _SFD_SET_DATA_VALUE_PTR(4U, *c6_q0);
          _SFD_SET_DATA_VALUE_PTR(5U, *c6_w_CoM);
          _SFD_SET_DATA_VALUE_PTR(6U, *c6_CoMIn);
          _SFD_SET_DATA_VALUE_PTR(7U, *c6_qIn);
          _SFD_SET_DATA_VALUE_PTR(8U, *c6_constraintsIn);
          _SFD_SET_DATA_VALUE_PTR(9U, *c6_w_H_b);
          _SFD_SET_DATA_VALUE_PTR(10U, *c6_CoMDes);
          _SFD_SET_DATA_VALUE_PTR(11U, *c6_qDes);
          _SFD_SET_DATA_VALUE_PTR(12U, *c6_constraints);
          _SFD_SET_DATA_VALUE_PTR(13U, *c6_l_sole_H_b);
          _SFD_SET_DATA_VALUE_PTR(14U, *c6_r_sole_H_b);
          _SFD_SET_DATA_VALUE_PTR(15U, &chartInstance->c6_sm);
          _SFD_SET_DATA_VALUE_PTR(16U, *c6_impedances);
          _SFD_SET_DATA_VALUE_PTR(17U, &chartInstance->c6_gain);
          _SFD_SET_DATA_VALUE_PTR(18U, c6_currentState);
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
  return "5CEz2d8DT1mkEZ0bysJblC";
}

static void sf_opaque_initialize_c6_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_torqueBalancing2012b
    ((SFc6_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c6_torqueBalancing2012b((SFc6_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c6_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c6_torqueBalancing2012b((SFc6_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c6_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c6_torqueBalancing2012b((SFc6_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c6_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c6_torqueBalancing2012b((SFc6_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_torqueBalancing2012b
    ((SFc6_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c6_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_torqueBalancing2012b((SFc6_torqueBalancing2012bInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c6_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c6_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c6_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c6_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_torqueBalancing2012bInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c6_torqueBalancing2012b((SFc6_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_torqueBalancing2012b((SFc6_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c6_torqueBalancing2012b
      ((SFc6_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_torqueBalancing2012b(SimStruct *S)
{
  /* Actual parameters from chart:
     gain sm
   */
  const char_T *rtParamNames[] = { "gain", "sm" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0],
    sf_get_param_data_type_id(S,0));
  ssRegDlgParamAsRunTimeParam(S, 1, 1, rtParamNames[1],
    sf_get_param_data_type_id(S,1));
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,6,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,6);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 9, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 10, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,11);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,6);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=6; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 11; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3273014320U));
  ssSetChecksum1(S,(3568349280U));
  ssSetChecksum2(S,(3032996143U));
  ssSetChecksum3(S,(784195736U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_torqueBalancing2012b(SimStruct *S)
{
  SFc6_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc6_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc6_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c6_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c6_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_torqueBalancing2012b;
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

void c6_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void *
  data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
