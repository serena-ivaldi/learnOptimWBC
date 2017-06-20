/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c13_torqueBalancing2012b.h"
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
static const char * c13_debug_family_names[23] = { "nargin", "nargout",
  "wrench_rightFoot", "wrench_leftFoot", "CoM_0", "q0", "l_sole_CoM",
  "r_sole_CoM", "qj", "t", "l_sole_H_b", "r_sole_H_b", "sm", "gain", "w_H_b",
  "CoMDes", "qDes", "constraints", "impedances", "kpCom", "kdCom",
  "currentState", "jointsSmoothingTime" };

static const char * c13_b_debug_family_names[31] = { "fixed_link_CoMDes",
  "CoMError", "i", "qTildeRLeg", "qTildeLLeg", "nargin", "nargout", "CoM_0",
  "q0", "l_sole_CoM", "r_sole_CoM", "qj", "t", "wrench_rightFoot",
  "wrench_leftFoot", "l_sole_H_b", "r_sole_H_b", "sm", "gain", "w_H_b", "CoMDes",
  "qDes", "constraints", "impedances", "kpCom", "kdCom", "currentState",
  "jointsSmoothingTime", "state", "tSwitch", "w_H_fixedLink" };

/* Function Declarations */
static void initialize_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void c13_update_debugger_state_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c13_st);
static void finalize_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c13_torqueBalancing2012b(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void c13_stateMachine(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_CoM_0[3], real_T c13_q0[23], real_T c13_l_sole_CoM
  [3], real_T c13_r_sole_CoM[3], real_T c13_qj[23], real_T c13_t, real_T
  c13_wrench_rightFoot[6], real_T c13_wrench_leftFoot[6], real_T c13_l_sole_H_b
  [16], real_T c13_r_sole_H_b[16], c13_struct_rUGQ0INmvPpaxIctEGl5sE *c13_b_sm,
  c13_struct_kzTB0QQWoOlMoMhgKf6sK *c13_b_gain, real_T c13_w_H_b[16], real_T
  c13_CoMDes[3], real_T c13_qDes[23], real_T c13_constraints[2], real_T
  c13_impedances[23], real_T c13_kpCom[3], real_T c13_kdCom[3], real_T
  *c13_currentState, real_T *c13_jointsSmoothingTime);
static void init_script_number_translation(uint32_T c13_machineNumber, uint32_T
  c13_chartNumber);
static const mxArray *c13_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static real_T c13_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_jointsSmoothingTime, const char_T
  *c13_identifier);
static real_T c13_b_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId);
static void c13_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_b_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_c_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_kdCom, const char_T *c13_identifier, real_T
  c13_y[3]);
static void c13_d_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[3]);
static void c13_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_c_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_e_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_impedances, const char_T *c13_identifier,
  real_T c13_y[23]);
static void c13_f_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[23]);
static void c13_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_d_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_g_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_constraints, const char_T *c13_identifier,
  real_T c13_y[2]);
static void c13_h_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[2]);
static void c13_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_e_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_i_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_qDes, const char_T *c13_identifier, real_T
  c13_y[23]);
static void c13_j_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[23]);
static void c13_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_f_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_k_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_CoMDes, const char_T *c13_identifier,
  real_T c13_y[3]);
static void c13_l_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[3]);
static void c13_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_g_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_m_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_w_H_b, const char_T *c13_identifier, real_T
  c13_y[16]);
static void c13_n_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[16]);
static void c13_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_h_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_o_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_kzTB0QQWoOlMoMhgKf6sK *c13_y);
static void c13_p_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[9]);
static void c13_q_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[4]);
static void c13_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_i_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_r_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_rUGQ0INmvPpaxIctEGl5sE *c13_y);
static boolean_T c13_s_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId);
static void c13_t_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_DnBdbfPNxiIjhNOyZMmfsE *c13_y);
static void c13_u_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[39]);
static c13_struct_KJR2itYvhBuAkuR6dKZHUC c13_v_emlrt_marshallIn
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c13_u,
   const emlrtMsgIdentifier *c13_parentId);
static void c13_w_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_0U0wBk2LiR1OqsMsUngxdD *c13_y);
static void c13_x_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[299]);
static void c13_y_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[192]);
static void c13_ab_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[72]);
static void c13_bb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[13]);
static void c13_cb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[8]);
static void c13_db_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_9LpOi5JXaV67jTuay8hWaH *c13_y);
static void c13_eb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[24]);
static void c13_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_j_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static const mxArray *c13_k_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static const mxArray *c13_l_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_fb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_b_w_H_fixedLink, const char_T
  *c13_identifier, real_T c13_y[16]);
static void c13_gb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[16]);
static void c13_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_m_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static real_T c13_hb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_b_tSwitch, const char_T *c13_identifier);
static real_T c13_ib_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId);
static void c13_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_n_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static real_T c13_jb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_b_state, const char_T *c13_identifier);
static real_T c13_kb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId);
static void c13_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static void c13_lb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[6]);
static void c13_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static const mxArray *c13_o_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static void c13_mb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[4]);
static void c13_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static void c13_info_helper(c13_ResolvedFunctionInfo c13_info[145]);
static void c13_b_info_helper(c13_ResolvedFunctionInfo c13_info[145]);
static void c13_c_info_helper(c13_ResolvedFunctionInfo c13_info[145]);
static void c13_eye(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c13_I[16]);
static void c13_eml_scalar_eg(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c13_mrdivide(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_A[16], real_T c13_B[16], real_T c13_y[16]);
static void c13_realmin(SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void c13_eps(SFc13_torqueBalancing2012bInstanceStruct *chartInstance);
static void c13_eml_matlab_zgetrf(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_b_A[16], int32_T c13_ipiv[4],
  int32_T *c13_info);
static void c13_check_forloop_overflow_error
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c13_overflow);
static void c13_eml_xger(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c13_m, int32_T c13_n, real_T c13_alpha1, int32_T c13_ix0, int32_T
  c13_iy0, real_T c13_A[16], int32_T c13_ia0, real_T c13_b_A[16]);
static void c13_eml_warning(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c13_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[16], real_T c13_b_B[16]);
static void c13_below_threshold(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c13_b_eml_scalar_eg(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c13_b_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[16], real_T c13_b_B[16]);
static void c13_mldivide(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_A[16], real_T c13_B[4], real_T c13_Y[4]);
static void c13_c_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[4], real_T c13_b_B[4]);
static void c13_b_below_threshold(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c13_c_eml_scalar_eg(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c13_d_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[4], real_T c13_b_B[4]);
static real_T c13_norm(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_x);
static real_T c13_b_norm(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_x[6]);
static real_T c13_c_norm(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_x[2]);
static const mxArray *c13_p_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData);
static int32_T c13_nb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId);
static void c13_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData);
static uint8_T c13_ob_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c13_b_is_active_c13_torqueBalancing2012b, const
  char_T *c13_identifier);
static uint8_T c13_pb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId);
static void c13_b_eml_matlab_zgetrf(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], int32_T c13_ipiv[4], int32_T *c13_info);
static void c13_b_eml_xger(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c13_m, int32_T c13_n, real_T c13_alpha1, int32_T
  c13_ix0, int32_T c13_iy0, real_T c13_A[16], int32_T c13_ia0);
static void c13_e_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[16]);
static void c13_f_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[16]);
static void c13_g_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[4]);
static void c13_h_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[4]);
static void init_dsm_address_info(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c13_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c13_state_not_empty = FALSE;
  chartInstance->c13_tSwitch_not_empty = FALSE;
  chartInstance->c13_w_H_fixedLink_not_empty = FALSE;
  chartInstance->c13_is_active_c13_torqueBalancing2012b = 0U;
}

static void initialize_params_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c13_m0 = NULL;
  const mxArray *c13_mxField;
  c13_struct_rUGQ0INmvPpaxIctEGl5sE c13_r0;
  const mxArray *c13_m1 = NULL;
  const mxArray *c13_b_mxField;
  const mxArray *c13_m2 = NULL;
  const mxArray *c13_c_mxField;
  const mxArray *c13_m3 = NULL;
  const mxArray *c13_d_mxField;
  const mxArray *c13_m4 = NULL;
  const mxArray *c13_e_mxField;
  const mxArray *c13_m5 = NULL;
  const mxArray *c13_f_mxField;
  c13_struct_kzTB0QQWoOlMoMhgKf6sK c13_r1;
  sf_set_error_prefix_string(
    "Error evaluating data 'sm' in the parent workspace.\n");
  c13_m0 = sf_mex_get_sfun_param(chartInstance->S, 1, 1);
  c13_mxField = sf_mex_getfield(c13_m0, "skipYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), &c13_r0.skipYoga, 1, 11, 0U,
                      0, 0U, 0);
  c13_mxField = sf_mex_getfield(c13_m0, "demoOnlyRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), &c13_r0.demoOnlyRightFoot,
                      1, 11, 0U, 0, 0U, 0);
  c13_mxField = sf_mex_getfield(c13_m0, "yogaAlsoOnRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), &c13_r0.yogaAlsoOnRightFoot,
                      1, 11, 0U, 0, 0U, 0);
  c13_mxField = sf_mex_getfield(c13_m0, "yogaInLoop", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), &c13_r0.yogaInLoop, 1, 11,
                      0U, 0, 0U, 0);
  c13_mxField = sf_mex_getfield(c13_m0, "com", "sm", 0);
  c13_m1 = sf_mex_dup(c13_mxField);
  c13_b_mxField = sf_mex_getfield(c13_m1, "threshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_b_mxField), &c13_r0.com.threshold, 1,
                      0, 0U, 0, 0U, 0);
  c13_b_mxField = sf_mex_getfield(c13_m1, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_b_mxField), c13_r0.com.states, 1, 0,
                      0U, 1, 0U, 2, 13, 3);
  sf_mex_destroy(&c13_m1);
  c13_mxField = sf_mex_getfield(c13_m0, "wrench", "sm", 0);
  c13_m2 = sf_mex_dup(c13_mxField);
  c13_c_mxField = sf_mex_getfield(c13_m2, "thresholdContactOn", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_c_mxField),
                      &c13_r0.wrench.thresholdContactOn, 1, 0, 0U, 0, 0U, 0);
  c13_c_mxField = sf_mex_getfield(c13_m2, "thresholdContactOff", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_c_mxField),
                      &c13_r0.wrench.thresholdContactOff, 1, 0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c13_m2);
  c13_mxField = sf_mex_getfield(c13_m0, "joints", "sm", 0);
  c13_m3 = sf_mex_dup(c13_mxField);
  c13_d_mxField = sf_mex_getfield(c13_m3, "thresholdNotInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_d_mxField),
                      &c13_r0.joints.thresholdNotInContact, 1, 0, 0U, 0, 0U, 0);
  c13_d_mxField = sf_mex_getfield(c13_m3, "thresholdInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_d_mxField),
                      &c13_r0.joints.thresholdInContact, 1, 0, 0U, 0, 0U, 0);
  c13_d_mxField = sf_mex_getfield(c13_m3, "pauseTimeLastPostureL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_d_mxField),
                      &c13_r0.joints.pauseTimeLastPostureL, 1, 0, 0U, 0, 0U, 0);
  c13_d_mxField = sf_mex_getfield(c13_m3, "pauseTimeLastPostureR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_d_mxField),
                      &c13_r0.joints.pauseTimeLastPostureR, 1, 0, 0U, 0, 0U, 0);
  c13_d_mxField = sf_mex_getfield(c13_m3, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_d_mxField), c13_r0.joints.states, 1,
                      0, 0U, 1, 0U, 2, 13, 23);
  c13_d_mxField = sf_mex_getfield(c13_m3, "pointsL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_d_mxField), c13_r0.joints.pointsL, 1,
                      0, 0U, 1, 0U, 2, 8, 24);
  c13_d_mxField = sf_mex_getfield(c13_m3, "pointsR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_d_mxField), c13_r0.joints.pointsR, 1,
                      0, 0U, 1, 0U, 2, 8, 24);
  c13_d_mxField = sf_mex_getfield(c13_m3, "standUpPositions", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_d_mxField),
                      c13_r0.joints.standUpPositions, 1, 0, 0U, 1, 0U, 2, 8, 9);
  sf_mex_destroy(&c13_m3);
  c13_mxField = sf_mex_getfield(c13_m0, "stateAt0", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), &c13_r0.stateAt0, 1, 0, 0U,
                      0, 0U, 0);
  c13_mxField = sf_mex_getfield(c13_m0, "DT", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), &c13_r0.DT, 1, 0, 0U, 0, 0U,
                      0);
  c13_mxField = sf_mex_getfield(c13_m0, "waitingTimeAfterYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField),
                      &c13_r0.waitingTimeAfterYoga, 1, 0, 0U, 0, 0U, 0);
  c13_mxField = sf_mex_getfield(c13_m0, "jointsSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), c13_r0.jointsSmoothingTimes,
                      1, 0, 0U, 1, 0U, 2, 13, 1);
  c13_mxField = sf_mex_getfield(c13_m0, "tBalancing", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), &c13_r0.tBalancing, 1, 0,
                      0U, 0, 0U, 0);
  c13_mxField = sf_mex_getfield(c13_m0, "alsoSitDown", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), &c13_r0.alsoSitDown, 1, 11,
                      0U, 0, 0U, 0);
  c13_mxField = sf_mex_getfield(c13_m0, "jointsAndCoMSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField),
                      c13_r0.jointsAndCoMSmoothingTimes, 1, 0, 0U, 1, 0U, 2, 8,
                      1);
  c13_mxField = sf_mex_getfield(c13_m0, "CoM", "sm", 0);
  c13_m4 = sf_mex_dup(c13_mxField);
  c13_e_mxField = sf_mex_getfield(c13_m4, "standUpDeltaCoM", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_e_mxField),
                      c13_r0.CoM.standUpDeltaCoM, 1, 0, 0U, 1, 0U, 2, 8, 3);
  sf_mex_destroy(&c13_m4);
  c13_mxField = sf_mex_getfield(c13_m0, "LwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), c13_r0.LwrenchThreshold, 1,
                      0, 0U, 1, 0U, 2, 8, 1);
  c13_mxField = sf_mex_getfield(c13_m0, "RwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), c13_r0.RwrenchThreshold, 1,
                      0, 0U, 1, 0U, 2, 8, 1);
  c13_mxField = sf_mex_getfield(c13_m0, "RArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), c13_r0.RArmThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  c13_mxField = sf_mex_getfield(c13_m0, "LArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c13_mxField), c13_r0.LArmThreshold, 1, 0,
                      0U, 1, 0U, 2, 8, 1);
  sf_mex_destroy(&c13_m0);
  chartInstance->c13_sm = c13_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'gain' in the parent workspace.\n");
  c13_m5 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c13_f_mxField = sf_mex_getfield(c13_m5, "qTildeMax", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), &c13_r1.qTildeMax, 1, 0,
                      0U, 0, 0U, 0);
  c13_f_mxField = sf_mex_getfield(c13_m5, "SmoothingTimeImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField),
                      &c13_r1.SmoothingTimeImp, 1, 0, 0U, 0, 0U, 0);
  c13_f_mxField = sf_mex_getfield(c13_m5, "SmoothingTimeGainScheduling", "gain",
    0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField),
                      &c13_r1.SmoothingTimeGainScheduling, 1, 0, 0U, 0, 0U, 0);
  c13_f_mxField = sf_mex_getfield(c13_m5, "PCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), c13_r1.PCOM, 1, 0, 0U,
                      1, 0U, 2, 3, 3);
  c13_f_mxField = sf_mex_getfield(c13_m5, "ICOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), c13_r1.ICOM, 1, 0, 0U,
                      1, 0U, 2, 3, 3);
  c13_f_mxField = sf_mex_getfield(c13_m5, "DCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), c13_r1.DCOM, 1, 0, 0U,
                      1, 0U, 2, 3, 3);
  c13_f_mxField = sf_mex_getfield(c13_m5, "PAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField),
                      &c13_r1.PAngularMomentum, 1, 0, 0U, 0, 0U, 0);
  c13_f_mxField = sf_mex_getfield(c13_m5, "DAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField),
                      &c13_r1.DAngularMomentum, 1, 0, 0U, 0, 0U, 0);
  c13_f_mxField = sf_mex_getfield(c13_m5, "integral", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), c13_r1.integral, 1, 0,
                      0U, 1, 0U, 2, 1, 23);
  c13_f_mxField = sf_mex_getfield(c13_m5, "impedances", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), c13_r1.impedances, 1, 0,
                      0U, 1, 0U, 2, 1, 23);
  c13_f_mxField = sf_mex_getfield(c13_m5, "dampings", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), c13_r1.dampings, 1, 0,
                      0U, 1, 0U, 2, 1, 23);
  c13_f_mxField = sf_mex_getfield(c13_m5, "increasingRatesImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField),
                      c13_r1.increasingRatesImp, 1, 0, 0U, 1, 0U, 2, 1, 23);
  c13_f_mxField = sf_mex_getfield(c13_m5, "footSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), c13_r1.footSize, 1, 0,
                      0U, 1, 0U, 2, 2, 2);
  c13_f_mxField = sf_mex_getfield(c13_m5, "legSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c13_f_mxField), c13_r1.legSize, 1, 0,
                      0U, 1, 0U, 2, 2, 2);
  sf_mex_destroy(&c13_m5);
  chartInstance->c13_gain = c13_r1;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c13_update_debugger_state_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c13_st;
  const mxArray *c13_y = NULL;
  int32_T c13_i0;
  real_T c13_u[3];
  const mxArray *c13_b_y = NULL;
  int32_T c13_i1;
  real_T c13_b_u[2];
  const mxArray *c13_c_y = NULL;
  real_T c13_hoistedGlobal;
  real_T c13_c_u;
  const mxArray *c13_d_y = NULL;
  int32_T c13_i2;
  real_T c13_d_u[23];
  const mxArray *c13_e_y = NULL;
  real_T c13_b_hoistedGlobal;
  real_T c13_e_u;
  const mxArray *c13_f_y = NULL;
  int32_T c13_i3;
  real_T c13_f_u[3];
  const mxArray *c13_g_y = NULL;
  int32_T c13_i4;
  real_T c13_g_u[3];
  const mxArray *c13_h_y = NULL;
  int32_T c13_i5;
  real_T c13_h_u[23];
  const mxArray *c13_i_y = NULL;
  int32_T c13_i6;
  real_T c13_i_u[16];
  const mxArray *c13_j_y = NULL;
  real_T c13_c_hoistedGlobal;
  real_T c13_j_u;
  const mxArray *c13_k_y = NULL;
  real_T c13_d_hoistedGlobal;
  real_T c13_k_u;
  const mxArray *c13_l_y = NULL;
  int32_T c13_i7;
  real_T c13_l_u[16];
  const mxArray *c13_m_y = NULL;
  uint8_T c13_e_hoistedGlobal;
  uint8_T c13_m_u;
  const mxArray *c13_n_y = NULL;
  real_T *c13_currentState;
  real_T *c13_jointsSmoothingTime;
  real_T (*c13_w_H_b)[16];
  real_T (*c13_qDes)[23];
  real_T (*c13_kpCom)[3];
  real_T (*c13_kdCom)[3];
  real_T (*c13_impedances)[23];
  real_T (*c13_constraints)[2];
  real_T (*c13_CoMDes)[3];
  c13_jointsSmoothingTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 9);
  c13_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c13_kdCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 7);
  c13_kpCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 6);
  c13_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 5);
  c13_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 4);
  c13_qDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c13_CoMDes = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c13_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c13_st = NULL;
  c13_st = NULL;
  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_createcellarray(13), FALSE);
  for (c13_i0 = 0; c13_i0 < 3; c13_i0++) {
    c13_u[c13_i0] = (*c13_CoMDes)[c13_i0];
  }

  c13_b_y = NULL;
  sf_mex_assign(&c13_b_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c13_y, 0, c13_b_y);
  for (c13_i1 = 0; c13_i1 < 2; c13_i1++) {
    c13_b_u[c13_i1] = (*c13_constraints)[c13_i1];
  }

  c13_c_y = NULL;
  sf_mex_assign(&c13_c_y, sf_mex_create("y", c13_b_u, 0, 0U, 1U, 0U, 1, 2),
                FALSE);
  sf_mex_setcell(c13_y, 1, c13_c_y);
  c13_hoistedGlobal = *c13_currentState;
  c13_c_u = c13_hoistedGlobal;
  c13_d_y = NULL;
  sf_mex_assign(&c13_d_y, sf_mex_create("y", &c13_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c13_y, 2, c13_d_y);
  for (c13_i2 = 0; c13_i2 < 23; c13_i2++) {
    c13_d_u[c13_i2] = (*c13_impedances)[c13_i2];
  }

  c13_e_y = NULL;
  sf_mex_assign(&c13_e_y, sf_mex_create("y", c13_d_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_setcell(c13_y, 3, c13_e_y);
  c13_b_hoistedGlobal = *c13_jointsSmoothingTime;
  c13_e_u = c13_b_hoistedGlobal;
  c13_f_y = NULL;
  sf_mex_assign(&c13_f_y, sf_mex_create("y", &c13_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c13_y, 4, c13_f_y);
  for (c13_i3 = 0; c13_i3 < 3; c13_i3++) {
    c13_f_u[c13_i3] = (*c13_kdCom)[c13_i3];
  }

  c13_g_y = NULL;
  sf_mex_assign(&c13_g_y, sf_mex_create("y", c13_f_u, 0, 0U, 1U, 0U, 2, 1, 3),
                FALSE);
  sf_mex_setcell(c13_y, 5, c13_g_y);
  for (c13_i4 = 0; c13_i4 < 3; c13_i4++) {
    c13_g_u[c13_i4] = (*c13_kpCom)[c13_i4];
  }

  c13_h_y = NULL;
  sf_mex_assign(&c13_h_y, sf_mex_create("y", c13_g_u, 0, 0U, 1U, 0U, 2, 1, 3),
                FALSE);
  sf_mex_setcell(c13_y, 6, c13_h_y);
  for (c13_i5 = 0; c13_i5 < 23; c13_i5++) {
    c13_h_u[c13_i5] = (*c13_qDes)[c13_i5];
  }

  c13_i_y = NULL;
  sf_mex_assign(&c13_i_y, sf_mex_create("y", c13_h_u, 0, 0U, 1U, 0U, 1, 23),
                FALSE);
  sf_mex_setcell(c13_y, 7, c13_i_y);
  for (c13_i6 = 0; c13_i6 < 16; c13_i6++) {
    c13_i_u[c13_i6] = (*c13_w_H_b)[c13_i6];
  }

  c13_j_y = NULL;
  sf_mex_assign(&c13_j_y, sf_mex_create("y", c13_i_u, 0, 0U, 1U, 0U, 2, 4, 4),
                FALSE);
  sf_mex_setcell(c13_y, 8, c13_j_y);
  c13_c_hoistedGlobal = chartInstance->c13_state;
  c13_j_u = c13_c_hoistedGlobal;
  c13_k_y = NULL;
  if (!chartInstance->c13_state_not_empty) {
    sf_mex_assign(&c13_k_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c13_k_y, sf_mex_create("y", &c13_j_u, 0, 0U, 0U, 0U, 0),
                  FALSE);
  }

  sf_mex_setcell(c13_y, 9, c13_k_y);
  c13_d_hoistedGlobal = chartInstance->c13_tSwitch;
  c13_k_u = c13_d_hoistedGlobal;
  c13_l_y = NULL;
  if (!chartInstance->c13_tSwitch_not_empty) {
    sf_mex_assign(&c13_l_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c13_l_y, sf_mex_create("y", &c13_k_u, 0, 0U, 0U, 0U, 0),
                  FALSE);
  }

  sf_mex_setcell(c13_y, 10, c13_l_y);
  for (c13_i7 = 0; c13_i7 < 16; c13_i7++) {
    c13_l_u[c13_i7] = chartInstance->c13_w_H_fixedLink[c13_i7];
  }

  c13_m_y = NULL;
  if (!chartInstance->c13_w_H_fixedLink_not_empty) {
    sf_mex_assign(&c13_m_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c13_m_y, sf_mex_create("y", c13_l_u, 0, 0U, 1U, 0U, 2, 4, 4),
                  FALSE);
  }

  sf_mex_setcell(c13_y, 11, c13_m_y);
  c13_e_hoistedGlobal = chartInstance->c13_is_active_c13_torqueBalancing2012b;
  c13_m_u = c13_e_hoistedGlobal;
  c13_n_y = NULL;
  sf_mex_assign(&c13_n_y, sf_mex_create("y", &c13_m_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c13_y, 12, c13_n_y);
  sf_mex_assign(&c13_st, c13_y, FALSE);
  return c13_st;
}

static void set_sim_state_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c13_st)
{
  const mxArray *c13_u;
  real_T c13_dv0[3];
  int32_T c13_i8;
  real_T c13_dv1[2];
  int32_T c13_i9;
  real_T c13_dv2[23];
  int32_T c13_i10;
  real_T c13_dv3[3];
  int32_T c13_i11;
  real_T c13_dv4[3];
  int32_T c13_i12;
  real_T c13_dv5[23];
  int32_T c13_i13;
  real_T c13_dv6[16];
  int32_T c13_i14;
  real_T c13_dv7[16];
  int32_T c13_i15;
  real_T *c13_currentState;
  real_T *c13_jointsSmoothingTime;
  real_T (*c13_CoMDes)[3];
  real_T (*c13_constraints)[2];
  real_T (*c13_qDes)[23];
  real_T (*c13_w_H_b)[16];
  real_T (*c13_kpCom)[3];
  real_T (*c13_kdCom)[3];
  real_T (*c13_impedances)[23];
  c13_jointsSmoothingTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 9);
  c13_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c13_kdCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 7);
  c13_kpCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 6);
  c13_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 5);
  c13_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 4);
  c13_qDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c13_CoMDes = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c13_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c13_doneDoubleBufferReInit = TRUE;
  c13_u = sf_mex_dup(c13_st);
  c13_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 0)),
    "CoMDes", c13_dv0);
  for (c13_i8 = 0; c13_i8 < 3; c13_i8++) {
    (*c13_CoMDes)[c13_i8] = c13_dv0[c13_i8];
  }

  c13_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 1)),
    "constraints", c13_dv1);
  for (c13_i9 = 0; c13_i9 < 2; c13_i9++) {
    (*c13_constraints)[c13_i9] = c13_dv1[c13_i9];
  }

  *c13_currentState = c13_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c13_u, 2)), "currentState");
  c13_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 3)),
    "impedances", c13_dv2);
  for (c13_i10 = 0; c13_i10 < 23; c13_i10++) {
    (*c13_impedances)[c13_i10] = c13_dv2[c13_i10];
  }

  *c13_jointsSmoothingTime = c13_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c13_u, 4)), "jointsSmoothingTime");
  c13_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 5)),
    "kdCom", c13_dv3);
  for (c13_i11 = 0; c13_i11 < 3; c13_i11++) {
    (*c13_kdCom)[c13_i11] = c13_dv3[c13_i11];
  }

  c13_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 6)),
    "kpCom", c13_dv4);
  for (c13_i12 = 0; c13_i12 < 3; c13_i12++) {
    (*c13_kpCom)[c13_i12] = c13_dv4[c13_i12];
  }

  c13_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 7)),
    "qDes", c13_dv5);
  for (c13_i13 = 0; c13_i13 < 23; c13_i13++) {
    (*c13_qDes)[c13_i13] = c13_dv5[c13_i13];
  }

  c13_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 8)),
    "w_H_b", c13_dv6);
  for (c13_i14 = 0; c13_i14 < 16; c13_i14++) {
    (*c13_w_H_b)[c13_i14] = c13_dv6[c13_i14];
  }

  chartInstance->c13_state = c13_jb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c13_u, 9)), "state");
  chartInstance->c13_tSwitch = c13_hb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c13_u, 10)), "tSwitch");
  c13_fb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 11)),
    "w_H_fixedLink", c13_dv7);
  for (c13_i15 = 0; c13_i15 < 16; c13_i15++) {
    chartInstance->c13_w_H_fixedLink[c13_i15] = c13_dv7[c13_i15];
  }

  chartInstance->c13_is_active_c13_torqueBalancing2012b =
    c13_ob_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c13_u, 12)),
    "is_active_c13_torqueBalancing2012b");
  sf_mex_destroy(&c13_u);
  c13_update_debugger_state_c13_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c13_st);
}

static void finalize_c13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c13_torqueBalancing2012b(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c13_i16;
  int32_T c13_i17;
  int32_T c13_i18;
  int32_T c13_i19;
  int32_T c13_i20;
  int32_T c13_i21;
  int32_T c13_i22;
  int32_T c13_i23;
  int32_T c13_i24;
  int32_T c13_i25;
  int32_T c13_i26;
  int32_T c13_i27;
  int32_T c13_i28;
  int32_T c13_i29;
  int32_T c13_i30;
  int32_T c13_i31;
  real_T c13_hoistedGlobal;
  int32_T c13_i32;
  real_T c13_wrench_rightFoot[6];
  int32_T c13_i33;
  real_T c13_wrench_leftFoot[6];
  int32_T c13_i34;
  real_T c13_CoM_0[3];
  int32_T c13_i35;
  real_T c13_q0[23];
  int32_T c13_i36;
  real_T c13_l_sole_CoM[3];
  int32_T c13_i37;
  real_T c13_r_sole_CoM[3];
  int32_T c13_i38;
  real_T c13_qj[23];
  real_T c13_t;
  int32_T c13_i39;
  real_T c13_l_sole_H_b[16];
  int32_T c13_i40;
  real_T c13_r_sole_H_b[16];
  c13_struct_rUGQ0INmvPpaxIctEGl5sE c13_b_sm;
  c13_struct_kzTB0QQWoOlMoMhgKf6sK c13_b_gain;
  uint32_T c13_debug_family_var_map[23];
  real_T c13_nargin = 12.0;
  real_T c13_nargout = 9.0;
  real_T c13_w_H_b[16];
  real_T c13_CoMDes[3];
  real_T c13_qDes[23];
  real_T c13_constraints[2];
  real_T c13_impedances[23];
  real_T c13_kpCom[3];
  real_T c13_kdCom[3];
  real_T c13_currentState;
  real_T c13_jointsSmoothingTime;
  int32_T c13_i41;
  real_T c13_b_CoM_0[3];
  int32_T c13_i42;
  real_T c13_b_q0[23];
  int32_T c13_i43;
  real_T c13_b_l_sole_CoM[3];
  int32_T c13_i44;
  real_T c13_b_r_sole_CoM[3];
  int32_T c13_i45;
  real_T c13_b_qj[23];
  int32_T c13_i46;
  real_T c13_b_wrench_rightFoot[6];
  int32_T c13_i47;
  real_T c13_b_wrench_leftFoot[6];
  int32_T c13_i48;
  real_T c13_b_l_sole_H_b[16];
  int32_T c13_i49;
  real_T c13_b_r_sole_H_b[16];
  c13_struct_rUGQ0INmvPpaxIctEGl5sE c13_c_sm;
  c13_struct_kzTB0QQWoOlMoMhgKf6sK c13_c_gain;
  real_T c13_b_jointsSmoothingTime;
  real_T c13_b_currentState;
  real_T c13_b_kdCom[3];
  real_T c13_b_kpCom[3];
  real_T c13_b_impedances[23];
  real_T c13_b_constraints[2];
  real_T c13_b_qDes[23];
  real_T c13_b_CoMDes[3];
  real_T c13_b_w_H_b[16];
  int32_T c13_i50;
  int32_T c13_i51;
  int32_T c13_i52;
  int32_T c13_i53;
  int32_T c13_i54;
  int32_T c13_i55;
  int32_T c13_i56;
  int32_T c13_i57;
  int32_T c13_i58;
  int32_T c13_i59;
  int32_T c13_i60;
  int32_T c13_i61;
  int32_T c13_i62;
  int32_T c13_i63;
  real_T *c13_b_t;
  real_T *c13_c_currentState;
  real_T *c13_c_jointsSmoothingTime;
  real_T (*c13_c_w_H_b)[16];
  real_T (*c13_c_CoMDes)[3];
  real_T (*c13_c_qDes)[23];
  real_T (*c13_c_constraints)[2];
  real_T (*c13_c_kdCom)[3];
  real_T (*c13_c_kpCom)[3];
  real_T (*c13_c_impedances)[23];
  real_T (*c13_c_r_sole_H_b)[16];
  real_T (*c13_c_l_sole_H_b)[16];
  real_T (*c13_c_qj)[23];
  real_T (*c13_c_r_sole_CoM)[3];
  real_T (*c13_c_l_sole_CoM)[3];
  real_T (*c13_c_q0)[23];
  real_T (*c13_c_CoM_0)[3];
  real_T (*c13_c_wrench_leftFoot)[6];
  real_T (*c13_c_wrench_rightFoot)[6];
  c13_c_jointsSmoothingTime = (real_T *)ssGetOutputPortSignal(chartInstance->S,
    9);
  c13_c_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c13_c_kdCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 7);
  c13_c_kpCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 6);
  c13_c_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 5);
  c13_c_r_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 9);
  c13_c_l_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 8);
  c13_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c13_c_qj = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 6);
  c13_c_r_sole_CoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c13_c_l_sole_CoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
  c13_c_q0 = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c13_c_CoM_0 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c13_c_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 4);
  c13_c_qDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c13_c_CoMDes = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c13_c_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c13_c_wrench_leftFoot = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
    1);
  c13_c_wrench_rightFoot = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
    0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 11U, chartInstance->c13_sfEvent);
  for (c13_i16 = 0; c13_i16 < 6; c13_i16++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_wrench_rightFoot)[c13_i16], 0U);
  }

  for (c13_i17 = 0; c13_i17 < 6; c13_i17++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_wrench_leftFoot)[c13_i17], 1U);
  }

  for (c13_i18 = 0; c13_i18 < 16; c13_i18++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_w_H_b)[c13_i18], 2U);
  }

  for (c13_i19 = 0; c13_i19 < 3; c13_i19++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_CoMDes)[c13_i19], 3U);
  }

  for (c13_i20 = 0; c13_i20 < 23; c13_i20++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_qDes)[c13_i20], 4U);
  }

  for (c13_i21 = 0; c13_i21 < 2; c13_i21++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_constraints)[c13_i21], 5U);
  }

  for (c13_i22 = 0; c13_i22 < 3; c13_i22++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_CoM_0)[c13_i22], 6U);
  }

  for (c13_i23 = 0; c13_i23 < 23; c13_i23++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_q0)[c13_i23], 7U);
  }

  for (c13_i24 = 0; c13_i24 < 3; c13_i24++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_l_sole_CoM)[c13_i24], 8U);
  }

  for (c13_i25 = 0; c13_i25 < 3; c13_i25++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_r_sole_CoM)[c13_i25], 9U);
  }

  for (c13_i26 = 0; c13_i26 < 23; c13_i26++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_qj)[c13_i26], 10U);
  }

  _SFD_DATA_RANGE_CHECK(*c13_b_t, 11U);
  for (c13_i27 = 0; c13_i27 < 16; c13_i27++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_l_sole_H_b)[c13_i27], 12U);
  }

  for (c13_i28 = 0; c13_i28 < 16; c13_i28++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_r_sole_H_b)[c13_i28], 13U);
  }

  for (c13_i29 = 0; c13_i29 < 23; c13_i29++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_impedances)[c13_i29], 15U);
  }

  for (c13_i30 = 0; c13_i30 < 3; c13_i30++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_kpCom)[c13_i30], 16U);
  }

  for (c13_i31 = 0; c13_i31 < 3; c13_i31++) {
    _SFD_DATA_RANGE_CHECK((*c13_c_kdCom)[c13_i31], 17U);
  }

  _SFD_DATA_RANGE_CHECK(*c13_c_currentState, 19U);
  _SFD_DATA_RANGE_CHECK(*c13_c_jointsSmoothingTime, 20U);
  chartInstance->c13_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 11U, chartInstance->c13_sfEvent);
  c13_hoistedGlobal = *c13_b_t;
  for (c13_i32 = 0; c13_i32 < 6; c13_i32++) {
    c13_wrench_rightFoot[c13_i32] = (*c13_c_wrench_rightFoot)[c13_i32];
  }

  for (c13_i33 = 0; c13_i33 < 6; c13_i33++) {
    c13_wrench_leftFoot[c13_i33] = (*c13_c_wrench_leftFoot)[c13_i33];
  }

  for (c13_i34 = 0; c13_i34 < 3; c13_i34++) {
    c13_CoM_0[c13_i34] = (*c13_c_CoM_0)[c13_i34];
  }

  for (c13_i35 = 0; c13_i35 < 23; c13_i35++) {
    c13_q0[c13_i35] = (*c13_c_q0)[c13_i35];
  }

  for (c13_i36 = 0; c13_i36 < 3; c13_i36++) {
    c13_l_sole_CoM[c13_i36] = (*c13_c_l_sole_CoM)[c13_i36];
  }

  for (c13_i37 = 0; c13_i37 < 3; c13_i37++) {
    c13_r_sole_CoM[c13_i37] = (*c13_c_r_sole_CoM)[c13_i37];
  }

  for (c13_i38 = 0; c13_i38 < 23; c13_i38++) {
    c13_qj[c13_i38] = (*c13_c_qj)[c13_i38];
  }

  c13_t = c13_hoistedGlobal;
  for (c13_i39 = 0; c13_i39 < 16; c13_i39++) {
    c13_l_sole_H_b[c13_i39] = (*c13_c_l_sole_H_b)[c13_i39];
  }

  for (c13_i40 = 0; c13_i40 < 16; c13_i40++) {
    c13_r_sole_H_b[c13_i40] = (*c13_c_r_sole_H_b)[c13_i40];
  }

  c13_b_sm = chartInstance->c13_sm;
  c13_b_gain = chartInstance->c13_gain;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 23U, 23U, c13_debug_family_names,
    c13_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_nargin, 0U, c13_sf_marshallOut,
    c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_nargout, 1U, c13_sf_marshallOut,
    c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_wrench_rightFoot, 2U, c13_k_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_wrench_leftFoot, 3U, c13_k_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_CoM_0, 4U, c13_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_q0, 5U, c13_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_l_sole_CoM, 6U, c13_j_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_r_sole_CoM, 7U, c13_j_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_qj, 8U, c13_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c13_t, 9U, c13_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_l_sole_H_b, 10U, c13_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c13_r_sole_H_b, 11U, c13_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_b_sm, 12U, c13_i_sf_marshallOut,
    c13_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_b_gain, 13U, c13_h_sf_marshallOut,
    c13_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_w_H_b, 14U, c13_g_sf_marshallOut,
    c13_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_CoMDes, 15U, c13_f_sf_marshallOut,
    c13_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_qDes, 16U, c13_e_sf_marshallOut,
    c13_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_constraints, 17U,
    c13_d_sf_marshallOut, c13_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_impedances, 18U, c13_c_sf_marshallOut,
    c13_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_kpCom, 19U, c13_b_sf_marshallOut,
    c13_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_kdCom, 20U, c13_b_sf_marshallOut,
    c13_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_currentState, 21U,
    c13_sf_marshallOut, c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_jointsSmoothingTime, 22U,
    c13_sf_marshallOut, c13_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c13_sfEvent, 5);
  for (c13_i41 = 0; c13_i41 < 3; c13_i41++) {
    c13_b_CoM_0[c13_i41] = c13_CoM_0[c13_i41];
  }

  for (c13_i42 = 0; c13_i42 < 23; c13_i42++) {
    c13_b_q0[c13_i42] = c13_q0[c13_i42];
  }

  for (c13_i43 = 0; c13_i43 < 3; c13_i43++) {
    c13_b_l_sole_CoM[c13_i43] = c13_l_sole_CoM[c13_i43];
  }

  for (c13_i44 = 0; c13_i44 < 3; c13_i44++) {
    c13_b_r_sole_CoM[c13_i44] = c13_r_sole_CoM[c13_i44];
  }

  for (c13_i45 = 0; c13_i45 < 23; c13_i45++) {
    c13_b_qj[c13_i45] = c13_qj[c13_i45];
  }

  for (c13_i46 = 0; c13_i46 < 6; c13_i46++) {
    c13_b_wrench_rightFoot[c13_i46] = c13_wrench_rightFoot[c13_i46];
  }

  for (c13_i47 = 0; c13_i47 < 6; c13_i47++) {
    c13_b_wrench_leftFoot[c13_i47] = c13_wrench_leftFoot[c13_i47];
  }

  for (c13_i48 = 0; c13_i48 < 16; c13_i48++) {
    c13_b_l_sole_H_b[c13_i48] = c13_l_sole_H_b[c13_i48];
  }

  for (c13_i49 = 0; c13_i49 < 16; c13_i49++) {
    c13_b_r_sole_H_b[c13_i49] = c13_r_sole_H_b[c13_i49];
  }

  c13_c_sm = c13_b_sm;
  c13_c_gain = c13_b_gain;
  c13_stateMachine(chartInstance, c13_b_CoM_0, c13_b_q0, c13_b_l_sole_CoM,
                   c13_b_r_sole_CoM, c13_b_qj, c13_t, c13_b_wrench_rightFoot,
                   c13_b_wrench_leftFoot, c13_b_l_sole_H_b, c13_b_r_sole_H_b,
                   &c13_c_sm, &c13_c_gain, c13_b_w_H_b, c13_b_CoMDes, c13_b_qDes,
                   c13_b_constraints, c13_b_impedances, c13_b_kpCom, c13_b_kdCom,
                   &c13_b_currentState, &c13_b_jointsSmoothingTime);
  for (c13_i50 = 0; c13_i50 < 16; c13_i50++) {
    c13_w_H_b[c13_i50] = c13_b_w_H_b[c13_i50];
  }

  for (c13_i51 = 0; c13_i51 < 3; c13_i51++) {
    c13_CoMDes[c13_i51] = c13_b_CoMDes[c13_i51];
  }

  for (c13_i52 = 0; c13_i52 < 23; c13_i52++) {
    c13_qDes[c13_i52] = c13_b_qDes[c13_i52];
  }

  for (c13_i53 = 0; c13_i53 < 2; c13_i53++) {
    c13_constraints[c13_i53] = c13_b_constraints[c13_i53];
  }

  for (c13_i54 = 0; c13_i54 < 23; c13_i54++) {
    c13_impedances[c13_i54] = c13_b_impedances[c13_i54];
  }

  for (c13_i55 = 0; c13_i55 < 3; c13_i55++) {
    c13_kpCom[c13_i55] = c13_b_kpCom[c13_i55];
  }

  for (c13_i56 = 0; c13_i56 < 3; c13_i56++) {
    c13_kdCom[c13_i56] = c13_b_kdCom[c13_i56];
  }

  c13_currentState = c13_b_currentState;
  c13_jointsSmoothingTime = c13_b_jointsSmoothingTime;
  _SFD_EML_CALL(0U, chartInstance->c13_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
  for (c13_i57 = 0; c13_i57 < 16; c13_i57++) {
    (*c13_c_w_H_b)[c13_i57] = c13_w_H_b[c13_i57];
  }

  for (c13_i58 = 0; c13_i58 < 3; c13_i58++) {
    (*c13_c_CoMDes)[c13_i58] = c13_CoMDes[c13_i58];
  }

  for (c13_i59 = 0; c13_i59 < 23; c13_i59++) {
    (*c13_c_qDes)[c13_i59] = c13_qDes[c13_i59];
  }

  for (c13_i60 = 0; c13_i60 < 2; c13_i60++) {
    (*c13_c_constraints)[c13_i60] = c13_constraints[c13_i60];
  }

  for (c13_i61 = 0; c13_i61 < 23; c13_i61++) {
    (*c13_c_impedances)[c13_i61] = c13_impedances[c13_i61];
  }

  for (c13_i62 = 0; c13_i62 < 3; c13_i62++) {
    (*c13_c_kpCom)[c13_i62] = c13_kpCom[c13_i62];
  }

  for (c13_i63 = 0; c13_i63 < 3; c13_i63++) {
    (*c13_c_kdCom)[c13_i63] = c13_kdCom[c13_i63];
  }

  *c13_c_currentState = c13_currentState;
  *c13_c_jointsSmoothingTime = c13_jointsSmoothingTime;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 11U, chartInstance->c13_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc13_torqueBalancing2012b
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c13_stateMachine(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_CoM_0[3], real_T c13_q0[23], real_T c13_l_sole_CoM
  [3], real_T c13_r_sole_CoM[3], real_T c13_qj[23], real_T c13_t, real_T
  c13_wrench_rightFoot[6], real_T c13_wrench_leftFoot[6], real_T c13_l_sole_H_b
  [16], real_T c13_r_sole_H_b[16], c13_struct_rUGQ0INmvPpaxIctEGl5sE *c13_b_sm,
  c13_struct_kzTB0QQWoOlMoMhgKf6sK *c13_b_gain, real_T c13_w_H_b[16], real_T
  c13_CoMDes[3], real_T c13_qDes[23], real_T c13_constraints[2], real_T
  c13_impedances[23], real_T c13_kpCom[3], real_T c13_kdCom[3], real_T
  *c13_currentState, real_T *c13_jointsSmoothingTime)
{
  uint32_T c13_debug_family_var_map[31];
  real_T c13_fixed_link_CoMDes[4];
  real_T c13_CoMError[3];
  real_T c13_i;
  real_T c13_qTildeRLeg[6];
  real_T c13_qTildeLLeg[6];
  real_T c13_nargin = 12.0;
  real_T c13_nargout = 9.0;
  real_T c13_dv8[16];
  int32_T c13_i64;
  int32_T c13_i65;
  int32_T c13_i66;
  int32_T c13_i67;
  real_T c13_dv9[16];
  int32_T c13_i68;
  int32_T c13_i69;
  int32_T c13_i70;
  int32_T c13_i71;
  int32_T c13_i72;
  int32_T c13_i73;
  int32_T c13_i74;
  real_T c13_hoistedGlobal[16];
  int32_T c13_i75;
  real_T c13_b[16];
  int32_T c13_i76;
  int32_T c13_i77;
  int32_T c13_i78;
  real_T c13_C[16];
  int32_T c13_i79;
  int32_T c13_i80;
  int32_T c13_i81;
  int32_T c13_i82;
  int32_T c13_i83;
  int32_T c13_i84;
  int32_T c13_i85;
  int32_T c13_i86;
  int32_T c13_i87;
  int32_T c13_i88;
  int32_T c13_i89;
  int32_T c13_i90;
  int32_T c13_i91;
  int32_T c13_i92;
  int32_T c13_i93;
  int32_T c13_i94;
  real_T c13_b_C[16];
  int32_T c13_i95;
  real_T c13_b_r_sole_H_b[16];
  real_T c13_dv10[16];
  int32_T c13_i96;
  int32_T c13_i97;
  int32_T c13_i98;
  int32_T c13_i99;
  int32_T c13_i100;
  int32_T c13_i101;
  int32_T c13_i102;
  int32_T c13_i103;
  int32_T c13_i104;
  int32_T c13_i105;
  int32_T c13_i106;
  int32_T c13_i107;
  int32_T c13_i108;
  int32_T c13_i109;
  int32_T c13_i110;
  int32_T c13_i111;
  real_T c13_dv11[3];
  int32_T c13_i112;
  int32_T c13_i113;
  int32_T c13_i114;
  int32_T c13_i115;
  int32_T c13_i116;
  int32_T c13_i117;
  int32_T c13_i118;
  real_T c13_dv12[16];
  int32_T c13_i119;
  real_T c13_b_CoMDes[4];
  real_T c13_dv13[4];
  int32_T c13_i120;
  int32_T c13_i121;
  int32_T c13_i122;
  int32_T c13_i123;
  int32_T c13_i124;
  int32_T c13_i125;
  int32_T c13_i126;
  int32_T c13_i127;
  int32_T c13_i128;
  int32_T c13_i129;
  int32_T c13_i130;
  int32_T c13_i131;
  int32_T c13_i132;
  int32_T c13_i133;
  int32_T c13_i134;
  int32_T c13_i135;
  int32_T c13_i136;
  int32_T c13_i137;
  int32_T c13_i138;
  real_T c13_dv14[3];
  int32_T c13_i139;
  int32_T c13_i140;
  int32_T c13_i141;
  int32_T c13_i142;
  int32_T c13_i143;
  int32_T c13_i144;
  int32_T c13_i145;
  int32_T c13_i146;
  int32_T c13_i147;
  int32_T c13_i148;
  int32_T c13_i149;
  int32_T c13_i150;
  int32_T c13_i151;
  int32_T c13_i152;
  int32_T c13_i153;
  int32_T c13_i154;
  int32_T c13_i155;
  int32_T c13_i156;
  int32_T c13_i157;
  int32_T c13_i158;
  int32_T c13_i159;
  int32_T c13_i160;
  int32_T c13_i161;
  int32_T c13_i162;
  real_T c13_dv15[3];
  int32_T c13_i163;
  int32_T c13_i164;
  int32_T c13_i165;
  int32_T c13_i166;
  int32_T c13_i167;
  int32_T c13_i168;
  int32_T c13_i169;
  int32_T c13_i170;
  int32_T c13_i171;
  int32_T c13_b_i;
  int32_T c13_c_i;
  int32_T c13_i172;
  int32_T c13_i173;
  int32_T c13_i174;
  int32_T c13_i175;
  int32_T c13_i176;
  int32_T c13_i177;
  int32_T c13_i178;
  int32_T c13_i179;
  int32_T c13_i180;
  int32_T c13_i181;
  int32_T c13_i182;
  int32_T c13_i183;
  int32_T c13_i184;
  int32_T c13_i185;
  int32_T c13_i186;
  int32_T c13_i187;
  int32_T c13_i188;
  int32_T c13_i189;
  real_T c13_dv16[3];
  int32_T c13_i190;
  int32_T c13_i191;
  int32_T c13_i192;
  int32_T c13_i193;
  int32_T c13_i194;
  int32_T c13_i195;
  int32_T c13_i196;
  int32_T c13_i197;
  int32_T c13_i198;
  int32_T c13_i199;
  int32_T c13_i200;
  int32_T c13_i201;
  real_T c13_b_qTildeRLeg[6];
  real_T c13_a;
  real_T c13_y;
  real_T c13_A;
  real_T c13_x;
  real_T c13_b_x;
  real_T c13_b_y;
  int32_T c13_i202;
  real_T c13_b_qTildeLLeg[6];
  real_T c13_b_a;
  real_T c13_c_y;
  real_T c13_b_A;
  real_T c13_c_x;
  real_T c13_d_x;
  real_T c13_d_y;
  int32_T c13_i203;
  int32_T c13_i204;
  int32_T c13_i205;
  int32_T c13_i206;
  int32_T c13_i207;
  int32_T c13_i208;
  int32_T c13_i209;
  int32_T c13_i210;
  int32_T c13_i211;
  int32_T c13_i212;
  int32_T c13_i213;
  int32_T c13_i214;
  int32_T c13_i215;
  int32_T c13_i216;
  int32_T c13_i217;
  real_T c13_dv17[3];
  int32_T c13_i218;
  int32_T c13_i219;
  int32_T c13_i220;
  int32_T c13_i221;
  int32_T c13_i222;
  int32_T c13_i223;
  int32_T c13_i224;
  int32_T c13_i225;
  int32_T c13_i226;
  int32_T c13_i227;
  int32_T c13_i228;
  int32_T c13_i229;
  int32_T c13_i230;
  int32_T c13_i231;
  int32_T c13_i232;
  int32_T c13_i233;
  int32_T c13_i234;
  int32_T c13_i235;
  int32_T c13_i236;
  int32_T c13_i237;
  int32_T c13_i238;
  int32_T c13_i239;
  int32_T c13_i240;
  int32_T c13_i241;
  int32_T c13_i242;
  int32_T c13_i243;
  int32_T c13_i244;
  int32_T c13_i245;
  int32_T c13_i246;
  int32_T c13_i247;
  real_T c13_b_b;
  real_T c13_e_y;
  int32_T c13_i248;
  real_T c13_b_l_sole_CoM[2];
  int32_T c13_i249;
  int32_T c13_i250;
  int32_T c13_i251;
  int32_T c13_i252;
  int32_T c13_i253;
  int32_T c13_i254;
  int32_T c13_i255;
  int32_T c13_i256;
  real_T c13_c_C[16];
  int32_T c13_i257;
  real_T c13_c_r_sole_H_b[16];
  real_T c13_dv18[16];
  int32_T c13_i258;
  int32_T c13_i259;
  int32_T c13_i260;
  int32_T c13_i261;
  int32_T c13_i262;
  int32_T c13_i263;
  int32_T c13_i264;
  int32_T c13_i265;
  int32_T c13_i266;
  int32_T c13_i267;
  int32_T c13_i268;
  int32_T c13_i269;
  int32_T c13_i270;
  int32_T c13_i271;
  int32_T c13_i272;
  int32_T c13_i273;
  int32_T c13_i274;
  real_T c13_dv19[3];
  int32_T c13_i275;
  int32_T c13_i276;
  real_T c13_dv20[16];
  int32_T c13_i277;
  real_T c13_c_CoMDes[4];
  real_T c13_dv21[4];
  int32_T c13_i278;
  int32_T c13_i279;
  int32_T c13_i280;
  int32_T c13_i281;
  int32_T c13_i282;
  int32_T c13_i283;
  int32_T c13_i284;
  int32_T c13_i285;
  int32_T c13_i286;
  int32_T c13_i287;
  int32_T c13_i288;
  int32_T c13_i289;
  int32_T c13_i290;
  int32_T c13_i291;
  int32_T c13_i292;
  int32_T c13_i293;
  int32_T c13_i294;
  int32_T c13_i295;
  int32_T c13_i296;
  int32_T c13_i297;
  int32_T c13_i298;
  int32_T c13_i299;
  int32_T c13_i300;
  int32_T c13_i301;
  int32_T c13_i302;
  real_T c13_dv22[3];
  int32_T c13_i303;
  int32_T c13_i304;
  int32_T c13_i305;
  int32_T c13_i306;
  int32_T c13_i307;
  int32_T c13_i308;
  int32_T c13_i309;
  int32_T c13_i310;
  int32_T c13_i311;
  int32_T c13_i312;
  int32_T c13_i313;
  int32_T c13_i314;
  int32_T c13_i315;
  int32_T c13_i316;
  int32_T c13_i317;
  int32_T c13_i318;
  int32_T c13_i319;
  int32_T c13_i320;
  int32_T c13_i321;
  int32_T c13_i322;
  int32_T c13_i323;
  int32_T c13_i324;
  int32_T c13_i325;
  int32_T c13_i326;
  real_T c13_dv23[3];
  int32_T c13_i327;
  int32_T c13_i328;
  int32_T c13_i329;
  int32_T c13_i330;
  int32_T c13_i331;
  int32_T c13_i332;
  int32_T c13_i333;
  int32_T c13_i334;
  int32_T c13_d_i;
  int32_T c13_e_i;
  int32_T c13_i335;
  int32_T c13_i336;
  int32_T c13_i337;
  int32_T c13_i338;
  int32_T c13_i339;
  int32_T c13_i340;
  int32_T c13_i341;
  int32_T c13_i342;
  int32_T c13_i343;
  int32_T c13_i344;
  int32_T c13_i345;
  int32_T c13_i346;
  int32_T c13_i347;
  int32_T c13_i348;
  int32_T c13_i349;
  int32_T c13_i350;
  int32_T c13_i351;
  int32_T c13_i352;
  int32_T c13_i353;
  real_T c13_dv24[3];
  int32_T c13_i354;
  int32_T c13_i355;
  int32_T c13_i356;
  int32_T c13_i357;
  int32_T c13_i358;
  int32_T c13_i359;
  int32_T c13_i360;
  int32_T c13_i361;
  int32_T c13_i362;
  int32_T c13_i363;
  int32_T c13_i364;
  real_T c13_c_qTildeRLeg[6];
  real_T c13_c_a;
  real_T c13_f_y;
  real_T c13_c_A;
  real_T c13_e_x;
  real_T c13_f_x;
  real_T c13_g_y;
  int32_T c13_i365;
  real_T c13_c_qTildeLLeg[6];
  real_T c13_d_a;
  real_T c13_h_y;
  real_T c13_d_A;
  real_T c13_g_x;
  real_T c13_h_x;
  real_T c13_i_y;
  int32_T c13_i366;
  int32_T c13_i367;
  int32_T c13_i368;
  int32_T c13_i369;
  int32_T c13_i370;
  int32_T c13_i371;
  int32_T c13_i372;
  int32_T c13_i373;
  int32_T c13_i374;
  int32_T c13_i375;
  int32_T c13_i376;
  int32_T c13_i377;
  int32_T c13_i378;
  int32_T c13_i379;
  int32_T c13_i380;
  int32_T c13_i381;
  real_T c13_dv25[3];
  int32_T c13_i382;
  int32_T c13_i383;
  int32_T c13_i384;
  int32_T c13_i385;
  int32_T c13_i386;
  int32_T c13_i387;
  int32_T c13_i388;
  int32_T c13_i389;
  int32_T c13_i390;
  int32_T c13_i391;
  int32_T c13_i392;
  int32_T c13_i393;
  int32_T c13_i394;
  int32_T c13_i395;
  int32_T c13_i396;
  int32_T c13_i397;
  int32_T c13_i398;
  int32_T c13_i399;
  int32_T c13_i400;
  int32_T c13_i401;
  int32_T c13_i402;
  int32_T c13_i403;
  int32_T c13_i404;
  int32_T c13_i405;
  int32_T c13_i406;
  int32_T c13_i407;
  int32_T c13_i408;
  int32_T c13_i409;
  int32_T c13_i410;
  int32_T c13_i411;
  int32_T c13_i412;
  int32_T c13_i413;
  int32_T c13_i414;
  int32_T c13_i415;
  int32_T c13_i416;
  real_T c13_d_C[16];
  int32_T c13_i417;
  real_T c13_b_l_sole_H_b[16];
  real_T c13_dv26[16];
  int32_T c13_i418;
  int32_T c13_i419;
  int32_T c13_i420;
  int32_T c13_i421;
  int32_T c13_i422;
  int32_T c13_i423;
  int32_T c13_i424;
  int32_T c13_i425;
  int32_T c13_i426;
  real_T c13_e_C[16];
  int32_T c13_i427;
  real_T c13_d_r_sole_H_b[16];
  real_T c13_dv27[16];
  int32_T c13_i428;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  boolean_T guard5 = FALSE;
  boolean_T guard6 = FALSE;
  boolean_T guard7 = FALSE;
  boolean_T guard8 = FALSE;
  boolean_T guard9 = FALSE;
  boolean_T guard10 = FALSE;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 31U, 31U, c13_b_debug_family_names,
    c13_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_fixed_link_CoMDes, 0U,
    c13_o_sf_marshallOut, c13_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_CoMError, 1U, c13_f_sf_marshallOut,
    c13_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_i, 2U, c13_sf_marshallOut,
    c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_qTildeRLeg, 3U, c13_k_sf_marshallOut,
    c13_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_qTildeLLeg, 4U, c13_k_sf_marshallOut,
    c13_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_nargin, 5U, c13_sf_marshallOut,
    c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_nargout, 6U, c13_sf_marshallOut,
    c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_CoM_0, 7U, c13_f_sf_marshallOut,
    c13_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_q0, 8U, c13_e_sf_marshallOut,
    c13_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_l_sole_CoM, 9U, c13_f_sf_marshallOut,
    c13_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_r_sole_CoM, 10U, c13_f_sf_marshallOut,
    c13_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_qj, 11U, c13_e_sf_marshallOut,
    c13_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c13_t, 12U, c13_sf_marshallOut,
    c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_wrench_rightFoot, 13U,
    c13_k_sf_marshallOut, c13_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_wrench_leftFoot, 14U,
    c13_k_sf_marshallOut, c13_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_l_sole_H_b, 15U, c13_g_sf_marshallOut,
    c13_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_r_sole_H_b, 16U, c13_g_sf_marshallOut,
    c13_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_b_sm, 17U, c13_i_sf_marshallOut,
    c13_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_b_gain, 18U, c13_h_sf_marshallOut,
    c13_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_w_H_b, 19U, c13_g_sf_marshallOut,
    c13_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_CoMDes, 20U, c13_f_sf_marshallOut,
    c13_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_qDes, 21U, c13_e_sf_marshallOut,
    c13_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_constraints, 22U,
    c13_d_sf_marshallOut, c13_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_impedances, 23U, c13_c_sf_marshallOut,
    c13_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_kpCom, 24U, c13_b_sf_marshallOut,
    c13_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_kdCom, 25U, c13_b_sf_marshallOut,
    c13_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_currentState, 26U, c13_sf_marshallOut,
    c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c13_jointsSmoothingTime, 27U,
    c13_sf_marshallOut, c13_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c13_state, 28U,
    c13_n_sf_marshallOut, c13_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c13_tSwitch, 29U,
    c13_m_sf_marshallOut, c13_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c13_w_H_fixedLink, 30U,
    c13_l_sf_marshallOut, c13_j_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 5);
  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 6);
  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 7);
  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 9);
  guard9 = FALSE;
  guard10 = FALSE;
  if (CV_SCRIPT_COND(0, 0, !chartInstance->c13_state_not_empty)) {
    guard10 = TRUE;
  } else if (CV_SCRIPT_COND(0, 1, !chartInstance->c13_tSwitch_not_empty)) {
    guard10 = TRUE;
  } else if (CV_SCRIPT_COND(0, 2, !chartInstance->c13_w_H_fixedLink_not_empty))
  {
    guard9 = TRUE;
  } else {
    CV_SCRIPT_MCDC(0, 0, FALSE);
    CV_SCRIPT_IF(0, 0, FALSE);
  }

  if (guard10 == TRUE) {
    guard9 = TRUE;
  }

  if (guard9 == TRUE) {
    CV_SCRIPT_MCDC(0, 0, TRUE);
    CV_SCRIPT_IF(0, 0, TRUE);
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 10);
    chartInstance->c13_state = c13_b_sm->stateAt0;
    chartInstance->c13_state_not_empty = TRUE;
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 11);
    chartInstance->c13_tSwitch = 0.0;
    chartInstance->c13_tSwitch_not_empty = TRUE;
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 12);
    c13_eye(chartInstance, c13_dv8);
    for (c13_i64 = 0; c13_i64 < 16; c13_i64++) {
      chartInstance->c13_w_H_fixedLink[c13_i64] = c13_dv8[c13_i64];
    }

    chartInstance->c13_w_H_fixedLink_not_empty = TRUE;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 15);
  for (c13_i65 = 0; c13_i65 < 3; c13_i65++) {
    c13_CoMDes[c13_i65] = c13_CoM_0[c13_i65];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 16);
  for (c13_i66 = 0; c13_i66 < 2; c13_i66++) {
    c13_constraints[c13_i66] = 1.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 17);
  for (c13_i67 = 0; c13_i67 < 23; c13_i67++) {
    c13_qDes[c13_i67] = c13_q0[c13_i67];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 18);
  c13_eye(chartInstance, c13_dv9);
  for (c13_i68 = 0; c13_i68 < 16; c13_i68++) {
    c13_w_H_b[c13_i68] = c13_dv9[c13_i68];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 19);
  for (c13_i69 = 0; c13_i69 < 23; c13_i69++) {
    c13_impedances[c13_i69] = c13_b_gain->impedances[c13_i69];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 20);
  c13_i70 = 0;
  for (c13_i71 = 0; c13_i71 < 3; c13_i71++) {
    c13_kpCom[c13_i71] = c13_b_gain->PCOM[c13_i70];
    c13_i70 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 21);
  c13_i72 = 0;
  for (c13_i73 = 0; c13_i73 < 3; c13_i73++) {
    c13_kdCom[c13_i73] = c13_b_gain->DCOM[c13_i72];
    c13_i72 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 24);
  if (CV_SCRIPT_IF(0, 1, chartInstance->c13_state == 1.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 25);
    for (c13_i74 = 0; c13_i74 < 16; c13_i74++) {
      c13_hoistedGlobal[c13_i74] = chartInstance->c13_w_H_fixedLink[c13_i74];
    }

    for (c13_i75 = 0; c13_i75 < 16; c13_i75++) {
      c13_b[c13_i75] = c13_l_sole_H_b[c13_i75];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i76 = 0; c13_i76 < 16; c13_i76++) {
      c13_w_H_b[c13_i76] = 0.0;
    }

    for (c13_i77 = 0; c13_i77 < 16; c13_i77++) {
      c13_w_H_b[c13_i77] = 0.0;
    }

    for (c13_i78 = 0; c13_i78 < 16; c13_i78++) {
      c13_C[c13_i78] = c13_w_H_b[c13_i78];
    }

    for (c13_i79 = 0; c13_i79 < 16; c13_i79++) {
      c13_w_H_b[c13_i79] = c13_C[c13_i79];
    }

    for (c13_i80 = 0; c13_i80 < 16; c13_i80++) {
      c13_C[c13_i80] = c13_w_H_b[c13_i80];
    }

    for (c13_i81 = 0; c13_i81 < 16; c13_i81++) {
      c13_w_H_b[c13_i81] = c13_C[c13_i81];
    }

    for (c13_i82 = 0; c13_i82 < 4; c13_i82++) {
      c13_i83 = 0;
      for (c13_i84 = 0; c13_i84 < 4; c13_i84++) {
        c13_w_H_b[c13_i83 + c13_i82] = 0.0;
        c13_i85 = 0;
        for (c13_i86 = 0; c13_i86 < 4; c13_i86++) {
          c13_w_H_b[c13_i83 + c13_i82] += c13_hoistedGlobal[c13_i85 + c13_i82] *
            c13_b[c13_i86 + c13_i83];
          c13_i85 += 4;
        }

        c13_i83 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 27);
    if (CV_SCRIPT_IF(0, 2, c13_t > c13_b_sm->tBalancing)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 28);
      chartInstance->c13_state = 2.0;
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 29);
      if (CV_SCRIPT_IF(0, 3, c13_b_sm->demoOnlyRightFoot)) {
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 30);
        for (c13_i87 = 0; c13_i87 < 16; c13_i87++) {
          c13_hoistedGlobal[c13_i87] = chartInstance->c13_w_H_fixedLink[c13_i87];
        }

        for (c13_i88 = 0; c13_i88 < 16; c13_i88++) {
          c13_b[c13_i88] = c13_l_sole_H_b[c13_i88];
        }

        c13_eml_scalar_eg(chartInstance);
        c13_eml_scalar_eg(chartInstance);
        for (c13_i89 = 0; c13_i89 < 4; c13_i89++) {
          c13_i90 = 0;
          for (c13_i91 = 0; c13_i91 < 4; c13_i91++) {
            c13_C[c13_i90 + c13_i89] = 0.0;
            c13_i92 = 0;
            for (c13_i93 = 0; c13_i93 < 4; c13_i93++) {
              c13_C[c13_i90 + c13_i89] += c13_hoistedGlobal[c13_i92 + c13_i89] *
                c13_b[c13_i93 + c13_i90];
              c13_i92 += 4;
            }

            c13_i90 += 4;
          }
        }

        for (c13_i94 = 0; c13_i94 < 16; c13_i94++) {
          c13_b_C[c13_i94] = c13_C[c13_i94];
        }

        for (c13_i95 = 0; c13_i95 < 16; c13_i95++) {
          c13_b_r_sole_H_b[c13_i95] = c13_r_sole_H_b[c13_i95];
        }

        c13_mrdivide(chartInstance, c13_b_C, c13_b_r_sole_H_b, c13_dv10);
        for (c13_i96 = 0; c13_i96 < 16; c13_i96++) {
          chartInstance->c13_w_H_fixedLink[c13_i96] = c13_dv10[c13_i96];
        }

        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 31);
        chartInstance->c13_state = 8.0;
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 38);
  if (CV_SCRIPT_IF(0, 4, chartInstance->c13_state == 2.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 39);
    for (c13_i97 = 0; c13_i97 < 16; c13_i97++) {
      c13_hoistedGlobal[c13_i97] = chartInstance->c13_w_H_fixedLink[c13_i97];
    }

    for (c13_i98 = 0; c13_i98 < 16; c13_i98++) {
      c13_b[c13_i98] = c13_l_sole_H_b[c13_i98];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i99 = 0; c13_i99 < 16; c13_i99++) {
      c13_w_H_b[c13_i99] = 0.0;
    }

    for (c13_i100 = 0; c13_i100 < 16; c13_i100++) {
      c13_w_H_b[c13_i100] = 0.0;
    }

    for (c13_i101 = 0; c13_i101 < 16; c13_i101++) {
      c13_C[c13_i101] = c13_w_H_b[c13_i101];
    }

    for (c13_i102 = 0; c13_i102 < 16; c13_i102++) {
      c13_w_H_b[c13_i102] = c13_C[c13_i102];
    }

    for (c13_i103 = 0; c13_i103 < 16; c13_i103++) {
      c13_C[c13_i103] = c13_w_H_b[c13_i103];
    }

    for (c13_i104 = 0; c13_i104 < 16; c13_i104++) {
      c13_w_H_b[c13_i104] = c13_C[c13_i104];
    }

    for (c13_i105 = 0; c13_i105 < 4; c13_i105++) {
      c13_i106 = 0;
      for (c13_i107 = 0; c13_i107 < 4; c13_i107++) {
        c13_w_H_b[c13_i106 + c13_i105] = 0.0;
        c13_i108 = 0;
        for (c13_i109 = 0; c13_i109 < 4; c13_i109++) {
          c13_w_H_b[c13_i106 + c13_i105] += c13_hoistedGlobal[c13_i108 +
            c13_i105] * c13_b[c13_i109 + c13_i106];
          c13_i108 += 4;
        }

        c13_i106 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 44);
    c13_i110 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i111 = 0; c13_i111 < 2; c13_i111++) {
      c13_dv11[c13_i111] = chartInstance->c13_w_H_fixedLink[c13_i111 + 12];
    }

    c13_dv11[2] = c13_CoM_0[2];
    for (c13_i112 = 0; c13_i112 < 3; c13_i112++) {
      c13_CoMDes[c13_i112] = c13_dv11[c13_i112] + c13_b_sm->com.states[c13_i110
        + 13 * c13_i112];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 46);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i113 = 0; c13_i113 < 23; c13_i113++) {
      c13_impedances[c13_i113] = c13_b_gain->impedances[c13_i113];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 47);
    c13_i114 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i115 = 0; c13_i115 < 3; c13_i115++) {
      c13_kpCom[c13_i115] = c13_b_gain->PCOM[c13_i114 + 3 * c13_i115];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 48);
    c13_i116 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i117 = 0; c13_i117 < 3; c13_i117++) {
      c13_kdCom[c13_i117] = c13_b_gain->DCOM[c13_i116 + 3 * c13_i117];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 50);
    for (c13_i118 = 0; c13_i118 < 16; c13_i118++) {
      c13_dv12[c13_i118] = chartInstance->c13_w_H_fixedLink[c13_i118];
    }

    for (c13_i119 = 0; c13_i119 < 3; c13_i119++) {
      c13_b_CoMDes[c13_i119] = c13_CoMDes[c13_i119];
    }

    c13_b_CoMDes[3] = 1.0;
    c13_mldivide(chartInstance, c13_dv12, c13_b_CoMDes, c13_dv13);
    for (c13_i120 = 0; c13_i120 < 4; c13_i120++) {
      c13_fixed_link_CoMDes[c13_i120] = c13_dv13[c13_i120];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 52);
    for (c13_i121 = 0; c13_i121 < 3; c13_i121++) {
      c13_CoMError[c13_i121] = c13_fixed_link_CoMDes[c13_i121] -
        c13_l_sole_CoM[c13_i121];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 54);
    c13_i122 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i123 = 0; c13_i123 < 23; c13_i123++) {
      c13_qDes[c13_i123] = c13_b_sm->joints.states[c13_i122 + 13 * c13_i123];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 56);
    guard8 = FALSE;
    if (CV_SCRIPT_COND(0, 3, c13_norm(chartInstance, c13_CoMError[1]) <
                       c13_b_sm->com.threshold)) {
      if (CV_SCRIPT_COND(0, 4, c13_wrench_rightFoot[2] <
                         c13_b_sm->wrench.thresholdContactOff)) {
        CV_SCRIPT_MCDC(0, 1, TRUE);
        CV_SCRIPT_IF(0, 5, TRUE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 57);
        chartInstance->c13_state = 3.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 58);
        chartInstance->c13_tSwitch = c13_t;
      } else {
        guard8 = TRUE;
      }
    } else {
      guard8 = TRUE;
    }

    if (guard8 == TRUE) {
      CV_SCRIPT_MCDC(0, 1, FALSE);
      CV_SCRIPT_IF(0, 5, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 63);
  if (CV_SCRIPT_IF(0, 6, chartInstance->c13_state == 3.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 64);
    for (c13_i124 = 0; c13_i124 < 16; c13_i124++) {
      c13_hoistedGlobal[c13_i124] = chartInstance->c13_w_H_fixedLink[c13_i124];
    }

    for (c13_i125 = 0; c13_i125 < 16; c13_i125++) {
      c13_b[c13_i125] = c13_l_sole_H_b[c13_i125];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i126 = 0; c13_i126 < 16; c13_i126++) {
      c13_w_H_b[c13_i126] = 0.0;
    }

    for (c13_i127 = 0; c13_i127 < 16; c13_i127++) {
      c13_w_H_b[c13_i127] = 0.0;
    }

    for (c13_i128 = 0; c13_i128 < 16; c13_i128++) {
      c13_C[c13_i128] = c13_w_H_b[c13_i128];
    }

    for (c13_i129 = 0; c13_i129 < 16; c13_i129++) {
      c13_w_H_b[c13_i129] = c13_C[c13_i129];
    }

    for (c13_i130 = 0; c13_i130 < 16; c13_i130++) {
      c13_C[c13_i130] = c13_w_H_b[c13_i130];
    }

    for (c13_i131 = 0; c13_i131 < 16; c13_i131++) {
      c13_w_H_b[c13_i131] = c13_C[c13_i131];
    }

    for (c13_i132 = 0; c13_i132 < 4; c13_i132++) {
      c13_i133 = 0;
      for (c13_i134 = 0; c13_i134 < 4; c13_i134++) {
        c13_w_H_b[c13_i133 + c13_i132] = 0.0;
        c13_i135 = 0;
        for (c13_i136 = 0; c13_i136 < 4; c13_i136++) {
          c13_w_H_b[c13_i133 + c13_i132] += c13_hoistedGlobal[c13_i135 +
            c13_i132] * c13_b[c13_i136 + c13_i133];
          c13_i135 += 4;
        }

        c13_i133 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 69);
    c13_i137 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i138 = 0; c13_i138 < 2; c13_i138++) {
      c13_dv14[c13_i138] = chartInstance->c13_w_H_fixedLink[c13_i138 + 12];
    }

    c13_dv14[2] = c13_CoM_0[2];
    for (c13_i139 = 0; c13_i139 < 3; c13_i139++) {
      c13_CoMDes[c13_i139] = c13_dv14[c13_i139] + c13_b_sm->com.states[c13_i137
        + 13 * c13_i139];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 72);
    for (c13_i140 = 0; c13_i140 < 2; c13_i140++) {
      c13_constraints[c13_i140] = 1.0 - (real_T)c13_i140;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 74);
    c13_i141 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i142 = 0; c13_i142 < 23; c13_i142++) {
      c13_qDes[c13_i142] = c13_b_sm->joints.states[c13_i141 + 13 * c13_i142];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 75);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i143 = 0; c13_i143 < 23; c13_i143++) {
      c13_impedances[c13_i143] = c13_b_gain->impedances[c13_i143];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 76);
    c13_i144 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i145 = 0; c13_i145 < 3; c13_i145++) {
      c13_kpCom[c13_i145] = c13_b_gain->PCOM[c13_i144 + 3 * c13_i145];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 77);
    c13_i146 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i147 = 0; c13_i147 < 3; c13_i147++) {
      c13_kdCom[c13_i147] = c13_b_gain->DCOM[c13_i146 + 3 * c13_i147];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 79);
    if (CV_SCRIPT_IF(0, 7, c13_t > chartInstance->c13_tSwitch + c13_b_sm->DT)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 80);
      chartInstance->c13_state = 4.0;
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 81);
      chartInstance->c13_tSwitch = c13_t;
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 82);
      if (CV_SCRIPT_IF(0, 8, c13_b_sm->skipYoga)) {
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 83);
        chartInstance->c13_state = 5.0;
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 89);
  if (CV_SCRIPT_IF(0, 9, chartInstance->c13_state == 4.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 90);
    for (c13_i148 = 0; c13_i148 < 16; c13_i148++) {
      c13_hoistedGlobal[c13_i148] = chartInstance->c13_w_H_fixedLink[c13_i148];
    }

    for (c13_i149 = 0; c13_i149 < 16; c13_i149++) {
      c13_b[c13_i149] = c13_l_sole_H_b[c13_i149];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i150 = 0; c13_i150 < 16; c13_i150++) {
      c13_w_H_b[c13_i150] = 0.0;
    }

    for (c13_i151 = 0; c13_i151 < 16; c13_i151++) {
      c13_w_H_b[c13_i151] = 0.0;
    }

    for (c13_i152 = 0; c13_i152 < 16; c13_i152++) {
      c13_C[c13_i152] = c13_w_H_b[c13_i152];
    }

    for (c13_i153 = 0; c13_i153 < 16; c13_i153++) {
      c13_w_H_b[c13_i153] = c13_C[c13_i153];
    }

    for (c13_i154 = 0; c13_i154 < 16; c13_i154++) {
      c13_C[c13_i154] = c13_w_H_b[c13_i154];
    }

    for (c13_i155 = 0; c13_i155 < 16; c13_i155++) {
      c13_w_H_b[c13_i155] = c13_C[c13_i155];
    }

    for (c13_i156 = 0; c13_i156 < 4; c13_i156++) {
      c13_i157 = 0;
      for (c13_i158 = 0; c13_i158 < 4; c13_i158++) {
        c13_w_H_b[c13_i157 + c13_i156] = 0.0;
        c13_i159 = 0;
        for (c13_i160 = 0; c13_i160 < 4; c13_i160++) {
          c13_w_H_b[c13_i157 + c13_i156] += c13_hoistedGlobal[c13_i159 +
            c13_i156] * c13_b[c13_i160 + c13_i157];
          c13_i159 += 4;
        }

        c13_i157 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 96);
    c13_i161 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i162 = 0; c13_i162 < 2; c13_i162++) {
      c13_dv15[c13_i162] = chartInstance->c13_w_H_fixedLink[c13_i162 + 12];
    }

    c13_dv15[2] = c13_CoM_0[2];
    for (c13_i163 = 0; c13_i163 < 3; c13_i163++) {
      c13_CoMDes[c13_i163] = c13_dv15[c13_i163] + c13_b_sm->com.states[c13_i161
        + 13 * c13_i163];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 98);
    for (c13_i164 = 0; c13_i164 < 2; c13_i164++) {
      c13_constraints[c13_i164] = 1.0 - (real_T)c13_i164;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 100);
    c13_i165 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i166 = 0; c13_i166 < 23; c13_i166++) {
      c13_qDes[c13_i166] = c13_b_sm->joints.states[c13_i165 + 13 * c13_i166];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 101);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i167 = 0; c13_i167 < 23; c13_i167++) {
      c13_impedances[c13_i167] = c13_b_gain->impedances[c13_i167];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 102);
    c13_i168 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i169 = 0; c13_i169 < 3; c13_i169++) {
      c13_kpCom[c13_i169] = c13_b_gain->PCOM[c13_i168 + 3 * c13_i169];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 103);
    c13_i170 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i171 = 0; c13_i171 < 3; c13_i171++) {
      c13_kdCom[c13_i171] = c13_b_gain->DCOM[c13_i170 + 3 * c13_i171];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 105);
    c13_i = 1.0;
    c13_b_i = 0;
    while (c13_b_i < 7) {
      c13_i = 1.0 + (real_T)c13_b_i;
      CV_SCRIPT_FOR(0, 0, 1);
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 106);
      guard7 = FALSE;
      if (CV_SCRIPT_COND(0, 5, c13_t > c13_b_sm->joints.pointsL[(int32_T)(real_T)
                         _SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.pointsL",
            (int32_T)_SFD_INTEGER_CHECK("i", c13_i), 1, 8, 1, 0) - 1] +
                         chartInstance->c13_tSwitch)) {
        if (CV_SCRIPT_COND(0, 6, c13_t <= c13_b_sm->joints.pointsL[(int32_T)
                           (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
              "sm.joints.pointsL", (int32_T)_SFD_INTEGER_CHECK("i+1", c13_i +
               1.0), 1, 8, 1, 0) - 1] + chartInstance->c13_tSwitch)) {
          CV_SCRIPT_MCDC(0, 2, TRUE);
          CV_SCRIPT_IF(0, 10, TRUE);
          _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 107);
          c13_c_i = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
            "sm.joints.pointsL", (int32_T)_SFD_INTEGER_CHECK("i", c13_i), 1, 8,
            1, 0) - 1;
          for (c13_i172 = 0; c13_i172 < 23; c13_i172++) {
            c13_qDes[c13_i172] = c13_b_sm->joints.pointsL[c13_c_i + ((1 +
              c13_i172) << 3)];
          }
        } else {
          guard7 = TRUE;
        }
      } else {
        guard7 = TRUE;
      }

      if (guard7 == TRUE) {
        CV_SCRIPT_MCDC(0, 2, FALSE);
        CV_SCRIPT_IF(0, 10, FALSE);
      }

      c13_b_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    CV_SCRIPT_FOR(0, 0, 0);
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 110);
    if (CV_SCRIPT_IF(0, 11, c13_t > c13_b_sm->joints.pointsL[7] +
                     chartInstance->c13_tSwitch)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 111);
      c13_i173 = 0;
      for (c13_i174 = 0; c13_i174 < 23; c13_i174++) {
        c13_qDes[c13_i174] = c13_b_sm->joints.pointsL[c13_i173 + 15];
        c13_i173 += 8;
      }

      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 112);
      if (CV_SCRIPT_IF(0, 12, c13_t > ((c13_b_sm->joints.pointsL[7] +
             chartInstance->c13_tSwitch) + c13_b_sm->jointsSmoothingTimes
            [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
             "sm.jointsSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
              chartInstance->c13_state), 1, 13, 1, 0) - 1]) +
                       c13_b_sm->joints.pauseTimeLastPostureL)) {
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 113);
        chartInstance->c13_state = 5.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 114);
        chartInstance->c13_tSwitch = c13_t;
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 120);
  if (CV_SCRIPT_IF(0, 13, chartInstance->c13_state == 5.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 121);
    for (c13_i175 = 0; c13_i175 < 16; c13_i175++) {
      c13_hoistedGlobal[c13_i175] = chartInstance->c13_w_H_fixedLink[c13_i175];
    }

    for (c13_i176 = 0; c13_i176 < 16; c13_i176++) {
      c13_b[c13_i176] = c13_l_sole_H_b[c13_i176];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i177 = 0; c13_i177 < 16; c13_i177++) {
      c13_w_H_b[c13_i177] = 0.0;
    }

    for (c13_i178 = 0; c13_i178 < 16; c13_i178++) {
      c13_w_H_b[c13_i178] = 0.0;
    }

    for (c13_i179 = 0; c13_i179 < 16; c13_i179++) {
      c13_C[c13_i179] = c13_w_H_b[c13_i179];
    }

    for (c13_i180 = 0; c13_i180 < 16; c13_i180++) {
      c13_w_H_b[c13_i180] = c13_C[c13_i180];
    }

    for (c13_i181 = 0; c13_i181 < 16; c13_i181++) {
      c13_C[c13_i181] = c13_w_H_b[c13_i181];
    }

    for (c13_i182 = 0; c13_i182 < 16; c13_i182++) {
      c13_w_H_b[c13_i182] = c13_C[c13_i182];
    }

    for (c13_i183 = 0; c13_i183 < 4; c13_i183++) {
      c13_i184 = 0;
      for (c13_i185 = 0; c13_i185 < 4; c13_i185++) {
        c13_w_H_b[c13_i184 + c13_i183] = 0.0;
        c13_i186 = 0;
        for (c13_i187 = 0; c13_i187 < 4; c13_i187++) {
          c13_w_H_b[c13_i184 + c13_i183] += c13_hoistedGlobal[c13_i186 +
            c13_i183] * c13_b[c13_i187 + c13_i184];
          c13_i186 += 4;
        }

        c13_i184 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 126);
    c13_i188 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i189 = 0; c13_i189 < 2; c13_i189++) {
      c13_dv16[c13_i189] = chartInstance->c13_w_H_fixedLink[c13_i189 + 12];
    }

    c13_dv16[2] = c13_CoM_0[2];
    for (c13_i190 = 0; c13_i190 < 3; c13_i190++) {
      c13_CoMDes[c13_i190] = c13_dv16[c13_i190] + c13_b_sm->com.states[c13_i188
        + 13 * c13_i190];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 128U);
    for (c13_i191 = 0; c13_i191 < 2; c13_i191++) {
      c13_constraints[c13_i191] = 1.0 - (real_T)c13_i191;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 130U);
    c13_i192 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i193 = 0; c13_i193 < 23; c13_i193++) {
      c13_qDes[c13_i193] = c13_b_sm->joints.states[c13_i192 + 13 * c13_i193];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 131U);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i194 = 0; c13_i194 < 23; c13_i194++) {
      c13_impedances[c13_i194] = c13_b_gain->impedances[c13_i194];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 132U);
    c13_i195 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i196 = 0; c13_i196 < 3; c13_i196++) {
      c13_kpCom[c13_i196] = c13_b_gain->PCOM[c13_i195 + 3 * c13_i196];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 133U);
    c13_i197 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i198 = 0; c13_i198 < 3; c13_i198++) {
      c13_kdCom[c13_i198] = c13_b_gain->DCOM[c13_i197 + 3 * c13_i198];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 135U);
    for (c13_i199 = 0; c13_i199 < 6; c13_i199++) {
      c13_qTildeRLeg[c13_i199] = c13_qj[c13_i199 + 17] - c13_qDes[c13_i199 + 17];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 137U);
    for (c13_i200 = 0; c13_i200 < 6; c13_i200++) {
      c13_qTildeLLeg[c13_i200] = c13_qj[c13_i200 + 11] - c13_qDes[c13_i200 + 11];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 139U);
    for (c13_i201 = 0; c13_i201 < 6; c13_i201++) {
      c13_b_qTildeRLeg[c13_i201] = c13_qTildeRLeg[c13_i201];
    }

    c13_a = c13_b_norm(chartInstance, c13_b_qTildeRLeg);
    c13_y = c13_a * 180.0;
    c13_A = c13_y;
    c13_x = c13_A;
    c13_b_x = c13_x;
    c13_b_y = c13_b_x / 3.1415926535897931;
    guard6 = FALSE;
    if (CV_SCRIPT_COND(0, 7, c13_b_y < c13_b_sm->joints.thresholdNotInContact))
    {
      for (c13_i202 = 0; c13_i202 < 6; c13_i202++) {
        c13_b_qTildeLLeg[c13_i202] = c13_qTildeLLeg[c13_i202];
      }

      c13_b_a = c13_b_norm(chartInstance, c13_b_qTildeLLeg);
      c13_c_y = c13_b_a * 180.0;
      c13_b_A = c13_c_y;
      c13_c_x = c13_b_A;
      c13_d_x = c13_c_x;
      c13_d_y = c13_d_x / 3.1415926535897931;
      if (CV_SCRIPT_COND(0, 8, c13_d_y < c13_b_sm->joints.thresholdInContact)) {
        CV_SCRIPT_MCDC(0, 3, TRUE);
        CV_SCRIPT_IF(0, 14, TRUE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 140U);
        chartInstance->c13_state = 6.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 141U);
        chartInstance->c13_tSwitch = c13_t;
      } else {
        guard6 = TRUE;
      }
    } else {
      guard6 = TRUE;
    }

    if (guard6 == TRUE) {
      CV_SCRIPT_MCDC(0, 3, FALSE);
      CV_SCRIPT_IF(0, 14, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 146U);
  if (CV_SCRIPT_IF(0, 15, chartInstance->c13_state == 6.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 147U);
    for (c13_i203 = 0; c13_i203 < 16; c13_i203++) {
      c13_hoistedGlobal[c13_i203] = chartInstance->c13_w_H_fixedLink[c13_i203];
    }

    for (c13_i204 = 0; c13_i204 < 16; c13_i204++) {
      c13_b[c13_i204] = c13_l_sole_H_b[c13_i204];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i205 = 0; c13_i205 < 16; c13_i205++) {
      c13_w_H_b[c13_i205] = 0.0;
    }

    for (c13_i206 = 0; c13_i206 < 16; c13_i206++) {
      c13_w_H_b[c13_i206] = 0.0;
    }

    for (c13_i207 = 0; c13_i207 < 16; c13_i207++) {
      c13_C[c13_i207] = c13_w_H_b[c13_i207];
    }

    for (c13_i208 = 0; c13_i208 < 16; c13_i208++) {
      c13_w_H_b[c13_i208] = c13_C[c13_i208];
    }

    for (c13_i209 = 0; c13_i209 < 16; c13_i209++) {
      c13_C[c13_i209] = c13_w_H_b[c13_i209];
    }

    for (c13_i210 = 0; c13_i210 < 16; c13_i210++) {
      c13_w_H_b[c13_i210] = c13_C[c13_i210];
    }

    for (c13_i211 = 0; c13_i211 < 4; c13_i211++) {
      c13_i212 = 0;
      for (c13_i213 = 0; c13_i213 < 4; c13_i213++) {
        c13_w_H_b[c13_i212 + c13_i211] = 0.0;
        c13_i214 = 0;
        for (c13_i215 = 0; c13_i215 < 4; c13_i215++) {
          c13_w_H_b[c13_i212 + c13_i211] += c13_hoistedGlobal[c13_i214 +
            c13_i211] * c13_b[c13_i215 + c13_i212];
          c13_i214 += 4;
        }

        c13_i212 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 152U);
    c13_i216 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i217 = 0; c13_i217 < 2; c13_i217++) {
      c13_dv17[c13_i217] = chartInstance->c13_w_H_fixedLink[c13_i217 + 12];
    }

    c13_dv17[2] = c13_CoM_0[2];
    for (c13_i218 = 0; c13_i218 < 3; c13_i218++) {
      c13_CoMDes[c13_i218] = c13_dv17[c13_i218] + c13_b_sm->com.states[c13_i216
        + 13 * c13_i218];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 154U);
    for (c13_i219 = 0; c13_i219 < 2; c13_i219++) {
      c13_constraints[c13_i219] = 1.0 - (real_T)c13_i219;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 156U);
    c13_i220 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i221 = 0; c13_i221 < 23; c13_i221++) {
      c13_qDes[c13_i221] = c13_b_sm->joints.states[c13_i220 + 13 * c13_i221];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 157U);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i222 = 0; c13_i222 < 23; c13_i222++) {
      c13_impedances[c13_i222] = c13_b_gain->impedances[c13_i222];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 158U);
    c13_i223 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i224 = 0; c13_i224 < 3; c13_i224++) {
      c13_kpCom[c13_i224] = c13_b_gain->PCOM[c13_i223 + 3 * c13_i224];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 159U);
    c13_i225 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i226 = 0; c13_i226 < 3; c13_i226++) {
      c13_kdCom[c13_i226] = c13_b_gain->DCOM[c13_i225 + 3 * c13_i226];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 161U);
    if (CV_SCRIPT_IF(0, 16, c13_wrench_rightFoot[2] >
                     c13_b_sm->wrench.thresholdContactOn)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 162U);
      chartInstance->c13_state = 7.0;
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 163U);
      chartInstance->c13_tSwitch = c13_t;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 168U);
  if (CV_SCRIPT_IF(0, 17, chartInstance->c13_state == 7.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 169U);
    for (c13_i227 = 0; c13_i227 < 16; c13_i227++) {
      c13_hoistedGlobal[c13_i227] = chartInstance->c13_w_H_fixedLink[c13_i227];
    }

    for (c13_i228 = 0; c13_i228 < 16; c13_i228++) {
      c13_b[c13_i228] = c13_l_sole_H_b[c13_i228];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i229 = 0; c13_i229 < 16; c13_i229++) {
      c13_w_H_b[c13_i229] = 0.0;
    }

    for (c13_i230 = 0; c13_i230 < 16; c13_i230++) {
      c13_w_H_b[c13_i230] = 0.0;
    }

    for (c13_i231 = 0; c13_i231 < 16; c13_i231++) {
      c13_C[c13_i231] = c13_w_H_b[c13_i231];
    }

    for (c13_i232 = 0; c13_i232 < 16; c13_i232++) {
      c13_w_H_b[c13_i232] = c13_C[c13_i232];
    }

    for (c13_i233 = 0; c13_i233 < 16; c13_i233++) {
      c13_C[c13_i233] = c13_w_H_b[c13_i233];
    }

    for (c13_i234 = 0; c13_i234 < 16; c13_i234++) {
      c13_w_H_b[c13_i234] = c13_C[c13_i234];
    }

    for (c13_i235 = 0; c13_i235 < 4; c13_i235++) {
      c13_i236 = 0;
      for (c13_i237 = 0; c13_i237 < 4; c13_i237++) {
        c13_w_H_b[c13_i236 + c13_i235] = 0.0;
        c13_i238 = 0;
        for (c13_i239 = 0; c13_i239 < 4; c13_i239++) {
          c13_w_H_b[c13_i236 + c13_i235] += c13_hoistedGlobal[c13_i238 +
            c13_i235] * c13_b[c13_i239 + c13_i236];
          c13_i238 += 4;
        }

        c13_i236 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 172U);
    c13_i240 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i241 = 0; c13_i241 < 3; c13_i241++) {
      c13_CoMDes[c13_i241] = c13_CoM_0[c13_i241] + c13_b_sm->com.states[c13_i240
        + 13 * c13_i241];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 175U);
    for (c13_i242 = 0; c13_i242 < 2; c13_i242++) {
      c13_constraints[c13_i242] = 1.0;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 176U);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i243 = 0; c13_i243 < 23; c13_i243++) {
      c13_impedances[c13_i243] = c13_b_gain->impedances[c13_i243];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 177U);
    c13_i244 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i245 = 0; c13_i245 < 3; c13_i245++) {
      c13_kpCom[c13_i245] = c13_b_gain->PCOM[c13_i244 + 3 * c13_i245];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 178U);
    c13_i246 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i247 = 0; c13_i247 < 3; c13_i247++) {
      c13_kdCom[c13_i247] = c13_b_gain->DCOM[c13_i246 + 3 * c13_i247];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 179U);
    c13_b_b = c13_b_sm->com.threshold;
    c13_e_y = 10.0 * c13_b_b;
    for (c13_i248 = 0; c13_i248 < 2; c13_i248++) {
      c13_b_l_sole_CoM[c13_i248] = c13_l_sole_CoM[c13_i248] -
        c13_CoMDes[c13_i248];
    }

    guard4 = FALSE;
    guard5 = FALSE;
    if (CV_SCRIPT_COND(0, 9, c13_c_norm(chartInstance, c13_b_l_sole_CoM) <
                       c13_e_y)) {
      if (CV_SCRIPT_COND(0, 10, c13_b_sm->yogaAlsoOnRightFoot)) {
        if (CV_SCRIPT_COND(0, 11, c13_t > chartInstance->c13_tSwitch +
                           c13_b_sm->tBalancing)) {
          CV_SCRIPT_MCDC(0, 4, TRUE);
          CV_SCRIPT_IF(0, 18, TRUE);
          _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 180U);
          for (c13_i249 = 0; c13_i249 < 16; c13_i249++) {
            c13_hoistedGlobal[c13_i249] = chartInstance->
              c13_w_H_fixedLink[c13_i249];
          }

          for (c13_i250 = 0; c13_i250 < 16; c13_i250++) {
            c13_b[c13_i250] = c13_l_sole_H_b[c13_i250];
          }

          c13_eml_scalar_eg(chartInstance);
          c13_eml_scalar_eg(chartInstance);
          for (c13_i251 = 0; c13_i251 < 4; c13_i251++) {
            c13_i252 = 0;
            for (c13_i253 = 0; c13_i253 < 4; c13_i253++) {
              c13_C[c13_i252 + c13_i251] = 0.0;
              c13_i254 = 0;
              for (c13_i255 = 0; c13_i255 < 4; c13_i255++) {
                c13_C[c13_i252 + c13_i251] += c13_hoistedGlobal[c13_i254 +
                  c13_i251] * c13_b[c13_i255 + c13_i252];
                c13_i254 += 4;
              }

              c13_i252 += 4;
            }
          }

          for (c13_i256 = 0; c13_i256 < 16; c13_i256++) {
            c13_c_C[c13_i256] = c13_C[c13_i256];
          }

          for (c13_i257 = 0; c13_i257 < 16; c13_i257++) {
            c13_c_r_sole_H_b[c13_i257] = c13_r_sole_H_b[c13_i257];
          }

          c13_mrdivide(chartInstance, c13_c_C, c13_c_r_sole_H_b, c13_dv18);
          for (c13_i258 = 0; c13_i258 < 16; c13_i258++) {
            chartInstance->c13_w_H_fixedLink[c13_i258] = c13_dv18[c13_i258];
          }

          _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 181U);
          chartInstance->c13_state = 8.0;
          _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 182U);
          chartInstance->c13_tSwitch = c13_t;
        } else {
          guard4 = TRUE;
        }
      } else {
        guard5 = TRUE;
      }
    } else {
      guard5 = TRUE;
    }

    if (guard5 == TRUE) {
      guard4 = TRUE;
    }

    if (guard4 == TRUE) {
      CV_SCRIPT_MCDC(0, 4, FALSE);
      CV_SCRIPT_IF(0, 18, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 192U);
  if (CV_SCRIPT_IF(0, 19, chartInstance->c13_state == 8.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 193U);
    for (c13_i259 = 0; c13_i259 < 2; c13_i259++) {
      c13_constraints[c13_i259] = 1.0;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 194U);
    for (c13_i260 = 0; c13_i260 < 16; c13_i260++) {
      c13_hoistedGlobal[c13_i260] = chartInstance->c13_w_H_fixedLink[c13_i260];
    }

    for (c13_i261 = 0; c13_i261 < 16; c13_i261++) {
      c13_b[c13_i261] = c13_r_sole_H_b[c13_i261];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i262 = 0; c13_i262 < 16; c13_i262++) {
      c13_w_H_b[c13_i262] = 0.0;
    }

    for (c13_i263 = 0; c13_i263 < 16; c13_i263++) {
      c13_w_H_b[c13_i263] = 0.0;
    }

    for (c13_i264 = 0; c13_i264 < 16; c13_i264++) {
      c13_C[c13_i264] = c13_w_H_b[c13_i264];
    }

    for (c13_i265 = 0; c13_i265 < 16; c13_i265++) {
      c13_w_H_b[c13_i265] = c13_C[c13_i265];
    }

    for (c13_i266 = 0; c13_i266 < 16; c13_i266++) {
      c13_C[c13_i266] = c13_w_H_b[c13_i266];
    }

    for (c13_i267 = 0; c13_i267 < 16; c13_i267++) {
      c13_w_H_b[c13_i267] = c13_C[c13_i267];
    }

    for (c13_i268 = 0; c13_i268 < 4; c13_i268++) {
      c13_i269 = 0;
      for (c13_i270 = 0; c13_i270 < 4; c13_i270++) {
        c13_w_H_b[c13_i269 + c13_i268] = 0.0;
        c13_i271 = 0;
        for (c13_i272 = 0; c13_i272 < 4; c13_i272++) {
          c13_w_H_b[c13_i269 + c13_i268] += c13_hoistedGlobal[c13_i271 +
            c13_i268] * c13_b[c13_i272 + c13_i269];
          c13_i271 += 4;
        }

        c13_i269 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 199U);
    c13_i273 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i274 = 0; c13_i274 < 2; c13_i274++) {
      c13_dv19[c13_i274] = chartInstance->c13_w_H_fixedLink[c13_i274 + 12];
    }

    c13_dv19[2] = c13_CoM_0[2];
    for (c13_i275 = 0; c13_i275 < 3; c13_i275++) {
      c13_CoMDes[c13_i275] = c13_dv19[c13_i275] + c13_b_sm->com.states[c13_i273
        + 13 * c13_i275];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 201U);
    for (c13_i276 = 0; c13_i276 < 16; c13_i276++) {
      c13_dv20[c13_i276] = chartInstance->c13_w_H_fixedLink[c13_i276];
    }

    for (c13_i277 = 0; c13_i277 < 3; c13_i277++) {
      c13_c_CoMDes[c13_i277] = c13_CoMDes[c13_i277];
    }

    c13_c_CoMDes[3] = 1.0;
    c13_mldivide(chartInstance, c13_dv20, c13_c_CoMDes, c13_dv21);
    for (c13_i278 = 0; c13_i278 < 4; c13_i278++) {
      c13_fixed_link_CoMDes[c13_i278] = c13_dv21[c13_i278];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 203U);
    for (c13_i279 = 0; c13_i279 < 3; c13_i279++) {
      c13_CoMError[c13_i279] = c13_fixed_link_CoMDes[c13_i279] -
        c13_r_sole_CoM[c13_i279];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 205U);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i280 = 0; c13_i280 < 23; c13_i280++) {
      c13_impedances[c13_i280] = c13_b_gain->impedances[c13_i280];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 206U);
    c13_i281 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i282 = 0; c13_i282 < 3; c13_i282++) {
      c13_kpCom[c13_i282] = c13_b_gain->PCOM[c13_i281 + 3 * c13_i282];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 207U);
    c13_i283 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i284 = 0; c13_i284 < 3; c13_i284++) {
      c13_kdCom[c13_i284] = c13_b_gain->DCOM[c13_i283 + 3 * c13_i284];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 208U);
    c13_i285 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i286 = 0; c13_i286 < 23; c13_i286++) {
      c13_qDes[c13_i286] = c13_b_sm->joints.states[c13_i285 + 13 * c13_i286];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 210U);
    guard3 = FALSE;
    if (CV_SCRIPT_COND(0, 12, c13_norm(chartInstance, c13_CoMError[1]) <
                       c13_b_sm->com.threshold)) {
      if (CV_SCRIPT_COND(0, 13, c13_wrench_leftFoot[2] <
                         c13_b_sm->wrench.thresholdContactOff)) {
        CV_SCRIPT_MCDC(0, 5, TRUE);
        CV_SCRIPT_IF(0, 20, TRUE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 211U);
        chartInstance->c13_state = 9.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 212U);
        chartInstance->c13_tSwitch = c13_t;
      } else {
        guard3 = TRUE;
      }
    } else {
      guard3 = TRUE;
    }

    if (guard3 == TRUE) {
      CV_SCRIPT_MCDC(0, 5, FALSE);
      CV_SCRIPT_IF(0, 20, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 218U);
  if (CV_SCRIPT_IF(0, 21, chartInstance->c13_state == 9.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 219U);
    for (c13_i287 = 0; c13_i287 < 2; c13_i287++) {
      c13_constraints[c13_i287] = (real_T)c13_i287;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 220U);
    for (c13_i288 = 0; c13_i288 < 16; c13_i288++) {
      c13_hoistedGlobal[c13_i288] = chartInstance->c13_w_H_fixedLink[c13_i288];
    }

    for (c13_i289 = 0; c13_i289 < 16; c13_i289++) {
      c13_b[c13_i289] = c13_r_sole_H_b[c13_i289];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i290 = 0; c13_i290 < 16; c13_i290++) {
      c13_w_H_b[c13_i290] = 0.0;
    }

    for (c13_i291 = 0; c13_i291 < 16; c13_i291++) {
      c13_w_H_b[c13_i291] = 0.0;
    }

    for (c13_i292 = 0; c13_i292 < 16; c13_i292++) {
      c13_C[c13_i292] = c13_w_H_b[c13_i292];
    }

    for (c13_i293 = 0; c13_i293 < 16; c13_i293++) {
      c13_w_H_b[c13_i293] = c13_C[c13_i293];
    }

    for (c13_i294 = 0; c13_i294 < 16; c13_i294++) {
      c13_C[c13_i294] = c13_w_H_b[c13_i294];
    }

    for (c13_i295 = 0; c13_i295 < 16; c13_i295++) {
      c13_w_H_b[c13_i295] = c13_C[c13_i295];
    }

    for (c13_i296 = 0; c13_i296 < 4; c13_i296++) {
      c13_i297 = 0;
      for (c13_i298 = 0; c13_i298 < 4; c13_i298++) {
        c13_w_H_b[c13_i297 + c13_i296] = 0.0;
        c13_i299 = 0;
        for (c13_i300 = 0; c13_i300 < 4; c13_i300++) {
          c13_w_H_b[c13_i297 + c13_i296] += c13_hoistedGlobal[c13_i299 +
            c13_i296] * c13_b[c13_i300 + c13_i297];
          c13_i299 += 4;
        }

        c13_i297 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 222U);
    c13_i301 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i302 = 0; c13_i302 < 2; c13_i302++) {
      c13_dv22[c13_i302] = chartInstance->c13_w_H_fixedLink[c13_i302 + 12];
    }

    c13_dv22[2] = c13_CoM_0[2];
    for (c13_i303 = 0; c13_i303 < 3; c13_i303++) {
      c13_CoMDes[c13_i303] = c13_dv22[c13_i303] + c13_b_sm->com.states[c13_i301
        + 13 * c13_i303];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 223U);
    c13_i304 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i305 = 0; c13_i305 < 23; c13_i305++) {
      c13_qDes[c13_i305] = c13_b_sm->joints.states[c13_i304 + 13 * c13_i305];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 224U);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i306 = 0; c13_i306 < 23; c13_i306++) {
      c13_impedances[c13_i306] = c13_b_gain->impedances[c13_i306];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 225U);
    c13_i307 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i308 = 0; c13_i308 < 3; c13_i308++) {
      c13_kpCom[c13_i308] = c13_b_gain->PCOM[c13_i307 + 3 * c13_i308];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 226U);
    c13_i309 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i310 = 0; c13_i310 < 3; c13_i310++) {
      c13_kdCom[c13_i310] = c13_b_gain->DCOM[c13_i309 + 3 * c13_i310];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 227U);
    if (CV_SCRIPT_IF(0, 22, c13_t > chartInstance->c13_tSwitch + c13_b_sm->DT))
    {
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 228U);
      chartInstance->c13_state = 10.0;
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 229U);
      chartInstance->c13_tSwitch = c13_t;
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 230U);
      if (CV_SCRIPT_IF(0, 23, c13_b_sm->skipYoga)) {
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 231U);
        chartInstance->c13_state = 11.0;
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 237U);
  if (CV_SCRIPT_IF(0, 24, chartInstance->c13_state == 10.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 238U);
    for (c13_i311 = 0; c13_i311 < 2; c13_i311++) {
      c13_constraints[c13_i311] = (real_T)c13_i311;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 239U);
    for (c13_i312 = 0; c13_i312 < 16; c13_i312++) {
      c13_hoistedGlobal[c13_i312] = chartInstance->c13_w_H_fixedLink[c13_i312];
    }

    for (c13_i313 = 0; c13_i313 < 16; c13_i313++) {
      c13_b[c13_i313] = c13_r_sole_H_b[c13_i313];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i314 = 0; c13_i314 < 16; c13_i314++) {
      c13_w_H_b[c13_i314] = 0.0;
    }

    for (c13_i315 = 0; c13_i315 < 16; c13_i315++) {
      c13_w_H_b[c13_i315] = 0.0;
    }

    for (c13_i316 = 0; c13_i316 < 16; c13_i316++) {
      c13_C[c13_i316] = c13_w_H_b[c13_i316];
    }

    for (c13_i317 = 0; c13_i317 < 16; c13_i317++) {
      c13_w_H_b[c13_i317] = c13_C[c13_i317];
    }

    for (c13_i318 = 0; c13_i318 < 16; c13_i318++) {
      c13_C[c13_i318] = c13_w_H_b[c13_i318];
    }

    for (c13_i319 = 0; c13_i319 < 16; c13_i319++) {
      c13_w_H_b[c13_i319] = c13_C[c13_i319];
    }

    for (c13_i320 = 0; c13_i320 < 4; c13_i320++) {
      c13_i321 = 0;
      for (c13_i322 = 0; c13_i322 < 4; c13_i322++) {
        c13_w_H_b[c13_i321 + c13_i320] = 0.0;
        c13_i323 = 0;
        for (c13_i324 = 0; c13_i324 < 4; c13_i324++) {
          c13_w_H_b[c13_i321 + c13_i320] += c13_hoistedGlobal[c13_i323 +
            c13_i320] * c13_b[c13_i324 + c13_i321];
          c13_i323 += 4;
        }

        c13_i321 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 241U);
    c13_i325 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i326 = 0; c13_i326 < 2; c13_i326++) {
      c13_dv23[c13_i326] = chartInstance->c13_w_H_fixedLink[c13_i326 + 12];
    }

    c13_dv23[2] = c13_CoM_0[2];
    for (c13_i327 = 0; c13_i327 < 3; c13_i327++) {
      c13_CoMDes[c13_i327] = c13_dv23[c13_i327] + c13_b_sm->com.states[c13_i325
        + 13 * c13_i327];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 242U);
    c13_i328 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i329 = 0; c13_i329 < 23; c13_i329++) {
      c13_qDes[c13_i329] = c13_b_sm->joints.states[c13_i328 + 13 * c13_i329];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 243U);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i330 = 0; c13_i330 < 23; c13_i330++) {
      c13_impedances[c13_i330] = c13_b_gain->impedances[c13_i330];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 244U);
    c13_i331 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i332 = 0; c13_i332 < 3; c13_i332++) {
      c13_kpCom[c13_i332] = c13_b_gain->PCOM[c13_i331 + 3 * c13_i332];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 245U);
    c13_i333 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i334 = 0; c13_i334 < 3; c13_i334++) {
      c13_kdCom[c13_i334] = c13_b_gain->DCOM[c13_i333 + 3 * c13_i334];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 247U);
    c13_i = 1.0;
    c13_d_i = 0;
    while (c13_d_i < 7) {
      c13_i = 1.0 + (real_T)c13_d_i;
      CV_SCRIPT_FOR(0, 1, 1);
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 248U);
      guard2 = FALSE;
      if (CV_SCRIPT_COND(0, 14, c13_t > c13_b_sm->joints.pointsR[(int32_T)
                         (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.pointsR",
            (int32_T)_SFD_INTEGER_CHECK("i", c13_i), 1, 8, 1, 0) - 1] +
                         chartInstance->c13_tSwitch)) {
        if (CV_SCRIPT_COND(0, 15, c13_t <= c13_b_sm->joints.pointsR[(int32_T)
                           (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
              "sm.joints.pointsR", (int32_T)_SFD_INTEGER_CHECK("i+1", c13_i +
               1.0), 1, 8, 1, 0) - 1] + chartInstance->c13_tSwitch)) {
          CV_SCRIPT_MCDC(0, 6, TRUE);
          CV_SCRIPT_IF(0, 25, TRUE);
          _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 249U);
          c13_e_i = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
            "sm.joints.pointsR", (int32_T)_SFD_INTEGER_CHECK("i", c13_i), 1, 8,
            1, 0) - 1;
          for (c13_i335 = 0; c13_i335 < 23; c13_i335++) {
            c13_qDes[c13_i335] = c13_b_sm->joints.pointsR[c13_e_i + ((1 +
              c13_i335) << 3)];
          }
        } else {
          guard2 = TRUE;
        }
      } else {
        guard2 = TRUE;
      }

      if (guard2 == TRUE) {
        CV_SCRIPT_MCDC(0, 6, FALSE);
        CV_SCRIPT_IF(0, 25, FALSE);
      }

      c13_d_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    CV_SCRIPT_FOR(0, 1, 0);
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 252U);
    if (CV_SCRIPT_IF(0, 26, c13_t > c13_b_sm->joints.pointsR[7] +
                     chartInstance->c13_tSwitch)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 253U);
      c13_i336 = 0;
      for (c13_i337 = 0; c13_i337 < 23; c13_i337++) {
        c13_qDes[c13_i337] = c13_b_sm->joints.pointsR[c13_i336 + 15];
        c13_i336 += 8;
      }

      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 254U);
      if (CV_SCRIPT_IF(0, 27, c13_t > ((c13_b_sm->joints.pointsR[7] +
             chartInstance->c13_tSwitch) + c13_b_sm->jointsSmoothingTimes
            [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
             "sm.jointsSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
              chartInstance->c13_state), 1, 13, 1, 0) - 1]) +
                       c13_b_sm->joints.pauseTimeLastPostureR)) {
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, MAX_uint8_T);
        chartInstance->c13_state = 11.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 256);
        chartInstance->c13_tSwitch = c13_t;
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 262);
  if (CV_SCRIPT_IF(0, 28, chartInstance->c13_state == 11.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 263);
    for (c13_i338 = 0; c13_i338 < 2; c13_i338++) {
      c13_constraints[c13_i338] = (real_T)c13_i338;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 264);
    for (c13_i339 = 0; c13_i339 < 16; c13_i339++) {
      c13_hoistedGlobal[c13_i339] = chartInstance->c13_w_H_fixedLink[c13_i339];
    }

    for (c13_i340 = 0; c13_i340 < 16; c13_i340++) {
      c13_b[c13_i340] = c13_r_sole_H_b[c13_i340];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i341 = 0; c13_i341 < 16; c13_i341++) {
      c13_w_H_b[c13_i341] = 0.0;
    }

    for (c13_i342 = 0; c13_i342 < 16; c13_i342++) {
      c13_w_H_b[c13_i342] = 0.0;
    }

    for (c13_i343 = 0; c13_i343 < 16; c13_i343++) {
      c13_C[c13_i343] = c13_w_H_b[c13_i343];
    }

    for (c13_i344 = 0; c13_i344 < 16; c13_i344++) {
      c13_w_H_b[c13_i344] = c13_C[c13_i344];
    }

    for (c13_i345 = 0; c13_i345 < 16; c13_i345++) {
      c13_C[c13_i345] = c13_w_H_b[c13_i345];
    }

    for (c13_i346 = 0; c13_i346 < 16; c13_i346++) {
      c13_w_H_b[c13_i346] = c13_C[c13_i346];
    }

    for (c13_i347 = 0; c13_i347 < 4; c13_i347++) {
      c13_i348 = 0;
      for (c13_i349 = 0; c13_i349 < 4; c13_i349++) {
        c13_w_H_b[c13_i348 + c13_i347] = 0.0;
        c13_i350 = 0;
        for (c13_i351 = 0; c13_i351 < 4; c13_i351++) {
          c13_w_H_b[c13_i348 + c13_i347] += c13_hoistedGlobal[c13_i350 +
            c13_i347] * c13_b[c13_i351 + c13_i348];
          c13_i350 += 4;
        }

        c13_i348 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 266);
    c13_i352 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i353 = 0; c13_i353 < 2; c13_i353++) {
      c13_dv24[c13_i353] = chartInstance->c13_w_H_fixedLink[c13_i353 + 12];
    }

    c13_dv24[2] = c13_CoM_0[2];
    for (c13_i354 = 0; c13_i354 < 3; c13_i354++) {
      c13_CoMDes[c13_i354] = c13_dv24[c13_i354] + c13_b_sm->com.states[c13_i352
        + 13 * c13_i354];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 267);
    c13_i355 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i356 = 0; c13_i356 < 23; c13_i356++) {
      c13_qDes[c13_i356] = c13_b_sm->joints.states[c13_i355 + 13 * c13_i356];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 268);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i357 = 0; c13_i357 < 23; c13_i357++) {
      c13_impedances[c13_i357] = c13_b_gain->impedances[c13_i357];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 269);
    c13_i358 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i359 = 0; c13_i359 < 3; c13_i359++) {
      c13_kpCom[c13_i359] = c13_b_gain->PCOM[c13_i358 + 3 * c13_i359];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 270);
    c13_i360 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i361 = 0; c13_i361 < 3; c13_i361++) {
      c13_kdCom[c13_i361] = c13_b_gain->DCOM[c13_i360 + 3 * c13_i361];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 272);
    for (c13_i362 = 0; c13_i362 < 6; c13_i362++) {
      c13_qTildeRLeg[c13_i362] = c13_qj[c13_i362 + 17] - c13_qDes[c13_i362 + 17];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 274);
    for (c13_i363 = 0; c13_i363 < 6; c13_i363++) {
      c13_qTildeLLeg[c13_i363] = c13_qj[c13_i363 + 11] - c13_qDes[c13_i363 + 11];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 276);
    for (c13_i364 = 0; c13_i364 < 6; c13_i364++) {
      c13_c_qTildeRLeg[c13_i364] = c13_qTildeRLeg[c13_i364];
    }

    c13_c_a = c13_b_norm(chartInstance, c13_c_qTildeRLeg);
    c13_f_y = c13_c_a * 180.0;
    c13_c_A = c13_f_y;
    c13_e_x = c13_c_A;
    c13_f_x = c13_e_x;
    c13_g_y = c13_f_x / 3.1415926535897931;
    guard1 = FALSE;
    if (CV_SCRIPT_COND(0, 16, c13_g_y < c13_b_sm->joints.thresholdInContact)) {
      for (c13_i365 = 0; c13_i365 < 6; c13_i365++) {
        c13_c_qTildeLLeg[c13_i365] = c13_qTildeLLeg[c13_i365];
      }

      c13_d_a = c13_b_norm(chartInstance, c13_c_qTildeLLeg);
      c13_h_y = c13_d_a * 180.0;
      c13_d_A = c13_h_y;
      c13_g_x = c13_d_A;
      c13_h_x = c13_g_x;
      c13_i_y = c13_h_x / 3.1415926535897931;
      if (CV_SCRIPT_COND(0, 17, c13_i_y < c13_b_sm->joints.thresholdNotInContact))
      {
        CV_SCRIPT_MCDC(0, 7, TRUE);
        CV_SCRIPT_IF(0, 29, TRUE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 277);
        chartInstance->c13_state = 12.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 278);
        chartInstance->c13_tSwitch = c13_t;
      } else {
        guard1 = TRUE;
      }
    } else {
      guard1 = TRUE;
    }

    if (guard1 == TRUE) {
      CV_SCRIPT_MCDC(0, 7, FALSE);
      CV_SCRIPT_IF(0, 29, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 283);
  if (CV_SCRIPT_IF(0, 30, chartInstance->c13_state == 12.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 284);
    for (c13_i366 = 0; c13_i366 < 2; c13_i366++) {
      c13_constraints[c13_i366] = (real_T)c13_i366;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 285);
    for (c13_i367 = 0; c13_i367 < 16; c13_i367++) {
      c13_hoistedGlobal[c13_i367] = chartInstance->c13_w_H_fixedLink[c13_i367];
    }

    for (c13_i368 = 0; c13_i368 < 16; c13_i368++) {
      c13_b[c13_i368] = c13_r_sole_H_b[c13_i368];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i369 = 0; c13_i369 < 16; c13_i369++) {
      c13_w_H_b[c13_i369] = 0.0;
    }

    for (c13_i370 = 0; c13_i370 < 16; c13_i370++) {
      c13_w_H_b[c13_i370] = 0.0;
    }

    for (c13_i371 = 0; c13_i371 < 16; c13_i371++) {
      c13_C[c13_i371] = c13_w_H_b[c13_i371];
    }

    for (c13_i372 = 0; c13_i372 < 16; c13_i372++) {
      c13_w_H_b[c13_i372] = c13_C[c13_i372];
    }

    for (c13_i373 = 0; c13_i373 < 16; c13_i373++) {
      c13_C[c13_i373] = c13_w_H_b[c13_i373];
    }

    for (c13_i374 = 0; c13_i374 < 16; c13_i374++) {
      c13_w_H_b[c13_i374] = c13_C[c13_i374];
    }

    for (c13_i375 = 0; c13_i375 < 4; c13_i375++) {
      c13_i376 = 0;
      for (c13_i377 = 0; c13_i377 < 4; c13_i377++) {
        c13_w_H_b[c13_i376 + c13_i375] = 0.0;
        c13_i378 = 0;
        for (c13_i379 = 0; c13_i379 < 4; c13_i379++) {
          c13_w_H_b[c13_i376 + c13_i375] += c13_hoistedGlobal[c13_i378 +
            c13_i375] * c13_b[c13_i379 + c13_i376];
          c13_i378 += 4;
        }

        c13_i376 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 287);
    c13_i380 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.com.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i381 = 0; c13_i381 < 2; c13_i381++) {
      c13_dv25[c13_i381] = chartInstance->c13_w_H_fixedLink[c13_i381 + 12];
    }

    c13_dv25[2] = c13_CoM_0[2];
    for (c13_i382 = 0; c13_i382 < 3; c13_i382++) {
      c13_CoMDes[c13_i382] = c13_dv25[c13_i382] + c13_b_sm->com.states[c13_i380
        + 13 * c13_i382];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 288);
    c13_i383 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.states",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1,
      0) - 1;
    for (c13_i384 = 0; c13_i384 < 23; c13_i384++) {
      c13_qDes[c13_i384] = c13_b_sm->joints.states[c13_i383 + 13 * c13_i384];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 289);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i385 = 0; c13_i385 < 23; c13_i385++) {
      c13_impedances[c13_i385] = c13_b_gain->impedances[c13_i385];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 290);
    c13_i386 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i387 = 0; c13_i387 < 3; c13_i387++) {
      c13_kpCom[c13_i387] = c13_b_gain->PCOM[c13_i386 + 3 * c13_i387];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 291);
    c13_i388 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i389 = 0; c13_i389 < 3; c13_i389++) {
      c13_kdCom[c13_i389] = c13_b_gain->DCOM[c13_i388 + 3 * c13_i389];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 293);
    if (CV_SCRIPT_IF(0, 31, c13_wrench_leftFoot[2] >
                     c13_b_sm->wrench.thresholdContactOn)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 294);
      chartInstance->c13_state = 13.0;
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 295);
      chartInstance->c13_tSwitch = c13_t;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 300);
  if (CV_SCRIPT_IF(0, 32, chartInstance->c13_state == 13.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 301);
    for (c13_i390 = 0; c13_i390 < 16; c13_i390++) {
      c13_hoistedGlobal[c13_i390] = chartInstance->c13_w_H_fixedLink[c13_i390];
    }

    for (c13_i391 = 0; c13_i391 < 16; c13_i391++) {
      c13_b[c13_i391] = c13_r_sole_H_b[c13_i391];
    }

    c13_eml_scalar_eg(chartInstance);
    c13_eml_scalar_eg(chartInstance);
    for (c13_i392 = 0; c13_i392 < 16; c13_i392++) {
      c13_w_H_b[c13_i392] = 0.0;
    }

    for (c13_i393 = 0; c13_i393 < 16; c13_i393++) {
      c13_w_H_b[c13_i393] = 0.0;
    }

    for (c13_i394 = 0; c13_i394 < 16; c13_i394++) {
      c13_C[c13_i394] = c13_w_H_b[c13_i394];
    }

    for (c13_i395 = 0; c13_i395 < 16; c13_i395++) {
      c13_w_H_b[c13_i395] = c13_C[c13_i395];
    }

    for (c13_i396 = 0; c13_i396 < 16; c13_i396++) {
      c13_C[c13_i396] = c13_w_H_b[c13_i396];
    }

    for (c13_i397 = 0; c13_i397 < 16; c13_i397++) {
      c13_w_H_b[c13_i397] = c13_C[c13_i397];
    }

    for (c13_i398 = 0; c13_i398 < 4; c13_i398++) {
      c13_i399 = 0;
      for (c13_i400 = 0; c13_i400 < 4; c13_i400++) {
        c13_w_H_b[c13_i399 + c13_i398] = 0.0;
        c13_i401 = 0;
        for (c13_i402 = 0; c13_i402 < 4; c13_i402++) {
          c13_w_H_b[c13_i399 + c13_i398] += c13_hoistedGlobal[c13_i401 +
            c13_i398] * c13_b[c13_i402 + c13_i399];
          c13_i401 += 4;
        }

        c13_i399 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 302);
    for (c13_i403 = 0; c13_i403 < 2; c13_i403++) {
      c13_constraints[c13_i403] = 1.0;
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 303);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 1, 1, 0);
    for (c13_i404 = 0; c13_i404 < 23; c13_i404++) {
      c13_impedances[c13_i404] = c13_b_gain->impedances[c13_i404];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 304);
    c13_i405 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i406 = 0; c13_i406 < 3; c13_i406++) {
      c13_kpCom[c13_i406] = c13_b_gain->PCOM[c13_i405 + 3 * c13_i406];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 305);
    c13_i407 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM",
      (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 3, 1, 0)
      - 1;
    for (c13_i408 = 0; c13_i408 < 3; c13_i408++) {
      c13_kdCom[c13_i408] = c13_b_gain->DCOM[c13_i407 + 3 * c13_i408];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 306);
    if (CV_SCRIPT_IF(0, 33, c13_t - chartInstance->c13_tSwitch >
                     c13_b_sm->tBalancing)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 307);
      if (CV_SCRIPT_IF(0, 34, c13_b_sm->yogaInLoop)) {
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 308);
        chartInstance->c13_state = 2.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 309);
        for (c13_i409 = 0; c13_i409 < 16; c13_i409++) {
          c13_hoistedGlobal[c13_i409] = chartInstance->
            c13_w_H_fixedLink[c13_i409];
        }

        for (c13_i410 = 0; c13_i410 < 16; c13_i410++) {
          c13_b[c13_i410] = c13_r_sole_H_b[c13_i410];
        }

        c13_eml_scalar_eg(chartInstance);
        c13_eml_scalar_eg(chartInstance);
        for (c13_i411 = 0; c13_i411 < 4; c13_i411++) {
          c13_i412 = 0;
          for (c13_i413 = 0; c13_i413 < 4; c13_i413++) {
            c13_C[c13_i412 + c13_i411] = 0.0;
            c13_i414 = 0;
            for (c13_i415 = 0; c13_i415 < 4; c13_i415++) {
              c13_C[c13_i412 + c13_i411] += c13_hoistedGlobal[c13_i414 +
                c13_i411] * c13_b[c13_i415 + c13_i412];
              c13_i414 += 4;
            }

            c13_i412 += 4;
          }
        }

        for (c13_i416 = 0; c13_i416 < 16; c13_i416++) {
          c13_d_C[c13_i416] = c13_C[c13_i416];
        }

        for (c13_i417 = 0; c13_i417 < 16; c13_i417++) {
          c13_b_l_sole_H_b[c13_i417] = c13_l_sole_H_b[c13_i417];
        }

        c13_mrdivide(chartInstance, c13_d_C, c13_b_l_sole_H_b, c13_dv26);
        for (c13_i418 = 0; c13_i418 < 16; c13_i418++) {
          chartInstance->c13_w_H_fixedLink[c13_i418] = c13_dv26[c13_i418];
        }

        _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 310);
        if (CV_SCRIPT_IF(0, 35, c13_b_sm->demoOnlyRightFoot)) {
          _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 311);
          chartInstance->c13_state = 8.0;
          _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 312);
          for (c13_i419 = 0; c13_i419 < 16; c13_i419++) {
            c13_hoistedGlobal[c13_i419] = chartInstance->
              c13_w_H_fixedLink[c13_i419];
          }

          for (c13_i420 = 0; c13_i420 < 16; c13_i420++) {
            c13_b[c13_i420] = c13_l_sole_H_b[c13_i420];
          }

          c13_eml_scalar_eg(chartInstance);
          c13_eml_scalar_eg(chartInstance);
          for (c13_i421 = 0; c13_i421 < 4; c13_i421++) {
            c13_i422 = 0;
            for (c13_i423 = 0; c13_i423 < 4; c13_i423++) {
              c13_C[c13_i422 + c13_i421] = 0.0;
              c13_i424 = 0;
              for (c13_i425 = 0; c13_i425 < 4; c13_i425++) {
                c13_C[c13_i422 + c13_i421] += c13_hoistedGlobal[c13_i424 +
                  c13_i421] * c13_b[c13_i425 + c13_i422];
                c13_i424 += 4;
              }

              c13_i422 += 4;
            }
          }

          for (c13_i426 = 0; c13_i426 < 16; c13_i426++) {
            c13_e_C[c13_i426] = c13_C[c13_i426];
          }

          for (c13_i427 = 0; c13_i427 < 16; c13_i427++) {
            c13_d_r_sole_H_b[c13_i427] = c13_r_sole_H_b[c13_i427];
          }

          c13_mrdivide(chartInstance, c13_e_C, c13_d_r_sole_H_b, c13_dv27);
          for (c13_i428 = 0; c13_i428 < 16; c13_i428++) {
            chartInstance->c13_w_H_fixedLink[c13_i428] = c13_dv27[c13_i428];
          }
        }
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 319);
  *c13_currentState = chartInstance->c13_state;
  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, 320);
  *c13_jointsSmoothingTime = c13_b_sm->jointsSmoothingTimes[(int32_T)(real_T)
    _SFD_EML_ARRAY_BOUNDS_CHECK("sm.jointsSmoothingTimes", (int32_T)
    _SFD_INTEGER_CHECK("state", chartInstance->c13_state), 1, 13, 1, 0) - 1];
  _SFD_SCRIPT_CALL(0U, chartInstance->c13_sfEvent, -320);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c13_machineNumber, uint32_T
  c13_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c13_chartNumber, 0U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m"));
}

static const mxArray *c13_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  real_T c13_u;
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  c13_u = *(real_T *)c13_inData;
  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", &c13_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static real_T c13_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_jointsSmoothingTime, const char_T
  *c13_identifier)
{
  real_T c13_y;
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_y = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c13_jointsSmoothingTime), &c13_thisId);
  sf_mex_destroy(&c13_jointsSmoothingTime);
  return c13_y;
}

static real_T c13_b_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId)
{
  real_T c13_y;
  real_T c13_d0;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), &c13_d0, 1, 0, 0U, 0, 0U, 0);
  c13_y = c13_d0;
  sf_mex_destroy(&c13_u);
  return c13_y;
}

static void c13_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_jointsSmoothingTime;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_jointsSmoothingTime = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_y = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c13_jointsSmoothingTime), &c13_thisId);
  sf_mex_destroy(&c13_jointsSmoothingTime);
  *(real_T *)c13_outData = c13_y;
  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_b_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i429;
  real_T c13_b_inData[3];
  int32_T c13_i430;
  real_T c13_u[3];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  for (c13_i429 = 0; c13_i429 < 3; c13_i429++) {
    c13_b_inData[c13_i429] = (*(real_T (*)[3])c13_inData)[c13_i429];
  }

  for (c13_i430 = 0; c13_i430 < 3; c13_i430++) {
    c13_u[c13_i430] = c13_b_inData[c13_i430];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 2, 1, 3), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_c_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_kdCom, const char_T *c13_identifier, real_T
  c13_y[3])
{
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_kdCom), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_kdCom);
}

static void c13_d_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[3])
{
  real_T c13_dv28[3];
  int32_T c13_i431;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv28, 1, 0, 0U, 1, 0U, 2, 1,
                3);
  for (c13_i431 = 0; c13_i431 < 3; c13_i431++) {
    c13_y[c13_i431] = c13_dv28[c13_i431];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_kdCom;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[3];
  int32_T c13_i432;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_kdCom = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_kdCom), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_kdCom);
  for (c13_i432 = 0; c13_i432 < 3; c13_i432++) {
    (*(real_T (*)[3])c13_outData)[c13_i432] = c13_y[c13_i432];
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_c_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i433;
  real_T c13_b_inData[23];
  int32_T c13_i434;
  real_T c13_u[23];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  for (c13_i433 = 0; c13_i433 < 23; c13_i433++) {
    c13_b_inData[c13_i433] = (*(real_T (*)[23])c13_inData)[c13_i433];
  }

  for (c13_i434 = 0; c13_i434 < 23; c13_i434++) {
    c13_u[c13_i434] = c13_b_inData[c13_i434];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_e_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_impedances, const char_T *c13_identifier,
  real_T c13_y[23])
{
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_impedances), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_impedances);
}

static void c13_f_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[23])
{
  real_T c13_dv29[23];
  int32_T c13_i435;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv29, 1, 0, 0U, 1, 0U, 2, 1,
                23);
  for (c13_i435 = 0; c13_i435 < 23; c13_i435++) {
    c13_y[c13_i435] = c13_dv29[c13_i435];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_impedances;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[23];
  int32_T c13_i436;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_impedances = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_impedances), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_impedances);
  for (c13_i436 = 0; c13_i436 < 23; c13_i436++) {
    (*(real_T (*)[23])c13_outData)[c13_i436] = c13_y[c13_i436];
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_d_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i437;
  real_T c13_b_inData[2];
  int32_T c13_i438;
  real_T c13_u[2];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  for (c13_i437 = 0; c13_i437 < 2; c13_i437++) {
    c13_b_inData[c13_i437] = (*(real_T (*)[2])c13_inData)[c13_i437];
  }

  for (c13_i438 = 0; c13_i438 < 2; c13_i438++) {
    c13_u[c13_i438] = c13_b_inData[c13_i438];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_g_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_constraints, const char_T *c13_identifier,
  real_T c13_y[2])
{
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_constraints), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_constraints);
}

static void c13_h_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[2])
{
  real_T c13_dv30[2];
  int32_T c13_i439;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv30, 1, 0, 0U, 1, 0U, 1, 2);
  for (c13_i439 = 0; c13_i439 < 2; c13_i439++) {
    c13_y[c13_i439] = c13_dv30[c13_i439];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_constraints;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[2];
  int32_T c13_i440;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_constraints = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_constraints), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_constraints);
  for (c13_i440 = 0; c13_i440 < 2; c13_i440++) {
    (*(real_T (*)[2])c13_outData)[c13_i440] = c13_y[c13_i440];
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_e_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i441;
  real_T c13_b_inData[23];
  int32_T c13_i442;
  real_T c13_u[23];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  for (c13_i441 = 0; c13_i441 < 23; c13_i441++) {
    c13_b_inData[c13_i441] = (*(real_T (*)[23])c13_inData)[c13_i441];
  }

  for (c13_i442 = 0; c13_i442 < 23; c13_i442++) {
    c13_u[c13_i442] = c13_b_inData[c13_i442];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_i_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_qDes, const char_T *c13_identifier, real_T
  c13_y[23])
{
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_qDes), &c13_thisId, c13_y);
  sf_mex_destroy(&c13_qDes);
}

static void c13_j_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[23])
{
  real_T c13_dv31[23];
  int32_T c13_i443;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv31, 1, 0, 0U, 1, 0U, 1,
                23);
  for (c13_i443 = 0; c13_i443 < 23; c13_i443++) {
    c13_y[c13_i443] = c13_dv31[c13_i443];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_qDes;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[23];
  int32_T c13_i444;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_qDes = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_qDes), &c13_thisId, c13_y);
  sf_mex_destroy(&c13_qDes);
  for (c13_i444 = 0; c13_i444 < 23; c13_i444++) {
    (*(real_T (*)[23])c13_outData)[c13_i444] = c13_y[c13_i444];
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_f_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i445;
  real_T c13_b_inData[3];
  int32_T c13_i446;
  real_T c13_u[3];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  for (c13_i445 = 0; c13_i445 < 3; c13_i445++) {
    c13_b_inData[c13_i445] = (*(real_T (*)[3])c13_inData)[c13_i445];
  }

  for (c13_i446 = 0; c13_i446 < 3; c13_i446++) {
    c13_u[c13_i446] = c13_b_inData[c13_i446];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_k_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_CoMDes, const char_T *c13_identifier,
  real_T c13_y[3])
{
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_CoMDes), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_CoMDes);
}

static void c13_l_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[3])
{
  real_T c13_dv32[3];
  int32_T c13_i447;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv32, 1, 0, 0U, 1, 0U, 1, 3);
  for (c13_i447 = 0; c13_i447 < 3; c13_i447++) {
    c13_y[c13_i447] = c13_dv32[c13_i447];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_CoMDes;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[3];
  int32_T c13_i448;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_CoMDes = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_CoMDes), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_CoMDes);
  for (c13_i448 = 0; c13_i448 < 3; c13_i448++) {
    (*(real_T (*)[3])c13_outData)[c13_i448] = c13_y[c13_i448];
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_g_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i449;
  int32_T c13_i450;
  int32_T c13_i451;
  real_T c13_b_inData[16];
  int32_T c13_i452;
  int32_T c13_i453;
  int32_T c13_i454;
  real_T c13_u[16];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  c13_i449 = 0;
  for (c13_i450 = 0; c13_i450 < 4; c13_i450++) {
    for (c13_i451 = 0; c13_i451 < 4; c13_i451++) {
      c13_b_inData[c13_i451 + c13_i449] = (*(real_T (*)[16])c13_inData)[c13_i451
        + c13_i449];
    }

    c13_i449 += 4;
  }

  c13_i452 = 0;
  for (c13_i453 = 0; c13_i453 < 4; c13_i453++) {
    for (c13_i454 = 0; c13_i454 < 4; c13_i454++) {
      c13_u[c13_i454 + c13_i452] = c13_b_inData[c13_i454 + c13_i452];
    }

    c13_i452 += 4;
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_m_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_w_H_b, const char_T *c13_identifier, real_T
  c13_y[16])
{
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_w_H_b), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_w_H_b);
}

static void c13_n_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[16])
{
  real_T c13_dv33[16];
  int32_T c13_i455;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv33, 1, 0, 0U, 1, 0U, 2, 4,
                4);
  for (c13_i455 = 0; c13_i455 < 16; c13_i455++) {
    c13_y[c13_i455] = c13_dv33[c13_i455];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_w_H_b;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[16];
  int32_T c13_i456;
  int32_T c13_i457;
  int32_T c13_i458;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_w_H_b = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_w_H_b), &c13_thisId,
    c13_y);
  sf_mex_destroy(&c13_w_H_b);
  c13_i456 = 0;
  for (c13_i457 = 0; c13_i457 < 4; c13_i457++) {
    for (c13_i458 = 0; c13_i458 < 4; c13_i458++) {
      (*(real_T (*)[16])c13_outData)[c13_i458 + c13_i456] = c13_y[c13_i458 +
        c13_i456];
    }

    c13_i456 += 4;
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_h_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  c13_struct_kzTB0QQWoOlMoMhgKf6sK c13_u;
  const mxArray *c13_y = NULL;
  real_T c13_b_u;
  const mxArray *c13_b_y = NULL;
  real_T c13_c_u;
  const mxArray *c13_c_y = NULL;
  real_T c13_d_u;
  const mxArray *c13_d_y = NULL;
  int32_T c13_i459;
  real_T c13_e_u[9];
  const mxArray *c13_e_y = NULL;
  int32_T c13_i460;
  real_T c13_f_u[9];
  const mxArray *c13_f_y = NULL;
  int32_T c13_i461;
  real_T c13_g_u[9];
  const mxArray *c13_g_y = NULL;
  real_T c13_h_u;
  const mxArray *c13_h_y = NULL;
  real_T c13_i_u;
  const mxArray *c13_i_y = NULL;
  int32_T c13_i462;
  real_T c13_j_u[23];
  const mxArray *c13_j_y = NULL;
  int32_T c13_i463;
  real_T c13_k_u[23];
  const mxArray *c13_k_y = NULL;
  int32_T c13_i464;
  real_T c13_l_u[23];
  const mxArray *c13_l_y = NULL;
  int32_T c13_i465;
  real_T c13_m_u[23];
  const mxArray *c13_m_y = NULL;
  int32_T c13_i466;
  real_T c13_n_u[4];
  const mxArray *c13_n_y = NULL;
  int32_T c13_i467;
  real_T c13_o_u[4];
  const mxArray *c13_o_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  c13_u = *(c13_struct_kzTB0QQWoOlMoMhgKf6sK *)c13_inData;
  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c13_b_u = c13_u.qTildeMax;
  c13_b_y = NULL;
  sf_mex_assign(&c13_b_y, sf_mex_create("y", &c13_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_b_y, "qTildeMax", "qTildeMax", 0);
  c13_c_u = c13_u.SmoothingTimeImp;
  c13_c_y = NULL;
  sf_mex_assign(&c13_c_y, sf_mex_create("y", &c13_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_c_y, "SmoothingTimeImp", "SmoothingTimeImp", 0);
  c13_d_u = c13_u.SmoothingTimeGainScheduling;
  c13_d_y = NULL;
  sf_mex_assign(&c13_d_y, sf_mex_create("y", &c13_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_d_y, "SmoothingTimeGainScheduling",
                  "SmoothingTimeGainScheduling", 0);
  for (c13_i459 = 0; c13_i459 < 9; c13_i459++) {
    c13_e_u[c13_i459] = c13_u.PCOM[c13_i459];
  }

  c13_e_y = NULL;
  sf_mex_assign(&c13_e_y, sf_mex_create("y", c13_e_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c13_y, c13_e_y, "PCOM", "PCOM", 0);
  for (c13_i460 = 0; c13_i460 < 9; c13_i460++) {
    c13_f_u[c13_i460] = c13_u.ICOM[c13_i460];
  }

  c13_f_y = NULL;
  sf_mex_assign(&c13_f_y, sf_mex_create("y", c13_f_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c13_y, c13_f_y, "ICOM", "ICOM", 0);
  for (c13_i461 = 0; c13_i461 < 9; c13_i461++) {
    c13_g_u[c13_i461] = c13_u.DCOM[c13_i461];
  }

  c13_g_y = NULL;
  sf_mex_assign(&c13_g_y, sf_mex_create("y", c13_g_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c13_y, c13_g_y, "DCOM", "DCOM", 0);
  c13_h_u = c13_u.PAngularMomentum;
  c13_h_y = NULL;
  sf_mex_assign(&c13_h_y, sf_mex_create("y", &c13_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_h_y, "PAngularMomentum", "PAngularMomentum", 0);
  c13_i_u = c13_u.DAngularMomentum;
  c13_i_y = NULL;
  sf_mex_assign(&c13_i_y, sf_mex_create("y", &c13_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_i_y, "DAngularMomentum", "DAngularMomentum", 0);
  for (c13_i462 = 0; c13_i462 < 23; c13_i462++) {
    c13_j_u[c13_i462] = c13_u.integral[c13_i462];
  }

  c13_j_y = NULL;
  sf_mex_assign(&c13_j_y, sf_mex_create("y", c13_j_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c13_y, c13_j_y, "integral", "integral", 0);
  for (c13_i463 = 0; c13_i463 < 23; c13_i463++) {
    c13_k_u[c13_i463] = c13_u.impedances[c13_i463];
  }

  c13_k_y = NULL;
  sf_mex_assign(&c13_k_y, sf_mex_create("y", c13_k_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c13_y, c13_k_y, "impedances", "impedances", 0);
  for (c13_i464 = 0; c13_i464 < 23; c13_i464++) {
    c13_l_u[c13_i464] = c13_u.dampings[c13_i464];
  }

  c13_l_y = NULL;
  sf_mex_assign(&c13_l_y, sf_mex_create("y", c13_l_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c13_y, c13_l_y, "dampings", "dampings", 0);
  for (c13_i465 = 0; c13_i465 < 23; c13_i465++) {
    c13_m_u[c13_i465] = c13_u.increasingRatesImp[c13_i465];
  }

  c13_m_y = NULL;
  sf_mex_assign(&c13_m_y, sf_mex_create("y", c13_m_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c13_y, c13_m_y, "increasingRatesImp", "increasingRatesImp", 0);
  for (c13_i466 = 0; c13_i466 < 4; c13_i466++) {
    c13_n_u[c13_i466] = c13_u.footSize[c13_i466];
  }

  c13_n_y = NULL;
  sf_mex_assign(&c13_n_y, sf_mex_create("y", c13_n_u, 0, 0U, 1U, 0U, 2, 2, 2),
                FALSE);
  sf_mex_addfield(c13_y, c13_n_y, "footSize", "footSize", 0);
  for (c13_i467 = 0; c13_i467 < 4; c13_i467++) {
    c13_o_u[c13_i467] = c13_u.legSize[c13_i467];
  }

  c13_o_y = NULL;
  sf_mex_assign(&c13_o_y, sf_mex_create("y", c13_o_u, 0, 0U, 1U, 0U, 2, 2, 2),
                FALSE);
  sf_mex_addfield(c13_y, c13_o_y, "legSize", "legSize", 0);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_o_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_kzTB0QQWoOlMoMhgKf6sK *c13_y)
{
  emlrtMsgIdentifier c13_thisId;
  static const char * c13_fieldNames[14] = { "qTildeMax", "SmoothingTimeImp",
    "SmoothingTimeGainScheduling", "PCOM", "ICOM", "DCOM", "PAngularMomentum",
    "DAngularMomentum", "integral", "impedances", "dampings",
    "increasingRatesImp", "footSize", "legSize" };

  c13_thisId.fParent = c13_parentId;
  sf_mex_check_struct(c13_parentId, c13_u, 14, c13_fieldNames, 0U, 0);
  c13_thisId.fIdentifier = "qTildeMax";
  c13_y->qTildeMax = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "qTildeMax", "qTildeMax", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "SmoothingTimeImp";
  c13_y->SmoothingTimeImp = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "SmoothingTimeImp", "SmoothingTimeImp", 0)),
    &c13_thisId);
  c13_thisId.fIdentifier = "SmoothingTimeGainScheduling";
  c13_y->SmoothingTimeGainScheduling = c13_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c13_u, "SmoothingTimeGainScheduling",
    "SmoothingTimeGainScheduling", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "PCOM";
  c13_p_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u, "PCOM",
    "PCOM", 0)), &c13_thisId, c13_y->PCOM);
  c13_thisId.fIdentifier = "ICOM";
  c13_p_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u, "ICOM",
    "ICOM", 0)), &c13_thisId, c13_y->ICOM);
  c13_thisId.fIdentifier = "DCOM";
  c13_p_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u, "DCOM",
    "DCOM", 0)), &c13_thisId, c13_y->DCOM);
  c13_thisId.fIdentifier = "PAngularMomentum";
  c13_y->PAngularMomentum = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "PAngularMomentum", "PAngularMomentum", 0)),
    &c13_thisId);
  c13_thisId.fIdentifier = "DAngularMomentum";
  c13_y->DAngularMomentum = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "DAngularMomentum", "DAngularMomentum", 0)),
    &c13_thisId);
  c13_thisId.fIdentifier = "integral";
  c13_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "integral", "integral", 0)), &c13_thisId, c13_y->integral);
  c13_thisId.fIdentifier = "impedances";
  c13_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "impedances", "impedances", 0)), &c13_thisId, c13_y->impedances);
  c13_thisId.fIdentifier = "dampings";
  c13_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "dampings", "dampings", 0)), &c13_thisId, c13_y->dampings);
  c13_thisId.fIdentifier = "increasingRatesImp";
  c13_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "increasingRatesImp", "increasingRatesImp", 0)), &c13_thisId,
    c13_y->increasingRatesImp);
  c13_thisId.fIdentifier = "footSize";
  c13_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "footSize", "footSize", 0)), &c13_thisId, c13_y->footSize);
  c13_thisId.fIdentifier = "legSize";
  c13_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "legSize", "legSize", 0)), &c13_thisId, c13_y->legSize);
  sf_mex_destroy(&c13_u);
}

static void c13_p_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[9])
{
  real_T c13_dv34[9];
  int32_T c13_i468;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv34, 1, 0, 0U, 1, 0U, 2, 3,
                3);
  for (c13_i468 = 0; c13_i468 < 9; c13_i468++) {
    c13_y[c13_i468] = c13_dv34[c13_i468];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_q_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[4])
{
  real_T c13_dv35[4];
  int32_T c13_i469;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv35, 1, 0, 0U, 1, 0U, 2, 2,
                2);
  for (c13_i469 = 0; c13_i469 < 4; c13_i469++) {
    c13_y[c13_i469] = c13_dv35[c13_i469];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_b_gain;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  c13_struct_kzTB0QQWoOlMoMhgKf6sK c13_y;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_b_gain = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_gain), &c13_thisId,
    &c13_y);
  sf_mex_destroy(&c13_b_gain);
  *(c13_struct_kzTB0QQWoOlMoMhgKf6sK *)c13_outData = c13_y;
  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_i_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData;
  c13_struct_rUGQ0INmvPpaxIctEGl5sE c13_u;
  const mxArray *c13_y = NULL;
  boolean_T c13_b_u;
  const mxArray *c13_b_y = NULL;
  boolean_T c13_c_u;
  const mxArray *c13_c_y = NULL;
  boolean_T c13_d_u;
  const mxArray *c13_d_y = NULL;
  boolean_T c13_e_u;
  const mxArray *c13_e_y = NULL;
  c13_struct_DnBdbfPNxiIjhNOyZMmfsE c13_f_u;
  const mxArray *c13_f_y = NULL;
  real_T c13_g_u;
  const mxArray *c13_g_y = NULL;
  int32_T c13_i470;
  real_T c13_h_u[39];
  const mxArray *c13_h_y = NULL;
  c13_struct_KJR2itYvhBuAkuR6dKZHUC c13_i_u;
  const mxArray *c13_i_y = NULL;
  real_T c13_j_u;
  const mxArray *c13_j_y = NULL;
  real_T c13_k_u;
  const mxArray *c13_k_y = NULL;
  c13_struct_0U0wBk2LiR1OqsMsUngxdD c13_l_u;
  const mxArray *c13_l_y = NULL;
  real_T c13_m_u;
  const mxArray *c13_m_y = NULL;
  real_T c13_n_u;
  const mxArray *c13_n_y = NULL;
  real_T c13_o_u;
  const mxArray *c13_o_y = NULL;
  real_T c13_p_u;
  const mxArray *c13_p_y = NULL;
  int32_T c13_i471;
  real_T c13_q_u[299];
  const mxArray *c13_q_y = NULL;
  int32_T c13_i472;
  real_T c13_r_u[192];
  const mxArray *c13_r_y = NULL;
  int32_T c13_i473;
  real_T c13_s_u[192];
  const mxArray *c13_s_y = NULL;
  int32_T c13_i474;
  real_T c13_t_u[72];
  const mxArray *c13_t_y = NULL;
  real_T c13_u_u;
  const mxArray *c13_u_y = NULL;
  real_T c13_v_u;
  const mxArray *c13_v_y = NULL;
  real_T c13_w_u;
  const mxArray *c13_w_y = NULL;
  int32_T c13_i475;
  real_T c13_x_u[13];
  const mxArray *c13_x_y = NULL;
  real_T c13_y_u;
  const mxArray *c13_y_y = NULL;
  boolean_T c13_ab_u;
  const mxArray *c13_ab_y = NULL;
  int32_T c13_i476;
  real_T c13_bb_u[8];
  const mxArray *c13_bb_y = NULL;
  c13_struct_9LpOi5JXaV67jTuay8hWaH c13_cb_u;
  const mxArray *c13_cb_y = NULL;
  int32_T c13_i477;
  real_T c13_db_u[24];
  const mxArray *c13_db_y = NULL;
  int32_T c13_i478;
  real_T c13_eb_u[8];
  const mxArray *c13_eb_y = NULL;
  int32_T c13_i479;
  real_T c13_fb_u[8];
  const mxArray *c13_fb_y = NULL;
  int32_T c13_i480;
  real_T c13_gb_u[8];
  const mxArray *c13_gb_y = NULL;
  int32_T c13_i481;
  real_T c13_hb_u[8];
  const mxArray *c13_hb_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  c13_mxArrayOutData = NULL;
  c13_u = *(c13_struct_rUGQ0INmvPpaxIctEGl5sE *)c13_inData;
  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c13_b_u = c13_u.skipYoga;
  c13_b_y = NULL;
  sf_mex_assign(&c13_b_y, sf_mex_create("y", &c13_b_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_b_y, "skipYoga", "skipYoga", 0);
  c13_c_u = c13_u.demoOnlyRightFoot;
  c13_c_y = NULL;
  sf_mex_assign(&c13_c_y, sf_mex_create("y", &c13_c_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_c_y, "demoOnlyRightFoot", "demoOnlyRightFoot", 0);
  c13_d_u = c13_u.yogaAlsoOnRightFoot;
  c13_d_y = NULL;
  sf_mex_assign(&c13_d_y, sf_mex_create("y", &c13_d_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_d_y, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot",
                  0);
  c13_e_u = c13_u.yogaInLoop;
  c13_e_y = NULL;
  sf_mex_assign(&c13_e_y, sf_mex_create("y", &c13_e_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_e_y, "yogaInLoop", "yogaInLoop", 0);
  c13_f_u = c13_u.com;
  c13_f_y = NULL;
  sf_mex_assign(&c13_f_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c13_g_u = c13_f_u.threshold;
  c13_g_y = NULL;
  sf_mex_assign(&c13_g_y, sf_mex_create("y", &c13_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_f_y, c13_g_y, "threshold", "threshold", 0);
  for (c13_i470 = 0; c13_i470 < 39; c13_i470++) {
    c13_h_u[c13_i470] = c13_f_u.states[c13_i470];
  }

  c13_h_y = NULL;
  sf_mex_assign(&c13_h_y, sf_mex_create("y", c13_h_u, 0, 0U, 1U, 0U, 2, 13, 3),
                FALSE);
  sf_mex_addfield(c13_f_y, c13_h_y, "states", "states", 0);
  sf_mex_addfield(c13_y, c13_f_y, "com", "com", 0);
  c13_i_u = c13_u.wrench;
  c13_i_y = NULL;
  sf_mex_assign(&c13_i_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c13_j_u = c13_i_u.thresholdContactOn;
  c13_j_y = NULL;
  sf_mex_assign(&c13_j_y, sf_mex_create("y", &c13_j_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_i_y, c13_j_y, "thresholdContactOn", "thresholdContactOn",
                  0);
  c13_k_u = c13_i_u.thresholdContactOff;
  c13_k_y = NULL;
  sf_mex_assign(&c13_k_y, sf_mex_create("y", &c13_k_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_i_y, c13_k_y, "thresholdContactOff", "thresholdContactOff",
                  0);
  sf_mex_addfield(c13_y, c13_i_y, "wrench", "wrench", 0);
  c13_l_u = c13_u.joints;
  c13_l_y = NULL;
  sf_mex_assign(&c13_l_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c13_m_u = c13_l_u.thresholdNotInContact;
  c13_m_y = NULL;
  sf_mex_assign(&c13_m_y, sf_mex_create("y", &c13_m_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_l_y, c13_m_y, "thresholdNotInContact",
                  "thresholdNotInContact", 0);
  c13_n_u = c13_l_u.thresholdInContact;
  c13_n_y = NULL;
  sf_mex_assign(&c13_n_y, sf_mex_create("y", &c13_n_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_l_y, c13_n_y, "thresholdInContact", "thresholdInContact",
                  0);
  c13_o_u = c13_l_u.pauseTimeLastPostureL;
  c13_o_y = NULL;
  sf_mex_assign(&c13_o_y, sf_mex_create("y", &c13_o_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_l_y, c13_o_y, "pauseTimeLastPostureL",
                  "pauseTimeLastPostureL", 0);
  c13_p_u = c13_l_u.pauseTimeLastPostureR;
  c13_p_y = NULL;
  sf_mex_assign(&c13_p_y, sf_mex_create("y", &c13_p_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_l_y, c13_p_y, "pauseTimeLastPostureR",
                  "pauseTimeLastPostureR", 0);
  for (c13_i471 = 0; c13_i471 < 299; c13_i471++) {
    c13_q_u[c13_i471] = c13_l_u.states[c13_i471];
  }

  c13_q_y = NULL;
  sf_mex_assign(&c13_q_y, sf_mex_create("y", c13_q_u, 0, 0U, 1U, 0U, 2, 13, 23),
                FALSE);
  sf_mex_addfield(c13_l_y, c13_q_y, "states", "states", 0);
  for (c13_i472 = 0; c13_i472 < 192; c13_i472++) {
    c13_r_u[c13_i472] = c13_l_u.pointsL[c13_i472];
  }

  c13_r_y = NULL;
  sf_mex_assign(&c13_r_y, sf_mex_create("y", c13_r_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c13_l_y, c13_r_y, "pointsL", "pointsL", 0);
  for (c13_i473 = 0; c13_i473 < 192; c13_i473++) {
    c13_s_u[c13_i473] = c13_l_u.pointsR[c13_i473];
  }

  c13_s_y = NULL;
  sf_mex_assign(&c13_s_y, sf_mex_create("y", c13_s_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c13_l_y, c13_s_y, "pointsR", "pointsR", 0);
  for (c13_i474 = 0; c13_i474 < 72; c13_i474++) {
    c13_t_u[c13_i474] = c13_l_u.standUpPositions[c13_i474];
  }

  c13_t_y = NULL;
  sf_mex_assign(&c13_t_y, sf_mex_create("y", c13_t_u, 0, 0U, 1U, 0U, 2, 8, 9),
                FALSE);
  sf_mex_addfield(c13_l_y, c13_t_y, "standUpPositions", "standUpPositions", 0);
  sf_mex_addfield(c13_y, c13_l_y, "joints", "joints", 0);
  c13_u_u = c13_u.stateAt0;
  c13_u_y = NULL;
  sf_mex_assign(&c13_u_y, sf_mex_create("y", &c13_u_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_u_y, "stateAt0", "stateAt0", 0);
  c13_v_u = c13_u.DT;
  c13_v_y = NULL;
  sf_mex_assign(&c13_v_y, sf_mex_create("y", &c13_v_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_v_y, "DT", "DT", 0);
  c13_w_u = c13_u.waitingTimeAfterYoga;
  c13_w_y = NULL;
  sf_mex_assign(&c13_w_y, sf_mex_create("y", &c13_w_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_w_y, "waitingTimeAfterYoga", "waitingTimeAfterYoga",
                  0);
  for (c13_i475 = 0; c13_i475 < 13; c13_i475++) {
    c13_x_u[c13_i475] = c13_u.jointsSmoothingTimes[c13_i475];
  }

  c13_x_y = NULL;
  sf_mex_assign(&c13_x_y, sf_mex_create("y", c13_x_u, 0, 0U, 1U, 0U, 2, 13, 1),
                FALSE);
  sf_mex_addfield(c13_y, c13_x_y, "jointsSmoothingTimes", "jointsSmoothingTimes",
                  0);
  c13_y_u = c13_u.tBalancing;
  c13_y_y = NULL;
  sf_mex_assign(&c13_y_y, sf_mex_create("y", &c13_y_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c13_y, c13_y_y, "tBalancing", "tBalancing", 0);
  c13_ab_u = c13_u.alsoSitDown;
  c13_ab_y = NULL;
  sf_mex_assign(&c13_ab_y, sf_mex_create("y", &c13_ab_u, 11, 0U, 0U, 0U, 0),
                FALSE);
  sf_mex_addfield(c13_y, c13_ab_y, "alsoSitDown", "alsoSitDown", 0);
  for (c13_i476 = 0; c13_i476 < 8; c13_i476++) {
    c13_bb_u[c13_i476] = c13_u.jointsAndCoMSmoothingTimes[c13_i476];
  }

  c13_bb_y = NULL;
  sf_mex_assign(&c13_bb_y, sf_mex_create("y", c13_bb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c13_y, c13_bb_y, "jointsAndCoMSmoothingTimes",
                  "jointsAndCoMSmoothingTimes", 0);
  c13_cb_u = c13_u.CoM;
  c13_cb_y = NULL;
  sf_mex_assign(&c13_cb_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  for (c13_i477 = 0; c13_i477 < 24; c13_i477++) {
    c13_db_u[c13_i477] = c13_cb_u.standUpDeltaCoM[c13_i477];
  }

  c13_db_y = NULL;
  sf_mex_assign(&c13_db_y, sf_mex_create("y", c13_db_u, 0, 0U, 1U, 0U, 2, 8, 3),
                FALSE);
  sf_mex_addfield(c13_cb_y, c13_db_y, "standUpDeltaCoM", "standUpDeltaCoM", 0);
  sf_mex_addfield(c13_y, c13_cb_y, "CoM", "CoM", 0);
  for (c13_i478 = 0; c13_i478 < 8; c13_i478++) {
    c13_eb_u[c13_i478] = c13_u.LwrenchThreshold[c13_i478];
  }

  c13_eb_y = NULL;
  sf_mex_assign(&c13_eb_y, sf_mex_create("y", c13_eb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c13_y, c13_eb_y, "LwrenchThreshold", "LwrenchThreshold", 0);
  for (c13_i479 = 0; c13_i479 < 8; c13_i479++) {
    c13_fb_u[c13_i479] = c13_u.RwrenchThreshold[c13_i479];
  }

  c13_fb_y = NULL;
  sf_mex_assign(&c13_fb_y, sf_mex_create("y", c13_fb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c13_y, c13_fb_y, "RwrenchThreshold", "RwrenchThreshold", 0);
  for (c13_i480 = 0; c13_i480 < 8; c13_i480++) {
    c13_gb_u[c13_i480] = c13_u.RArmThreshold[c13_i480];
  }

  c13_gb_y = NULL;
  sf_mex_assign(&c13_gb_y, sf_mex_create("y", c13_gb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c13_y, c13_gb_y, "RArmThreshold", "RArmThreshold", 0);
  for (c13_i481 = 0; c13_i481 < 8; c13_i481++) {
    c13_hb_u[c13_i481] = c13_u.LArmThreshold[c13_i481];
  }

  c13_hb_y = NULL;
  sf_mex_assign(&c13_hb_y, sf_mex_create("y", c13_hb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c13_y, c13_hb_y, "LArmThreshold", "LArmThreshold", 0);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_r_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_rUGQ0INmvPpaxIctEGl5sE *c13_y)
{
  emlrtMsgIdentifier c13_thisId;
  static const char * c13_fieldNames[19] = { "skipYoga", "demoOnlyRightFoot",
    "yogaAlsoOnRightFoot", "yogaInLoop", "com", "wrench", "joints", "stateAt0",
    "DT", "waitingTimeAfterYoga", "jointsSmoothingTimes", "tBalancing",
    "alsoSitDown", "jointsAndCoMSmoothingTimes", "CoM", "LwrenchThreshold",
    "RwrenchThreshold", "RArmThreshold", "LArmThreshold" };

  c13_thisId.fParent = c13_parentId;
  sf_mex_check_struct(c13_parentId, c13_u, 19, c13_fieldNames, 0U, 0);
  c13_thisId.fIdentifier = "skipYoga";
  c13_y->skipYoga = c13_s_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "skipYoga", "skipYoga", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "demoOnlyRightFoot";
  c13_y->demoOnlyRightFoot = c13_s_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "demoOnlyRightFoot", "demoOnlyRightFoot", 0)),
    &c13_thisId);
  c13_thisId.fIdentifier = "yogaAlsoOnRightFoot";
  c13_y->yogaAlsoOnRightFoot = c13_s_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot", 0)),
    &c13_thisId);
  c13_thisId.fIdentifier = "yogaInLoop";
  c13_y->yogaInLoop = c13_s_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "yogaInLoop", "yogaInLoop", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "com";
  c13_t_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u, "com",
    "com", 0)), &c13_thisId, &c13_y->com);
  c13_thisId.fIdentifier = "wrench";
  c13_y->wrench = c13_v_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "wrench", "wrench", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "joints";
  c13_w_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "joints", "joints", 0)), &c13_thisId, &c13_y->joints);
  c13_thisId.fIdentifier = "stateAt0";
  c13_y->stateAt0 = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "stateAt0", "stateAt0", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "DT";
  c13_y->DT = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c13_u, "DT", "DT", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "waitingTimeAfterYoga";
  c13_y->waitingTimeAfterYoga = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "waitingTimeAfterYoga", "waitingTimeAfterYoga", 0)),
    &c13_thisId);
  c13_thisId.fIdentifier = "jointsSmoothingTimes";
  c13_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "jointsSmoothingTimes", "jointsSmoothingTimes", 0)), &c13_thisId,
    c13_y->jointsSmoothingTimes);
  c13_thisId.fIdentifier = "tBalancing";
  c13_y->tBalancing = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "tBalancing", "tBalancing", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "alsoSitDown";
  c13_y->alsoSitDown = c13_s_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "alsoSitDown", "alsoSitDown", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "jointsAndCoMSmoothingTimes";
  c13_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "jointsAndCoMSmoothingTimes", "jointsAndCoMSmoothingTimes", 0)), &c13_thisId,
    c13_y->jointsAndCoMSmoothingTimes);
  c13_thisId.fIdentifier = "CoM";
  c13_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u, "CoM",
    "CoM", 0)), &c13_thisId, &c13_y->CoM);
  c13_thisId.fIdentifier = "LwrenchThreshold";
  c13_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "LwrenchThreshold", "LwrenchThreshold", 0)), &c13_thisId,
    c13_y->LwrenchThreshold);
  c13_thisId.fIdentifier = "RwrenchThreshold";
  c13_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "RwrenchThreshold", "RwrenchThreshold", 0)), &c13_thisId,
    c13_y->RwrenchThreshold);
  c13_thisId.fIdentifier = "RArmThreshold";
  c13_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "RArmThreshold", "RArmThreshold", 0)), &c13_thisId, c13_y->RArmThreshold);
  c13_thisId.fIdentifier = "LArmThreshold";
  c13_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "LArmThreshold", "LArmThreshold", 0)), &c13_thisId, c13_y->LArmThreshold);
  sf_mex_destroy(&c13_u);
}

static boolean_T c13_s_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId)
{
  boolean_T c13_y;
  boolean_T c13_b0;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), &c13_b0, 1, 11, 0U, 0, 0U, 0);
  c13_y = c13_b0;
  sf_mex_destroy(&c13_u);
  return c13_y;
}

static void c13_t_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_DnBdbfPNxiIjhNOyZMmfsE *c13_y)
{
  emlrtMsgIdentifier c13_thisId;
  static const char * c13_fieldNames[2] = { "threshold", "states" };

  c13_thisId.fParent = c13_parentId;
  sf_mex_check_struct(c13_parentId, c13_u, 2, c13_fieldNames, 0U, 0);
  c13_thisId.fIdentifier = "threshold";
  c13_y->threshold = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "threshold", "threshold", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "states";
  c13_u_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "states", "states", 0)), &c13_thisId, c13_y->states);
  sf_mex_destroy(&c13_u);
}

static void c13_u_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[39])
{
  real_T c13_dv36[39];
  int32_T c13_i482;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv36, 1, 0, 0U, 1, 0U, 2,
                13, 3);
  for (c13_i482 = 0; c13_i482 < 39; c13_i482++) {
    c13_y[c13_i482] = c13_dv36[c13_i482];
  }

  sf_mex_destroy(&c13_u);
}

static c13_struct_KJR2itYvhBuAkuR6dKZHUC c13_v_emlrt_marshallIn
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c13_u,
   const emlrtMsgIdentifier *c13_parentId)
{
  c13_struct_KJR2itYvhBuAkuR6dKZHUC c13_y;
  emlrtMsgIdentifier c13_thisId;
  static const char * c13_fieldNames[2] = { "thresholdContactOn",
    "thresholdContactOff" };

  c13_thisId.fParent = c13_parentId;
  sf_mex_check_struct(c13_parentId, c13_u, 2, c13_fieldNames, 0U, 0);
  c13_thisId.fIdentifier = "thresholdContactOn";
  c13_y.thresholdContactOn = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "thresholdContactOn", "thresholdContactOn", 0)),
    &c13_thisId);
  c13_thisId.fIdentifier = "thresholdContactOff";
  c13_y.thresholdContactOff = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "thresholdContactOff", "thresholdContactOff", 0)),
    &c13_thisId);
  sf_mex_destroy(&c13_u);
  return c13_y;
}

static void c13_w_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_0U0wBk2LiR1OqsMsUngxdD *c13_y)
{
  emlrtMsgIdentifier c13_thisId;
  static const char * c13_fieldNames[8] = { "thresholdNotInContact",
    "thresholdInContact", "pauseTimeLastPostureL", "pauseTimeLastPostureR",
    "states", "pointsL", "pointsR", "standUpPositions" };

  c13_thisId.fParent = c13_parentId;
  sf_mex_check_struct(c13_parentId, c13_u, 8, c13_fieldNames, 0U, 0);
  c13_thisId.fIdentifier = "thresholdNotInContact";
  c13_y->thresholdNotInContact = c13_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c13_u, "thresholdNotInContact",
    "thresholdNotInContact", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "thresholdInContact";
  c13_y->thresholdInContact = c13_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c13_u, "thresholdInContact", "thresholdInContact", 0)),
    &c13_thisId);
  c13_thisId.fIdentifier = "pauseTimeLastPostureL";
  c13_y->pauseTimeLastPostureL = c13_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c13_u, "pauseTimeLastPostureL",
    "pauseTimeLastPostureL", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "pauseTimeLastPostureR";
  c13_y->pauseTimeLastPostureR = c13_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c13_u, "pauseTimeLastPostureR",
    "pauseTimeLastPostureR", 0)), &c13_thisId);
  c13_thisId.fIdentifier = "states";
  c13_x_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "states", "states", 0)), &c13_thisId, c13_y->states);
  c13_thisId.fIdentifier = "pointsL";
  c13_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "pointsL", "pointsL", 0)), &c13_thisId, c13_y->pointsL);
  c13_thisId.fIdentifier = "pointsR";
  c13_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "pointsR", "pointsR", 0)), &c13_thisId, c13_y->pointsR);
  c13_thisId.fIdentifier = "standUpPositions";
  c13_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "standUpPositions", "standUpPositions", 0)), &c13_thisId,
    c13_y->standUpPositions);
  sf_mex_destroy(&c13_u);
}

static void c13_x_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[299])
{
  real_T c13_dv37[299];
  int32_T c13_i483;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv37, 1, 0, 0U, 1, 0U, 2,
                13, 23);
  for (c13_i483 = 0; c13_i483 < 299; c13_i483++) {
    c13_y[c13_i483] = c13_dv37[c13_i483];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_y_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[192])
{
  real_T c13_dv38[192];
  int32_T c13_i484;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv38, 1, 0, 0U, 1, 0U, 2, 8,
                24);
  for (c13_i484 = 0; c13_i484 < 192; c13_i484++) {
    c13_y[c13_i484] = c13_dv38[c13_i484];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_ab_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[72])
{
  real_T c13_dv39[72];
  int32_T c13_i485;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv39, 1, 0, 0U, 1, 0U, 2, 8,
                9);
  for (c13_i485 = 0; c13_i485 < 72; c13_i485++) {
    c13_y[c13_i485] = c13_dv39[c13_i485];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_bb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[13])
{
  real_T c13_dv40[13];
  int32_T c13_i486;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv40, 1, 0, 0U, 1, 0U, 2,
                13, 1);
  for (c13_i486 = 0; c13_i486 < 13; c13_i486++) {
    c13_y[c13_i486] = c13_dv40[c13_i486];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_cb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[8])
{
  real_T c13_dv41[8];
  int32_T c13_i487;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv41, 1, 0, 0U, 1, 0U, 2, 8,
                1);
  for (c13_i487 = 0; c13_i487 < 8; c13_i487++) {
    c13_y[c13_i487] = c13_dv41[c13_i487];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_db_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  c13_struct_9LpOi5JXaV67jTuay8hWaH *c13_y)
{
  emlrtMsgIdentifier c13_thisId;
  static const char * c13_fieldNames[1] = { "standUpDeltaCoM" };

  c13_thisId.fParent = c13_parentId;
  sf_mex_check_struct(c13_parentId, c13_u, 1, c13_fieldNames, 0U, 0);
  c13_thisId.fIdentifier = "standUpDeltaCoM";
  c13_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c13_u,
    "standUpDeltaCoM", "standUpDeltaCoM", 0)), &c13_thisId,
    c13_y->standUpDeltaCoM);
  sf_mex_destroy(&c13_u);
}

static void c13_eb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[24])
{
  real_T c13_dv42[24];
  int32_T c13_i488;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv42, 1, 0, 0U, 1, 0U, 2, 8,
                3);
  for (c13_i488 = 0; c13_i488 < 24; c13_i488++) {
    c13_y[c13_i488] = c13_dv42[c13_i488];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_b_sm;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  c13_struct_rUGQ0INmvPpaxIctEGl5sE c13_y;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_b_sm = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_sm), &c13_thisId,
    &c13_y);
  sf_mex_destroy(&c13_b_sm);
  *(c13_struct_rUGQ0INmvPpaxIctEGl5sE *)c13_outData = c13_y;
  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_j_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i489;
  real_T c13_b_inData[3];
  int32_T c13_i490;
  real_T c13_u[3];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  for (c13_i489 = 0; c13_i489 < 3; c13_i489++) {
    c13_b_inData[c13_i489] = (*(real_T (*)[3])c13_inData)[c13_i489];
  }

  for (c13_i490 = 0; c13_i490 < 3; c13_i490++) {
    c13_u[c13_i490] = c13_b_inData[c13_i490];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 2, 3, 1), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static const mxArray *c13_k_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i491;
  real_T c13_b_inData[6];
  int32_T c13_i492;
  real_T c13_u[6];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  for (c13_i491 = 0; c13_i491 < 6; c13_i491++) {
    c13_b_inData[c13_i491] = (*(real_T (*)[6])c13_inData)[c13_i491];
  }

  for (c13_i492 = 0; c13_i492 < 6; c13_i492++) {
    c13_u[c13_i492] = c13_b_inData[c13_i492];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static const mxArray *c13_l_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i493;
  int32_T c13_i494;
  int32_T c13_i495;
  real_T c13_b_inData[16];
  int32_T c13_i496;
  int32_T c13_i497;
  int32_T c13_i498;
  real_T c13_u[16];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  c13_i493 = 0;
  for (c13_i494 = 0; c13_i494 < 4; c13_i494++) {
    for (c13_i495 = 0; c13_i495 < 4; c13_i495++) {
      c13_b_inData[c13_i495 + c13_i493] = (*(real_T (*)[16])c13_inData)[c13_i495
        + c13_i493];
    }

    c13_i493 += 4;
  }

  c13_i496 = 0;
  for (c13_i497 = 0; c13_i497 < 4; c13_i497++) {
    for (c13_i498 = 0; c13_i498 < 4; c13_i498++) {
      c13_u[c13_i498 + c13_i496] = c13_b_inData[c13_i498 + c13_i496];
    }

    c13_i496 += 4;
  }

  c13_y = NULL;
  if (!chartInstance->c13_w_H_fixedLink_not_empty) {
    sf_mex_assign(&c13_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 2, 4, 4),
                  FALSE);
  }

  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_fb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_b_w_H_fixedLink, const char_T
  *c13_identifier, real_T c13_y[16])
{
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_w_H_fixedLink),
    &c13_thisId, c13_y);
  sf_mex_destroy(&c13_b_w_H_fixedLink);
}

static void c13_gb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[16])
{
  real_T c13_dv43[16];
  int32_T c13_i499;
  if (mxIsEmpty(c13_u)) {
    chartInstance->c13_w_H_fixedLink_not_empty = FALSE;
  } else {
    chartInstance->c13_w_H_fixedLink_not_empty = TRUE;
    sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv43, 1, 0, 0U, 1, 0U, 2,
                  4, 4);
    for (c13_i499 = 0; c13_i499 < 16; c13_i499++) {
      c13_y[c13_i499] = c13_dv43[c13_i499];
    }
  }

  sf_mex_destroy(&c13_u);
}

static void c13_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_b_w_H_fixedLink;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[16];
  int32_T c13_i500;
  int32_T c13_i501;
  int32_T c13_i502;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_b_w_H_fixedLink = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_w_H_fixedLink),
    &c13_thisId, c13_y);
  sf_mex_destroy(&c13_b_w_H_fixedLink);
  c13_i500 = 0;
  for (c13_i501 = 0; c13_i501 < 4; c13_i501++) {
    for (c13_i502 = 0; c13_i502 < 4; c13_i502++) {
      (*(real_T (*)[16])c13_outData)[c13_i502 + c13_i500] = c13_y[c13_i502 +
        c13_i500];
    }

    c13_i500 += 4;
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_m_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  real_T c13_u;
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  c13_u = *(real_T *)c13_inData;
  c13_y = NULL;
  if (!chartInstance->c13_tSwitch_not_empty) {
    sf_mex_assign(&c13_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c13_y, sf_mex_create("y", &c13_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static real_T c13_hb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_b_tSwitch, const char_T *c13_identifier)
{
  real_T c13_y;
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_y = c13_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_tSwitch),
    &c13_thisId);
  sf_mex_destroy(&c13_b_tSwitch);
  return c13_y;
}

static real_T c13_ib_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId)
{
  real_T c13_y;
  real_T c13_d1;
  if (mxIsEmpty(c13_u)) {
    chartInstance->c13_tSwitch_not_empty = FALSE;
  } else {
    chartInstance->c13_tSwitch_not_empty = TRUE;
    sf_mex_import(c13_parentId, sf_mex_dup(c13_u), &c13_d1, 1, 0, 0U, 0, 0U, 0);
    c13_y = c13_d1;
  }

  sf_mex_destroy(&c13_u);
  return c13_y;
}

static void c13_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_b_tSwitch;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_b_tSwitch = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_y = c13_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_tSwitch),
    &c13_thisId);
  sf_mex_destroy(&c13_b_tSwitch);
  *(real_T *)c13_outData = c13_y;
  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_n_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  real_T c13_u;
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  c13_u = *(real_T *)c13_inData;
  c13_y = NULL;
  if (!chartInstance->c13_state_not_empty) {
    sf_mex_assign(&c13_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c13_y, sf_mex_create("y", &c13_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static real_T c13_jb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_b_state, const char_T *c13_identifier)
{
  real_T c13_y;
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_y = c13_kb_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_state),
    &c13_thisId);
  sf_mex_destroy(&c13_b_state);
  return c13_y;
}

static real_T c13_kb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId)
{
  real_T c13_y;
  real_T c13_d2;
  if (mxIsEmpty(c13_u)) {
    chartInstance->c13_state_not_empty = FALSE;
  } else {
    chartInstance->c13_state_not_empty = TRUE;
    sf_mex_import(c13_parentId, sf_mex_dup(c13_u), &c13_d2, 1, 0, 0U, 0, 0U, 0);
    c13_y = c13_d2;
  }

  sf_mex_destroy(&c13_u);
  return c13_y;
}

static void c13_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_b_state;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_b_state = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_y = c13_kb_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_state),
    &c13_thisId);
  sf_mex_destroy(&c13_b_state);
  *(real_T *)c13_outData = c13_y;
  sf_mex_destroy(&c13_mxArrayInData);
}

static void c13_lb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[6])
{
  real_T c13_dv44[6];
  int32_T c13_i503;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv44, 1, 0, 0U, 1, 0U, 1, 6);
  for (c13_i503 = 0; c13_i503 < 6; c13_i503++) {
    c13_y[c13_i503] = c13_dv44[c13_i503];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_wrench_leftFoot;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[6];
  int32_T c13_i504;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_wrench_leftFoot = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_lb_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_wrench_leftFoot),
    &c13_thisId, c13_y);
  sf_mex_destroy(&c13_wrench_leftFoot);
  for (c13_i504 = 0; c13_i504 < 6; c13_i504++) {
    (*(real_T (*)[6])c13_outData)[c13_i504] = c13_y[c13_i504];
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

static const mxArray *c13_o_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_i505;
  real_T c13_b_inData[4];
  int32_T c13_i506;
  real_T c13_u[4];
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  for (c13_i505 = 0; c13_i505 < 4; c13_i505++) {
    c13_b_inData[c13_i505] = (*(real_T (*)[4])c13_inData)[c13_i505];
  }

  for (c13_i506 = 0; c13_i506 < 4; c13_i506++) {
    c13_u[c13_i506] = c13_b_inData[c13_i506];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static void c13_mb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId,
  real_T c13_y[4])
{
  real_T c13_dv45[4];
  int32_T c13_i507;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), c13_dv45, 1, 0, 0U, 1, 0U, 1, 4);
  for (c13_i507 = 0; c13_i507 < 4; c13_i507++) {
    c13_y[c13_i507] = c13_dv45[c13_i507];
  }

  sf_mex_destroy(&c13_u);
}

static void c13_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_fixed_link_CoMDes;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  real_T c13_y[4];
  int32_T c13_i508;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_fixed_link_CoMDes = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_mb_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_fixed_link_CoMDes),
    &c13_thisId, c13_y);
  sf_mex_destroy(&c13_fixed_link_CoMDes);
  for (c13_i508 = 0; c13_i508 < 4; c13_i508++) {
    (*(real_T (*)[4])c13_outData)[c13_i508] = c13_y[c13_i508];
  }

  sf_mex_destroy(&c13_mxArrayInData);
}

const mxArray *sf_c13_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c13_nameCaptureInfo;
  c13_ResolvedFunctionInfo c13_info[145];
  const mxArray *c13_m6 = NULL;
  int32_T c13_i509;
  c13_ResolvedFunctionInfo *c13_r2;
  c13_nameCaptureInfo = NULL;
  c13_nameCaptureInfo = NULL;
  c13_info_helper(c13_info);
  c13_b_info_helper(c13_info);
  c13_c_info_helper(c13_info);
  sf_mex_assign(&c13_m6, sf_mex_createstruct("nameCaptureInfo", 1, 145), FALSE);
  for (c13_i509 = 0; c13_i509 < 145; c13_i509++) {
    c13_r2 = &c13_info[c13_i509];
    sf_mex_addfield(c13_m6, sf_mex_create("nameCaptureInfo", c13_r2->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c13_r2->context)), "context", "nameCaptureInfo",
                    c13_i509);
    sf_mex_addfield(c13_m6, sf_mex_create("nameCaptureInfo", c13_r2->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c13_r2->name)), "name", "nameCaptureInfo",
                    c13_i509);
    sf_mex_addfield(c13_m6, sf_mex_create("nameCaptureInfo",
      c13_r2->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c13_r2->dominantType)),
                    "dominantType", "nameCaptureInfo", c13_i509);
    sf_mex_addfield(c13_m6, sf_mex_create("nameCaptureInfo", c13_r2->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c13_r2->resolved)), "resolved",
                    "nameCaptureInfo", c13_i509);
    sf_mex_addfield(c13_m6, sf_mex_create("nameCaptureInfo", &c13_r2->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c13_i509);
    sf_mex_addfield(c13_m6, sf_mex_create("nameCaptureInfo", &c13_r2->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c13_i509);
    sf_mex_addfield(c13_m6, sf_mex_create("nameCaptureInfo",
      &c13_r2->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c13_i509);
    sf_mex_addfield(c13_m6, sf_mex_create("nameCaptureInfo",
      &c13_r2->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c13_i509);
  }

  sf_mex_assign(&c13_nameCaptureInfo, c13_m6, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c13_nameCaptureInfo);
  return c13_nameCaptureInfo;
}

static void c13_info_helper(c13_ResolvedFunctionInfo c13_info[145])
{
  c13_info[0].context = "";
  c13_info[0].name = "stateMachine";
  c13_info[0].dominantType = "struct";
  c13_info[0].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m";
  c13_info[0].fileTimeLo = 1495631764U;
  c13_info[0].fileTimeHi = 0U;
  c13_info[0].mFileTimeLo = 0U;
  c13_info[0].mFileTimeHi = 0U;
  c13_info[1].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m";
  c13_info[1].name = "eye";
  c13_info[1].dominantType = "double";
  c13_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c13_info[1].fileTimeLo = 1286818688U;
  c13_info[1].fileTimeHi = 0U;
  c13_info[1].mFileTimeLo = 0U;
  c13_info[1].mFileTimeHi = 0U;
  c13_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c13_info[2].name = "eml_assert_valid_size_arg";
  c13_info[2].dominantType = "double";
  c13_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c13_info[2].fileTimeLo = 1286818694U;
  c13_info[2].fileTimeHi = 0U;
  c13_info[2].mFileTimeLo = 0U;
  c13_info[2].mFileTimeHi = 0U;
  c13_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c13_info[3].name = "isinf";
  c13_info[3].dominantType = "double";
  c13_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c13_info[3].fileTimeLo = 1286818760U;
  c13_info[3].fileTimeHi = 0U;
  c13_info[3].mFileTimeLo = 0U;
  c13_info[3].mFileTimeHi = 0U;
  c13_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c13_info[4].name = "mtimes";
  c13_info[4].dominantType = "double";
  c13_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c13_info[4].fileTimeLo = 1289519692U;
  c13_info[4].fileTimeHi = 0U;
  c13_info[4].mFileTimeLo = 0U;
  c13_info[4].mFileTimeHi = 0U;
  c13_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c13_info[5].name = "eml_index_class";
  c13_info[5].dominantType = "";
  c13_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[5].fileTimeLo = 1323170578U;
  c13_info[5].fileTimeHi = 0U;
  c13_info[5].mFileTimeLo = 0U;
  c13_info[5].mFileTimeHi = 0U;
  c13_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c13_info[6].name = "intmax";
  c13_info[6].dominantType = "char";
  c13_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c13_info[6].fileTimeLo = 1311255316U;
  c13_info[6].fileTimeHi = 0U;
  c13_info[6].mFileTimeLo = 0U;
  c13_info[6].mFileTimeHi = 0U;
  c13_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c13_info[7].name = "eml_is_float_class";
  c13_info[7].dominantType = "char";
  c13_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c13_info[7].fileTimeLo = 1286818782U;
  c13_info[7].fileTimeHi = 0U;
  c13_info[7].mFileTimeLo = 0U;
  c13_info[7].mFileTimeHi = 0U;
  c13_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c13_info[8].name = "min";
  c13_info[8].dominantType = "double";
  c13_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c13_info[8].fileTimeLo = 1311255318U;
  c13_info[8].fileTimeHi = 0U;
  c13_info[8].mFileTimeLo = 0U;
  c13_info[8].mFileTimeHi = 0U;
  c13_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c13_info[9].name = "eml_min_or_max";
  c13_info[9].dominantType = "char";
  c13_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c13_info[9].fileTimeLo = 1334071490U;
  c13_info[9].fileTimeHi = 0U;
  c13_info[9].mFileTimeLo = 0U;
  c13_info[9].mFileTimeHi = 0U;
  c13_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c13_info[10].name = "eml_scalar_eg";
  c13_info[10].dominantType = "double";
  c13_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[10].fileTimeLo = 1286818796U;
  c13_info[10].fileTimeHi = 0U;
  c13_info[10].mFileTimeLo = 0U;
  c13_info[10].mFileTimeHi = 0U;
  c13_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c13_info[11].name = "eml_scalexp_alloc";
  c13_info[11].dominantType = "double";
  c13_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c13_info[11].fileTimeLo = 1352424860U;
  c13_info[11].fileTimeHi = 0U;
  c13_info[11].mFileTimeLo = 0U;
  c13_info[11].mFileTimeHi = 0U;
  c13_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c13_info[12].name = "eml_index_class";
  c13_info[12].dominantType = "";
  c13_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[12].fileTimeLo = 1323170578U;
  c13_info[12].fileTimeHi = 0U;
  c13_info[12].mFileTimeLo = 0U;
  c13_info[12].mFileTimeHi = 0U;
  c13_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c13_info[13].name = "eml_scalar_eg";
  c13_info[13].dominantType = "double";
  c13_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[13].fileTimeLo = 1286818796U;
  c13_info[13].fileTimeHi = 0U;
  c13_info[13].mFileTimeLo = 0U;
  c13_info[13].mFileTimeHi = 0U;
  c13_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c13_info[14].name = "eml_index_class";
  c13_info[14].dominantType = "";
  c13_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[14].fileTimeLo = 1323170578U;
  c13_info[14].fileTimeHi = 0U;
  c13_info[14].mFileTimeLo = 0U;
  c13_info[14].mFileTimeHi = 0U;
  c13_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c13_info[15].name = "eml_int_forloop_overflow_check";
  c13_info[15].dominantType = "";
  c13_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[15].fileTimeLo = 1346510340U;
  c13_info[15].fileTimeHi = 0U;
  c13_info[15].mFileTimeLo = 0U;
  c13_info[15].mFileTimeHi = 0U;
  c13_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c13_info[16].name = "intmax";
  c13_info[16].dominantType = "char";
  c13_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c13_info[16].fileTimeLo = 1311255316U;
  c13_info[16].fileTimeHi = 0U;
  c13_info[16].mFileTimeLo = 0U;
  c13_info[16].mFileTimeHi = 0U;
  c13_info[17].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m";
  c13_info[17].name = "mtimes";
  c13_info[17].dominantType = "double";
  c13_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c13_info[17].fileTimeLo = 1289519692U;
  c13_info[17].fileTimeHi = 0U;
  c13_info[17].mFileTimeLo = 0U;
  c13_info[17].mFileTimeHi = 0U;
  c13_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c13_info[18].name = "eml_index_class";
  c13_info[18].dominantType = "";
  c13_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[18].fileTimeLo = 1323170578U;
  c13_info[18].fileTimeHi = 0U;
  c13_info[18].mFileTimeLo = 0U;
  c13_info[18].mFileTimeHi = 0U;
  c13_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c13_info[19].name = "eml_scalar_eg";
  c13_info[19].dominantType = "double";
  c13_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[19].fileTimeLo = 1286818796U;
  c13_info[19].fileTimeHi = 0U;
  c13_info[19].mFileTimeLo = 0U;
  c13_info[19].mFileTimeHi = 0U;
  c13_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c13_info[20].name = "eml_xgemm";
  c13_info[20].dominantType = "char";
  c13_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c13_info[20].fileTimeLo = 1299076772U;
  c13_info[20].fileTimeHi = 0U;
  c13_info[20].mFileTimeLo = 0U;
  c13_info[20].mFileTimeHi = 0U;
  c13_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c13_info[21].name = "eml_blas_inline";
  c13_info[21].dominantType = "";
  c13_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c13_info[21].fileTimeLo = 1299076768U;
  c13_info[21].fileTimeHi = 0U;
  c13_info[21].mFileTimeLo = 0U;
  c13_info[21].mFileTimeHi = 0U;
  c13_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c13_info[22].name = "mtimes";
  c13_info[22].dominantType = "double";
  c13_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c13_info[22].fileTimeLo = 1289519692U;
  c13_info[22].fileTimeHi = 0U;
  c13_info[22].mFileTimeLo = 0U;
  c13_info[22].mFileTimeHi = 0U;
  c13_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c13_info[23].name = "eml_index_class";
  c13_info[23].dominantType = "";
  c13_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[23].fileTimeLo = 1323170578U;
  c13_info[23].fileTimeHi = 0U;
  c13_info[23].mFileTimeLo = 0U;
  c13_info[23].mFileTimeHi = 0U;
  c13_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c13_info[24].name = "eml_scalar_eg";
  c13_info[24].dominantType = "double";
  c13_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[24].fileTimeLo = 1286818796U;
  c13_info[24].fileTimeHi = 0U;
  c13_info[24].mFileTimeLo = 0U;
  c13_info[24].mFileTimeHi = 0U;
  c13_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c13_info[25].name = "eml_refblas_xgemm";
  c13_info[25].dominantType = "char";
  c13_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c13_info[25].fileTimeLo = 1299076774U;
  c13_info[25].fileTimeHi = 0U;
  c13_info[25].mFileTimeLo = 0U;
  c13_info[25].mFileTimeHi = 0U;
  c13_info[26].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m";
  c13_info[26].name = "mrdivide";
  c13_info[26].dominantType = "double";
  c13_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c13_info[26].fileTimeLo = 1357951548U;
  c13_info[26].fileTimeHi = 0U;
  c13_info[26].mFileTimeLo = 1319729966U;
  c13_info[26].mFileTimeHi = 0U;
  c13_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c13_info[27].name = "mldivide";
  c13_info[27].dominantType = "double";
  c13_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c13_info[27].fileTimeLo = 1357951548U;
  c13_info[27].fileTimeHi = 0U;
  c13_info[27].mFileTimeLo = 1319729966U;
  c13_info[27].mFileTimeHi = 0U;
  c13_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c13_info[28].name = "eml_lusolve";
  c13_info[28].dominantType = "double";
  c13_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c13_info[28].fileTimeLo = 1309451196U;
  c13_info[28].fileTimeHi = 0U;
  c13_info[28].mFileTimeLo = 0U;
  c13_info[28].mFileTimeHi = 0U;
  c13_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c13_info[29].name = "eml_index_class";
  c13_info[29].dominantType = "";
  c13_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[29].fileTimeLo = 1323170578U;
  c13_info[29].fileTimeHi = 0U;
  c13_info[29].mFileTimeLo = 0U;
  c13_info[29].mFileTimeHi = 0U;
  c13_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c13_info[30].name = "eml_index_class";
  c13_info[30].dominantType = "";
  c13_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[30].fileTimeLo = 1323170578U;
  c13_info[30].fileTimeHi = 0U;
  c13_info[30].mFileTimeLo = 0U;
  c13_info[30].mFileTimeHi = 0U;
  c13_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c13_info[31].name = "eml_xgetrf";
  c13_info[31].dominantType = "double";
  c13_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c13_info[31].fileTimeLo = 1286818806U;
  c13_info[31].fileTimeHi = 0U;
  c13_info[31].mFileTimeLo = 0U;
  c13_info[31].mFileTimeHi = 0U;
  c13_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c13_info[32].name = "eml_lapack_xgetrf";
  c13_info[32].dominantType = "double";
  c13_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c13_info[32].fileTimeLo = 1286818810U;
  c13_info[32].fileTimeHi = 0U;
  c13_info[32].mFileTimeLo = 0U;
  c13_info[32].mFileTimeHi = 0U;
  c13_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c13_info[33].name = "eml_matlab_zgetrf";
  c13_info[33].dominantType = "double";
  c13_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[33].fileTimeLo = 1302688994U;
  c13_info[33].fileTimeHi = 0U;
  c13_info[33].mFileTimeLo = 0U;
  c13_info[33].mFileTimeHi = 0U;
  c13_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[34].name = "realmin";
  c13_info[34].dominantType = "char";
  c13_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c13_info[34].fileTimeLo = 1307651242U;
  c13_info[34].fileTimeHi = 0U;
  c13_info[34].mFileTimeLo = 0U;
  c13_info[34].mFileTimeHi = 0U;
  c13_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c13_info[35].name = "eml_realmin";
  c13_info[35].dominantType = "char";
  c13_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c13_info[35].fileTimeLo = 1307651244U;
  c13_info[35].fileTimeHi = 0U;
  c13_info[35].mFileTimeLo = 0U;
  c13_info[35].mFileTimeHi = 0U;
  c13_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c13_info[36].name = "eml_float_model";
  c13_info[36].dominantType = "char";
  c13_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c13_info[36].fileTimeLo = 1326727996U;
  c13_info[36].fileTimeHi = 0U;
  c13_info[36].mFileTimeLo = 0U;
  c13_info[36].mFileTimeHi = 0U;
  c13_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[37].name = "eps";
  c13_info[37].dominantType = "char";
  c13_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c13_info[37].fileTimeLo = 1326727996U;
  c13_info[37].fileTimeHi = 0U;
  c13_info[37].mFileTimeLo = 0U;
  c13_info[37].mFileTimeHi = 0U;
  c13_info[38].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c13_info[38].name = "eml_is_float_class";
  c13_info[38].dominantType = "char";
  c13_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c13_info[38].fileTimeLo = 1286818782U;
  c13_info[38].fileTimeHi = 0U;
  c13_info[38].mFileTimeLo = 0U;
  c13_info[38].mFileTimeHi = 0U;
  c13_info[39].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c13_info[39].name = "eml_eps";
  c13_info[39].dominantType = "char";
  c13_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c13_info[39].fileTimeLo = 1326727996U;
  c13_info[39].fileTimeHi = 0U;
  c13_info[39].mFileTimeLo = 0U;
  c13_info[39].mFileTimeHi = 0U;
  c13_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c13_info[40].name = "eml_float_model";
  c13_info[40].dominantType = "char";
  c13_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c13_info[40].fileTimeLo = 1326727996U;
  c13_info[40].fileTimeHi = 0U;
  c13_info[40].mFileTimeLo = 0U;
  c13_info[40].mFileTimeHi = 0U;
  c13_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[41].name = "min";
  c13_info[41].dominantType = "coder.internal.indexInt";
  c13_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c13_info[41].fileTimeLo = 1311255318U;
  c13_info[41].fileTimeHi = 0U;
  c13_info[41].mFileTimeLo = 0U;
  c13_info[41].mFileTimeHi = 0U;
  c13_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c13_info[42].name = "eml_scalar_eg";
  c13_info[42].dominantType = "coder.internal.indexInt";
  c13_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[42].fileTimeLo = 1286818796U;
  c13_info[42].fileTimeHi = 0U;
  c13_info[42].mFileTimeLo = 0U;
  c13_info[42].mFileTimeHi = 0U;
  c13_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c13_info[43].name = "eml_scalexp_alloc";
  c13_info[43].dominantType = "coder.internal.indexInt";
  c13_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c13_info[43].fileTimeLo = 1352424860U;
  c13_info[43].fileTimeHi = 0U;
  c13_info[43].mFileTimeLo = 0U;
  c13_info[43].mFileTimeHi = 0U;
  c13_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c13_info[44].name = "eml_scalar_eg";
  c13_info[44].dominantType = "coder.internal.indexInt";
  c13_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[44].fileTimeLo = 1286818796U;
  c13_info[44].fileTimeHi = 0U;
  c13_info[44].mFileTimeLo = 0U;
  c13_info[44].mFileTimeHi = 0U;
  c13_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[45].name = "colon";
  c13_info[45].dominantType = "double";
  c13_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c13_info[45].fileTimeLo = 1348191928U;
  c13_info[45].fileTimeHi = 0U;
  c13_info[45].mFileTimeLo = 0U;
  c13_info[45].mFileTimeHi = 0U;
  c13_info[46].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c13_info[46].name = "colon";
  c13_info[46].dominantType = "double";
  c13_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c13_info[46].fileTimeLo = 1348191928U;
  c13_info[46].fileTimeHi = 0U;
  c13_info[46].mFileTimeLo = 0U;
  c13_info[46].mFileTimeHi = 0U;
  c13_info[47].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c13_info[47].name = "floor";
  c13_info[47].dominantType = "double";
  c13_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c13_info[47].fileTimeLo = 1343830380U;
  c13_info[47].fileTimeHi = 0U;
  c13_info[47].mFileTimeLo = 0U;
  c13_info[47].mFileTimeHi = 0U;
  c13_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c13_info[48].name = "eml_scalar_floor";
  c13_info[48].dominantType = "double";
  c13_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c13_info[48].fileTimeLo = 1286818726U;
  c13_info[48].fileTimeHi = 0U;
  c13_info[48].mFileTimeLo = 0U;
  c13_info[48].mFileTimeHi = 0U;
  c13_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c13_info[49].name = "intmin";
  c13_info[49].dominantType = "char";
  c13_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c13_info[49].fileTimeLo = 1311255318U;
  c13_info[49].fileTimeHi = 0U;
  c13_info[49].mFileTimeLo = 0U;
  c13_info[49].mFileTimeHi = 0U;
  c13_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c13_info[50].name = "intmax";
  c13_info[50].dominantType = "char";
  c13_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c13_info[50].fileTimeLo = 1311255316U;
  c13_info[50].fileTimeHi = 0U;
  c13_info[50].mFileTimeLo = 0U;
  c13_info[50].mFileTimeHi = 0U;
  c13_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c13_info[51].name = "intmin";
  c13_info[51].dominantType = "char";
  c13_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c13_info[51].fileTimeLo = 1311255318U;
  c13_info[51].fileTimeHi = 0U;
  c13_info[51].mFileTimeLo = 0U;
  c13_info[51].mFileTimeHi = 0U;
  c13_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c13_info[52].name = "intmax";
  c13_info[52].dominantType = "char";
  c13_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c13_info[52].fileTimeLo = 1311255316U;
  c13_info[52].fileTimeHi = 0U;
  c13_info[52].mFileTimeLo = 0U;
  c13_info[52].mFileTimeHi = 0U;
  c13_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c13_info[53].name = "eml_isa_uint";
  c13_info[53].dominantType = "coder.internal.indexInt";
  c13_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c13_info[53].fileTimeLo = 1286818784U;
  c13_info[53].fileTimeHi = 0U;
  c13_info[53].mFileTimeLo = 0U;
  c13_info[53].mFileTimeHi = 0U;
  c13_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c13_info[54].name = "eml_unsigned_class";
  c13_info[54].dominantType = "char";
  c13_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c13_info[54].fileTimeLo = 1323170580U;
  c13_info[54].fileTimeHi = 0U;
  c13_info[54].mFileTimeLo = 0U;
  c13_info[54].mFileTimeHi = 0U;
  c13_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c13_info[55].name = "eml_index_class";
  c13_info[55].dominantType = "";
  c13_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[55].fileTimeLo = 1323170578U;
  c13_info[55].fileTimeHi = 0U;
  c13_info[55].mFileTimeLo = 0U;
  c13_info[55].mFileTimeHi = 0U;
  c13_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c13_info[56].name = "eml_index_class";
  c13_info[56].dominantType = "";
  c13_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[56].fileTimeLo = 1323170578U;
  c13_info[56].fileTimeHi = 0U;
  c13_info[56].mFileTimeLo = 0U;
  c13_info[56].mFileTimeHi = 0U;
  c13_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c13_info[57].name = "intmax";
  c13_info[57].dominantType = "char";
  c13_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c13_info[57].fileTimeLo = 1311255316U;
  c13_info[57].fileTimeHi = 0U;
  c13_info[57].mFileTimeLo = 0U;
  c13_info[57].mFileTimeHi = 0U;
  c13_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c13_info[58].name = "eml_isa_uint";
  c13_info[58].dominantType = "coder.internal.indexInt";
  c13_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c13_info[58].fileTimeLo = 1286818784U;
  c13_info[58].fileTimeHi = 0U;
  c13_info[58].mFileTimeLo = 0U;
  c13_info[58].mFileTimeHi = 0U;
  c13_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c13_info[59].name = "eml_index_plus";
  c13_info[59].dominantType = "double";
  c13_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[59].fileTimeLo = 1286818778U;
  c13_info[59].fileTimeHi = 0U;
  c13_info[59].mFileTimeLo = 0U;
  c13_info[59].mFileTimeHi = 0U;
  c13_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[60].name = "eml_index_class";
  c13_info[60].dominantType = "";
  c13_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[60].fileTimeLo = 1323170578U;
  c13_info[60].fileTimeHi = 0U;
  c13_info[60].mFileTimeLo = 0U;
  c13_info[60].mFileTimeHi = 0U;
  c13_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c13_info[61].name = "eml_int_forloop_overflow_check";
  c13_info[61].dominantType = "";
  c13_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[61].fileTimeLo = 1346510340U;
  c13_info[61].fileTimeHi = 0U;
  c13_info[61].mFileTimeLo = 0U;
  c13_info[61].mFileTimeHi = 0U;
  c13_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[62].name = "eml_index_class";
  c13_info[62].dominantType = "";
  c13_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[62].fileTimeLo = 1323170578U;
  c13_info[62].fileTimeHi = 0U;
  c13_info[62].mFileTimeLo = 0U;
  c13_info[62].mFileTimeHi = 0U;
  c13_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[63].name = "eml_index_plus";
  c13_info[63].dominantType = "double";
  c13_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[63].fileTimeLo = 1286818778U;
  c13_info[63].fileTimeHi = 0U;
  c13_info[63].mFileTimeLo = 0U;
  c13_info[63].mFileTimeHi = 0U;
}

static void c13_b_info_helper(c13_ResolvedFunctionInfo c13_info[145])
{
  c13_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[64].name = "eml_int_forloop_overflow_check";
  c13_info[64].dominantType = "";
  c13_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[64].fileTimeLo = 1346510340U;
  c13_info[64].fileTimeHi = 0U;
  c13_info[64].mFileTimeLo = 0U;
  c13_info[64].mFileTimeHi = 0U;
  c13_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[65].name = "eml_index_minus";
  c13_info[65].dominantType = "double";
  c13_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c13_info[65].fileTimeLo = 1286818778U;
  c13_info[65].fileTimeHi = 0U;
  c13_info[65].mFileTimeLo = 0U;
  c13_info[65].mFileTimeHi = 0U;
  c13_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c13_info[66].name = "eml_index_class";
  c13_info[66].dominantType = "";
  c13_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[66].fileTimeLo = 1323170578U;
  c13_info[66].fileTimeHi = 0U;
  c13_info[66].mFileTimeLo = 0U;
  c13_info[66].mFileTimeHi = 0U;
  c13_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[67].name = "eml_index_minus";
  c13_info[67].dominantType = "coder.internal.indexInt";
  c13_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c13_info[67].fileTimeLo = 1286818778U;
  c13_info[67].fileTimeHi = 0U;
  c13_info[67].mFileTimeLo = 0U;
  c13_info[67].mFileTimeHi = 0U;
  c13_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[68].name = "eml_index_times";
  c13_info[68].dominantType = "coder.internal.indexInt";
  c13_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c13_info[68].fileTimeLo = 1286818780U;
  c13_info[68].fileTimeHi = 0U;
  c13_info[68].mFileTimeLo = 0U;
  c13_info[68].mFileTimeHi = 0U;
  c13_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c13_info[69].name = "eml_index_class";
  c13_info[69].dominantType = "";
  c13_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[69].fileTimeLo = 1323170578U;
  c13_info[69].fileTimeHi = 0U;
  c13_info[69].mFileTimeLo = 0U;
  c13_info[69].mFileTimeHi = 0U;
  c13_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[70].name = "eml_index_plus";
  c13_info[70].dominantType = "coder.internal.indexInt";
  c13_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[70].fileTimeLo = 1286818778U;
  c13_info[70].fileTimeHi = 0U;
  c13_info[70].mFileTimeLo = 0U;
  c13_info[70].mFileTimeHi = 0U;
  c13_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[71].name = "eml_ixamax";
  c13_info[71].dominantType = "double";
  c13_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c13_info[71].fileTimeLo = 1299076770U;
  c13_info[71].fileTimeHi = 0U;
  c13_info[71].mFileTimeLo = 0U;
  c13_info[71].mFileTimeHi = 0U;
  c13_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c13_info[72].name = "eml_blas_inline";
  c13_info[72].dominantType = "";
  c13_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c13_info[72].fileTimeLo = 1299076768U;
  c13_info[72].fileTimeHi = 0U;
  c13_info[72].mFileTimeLo = 0U;
  c13_info[72].mFileTimeHi = 0U;
  c13_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c13_info[73].name = "length";
  c13_info[73].dominantType = "double";
  c13_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c13_info[73].fileTimeLo = 1303146206U;
  c13_info[73].fileTimeHi = 0U;
  c13_info[73].mFileTimeLo = 0U;
  c13_info[73].mFileTimeHi = 0U;
  c13_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c13_info[74].name = "eml_index_class";
  c13_info[74].dominantType = "";
  c13_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[74].fileTimeLo = 1323170578U;
  c13_info[74].fileTimeHi = 0U;
  c13_info[74].mFileTimeLo = 0U;
  c13_info[74].mFileTimeHi = 0U;
  c13_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c13_info[75].name = "eml_index_class";
  c13_info[75].dominantType = "";
  c13_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[75].fileTimeLo = 1323170578U;
  c13_info[75].fileTimeHi = 0U;
  c13_info[75].mFileTimeLo = 0U;
  c13_info[75].mFileTimeHi = 0U;
  c13_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c13_info[76].name = "eml_refblas_ixamax";
  c13_info[76].dominantType = "double";
  c13_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c13_info[76].fileTimeLo = 1299076770U;
  c13_info[76].fileTimeHi = 0U;
  c13_info[76].mFileTimeLo = 0U;
  c13_info[76].mFileTimeHi = 0U;
  c13_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c13_info[77].name = "eml_index_class";
  c13_info[77].dominantType = "";
  c13_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[77].fileTimeLo = 1323170578U;
  c13_info[77].fileTimeHi = 0U;
  c13_info[77].mFileTimeLo = 0U;
  c13_info[77].mFileTimeHi = 0U;
  c13_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c13_info[78].name = "eml_xcabs1";
  c13_info[78].dominantType = "double";
  c13_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c13_info[78].fileTimeLo = 1286818706U;
  c13_info[78].fileTimeHi = 0U;
  c13_info[78].mFileTimeLo = 0U;
  c13_info[78].mFileTimeHi = 0U;
  c13_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c13_info[79].name = "abs";
  c13_info[79].dominantType = "double";
  c13_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c13_info[79].fileTimeLo = 1343830366U;
  c13_info[79].fileTimeHi = 0U;
  c13_info[79].mFileTimeLo = 0U;
  c13_info[79].mFileTimeHi = 0U;
  c13_info[80].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c13_info[80].name = "eml_scalar_abs";
  c13_info[80].dominantType = "double";
  c13_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c13_info[80].fileTimeLo = 1286818712U;
  c13_info[80].fileTimeHi = 0U;
  c13_info[80].mFileTimeLo = 0U;
  c13_info[80].mFileTimeHi = 0U;
  c13_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c13_info[81].name = "eml_int_forloop_overflow_check";
  c13_info[81].dominantType = "";
  c13_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[81].fileTimeLo = 1346510340U;
  c13_info[81].fileTimeHi = 0U;
  c13_info[81].mFileTimeLo = 0U;
  c13_info[81].mFileTimeHi = 0U;
  c13_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c13_info[82].name = "eml_index_plus";
  c13_info[82].dominantType = "coder.internal.indexInt";
  c13_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[82].fileTimeLo = 1286818778U;
  c13_info[82].fileTimeHi = 0U;
  c13_info[82].mFileTimeLo = 0U;
  c13_info[82].mFileTimeHi = 0U;
  c13_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[83].name = "eml_xswap";
  c13_info[83].dominantType = "double";
  c13_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c13_info[83].fileTimeLo = 1299076778U;
  c13_info[83].fileTimeHi = 0U;
  c13_info[83].mFileTimeLo = 0U;
  c13_info[83].mFileTimeHi = 0U;
  c13_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c13_info[84].name = "eml_blas_inline";
  c13_info[84].dominantType = "";
  c13_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c13_info[84].fileTimeLo = 1299076768U;
  c13_info[84].fileTimeHi = 0U;
  c13_info[84].mFileTimeLo = 0U;
  c13_info[84].mFileTimeHi = 0U;
  c13_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c13_info[85].name = "eml_index_class";
  c13_info[85].dominantType = "";
  c13_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[85].fileTimeLo = 1323170578U;
  c13_info[85].fileTimeHi = 0U;
  c13_info[85].mFileTimeLo = 0U;
  c13_info[85].mFileTimeHi = 0U;
  c13_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c13_info[86].name = "eml_refblas_xswap";
  c13_info[86].dominantType = "double";
  c13_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c13_info[86].fileTimeLo = 1299076786U;
  c13_info[86].fileTimeHi = 0U;
  c13_info[86].mFileTimeLo = 0U;
  c13_info[86].mFileTimeHi = 0U;
  c13_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c13_info[87].name = "eml_index_class";
  c13_info[87].dominantType = "";
  c13_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[87].fileTimeLo = 1323170578U;
  c13_info[87].fileTimeHi = 0U;
  c13_info[87].mFileTimeLo = 0U;
  c13_info[87].mFileTimeHi = 0U;
  c13_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c13_info[88].name = "abs";
  c13_info[88].dominantType = "coder.internal.indexInt";
  c13_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c13_info[88].fileTimeLo = 1343830366U;
  c13_info[88].fileTimeHi = 0U;
  c13_info[88].mFileTimeLo = 0U;
  c13_info[88].mFileTimeHi = 0U;
  c13_info[89].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c13_info[89].name = "eml_scalar_abs";
  c13_info[89].dominantType = "coder.internal.indexInt";
  c13_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c13_info[89].fileTimeLo = 1286818712U;
  c13_info[89].fileTimeHi = 0U;
  c13_info[89].mFileTimeLo = 0U;
  c13_info[89].mFileTimeHi = 0U;
  c13_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c13_info[90].name = "eml_int_forloop_overflow_check";
  c13_info[90].dominantType = "";
  c13_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[90].fileTimeLo = 1346510340U;
  c13_info[90].fileTimeHi = 0U;
  c13_info[90].mFileTimeLo = 0U;
  c13_info[90].mFileTimeHi = 0U;
  c13_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c13_info[91].name = "eml_index_plus";
  c13_info[91].dominantType = "coder.internal.indexInt";
  c13_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[91].fileTimeLo = 1286818778U;
  c13_info[91].fileTimeHi = 0U;
  c13_info[91].mFileTimeLo = 0U;
  c13_info[91].mFileTimeHi = 0U;
  c13_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[92].name = "eml_div";
  c13_info[92].dominantType = "double";
  c13_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c13_info[92].fileTimeLo = 1313347810U;
  c13_info[92].fileTimeHi = 0U;
  c13_info[92].mFileTimeLo = 0U;
  c13_info[92].mFileTimeHi = 0U;
  c13_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c13_info[93].name = "eml_xgeru";
  c13_info[93].dominantType = "double";
  c13_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c13_info[93].fileTimeLo = 1299076774U;
  c13_info[93].fileTimeHi = 0U;
  c13_info[93].mFileTimeLo = 0U;
  c13_info[93].mFileTimeHi = 0U;
  c13_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c13_info[94].name = "eml_blas_inline";
  c13_info[94].dominantType = "";
  c13_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c13_info[94].fileTimeLo = 1299076768U;
  c13_info[94].fileTimeHi = 0U;
  c13_info[94].mFileTimeLo = 0U;
  c13_info[94].mFileTimeHi = 0U;
  c13_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c13_info[95].name = "eml_xger";
  c13_info[95].dominantType = "double";
  c13_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c13_info[95].fileTimeLo = 1299076774U;
  c13_info[95].fileTimeHi = 0U;
  c13_info[95].mFileTimeLo = 0U;
  c13_info[95].mFileTimeHi = 0U;
  c13_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c13_info[96].name = "eml_blas_inline";
  c13_info[96].dominantType = "";
  c13_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c13_info[96].fileTimeLo = 1299076768U;
  c13_info[96].fileTimeHi = 0U;
  c13_info[96].mFileTimeLo = 0U;
  c13_info[96].mFileTimeHi = 0U;
  c13_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c13_info[97].name = "intmax";
  c13_info[97].dominantType = "char";
  c13_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c13_info[97].fileTimeLo = 1311255316U;
  c13_info[97].fileTimeHi = 0U;
  c13_info[97].mFileTimeLo = 0U;
  c13_info[97].mFileTimeHi = 0U;
  c13_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c13_info[98].name = "min";
  c13_info[98].dominantType = "double";
  c13_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c13_info[98].fileTimeLo = 1311255318U;
  c13_info[98].fileTimeHi = 0U;
  c13_info[98].mFileTimeLo = 0U;
  c13_info[98].mFileTimeHi = 0U;
  c13_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c13_info[99].name = "mtimes";
  c13_info[99].dominantType = "double";
  c13_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c13_info[99].fileTimeLo = 1289519692U;
  c13_info[99].fileTimeHi = 0U;
  c13_info[99].mFileTimeLo = 0U;
  c13_info[99].mFileTimeHi = 0U;
  c13_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c13_info[100].name = "eml_index_class";
  c13_info[100].dominantType = "";
  c13_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[100].fileTimeLo = 1323170578U;
  c13_info[100].fileTimeHi = 0U;
  c13_info[100].mFileTimeLo = 0U;
  c13_info[100].mFileTimeHi = 0U;
  c13_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c13_info[101].name = "eml_refblas_xger";
  c13_info[101].dominantType = "double";
  c13_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c13_info[101].fileTimeLo = 1299076776U;
  c13_info[101].fileTimeHi = 0U;
  c13_info[101].mFileTimeLo = 0U;
  c13_info[101].mFileTimeHi = 0U;
  c13_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c13_info[102].name = "eml_refblas_xgerx";
  c13_info[102].dominantType = "char";
  c13_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c13_info[102].fileTimeLo = 1299076778U;
  c13_info[102].fileTimeHi = 0U;
  c13_info[102].mFileTimeLo = 0U;
  c13_info[102].mFileTimeHi = 0U;
  c13_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c13_info[103].name = "eml_index_class";
  c13_info[103].dominantType = "";
  c13_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[103].fileTimeLo = 1323170578U;
  c13_info[103].fileTimeHi = 0U;
  c13_info[103].mFileTimeLo = 0U;
  c13_info[103].mFileTimeHi = 0U;
  c13_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c13_info[104].name = "abs";
  c13_info[104].dominantType = "coder.internal.indexInt";
  c13_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c13_info[104].fileTimeLo = 1343830366U;
  c13_info[104].fileTimeHi = 0U;
  c13_info[104].mFileTimeLo = 0U;
  c13_info[104].mFileTimeHi = 0U;
  c13_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c13_info[105].name = "eml_index_minus";
  c13_info[105].dominantType = "double";
  c13_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c13_info[105].fileTimeLo = 1286818778U;
  c13_info[105].fileTimeHi = 0U;
  c13_info[105].mFileTimeLo = 0U;
  c13_info[105].mFileTimeHi = 0U;
  c13_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c13_info[106].name = "eml_int_forloop_overflow_check";
  c13_info[106].dominantType = "";
  c13_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[106].fileTimeLo = 1346510340U;
  c13_info[106].fileTimeHi = 0U;
  c13_info[106].mFileTimeLo = 0U;
  c13_info[106].mFileTimeHi = 0U;
  c13_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c13_info[107].name = "eml_index_plus";
  c13_info[107].dominantType = "double";
  c13_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[107].fileTimeLo = 1286818778U;
  c13_info[107].fileTimeHi = 0U;
  c13_info[107].mFileTimeLo = 0U;
  c13_info[107].mFileTimeHi = 0U;
  c13_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c13_info[108].name = "eml_index_plus";
  c13_info[108].dominantType = "coder.internal.indexInt";
  c13_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[108].fileTimeLo = 1286818778U;
  c13_info[108].fileTimeHi = 0U;
  c13_info[108].mFileTimeLo = 0U;
  c13_info[108].mFileTimeHi = 0U;
  c13_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c13_info[109].name = "eml_warning";
  c13_info[109].dominantType = "char";
  c13_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c13_info[109].fileTimeLo = 1286818802U;
  c13_info[109].fileTimeHi = 0U;
  c13_info[109].mFileTimeLo = 0U;
  c13_info[109].mFileTimeHi = 0U;
  c13_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c13_info[110].name = "eml_scalar_eg";
  c13_info[110].dominantType = "double";
  c13_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[110].fileTimeLo = 1286818796U;
  c13_info[110].fileTimeHi = 0U;
  c13_info[110].mFileTimeLo = 0U;
  c13_info[110].mFileTimeHi = 0U;
  c13_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c13_info[111].name = "eml_int_forloop_overflow_check";
  c13_info[111].dominantType = "";
  c13_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[111].fileTimeLo = 1346510340U;
  c13_info[111].fileTimeHi = 0U;
  c13_info[111].mFileTimeLo = 0U;
  c13_info[111].mFileTimeHi = 0U;
  c13_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c13_info[112].name = "eml_xtrsm";
  c13_info[112].dominantType = "char";
  c13_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c13_info[112].fileTimeLo = 1299076778U;
  c13_info[112].fileTimeHi = 0U;
  c13_info[112].mFileTimeLo = 0U;
  c13_info[112].mFileTimeHi = 0U;
  c13_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c13_info[113].name = "eml_blas_inline";
  c13_info[113].dominantType = "";
  c13_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c13_info[113].fileTimeLo = 1299076768U;
  c13_info[113].fileTimeHi = 0U;
  c13_info[113].mFileTimeLo = 0U;
  c13_info[113].mFileTimeHi = 0U;
  c13_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c13_info[114].name = "mtimes";
  c13_info[114].dominantType = "double";
  c13_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c13_info[114].fileTimeLo = 1289519692U;
  c13_info[114].fileTimeHi = 0U;
  c13_info[114].mFileTimeLo = 0U;
  c13_info[114].mFileTimeHi = 0U;
  c13_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c13_info[115].name = "eml_index_class";
  c13_info[115].dominantType = "";
  c13_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[115].fileTimeLo = 1323170578U;
  c13_info[115].fileTimeHi = 0U;
  c13_info[115].mFileTimeLo = 0U;
  c13_info[115].mFileTimeHi = 0U;
  c13_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c13_info[116].name = "eml_scalar_eg";
  c13_info[116].dominantType = "double";
  c13_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[116].fileTimeLo = 1286818796U;
  c13_info[116].fileTimeHi = 0U;
  c13_info[116].mFileTimeLo = 0U;
  c13_info[116].mFileTimeHi = 0U;
  c13_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c13_info[117].name = "eml_refblas_xtrsm";
  c13_info[117].dominantType = "char";
  c13_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[117].fileTimeLo = 1299076786U;
  c13_info[117].fileTimeHi = 0U;
  c13_info[117].mFileTimeLo = 0U;
  c13_info[117].mFileTimeHi = 0U;
  c13_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[118].name = "eml_scalar_eg";
  c13_info[118].dominantType = "double";
  c13_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c13_info[118].fileTimeLo = 1286818796U;
  c13_info[118].fileTimeHi = 0U;
  c13_info[118].mFileTimeLo = 0U;
  c13_info[118].mFileTimeHi = 0U;
  c13_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[119].name = "eml_index_minus";
  c13_info[119].dominantType = "double";
  c13_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c13_info[119].fileTimeLo = 1286818778U;
  c13_info[119].fileTimeHi = 0U;
  c13_info[119].mFileTimeLo = 0U;
  c13_info[119].mFileTimeHi = 0U;
  c13_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[120].name = "eml_index_class";
  c13_info[120].dominantType = "";
  c13_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[120].fileTimeLo = 1323170578U;
  c13_info[120].fileTimeHi = 0U;
  c13_info[120].mFileTimeLo = 0U;
  c13_info[120].mFileTimeHi = 0U;
  c13_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[121].name = "eml_int_forloop_overflow_check";
  c13_info[121].dominantType = "";
  c13_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[121].fileTimeLo = 1346510340U;
  c13_info[121].fileTimeHi = 0U;
  c13_info[121].mFileTimeLo = 0U;
  c13_info[121].mFileTimeHi = 0U;
  c13_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[122].name = "eml_index_times";
  c13_info[122].dominantType = "coder.internal.indexInt";
  c13_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c13_info[122].fileTimeLo = 1286818780U;
  c13_info[122].fileTimeHi = 0U;
  c13_info[122].mFileTimeLo = 0U;
  c13_info[122].mFileTimeHi = 0U;
  c13_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[123].name = "eml_index_plus";
  c13_info[123].dominantType = "coder.internal.indexInt";
  c13_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[123].fileTimeLo = 1286818778U;
  c13_info[123].fileTimeHi = 0U;
  c13_info[123].mFileTimeLo = 0U;
  c13_info[123].mFileTimeHi = 0U;
  c13_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[124].name = "eml_index_plus";
  c13_info[124].dominantType = "double";
  c13_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[124].fileTimeLo = 1286818778U;
  c13_info[124].fileTimeHi = 0U;
  c13_info[124].mFileTimeLo = 0U;
  c13_info[124].mFileTimeHi = 0U;
  c13_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c13_info[125].name = "intmin";
  c13_info[125].dominantType = "char";
  c13_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c13_info[125].fileTimeLo = 1311255318U;
  c13_info[125].fileTimeHi = 0U;
  c13_info[125].mFileTimeLo = 0U;
  c13_info[125].mFileTimeHi = 0U;
  c13_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c13_info[126].name = "eml_div";
  c13_info[126].dominantType = "double";
  c13_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c13_info[126].fileTimeLo = 1313347810U;
  c13_info[126].fileTimeHi = 0U;
  c13_info[126].mFileTimeLo = 0U;
  c13_info[126].mFileTimeHi = 0U;
  c13_info[127].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m";
  c13_info[127].name = "mldivide";
  c13_info[127].dominantType = "double";
  c13_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c13_info[127].fileTimeLo = 1357951548U;
  c13_info[127].fileTimeHi = 0U;
  c13_info[127].mFileTimeLo = 1319729966U;
  c13_info[127].mFileTimeHi = 0U;
}

static void c13_c_info_helper(c13_ResolvedFunctionInfo c13_info[145])
{
  c13_info[128].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m";
  c13_info[128].name = "norm";
  c13_info[128].dominantType = "double";
  c13_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c13_info[128].fileTimeLo = 1336522094U;
  c13_info[128].fileTimeHi = 0U;
  c13_info[128].mFileTimeLo = 0U;
  c13_info[128].mFileTimeHi = 0U;
  c13_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c13_info[129].name = "abs";
  c13_info[129].dominantType = "double";
  c13_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c13_info[129].fileTimeLo = 1343830366U;
  c13_info[129].fileTimeHi = 0U;
  c13_info[129].mFileTimeLo = 0U;
  c13_info[129].mFileTimeHi = 0U;
  c13_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c13_info[130].name = "eml_index_class";
  c13_info[130].dominantType = "";
  c13_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[130].fileTimeLo = 1323170578U;
  c13_info[130].fileTimeHi = 0U;
  c13_info[130].mFileTimeLo = 0U;
  c13_info[130].mFileTimeHi = 0U;
  c13_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c13_info[131].name = "eml_xnrm2";
  c13_info[131].dominantType = "double";
  c13_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c13_info[131].fileTimeLo = 1299076776U;
  c13_info[131].fileTimeHi = 0U;
  c13_info[131].mFileTimeLo = 0U;
  c13_info[131].mFileTimeHi = 0U;
  c13_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c13_info[132].name = "eml_blas_inline";
  c13_info[132].dominantType = "";
  c13_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c13_info[132].fileTimeLo = 1299076768U;
  c13_info[132].fileTimeHi = 0U;
  c13_info[132].mFileTimeLo = 0U;
  c13_info[132].mFileTimeHi = 0U;
  c13_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c13_info[133].name = "eml_index_class";
  c13_info[133].dominantType = "";
  c13_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[133].fileTimeLo = 1323170578U;
  c13_info[133].fileTimeHi = 0U;
  c13_info[133].mFileTimeLo = 0U;
  c13_info[133].mFileTimeHi = 0U;
  c13_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c13_info[134].name = "eml_refblas_xnrm2";
  c13_info[134].dominantType = "double";
  c13_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c13_info[134].fileTimeLo = 1299076784U;
  c13_info[134].fileTimeHi = 0U;
  c13_info[134].mFileTimeLo = 0U;
  c13_info[134].mFileTimeHi = 0U;
  c13_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c13_info[135].name = "realmin";
  c13_info[135].dominantType = "char";
  c13_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c13_info[135].fileTimeLo = 1307651242U;
  c13_info[135].fileTimeHi = 0U;
  c13_info[135].mFileTimeLo = 0U;
  c13_info[135].mFileTimeHi = 0U;
  c13_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c13_info[136].name = "eml_index_class";
  c13_info[136].dominantType = "";
  c13_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c13_info[136].fileTimeLo = 1323170578U;
  c13_info[136].fileTimeHi = 0U;
  c13_info[136].mFileTimeLo = 0U;
  c13_info[136].mFileTimeHi = 0U;
  c13_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c13_info[137].name = "eml_index_minus";
  c13_info[137].dominantType = "double";
  c13_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c13_info[137].fileTimeLo = 1286818778U;
  c13_info[137].fileTimeHi = 0U;
  c13_info[137].mFileTimeLo = 0U;
  c13_info[137].mFileTimeHi = 0U;
  c13_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c13_info[138].name = "eml_index_times";
  c13_info[138].dominantType = "coder.internal.indexInt";
  c13_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c13_info[138].fileTimeLo = 1286818780U;
  c13_info[138].fileTimeHi = 0U;
  c13_info[138].mFileTimeLo = 0U;
  c13_info[138].mFileTimeHi = 0U;
  c13_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c13_info[139].name = "eml_index_plus";
  c13_info[139].dominantType = "coder.internal.indexInt";
  c13_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c13_info[139].fileTimeLo = 1286818778U;
  c13_info[139].fileTimeHi = 0U;
  c13_info[139].mFileTimeLo = 0U;
  c13_info[139].mFileTimeHi = 0U;
  c13_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c13_info[140].name = "eml_int_forloop_overflow_check";
  c13_info[140].dominantType = "";
  c13_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c13_info[140].fileTimeLo = 1346510340U;
  c13_info[140].fileTimeHi = 0U;
  c13_info[140].mFileTimeLo = 0U;
  c13_info[140].mFileTimeHi = 0U;
  c13_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c13_info[141].name = "abs";
  c13_info[141].dominantType = "double";
  c13_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c13_info[141].fileTimeLo = 1343830366U;
  c13_info[141].fileTimeHi = 0U;
  c13_info[141].mFileTimeLo = 0U;
  c13_info[141].mFileTimeHi = 0U;
  c13_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c13_info[142].name = "rdivide";
  c13_info[142].dominantType = "double";
  c13_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c13_info[142].fileTimeLo = 1346510388U;
  c13_info[142].fileTimeHi = 0U;
  c13_info[142].mFileTimeLo = 0U;
  c13_info[142].mFileTimeHi = 0U;
  c13_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c13_info[143].name = "eml_scalexp_compatible";
  c13_info[143].dominantType = "double";
  c13_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c13_info[143].fileTimeLo = 1286818796U;
  c13_info[143].fileTimeHi = 0U;
  c13_info[143].mFileTimeLo = 0U;
  c13_info[143].mFileTimeHi = 0U;
  c13_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c13_info[144].name = "eml_div";
  c13_info[144].dominantType = "double";
  c13_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c13_info[144].fileTimeLo = 1313347810U;
  c13_info[144].fileTimeHi = 0U;
  c13_info[144].mFileTimeLo = 0U;
  c13_info[144].mFileTimeHi = 0U;
}

static void c13_eye(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c13_I[16])
{
  int32_T c13_i510;
  int32_T c13_i;
  int32_T c13_b_i;
  for (c13_i510 = 0; c13_i510 < 16; c13_i510++) {
    c13_I[c13_i510] = 0.0;
  }

  for (c13_i = 1; c13_i < 5; c13_i++) {
    c13_b_i = c13_i;
    c13_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c13_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_b_i), 1, 4, 2, 0) - 1)
            << 2)) - 1] = 1.0;
  }
}

static void c13_eml_scalar_eg(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c13_mrdivide(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_A[16], real_T c13_B[16], real_T c13_y[16])
{
  int32_T c13_i511;
  int32_T c13_i512;
  int32_T c13_i513;
  int32_T c13_i514;
  real_T c13_b_A[16];
  int32_T c13_i515;
  int32_T c13_i516;
  int32_T c13_i517;
  int32_T c13_i518;
  real_T c13_b_B[16];
  int32_T c13_info;
  int32_T c13_ipiv[4];
  int32_T c13_b_info;
  int32_T c13_c_info;
  int32_T c13_d_info;
  int32_T c13_i;
  int32_T c13_b_i;
  int32_T c13_ip;
  int32_T c13_j;
  int32_T c13_b_j;
  real_T c13_temp;
  int32_T c13_i519;
  real_T c13_c_A[16];
  int32_T c13_i520;
  real_T c13_d_A[16];
  int32_T c13_i521;
  int32_T c13_i522;
  int32_T c13_i523;
  int32_T c13_i524;
  c13_i511 = 0;
  for (c13_i512 = 0; c13_i512 < 4; c13_i512++) {
    c13_i513 = 0;
    for (c13_i514 = 0; c13_i514 < 4; c13_i514++) {
      c13_b_A[c13_i514 + c13_i511] = c13_B[c13_i513 + c13_i512];
      c13_i513 += 4;
    }

    c13_i511 += 4;
  }

  c13_i515 = 0;
  for (c13_i516 = 0; c13_i516 < 4; c13_i516++) {
    c13_i517 = 0;
    for (c13_i518 = 0; c13_i518 < 4; c13_i518++) {
      c13_b_B[c13_i518 + c13_i515] = c13_A[c13_i517 + c13_i516];
      c13_i517 += 4;
    }

    c13_i515 += 4;
  }

  c13_b_eml_matlab_zgetrf(chartInstance, c13_b_A, c13_ipiv, &c13_info);
  c13_b_info = c13_info;
  c13_c_info = c13_b_info;
  c13_d_info = c13_c_info;
  if (c13_d_info > 0) {
    c13_eml_warning(chartInstance);
  }

  c13_eml_scalar_eg(chartInstance);
  for (c13_i = 1; c13_i < 5; c13_i++) {
    c13_b_i = c13_i;
    if (c13_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_b_i), 1, 4, 1, 0) - 1] != c13_b_i) {
      c13_ip = c13_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c13_b_i), 1, 4, 1, 0) - 1];
      for (c13_j = 1; c13_j < 5; c13_j++) {
        c13_b_j = c13_j;
        c13_temp = c13_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c13_b_i), 1, 4, 1, 0) +
                            ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c13_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c13_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c13_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK
                   ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_b_j), 1, 4,
                    2, 0) - 1) << 2)) - 1] = c13_b_B
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c13_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_b_j), 1, 4, 2, 0) - 1)
             << 2)) - 1];
        c13_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c13_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                    "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_b_j), 1, 4,
                    2, 0) - 1) << 2)) - 1] = c13_temp;
      }
    }
  }

  for (c13_i519 = 0; c13_i519 < 16; c13_i519++) {
    c13_c_A[c13_i519] = c13_b_A[c13_i519];
  }

  c13_e_eml_xtrsm(chartInstance, c13_c_A, c13_b_B);
  for (c13_i520 = 0; c13_i520 < 16; c13_i520++) {
    c13_d_A[c13_i520] = c13_b_A[c13_i520];
  }

  c13_f_eml_xtrsm(chartInstance, c13_d_A, c13_b_B);
  c13_i521 = 0;
  for (c13_i522 = 0; c13_i522 < 4; c13_i522++) {
    c13_i523 = 0;
    for (c13_i524 = 0; c13_i524 < 4; c13_i524++) {
      c13_y[c13_i524 + c13_i521] = c13_b_B[c13_i523 + c13_i522];
      c13_i523 += 4;
    }

    c13_i521 += 4;
  }
}

static void c13_realmin(SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c13_eps(SFc13_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c13_eml_matlab_zgetrf(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_b_A[16], int32_T c13_ipiv[4],
  int32_T *c13_info)
{
  int32_T c13_i525;
  for (c13_i525 = 0; c13_i525 < 16; c13_i525++) {
    c13_b_A[c13_i525] = c13_A[c13_i525];
  }

  c13_b_eml_matlab_zgetrf(chartInstance, c13_b_A, c13_ipiv, c13_info);
}

static void c13_check_forloop_overflow_error
  (SFc13_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c13_overflow)
{
  int32_T c13_i526;
  static char_T c13_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c13_u[34];
  const mxArray *c13_y = NULL;
  int32_T c13_i527;
  static char_T c13_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c13_b_u[23];
  const mxArray *c13_b_y = NULL;
  if (!c13_overflow) {
  } else {
    for (c13_i526 = 0; c13_i526 < 34; c13_i526++) {
      c13_u[c13_i526] = c13_cv0[c13_i526];
    }

    c13_y = NULL;
    sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c13_i527 = 0; c13_i527 < 23; c13_i527++) {
      c13_b_u[c13_i527] = c13_cv1[c13_i527];
    }

    c13_b_y = NULL;
    sf_mex_assign(&c13_b_y, sf_mex_create("y", c13_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c13_y, 14, c13_b_y));
  }
}

static void c13_eml_xger(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c13_m, int32_T c13_n, real_T c13_alpha1, int32_T c13_ix0, int32_T
  c13_iy0, real_T c13_A[16], int32_T c13_ia0, real_T c13_b_A[16])
{
  int32_T c13_i528;
  for (c13_i528 = 0; c13_i528 < 16; c13_i528++) {
    c13_b_A[c13_i528] = c13_A[c13_i528];
  }

  c13_b_eml_xger(chartInstance, c13_m, c13_n, c13_alpha1, c13_ix0, c13_iy0,
                 c13_b_A, c13_ia0);
}

static void c13_eml_warning(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c13_i529;
  static char_T c13_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c13_u[27];
  const mxArray *c13_y = NULL;
  for (c13_i529 = 0; c13_i529 < 27; c13_i529++) {
    c13_u[c13_i529] = c13_varargin_1[c13_i529];
  }

  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", c13_u, 10, 0U, 1U, 0U, 2, 1, 27),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c13_y));
}

static void c13_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[16], real_T c13_b_B[16])
{
  int32_T c13_i530;
  int32_T c13_i531;
  real_T c13_b_A[16];
  for (c13_i530 = 0; c13_i530 < 16; c13_i530++) {
    c13_b_B[c13_i530] = c13_B[c13_i530];
  }

  for (c13_i531 = 0; c13_i531 < 16; c13_i531++) {
    c13_b_A[c13_i531] = c13_A[c13_i531];
  }

  c13_e_eml_xtrsm(chartInstance, c13_b_A, c13_b_B);
}

static void c13_below_threshold(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c13_b_eml_scalar_eg(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c13_b_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[16], real_T c13_b_B[16])
{
  int32_T c13_i532;
  int32_T c13_i533;
  real_T c13_b_A[16];
  for (c13_i532 = 0; c13_i532 < 16; c13_i532++) {
    c13_b_B[c13_i532] = c13_B[c13_i532];
  }

  for (c13_i533 = 0; c13_i533 < 16; c13_i533++) {
    c13_b_A[c13_i533] = c13_A[c13_i533];
  }

  c13_f_eml_xtrsm(chartInstance, c13_b_A, c13_b_B);
}

static void c13_mldivide(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_A[16], real_T c13_B[4], real_T c13_Y[4])
{
  int32_T c13_i534;
  real_T c13_b_A[16];
  int32_T c13_info;
  int32_T c13_ipiv[4];
  int32_T c13_b_info;
  int32_T c13_c_info;
  int32_T c13_d_info;
  int32_T c13_i535;
  int32_T c13_i;
  int32_T c13_b_i;
  int32_T c13_ip;
  real_T c13_temp;
  int32_T c13_i536;
  real_T c13_c_A[16];
  int32_T c13_i537;
  real_T c13_d_A[16];
  for (c13_i534 = 0; c13_i534 < 16; c13_i534++) {
    c13_b_A[c13_i534] = c13_A[c13_i534];
  }

  c13_b_eml_matlab_zgetrf(chartInstance, c13_b_A, c13_ipiv, &c13_info);
  c13_b_info = c13_info;
  c13_c_info = c13_b_info;
  c13_d_info = c13_c_info;
  if (c13_d_info > 0) {
    c13_eml_warning(chartInstance);
  }

  for (c13_i535 = 0; c13_i535 < 4; c13_i535++) {
    c13_Y[c13_i535] = c13_B[c13_i535];
  }

  for (c13_i = 1; c13_i < 5; c13_i++) {
    c13_b_i = c13_i;
    if (c13_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_b_i), 1, 4, 1, 0) - 1] != c13_b_i) {
      c13_ip = c13_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c13_b_i), 1, 4, 1, 0) - 1];
      c13_temp = c13_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c13_b_i), 1, 4, 1, 0) - 1];
      c13_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c13_b_i), 1, 4, 1, 0) - 1] = c13_Y[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_ip), 1, 4, 1, 0) - 1];
      c13_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c13_ip), 1, 4, 1, 0) - 1] = c13_temp;
    }
  }

  for (c13_i536 = 0; c13_i536 < 16; c13_i536++) {
    c13_c_A[c13_i536] = c13_b_A[c13_i536];
  }

  c13_g_eml_xtrsm(chartInstance, c13_c_A, c13_Y);
  for (c13_i537 = 0; c13_i537 < 16; c13_i537++) {
    c13_d_A[c13_i537] = c13_b_A[c13_i537];
  }

  c13_h_eml_xtrsm(chartInstance, c13_d_A, c13_Y);
}

static void c13_c_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[4], real_T c13_b_B[4])
{
  int32_T c13_i538;
  int32_T c13_i539;
  real_T c13_b_A[16];
  for (c13_i538 = 0; c13_i538 < 4; c13_i538++) {
    c13_b_B[c13_i538] = c13_B[c13_i538];
  }

  for (c13_i539 = 0; c13_i539 < 16; c13_i539++) {
    c13_b_A[c13_i539] = c13_A[c13_i539];
  }

  c13_g_eml_xtrsm(chartInstance, c13_b_A, c13_b_B);
}

static void c13_b_below_threshold(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c13_c_eml_scalar_eg(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c13_d_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[4], real_T c13_b_B[4])
{
  int32_T c13_i540;
  int32_T c13_i541;
  real_T c13_b_A[16];
  for (c13_i540 = 0; c13_i540 < 4; c13_i540++) {
    c13_b_B[c13_i540] = c13_B[c13_i540];
  }

  for (c13_i541 = 0; c13_i541 < 16; c13_i541++) {
    c13_b_A[c13_i541] = c13_A[c13_i541];
  }

  c13_h_eml_xtrsm(chartInstance, c13_b_A, c13_b_B);
}

static real_T c13_norm(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_x)
{
  real_T c13_b_x;
  real_T c13_c_x;
  c13_b_x = c13_x;
  c13_c_x = c13_b_x;
  return muDoubleScalarAbs(c13_c_x);
}

static real_T c13_b_norm(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_x[6])
{
  real_T c13_y;
  real_T c13_scale;
  int32_T c13_k;
  int32_T c13_b_k;
  real_T c13_b_x;
  real_T c13_c_x;
  real_T c13_absxk;
  real_T c13_t;
  c13_y = 0.0;
  c13_realmin(chartInstance);
  c13_scale = 2.2250738585072014E-308;
  for (c13_k = 1; c13_k < 7; c13_k++) {
    c13_b_k = c13_k;
    c13_b_x = c13_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c13_b_k), 1, 6, 1, 0) - 1];
    c13_c_x = c13_b_x;
    c13_absxk = muDoubleScalarAbs(c13_c_x);
    if (c13_absxk > c13_scale) {
      c13_t = c13_scale / c13_absxk;
      c13_y = 1.0 + c13_y * c13_t * c13_t;
      c13_scale = c13_absxk;
    } else {
      c13_t = c13_absxk / c13_scale;
      c13_y += c13_t * c13_t;
    }
  }

  return c13_scale * muDoubleScalarSqrt(c13_y);
}

static real_T c13_c_norm(SFc13_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c13_x[2])
{
  real_T c13_y;
  real_T c13_scale;
  int32_T c13_k;
  int32_T c13_b_k;
  real_T c13_b_x;
  real_T c13_c_x;
  real_T c13_absxk;
  real_T c13_t;
  c13_y = 0.0;
  c13_realmin(chartInstance);
  c13_scale = 2.2250738585072014E-308;
  for (c13_k = 1; c13_k < 3; c13_k++) {
    c13_b_k = c13_k;
    c13_b_x = c13_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c13_b_k), 1, 2, 1, 0) - 1];
    c13_c_x = c13_b_x;
    c13_absxk = muDoubleScalarAbs(c13_c_x);
    if (c13_absxk > c13_scale) {
      c13_t = c13_scale / c13_absxk;
      c13_y = 1.0 + c13_y * c13_t * c13_t;
      c13_scale = c13_absxk;
    } else {
      c13_t = c13_absxk / c13_scale;
      c13_y += c13_t * c13_t;
    }
  }

  return c13_scale * muDoubleScalarSqrt(c13_y);
}

static const mxArray *c13_p_sf_marshallOut(void *chartInstanceVoid, void
  *c13_inData)
{
  const mxArray *c13_mxArrayOutData = NULL;
  int32_T c13_u;
  const mxArray *c13_y = NULL;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_mxArrayOutData = NULL;
  c13_u = *(int32_T *)c13_inData;
  c13_y = NULL;
  sf_mex_assign(&c13_y, sf_mex_create("y", &c13_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c13_mxArrayOutData, c13_y, FALSE);
  return c13_mxArrayOutData;
}

static int32_T c13_nb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId)
{
  int32_T c13_y;
  int32_T c13_i542;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), &c13_i542, 1, 6, 0U, 0, 0U, 0);
  c13_y = c13_i542;
  sf_mex_destroy(&c13_u);
  return c13_y;
}

static void c13_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c13_mxArrayInData, const char_T *c13_varName, void *c13_outData)
{
  const mxArray *c13_b_sfEvent;
  const char_T *c13_identifier;
  emlrtMsgIdentifier c13_thisId;
  int32_T c13_y;
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c13_b_sfEvent = sf_mex_dup(c13_mxArrayInData);
  c13_identifier = c13_varName;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_y = c13_nb_emlrt_marshallIn(chartInstance, sf_mex_dup(c13_b_sfEvent),
    &c13_thisId);
  sf_mex_destroy(&c13_b_sfEvent);
  *(int32_T *)c13_outData = c13_y;
  sf_mex_destroy(&c13_mxArrayInData);
}

static uint8_T c13_ob_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c13_b_is_active_c13_torqueBalancing2012b, const
  char_T *c13_identifier)
{
  uint8_T c13_y;
  emlrtMsgIdentifier c13_thisId;
  c13_thisId.fIdentifier = c13_identifier;
  c13_thisId.fParent = NULL;
  c13_y = c13_pb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c13_b_is_active_c13_torqueBalancing2012b), &c13_thisId);
  sf_mex_destroy(&c13_b_is_active_c13_torqueBalancing2012b);
  return c13_y;
}

static uint8_T c13_pb_emlrt_marshallIn(SFc13_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c13_u, const emlrtMsgIdentifier *c13_parentId)
{
  uint8_T c13_y;
  uint8_T c13_u0;
  sf_mex_import(c13_parentId, sf_mex_dup(c13_u), &c13_u0, 1, 3, 0U, 0, 0U, 0);
  c13_y = c13_u0;
  sf_mex_destroy(&c13_u);
  return c13_y;
}

static void c13_b_eml_matlab_zgetrf(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], int32_T c13_ipiv[4], int32_T *c13_info)
{
  int32_T c13_i543;
  int32_T c13_j;
  int32_T c13_b_j;
  int32_T c13_a;
  int32_T c13_jm1;
  int32_T c13_b;
  int32_T c13_mmj;
  int32_T c13_b_a;
  int32_T c13_c;
  int32_T c13_b_b;
  int32_T c13_jj;
  int32_T c13_c_a;
  int32_T c13_jp1j;
  int32_T c13_d_a;
  int32_T c13_b_c;
  int32_T c13_n;
  int32_T c13_ix0;
  int32_T c13_b_n;
  int32_T c13_b_ix0;
  int32_T c13_c_n;
  int32_T c13_c_ix0;
  int32_T c13_idxmax;
  int32_T c13_ix;
  real_T c13_x;
  real_T c13_b_x;
  real_T c13_c_x;
  real_T c13_y;
  real_T c13_d_x;
  real_T c13_e_x;
  real_T c13_b_y;
  real_T c13_smax;
  int32_T c13_d_n;
  int32_T c13_c_b;
  int32_T c13_d_b;
  boolean_T c13_overflow;
  int32_T c13_k;
  int32_T c13_b_k;
  int32_T c13_e_a;
  real_T c13_f_x;
  real_T c13_g_x;
  real_T c13_h_x;
  real_T c13_c_y;
  real_T c13_i_x;
  real_T c13_j_x;
  real_T c13_d_y;
  real_T c13_s;
  int32_T c13_f_a;
  int32_T c13_jpiv_offset;
  int32_T c13_g_a;
  int32_T c13_e_b;
  int32_T c13_jpiv;
  int32_T c13_h_a;
  int32_T c13_f_b;
  int32_T c13_c_c;
  int32_T c13_g_b;
  int32_T c13_jrow;
  int32_T c13_i_a;
  int32_T c13_h_b;
  int32_T c13_jprow;
  int32_T c13_d_ix0;
  int32_T c13_iy0;
  int32_T c13_e_ix0;
  int32_T c13_b_iy0;
  int32_T c13_f_ix0;
  int32_T c13_c_iy0;
  int32_T c13_b_ix;
  int32_T c13_iy;
  int32_T c13_c_k;
  real_T c13_temp;
  int32_T c13_j_a;
  int32_T c13_k_a;
  int32_T c13_b_jp1j;
  int32_T c13_l_a;
  int32_T c13_d_c;
  int32_T c13_m_a;
  int32_T c13_i_b;
  int32_T c13_i544;
  int32_T c13_n_a;
  int32_T c13_j_b;
  int32_T c13_o_a;
  int32_T c13_k_b;
  boolean_T c13_b_overflow;
  int32_T c13_i;
  int32_T c13_b_i;
  real_T c13_k_x;
  real_T c13_e_y;
  real_T c13_z;
  int32_T c13_l_b;
  int32_T c13_e_c;
  int32_T c13_p_a;
  int32_T c13_f_c;
  int32_T c13_q_a;
  int32_T c13_g_c;
  int32_T c13_m;
  int32_T c13_e_n;
  int32_T c13_g_ix0;
  int32_T c13_d_iy0;
  int32_T c13_ia0;
  real_T c13_d3;
  c13_realmin(chartInstance);
  c13_eps(chartInstance);
  for (c13_i543 = 0; c13_i543 < 4; c13_i543++) {
    c13_ipiv[c13_i543] = 1 + c13_i543;
  }

  *c13_info = 0;
  for (c13_j = 1; c13_j < 4; c13_j++) {
    c13_b_j = c13_j;
    c13_a = c13_b_j - 1;
    c13_jm1 = c13_a;
    c13_b = c13_b_j;
    c13_mmj = 4 - c13_b;
    c13_b_a = c13_jm1;
    c13_c = c13_b_a * 5;
    c13_b_b = c13_c + 1;
    c13_jj = c13_b_b;
    c13_c_a = c13_jj + 1;
    c13_jp1j = c13_c_a;
    c13_d_a = c13_mmj;
    c13_b_c = c13_d_a;
    c13_n = c13_b_c + 1;
    c13_ix0 = c13_jj;
    c13_b_n = c13_n;
    c13_b_ix0 = c13_ix0;
    c13_c_n = c13_b_n;
    c13_c_ix0 = c13_b_ix0;
    if (c13_c_n < 1) {
      c13_idxmax = 0;
    } else {
      c13_idxmax = 1;
      if (c13_c_n > 1) {
        c13_ix = c13_c_ix0;
        c13_x = c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c13_ix), 1, 16, 1, 0) - 1];
        c13_b_x = c13_x;
        c13_c_x = c13_b_x;
        c13_y = muDoubleScalarAbs(c13_c_x);
        c13_d_x = 0.0;
        c13_e_x = c13_d_x;
        c13_b_y = muDoubleScalarAbs(c13_e_x);
        c13_smax = c13_y + c13_b_y;
        c13_d_n = c13_c_n;
        c13_c_b = c13_d_n;
        c13_d_b = c13_c_b;
        if (2 > c13_d_b) {
          c13_overflow = FALSE;
        } else {
          c13_overflow = (c13_d_b > 2147483646);
        }

        if (c13_overflow) {
          c13_check_forloop_overflow_error(chartInstance, c13_overflow);
        }

        for (c13_k = 2; c13_k <= c13_d_n; c13_k++) {
          c13_b_k = c13_k;
          c13_e_a = c13_ix + 1;
          c13_ix = c13_e_a;
          c13_f_x = c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c13_ix), 1, 16, 1, 0) - 1];
          c13_g_x = c13_f_x;
          c13_h_x = c13_g_x;
          c13_c_y = muDoubleScalarAbs(c13_h_x);
          c13_i_x = 0.0;
          c13_j_x = c13_i_x;
          c13_d_y = muDoubleScalarAbs(c13_j_x);
          c13_s = c13_c_y + c13_d_y;
          if (c13_s > c13_smax) {
            c13_idxmax = c13_b_k;
            c13_smax = c13_s;
          }
        }
      }
    }

    c13_f_a = c13_idxmax - 1;
    c13_jpiv_offset = c13_f_a;
    c13_g_a = c13_jj;
    c13_e_b = c13_jpiv_offset;
    c13_jpiv = c13_g_a + c13_e_b;
    if (c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if (c13_jpiv_offset != 0) {
        c13_h_a = c13_b_j;
        c13_f_b = c13_jpiv_offset;
        c13_c_c = c13_h_a + c13_f_b;
        c13_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_b_j), 1, 4, 1, 0) - 1] = c13_c_c;
        c13_g_b = c13_jm1 + 1;
        c13_jrow = c13_g_b;
        c13_i_a = c13_jrow;
        c13_h_b = c13_jpiv_offset;
        c13_jprow = c13_i_a + c13_h_b;
        c13_d_ix0 = c13_jrow;
        c13_iy0 = c13_jprow;
        c13_e_ix0 = c13_d_ix0;
        c13_b_iy0 = c13_iy0;
        c13_f_ix0 = c13_e_ix0;
        c13_c_iy0 = c13_b_iy0;
        c13_b_ix = c13_f_ix0;
        c13_iy = c13_c_iy0;
        for (c13_c_k = 1; c13_c_k < 5; c13_c_k++) {
          c13_temp = c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c13_b_ix), 1, 16, 1, 0) - 1];
          c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_b_ix), 1, 16, 1, 0) - 1] =
            c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_iy), 1, 16, 1, 0) - 1];
          c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_iy), 1, 16, 1, 0) - 1] = c13_temp;
          c13_j_a = c13_b_ix + 4;
          c13_b_ix = c13_j_a;
          c13_k_a = c13_iy + 4;
          c13_iy = c13_k_a;
        }
      }

      c13_b_jp1j = c13_jp1j;
      c13_l_a = c13_mmj;
      c13_d_c = c13_l_a;
      c13_m_a = c13_jp1j;
      c13_i_b = c13_d_c - 1;
      c13_i544 = c13_m_a + c13_i_b;
      c13_n_a = c13_b_jp1j;
      c13_j_b = c13_i544;
      c13_o_a = c13_n_a;
      c13_k_b = c13_j_b;
      if (c13_o_a > c13_k_b) {
        c13_b_overflow = FALSE;
      } else {
        c13_b_overflow = (c13_k_b > 2147483646);
      }

      if (c13_b_overflow) {
        c13_check_forloop_overflow_error(chartInstance, c13_b_overflow);
      }

      for (c13_i = c13_b_jp1j; c13_i <= c13_i544; c13_i++) {
        c13_b_i = c13_i;
        c13_k_x = c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c13_b_i), 1, 16, 1, 0) - 1];
        c13_e_y = c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c13_jj), 1, 16, 1, 0) - 1];
        c13_z = c13_k_x / c13_e_y;
        c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_b_i), 1, 16, 1, 0) - 1] = c13_z;
      }
    } else {
      *c13_info = c13_b_j;
    }

    c13_l_b = c13_b_j;
    c13_e_c = 4 - c13_l_b;
    c13_p_a = c13_jj;
    c13_f_c = c13_p_a;
    c13_q_a = c13_jj;
    c13_g_c = c13_q_a;
    c13_m = c13_mmj;
    c13_e_n = c13_e_c;
    c13_g_ix0 = c13_jp1j;
    c13_d_iy0 = c13_f_c + 4;
    c13_ia0 = c13_g_c + 5;
    c13_d3 = -1.0;
    c13_b_eml_xger(chartInstance, c13_m, c13_e_n, c13_d3, c13_g_ix0, c13_d_iy0,
                   c13_A, c13_ia0);
  }

  if (*c13_info == 0) {
    if (!(c13_A[15] != 0.0)) {
      *c13_info = 4;
    }
  }
}

static void c13_b_eml_xger(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c13_m, int32_T c13_n, real_T c13_alpha1, int32_T
  c13_ix0, int32_T c13_iy0, real_T c13_A[16], int32_T c13_ia0)
{
  int32_T c13_b_m;
  int32_T c13_b_n;
  real_T c13_b_alpha1;
  int32_T c13_b_ix0;
  int32_T c13_b_iy0;
  int32_T c13_b_ia0;
  int32_T c13_c_m;
  int32_T c13_c_n;
  real_T c13_c_alpha1;
  int32_T c13_c_ix0;
  int32_T c13_c_iy0;
  int32_T c13_c_ia0;
  int32_T c13_d_m;
  int32_T c13_d_n;
  real_T c13_d_alpha1;
  int32_T c13_d_ix0;
  int32_T c13_d_iy0;
  int32_T c13_d_ia0;
  int32_T c13_ixstart;
  int32_T c13_a;
  int32_T c13_jA;
  int32_T c13_jy;
  int32_T c13_e_n;
  int32_T c13_b;
  int32_T c13_b_b;
  boolean_T c13_overflow;
  int32_T c13_j;
  real_T c13_yjy;
  real_T c13_temp;
  int32_T c13_ix;
  int32_T c13_c_b;
  int32_T c13_i545;
  int32_T c13_b_a;
  int32_T c13_d_b;
  int32_T c13_i546;
  int32_T c13_c_a;
  int32_T c13_e_b;
  int32_T c13_d_a;
  int32_T c13_f_b;
  boolean_T c13_b_overflow;
  int32_T c13_ijA;
  int32_T c13_b_ijA;
  int32_T c13_e_a;
  int32_T c13_f_a;
  int32_T c13_g_a;
  c13_b_m = c13_m;
  c13_b_n = c13_n;
  c13_b_alpha1 = c13_alpha1;
  c13_b_ix0 = c13_ix0;
  c13_b_iy0 = c13_iy0;
  c13_b_ia0 = c13_ia0;
  c13_c_m = c13_b_m;
  c13_c_n = c13_b_n;
  c13_c_alpha1 = c13_b_alpha1;
  c13_c_ix0 = c13_b_ix0;
  c13_c_iy0 = c13_b_iy0;
  c13_c_ia0 = c13_b_ia0;
  c13_d_m = c13_c_m;
  c13_d_n = c13_c_n;
  c13_d_alpha1 = c13_c_alpha1;
  c13_d_ix0 = c13_c_ix0;
  c13_d_iy0 = c13_c_iy0;
  c13_d_ia0 = c13_c_ia0;
  if (c13_d_alpha1 == 0.0) {
  } else {
    c13_ixstart = c13_d_ix0;
    c13_a = c13_d_ia0 - 1;
    c13_jA = c13_a;
    c13_jy = c13_d_iy0;
    c13_e_n = c13_d_n;
    c13_b = c13_e_n;
    c13_b_b = c13_b;
    if (1 > c13_b_b) {
      c13_overflow = FALSE;
    } else {
      c13_overflow = (c13_b_b > 2147483646);
    }

    if (c13_overflow) {
      c13_check_forloop_overflow_error(chartInstance, c13_overflow);
    }

    for (c13_j = 1; c13_j <= c13_e_n; c13_j++) {
      c13_yjy = c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c13_jy), 1, 16, 1, 0) - 1];
      if (c13_yjy != 0.0) {
        c13_temp = c13_yjy * c13_d_alpha1;
        c13_ix = c13_ixstart;
        c13_c_b = c13_jA + 1;
        c13_i545 = c13_c_b;
        c13_b_a = c13_d_m;
        c13_d_b = c13_jA;
        c13_i546 = c13_b_a + c13_d_b;
        c13_c_a = c13_i545;
        c13_e_b = c13_i546;
        c13_d_a = c13_c_a;
        c13_f_b = c13_e_b;
        if (c13_d_a > c13_f_b) {
          c13_b_overflow = FALSE;
        } else {
          c13_b_overflow = (c13_f_b > 2147483646);
        }

        if (c13_b_overflow) {
          c13_check_forloop_overflow_error(chartInstance, c13_b_overflow);
        }

        for (c13_ijA = c13_i545; c13_ijA <= c13_i546; c13_ijA++) {
          c13_b_ijA = c13_ijA;
          c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_b_ijA), 1, 16, 1, 0) - 1] =
            c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_b_ijA), 1, 16, 1, 0) - 1] +
            c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_ix), 1, 16, 1, 0) - 1] * c13_temp;
          c13_e_a = c13_ix + 1;
          c13_ix = c13_e_a;
        }
      }

      c13_f_a = c13_jy + 4;
      c13_jy = c13_f_a;
      c13_g_a = c13_jA + 4;
      c13_jA = c13_g_a;
    }
  }
}

static void c13_e_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[16])
{
  int32_T c13_j;
  int32_T c13_b_j;
  int32_T c13_a;
  int32_T c13_c;
  int32_T c13_b;
  int32_T c13_b_c;
  int32_T c13_b_b;
  int32_T c13_jBcol;
  int32_T c13_k;
  int32_T c13_b_k;
  int32_T c13_b_a;
  int32_T c13_c_c;
  int32_T c13_c_b;
  int32_T c13_d_c;
  int32_T c13_d_b;
  int32_T c13_kAcol;
  int32_T c13_c_a;
  int32_T c13_e_b;
  int32_T c13_e_c;
  int32_T c13_d_a;
  int32_T c13_i547;
  boolean_T c13_overflow;
  int32_T c13_i;
  int32_T c13_b_i;
  int32_T c13_e_a;
  int32_T c13_f_b;
  int32_T c13_f_c;
  int32_T c13_f_a;
  int32_T c13_g_b;
  int32_T c13_g_c;
  int32_T c13_g_a;
  int32_T c13_h_b;
  int32_T c13_h_c;
  int32_T c13_h_a;
  int32_T c13_i_b;
  int32_T c13_i_c;
  c13_below_threshold(chartInstance);
  c13_b_eml_scalar_eg(chartInstance);
  for (c13_j = 1; c13_j < 5; c13_j++) {
    c13_b_j = c13_j;
    c13_a = c13_b_j;
    c13_c = c13_a;
    c13_b = c13_c - 1;
    c13_b_c = c13_b << 2;
    c13_b_b = c13_b_c;
    c13_jBcol = c13_b_b;
    for (c13_k = 1; c13_k < 5; c13_k++) {
      c13_b_k = c13_k;
      c13_b_a = c13_b_k;
      c13_c_c = c13_b_a;
      c13_c_b = c13_c_c - 1;
      c13_d_c = c13_c_b << 2;
      c13_d_b = c13_d_c;
      c13_kAcol = c13_d_b;
      c13_c_a = c13_b_k;
      c13_e_b = c13_jBcol;
      c13_e_c = c13_c_a + c13_e_b;
      if (c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c13_d_a = c13_b_k;
        c13_i547 = c13_d_a;
        c13_overflow = FALSE;
        if (c13_overflow) {
          c13_check_forloop_overflow_error(chartInstance, c13_overflow);
        }

        for (c13_i = c13_i547 + 1; c13_i < 5; c13_i++) {
          c13_b_i = c13_i;
          c13_e_a = c13_b_i;
          c13_f_b = c13_jBcol;
          c13_f_c = c13_e_a + c13_f_b;
          c13_f_a = c13_b_i;
          c13_g_b = c13_jBcol;
          c13_g_c = c13_f_a + c13_g_b;
          c13_g_a = c13_b_k;
          c13_h_b = c13_jBcol;
          c13_h_c = c13_g_a + c13_h_b;
          c13_h_a = c13_b_i;
          c13_i_b = c13_kAcol;
          c13_i_c = c13_h_a + c13_i_b;
          c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_f_c), 1, 16, 1, 0) - 1] =
            c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_g_c), 1, 16, 1, 0) - 1] -
            c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_h_c), 1, 16, 1, 0) - 1] *
            c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_i_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void c13_f_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[16])
{
  int32_T c13_j;
  int32_T c13_b_j;
  int32_T c13_a;
  int32_T c13_c;
  int32_T c13_b;
  int32_T c13_b_c;
  int32_T c13_b_b;
  int32_T c13_jBcol;
  int32_T c13_k;
  int32_T c13_b_k;
  int32_T c13_b_a;
  int32_T c13_c_c;
  int32_T c13_c_b;
  int32_T c13_d_c;
  int32_T c13_d_b;
  int32_T c13_kAcol;
  int32_T c13_c_a;
  int32_T c13_e_b;
  int32_T c13_e_c;
  int32_T c13_d_a;
  int32_T c13_f_b;
  int32_T c13_f_c;
  int32_T c13_e_a;
  int32_T c13_g_b;
  int32_T c13_g_c;
  int32_T c13_f_a;
  int32_T c13_h_b;
  int32_T c13_h_c;
  real_T c13_x;
  real_T c13_y;
  real_T c13_z;
  int32_T c13_g_a;
  int32_T c13_i548;
  int32_T c13_i_b;
  int32_T c13_j_b;
  boolean_T c13_overflow;
  int32_T c13_i;
  int32_T c13_b_i;
  int32_T c13_h_a;
  int32_T c13_k_b;
  int32_T c13_i_c;
  int32_T c13_i_a;
  int32_T c13_l_b;
  int32_T c13_j_c;
  int32_T c13_j_a;
  int32_T c13_m_b;
  int32_T c13_k_c;
  int32_T c13_k_a;
  int32_T c13_n_b;
  int32_T c13_l_c;
  c13_below_threshold(chartInstance);
  c13_b_eml_scalar_eg(chartInstance);
  for (c13_j = 1; c13_j < 5; c13_j++) {
    c13_b_j = c13_j;
    c13_a = c13_b_j;
    c13_c = c13_a;
    c13_b = c13_c - 1;
    c13_b_c = c13_b << 2;
    c13_b_b = c13_b_c;
    c13_jBcol = c13_b_b;
    for (c13_k = 4; c13_k > 0; c13_k--) {
      c13_b_k = c13_k;
      c13_b_a = c13_b_k;
      c13_c_c = c13_b_a;
      c13_c_b = c13_c_c - 1;
      c13_d_c = c13_c_b << 2;
      c13_d_b = c13_d_c;
      c13_kAcol = c13_d_b;
      c13_c_a = c13_b_k;
      c13_e_b = c13_jBcol;
      c13_e_c = c13_c_a + c13_e_b;
      if (c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c13_d_a = c13_b_k;
        c13_f_b = c13_jBcol;
        c13_f_c = c13_d_a + c13_f_b;
        c13_e_a = c13_b_k;
        c13_g_b = c13_jBcol;
        c13_g_c = c13_e_a + c13_g_b;
        c13_f_a = c13_b_k;
        c13_h_b = c13_kAcol;
        c13_h_c = c13_f_a + c13_h_b;
        c13_x = c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c13_g_c), 1, 16, 1, 0) - 1];
        c13_y = c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c13_h_c), 1, 16, 1, 0) - 1];
        c13_z = c13_x / c13_y;
        c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_f_c), 1, 16, 1, 0) - 1] = c13_z;
        c13_g_a = c13_b_k - 1;
        c13_i548 = c13_g_a;
        c13_i_b = c13_i548;
        c13_j_b = c13_i_b;
        if (1 > c13_j_b) {
          c13_overflow = FALSE;
        } else {
          c13_overflow = (c13_j_b > 2147483646);
        }

        if (c13_overflow) {
          c13_check_forloop_overflow_error(chartInstance, c13_overflow);
        }

        for (c13_i = 1; c13_i <= c13_i548; c13_i++) {
          c13_b_i = c13_i;
          c13_h_a = c13_b_i;
          c13_k_b = c13_jBcol;
          c13_i_c = c13_h_a + c13_k_b;
          c13_i_a = c13_b_i;
          c13_l_b = c13_jBcol;
          c13_j_c = c13_i_a + c13_l_b;
          c13_j_a = c13_b_k;
          c13_m_b = c13_jBcol;
          c13_k_c = c13_j_a + c13_m_b;
          c13_k_a = c13_b_i;
          c13_n_b = c13_kAcol;
          c13_l_c = c13_k_a + c13_n_b;
          c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_i_c), 1, 16, 1, 0) - 1] =
            c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_j_c), 1, 16, 1, 0) - 1] -
            c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_k_c), 1, 16, 1, 0) - 1] *
            c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c13_l_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void c13_g_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[4])
{
  int32_T c13_k;
  int32_T c13_b_k;
  int32_T c13_a;
  int32_T c13_c;
  int32_T c13_b;
  int32_T c13_b_c;
  int32_T c13_b_b;
  int32_T c13_kAcol;
  int32_T c13_b_a;
  int32_T c13_c_c;
  int32_T c13_c_a;
  int32_T c13_i549;
  boolean_T c13_overflow;
  int32_T c13_i;
  int32_T c13_b_i;
  int32_T c13_d_a;
  int32_T c13_d_c;
  int32_T c13_e_a;
  int32_T c13_e_c;
  int32_T c13_f_a;
  int32_T c13_f_c;
  int32_T c13_g_a;
  int32_T c13_c_b;
  int32_T c13_g_c;
  c13_b_below_threshold(chartInstance);
  c13_c_eml_scalar_eg(chartInstance);
  for (c13_k = 1; c13_k < 5; c13_k++) {
    c13_b_k = c13_k;
    c13_a = c13_b_k;
    c13_c = c13_a;
    c13_b = c13_c - 1;
    c13_b_c = c13_b << 2;
    c13_b_b = c13_b_c;
    c13_kAcol = c13_b_b;
    c13_b_a = c13_b_k;
    c13_c_c = c13_b_a;
    if (c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_c_c), 1, 4, 1, 0) - 1] != 0.0) {
      c13_c_a = c13_b_k;
      c13_i549 = c13_c_a;
      c13_overflow = FALSE;
      if (c13_overflow) {
        c13_check_forloop_overflow_error(chartInstance, c13_overflow);
      }

      for (c13_i = c13_i549 + 1; c13_i < 5; c13_i++) {
        c13_b_i = c13_i;
        c13_d_a = c13_b_i;
        c13_d_c = c13_d_a;
        c13_e_a = c13_b_i;
        c13_e_c = c13_e_a;
        c13_f_a = c13_b_k;
        c13_f_c = c13_f_a;
        c13_g_a = c13_b_i;
        c13_c_b = c13_kAcol;
        c13_g_c = c13_g_a + c13_c_b;
        c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_d_c), 1, 4, 1, 0) - 1] = c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_e_c), 1, 4, 1, 0) - 1]
          - c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_f_c), 1, 4, 1, 0) - 1] * c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_g_c), 1, 16, 1, 0) -
          1];
      }
    }
  }
}

static void c13_h_eml_xtrsm(SFc13_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c13_A[16], real_T c13_B[4])
{
  int32_T c13_k;
  int32_T c13_b_k;
  int32_T c13_a;
  int32_T c13_c;
  int32_T c13_b;
  int32_T c13_b_c;
  int32_T c13_b_b;
  int32_T c13_kAcol;
  int32_T c13_b_a;
  int32_T c13_c_c;
  int32_T c13_c_a;
  int32_T c13_d_c;
  int32_T c13_d_a;
  int32_T c13_e_c;
  int32_T c13_e_a;
  int32_T c13_c_b;
  int32_T c13_f_c;
  real_T c13_x;
  real_T c13_y;
  real_T c13_z;
  int32_T c13_f_a;
  int32_T c13_i550;
  int32_T c13_d_b;
  int32_T c13_e_b;
  boolean_T c13_overflow;
  int32_T c13_i;
  int32_T c13_b_i;
  int32_T c13_g_a;
  int32_T c13_g_c;
  int32_T c13_h_a;
  int32_T c13_h_c;
  int32_T c13_i_a;
  int32_T c13_i_c;
  int32_T c13_j_a;
  int32_T c13_f_b;
  int32_T c13_j_c;
  c13_b_below_threshold(chartInstance);
  c13_c_eml_scalar_eg(chartInstance);
  for (c13_k = 4; c13_k > 0; c13_k--) {
    c13_b_k = c13_k;
    c13_a = c13_b_k;
    c13_c = c13_a;
    c13_b = c13_c - 1;
    c13_b_c = c13_b << 2;
    c13_b_b = c13_b_c;
    c13_kAcol = c13_b_b;
    c13_b_a = c13_b_k;
    c13_c_c = c13_b_a;
    if (c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_c_c), 1, 4, 1, 0) - 1] != 0.0) {
      c13_c_a = c13_b_k;
      c13_d_c = c13_c_a;
      c13_d_a = c13_b_k;
      c13_e_c = c13_d_a;
      c13_e_a = c13_b_k;
      c13_c_b = c13_kAcol;
      c13_f_c = c13_e_a + c13_c_b;
      c13_x = c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c13_e_c), 1, 4, 1, 0) - 1];
      c13_y = c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c13_f_c), 1, 16, 1, 0) - 1];
      c13_z = c13_x / c13_y;
      c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c13_d_c), 1, 4, 1, 0) - 1] = c13_z;
      c13_f_a = c13_b_k - 1;
      c13_i550 = c13_f_a;
      c13_d_b = c13_i550;
      c13_e_b = c13_d_b;
      if (1 > c13_e_b) {
        c13_overflow = FALSE;
      } else {
        c13_overflow = (c13_e_b > 2147483646);
      }

      if (c13_overflow) {
        c13_check_forloop_overflow_error(chartInstance, c13_overflow);
      }

      for (c13_i = 1; c13_i <= c13_i550; c13_i++) {
        c13_b_i = c13_i;
        c13_g_a = c13_b_i;
        c13_g_c = c13_g_a;
        c13_h_a = c13_b_i;
        c13_h_c = c13_h_a;
        c13_i_a = c13_b_k;
        c13_i_c = c13_i_a;
        c13_j_a = c13_b_i;
        c13_f_b = c13_kAcol;
        c13_j_c = c13_j_a + c13_f_b;
        c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_g_c), 1, 4, 1, 0) - 1] = c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_h_c), 1, 4, 1, 0) - 1]
          - c13_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c13_i_c), 1, 4, 1, 0) - 1] * c13_A[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c13_j_c), 1, 16, 1, 0) -
          1];
      }
    }
  }
}

static void init_dsm_address_info(SFc13_torqueBalancing2012bInstanceStruct
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

void sf_c13_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3828880859U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2943770507U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4071994872U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2061659648U);
}

mxArray *sf_c13_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("Wizc3InTzRfgxqurK4FrU");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,10,3,dataFields);

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
      pr[0] = (double)(3);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
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
      pr[0] = (double)(23);
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
      pr[0] = (double)(1);
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
      pr[0] = (double)(4);
      pr[1] = (double)(4);
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

    mxArray *mxData = mxCreateStructMatrix(1,9,3,dataFields);

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
      pr[1] = (double)(3);
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
      pr[0] = (double)(1);
      pr[1] = (double)(3);
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
      pr[0] = (double)(1);
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
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c13_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c13_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[52],T\"CoMDes\",},{M[1],M[43],T\"constraints\",},{M[1],M[68],T\"currentState\",},{M[1],M[66],T\"impedances\",},{M[1],M[81],T\"jointsSmoothingTime\",},{M[1],M[83],T\"kdCom\",},{M[1],M[82],T\"kpCom\",},{M[1],M[46],T\"qDes\",},{M[1],M[71],T\"w_H_b\",},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[284 289],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m\"}}}}",
    "100 S1x3'type','srcId','name','auxInfo'{{M[4],M[0],T\"tSwitch\",S'l','i','p'{{M1x2[306 313],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m\"}}},{M[4],M[0],T\"w_H_fixedLink\",S'l','i','p'{{M1x2[330 343],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachine.m\"}}},{M[8],M[0],T\"is_active_c13_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 13, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c13_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           13,
           1,
           1,
           21,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"wrench_rightFoot");
          _SFD_SET_DATA_PROPS(1,1,1,0,"wrench_leftFoot");
          _SFD_SET_DATA_PROPS(2,2,0,1,"w_H_b");
          _SFD_SET_DATA_PROPS(3,2,0,1,"CoMDes");
          _SFD_SET_DATA_PROPS(4,2,0,1,"qDes");
          _SFD_SET_DATA_PROPS(5,2,0,1,"constraints");
          _SFD_SET_DATA_PROPS(6,1,1,0,"CoM_0");
          _SFD_SET_DATA_PROPS(7,1,1,0,"q0");
          _SFD_SET_DATA_PROPS(8,1,1,0,"l_sole_CoM");
          _SFD_SET_DATA_PROPS(9,1,1,0,"r_sole_CoM");
          _SFD_SET_DATA_PROPS(10,1,1,0,"qj");
          _SFD_SET_DATA_PROPS(11,1,1,0,"t");
          _SFD_SET_DATA_PROPS(12,1,1,0,"l_sole_H_b");
          _SFD_SET_DATA_PROPS(13,1,1,0,"r_sole_H_b");
          _SFD_SET_DATA_PROPS(14,10,0,0,"sm");
          _SFD_SET_DATA_PROPS(15,2,0,1,"impedances");
          _SFD_SET_DATA_PROPS(16,2,0,1,"kpCom");
          _SFD_SET_DATA_PROPS(17,2,0,1,"kdCom");
          _SFD_SET_DATA_PROPS(18,10,0,0,"gain");
          _SFD_SET_DATA_PROPS(19,2,0,1,"currentState");
          _SFD_SET_DATA_PROPS(20,2,0,1,"jointsSmoothingTime");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,460);
        _SFD_CV_INIT_SCRIPT(0,1,36,0,0,0,2,0,18,8);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"stateMachine",0,-1,11878);
        _SFD_CV_INIT_SCRIPT_IF(0,0,350,413,-1,518);
        _SFD_CV_INIT_SCRIPT_IF(0,1,768,781,-1,1119);
        _SFD_CV_INIT_SCRIPT_IF(0,2,842,862,-1,1111);
        _SFD_CV_INIT_SCRIPT_IF(0,3,951,974,-1,1087);
        _SFD_CV_INIT_SCRIPT_IF(0,4,1160,1173,-1,2009);
        _SFD_CV_INIT_SCRIPT_IF(0,5,1848,1942,-1,2001);
        _SFD_CV_INIT_SCRIPT_IF(0,6,2043,2056,-1,2813);
        _SFD_CV_INIT_SCRIPT_IF(0,7,2642,2664,-1,2805);
        _SFD_CV_INIT_SCRIPT_IF(0,8,2734,2748,-1,2793);
        _SFD_CV_INIT_SCRIPT_IF(0,9,2845,2858,-1,3981);
        _SFD_CV_INIT_SCRIPT_IF(0,10,3497,3582,-1,3650);
        _SFD_CV_INIT_SCRIPT_IF(0,11,3671,3712,-1,3973);
        _SFD_CV_INIT_SCRIPT_IF(0,12,3776,3887,-1,3961);
        _SFD_CV_INIT_SCRIPT_IF(0,13,4022,4035,-1,4944);
        _SFD_CV_INIT_SCRIPT_IF(0,14,4756,4874,-1,4936);
        _SFD_CV_INIT_SCRIPT_IF(0,15,4983,4996,-1,5703);
        _SFD_CV_INIT_SCRIPT_IF(0,16,5582,5635,-1,5695);
        _SFD_CV_INIT_SCRIPT_IF(0,17,5751,5764,-1,6400);
        _SFD_CV_INIT_SCRIPT_IF(0,18,6124,6245,-1,6392);
        _SFD_CV_INIT_SCRIPT_IF(0,19,7049,7062,-1,7938);
        _SFD_CV_INIT_SCRIPT_IF(0,20,7776,7870,-1,7929);
        _SFD_CV_INIT_SCRIPT_IF(0,21,7978,7991,-1,8571);
        _SFD_CV_INIT_SCRIPT_IF(0,22,8398,8420,-1,8563);
        _SFD_CV_INIT_SCRIPT_IF(0,23,8491,8505,-1,8551);
        _SFD_CV_INIT_SCRIPT_IF(0,24,8604,8618,-1,9562);
        _SFD_CV_INIT_SCRIPT_IF(0,25,9075,9160,-1,9228);
        _SFD_CV_INIT_SCRIPT_IF(0,26,9249,9290,-1,9554);
        _SFD_CV_INIT_SCRIPT_IF(0,27,9354,9464,-1,9542);
        _SFD_CV_INIT_SCRIPT_IF(0,28,9603,9617,-1,10350);
        _SFD_CV_INIT_SCRIPT_IF(0,29,10161,10279,-1,10342);
        _SFD_CV_INIT_SCRIPT_IF(0,30,10389,10403,-1,10935);
        _SFD_CV_INIT_SCRIPT_IF(0,31,10811,10863,-1,10927);
        _SFD_CV_INIT_SCRIPT_IF(0,32,10983,10997,-1,11673);
        _SFD_CV_INIT_SCRIPT_IF(0,33,11264,11293,-1,11665);
        _SFD_CV_INIT_SCRIPT_IF(0,34,11360,11376,-1,11653);
        _SFD_CV_INIT_SCRIPT_IF(0,35,11486,11509,-1,11638);
        _SFD_CV_INIT_SCRIPT_FOR(0,0,3446,3485,3662);
        _SFD_CV_INIT_SCRIPT_FOR(0,1,9024,9063,9240);

        {
          static int condStart[] = { 353, 371, 391 };

          static int condEnd[] = { 367, 387, 413 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,0,353,413,3,0,&(condStart[0]),&(condEnd[0]),
            5,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1851, 1891 };

          static int condEnd[] = { 1887, 1942 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,1,1851,1942,2,3,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 3500, 3542 };

          static int condEnd[] = { 3538, 3582 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,2,3500,3582,2,5,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 4759, 4820 };

          static int condEnd[] = { 4816, 4874 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,3,4759,4874,2,7,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 6129, 6189, 6216 };

          static int condEnd[] = { 6184, 6211, 6243 };

          static int pfixExpr[] = { 0, 1, -3, 2, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,4,6128,6244,3,9,&(condStart[0]),&(condEnd[0]),
            5,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 7779, 7820 };

          static int condEnd[] = { 7815, 7870 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,5,7779,7870,2,12,&(condStart[0]),&(condEnd
            [0]),3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 9078, 9120 };

          static int condEnd[] = { 9116, 9160 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,6,9078,9160,2,14,&(condStart[0]),&(condEnd
            [0]),3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 10164, 10222 };

          static int condEnd[] = { 10218, 10279 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,7,10164,10279,2,16,&(condStart[0]),
            &(condEnd[0]),3,&(pfixExpr[0]));
        }

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
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_k_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_k_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_g_sf_marshallOut,(MexInFcnForType)
            c13_g_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_f_sf_marshallOut,(MexInFcnForType)
            c13_f_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_e_sf_marshallOut,(MexInFcnForType)
            c13_e_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_d_sf_marshallOut,(MexInFcnForType)
            c13_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_j_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_j_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c13_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_g_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_g_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(14,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c13_i_sf_marshallOut,(MexInFcnForType)
          c13_i_sf_marshallIn);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_c_sf_marshallOut,(MexInFcnForType)
            c13_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_b_sf_marshallOut,(MexInFcnForType)
            c13_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c13_b_sf_marshallOut,(MexInFcnForType)
            c13_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(18,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c13_h_sf_marshallOut,(MexInFcnForType)
          c13_h_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c13_sf_marshallOut,(MexInFcnForType)c13_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c13_sf_marshallOut,(MexInFcnForType)c13_sf_marshallIn);

        {
          real_T *c13_t;
          real_T *c13_currentState;
          real_T *c13_jointsSmoothingTime;
          real_T (*c13_wrench_rightFoot)[6];
          real_T (*c13_wrench_leftFoot)[6];
          real_T (*c13_w_H_b)[16];
          real_T (*c13_CoMDes)[3];
          real_T (*c13_qDes)[23];
          real_T (*c13_constraints)[2];
          real_T (*c13_CoM_0)[3];
          real_T (*c13_q0)[23];
          real_T (*c13_l_sole_CoM)[3];
          real_T (*c13_r_sole_CoM)[3];
          real_T (*c13_qj)[23];
          real_T (*c13_l_sole_H_b)[16];
          real_T (*c13_r_sole_H_b)[16];
          real_T (*c13_impedances)[23];
          real_T (*c13_kpCom)[3];
          real_T (*c13_kdCom)[3];
          c13_jointsSmoothingTime = (real_T *)ssGetOutputPortSignal
            (chartInstance->S, 9);
          c13_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
          c13_kdCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 7);
          c13_kpCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 6);
          c13_impedances = (real_T (*)[23])ssGetOutputPortSignal
            (chartInstance->S, 5);
          c13_r_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            9);
          c13_l_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            8);
          c13_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
          c13_qj = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 6);
          c13_r_sole_CoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            5);
          c13_l_sole_CoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            4);
          c13_q0 = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
          c13_CoM_0 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
          c13_constraints = (real_T (*)[2])ssGetOutputPortSignal
            (chartInstance->S, 4);
          c13_qDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
          c13_CoMDes = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
          c13_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
          c13_wrench_leftFoot = (real_T (*)[6])ssGetInputPortSignal
            (chartInstance->S, 1);
          c13_wrench_rightFoot = (real_T (*)[6])ssGetInputPortSignal
            (chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c13_wrench_rightFoot);
          _SFD_SET_DATA_VALUE_PTR(1U, *c13_wrench_leftFoot);
          _SFD_SET_DATA_VALUE_PTR(2U, *c13_w_H_b);
          _SFD_SET_DATA_VALUE_PTR(3U, *c13_CoMDes);
          _SFD_SET_DATA_VALUE_PTR(4U, *c13_qDes);
          _SFD_SET_DATA_VALUE_PTR(5U, *c13_constraints);
          _SFD_SET_DATA_VALUE_PTR(6U, *c13_CoM_0);
          _SFD_SET_DATA_VALUE_PTR(7U, *c13_q0);
          _SFD_SET_DATA_VALUE_PTR(8U, *c13_l_sole_CoM);
          _SFD_SET_DATA_VALUE_PTR(9U, *c13_r_sole_CoM);
          _SFD_SET_DATA_VALUE_PTR(10U, *c13_qj);
          _SFD_SET_DATA_VALUE_PTR(11U, c13_t);
          _SFD_SET_DATA_VALUE_PTR(12U, *c13_l_sole_H_b);
          _SFD_SET_DATA_VALUE_PTR(13U, *c13_r_sole_H_b);
          _SFD_SET_DATA_VALUE_PTR(14U, &chartInstance->c13_sm);
          _SFD_SET_DATA_VALUE_PTR(15U, *c13_impedances);
          _SFD_SET_DATA_VALUE_PTR(16U, *c13_kpCom);
          _SFD_SET_DATA_VALUE_PTR(17U, *c13_kdCom);
          _SFD_SET_DATA_VALUE_PTR(18U, &chartInstance->c13_gain);
          _SFD_SET_DATA_VALUE_PTR(19U, c13_currentState);
          _SFD_SET_DATA_VALUE_PTR(20U, c13_jointsSmoothingTime);
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
  return "O6Vzd73auY6P08e1tDcUhC";
}

static void sf_opaque_initialize_c13_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc13_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c13_torqueBalancing2012b
    ((SFc13_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c13_torqueBalancing2012b((SFc13_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c13_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c13_torqueBalancing2012b((SFc13_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c13_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c13_torqueBalancing2012b((SFc13_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c13_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c13_torqueBalancing2012b((SFc13_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c13_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c13_torqueBalancing2012b
    ((SFc13_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c13_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c13_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c13_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c13_torqueBalancing2012b
    ((SFc13_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c13_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c13_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c13_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c13_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c13_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc13_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c13_torqueBalancing2012b((SFc13_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc13_torqueBalancing2012b
    ((SFc13_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c13_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c13_torqueBalancing2012b
      ((SFc13_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c13_torqueBalancing2012b(SimStruct *S)
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
      13);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,13,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,13,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,13);
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
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,13,10);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,13,9);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=9; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 10; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,13);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2854564514U));
  ssSetChecksum1(S,(3931373591U));
  ssSetChecksum2(S,(1571133298U));
  ssSetChecksum3(S,(754710126U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c13_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c13_torqueBalancing2012b(SimStruct *S)
{
  SFc13_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc13_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc13_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc13_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c13_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c13_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c13_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c13_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c13_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c13_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c13_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c13_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c13_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c13_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c13_torqueBalancing2012b;
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

void c13_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c13_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c13_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c13_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c13_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
