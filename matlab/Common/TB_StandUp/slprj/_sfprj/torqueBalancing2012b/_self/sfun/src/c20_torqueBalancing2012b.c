/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c20_torqueBalancing2012b.h"
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
static const char * c20_debug_family_names[24] = { "nargin", "nargout", "qjRef",
  "CoM", "CoM_0", "l_sole_H_b", "l_upper_leg_contact_H_b", "t", "gain", "sm",
  "Lwrench", "Rwrench", "LHandWrench", "RHandWrench", "useExtArmForces", "w_H_b",
  "constraints", "impedances", "kpCom", "kdCom", "currentState",
  "jointsSmoothingTime", "qjDes", "CoM_Des" };

static const char * c20_b_debug_family_names[29] = { "constraints", "tDelta",
  "nargin", "nargout", "qjRef", "CoM", "CoM_0", "l_sole_H_b",
  "l_upper_leg_contact_H_b", "t", "gain", "sm", "Lwrench", "Rwrench",
  "LArmWrench", "RArmWrench", "useExtArmForces", "w_H_b", "impedances", "kpCom",
  "kdCom", "currentState", "jointsAndCoMSmoothingTime", "qjDes", "CoM_Des",
  "state", "tSwitch", "w_H_fixedLink", "CoMprevious" };

/* Function Declarations */
static void initialize_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void c20_update_debugger_state_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c20_st);
static void finalize_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c20_torqueBalancing2012b(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void c20_stateMachineStandUp(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_qjRef[23], real_T c20_CoM[3], real_T c20_CoM_0[3],
  real_T c20_l_sole_H_b[16], real_T c20_l_upper_leg_contact_H_b[16], real_T
  c20_t, c20_struct_kzTB0QQWoOlMoMhgKf6sK *c20_b_gain,
  c20_struct_rUGQ0INmvPpaxIctEGl5sE *c20_b_sm, real_T c20_Lwrench[6], real_T
  c20_Rwrench[6], real_T c20_LArmWrench[6], real_T c20_RArmWrench[6], boolean_T
  c20_useExtArmForces, real_T c20_w_H_b[16], real_T c20_impedances[23], real_T
  c20_kpCom[3], real_T c20_kdCom[3], real_T *c20_currentState, real_T
  *c20_jointsAndCoMSmoothingTime, real_T c20_qjDes[23], real_T c20_CoM_Des[3]);
static void init_script_number_translation(uint32_T c20_machineNumber, uint32_T
  c20_chartNumber);
static const mxArray *c20_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_CoM_Des, const char_T *c20_identifier,
  real_T c20_y[3]);
static void c20_b_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[3]);
static void c20_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_b_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_c_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_qjDes, const char_T *c20_identifier, real_T
  c20_y[23]);
static void c20_d_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[23]);
static void c20_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_c_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static real_T c20_e_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_jointsSmoothingTime, const char_T
  *c20_identifier);
static real_T c20_f_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void c20_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_d_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_g_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_kdCom, const char_T *c20_identifier, real_T
  c20_y[3]);
static void c20_h_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[3]);
static void c20_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_e_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_i_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_impedances, const char_T *c20_identifier,
  real_T c20_y[23]);
static void c20_j_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[23]);
static void c20_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_f_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_k_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_constraints, const char_T *c20_identifier,
  real_T c20_y[2]);
static void c20_l_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[2]);
static void c20_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_g_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_m_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_w_H_b, const char_T *c20_identifier, real_T
  c20_y[16]);
static void c20_n_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[16]);
static void c20_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_h_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static const mxArray *c20_i_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static const mxArray *c20_j_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_o_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_rUGQ0INmvPpaxIctEGl5sE *c20_y);
static boolean_T c20_p_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void c20_q_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_DnBdbfPNxiIjhNOyZMmfsE *c20_y);
static void c20_r_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[39]);
static c20_struct_KJR2itYvhBuAkuR6dKZHUC c20_s_emlrt_marshallIn
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c20_u,
   const emlrtMsgIdentifier *c20_parentId);
static void c20_t_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_0U0wBk2LiR1OqsMsUngxdD *c20_y);
static void c20_u_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[299]);
static void c20_v_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[192]);
static void c20_w_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[72]);
static void c20_x_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[13]);
static void c20_y_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[8]);
static void c20_ab_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_9LpOi5JXaV67jTuay8hWaH *c20_y);
static void c20_bb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[24]);
static void c20_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_k_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_cb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_kzTB0QQWoOlMoMhgKf6sK *c20_y);
static void c20_db_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[9]);
static void c20_eb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[4]);
static void c20_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_l_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static const mxArray *c20_m_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_fb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_b_CoMprevious, const char_T *c20_identifier,
  real_T c20_y[3]);
static void c20_gb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[3]);
static void c20_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_n_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static void c20_hb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_b_w_H_fixedLink, const char_T
  *c20_identifier, real_T c20_y[16]);
static void c20_ib_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[16]);
static void c20_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_o_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static real_T c20_jb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_b_tSwitch, const char_T *c20_identifier);
static real_T c20_kb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void c20_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static const mxArray *c20_p_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static real_T c20_lb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_b_state, const char_T *c20_identifier);
static real_T c20_mb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void c20_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static void c20_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static void c20_nb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[6]);
static void c20_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static void c20_info_helper(c20_ResolvedFunctionInfo c20_info[127]);
static void c20_b_info_helper(c20_ResolvedFunctionInfo c20_info[127]);
static void c20_mrdivide(SFc20_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c20_A[16], real_T c20_B[16], real_T c20_y[16]);
static void c20_realmin(SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void c20_eps(SFc20_torqueBalancing2012bInstanceStruct *chartInstance);
static void c20_eml_matlab_zgetrf(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_b_A[16], int32_T c20_ipiv[4],
  int32_T *c20_info);
static void c20_check_forloop_overflow_error
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c20_overflow);
static void c20_eml_xger(SFc20_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c20_m, int32_T c20_n, real_T c20_alpha1, int32_T c20_ix0, int32_T
  c20_iy0, real_T c20_A[16], int32_T c20_ia0, real_T c20_b_A[16]);
static void c20_eml_warning(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c20_eml_scalar_eg(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c20_eml_xtrsm(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_B[16], real_T c20_b_B[16]);
static void c20_below_threshold(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c20_b_eml_scalar_eg(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c20_b_eml_xtrsm(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_B[16], real_T c20_b_B[16]);
static void c20_eye(SFc20_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c20_I[16]);
static const mxArray *c20_q_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData);
static int32_T c20_ob_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void c20_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData);
static uint8_T c20_pb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c20_b_is_active_c20_torqueBalancing2012b, const
  char_T *c20_identifier);
static uint8_T c20_qb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId);
static void c20_b_eml_matlab_zgetrf(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], int32_T c20_ipiv[4], int32_T *c20_info);
static void c20_b_eml_xger(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c20_m, int32_T c20_n, real_T c20_alpha1, int32_T
  c20_ix0, int32_T c20_iy0, real_T c20_A[16], int32_T c20_ia0);
static void c20_c_eml_xtrsm(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_B[16]);
static void c20_d_eml_xtrsm(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_B[16]);
static void init_dsm_address_info(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c20_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c20_state_not_empty = FALSE;
  chartInstance->c20_tSwitch_not_empty = FALSE;
  chartInstance->c20_w_H_fixedLink_not_empty = FALSE;
  chartInstance->c20_CoMprevious_not_empty = FALSE;
  chartInstance->c20_is_active_c20_torqueBalancing2012b = 0U;
}

static void initialize_params_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c20_m0 = NULL;
  const mxArray *c20_mxField;
  c20_struct_kzTB0QQWoOlMoMhgKf6sK c20_r0;
  const mxArray *c20_m1 = NULL;
  const mxArray *c20_b_mxField;
  c20_struct_rUGQ0INmvPpaxIctEGl5sE c20_r1;
  const mxArray *c20_m2 = NULL;
  const mxArray *c20_c_mxField;
  const mxArray *c20_m3 = NULL;
  const mxArray *c20_d_mxField;
  const mxArray *c20_m4 = NULL;
  const mxArray *c20_e_mxField;
  const mxArray *c20_m5 = NULL;
  const mxArray *c20_f_mxField;
  sf_set_error_prefix_string(
    "Error evaluating data 'gain' in the parent workspace.\n");
  c20_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c20_mxField = sf_mex_getfield(c20_m0, "qTildeMax", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), &c20_r0.qTildeMax, 1, 0,
                      0U, 0, 0U, 0);
  c20_mxField = sf_mex_getfield(c20_m0, "SmoothingTimeImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), &c20_r0.SmoothingTimeImp,
                      1, 0, 0U, 0, 0U, 0);
  c20_mxField = sf_mex_getfield(c20_m0, "SmoothingTimeGainScheduling", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField),
                      &c20_r0.SmoothingTimeGainScheduling, 1, 0, 0U, 0, 0U, 0);
  c20_mxField = sf_mex_getfield(c20_m0, "PCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.PCOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c20_mxField = sf_mex_getfield(c20_m0, "ICOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.ICOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c20_mxField = sf_mex_getfield(c20_m0, "DCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.DCOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c20_mxField = sf_mex_getfield(c20_m0, "PAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), &c20_r0.PAngularMomentum,
                      1, 0, 0U, 0, 0U, 0);
  c20_mxField = sf_mex_getfield(c20_m0, "DAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), &c20_r0.DAngularMomentum,
                      1, 0, 0U, 0, 0U, 0);
  c20_mxField = sf_mex_getfield(c20_m0, "integral", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.integral, 1, 0, 0U,
                      1, 0U, 2, 1, 23);
  c20_mxField = sf_mex_getfield(c20_m0, "impedances", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.impedances, 1, 0,
                      0U, 1, 0U, 2, 1, 23);
  c20_mxField = sf_mex_getfield(c20_m0, "dampings", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.dampings, 1, 0, 0U,
                      1, 0U, 2, 1, 23);
  c20_mxField = sf_mex_getfield(c20_m0, "increasingRatesImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.increasingRatesImp,
                      1, 0, 0U, 1, 0U, 2, 1, 23);
  c20_mxField = sf_mex_getfield(c20_m0, "footSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.footSize, 1, 0, 0U,
                      1, 0U, 2, 2, 2);
  c20_mxField = sf_mex_getfield(c20_m0, "legSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c20_mxField), c20_r0.legSize, 1, 0, 0U,
                      1, 0U, 2, 2, 2);
  sf_mex_destroy(&c20_m0);
  chartInstance->c20_gain = c20_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'sm' in the parent workspace.\n");
  c20_m1 = sf_mex_get_sfun_param(chartInstance->S, 1, 1);
  c20_b_mxField = sf_mex_getfield(c20_m1, "skipYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), &c20_r1.skipYoga, 1, 11,
                      0U, 0, 0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "demoOnlyRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), &c20_r1.demoOnlyRightFoot,
                      1, 11, 0U, 0, 0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "yogaAlsoOnRightFoot", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField),
                      &c20_r1.yogaAlsoOnRightFoot, 1, 11, 0U, 0, 0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "yogaInLoop", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), &c20_r1.yogaInLoop, 1, 11,
                      0U, 0, 0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "com", "sm", 0);
  c20_m2 = sf_mex_dup(c20_b_mxField);
  c20_c_mxField = sf_mex_getfield(c20_m2, "threshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_c_mxField), &c20_r1.com.threshold, 1,
                      0, 0U, 0, 0U, 0);
  c20_c_mxField = sf_mex_getfield(c20_m2, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_c_mxField), c20_r1.com.states, 1, 0,
                      0U, 1, 0U, 2, 13, 3);
  sf_mex_destroy(&c20_m2);
  c20_b_mxField = sf_mex_getfield(c20_m1, "wrench", "sm", 0);
  c20_m3 = sf_mex_dup(c20_b_mxField);
  c20_d_mxField = sf_mex_getfield(c20_m3, "thresholdContactOn", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_d_mxField),
                      &c20_r1.wrench.thresholdContactOn, 1, 0, 0U, 0, 0U, 0);
  c20_d_mxField = sf_mex_getfield(c20_m3, "thresholdContactOff", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_d_mxField),
                      &c20_r1.wrench.thresholdContactOff, 1, 0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c20_m3);
  c20_b_mxField = sf_mex_getfield(c20_m1, "joints", "sm", 0);
  c20_m4 = sf_mex_dup(c20_b_mxField);
  c20_e_mxField = sf_mex_getfield(c20_m4, "thresholdNotInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_e_mxField),
                      &c20_r1.joints.thresholdNotInContact, 1, 0, 0U, 0, 0U, 0);
  c20_e_mxField = sf_mex_getfield(c20_m4, "thresholdInContact", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_e_mxField),
                      &c20_r1.joints.thresholdInContact, 1, 0, 0U, 0, 0U, 0);
  c20_e_mxField = sf_mex_getfield(c20_m4, "pauseTimeLastPostureL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_e_mxField),
                      &c20_r1.joints.pauseTimeLastPostureL, 1, 0, 0U, 0, 0U, 0);
  c20_e_mxField = sf_mex_getfield(c20_m4, "pauseTimeLastPostureR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_e_mxField),
                      &c20_r1.joints.pauseTimeLastPostureR, 1, 0, 0U, 0, 0U, 0);
  c20_e_mxField = sf_mex_getfield(c20_m4, "states", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_e_mxField), c20_r1.joints.states, 1,
                      0, 0U, 1, 0U, 2, 13, 23);
  c20_e_mxField = sf_mex_getfield(c20_m4, "pointsL", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_e_mxField), c20_r1.joints.pointsL, 1,
                      0, 0U, 1, 0U, 2, 8, 24);
  c20_e_mxField = sf_mex_getfield(c20_m4, "pointsR", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_e_mxField), c20_r1.joints.pointsR, 1,
                      0, 0U, 1, 0U, 2, 8, 24);
  c20_e_mxField = sf_mex_getfield(c20_m4, "standUpPositions", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_e_mxField),
                      c20_r1.joints.standUpPositions, 1, 0, 0U, 1, 0U, 2, 8, 9);
  sf_mex_destroy(&c20_m4);
  c20_b_mxField = sf_mex_getfield(c20_m1, "stateAt0", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), &c20_r1.stateAt0, 1, 0,
                      0U, 0, 0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "DT", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), &c20_r1.DT, 1, 0, 0U, 0,
                      0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "waitingTimeAfterYoga", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField),
                      &c20_r1.waitingTimeAfterYoga, 1, 0, 0U, 0, 0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "jointsSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField),
                      c20_r1.jointsSmoothingTimes, 1, 0, 0U, 1, 0U, 2, 13, 1);
  c20_b_mxField = sf_mex_getfield(c20_m1, "tBalancing", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), &c20_r1.tBalancing, 1, 0,
                      0U, 0, 0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "alsoSitDown", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), &c20_r1.alsoSitDown, 1,
                      11, 0U, 0, 0U, 0);
  c20_b_mxField = sf_mex_getfield(c20_m1, "jointsAndCoMSmoothingTimes", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField),
                      c20_r1.jointsAndCoMSmoothingTimes, 1, 0, 0U, 1, 0U, 2, 8,
                      1);
  c20_b_mxField = sf_mex_getfield(c20_m1, "CoM", "sm", 0);
  c20_m5 = sf_mex_dup(c20_b_mxField);
  c20_f_mxField = sf_mex_getfield(c20_m5, "standUpDeltaCoM", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_f_mxField),
                      c20_r1.CoM.standUpDeltaCoM, 1, 0, 0U, 1, 0U, 2, 8, 3);
  sf_mex_destroy(&c20_m5);
  c20_b_mxField = sf_mex_getfield(c20_m1, "LwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), c20_r1.LwrenchThreshold,
                      1, 0, 0U, 1, 0U, 2, 8, 1);
  c20_b_mxField = sf_mex_getfield(c20_m1, "RwrenchThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), c20_r1.RwrenchThreshold,
                      1, 0, 0U, 1, 0U, 2, 8, 1);
  c20_b_mxField = sf_mex_getfield(c20_m1, "RArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), c20_r1.RArmThreshold, 1,
                      0, 0U, 1, 0U, 2, 8, 1);
  c20_b_mxField = sf_mex_getfield(c20_m1, "LArmThreshold", "sm", 0);
  sf_mex_import_named("sm", sf_mex_dup(c20_b_mxField), c20_r1.LArmThreshold, 1,
                      0, 0U, 1, 0U, 2, 8, 1);
  sf_mex_destroy(&c20_m1);
  chartInstance->c20_sm = c20_r1;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c20_update_debugger_state_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c20_st;
  const mxArray *c20_y = NULL;
  int32_T c20_i0;
  real_T c20_u[3];
  const mxArray *c20_b_y = NULL;
  int32_T c20_i1;
  real_T c20_b_u[2];
  const mxArray *c20_c_y = NULL;
  real_T c20_hoistedGlobal;
  real_T c20_c_u;
  const mxArray *c20_d_y = NULL;
  int32_T c20_i2;
  real_T c20_d_u[23];
  const mxArray *c20_e_y = NULL;
  real_T c20_b_hoistedGlobal;
  real_T c20_e_u;
  const mxArray *c20_f_y = NULL;
  int32_T c20_i3;
  real_T c20_f_u[3];
  const mxArray *c20_g_y = NULL;
  int32_T c20_i4;
  real_T c20_g_u[3];
  const mxArray *c20_h_y = NULL;
  int32_T c20_i5;
  real_T c20_h_u[23];
  const mxArray *c20_i_y = NULL;
  int32_T c20_i6;
  real_T c20_i_u[16];
  const mxArray *c20_j_y = NULL;
  int32_T c20_i7;
  real_T c20_j_u[3];
  const mxArray *c20_k_y = NULL;
  real_T c20_c_hoistedGlobal;
  real_T c20_k_u;
  const mxArray *c20_l_y = NULL;
  real_T c20_d_hoistedGlobal;
  real_T c20_l_u;
  const mxArray *c20_m_y = NULL;
  int32_T c20_i8;
  real_T c20_m_u[16];
  const mxArray *c20_n_y = NULL;
  uint8_T c20_e_hoistedGlobal;
  uint8_T c20_n_u;
  const mxArray *c20_o_y = NULL;
  real_T *c20_currentState;
  real_T *c20_jointsSmoothingTime;
  real_T (*c20_w_H_b)[16];
  real_T (*c20_qjDes)[23];
  real_T (*c20_kpCom)[3];
  real_T (*c20_kdCom)[3];
  real_T (*c20_impedances)[23];
  real_T (*c20_constraints)[2];
  real_T (*c20_CoM_Des)[3];
  c20_CoM_Des = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 9);
  c20_qjDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 8);
  c20_jointsSmoothingTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c20_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c20_kdCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 5);
  c20_kpCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 4);
  c20_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c20_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c20_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c20_st = NULL;
  c20_st = NULL;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_createcellarray(14), FALSE);
  for (c20_i0 = 0; c20_i0 < 3; c20_i0++) {
    c20_u[c20_i0] = (*c20_CoM_Des)[c20_i0];
  }

  c20_b_y = NULL;
  sf_mex_assign(&c20_b_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c20_y, 0, c20_b_y);
  for (c20_i1 = 0; c20_i1 < 2; c20_i1++) {
    c20_b_u[c20_i1] = (*c20_constraints)[c20_i1];
  }

  c20_c_y = NULL;
  sf_mex_assign(&c20_c_y, sf_mex_create("y", c20_b_u, 0, 0U, 1U, 0U, 1, 2),
                FALSE);
  sf_mex_setcell(c20_y, 1, c20_c_y);
  c20_hoistedGlobal = *c20_currentState;
  c20_c_u = c20_hoistedGlobal;
  c20_d_y = NULL;
  sf_mex_assign(&c20_d_y, sf_mex_create("y", &c20_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c20_y, 2, c20_d_y);
  for (c20_i2 = 0; c20_i2 < 23; c20_i2++) {
    c20_d_u[c20_i2] = (*c20_impedances)[c20_i2];
  }

  c20_e_y = NULL;
  sf_mex_assign(&c20_e_y, sf_mex_create("y", c20_d_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_setcell(c20_y, 3, c20_e_y);
  c20_b_hoistedGlobal = *c20_jointsSmoothingTime;
  c20_e_u = c20_b_hoistedGlobal;
  c20_f_y = NULL;
  sf_mex_assign(&c20_f_y, sf_mex_create("y", &c20_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c20_y, 4, c20_f_y);
  for (c20_i3 = 0; c20_i3 < 3; c20_i3++) {
    c20_f_u[c20_i3] = (*c20_kdCom)[c20_i3];
  }

  c20_g_y = NULL;
  sf_mex_assign(&c20_g_y, sf_mex_create("y", c20_f_u, 0, 0U, 1U, 0U, 2, 1, 3),
                FALSE);
  sf_mex_setcell(c20_y, 5, c20_g_y);
  for (c20_i4 = 0; c20_i4 < 3; c20_i4++) {
    c20_g_u[c20_i4] = (*c20_kpCom)[c20_i4];
  }

  c20_h_y = NULL;
  sf_mex_assign(&c20_h_y, sf_mex_create("y", c20_g_u, 0, 0U, 1U, 0U, 2, 1, 3),
                FALSE);
  sf_mex_setcell(c20_y, 6, c20_h_y);
  for (c20_i5 = 0; c20_i5 < 23; c20_i5++) {
    c20_h_u[c20_i5] = (*c20_qjDes)[c20_i5];
  }

  c20_i_y = NULL;
  sf_mex_assign(&c20_i_y, sf_mex_create("y", c20_h_u, 0, 0U, 1U, 0U, 1, 23),
                FALSE);
  sf_mex_setcell(c20_y, 7, c20_i_y);
  for (c20_i6 = 0; c20_i6 < 16; c20_i6++) {
    c20_i_u[c20_i6] = (*c20_w_H_b)[c20_i6];
  }

  c20_j_y = NULL;
  sf_mex_assign(&c20_j_y, sf_mex_create("y", c20_i_u, 0, 0U, 1U, 0U, 2, 4, 4),
                FALSE);
  sf_mex_setcell(c20_y, 8, c20_j_y);
  for (c20_i7 = 0; c20_i7 < 3; c20_i7++) {
    c20_j_u[c20_i7] = chartInstance->c20_CoMprevious[c20_i7];
  }

  c20_k_y = NULL;
  if (!chartInstance->c20_CoMprevious_not_empty) {
    sf_mex_assign(&c20_k_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c20_k_y, sf_mex_create("y", c20_j_u, 0, 0U, 1U, 0U, 1, 3),
                  FALSE);
  }

  sf_mex_setcell(c20_y, 9, c20_k_y);
  c20_c_hoistedGlobal = chartInstance->c20_state;
  c20_k_u = c20_c_hoistedGlobal;
  c20_l_y = NULL;
  if (!chartInstance->c20_state_not_empty) {
    sf_mex_assign(&c20_l_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c20_l_y, sf_mex_create("y", &c20_k_u, 0, 0U, 0U, 0U, 0),
                  FALSE);
  }

  sf_mex_setcell(c20_y, 10, c20_l_y);
  c20_d_hoistedGlobal = chartInstance->c20_tSwitch;
  c20_l_u = c20_d_hoistedGlobal;
  c20_m_y = NULL;
  if (!chartInstance->c20_tSwitch_not_empty) {
    sf_mex_assign(&c20_m_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c20_m_y, sf_mex_create("y", &c20_l_u, 0, 0U, 0U, 0U, 0),
                  FALSE);
  }

  sf_mex_setcell(c20_y, 11, c20_m_y);
  for (c20_i8 = 0; c20_i8 < 16; c20_i8++) {
    c20_m_u[c20_i8] = chartInstance->c20_w_H_fixedLink[c20_i8];
  }

  c20_n_y = NULL;
  if (!chartInstance->c20_w_H_fixedLink_not_empty) {
    sf_mex_assign(&c20_n_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c20_n_y, sf_mex_create("y", c20_m_u, 0, 0U, 1U, 0U, 2, 4, 4),
                  FALSE);
  }

  sf_mex_setcell(c20_y, 12, c20_n_y);
  c20_e_hoistedGlobal = chartInstance->c20_is_active_c20_torqueBalancing2012b;
  c20_n_u = c20_e_hoistedGlobal;
  c20_o_y = NULL;
  sf_mex_assign(&c20_o_y, sf_mex_create("y", &c20_n_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c20_y, 13, c20_o_y);
  sf_mex_assign(&c20_st, c20_y, FALSE);
  return c20_st;
}

static void set_sim_state_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c20_st)
{
  const mxArray *c20_u;
  real_T c20_dv0[3];
  int32_T c20_i9;
  real_T c20_dv1[2];
  int32_T c20_i10;
  real_T c20_dv2[23];
  int32_T c20_i11;
  real_T c20_dv3[3];
  int32_T c20_i12;
  real_T c20_dv4[3];
  int32_T c20_i13;
  real_T c20_dv5[23];
  int32_T c20_i14;
  real_T c20_dv6[16];
  int32_T c20_i15;
  real_T c20_dv7[3];
  int32_T c20_i16;
  real_T c20_dv8[16];
  int32_T c20_i17;
  real_T *c20_currentState;
  real_T *c20_jointsSmoothingTime;
  real_T (*c20_CoM_Des)[3];
  real_T (*c20_constraints)[2];
  real_T (*c20_qjDes)[23];
  real_T (*c20_w_H_b)[16];
  real_T (*c20_kpCom)[3];
  real_T (*c20_kdCom)[3];
  real_T (*c20_impedances)[23];
  c20_CoM_Des = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 9);
  c20_qjDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 8);
  c20_jointsSmoothingTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c20_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c20_kdCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 5);
  c20_kpCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 4);
  c20_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c20_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c20_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c20_doneDoubleBufferReInit = TRUE;
  c20_u = sf_mex_dup(c20_st);
  c20_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 0)),
                       "CoM_Des", c20_dv0);
  for (c20_i9 = 0; c20_i9 < 3; c20_i9++) {
    (*c20_CoM_Des)[c20_i9] = c20_dv0[c20_i9];
  }

  c20_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 1)),
    "constraints", c20_dv1);
  for (c20_i10 = 0; c20_i10 < 2; c20_i10++) {
    (*c20_constraints)[c20_i10] = c20_dv1[c20_i10];
  }

  *c20_currentState = c20_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c20_u, 2)), "currentState");
  c20_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 3)),
    "impedances", c20_dv2);
  for (c20_i11 = 0; c20_i11 < 23; c20_i11++) {
    (*c20_impedances)[c20_i11] = c20_dv2[c20_i11];
  }

  *c20_jointsSmoothingTime = c20_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c20_u, 4)), "jointsSmoothingTime");
  c20_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 5)),
    "kdCom", c20_dv3);
  for (c20_i12 = 0; c20_i12 < 3; c20_i12++) {
    (*c20_kdCom)[c20_i12] = c20_dv3[c20_i12];
  }

  c20_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 6)),
    "kpCom", c20_dv4);
  for (c20_i13 = 0; c20_i13 < 3; c20_i13++) {
    (*c20_kpCom)[c20_i13] = c20_dv4[c20_i13];
  }

  c20_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 7)),
    "qjDes", c20_dv5);
  for (c20_i14 = 0; c20_i14 < 23; c20_i14++) {
    (*c20_qjDes)[c20_i14] = c20_dv5[c20_i14];
  }

  c20_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 8)),
    "w_H_b", c20_dv6);
  for (c20_i15 = 0; c20_i15 < 16; c20_i15++) {
    (*c20_w_H_b)[c20_i15] = c20_dv6[c20_i15];
  }

  c20_fb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 9)),
    "CoMprevious", c20_dv7);
  for (c20_i16 = 0; c20_i16 < 3; c20_i16++) {
    chartInstance->c20_CoMprevious[c20_i16] = c20_dv7[c20_i16];
  }

  chartInstance->c20_state = c20_lb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c20_u, 10)), "state");
  chartInstance->c20_tSwitch = c20_jb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c20_u, 11)), "tSwitch");
  c20_hb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 12)),
    "w_H_fixedLink", c20_dv8);
  for (c20_i17 = 0; c20_i17 < 16; c20_i17++) {
    chartInstance->c20_w_H_fixedLink[c20_i17] = c20_dv8[c20_i17];
  }

  chartInstance->c20_is_active_c20_torqueBalancing2012b =
    c20_pb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c20_u, 13)),
    "is_active_c20_torqueBalancing2012b");
  sf_mex_destroy(&c20_u);
  c20_update_debugger_state_c20_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c20_st);
}

static void finalize_c20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c20_torqueBalancing2012b(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c20_i18;
  int32_T c20_i19;
  int32_T c20_i20;
  int32_T c20_i21;
  int32_T c20_i22;
  int32_T c20_i23;
  int32_T c20_i24;
  int32_T c20_i25;
  int32_T c20_i26;
  int32_T c20_i27;
  int32_T c20_i28;
  int32_T c20_i29;
  int32_T c20_i30;
  int32_T c20_i31;
  int32_T c20_i32;
  int32_T c20_i33;
  real_T c20_hoistedGlobal;
  boolean_T c20_b_hoistedGlobal;
  int32_T c20_i34;
  real_T c20_qjRef[23];
  int32_T c20_i35;
  real_T c20_CoM[3];
  int32_T c20_i36;
  real_T c20_CoM_0[3];
  int32_T c20_i37;
  real_T c20_l_sole_H_b[16];
  int32_T c20_i38;
  real_T c20_l_upper_leg_contact_H_b[16];
  real_T c20_t;
  c20_struct_kzTB0QQWoOlMoMhgKf6sK c20_b_gain;
  c20_struct_rUGQ0INmvPpaxIctEGl5sE c20_b_sm;
  int32_T c20_i39;
  real_T c20_Lwrench[6];
  int32_T c20_i40;
  real_T c20_Rwrench[6];
  int32_T c20_i41;
  real_T c20_LHandWrench[6];
  int32_T c20_i42;
  real_T c20_RHandWrench[6];
  boolean_T c20_useExtArmForces;
  uint32_T c20_debug_family_var_map[24];
  real_T c20_nargin = 13.0;
  real_T c20_nargout = 9.0;
  real_T c20_w_H_b[16];
  real_T c20_constraints[2];
  real_T c20_impedances[23];
  real_T c20_kpCom[3];
  real_T c20_kdCom[3];
  real_T c20_currentState;
  real_T c20_jointsSmoothingTime;
  real_T c20_qjDes[23];
  real_T c20_CoM_Des[3];
  int32_T c20_i43;
  real_T c20_b_qjRef[23];
  int32_T c20_i44;
  real_T c20_b_CoM[3];
  int32_T c20_i45;
  real_T c20_b_CoM_0[3];
  int32_T c20_i46;
  real_T c20_b_l_sole_H_b[16];
  int32_T c20_i47;
  real_T c20_b_l_upper_leg_contact_H_b[16];
  c20_struct_kzTB0QQWoOlMoMhgKf6sK c20_c_gain;
  c20_struct_rUGQ0INmvPpaxIctEGl5sE c20_c_sm;
  int32_T c20_i48;
  real_T c20_b_Lwrench[6];
  int32_T c20_i49;
  real_T c20_b_Rwrench[6];
  int32_T c20_i50;
  real_T c20_b_LHandWrench[6];
  int32_T c20_i51;
  real_T c20_b_RHandWrench[6];
  real_T c20_b_CoM_Des[3];
  real_T c20_b_qjDes[23];
  real_T c20_b_jointsSmoothingTime;
  real_T c20_b_currentState;
  real_T c20_b_kdCom[3];
  real_T c20_b_kpCom[3];
  real_T c20_b_impedances[23];
  real_T c20_b_w_H_b[16];
  int32_T c20_i52;
  int32_T c20_i53;
  int32_T c20_i54;
  int32_T c20_i55;
  int32_T c20_i56;
  int32_T c20_i57;
  int32_T c20_i58;
  int32_T c20_i59;
  int32_T c20_i60;
  int32_T c20_i61;
  int32_T c20_i62;
  int32_T c20_i63;
  int32_T c20_i64;
  int32_T c20_i65;
  real_T *c20_b_t;
  real_T *c20_c_currentState;
  real_T *c20_c_jointsSmoothingTime;
  boolean_T *c20_b_useExtArmForces;
  real_T (*c20_c_w_H_b)[16];
  real_T (*c20_b_constraints)[2];
  real_T (*c20_c_qjDes)[23];
  real_T (*c20_c_CoM_Des)[3];
  real_T (*c20_c_kdCom)[3];
  real_T (*c20_c_kpCom)[3];
  real_T (*c20_c_impedances)[23];
  real_T (*c20_c_RHandWrench)[6];
  real_T (*c20_c_LHandWrench)[6];
  real_T (*c20_c_Rwrench)[6];
  real_T (*c20_c_Lwrench)[6];
  real_T (*c20_c_l_upper_leg_contact_H_b)[16];
  real_T (*c20_c_l_sole_H_b)[16];
  real_T (*c20_c_CoM_0)[3];
  real_T (*c20_c_CoM)[3];
  real_T (*c20_c_qjRef)[23];
  c20_b_useExtArmForces = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 10);
  c20_c_RHandWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 9);
  c20_c_LHandWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 8);
  c20_c_Rwrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 7);
  c20_c_Lwrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 6);
  c20_c_CoM_Des = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 9);
  c20_c_qjDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 8);
  c20_c_jointsSmoothingTime = (real_T *)ssGetOutputPortSignal(chartInstance->S,
    7);
  c20_c_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c20_c_kdCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 5);
  c20_c_kpCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 4);
  c20_c_impedances = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 3);
  c20_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c20_b_constraints = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 2);
  c20_c_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c20_c_l_upper_leg_contact_H_b = (real_T (*)[16])ssGetInputPortSignal
    (chartInstance->S, 4);
  c20_c_l_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 3);
  c20_c_CoM_0 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c20_c_CoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c20_c_qjRef = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 18U, chartInstance->c20_sfEvent);
  for (c20_i18 = 0; c20_i18 < 23; c20_i18++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_qjRef)[c20_i18], 0U);
  }

  for (c20_i19 = 0; c20_i19 < 3; c20_i19++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_CoM)[c20_i19], 1U);
  }

  for (c20_i20 = 0; c20_i20 < 3; c20_i20++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_CoM_0)[c20_i20], 2U);
  }

  for (c20_i21 = 0; c20_i21 < 16; c20_i21++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_l_sole_H_b)[c20_i21], 3U);
  }

  for (c20_i22 = 0; c20_i22 < 16; c20_i22++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_l_upper_leg_contact_H_b)[c20_i22], 4U);
  }

  for (c20_i23 = 0; c20_i23 < 16; c20_i23++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_w_H_b)[c20_i23], 5U);
  }

  for (c20_i24 = 0; c20_i24 < 2; c20_i24++) {
    _SFD_DATA_RANGE_CHECK((*c20_b_constraints)[c20_i24], 6U);
  }

  _SFD_DATA_RANGE_CHECK(*c20_b_t, 7U);
  for (c20_i25 = 0; c20_i25 < 23; c20_i25++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_impedances)[c20_i25], 8U);
  }

  for (c20_i26 = 0; c20_i26 < 3; c20_i26++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_kpCom)[c20_i26], 9U);
  }

  for (c20_i27 = 0; c20_i27 < 3; c20_i27++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_kdCom)[c20_i27], 10U);
  }

  _SFD_DATA_RANGE_CHECK(*c20_c_currentState, 12U);
  _SFD_DATA_RANGE_CHECK(*c20_c_jointsSmoothingTime, 13U);
  for (c20_i28 = 0; c20_i28 < 23; c20_i28++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_qjDes)[c20_i28], 15U);
  }

  for (c20_i29 = 0; c20_i29 < 3; c20_i29++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_CoM_Des)[c20_i29], 16U);
  }

  for (c20_i30 = 0; c20_i30 < 6; c20_i30++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_Lwrench)[c20_i30], 17U);
  }

  for (c20_i31 = 0; c20_i31 < 6; c20_i31++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_Rwrench)[c20_i31], 18U);
  }

  for (c20_i32 = 0; c20_i32 < 6; c20_i32++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_LHandWrench)[c20_i32], 19U);
  }

  for (c20_i33 = 0; c20_i33 < 6; c20_i33++) {
    _SFD_DATA_RANGE_CHECK((*c20_c_RHandWrench)[c20_i33], 20U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)*c20_b_useExtArmForces, 21U);
  chartInstance->c20_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 18U, chartInstance->c20_sfEvent);
  c20_hoistedGlobal = *c20_b_t;
  c20_b_hoistedGlobal = *c20_b_useExtArmForces;
  for (c20_i34 = 0; c20_i34 < 23; c20_i34++) {
    c20_qjRef[c20_i34] = (*c20_c_qjRef)[c20_i34];
  }

  for (c20_i35 = 0; c20_i35 < 3; c20_i35++) {
    c20_CoM[c20_i35] = (*c20_c_CoM)[c20_i35];
  }

  for (c20_i36 = 0; c20_i36 < 3; c20_i36++) {
    c20_CoM_0[c20_i36] = (*c20_c_CoM_0)[c20_i36];
  }

  for (c20_i37 = 0; c20_i37 < 16; c20_i37++) {
    c20_l_sole_H_b[c20_i37] = (*c20_c_l_sole_H_b)[c20_i37];
  }

  for (c20_i38 = 0; c20_i38 < 16; c20_i38++) {
    c20_l_upper_leg_contact_H_b[c20_i38] = (*c20_c_l_upper_leg_contact_H_b)
      [c20_i38];
  }

  c20_t = c20_hoistedGlobal;
  c20_b_gain = chartInstance->c20_gain;
  c20_b_sm = chartInstance->c20_sm;
  for (c20_i39 = 0; c20_i39 < 6; c20_i39++) {
    c20_Lwrench[c20_i39] = (*c20_c_Lwrench)[c20_i39];
  }

  for (c20_i40 = 0; c20_i40 < 6; c20_i40++) {
    c20_Rwrench[c20_i40] = (*c20_c_Rwrench)[c20_i40];
  }

  for (c20_i41 = 0; c20_i41 < 6; c20_i41++) {
    c20_LHandWrench[c20_i41] = (*c20_c_LHandWrench)[c20_i41];
  }

  for (c20_i42 = 0; c20_i42 < 6; c20_i42++) {
    c20_RHandWrench[c20_i42] = (*c20_c_RHandWrench)[c20_i42];
  }

  c20_useExtArmForces = c20_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 24U, 24U, c20_debug_family_names,
    c20_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_nargin, 0U, c20_c_sf_marshallOut,
    c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_nargout, 1U, c20_c_sf_marshallOut,
    c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_qjRef, 2U, c20_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_CoM, 3U, c20_l_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_CoM_0, 4U, c20_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_l_sole_H_b, 5U, c20_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_l_upper_leg_contact_H_b, 6U,
    c20_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c20_t, 7U, c20_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_b_gain, 8U, c20_k_sf_marshallOut,
    c20_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_b_sm, 9U, c20_j_sf_marshallOut,
    c20_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_Lwrench, 10U, c20_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_Rwrench, 11U, c20_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_LHandWrench, 12U, c20_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c20_RHandWrench, 13U, c20_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c20_useExtArmForces, 14U, c20_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_w_H_b, 15U, c20_g_sf_marshallOut,
    c20_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_constraints, 16U,
    c20_f_sf_marshallOut, c20_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_impedances, 17U, c20_e_sf_marshallOut,
    c20_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_kpCom, 18U, c20_d_sf_marshallOut,
    c20_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_kdCom, 19U, c20_d_sf_marshallOut,
    c20_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_currentState, 20U,
    c20_c_sf_marshallOut, c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_jointsSmoothingTime, 21U,
    c20_c_sf_marshallOut, c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_qjDes, 22U, c20_b_sf_marshallOut,
    c20_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_CoM_Des, 23U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, 4);
  for (c20_i43 = 0; c20_i43 < 23; c20_i43++) {
    c20_b_qjRef[c20_i43] = c20_qjRef[c20_i43];
  }

  for (c20_i44 = 0; c20_i44 < 3; c20_i44++) {
    c20_b_CoM[c20_i44] = c20_CoM[c20_i44];
  }

  for (c20_i45 = 0; c20_i45 < 3; c20_i45++) {
    c20_b_CoM_0[c20_i45] = c20_CoM_0[c20_i45];
  }

  for (c20_i46 = 0; c20_i46 < 16; c20_i46++) {
    c20_b_l_sole_H_b[c20_i46] = c20_l_sole_H_b[c20_i46];
  }

  for (c20_i47 = 0; c20_i47 < 16; c20_i47++) {
    c20_b_l_upper_leg_contact_H_b[c20_i47] = c20_l_upper_leg_contact_H_b[c20_i47];
  }

  c20_c_gain = c20_b_gain;
  c20_c_sm = c20_b_sm;
  for (c20_i48 = 0; c20_i48 < 6; c20_i48++) {
    c20_b_Lwrench[c20_i48] = c20_Lwrench[c20_i48];
  }

  for (c20_i49 = 0; c20_i49 < 6; c20_i49++) {
    c20_b_Rwrench[c20_i49] = c20_Rwrench[c20_i49];
  }

  for (c20_i50 = 0; c20_i50 < 6; c20_i50++) {
    c20_b_LHandWrench[c20_i50] = c20_LHandWrench[c20_i50];
  }

  for (c20_i51 = 0; c20_i51 < 6; c20_i51++) {
    c20_b_RHandWrench[c20_i51] = c20_RHandWrench[c20_i51];
  }

  c20_stateMachineStandUp(chartInstance, c20_b_qjRef, c20_b_CoM, c20_b_CoM_0,
    c20_b_l_sole_H_b, c20_b_l_upper_leg_contact_H_b, c20_t, &c20_c_gain,
    &c20_c_sm, c20_b_Lwrench, c20_b_Rwrench, c20_b_LHandWrench,
    c20_b_RHandWrench, c20_useExtArmForces, c20_b_w_H_b, c20_b_impedances,
    c20_b_kpCom, c20_b_kdCom, &c20_b_currentState, &c20_b_jointsSmoothingTime,
    c20_b_qjDes, c20_b_CoM_Des);
  for (c20_i52 = 0; c20_i52 < 16; c20_i52++) {
    c20_w_H_b[c20_i52] = c20_b_w_H_b[c20_i52];
  }

  for (c20_i53 = 0; c20_i53 < 2; c20_i53++) {
    c20_constraints[c20_i53] = 1.0;
  }

  for (c20_i54 = 0; c20_i54 < 23; c20_i54++) {
    c20_impedances[c20_i54] = c20_b_impedances[c20_i54];
  }

  for (c20_i55 = 0; c20_i55 < 3; c20_i55++) {
    c20_kpCom[c20_i55] = c20_b_kpCom[c20_i55];
  }

  for (c20_i56 = 0; c20_i56 < 3; c20_i56++) {
    c20_kdCom[c20_i56] = c20_b_kdCom[c20_i56];
  }

  c20_currentState = c20_b_currentState;
  c20_jointsSmoothingTime = c20_b_jointsSmoothingTime;
  for (c20_i57 = 0; c20_i57 < 23; c20_i57++) {
    c20_qjDes[c20_i57] = c20_b_qjDes[c20_i57];
  }

  for (c20_i58 = 0; c20_i58 < 3; c20_i58++) {
    c20_CoM_Des[c20_i58] = c20_b_CoM_Des[c20_i58];
  }

  _SFD_EML_CALL(0U, chartInstance->c20_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
  for (c20_i59 = 0; c20_i59 < 16; c20_i59++) {
    (*c20_c_w_H_b)[c20_i59] = c20_w_H_b[c20_i59];
  }

  for (c20_i60 = 0; c20_i60 < 2; c20_i60++) {
    (*c20_b_constraints)[c20_i60] = c20_constraints[c20_i60];
  }

  for (c20_i61 = 0; c20_i61 < 23; c20_i61++) {
    (*c20_c_impedances)[c20_i61] = c20_impedances[c20_i61];
  }

  for (c20_i62 = 0; c20_i62 < 3; c20_i62++) {
    (*c20_c_kpCom)[c20_i62] = c20_kpCom[c20_i62];
  }

  for (c20_i63 = 0; c20_i63 < 3; c20_i63++) {
    (*c20_c_kdCom)[c20_i63] = c20_kdCom[c20_i63];
  }

  *c20_c_currentState = c20_currentState;
  *c20_c_jointsSmoothingTime = c20_jointsSmoothingTime;
  for (c20_i64 = 0; c20_i64 < 23; c20_i64++) {
    (*c20_c_qjDes)[c20_i64] = c20_qjDes[c20_i64];
  }

  for (c20_i65 = 0; c20_i65 < 3; c20_i65++) {
    (*c20_c_CoM_Des)[c20_i65] = c20_CoM_Des[c20_i65];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 18U, chartInstance->c20_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc20_torqueBalancing2012b
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c20_stateMachineStandUp(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_qjRef[23], real_T c20_CoM[3], real_T c20_CoM_0[3],
  real_T c20_l_sole_H_b[16], real_T c20_l_upper_leg_contact_H_b[16], real_T
  c20_t, c20_struct_kzTB0QQWoOlMoMhgKf6sK *c20_b_gain,
  c20_struct_rUGQ0INmvPpaxIctEGl5sE *c20_b_sm, real_T c20_Lwrench[6], real_T
  c20_Rwrench[6], real_T c20_LArmWrench[6], real_T c20_RArmWrench[6], boolean_T
  c20_useExtArmForces, real_T c20_w_H_b[16], real_T c20_impedances[23], real_T
  c20_kpCom[3], real_T c20_kdCom[3], real_T *c20_currentState, real_T
  *c20_jointsAndCoMSmoothingTime, real_T c20_qjDes[23], real_T c20_CoM_Des[3])
{
  uint32_T c20_debug_family_var_map[29];
  real_T c20_constraints[2];
  real_T c20_tDelta;
  real_T c20_nargin = 13.0;
  real_T c20_nargout = 9.0;
  int32_T c20_i66;
  real_T c20_b_l_sole_H_b[16];
  int32_T c20_i67;
  real_T c20_b_l_upper_leg_contact_H_b[16];
  real_T c20_dv9[16];
  int32_T c20_i68;
  int32_T c20_i69;
  int32_T c20_i70;
  real_T c20_dv10[16];
  int32_T c20_i71;
  int32_T c20_i72;
  int32_T c20_i73;
  int32_T c20_i74;
  real_T c20_hoistedGlobal[16];
  int32_T c20_i75;
  real_T c20_b[16];
  int32_T c20_i76;
  int32_T c20_i77;
  int32_T c20_i78;
  real_T c20_C[16];
  int32_T c20_i79;
  int32_T c20_i80;
  int32_T c20_i81;
  int32_T c20_i82;
  int32_T c20_i83;
  int32_T c20_i84;
  int32_T c20_i85;
  int32_T c20_i86;
  int32_T c20_i87;
  int32_T c20_i88;
  int32_T c20_i89;
  int32_T c20_i90;
  int32_T c20_i91;
  int32_T c20_i92;
  int32_T c20_i93;
  int32_T c20_i94;
  int32_T c20_i95;
  int32_T c20_i96;
  int32_T c20_i97;
  int32_T c20_i98;
  int32_T c20_i99;
  int32_T c20_i100;
  int32_T c20_i101;
  static int32_T c20_iv0[4] = { 17, 18, 20, 21 };

  int32_T c20_i102;
  int32_T c20_i103;
  static int32_T c20_iv1[4] = { 11, 12, 14, 15 };

  int32_T c20_i104;
  int32_T c20_i105;
  int32_T c20_i106;
  int32_T c20_i107;
  int32_T c20_i108;
  int32_T c20_i109;
  int32_T c20_i110;
  int32_T c20_i111;
  int32_T c20_i112;
  int32_T c20_i113;
  int32_T c20_i114;
  int32_T c20_i115;
  int32_T c20_i116;
  int32_T c20_i117;
  real_T c20_b_C[16];
  int32_T c20_i118;
  real_T c20_c_l_sole_H_b[16];
  real_T c20_dv11[16];
  int32_T c20_i119;
  int32_T c20_i120;
  int32_T c20_i121;
  int32_T c20_i122;
  int32_T c20_i123;
  int32_T c20_i124;
  int32_T c20_i125;
  int32_T c20_i126;
  int32_T c20_i127;
  int32_T c20_i128;
  int32_T c20_i129;
  int32_T c20_i130;
  int32_T c20_i131;
  int32_T c20_i132;
  int32_T c20_i133;
  int32_T c20_i134;
  int32_T c20_i135;
  int32_T c20_i136;
  int32_T c20_i137;
  int32_T c20_i138;
  int32_T c20_i139;
  int32_T c20_i140;
  int32_T c20_i141;
  int32_T c20_i142;
  int32_T c20_i143;
  int32_T c20_i144;
  int32_T c20_i145;
  int32_T c20_i146;
  int32_T c20_i147;
  int32_T c20_i148;
  int32_T c20_i149;
  int32_T c20_i150;
  int32_T c20_i151;
  int32_T c20_i152;
  int32_T c20_i153;
  int32_T c20_i154;
  int32_T c20_i155;
  int32_T c20_i156;
  int32_T c20_i157;
  int32_T c20_i158;
  int32_T c20_i159;
  int32_T c20_i160;
  int32_T c20_i161;
  int32_T c20_i162;
  int32_T c20_i163;
  int32_T c20_i164;
  int32_T c20_i165;
  int32_T c20_i166;
  int32_T c20_i167;
  int32_T c20_i168;
  int32_T c20_i169;
  int32_T c20_i170;
  int32_T c20_i171;
  int32_T c20_i172;
  int32_T c20_i173;
  int32_T c20_i174;
  int32_T c20_i175;
  int32_T c20_i176;
  int32_T c20_i177;
  int32_T c20_i178;
  int32_T c20_i179;
  int32_T c20_i180;
  int32_T c20_i181;
  int32_T c20_i182;
  int32_T c20_i183;
  int32_T c20_i184;
  int32_T c20_i185;
  int32_T c20_i186;
  int32_T c20_i187;
  int32_T c20_i188;
  int32_T c20_i189;
  int32_T c20_i190;
  int32_T c20_i191;
  int32_T c20_i192;
  int32_T c20_i193;
  int32_T c20_i194;
  int32_T c20_i195;
  int32_T c20_i196;
  int32_T c20_i197;
  int32_T c20_i198;
  int32_T c20_i199;
  int32_T c20_i200;
  int32_T c20_i201;
  int32_T c20_i202;
  int32_T c20_i203;
  int32_T c20_i204;
  int32_T c20_i205;
  int32_T c20_i206;
  int32_T c20_i207;
  int32_T c20_i208;
  int32_T c20_i209;
  int32_T c20_i210;
  int32_T c20_i211;
  int32_T c20_i212;
  int32_T c20_i213;
  int32_T c20_i214;
  int32_T c20_i215;
  int32_T c20_i216;
  int32_T c20_i217;
  int32_T c20_i218;
  int32_T c20_i219;
  int32_T c20_i220;
  int32_T c20_i221;
  int32_T c20_i222;
  int32_T c20_i223;
  int32_T c20_i224;
  real_T c20_c_C[16];
  int32_T c20_i225;
  real_T c20_c_l_upper_leg_contact_H_b[16];
  real_T c20_dv12[16];
  int32_T c20_i226;
  int32_T c20_i227;
  int32_T c20_i228;
  int32_T c20_i229;
  int32_T c20_i230;
  int32_T c20_i231;
  int32_T c20_i232;
  int32_T c20_i233;
  int32_T c20_i234;
  int32_T c20_i235;
  int32_T c20_i236;
  int32_T c20_i237;
  int32_T c20_i238;
  int32_T c20_i239;
  int32_T c20_i240;
  int32_T c20_i241;
  int32_T c20_i242;
  int32_T c20_i243;
  int32_T c20_i244;
  int32_T c20_i245;
  int32_T c20_i246;
  int32_T c20_i247;
  int32_T c20_i248;
  int32_T c20_i249;
  int32_T c20_i250;
  int32_T c20_i251;
  int32_T c20_i252;
  int32_T c20_i253;
  int32_T c20_i254;
  int32_T c20_i255;
  int32_T c20_i256;
  int32_T c20_i257;
  int32_T c20_i258;
  int32_T c20_i259;
  int32_T c20_i260;
  int32_T c20_i261;
  int32_T c20_i262;
  int32_T c20_i263;
  int32_T c20_i264;
  int32_T c20_i265;
  int32_T c20_i266;
  int32_T c20_i267;
  int32_T c20_i268;
  int32_T c20_i269;
  int32_T c20_i270;
  int32_T c20_i271;
  int32_T c20_i272;
  int32_T c20_i273;
  int32_T c20_i274;
  int32_T c20_i275;
  int32_T c20_i276;
  int32_T c20_i277;
  int32_T c20_i278;
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
  boolean_T guard11 = FALSE;
  boolean_T guard12 = FALSE;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 29U, 29U, c20_b_debug_family_names,
    c20_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_constraints, 0U, c20_f_sf_marshallOut,
    c20_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_tDelta, 1U, c20_c_sf_marshallOut,
    c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_nargin, 2U, c20_c_sf_marshallOut,
    c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_nargout, 3U, c20_c_sf_marshallOut,
    c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_qjRef, 4U, c20_b_sf_marshallOut,
    c20_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_CoM, 5U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_CoM_0, 6U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_l_sole_H_b, 7U, c20_g_sf_marshallOut,
    c20_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_l_upper_leg_contact_H_b, 8U,
    c20_g_sf_marshallOut, c20_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_t, 9U, c20_c_sf_marshallOut,
    c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_b_gain, 10U, c20_k_sf_marshallOut,
    c20_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_b_sm, 11U, c20_j_sf_marshallOut,
    c20_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_Lwrench, 12U, c20_i_sf_marshallOut,
    c20_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_Rwrench, 13U, c20_i_sf_marshallOut,
    c20_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_LArmWrench, 14U, c20_i_sf_marshallOut,
    c20_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_RArmWrench, 15U, c20_i_sf_marshallOut,
    c20_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c20_useExtArmForces, 16U,
    c20_h_sf_marshallOut, c20_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_w_H_b, 17U, c20_g_sf_marshallOut,
    c20_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_impedances, 18U, c20_e_sf_marshallOut,
    c20_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_kpCom, 19U, c20_d_sf_marshallOut,
    c20_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_kdCom, 20U, c20_d_sf_marshallOut,
    c20_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_currentState, 21U,
    c20_c_sf_marshallOut, c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_jointsAndCoMSmoothingTime, 22U,
    c20_c_sf_marshallOut, c20_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_qjDes, 23U, c20_b_sf_marshallOut,
    c20_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c20_CoM_Des, 24U, c20_sf_marshallOut,
    c20_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c20_state, 25U,
    c20_p_sf_marshallOut, c20_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c20_tSwitch, 26U,
    c20_o_sf_marshallOut, c20_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c20_w_H_fixedLink, 27U,
    c20_n_sf_marshallOut, c20_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c20_CoMprevious, 28U,
    c20_m_sf_marshallOut, c20_j_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 4);
  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 5);
  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 6);
  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 7);
  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 9);
  guard10 = FALSE;
  guard11 = FALSE;
  guard12 = FALSE;
  if (CV_SCRIPT_COND(0, 0, !chartInstance->c20_state_not_empty)) {
    guard12 = TRUE;
  } else if (CV_SCRIPT_COND(0, 1, !chartInstance->c20_tSwitch_not_empty)) {
    guard12 = TRUE;
  } else if (CV_SCRIPT_COND(0, 2, !chartInstance->c20_w_H_fixedLink_not_empty))
  {
    guard11 = TRUE;
  } else if (CV_SCRIPT_COND(0, 3, !chartInstance->c20_CoMprevious_not_empty)) {
    guard10 = TRUE;
  } else {
    CV_SCRIPT_MCDC(0, 0, FALSE);
    CV_SCRIPT_IF(0, 0, FALSE);
  }

  if (guard12 == TRUE) {
    guard11 = TRUE;
  }

  if (guard11 == TRUE) {
    guard10 = TRUE;
  }

  if (guard10 == TRUE) {
    CV_SCRIPT_MCDC(0, 0, TRUE);
    CV_SCRIPT_IF(0, 0, TRUE);
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 10);
    chartInstance->c20_state = c20_b_sm->stateAt0;
    chartInstance->c20_state_not_empty = TRUE;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 11);
    chartInstance->c20_tSwitch = 0.0;
    chartInstance->c20_tSwitch_not_empty = TRUE;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 12);
    for (c20_i66 = 0; c20_i66 < 16; c20_i66++) {
      c20_b_l_sole_H_b[c20_i66] = c20_l_sole_H_b[c20_i66];
    }

    for (c20_i67 = 0; c20_i67 < 16; c20_i67++) {
      c20_b_l_upper_leg_contact_H_b[c20_i67] =
        c20_l_upper_leg_contact_H_b[c20_i67];
    }

    c20_mrdivide(chartInstance, c20_b_l_sole_H_b, c20_b_l_upper_leg_contact_H_b,
                 c20_dv9);
    for (c20_i68 = 0; c20_i68 < 16; c20_i68++) {
      chartInstance->c20_w_H_fixedLink[c20_i68] = c20_dv9[c20_i68];
    }

    chartInstance->c20_w_H_fixedLink_not_empty = TRUE;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 13);
    for (c20_i69 = 0; c20_i69 < 3; c20_i69++) {
      chartInstance->c20_CoMprevious[c20_i69] = c20_CoM_0[c20_i69];
    }

    chartInstance->c20_CoMprevious_not_empty = TRUE;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 16);
  for (c20_i70 = 0; c20_i70 < 2; c20_i70++) {
    c20_constraints[c20_i70] = 1.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 17);
  c20_eye(chartInstance, c20_dv10);
  for (c20_i71 = 0; c20_i71 < 16; c20_i71++) {
    c20_w_H_b[c20_i71] = c20_dv10[c20_i71];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 18);
  for (c20_i72 = 0; c20_i72 < 23; c20_i72++) {
    c20_qjDes[c20_i72] = c20_qjRef[c20_i72];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 19);
  for (c20_i73 = 0; c20_i73 < 3; c20_i73++) {
    c20_CoM_Des[c20_i73] = c20_CoM_0[c20_i73];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 20);
  *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes[(int32_T)
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("sm.jointsAndCoMSmoothingTimes",
    (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8, 1, 0)
    - 1];
  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 23);
  if (CV_SCRIPT_IF(0, 1, chartInstance->c20_state == 1.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 25);
    for (c20_i74 = 0; c20_i74 < 16; c20_i74++) {
      c20_hoistedGlobal[c20_i74] = chartInstance->c20_w_H_fixedLink[c20_i74];
    }

    for (c20_i75 = 0; c20_i75 < 16; c20_i75++) {
      c20_b[c20_i75] = c20_l_upper_leg_contact_H_b[c20_i75];
    }

    c20_eml_scalar_eg(chartInstance);
    c20_eml_scalar_eg(chartInstance);
    for (c20_i76 = 0; c20_i76 < 16; c20_i76++) {
      c20_w_H_b[c20_i76] = 0.0;
    }

    for (c20_i77 = 0; c20_i77 < 16; c20_i77++) {
      c20_w_H_b[c20_i77] = 0.0;
    }

    for (c20_i78 = 0; c20_i78 < 16; c20_i78++) {
      c20_C[c20_i78] = c20_w_H_b[c20_i78];
    }

    for (c20_i79 = 0; c20_i79 < 16; c20_i79++) {
      c20_w_H_b[c20_i79] = c20_C[c20_i79];
    }

    for (c20_i80 = 0; c20_i80 < 16; c20_i80++) {
      c20_C[c20_i80] = c20_w_H_b[c20_i80];
    }

    for (c20_i81 = 0; c20_i81 < 16; c20_i81++) {
      c20_w_H_b[c20_i81] = c20_C[c20_i81];
    }

    for (c20_i82 = 0; c20_i82 < 4; c20_i82++) {
      c20_i83 = 0;
      for (c20_i84 = 0; c20_i84 < 4; c20_i84++) {
        c20_w_H_b[c20_i83 + c20_i82] = 0.0;
        c20_i85 = 0;
        for (c20_i86 = 0; c20_i86 < 4; c20_i86++) {
          c20_w_H_b[c20_i83 + c20_i82] += c20_hoistedGlobal[c20_i85 + c20_i82] *
            c20_b[c20_i86 + c20_i83];
          c20_i85 += 4;
        }

        c20_i83 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 26);
    *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes
      [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.jointsAndCoMSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 30);
    if (CV_SCRIPT_IF(0, 2, (real_T)c20_useExtArmForces == 1.0)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 32);
      guard8 = FALSE;
      guard9 = FALSE;
      if (CV_SCRIPT_COND(0, 4, c20_t > c20_b_sm->tBalancing)) {
        if (CV_SCRIPT_COND(0, 5, c20_RArmWrench[0] > c20_b_sm->RArmThreshold
                           [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
              "sm.RArmThreshold", (int32_T)_SFD_INTEGER_CHECK("state",
               chartInstance->c20_state), 1, 8, 1, 0) - 1])) {
          if (CV_SCRIPT_COND(0, 6, c20_LArmWrench[0] > c20_b_sm->LArmThreshold
                             [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
                "sm.LArmThreshold", (int32_T)_SFD_INTEGER_CHECK("state",
                 chartInstance->c20_state), 1, 8, 1, 0) - 1])) {
            CV_SCRIPT_MCDC(0, 1, TRUE);
            CV_SCRIPT_IF(0, 3, TRUE);
            _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 33);
            chartInstance->c20_state = 2.0;
            _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 34);
            chartInstance->c20_tSwitch = c20_t;
          } else {
            guard8 = TRUE;
          }
        } else {
          guard9 = TRUE;
        }
      } else {
        guard9 = TRUE;
      }

      if (guard9 == TRUE) {
        guard8 = TRUE;
      }

      if (guard8 == TRUE) {
        CV_SCRIPT_MCDC(0, 1, FALSE);
        CV_SCRIPT_IF(0, 3, FALSE);
      }
    } else {
      _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 37);
      if (CV_SCRIPT_IF(0, 4, c20_t > c20_b_sm->tBalancing)) {
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 38);
        chartInstance->c20_state = 2.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 39);
        chartInstance->c20_tSwitch = c20_t;
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 45);
  if (CV_SCRIPT_IF(0, 5, chartInstance->c20_state == 2.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 47);
    for (c20_i87 = 0; c20_i87 < 16; c20_i87++) {
      c20_hoistedGlobal[c20_i87] = chartInstance->c20_w_H_fixedLink[c20_i87];
    }

    for (c20_i88 = 0; c20_i88 < 16; c20_i88++) {
      c20_b[c20_i88] = c20_l_upper_leg_contact_H_b[c20_i88];
    }

    c20_eml_scalar_eg(chartInstance);
    c20_eml_scalar_eg(chartInstance);
    for (c20_i89 = 0; c20_i89 < 16; c20_i89++) {
      c20_w_H_b[c20_i89] = 0.0;
    }

    for (c20_i90 = 0; c20_i90 < 16; c20_i90++) {
      c20_w_H_b[c20_i90] = 0.0;
    }

    for (c20_i91 = 0; c20_i91 < 16; c20_i91++) {
      c20_C[c20_i91] = c20_w_H_b[c20_i91];
    }

    for (c20_i92 = 0; c20_i92 < 16; c20_i92++) {
      c20_w_H_b[c20_i92] = c20_C[c20_i92];
    }

    for (c20_i93 = 0; c20_i93 < 16; c20_i93++) {
      c20_C[c20_i93] = c20_w_H_b[c20_i93];
    }

    for (c20_i94 = 0; c20_i94 < 16; c20_i94++) {
      c20_w_H_b[c20_i94] = c20_C[c20_i94];
    }

    for (c20_i95 = 0; c20_i95 < 4; c20_i95++) {
      c20_i96 = 0;
      for (c20_i97 = 0; c20_i97 < 4; c20_i97++) {
        c20_w_H_b[c20_i96 + c20_i95] = 0.0;
        c20_i98 = 0;
        for (c20_i99 = 0; c20_i99 < 4; c20_i99++) {
          c20_w_H_b[c20_i96 + c20_i95] += c20_hoistedGlobal[c20_i98 + c20_i95] *
            c20_b[c20_i99 + c20_i96];
          c20_i98 += 4;
        }

        c20_i96 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 50);
    c20_i100 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i101 = 0; c20_i101 < 4; c20_i101++) {
      c20_qjDes[c20_iv0[c20_i101]] = c20_b_sm->joints.standUpPositions[c20_i100
        + (c20_i101 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 51);
    c20_i102 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i103 = 0; c20_i103 < 4; c20_i103++) {
      c20_qjDes[c20_iv1[c20_i103]] = c20_b_sm->joints.standUpPositions[c20_i102
        + (c20_i103 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 52);
    c20_i104 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i105 = 0; c20_i105 < 4; c20_i105++) {
      c20_qjDes[7 + c20_i105] = c20_b_sm->joints.standUpPositions[c20_i104 + ((4
        + c20_i105) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 53);
    c20_i106 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i107 = 0; c20_i107 < 4; c20_i107++) {
      c20_qjDes[3 + c20_i107] = c20_b_sm->joints.standUpPositions[c20_i106 + ((4
        + c20_i107) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 54);
    c20_qjDes[0] = c20_b_sm->joints.standUpPositions[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.standUpPositions", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8, 1, 0) + 63];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 56);
    c20_tDelta = c20_t - chartInstance->c20_tSwitch;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 57);
    c20_i108 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.CoM.standUpDeltaCoM", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i109 = 0; c20_i109 < 3; c20_i109++) {
      c20_CoM_Des[c20_i109] = c20_CoM_0[c20_i109] +
        c20_b_sm->CoM.standUpDeltaCoM[c20_i108 + (c20_i109 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 58);
    *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes
      [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.jointsAndCoMSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 60);
    guard7 = FALSE;
    if (CV_SCRIPT_COND(0, 7, c20_Lwrench[2] + c20_Rwrench[2] >
                       c20_b_sm->LwrenchThreshold[(int32_T)(real_T)
                       _SFD_EML_ARRAY_BOUNDS_CHECK("sm.LwrenchThreshold",
          (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8,
          1, 0) - 1] + c20_b_sm->RwrenchThreshold[(int32_T)(real_T)
                       _SFD_EML_ARRAY_BOUNDS_CHECK("sm.RwrenchThreshold",
          (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8,
          1, 0) - 1])) {
      if (CV_SCRIPT_COND(0, 8, c20_tDelta > 1.5)) {
        CV_SCRIPT_MCDC(0, 2, TRUE);
        CV_SCRIPT_IF(0, 6, TRUE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 61);
        chartInstance->c20_state = 3.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 62);
        for (c20_i110 = 0; c20_i110 < 16; c20_i110++) {
          c20_hoistedGlobal[c20_i110] = chartInstance->
            c20_w_H_fixedLink[c20_i110];
        }

        for (c20_i111 = 0; c20_i111 < 16; c20_i111++) {
          c20_b[c20_i111] = c20_l_upper_leg_contact_H_b[c20_i111];
        }

        c20_eml_scalar_eg(chartInstance);
        c20_eml_scalar_eg(chartInstance);
        for (c20_i112 = 0; c20_i112 < 4; c20_i112++) {
          c20_i113 = 0;
          for (c20_i114 = 0; c20_i114 < 4; c20_i114++) {
            c20_C[c20_i113 + c20_i112] = 0.0;
            c20_i115 = 0;
            for (c20_i116 = 0; c20_i116 < 4; c20_i116++) {
              c20_C[c20_i113 + c20_i112] += c20_hoistedGlobal[c20_i115 +
                c20_i112] * c20_b[c20_i116 + c20_i113];
              c20_i115 += 4;
            }

            c20_i113 += 4;
          }
        }

        for (c20_i117 = 0; c20_i117 < 16; c20_i117++) {
          c20_b_C[c20_i117] = c20_C[c20_i117];
        }

        for (c20_i118 = 0; c20_i118 < 16; c20_i118++) {
          c20_c_l_sole_H_b[c20_i118] = c20_l_sole_H_b[c20_i118];
        }

        c20_mrdivide(chartInstance, c20_b_C, c20_c_l_sole_H_b, c20_dv11);
        for (c20_i119 = 0; c20_i119 < 16; c20_i119++) {
          chartInstance->c20_w_H_fixedLink[c20_i119] = c20_dv11[c20_i119];
        }

        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 63);
        chartInstance->c20_tSwitch = c20_t;
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 64);
        for (c20_i120 = 0; c20_i120 < 3; c20_i120++) {
          chartInstance->c20_CoMprevious[c20_i120] = c20_CoM[c20_i120];
        }
      } else {
        guard7 = TRUE;
      }
    } else {
      guard7 = TRUE;
    }

    if (guard7 == TRUE) {
      CV_SCRIPT_MCDC(0, 2, FALSE);
      CV_SCRIPT_IF(0, 6, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 70);
  if (CV_SCRIPT_IF(0, 7, chartInstance->c20_state == 3.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 72);
    for (c20_i121 = 0; c20_i121 < 16; c20_i121++) {
      c20_hoistedGlobal[c20_i121] = chartInstance->c20_w_H_fixedLink[c20_i121];
    }

    for (c20_i122 = 0; c20_i122 < 16; c20_i122++) {
      c20_b[c20_i122] = c20_l_sole_H_b[c20_i122];
    }

    c20_eml_scalar_eg(chartInstance);
    c20_eml_scalar_eg(chartInstance);
    for (c20_i123 = 0; c20_i123 < 16; c20_i123++) {
      c20_w_H_b[c20_i123] = 0.0;
    }

    for (c20_i124 = 0; c20_i124 < 16; c20_i124++) {
      c20_w_H_b[c20_i124] = 0.0;
    }

    for (c20_i125 = 0; c20_i125 < 16; c20_i125++) {
      c20_C[c20_i125] = c20_w_H_b[c20_i125];
    }

    for (c20_i126 = 0; c20_i126 < 16; c20_i126++) {
      c20_w_H_b[c20_i126] = c20_C[c20_i126];
    }

    for (c20_i127 = 0; c20_i127 < 16; c20_i127++) {
      c20_C[c20_i127] = c20_w_H_b[c20_i127];
    }

    for (c20_i128 = 0; c20_i128 < 16; c20_i128++) {
      c20_w_H_b[c20_i128] = c20_C[c20_i128];
    }

    for (c20_i129 = 0; c20_i129 < 4; c20_i129++) {
      c20_i130 = 0;
      for (c20_i131 = 0; c20_i131 < 4; c20_i131++) {
        c20_w_H_b[c20_i130 + c20_i129] = 0.0;
        c20_i132 = 0;
        for (c20_i133 = 0; c20_i133 < 4; c20_i133++) {
          c20_w_H_b[c20_i130 + c20_i129] += c20_hoistedGlobal[c20_i132 +
            c20_i129] * c20_b[c20_i133 + c20_i130];
          c20_i132 += 4;
        }

        c20_i130 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 75);
    c20_i134 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i135 = 0; c20_i135 < 4; c20_i135++) {
      c20_qjDes[c20_iv0[c20_i135]] = c20_b_sm->joints.standUpPositions[c20_i134
        + (c20_i135 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 76);
    c20_i136 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i137 = 0; c20_i137 < 4; c20_i137++) {
      c20_qjDes[c20_iv1[c20_i137]] = c20_b_sm->joints.standUpPositions[c20_i136
        + (c20_i137 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 77);
    c20_i138 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i139 = 0; c20_i139 < 4; c20_i139++) {
      c20_qjDes[7 + c20_i139] = c20_b_sm->joints.standUpPositions[c20_i138 + ((4
        + c20_i139) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 78);
    c20_i140 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i141 = 0; c20_i141 < 4; c20_i141++) {
      c20_qjDes[3 + c20_i141] = c20_b_sm->joints.standUpPositions[c20_i140 + ((4
        + c20_i141) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 79);
    c20_qjDes[0] = c20_b_sm->joints.standUpPositions[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.standUpPositions", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8, 1, 0) + 63];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 81);
    c20_i142 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.CoM.standUpDeltaCoM", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i143 = 0; c20_i143 < 3; c20_i143++) {
      c20_CoM_Des[c20_i143] = chartInstance->c20_CoMprevious[c20_i143] +
        c20_b_sm->CoM.standUpDeltaCoM[c20_i142 + (c20_i143 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 82);
    c20_tDelta = c20_t - chartInstance->c20_tSwitch;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 83);
    *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes
      [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.jointsAndCoMSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 85);
    guard5 = FALSE;
    guard6 = FALSE;
    if (CV_SCRIPT_COND(0, 9, c20_Lwrench[2] > c20_b_sm->LwrenchThreshold
                       [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
          "sm.LwrenchThreshold", (int32_T)_SFD_INTEGER_CHECK("state",
           chartInstance->c20_state), 1, 8, 1, 0) - 1])) {
      if (CV_SCRIPT_COND(0, 10, c20_Rwrench[2] > c20_b_sm->RwrenchThreshold
                         [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
            "sm.RwrenchThreshold", (int32_T)_SFD_INTEGER_CHECK("state",
             chartInstance->c20_state), 1, 8, 1, 0) - 1])) {
        if (CV_SCRIPT_COND(0, 11, c20_tDelta > 1.0)) {
          CV_SCRIPT_MCDC(0, 3, TRUE);
          CV_SCRIPT_IF(0, 8, TRUE);
          _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 86);
          chartInstance->c20_state = 4.0;
          _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 87);
          chartInstance->c20_tSwitch = c20_t;
          _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 88);
          for (c20_i144 = 0; c20_i144 < 3; c20_i144++) {
            chartInstance->c20_CoMprevious[c20_i144] = c20_CoM[c20_i144];
          }
        } else {
          guard5 = TRUE;
        }
      } else {
        guard6 = TRUE;
      }
    } else {
      guard6 = TRUE;
    }

    if (guard6 == TRUE) {
      guard5 = TRUE;
    }

    if (guard5 == TRUE) {
      CV_SCRIPT_MCDC(0, 3, FALSE);
      CV_SCRIPT_IF(0, 8, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 93);
  if (CV_SCRIPT_IF(0, 9, chartInstance->c20_state == 4.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 95);
    for (c20_i145 = 0; c20_i145 < 16; c20_i145++) {
      c20_hoistedGlobal[c20_i145] = chartInstance->c20_w_H_fixedLink[c20_i145];
    }

    for (c20_i146 = 0; c20_i146 < 16; c20_i146++) {
      c20_b[c20_i146] = c20_l_sole_H_b[c20_i146];
    }

    c20_eml_scalar_eg(chartInstance);
    c20_eml_scalar_eg(chartInstance);
    for (c20_i147 = 0; c20_i147 < 16; c20_i147++) {
      c20_w_H_b[c20_i147] = 0.0;
    }

    for (c20_i148 = 0; c20_i148 < 16; c20_i148++) {
      c20_w_H_b[c20_i148] = 0.0;
    }

    for (c20_i149 = 0; c20_i149 < 16; c20_i149++) {
      c20_C[c20_i149] = c20_w_H_b[c20_i149];
    }

    for (c20_i150 = 0; c20_i150 < 16; c20_i150++) {
      c20_w_H_b[c20_i150] = c20_C[c20_i150];
    }

    for (c20_i151 = 0; c20_i151 < 16; c20_i151++) {
      c20_C[c20_i151] = c20_w_H_b[c20_i151];
    }

    for (c20_i152 = 0; c20_i152 < 16; c20_i152++) {
      c20_w_H_b[c20_i152] = c20_C[c20_i152];
    }

    for (c20_i153 = 0; c20_i153 < 4; c20_i153++) {
      c20_i154 = 0;
      for (c20_i155 = 0; c20_i155 < 4; c20_i155++) {
        c20_w_H_b[c20_i154 + c20_i153] = 0.0;
        c20_i156 = 0;
        for (c20_i157 = 0; c20_i157 < 4; c20_i157++) {
          c20_w_H_b[c20_i154 + c20_i153] += c20_hoistedGlobal[c20_i156 +
            c20_i153] * c20_b[c20_i157 + c20_i154];
          c20_i156 += 4;
        }

        c20_i154 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 98);
    c20_i158 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i159 = 0; c20_i159 < 4; c20_i159++) {
      c20_qjDes[c20_iv0[c20_i159]] = c20_b_sm->joints.standUpPositions[c20_i158
        + (c20_i159 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 99);
    c20_i160 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i161 = 0; c20_i161 < 4; c20_i161++) {
      c20_qjDes[c20_iv1[c20_i161]] = c20_b_sm->joints.standUpPositions[c20_i160
        + (c20_i161 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 100);
    c20_i162 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i163 = 0; c20_i163 < 4; c20_i163++) {
      c20_qjDes[7 + c20_i163] = c20_b_sm->joints.standUpPositions[c20_i162 + ((4
        + c20_i163) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 101);
    c20_i164 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i165 = 0; c20_i165 < 4; c20_i165++) {
      c20_qjDes[3 + c20_i165] = c20_b_sm->joints.standUpPositions[c20_i164 + ((4
        + c20_i165) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 102);
    c20_qjDes[0] = c20_b_sm->joints.standUpPositions[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.standUpPositions", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8, 1, 0) + 63];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 104);
    c20_i166 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.CoM.standUpDeltaCoM", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i167 = 0; c20_i167 < 3; c20_i167++) {
      c20_CoM_Des[c20_i167] = chartInstance->c20_CoMprevious[c20_i167] +
        c20_b_sm->CoM.standUpDeltaCoM[c20_i166 + (c20_i167 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 105);
    *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes
      [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.jointsAndCoMSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 106);
    c20_tDelta = c20_t - chartInstance->c20_tSwitch;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 108);
    guard4 = FALSE;
    if (CV_SCRIPT_COND(0, 12, c20_tDelta > 2.5)) {
      if (CV_SCRIPT_COND(0, 13, c20_b_sm->alsoSitDown)) {
        CV_SCRIPT_MCDC(0, 4, TRUE);
        CV_SCRIPT_IF(0, 10, TRUE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 109);
        chartInstance->c20_state = 5.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 110);
        chartInstance->c20_tSwitch = c20_t;
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 111);
        for (c20_i168 = 0; c20_i168 < 3; c20_i168++) {
          chartInstance->c20_CoMprevious[c20_i168] = c20_CoM_Des[c20_i168];
        }
      } else {
        guard4 = TRUE;
      }
    } else {
      guard4 = TRUE;
    }

    if (guard4 == TRUE) {
      CV_SCRIPT_MCDC(0, 4, FALSE);
      CV_SCRIPT_IF(0, 10, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 117);
  if (CV_SCRIPT_IF(0, 11, chartInstance->c20_state == 5.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 119);
    for (c20_i169 = 0; c20_i169 < 16; c20_i169++) {
      c20_hoistedGlobal[c20_i169] = chartInstance->c20_w_H_fixedLink[c20_i169];
    }

    for (c20_i170 = 0; c20_i170 < 16; c20_i170++) {
      c20_b[c20_i170] = c20_l_sole_H_b[c20_i170];
    }

    c20_eml_scalar_eg(chartInstance);
    c20_eml_scalar_eg(chartInstance);
    for (c20_i171 = 0; c20_i171 < 16; c20_i171++) {
      c20_w_H_b[c20_i171] = 0.0;
    }

    for (c20_i172 = 0; c20_i172 < 16; c20_i172++) {
      c20_w_H_b[c20_i172] = 0.0;
    }

    for (c20_i173 = 0; c20_i173 < 16; c20_i173++) {
      c20_C[c20_i173] = c20_w_H_b[c20_i173];
    }

    for (c20_i174 = 0; c20_i174 < 16; c20_i174++) {
      c20_w_H_b[c20_i174] = c20_C[c20_i174];
    }

    for (c20_i175 = 0; c20_i175 < 16; c20_i175++) {
      c20_C[c20_i175] = c20_w_H_b[c20_i175];
    }

    for (c20_i176 = 0; c20_i176 < 16; c20_i176++) {
      c20_w_H_b[c20_i176] = c20_C[c20_i176];
    }

    for (c20_i177 = 0; c20_i177 < 4; c20_i177++) {
      c20_i178 = 0;
      for (c20_i179 = 0; c20_i179 < 4; c20_i179++) {
        c20_w_H_b[c20_i178 + c20_i177] = 0.0;
        c20_i180 = 0;
        for (c20_i181 = 0; c20_i181 < 4; c20_i181++) {
          c20_w_H_b[c20_i178 + c20_i177] += c20_hoistedGlobal[c20_i180 +
            c20_i177] * c20_b[c20_i181 + c20_i178];
          c20_i180 += 4;
        }

        c20_i178 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 122);
    c20_i182 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i183 = 0; c20_i183 < 4; c20_i183++) {
      c20_qjDes[c20_iv0[c20_i183]] = c20_b_sm->joints.standUpPositions[c20_i182
        + (c20_i183 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 123);
    c20_i184 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i185 = 0; c20_i185 < 4; c20_i185++) {
      c20_qjDes[c20_iv1[c20_i185]] = c20_b_sm->joints.standUpPositions[c20_i184
        + (c20_i185 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 124);
    c20_i186 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i187 = 0; c20_i187 < 4; c20_i187++) {
      c20_qjDes[7 + c20_i187] = c20_b_sm->joints.standUpPositions[c20_i186 + ((4
        + c20_i187) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 125);
    c20_i188 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i189 = 0; c20_i189 < 4; c20_i189++) {
      c20_qjDes[3 + c20_i189] = c20_b_sm->joints.standUpPositions[c20_i188 + ((4
        + c20_i189) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 126);
    c20_qjDes[0] = c20_b_sm->joints.standUpPositions[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.standUpPositions", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8, 1, 0) + 63];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 128U);
    c20_i190 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.CoM.standUpDeltaCoM", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i191 = 0; c20_i191 < 3; c20_i191++) {
      c20_CoM_Des[c20_i191] = chartInstance->c20_CoMprevious[c20_i191] +
        c20_b_sm->CoM.standUpDeltaCoM[c20_i190 + (c20_i191 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 129U);
    *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes
      [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.jointsAndCoMSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 130U);
    c20_tDelta = c20_t - chartInstance->c20_tSwitch;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 132U);
    guard2 = FALSE;
    guard3 = FALSE;
    if (CV_SCRIPT_COND(0, 14, c20_tDelta > 2.5)) {
      if (CV_SCRIPT_COND(0, 15, c20_RArmWrench[2] < c20_b_sm->RArmThreshold
                         [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
            "sm.RArmThreshold", (int32_T)_SFD_INTEGER_CHECK("state",
             chartInstance->c20_state), 1, 8, 1, 0) - 1])) {
        if (CV_SCRIPT_COND(0, 16, c20_LArmWrench[2] < c20_b_sm->LArmThreshold
                           [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
              "sm.LArmThreshold", (int32_T)_SFD_INTEGER_CHECK("state",
               chartInstance->c20_state), 1, 8, 1, 0) - 1])) {
          CV_SCRIPT_MCDC(0, 5, TRUE);
          CV_SCRIPT_IF(0, 12, TRUE);
          _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 133U);
          chartInstance->c20_state = 6.0;
          _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 134U);
          chartInstance->c20_tSwitch = c20_t;
          _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 135U);
          for (c20_i192 = 0; c20_i192 < 3; c20_i192++) {
            chartInstance->c20_CoMprevious[c20_i192] = c20_CoM[c20_i192];
          }
        } else {
          guard2 = TRUE;
        }
      } else {
        guard3 = TRUE;
      }
    } else {
      guard3 = TRUE;
    }

    if (guard3 == TRUE) {
      guard2 = TRUE;
    }

    if (guard2 == TRUE) {
      CV_SCRIPT_MCDC(0, 5, FALSE);
      CV_SCRIPT_IF(0, 12, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 141U);
  if (CV_SCRIPT_IF(0, 13, chartInstance->c20_state == 6.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 143U);
    for (c20_i193 = 0; c20_i193 < 16; c20_i193++) {
      c20_hoistedGlobal[c20_i193] = chartInstance->c20_w_H_fixedLink[c20_i193];
    }

    for (c20_i194 = 0; c20_i194 < 16; c20_i194++) {
      c20_b[c20_i194] = c20_l_sole_H_b[c20_i194];
    }

    c20_eml_scalar_eg(chartInstance);
    c20_eml_scalar_eg(chartInstance);
    for (c20_i195 = 0; c20_i195 < 16; c20_i195++) {
      c20_w_H_b[c20_i195] = 0.0;
    }

    for (c20_i196 = 0; c20_i196 < 16; c20_i196++) {
      c20_w_H_b[c20_i196] = 0.0;
    }

    for (c20_i197 = 0; c20_i197 < 16; c20_i197++) {
      c20_C[c20_i197] = c20_w_H_b[c20_i197];
    }

    for (c20_i198 = 0; c20_i198 < 16; c20_i198++) {
      c20_w_H_b[c20_i198] = c20_C[c20_i198];
    }

    for (c20_i199 = 0; c20_i199 < 16; c20_i199++) {
      c20_C[c20_i199] = c20_w_H_b[c20_i199];
    }

    for (c20_i200 = 0; c20_i200 < 16; c20_i200++) {
      c20_w_H_b[c20_i200] = c20_C[c20_i200];
    }

    for (c20_i201 = 0; c20_i201 < 4; c20_i201++) {
      c20_i202 = 0;
      for (c20_i203 = 0; c20_i203 < 4; c20_i203++) {
        c20_w_H_b[c20_i202 + c20_i201] = 0.0;
        c20_i204 = 0;
        for (c20_i205 = 0; c20_i205 < 4; c20_i205++) {
          c20_w_H_b[c20_i202 + c20_i201] += c20_hoistedGlobal[c20_i204 +
            c20_i201] * c20_b[c20_i205 + c20_i202];
          c20_i204 += 4;
        }

        c20_i202 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 146U);
    c20_i206 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i207 = 0; c20_i207 < 4; c20_i207++) {
      c20_qjDes[c20_iv0[c20_i207]] = c20_b_sm->joints.standUpPositions[c20_i206
        + (c20_i207 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 147U);
    c20_i208 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i209 = 0; c20_i209 < 4; c20_i209++) {
      c20_qjDes[c20_iv1[c20_i209]] = c20_b_sm->joints.standUpPositions[c20_i208
        + (c20_i209 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 148U);
    c20_i210 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i211 = 0; c20_i211 < 4; c20_i211++) {
      c20_qjDes[7 + c20_i211] = c20_b_sm->joints.standUpPositions[c20_i210 + ((4
        + c20_i211) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 149U);
    c20_i212 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i213 = 0; c20_i213 < 4; c20_i213++) {
      c20_qjDes[3 + c20_i213] = c20_b_sm->joints.standUpPositions[c20_i212 + ((4
        + c20_i213) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 150U);
    c20_qjDes[0] = c20_b_sm->joints.standUpPositions[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.standUpPositions", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8, 1, 0) + 63];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 152U);
    c20_i214 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.CoM.standUpDeltaCoM", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i215 = 0; c20_i215 < 3; c20_i215++) {
      c20_CoM_Des[c20_i215] = chartInstance->c20_CoMprevious[c20_i215] +
        c20_b_sm->CoM.standUpDeltaCoM[c20_i214 + (c20_i215 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 153U);
    *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes
      [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.jointsAndCoMSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 154U);
    c20_tDelta = c20_t - chartInstance->c20_tSwitch;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 156U);
    guard1 = FALSE;
    if (CV_SCRIPT_COND(0, 17, c20_Lwrench[2] + c20_Rwrench[2] <
                       c20_b_sm->LwrenchThreshold[(int32_T)(real_T)
                       _SFD_EML_ARRAY_BOUNDS_CHECK("sm.LwrenchThreshold",
          (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8,
          1, 0) - 1] + c20_b_sm->RwrenchThreshold[(int32_T)(real_T)
                       _SFD_EML_ARRAY_BOUNDS_CHECK("sm.RwrenchThreshold",
          (int32_T)_SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8,
          1, 0) - 1])) {
      if (CV_SCRIPT_COND(0, 18, c20_tDelta > 4.0)) {
        CV_SCRIPT_MCDC(0, 6, TRUE);
        CV_SCRIPT_IF(0, 14, TRUE);
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 157U);
        chartInstance->c20_state = 7.0;
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 158U);
        chartInstance->c20_tSwitch = c20_t;
        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 159U);
        for (c20_i216 = 0; c20_i216 < 3; c20_i216++) {
          chartInstance->c20_CoMprevious[c20_i216] = c20_CoM[c20_i216];
        }

        _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 160U);
        for (c20_i217 = 0; c20_i217 < 16; c20_i217++) {
          c20_hoistedGlobal[c20_i217] = chartInstance->
            c20_w_H_fixedLink[c20_i217];
        }

        for (c20_i218 = 0; c20_i218 < 16; c20_i218++) {
          c20_b[c20_i218] = c20_l_sole_H_b[c20_i218];
        }

        c20_eml_scalar_eg(chartInstance);
        c20_eml_scalar_eg(chartInstance);
        for (c20_i219 = 0; c20_i219 < 4; c20_i219++) {
          c20_i220 = 0;
          for (c20_i221 = 0; c20_i221 < 4; c20_i221++) {
            c20_C[c20_i220 + c20_i219] = 0.0;
            c20_i222 = 0;
            for (c20_i223 = 0; c20_i223 < 4; c20_i223++) {
              c20_C[c20_i220 + c20_i219] += c20_hoistedGlobal[c20_i222 +
                c20_i219] * c20_b[c20_i223 + c20_i220];
              c20_i222 += 4;
            }

            c20_i220 += 4;
          }
        }

        for (c20_i224 = 0; c20_i224 < 16; c20_i224++) {
          c20_c_C[c20_i224] = c20_C[c20_i224];
        }

        for (c20_i225 = 0; c20_i225 < 16; c20_i225++) {
          c20_c_l_upper_leg_contact_H_b[c20_i225] =
            c20_l_upper_leg_contact_H_b[c20_i225];
        }

        c20_mrdivide(chartInstance, c20_c_C, c20_c_l_upper_leg_contact_H_b,
                     c20_dv12);
        for (c20_i226 = 0; c20_i226 < 16; c20_i226++) {
          chartInstance->c20_w_H_fixedLink[c20_i226] = c20_dv12[c20_i226];
        }
      } else {
        guard1 = TRUE;
      }
    } else {
      guard1 = TRUE;
    }

    if (guard1 == TRUE) {
      CV_SCRIPT_MCDC(0, 6, FALSE);
      CV_SCRIPT_IF(0, 14, FALSE);
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 166U);
  if (CV_SCRIPT_IF(0, 15, chartInstance->c20_state == 7.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 168U);
    for (c20_i227 = 0; c20_i227 < 16; c20_i227++) {
      c20_hoistedGlobal[c20_i227] = chartInstance->c20_w_H_fixedLink[c20_i227];
    }

    for (c20_i228 = 0; c20_i228 < 16; c20_i228++) {
      c20_b[c20_i228] = c20_l_upper_leg_contact_H_b[c20_i228];
    }

    c20_eml_scalar_eg(chartInstance);
    c20_eml_scalar_eg(chartInstance);
    for (c20_i229 = 0; c20_i229 < 16; c20_i229++) {
      c20_w_H_b[c20_i229] = 0.0;
    }

    for (c20_i230 = 0; c20_i230 < 16; c20_i230++) {
      c20_w_H_b[c20_i230] = 0.0;
    }

    for (c20_i231 = 0; c20_i231 < 16; c20_i231++) {
      c20_C[c20_i231] = c20_w_H_b[c20_i231];
    }

    for (c20_i232 = 0; c20_i232 < 16; c20_i232++) {
      c20_w_H_b[c20_i232] = c20_C[c20_i232];
    }

    for (c20_i233 = 0; c20_i233 < 16; c20_i233++) {
      c20_C[c20_i233] = c20_w_H_b[c20_i233];
    }

    for (c20_i234 = 0; c20_i234 < 16; c20_i234++) {
      c20_w_H_b[c20_i234] = c20_C[c20_i234];
    }

    for (c20_i235 = 0; c20_i235 < 4; c20_i235++) {
      c20_i236 = 0;
      for (c20_i237 = 0; c20_i237 < 4; c20_i237++) {
        c20_w_H_b[c20_i236 + c20_i235] = 0.0;
        c20_i238 = 0;
        for (c20_i239 = 0; c20_i239 < 4; c20_i239++) {
          c20_w_H_b[c20_i236 + c20_i235] += c20_hoistedGlobal[c20_i238 +
            c20_i235] * c20_b[c20_i239 + c20_i236];
          c20_i238 += 4;
        }

        c20_i236 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 171U);
    c20_i240 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i241 = 0; c20_i241 < 4; c20_i241++) {
      c20_qjDes[c20_iv0[c20_i241]] = c20_b_sm->joints.standUpPositions[c20_i240
        + (c20_i241 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 172U);
    c20_i242 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i243 = 0; c20_i243 < 4; c20_i243++) {
      c20_qjDes[c20_iv1[c20_i243]] = c20_b_sm->joints.standUpPositions[c20_i242
        + (c20_i243 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 173U);
    c20_i244 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i245 = 0; c20_i245 < 4; c20_i245++) {
      c20_qjDes[7 + c20_i245] = c20_b_sm->joints.standUpPositions[c20_i244 + ((4
        + c20_i245) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 174U);
    c20_i246 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i247 = 0; c20_i247 < 4; c20_i247++) {
      c20_qjDes[3 + c20_i247] = c20_b_sm->joints.standUpPositions[c20_i246 + ((4
        + c20_i247) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 175U);
    c20_qjDes[0] = c20_b_sm->joints.standUpPositions[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.standUpPositions", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8, 1, 0) + 63];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 177U);
    c20_i248 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.CoM.standUpDeltaCoM", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i249 = 0; c20_i249 < 3; c20_i249++) {
      c20_CoM_Des[c20_i249] = chartInstance->c20_CoMprevious[c20_i249] +
        c20_b_sm->CoM.standUpDeltaCoM[c20_i248 + (c20_i249 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 178U);
    *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes
      [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.jointsAndCoMSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 179U);
    c20_tDelta = c20_t - chartInstance->c20_tSwitch;
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 181U);
    if (CV_SCRIPT_IF(0, 16, c20_tDelta > 10.0)) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 182U);
      chartInstance->c20_state = 8.0;
      _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 183U);
      for (c20_i250 = 0; c20_i250 < 3; c20_i250++) {
        chartInstance->c20_CoMprevious[c20_i250] = c20_CoM[c20_i250];
      }
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 189U);
  if (CV_SCRIPT_IF(0, 17, chartInstance->c20_state == 8.0)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 191U);
    for (c20_i251 = 0; c20_i251 < 16; c20_i251++) {
      c20_hoistedGlobal[c20_i251] = chartInstance->c20_w_H_fixedLink[c20_i251];
    }

    for (c20_i252 = 0; c20_i252 < 16; c20_i252++) {
      c20_b[c20_i252] = c20_l_upper_leg_contact_H_b[c20_i252];
    }

    c20_eml_scalar_eg(chartInstance);
    c20_eml_scalar_eg(chartInstance);
    for (c20_i253 = 0; c20_i253 < 16; c20_i253++) {
      c20_w_H_b[c20_i253] = 0.0;
    }

    for (c20_i254 = 0; c20_i254 < 16; c20_i254++) {
      c20_w_H_b[c20_i254] = 0.0;
    }

    for (c20_i255 = 0; c20_i255 < 16; c20_i255++) {
      c20_C[c20_i255] = c20_w_H_b[c20_i255];
    }

    for (c20_i256 = 0; c20_i256 < 16; c20_i256++) {
      c20_w_H_b[c20_i256] = c20_C[c20_i256];
    }

    for (c20_i257 = 0; c20_i257 < 16; c20_i257++) {
      c20_C[c20_i257] = c20_w_H_b[c20_i257];
    }

    for (c20_i258 = 0; c20_i258 < 16; c20_i258++) {
      c20_w_H_b[c20_i258] = c20_C[c20_i258];
    }

    for (c20_i259 = 0; c20_i259 < 4; c20_i259++) {
      c20_i260 = 0;
      for (c20_i261 = 0; c20_i261 < 4; c20_i261++) {
        c20_w_H_b[c20_i260 + c20_i259] = 0.0;
        c20_i262 = 0;
        for (c20_i263 = 0; c20_i263 < 4; c20_i263++) {
          c20_w_H_b[c20_i260 + c20_i259] += c20_hoistedGlobal[c20_i262 +
            c20_i259] * c20_b[c20_i263 + c20_i260];
          c20_i262 += 4;
        }

        c20_i260 += 4;
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 194U);
    c20_i264 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i265 = 0; c20_i265 < 4; c20_i265++) {
      c20_qjDes[c20_iv0[c20_i265]] = c20_b_sm->joints.standUpPositions[c20_i264
        + (c20_i265 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 195U);
    c20_i266 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i267 = 0; c20_i267 < 4; c20_i267++) {
      c20_qjDes[c20_iv1[c20_i267]] = c20_b_sm->joints.standUpPositions[c20_i266
        + (c20_i267 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 196U);
    c20_i268 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i269 = 0; c20_i269 < 4; c20_i269++) {
      c20_qjDes[7 + c20_i269] = c20_b_sm->joints.standUpPositions[c20_i268 + ((4
        + c20_i269) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 197U);
    c20_i270 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.joints.standUpPositions", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i271 = 0; c20_i271 < 4; c20_i271++) {
      c20_qjDes[3 + c20_i271] = c20_b_sm->joints.standUpPositions[c20_i270 + ((4
        + c20_i271) << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 198U);
    c20_qjDes[0] = c20_b_sm->joints.standUpPositions[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("sm.joints.standUpPositions", (int32_T)
      _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 8, 1, 0) + 63];
    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 200U);
    c20_i272 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.CoM.standUpDeltaCoM", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1;
    for (c20_i273 = 0; c20_i273 < 3; c20_i273++) {
      c20_CoM_Des[c20_i273] = chartInstance->c20_CoMprevious[c20_i273] +
        c20_b_sm->CoM.standUpDeltaCoM[c20_i272 + (c20_i273 << 3)];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 201U);
    *c20_jointsAndCoMSmoothingTime = c20_b_sm->jointsAndCoMSmoothingTimes
      [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "sm.jointsAndCoMSmoothingTimes", (int32_T)_SFD_INTEGER_CHECK("state",
      chartInstance->c20_state), 1, 8, 1, 0) - 1];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 205U);
  *c20_currentState = chartInstance->c20_state;
  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 206U);
  (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.impedances", (int32_T)
    _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 1, 1, 0);
  for (c20_i274 = 0; c20_i274 < 23; c20_i274++) {
    c20_impedances[c20_i274] = c20_b_gain->impedances[c20_i274];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 207U);
  c20_i275 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.PCOM", (int32_T)
    _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 3, 1, 0) - 1;
  for (c20_i276 = 0; c20_i276 < 3; c20_i276++) {
    c20_kpCom[c20_i276] = c20_b_gain->PCOM[c20_i275 + 3 * c20_i276];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, 208U);
  c20_i277 = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("gain.DCOM", (int32_T)
    _SFD_INTEGER_CHECK("state", chartInstance->c20_state), 1, 3, 1, 0) - 1;
  for (c20_i278 = 0; c20_i278 < 3; c20_i278++) {
    c20_kdCom[c20_i278] = c20_b_gain->DCOM[c20_i277 + 3 * c20_i278];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c20_sfEvent, -208);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c20_machineNumber, uint32_T
  c20_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c20_chartNumber, 0U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m"));
}

static const mxArray *c20_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i279;
  real_T c20_b_inData[3];
  int32_T c20_i280;
  real_T c20_u[3];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  for (c20_i279 = 0; c20_i279 < 3; c20_i279++) {
    c20_b_inData[c20_i279] = (*(real_T (*)[3])c20_inData)[c20_i279];
  }

  for (c20_i280 = 0; c20_i280 < 3; c20_i280++) {
    c20_u[c20_i280] = c20_b_inData[c20_i280];
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_CoM_Des, const char_T *c20_identifier,
  real_T c20_y[3])
{
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_CoM_Des), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_CoM_Des);
}

static void c20_b_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[3])
{
  real_T c20_dv13[3];
  int32_T c20_i281;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv13, 1, 0, 0U, 1, 0U, 1, 3);
  for (c20_i281 = 0; c20_i281 < 3; c20_i281++) {
    c20_y[c20_i281] = c20_dv13[c20_i281];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_CoM_Des;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[3];
  int32_T c20_i282;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_CoM_Des = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_CoM_Des), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_CoM_Des);
  for (c20_i282 = 0; c20_i282 < 3; c20_i282++) {
    (*(real_T (*)[3])c20_outData)[c20_i282] = c20_y[c20_i282];
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_b_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i283;
  real_T c20_b_inData[23];
  int32_T c20_i284;
  real_T c20_u[23];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  for (c20_i283 = 0; c20_i283 < 23; c20_i283++) {
    c20_b_inData[c20_i283] = (*(real_T (*)[23])c20_inData)[c20_i283];
  }

  for (c20_i284 = 0; c20_i284 < 23; c20_i284++) {
    c20_u[c20_i284] = c20_b_inData[c20_i284];
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_c_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_qjDes, const char_T *c20_identifier, real_T
  c20_y[23])
{
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_qjDes), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_qjDes);
}

static void c20_d_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[23])
{
  real_T c20_dv14[23];
  int32_T c20_i285;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv14, 1, 0, 0U, 1, 0U, 1,
                23);
  for (c20_i285 = 0; c20_i285 < 23; c20_i285++) {
    c20_y[c20_i285] = c20_dv14[c20_i285];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_qjDes;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[23];
  int32_T c20_i286;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_qjDes = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_qjDes), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_qjDes);
  for (c20_i286 = 0; c20_i286 < 23; c20_i286++) {
    (*(real_T (*)[23])c20_outData)[c20_i286] = c20_y[c20_i286];
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_c_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  real_T c20_u;
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_u = *(real_T *)c20_inData;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", &c20_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static real_T c20_e_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_jointsSmoothingTime, const char_T
  *c20_identifier)
{
  real_T c20_y;
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c20_jointsSmoothingTime), &c20_thisId);
  sf_mex_destroy(&c20_jointsSmoothingTime);
  return c20_y;
}

static real_T c20_f_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  real_T c20_y;
  real_T c20_d0;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_d0, 1, 0, 0U, 0, 0U, 0);
  c20_y = c20_d0;
  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_jointsSmoothingTime;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_jointsSmoothingTime = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c20_jointsSmoothingTime), &c20_thisId);
  sf_mex_destroy(&c20_jointsSmoothingTime);
  *(real_T *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_d_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i287;
  real_T c20_b_inData[3];
  int32_T c20_i288;
  real_T c20_u[3];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  for (c20_i287 = 0; c20_i287 < 3; c20_i287++) {
    c20_b_inData[c20_i287] = (*(real_T (*)[3])c20_inData)[c20_i287];
  }

  for (c20_i288 = 0; c20_i288 < 3; c20_i288++) {
    c20_u[c20_i288] = c20_b_inData[c20_i288];
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 2, 1, 3), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_g_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_kdCom, const char_T *c20_identifier, real_T
  c20_y[3])
{
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_kdCom), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_kdCom);
}

static void c20_h_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[3])
{
  real_T c20_dv15[3];
  int32_T c20_i289;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv15, 1, 0, 0U, 1, 0U, 2, 1,
                3);
  for (c20_i289 = 0; c20_i289 < 3; c20_i289++) {
    c20_y[c20_i289] = c20_dv15[c20_i289];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_kdCom;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[3];
  int32_T c20_i290;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_kdCom = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_kdCom), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_kdCom);
  for (c20_i290 = 0; c20_i290 < 3; c20_i290++) {
    (*(real_T (*)[3])c20_outData)[c20_i290] = c20_y[c20_i290];
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_e_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i291;
  real_T c20_b_inData[23];
  int32_T c20_i292;
  real_T c20_u[23];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  for (c20_i291 = 0; c20_i291 < 23; c20_i291++) {
    c20_b_inData[c20_i291] = (*(real_T (*)[23])c20_inData)[c20_i291];
  }

  for (c20_i292 = 0; c20_i292 < 23; c20_i292++) {
    c20_u[c20_i292] = c20_b_inData[c20_i292];
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_i_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_impedances, const char_T *c20_identifier,
  real_T c20_y[23])
{
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_impedances), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_impedances);
}

static void c20_j_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[23])
{
  real_T c20_dv16[23];
  int32_T c20_i293;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv16, 1, 0, 0U, 1, 0U, 2, 1,
                23);
  for (c20_i293 = 0; c20_i293 < 23; c20_i293++) {
    c20_y[c20_i293] = c20_dv16[c20_i293];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_impedances;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[23];
  int32_T c20_i294;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_impedances = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_impedances), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_impedances);
  for (c20_i294 = 0; c20_i294 < 23; c20_i294++) {
    (*(real_T (*)[23])c20_outData)[c20_i294] = c20_y[c20_i294];
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_f_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i295;
  real_T c20_b_inData[2];
  int32_T c20_i296;
  real_T c20_u[2];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  for (c20_i295 = 0; c20_i295 < 2; c20_i295++) {
    c20_b_inData[c20_i295] = (*(real_T (*)[2])c20_inData)[c20_i295];
  }

  for (c20_i296 = 0; c20_i296 < 2; c20_i296++) {
    c20_u[c20_i296] = c20_b_inData[c20_i296];
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_k_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_constraints, const char_T *c20_identifier,
  real_T c20_y[2])
{
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_constraints), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_constraints);
}

static void c20_l_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[2])
{
  real_T c20_dv17[2];
  int32_T c20_i297;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv17, 1, 0, 0U, 1, 0U, 1, 2);
  for (c20_i297 = 0; c20_i297 < 2; c20_i297++) {
    c20_y[c20_i297] = c20_dv17[c20_i297];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_constraints;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[2];
  int32_T c20_i298;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_constraints = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_constraints), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_constraints);
  for (c20_i298 = 0; c20_i298 < 2; c20_i298++) {
    (*(real_T (*)[2])c20_outData)[c20_i298] = c20_y[c20_i298];
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_g_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i299;
  int32_T c20_i300;
  int32_T c20_i301;
  real_T c20_b_inData[16];
  int32_T c20_i302;
  int32_T c20_i303;
  int32_T c20_i304;
  real_T c20_u[16];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_i299 = 0;
  for (c20_i300 = 0; c20_i300 < 4; c20_i300++) {
    for (c20_i301 = 0; c20_i301 < 4; c20_i301++) {
      c20_b_inData[c20_i301 + c20_i299] = (*(real_T (*)[16])c20_inData)[c20_i301
        + c20_i299];
    }

    c20_i299 += 4;
  }

  c20_i302 = 0;
  for (c20_i303 = 0; c20_i303 < 4; c20_i303++) {
    for (c20_i304 = 0; c20_i304 < 4; c20_i304++) {
      c20_u[c20_i304 + c20_i302] = c20_b_inData[c20_i304 + c20_i302];
    }

    c20_i302 += 4;
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_m_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_w_H_b, const char_T *c20_identifier, real_T
  c20_y[16])
{
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_w_H_b), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_w_H_b);
}

static void c20_n_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[16])
{
  real_T c20_dv18[16];
  int32_T c20_i305;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv18, 1, 0, 0U, 1, 0U, 2, 4,
                4);
  for (c20_i305 = 0; c20_i305 < 16; c20_i305++) {
    c20_y[c20_i305] = c20_dv18[c20_i305];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_w_H_b;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[16];
  int32_T c20_i306;
  int32_T c20_i307;
  int32_T c20_i308;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_w_H_b = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_w_H_b), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_w_H_b);
  c20_i306 = 0;
  for (c20_i307 = 0; c20_i307 < 4; c20_i307++) {
    for (c20_i308 = 0; c20_i308 < 4; c20_i308++) {
      (*(real_T (*)[16])c20_outData)[c20_i308 + c20_i306] = c20_y[c20_i308 +
        c20_i306];
    }

    c20_i306 += 4;
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_h_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  boolean_T c20_u;
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_u = *(boolean_T *)c20_inData;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", &c20_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static const mxArray *c20_i_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i309;
  real_T c20_b_inData[6];
  int32_T c20_i310;
  real_T c20_u[6];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  for (c20_i309 = 0; c20_i309 < 6; c20_i309++) {
    c20_b_inData[c20_i309] = (*(real_T (*)[6])c20_inData)[c20_i309];
  }

  for (c20_i310 = 0; c20_i310 < 6; c20_i310++) {
    c20_u[c20_i310] = c20_b_inData[c20_i310];
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static const mxArray *c20_j_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData;
  c20_struct_rUGQ0INmvPpaxIctEGl5sE c20_u;
  const mxArray *c20_y = NULL;
  boolean_T c20_b_u;
  const mxArray *c20_b_y = NULL;
  boolean_T c20_c_u;
  const mxArray *c20_c_y = NULL;
  boolean_T c20_d_u;
  const mxArray *c20_d_y = NULL;
  boolean_T c20_e_u;
  const mxArray *c20_e_y = NULL;
  c20_struct_DnBdbfPNxiIjhNOyZMmfsE c20_f_u;
  const mxArray *c20_f_y = NULL;
  real_T c20_g_u;
  const mxArray *c20_g_y = NULL;
  int32_T c20_i311;
  real_T c20_h_u[39];
  const mxArray *c20_h_y = NULL;
  c20_struct_KJR2itYvhBuAkuR6dKZHUC c20_i_u;
  const mxArray *c20_i_y = NULL;
  real_T c20_j_u;
  const mxArray *c20_j_y = NULL;
  real_T c20_k_u;
  const mxArray *c20_k_y = NULL;
  c20_struct_0U0wBk2LiR1OqsMsUngxdD c20_l_u;
  const mxArray *c20_l_y = NULL;
  real_T c20_m_u;
  const mxArray *c20_m_y = NULL;
  real_T c20_n_u;
  const mxArray *c20_n_y = NULL;
  real_T c20_o_u;
  const mxArray *c20_o_y = NULL;
  real_T c20_p_u;
  const mxArray *c20_p_y = NULL;
  int32_T c20_i312;
  real_T c20_q_u[299];
  const mxArray *c20_q_y = NULL;
  int32_T c20_i313;
  real_T c20_r_u[192];
  const mxArray *c20_r_y = NULL;
  int32_T c20_i314;
  real_T c20_s_u[192];
  const mxArray *c20_s_y = NULL;
  int32_T c20_i315;
  real_T c20_t_u[72];
  const mxArray *c20_t_y = NULL;
  real_T c20_u_u;
  const mxArray *c20_u_y = NULL;
  real_T c20_v_u;
  const mxArray *c20_v_y = NULL;
  real_T c20_w_u;
  const mxArray *c20_w_y = NULL;
  int32_T c20_i316;
  real_T c20_x_u[13];
  const mxArray *c20_x_y = NULL;
  real_T c20_y_u;
  const mxArray *c20_y_y = NULL;
  boolean_T c20_ab_u;
  const mxArray *c20_ab_y = NULL;
  int32_T c20_i317;
  real_T c20_bb_u[8];
  const mxArray *c20_bb_y = NULL;
  c20_struct_9LpOi5JXaV67jTuay8hWaH c20_cb_u;
  const mxArray *c20_cb_y = NULL;
  int32_T c20_i318;
  real_T c20_db_u[24];
  const mxArray *c20_db_y = NULL;
  int32_T c20_i319;
  real_T c20_eb_u[8];
  const mxArray *c20_eb_y = NULL;
  int32_T c20_i320;
  real_T c20_fb_u[8];
  const mxArray *c20_fb_y = NULL;
  int32_T c20_i321;
  real_T c20_gb_u[8];
  const mxArray *c20_gb_y = NULL;
  int32_T c20_i322;
  real_T c20_hb_u[8];
  const mxArray *c20_hb_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_mxArrayOutData = NULL;
  c20_u = *(c20_struct_rUGQ0INmvPpaxIctEGl5sE *)c20_inData;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c20_b_u = c20_u.skipYoga;
  c20_b_y = NULL;
  sf_mex_assign(&c20_b_y, sf_mex_create("y", &c20_b_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_b_y, "skipYoga", "skipYoga", 0);
  c20_c_u = c20_u.demoOnlyRightFoot;
  c20_c_y = NULL;
  sf_mex_assign(&c20_c_y, sf_mex_create("y", &c20_c_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_c_y, "demoOnlyRightFoot", "demoOnlyRightFoot", 0);
  c20_d_u = c20_u.yogaAlsoOnRightFoot;
  c20_d_y = NULL;
  sf_mex_assign(&c20_d_y, sf_mex_create("y", &c20_d_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_d_y, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot",
                  0);
  c20_e_u = c20_u.yogaInLoop;
  c20_e_y = NULL;
  sf_mex_assign(&c20_e_y, sf_mex_create("y", &c20_e_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_e_y, "yogaInLoop", "yogaInLoop", 0);
  c20_f_u = c20_u.com;
  c20_f_y = NULL;
  sf_mex_assign(&c20_f_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c20_g_u = c20_f_u.threshold;
  c20_g_y = NULL;
  sf_mex_assign(&c20_g_y, sf_mex_create("y", &c20_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_f_y, c20_g_y, "threshold", "threshold", 0);
  for (c20_i311 = 0; c20_i311 < 39; c20_i311++) {
    c20_h_u[c20_i311] = c20_f_u.states[c20_i311];
  }

  c20_h_y = NULL;
  sf_mex_assign(&c20_h_y, sf_mex_create("y", c20_h_u, 0, 0U, 1U, 0U, 2, 13, 3),
                FALSE);
  sf_mex_addfield(c20_f_y, c20_h_y, "states", "states", 0);
  sf_mex_addfield(c20_y, c20_f_y, "com", "com", 0);
  c20_i_u = c20_u.wrench;
  c20_i_y = NULL;
  sf_mex_assign(&c20_i_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c20_j_u = c20_i_u.thresholdContactOn;
  c20_j_y = NULL;
  sf_mex_assign(&c20_j_y, sf_mex_create("y", &c20_j_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_i_y, c20_j_y, "thresholdContactOn", "thresholdContactOn",
                  0);
  c20_k_u = c20_i_u.thresholdContactOff;
  c20_k_y = NULL;
  sf_mex_assign(&c20_k_y, sf_mex_create("y", &c20_k_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_i_y, c20_k_y, "thresholdContactOff", "thresholdContactOff",
                  0);
  sf_mex_addfield(c20_y, c20_i_y, "wrench", "wrench", 0);
  c20_l_u = c20_u.joints;
  c20_l_y = NULL;
  sf_mex_assign(&c20_l_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c20_m_u = c20_l_u.thresholdNotInContact;
  c20_m_y = NULL;
  sf_mex_assign(&c20_m_y, sf_mex_create("y", &c20_m_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_l_y, c20_m_y, "thresholdNotInContact",
                  "thresholdNotInContact", 0);
  c20_n_u = c20_l_u.thresholdInContact;
  c20_n_y = NULL;
  sf_mex_assign(&c20_n_y, sf_mex_create("y", &c20_n_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_l_y, c20_n_y, "thresholdInContact", "thresholdInContact",
                  0);
  c20_o_u = c20_l_u.pauseTimeLastPostureL;
  c20_o_y = NULL;
  sf_mex_assign(&c20_o_y, sf_mex_create("y", &c20_o_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_l_y, c20_o_y, "pauseTimeLastPostureL",
                  "pauseTimeLastPostureL", 0);
  c20_p_u = c20_l_u.pauseTimeLastPostureR;
  c20_p_y = NULL;
  sf_mex_assign(&c20_p_y, sf_mex_create("y", &c20_p_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_l_y, c20_p_y, "pauseTimeLastPostureR",
                  "pauseTimeLastPostureR", 0);
  for (c20_i312 = 0; c20_i312 < 299; c20_i312++) {
    c20_q_u[c20_i312] = c20_l_u.states[c20_i312];
  }

  c20_q_y = NULL;
  sf_mex_assign(&c20_q_y, sf_mex_create("y", c20_q_u, 0, 0U, 1U, 0U, 2, 13, 23),
                FALSE);
  sf_mex_addfield(c20_l_y, c20_q_y, "states", "states", 0);
  for (c20_i313 = 0; c20_i313 < 192; c20_i313++) {
    c20_r_u[c20_i313] = c20_l_u.pointsL[c20_i313];
  }

  c20_r_y = NULL;
  sf_mex_assign(&c20_r_y, sf_mex_create("y", c20_r_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c20_l_y, c20_r_y, "pointsL", "pointsL", 0);
  for (c20_i314 = 0; c20_i314 < 192; c20_i314++) {
    c20_s_u[c20_i314] = c20_l_u.pointsR[c20_i314];
  }

  c20_s_y = NULL;
  sf_mex_assign(&c20_s_y, sf_mex_create("y", c20_s_u, 0, 0U, 1U, 0U, 2, 8, 24),
                FALSE);
  sf_mex_addfield(c20_l_y, c20_s_y, "pointsR", "pointsR", 0);
  for (c20_i315 = 0; c20_i315 < 72; c20_i315++) {
    c20_t_u[c20_i315] = c20_l_u.standUpPositions[c20_i315];
  }

  c20_t_y = NULL;
  sf_mex_assign(&c20_t_y, sf_mex_create("y", c20_t_u, 0, 0U, 1U, 0U, 2, 8, 9),
                FALSE);
  sf_mex_addfield(c20_l_y, c20_t_y, "standUpPositions", "standUpPositions", 0);
  sf_mex_addfield(c20_y, c20_l_y, "joints", "joints", 0);
  c20_u_u = c20_u.stateAt0;
  c20_u_y = NULL;
  sf_mex_assign(&c20_u_y, sf_mex_create("y", &c20_u_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_u_y, "stateAt0", "stateAt0", 0);
  c20_v_u = c20_u.DT;
  c20_v_y = NULL;
  sf_mex_assign(&c20_v_y, sf_mex_create("y", &c20_v_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_v_y, "DT", "DT", 0);
  c20_w_u = c20_u.waitingTimeAfterYoga;
  c20_w_y = NULL;
  sf_mex_assign(&c20_w_y, sf_mex_create("y", &c20_w_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_w_y, "waitingTimeAfterYoga", "waitingTimeAfterYoga",
                  0);
  for (c20_i316 = 0; c20_i316 < 13; c20_i316++) {
    c20_x_u[c20_i316] = c20_u.jointsSmoothingTimes[c20_i316];
  }

  c20_x_y = NULL;
  sf_mex_assign(&c20_x_y, sf_mex_create("y", c20_x_u, 0, 0U, 1U, 0U, 2, 13, 1),
                FALSE);
  sf_mex_addfield(c20_y, c20_x_y, "jointsSmoothingTimes", "jointsSmoothingTimes",
                  0);
  c20_y_u = c20_u.tBalancing;
  c20_y_y = NULL;
  sf_mex_assign(&c20_y_y, sf_mex_create("y", &c20_y_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_y_y, "tBalancing", "tBalancing", 0);
  c20_ab_u = c20_u.alsoSitDown;
  c20_ab_y = NULL;
  sf_mex_assign(&c20_ab_y, sf_mex_create("y", &c20_ab_u, 11, 0U, 0U, 0U, 0),
                FALSE);
  sf_mex_addfield(c20_y, c20_ab_y, "alsoSitDown", "alsoSitDown", 0);
  for (c20_i317 = 0; c20_i317 < 8; c20_i317++) {
    c20_bb_u[c20_i317] = c20_u.jointsAndCoMSmoothingTimes[c20_i317];
  }

  c20_bb_y = NULL;
  sf_mex_assign(&c20_bb_y, sf_mex_create("y", c20_bb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c20_y, c20_bb_y, "jointsAndCoMSmoothingTimes",
                  "jointsAndCoMSmoothingTimes", 0);
  c20_cb_u = c20_u.CoM;
  c20_cb_y = NULL;
  sf_mex_assign(&c20_cb_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  for (c20_i318 = 0; c20_i318 < 24; c20_i318++) {
    c20_db_u[c20_i318] = c20_cb_u.standUpDeltaCoM[c20_i318];
  }

  c20_db_y = NULL;
  sf_mex_assign(&c20_db_y, sf_mex_create("y", c20_db_u, 0, 0U, 1U, 0U, 2, 8, 3),
                FALSE);
  sf_mex_addfield(c20_cb_y, c20_db_y, "standUpDeltaCoM", "standUpDeltaCoM", 0);
  sf_mex_addfield(c20_y, c20_cb_y, "CoM", "CoM", 0);
  for (c20_i319 = 0; c20_i319 < 8; c20_i319++) {
    c20_eb_u[c20_i319] = c20_u.LwrenchThreshold[c20_i319];
  }

  c20_eb_y = NULL;
  sf_mex_assign(&c20_eb_y, sf_mex_create("y", c20_eb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c20_y, c20_eb_y, "LwrenchThreshold", "LwrenchThreshold", 0);
  for (c20_i320 = 0; c20_i320 < 8; c20_i320++) {
    c20_fb_u[c20_i320] = c20_u.RwrenchThreshold[c20_i320];
  }

  c20_fb_y = NULL;
  sf_mex_assign(&c20_fb_y, sf_mex_create("y", c20_fb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c20_y, c20_fb_y, "RwrenchThreshold", "RwrenchThreshold", 0);
  for (c20_i321 = 0; c20_i321 < 8; c20_i321++) {
    c20_gb_u[c20_i321] = c20_u.RArmThreshold[c20_i321];
  }

  c20_gb_y = NULL;
  sf_mex_assign(&c20_gb_y, sf_mex_create("y", c20_gb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c20_y, c20_gb_y, "RArmThreshold", "RArmThreshold", 0);
  for (c20_i322 = 0; c20_i322 < 8; c20_i322++) {
    c20_hb_u[c20_i322] = c20_u.LArmThreshold[c20_i322];
  }

  c20_hb_y = NULL;
  sf_mex_assign(&c20_hb_y, sf_mex_create("y", c20_hb_u, 0, 0U, 1U, 0U, 2, 8, 1),
                FALSE);
  sf_mex_addfield(c20_y, c20_hb_y, "LArmThreshold", "LArmThreshold", 0);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_o_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_rUGQ0INmvPpaxIctEGl5sE *c20_y)
{
  emlrtMsgIdentifier c20_thisId;
  static const char * c20_fieldNames[19] = { "skipYoga", "demoOnlyRightFoot",
    "yogaAlsoOnRightFoot", "yogaInLoop", "com", "wrench", "joints", "stateAt0",
    "DT", "waitingTimeAfterYoga", "jointsSmoothingTimes", "tBalancing",
    "alsoSitDown", "jointsAndCoMSmoothingTimes", "CoM", "LwrenchThreshold",
    "RwrenchThreshold", "RArmThreshold", "LArmThreshold" };

  c20_thisId.fParent = c20_parentId;
  sf_mex_check_struct(c20_parentId, c20_u, 19, c20_fieldNames, 0U, 0);
  c20_thisId.fIdentifier = "skipYoga";
  c20_y->skipYoga = c20_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "skipYoga", "skipYoga", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "demoOnlyRightFoot";
  c20_y->demoOnlyRightFoot = c20_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "demoOnlyRightFoot", "demoOnlyRightFoot", 0)),
    &c20_thisId);
  c20_thisId.fIdentifier = "yogaAlsoOnRightFoot";
  c20_y->yogaAlsoOnRightFoot = c20_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "yogaAlsoOnRightFoot", "yogaAlsoOnRightFoot", 0)),
    &c20_thisId);
  c20_thisId.fIdentifier = "yogaInLoop";
  c20_y->yogaInLoop = c20_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "yogaInLoop", "yogaInLoop", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "com";
  c20_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u, "com",
    "com", 0)), &c20_thisId, &c20_y->com);
  c20_thisId.fIdentifier = "wrench";
  c20_y->wrench = c20_s_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "wrench", "wrench", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "joints";
  c20_t_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "joints", "joints", 0)), &c20_thisId, &c20_y->joints);
  c20_thisId.fIdentifier = "stateAt0";
  c20_y->stateAt0 = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "stateAt0", "stateAt0", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "DT";
  c20_y->DT = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c20_u, "DT", "DT", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "waitingTimeAfterYoga";
  c20_y->waitingTimeAfterYoga = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "waitingTimeAfterYoga", "waitingTimeAfterYoga", 0)),
    &c20_thisId);
  c20_thisId.fIdentifier = "jointsSmoothingTimes";
  c20_x_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "jointsSmoothingTimes", "jointsSmoothingTimes", 0)), &c20_thisId,
    c20_y->jointsSmoothingTimes);
  c20_thisId.fIdentifier = "tBalancing";
  c20_y->tBalancing = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "tBalancing", "tBalancing", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "alsoSitDown";
  c20_y->alsoSitDown = c20_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "alsoSitDown", "alsoSitDown", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "jointsAndCoMSmoothingTimes";
  c20_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "jointsAndCoMSmoothingTimes", "jointsAndCoMSmoothingTimes", 0)), &c20_thisId,
    c20_y->jointsAndCoMSmoothingTimes);
  c20_thisId.fIdentifier = "CoM";
  c20_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u, "CoM",
    "CoM", 0)), &c20_thisId, &c20_y->CoM);
  c20_thisId.fIdentifier = "LwrenchThreshold";
  c20_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "LwrenchThreshold", "LwrenchThreshold", 0)), &c20_thisId,
    c20_y->LwrenchThreshold);
  c20_thisId.fIdentifier = "RwrenchThreshold";
  c20_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "RwrenchThreshold", "RwrenchThreshold", 0)), &c20_thisId,
    c20_y->RwrenchThreshold);
  c20_thisId.fIdentifier = "RArmThreshold";
  c20_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "RArmThreshold", "RArmThreshold", 0)), &c20_thisId, c20_y->RArmThreshold);
  c20_thisId.fIdentifier = "LArmThreshold";
  c20_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "LArmThreshold", "LArmThreshold", 0)), &c20_thisId, c20_y->LArmThreshold);
  sf_mex_destroy(&c20_u);
}

static boolean_T c20_p_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  boolean_T c20_y;
  boolean_T c20_b0;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_b0, 1, 11, 0U, 0, 0U, 0);
  c20_y = c20_b0;
  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_q_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_DnBdbfPNxiIjhNOyZMmfsE *c20_y)
{
  emlrtMsgIdentifier c20_thisId;
  static const char * c20_fieldNames[2] = { "threshold", "states" };

  c20_thisId.fParent = c20_parentId;
  sf_mex_check_struct(c20_parentId, c20_u, 2, c20_fieldNames, 0U, 0);
  c20_thisId.fIdentifier = "threshold";
  c20_y->threshold = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "threshold", "threshold", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "states";
  c20_r_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "states", "states", 0)), &c20_thisId, c20_y->states);
  sf_mex_destroy(&c20_u);
}

static void c20_r_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[39])
{
  real_T c20_dv19[39];
  int32_T c20_i323;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv19, 1, 0, 0U, 1, 0U, 2,
                13, 3);
  for (c20_i323 = 0; c20_i323 < 39; c20_i323++) {
    c20_y[c20_i323] = c20_dv19[c20_i323];
  }

  sf_mex_destroy(&c20_u);
}

static c20_struct_KJR2itYvhBuAkuR6dKZHUC c20_s_emlrt_marshallIn
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c20_u,
   const emlrtMsgIdentifier *c20_parentId)
{
  c20_struct_KJR2itYvhBuAkuR6dKZHUC c20_y;
  emlrtMsgIdentifier c20_thisId;
  static const char * c20_fieldNames[2] = { "thresholdContactOn",
    "thresholdContactOff" };

  c20_thisId.fParent = c20_parentId;
  sf_mex_check_struct(c20_parentId, c20_u, 2, c20_fieldNames, 0U, 0);
  c20_thisId.fIdentifier = "thresholdContactOn";
  c20_y.thresholdContactOn = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "thresholdContactOn", "thresholdContactOn", 0)),
    &c20_thisId);
  c20_thisId.fIdentifier = "thresholdContactOff";
  c20_y.thresholdContactOff = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "thresholdContactOff", "thresholdContactOff", 0)),
    &c20_thisId);
  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_t_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_0U0wBk2LiR1OqsMsUngxdD *c20_y)
{
  emlrtMsgIdentifier c20_thisId;
  static const char * c20_fieldNames[8] = { "thresholdNotInContact",
    "thresholdInContact", "pauseTimeLastPostureL", "pauseTimeLastPostureR",
    "states", "pointsL", "pointsR", "standUpPositions" };

  c20_thisId.fParent = c20_parentId;
  sf_mex_check_struct(c20_parentId, c20_u, 8, c20_fieldNames, 0U, 0);
  c20_thisId.fIdentifier = "thresholdNotInContact";
  c20_y->thresholdNotInContact = c20_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c20_u, "thresholdNotInContact",
    "thresholdNotInContact", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "thresholdInContact";
  c20_y->thresholdInContact = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "thresholdInContact", "thresholdInContact", 0)),
    &c20_thisId);
  c20_thisId.fIdentifier = "pauseTimeLastPostureL";
  c20_y->pauseTimeLastPostureL = c20_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c20_u, "pauseTimeLastPostureL",
    "pauseTimeLastPostureL", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "pauseTimeLastPostureR";
  c20_y->pauseTimeLastPostureR = c20_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c20_u, "pauseTimeLastPostureR",
    "pauseTimeLastPostureR", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "states";
  c20_u_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "states", "states", 0)), &c20_thisId, c20_y->states);
  c20_thisId.fIdentifier = "pointsL";
  c20_v_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "pointsL", "pointsL", 0)), &c20_thisId, c20_y->pointsL);
  c20_thisId.fIdentifier = "pointsR";
  c20_v_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "pointsR", "pointsR", 0)), &c20_thisId, c20_y->pointsR);
  c20_thisId.fIdentifier = "standUpPositions";
  c20_w_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "standUpPositions", "standUpPositions", 0)), &c20_thisId,
    c20_y->standUpPositions);
  sf_mex_destroy(&c20_u);
}

static void c20_u_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[299])
{
  real_T c20_dv20[299];
  int32_T c20_i324;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv20, 1, 0, 0U, 1, 0U, 2,
                13, 23);
  for (c20_i324 = 0; c20_i324 < 299; c20_i324++) {
    c20_y[c20_i324] = c20_dv20[c20_i324];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_v_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[192])
{
  real_T c20_dv21[192];
  int32_T c20_i325;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv21, 1, 0, 0U, 1, 0U, 2, 8,
                24);
  for (c20_i325 = 0; c20_i325 < 192; c20_i325++) {
    c20_y[c20_i325] = c20_dv21[c20_i325];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_w_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[72])
{
  real_T c20_dv22[72];
  int32_T c20_i326;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv22, 1, 0, 0U, 1, 0U, 2, 8,
                9);
  for (c20_i326 = 0; c20_i326 < 72; c20_i326++) {
    c20_y[c20_i326] = c20_dv22[c20_i326];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_x_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[13])
{
  real_T c20_dv23[13];
  int32_T c20_i327;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv23, 1, 0, 0U, 1, 0U, 2,
                13, 1);
  for (c20_i327 = 0; c20_i327 < 13; c20_i327++) {
    c20_y[c20_i327] = c20_dv23[c20_i327];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_y_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[8])
{
  real_T c20_dv24[8];
  int32_T c20_i328;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv24, 1, 0, 0U, 1, 0U, 2, 8,
                1);
  for (c20_i328 = 0; c20_i328 < 8; c20_i328++) {
    c20_y[c20_i328] = c20_dv24[c20_i328];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_ab_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_9LpOi5JXaV67jTuay8hWaH *c20_y)
{
  emlrtMsgIdentifier c20_thisId;
  static const char * c20_fieldNames[1] = { "standUpDeltaCoM" };

  c20_thisId.fParent = c20_parentId;
  sf_mex_check_struct(c20_parentId, c20_u, 1, c20_fieldNames, 0U, 0);
  c20_thisId.fIdentifier = "standUpDeltaCoM";
  c20_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "standUpDeltaCoM", "standUpDeltaCoM", 0)), &c20_thisId,
    c20_y->standUpDeltaCoM);
  sf_mex_destroy(&c20_u);
}

static void c20_bb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[24])
{
  real_T c20_dv25[24];
  int32_T c20_i329;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv25, 1, 0, 0U, 1, 0U, 2, 8,
                3);
  for (c20_i329 = 0; c20_i329 < 24; c20_i329++) {
    c20_y[c20_i329] = c20_dv25[c20_i329];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_sm;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  c20_struct_rUGQ0INmvPpaxIctEGl5sE c20_y;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_b_sm = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_sm), &c20_thisId,
    &c20_y);
  sf_mex_destroy(&c20_b_sm);
  *(c20_struct_rUGQ0INmvPpaxIctEGl5sE *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_k_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  c20_struct_kzTB0QQWoOlMoMhgKf6sK c20_u;
  const mxArray *c20_y = NULL;
  real_T c20_b_u;
  const mxArray *c20_b_y = NULL;
  real_T c20_c_u;
  const mxArray *c20_c_y = NULL;
  real_T c20_d_u;
  const mxArray *c20_d_y = NULL;
  int32_T c20_i330;
  real_T c20_e_u[9];
  const mxArray *c20_e_y = NULL;
  int32_T c20_i331;
  real_T c20_f_u[9];
  const mxArray *c20_f_y = NULL;
  int32_T c20_i332;
  real_T c20_g_u[9];
  const mxArray *c20_g_y = NULL;
  real_T c20_h_u;
  const mxArray *c20_h_y = NULL;
  real_T c20_i_u;
  const mxArray *c20_i_y = NULL;
  int32_T c20_i333;
  real_T c20_j_u[23];
  const mxArray *c20_j_y = NULL;
  int32_T c20_i334;
  real_T c20_k_u[23];
  const mxArray *c20_k_y = NULL;
  int32_T c20_i335;
  real_T c20_l_u[23];
  const mxArray *c20_l_y = NULL;
  int32_T c20_i336;
  real_T c20_m_u[23];
  const mxArray *c20_m_y = NULL;
  int32_T c20_i337;
  real_T c20_n_u[4];
  const mxArray *c20_n_y = NULL;
  int32_T c20_i338;
  real_T c20_o_u[4];
  const mxArray *c20_o_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_u = *(c20_struct_kzTB0QQWoOlMoMhgKf6sK *)c20_inData;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c20_b_u = c20_u.qTildeMax;
  c20_b_y = NULL;
  sf_mex_assign(&c20_b_y, sf_mex_create("y", &c20_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_b_y, "qTildeMax", "qTildeMax", 0);
  c20_c_u = c20_u.SmoothingTimeImp;
  c20_c_y = NULL;
  sf_mex_assign(&c20_c_y, sf_mex_create("y", &c20_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_c_y, "SmoothingTimeImp", "SmoothingTimeImp", 0);
  c20_d_u = c20_u.SmoothingTimeGainScheduling;
  c20_d_y = NULL;
  sf_mex_assign(&c20_d_y, sf_mex_create("y", &c20_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_d_y, "SmoothingTimeGainScheduling",
                  "SmoothingTimeGainScheduling", 0);
  for (c20_i330 = 0; c20_i330 < 9; c20_i330++) {
    c20_e_u[c20_i330] = c20_u.PCOM[c20_i330];
  }

  c20_e_y = NULL;
  sf_mex_assign(&c20_e_y, sf_mex_create("y", c20_e_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c20_y, c20_e_y, "PCOM", "PCOM", 0);
  for (c20_i331 = 0; c20_i331 < 9; c20_i331++) {
    c20_f_u[c20_i331] = c20_u.ICOM[c20_i331];
  }

  c20_f_y = NULL;
  sf_mex_assign(&c20_f_y, sf_mex_create("y", c20_f_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c20_y, c20_f_y, "ICOM", "ICOM", 0);
  for (c20_i332 = 0; c20_i332 < 9; c20_i332++) {
    c20_g_u[c20_i332] = c20_u.DCOM[c20_i332];
  }

  c20_g_y = NULL;
  sf_mex_assign(&c20_g_y, sf_mex_create("y", c20_g_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c20_y, c20_g_y, "DCOM", "DCOM", 0);
  c20_h_u = c20_u.PAngularMomentum;
  c20_h_y = NULL;
  sf_mex_assign(&c20_h_y, sf_mex_create("y", &c20_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_h_y, "PAngularMomentum", "PAngularMomentum", 0);
  c20_i_u = c20_u.DAngularMomentum;
  c20_i_y = NULL;
  sf_mex_assign(&c20_i_y, sf_mex_create("y", &c20_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c20_y, c20_i_y, "DAngularMomentum", "DAngularMomentum", 0);
  for (c20_i333 = 0; c20_i333 < 23; c20_i333++) {
    c20_j_u[c20_i333] = c20_u.integral[c20_i333];
  }

  c20_j_y = NULL;
  sf_mex_assign(&c20_j_y, sf_mex_create("y", c20_j_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c20_y, c20_j_y, "integral", "integral", 0);
  for (c20_i334 = 0; c20_i334 < 23; c20_i334++) {
    c20_k_u[c20_i334] = c20_u.impedances[c20_i334];
  }

  c20_k_y = NULL;
  sf_mex_assign(&c20_k_y, sf_mex_create("y", c20_k_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c20_y, c20_k_y, "impedances", "impedances", 0);
  for (c20_i335 = 0; c20_i335 < 23; c20_i335++) {
    c20_l_u[c20_i335] = c20_u.dampings[c20_i335];
  }

  c20_l_y = NULL;
  sf_mex_assign(&c20_l_y, sf_mex_create("y", c20_l_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c20_y, c20_l_y, "dampings", "dampings", 0);
  for (c20_i336 = 0; c20_i336 < 23; c20_i336++) {
    c20_m_u[c20_i336] = c20_u.increasingRatesImp[c20_i336];
  }

  c20_m_y = NULL;
  sf_mex_assign(&c20_m_y, sf_mex_create("y", c20_m_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c20_y, c20_m_y, "increasingRatesImp", "increasingRatesImp", 0);
  for (c20_i337 = 0; c20_i337 < 4; c20_i337++) {
    c20_n_u[c20_i337] = c20_u.footSize[c20_i337];
  }

  c20_n_y = NULL;
  sf_mex_assign(&c20_n_y, sf_mex_create("y", c20_n_u, 0, 0U, 1U, 0U, 2, 2, 2),
                FALSE);
  sf_mex_addfield(c20_y, c20_n_y, "footSize", "footSize", 0);
  for (c20_i338 = 0; c20_i338 < 4; c20_i338++) {
    c20_o_u[c20_i338] = c20_u.legSize[c20_i338];
  }

  c20_o_y = NULL;
  sf_mex_assign(&c20_o_y, sf_mex_create("y", c20_o_u, 0, 0U, 1U, 0U, 2, 2, 2),
                FALSE);
  sf_mex_addfield(c20_y, c20_o_y, "legSize", "legSize", 0);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_cb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  c20_struct_kzTB0QQWoOlMoMhgKf6sK *c20_y)
{
  emlrtMsgIdentifier c20_thisId;
  static const char * c20_fieldNames[14] = { "qTildeMax", "SmoothingTimeImp",
    "SmoothingTimeGainScheduling", "PCOM", "ICOM", "DCOM", "PAngularMomentum",
    "DAngularMomentum", "integral", "impedances", "dampings",
    "increasingRatesImp", "footSize", "legSize" };

  c20_thisId.fParent = c20_parentId;
  sf_mex_check_struct(c20_parentId, c20_u, 14, c20_fieldNames, 0U, 0);
  c20_thisId.fIdentifier = "qTildeMax";
  c20_y->qTildeMax = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "qTildeMax", "qTildeMax", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "SmoothingTimeImp";
  c20_y->SmoothingTimeImp = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "SmoothingTimeImp", "SmoothingTimeImp", 0)),
    &c20_thisId);
  c20_thisId.fIdentifier = "SmoothingTimeGainScheduling";
  c20_y->SmoothingTimeGainScheduling = c20_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c20_u, "SmoothingTimeGainScheduling",
    "SmoothingTimeGainScheduling", 0)), &c20_thisId);
  c20_thisId.fIdentifier = "PCOM";
  c20_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "PCOM", "PCOM", 0)), &c20_thisId, c20_y->PCOM);
  c20_thisId.fIdentifier = "ICOM";
  c20_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "ICOM", "ICOM", 0)), &c20_thisId, c20_y->ICOM);
  c20_thisId.fIdentifier = "DCOM";
  c20_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "DCOM", "DCOM", 0)), &c20_thisId, c20_y->DCOM);
  c20_thisId.fIdentifier = "PAngularMomentum";
  c20_y->PAngularMomentum = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "PAngularMomentum", "PAngularMomentum", 0)),
    &c20_thisId);
  c20_thisId.fIdentifier = "DAngularMomentum";
  c20_y->DAngularMomentum = c20_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c20_u, "DAngularMomentum", "DAngularMomentum", 0)),
    &c20_thisId);
  c20_thisId.fIdentifier = "integral";
  c20_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "integral", "integral", 0)), &c20_thisId, c20_y->integral);
  c20_thisId.fIdentifier = "impedances";
  c20_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "impedances", "impedances", 0)), &c20_thisId, c20_y->impedances);
  c20_thisId.fIdentifier = "dampings";
  c20_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "dampings", "dampings", 0)), &c20_thisId, c20_y->dampings);
  c20_thisId.fIdentifier = "increasingRatesImp";
  c20_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "increasingRatesImp", "increasingRatesImp", 0)), &c20_thisId,
    c20_y->increasingRatesImp);
  c20_thisId.fIdentifier = "footSize";
  c20_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "footSize", "footSize", 0)), &c20_thisId, c20_y->footSize);
  c20_thisId.fIdentifier = "legSize";
  c20_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c20_u,
    "legSize", "legSize", 0)), &c20_thisId, c20_y->legSize);
  sf_mex_destroy(&c20_u);
}

static void c20_db_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[9])
{
  real_T c20_dv26[9];
  int32_T c20_i339;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv26, 1, 0, 0U, 1, 0U, 2, 3,
                3);
  for (c20_i339 = 0; c20_i339 < 9; c20_i339++) {
    c20_y[c20_i339] = c20_dv26[c20_i339];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_eb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[4])
{
  real_T c20_dv27[4];
  int32_T c20_i340;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv27, 1, 0, 0U, 1, 0U, 2, 2,
                2);
  for (c20_i340 = 0; c20_i340 < 4; c20_i340++) {
    c20_y[c20_i340] = c20_dv27[c20_i340];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_gain;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  c20_struct_kzTB0QQWoOlMoMhgKf6sK c20_y;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_b_gain = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_gain), &c20_thisId,
    &c20_y);
  sf_mex_destroy(&c20_b_gain);
  *(c20_struct_kzTB0QQWoOlMoMhgKf6sK *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_l_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i341;
  real_T c20_b_inData[3];
  int32_T c20_i342;
  real_T c20_u[3];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  for (c20_i341 = 0; c20_i341 < 3; c20_i341++) {
    c20_b_inData[c20_i341] = (*(real_T (*)[3])c20_inData)[c20_i341];
  }

  for (c20_i342 = 0; c20_i342 < 3; c20_i342++) {
    c20_u[c20_i342] = c20_b_inData[c20_i342];
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 2, 3, 1), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static const mxArray *c20_m_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i343;
  real_T c20_b_inData[3];
  int32_T c20_i344;
  real_T c20_u[3];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  for (c20_i343 = 0; c20_i343 < 3; c20_i343++) {
    c20_b_inData[c20_i343] = (*(real_T (*)[3])c20_inData)[c20_i343];
  }

  for (c20_i344 = 0; c20_i344 < 3; c20_i344++) {
    c20_u[c20_i344] = c20_b_inData[c20_i344];
  }

  c20_y = NULL;
  if (!chartInstance->c20_CoMprevious_not_empty) {
    sf_mex_assign(&c20_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  }

  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_fb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_b_CoMprevious, const char_T *c20_identifier,
  real_T c20_y[3])
{
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_CoMprevious),
    &c20_thisId, c20_y);
  sf_mex_destroy(&c20_b_CoMprevious);
}

static void c20_gb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[3])
{
  real_T c20_dv28[3];
  int32_T c20_i345;
  if (mxIsEmpty(c20_u)) {
    chartInstance->c20_CoMprevious_not_empty = FALSE;
  } else {
    chartInstance->c20_CoMprevious_not_empty = TRUE;
    sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv28, 1, 0, 0U, 1, 0U, 1,
                  3);
    for (c20_i345 = 0; c20_i345 < 3; c20_i345++) {
      c20_y[c20_i345] = c20_dv28[c20_i345];
    }
  }

  sf_mex_destroy(&c20_u);
}

static void c20_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_CoMprevious;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[3];
  int32_T c20_i346;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_b_CoMprevious = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_CoMprevious),
    &c20_thisId, c20_y);
  sf_mex_destroy(&c20_b_CoMprevious);
  for (c20_i346 = 0; c20_i346 < 3; c20_i346++) {
    (*(real_T (*)[3])c20_outData)[c20_i346] = c20_y[c20_i346];
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_n_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_i347;
  int32_T c20_i348;
  int32_T c20_i349;
  real_T c20_b_inData[16];
  int32_T c20_i350;
  int32_T c20_i351;
  int32_T c20_i352;
  real_T c20_u[16];
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_i347 = 0;
  for (c20_i348 = 0; c20_i348 < 4; c20_i348++) {
    for (c20_i349 = 0; c20_i349 < 4; c20_i349++) {
      c20_b_inData[c20_i349 + c20_i347] = (*(real_T (*)[16])c20_inData)[c20_i349
        + c20_i347];
    }

    c20_i347 += 4;
  }

  c20_i350 = 0;
  for (c20_i351 = 0; c20_i351 < 4; c20_i351++) {
    for (c20_i352 = 0; c20_i352 < 4; c20_i352++) {
      c20_u[c20_i352 + c20_i350] = c20_b_inData[c20_i352 + c20_i350];
    }

    c20_i350 += 4;
  }

  c20_y = NULL;
  if (!chartInstance->c20_w_H_fixedLink_not_empty) {
    sf_mex_assign(&c20_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 0, 0U, 1U, 0U, 2, 4, 4),
                  FALSE);
  }

  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static void c20_hb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_b_w_H_fixedLink, const char_T
  *c20_identifier, real_T c20_y[16])
{
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_w_H_fixedLink),
    &c20_thisId, c20_y);
  sf_mex_destroy(&c20_b_w_H_fixedLink);
}

static void c20_ib_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[16])
{
  real_T c20_dv29[16];
  int32_T c20_i353;
  if (mxIsEmpty(c20_u)) {
    chartInstance->c20_w_H_fixedLink_not_empty = FALSE;
  } else {
    chartInstance->c20_w_H_fixedLink_not_empty = TRUE;
    sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv29, 1, 0, 0U, 1, 0U, 2,
                  4, 4);
    for (c20_i353 = 0; c20_i353 < 16; c20_i353++) {
      c20_y[c20_i353] = c20_dv29[c20_i353];
    }
  }

  sf_mex_destroy(&c20_u);
}

static void c20_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_w_H_fixedLink;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[16];
  int32_T c20_i354;
  int32_T c20_i355;
  int32_T c20_i356;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_b_w_H_fixedLink = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_w_H_fixedLink),
    &c20_thisId, c20_y);
  sf_mex_destroy(&c20_b_w_H_fixedLink);
  c20_i354 = 0;
  for (c20_i355 = 0; c20_i355 < 4; c20_i355++) {
    for (c20_i356 = 0; c20_i356 < 4; c20_i356++) {
      (*(real_T (*)[16])c20_outData)[c20_i356 + c20_i354] = c20_y[c20_i356 +
        c20_i354];
    }

    c20_i354 += 4;
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_o_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  real_T c20_u;
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_u = *(real_T *)c20_inData;
  c20_y = NULL;
  if (!chartInstance->c20_tSwitch_not_empty) {
    sf_mex_assign(&c20_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c20_y, sf_mex_create("y", &c20_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static real_T c20_jb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_b_tSwitch, const char_T *c20_identifier)
{
  real_T c20_y;
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_kb_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_tSwitch),
    &c20_thisId);
  sf_mex_destroy(&c20_b_tSwitch);
  return c20_y;
}

static real_T c20_kb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  real_T c20_y;
  real_T c20_d1;
  if (mxIsEmpty(c20_u)) {
    chartInstance->c20_tSwitch_not_empty = FALSE;
  } else {
    chartInstance->c20_tSwitch_not_empty = TRUE;
    sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_d1, 1, 0, 0U, 0, 0U, 0);
    c20_y = c20_d1;
  }

  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_tSwitch;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_b_tSwitch = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_kb_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_tSwitch),
    &c20_thisId);
  sf_mex_destroy(&c20_b_tSwitch);
  *(real_T *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

static const mxArray *c20_p_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  real_T c20_u;
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_u = *(real_T *)c20_inData;
  c20_y = NULL;
  if (!chartInstance->c20_state_not_empty) {
    sf_mex_assign(&c20_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c20_y, sf_mex_create("y", &c20_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static real_T c20_lb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_b_state, const char_T *c20_identifier)
{
  real_T c20_y;
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_mb_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_state),
    &c20_thisId);
  sf_mex_destroy(&c20_b_state);
  return c20_y;
}

static real_T c20_mb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  real_T c20_y;
  real_T c20_d2;
  if (mxIsEmpty(c20_u)) {
    chartInstance->c20_state_not_empty = FALSE;
  } else {
    chartInstance->c20_state_not_empty = TRUE;
    sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_d2, 1, 0, 0U, 0, 0U, 0);
    c20_y = c20_d2;
  }

  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_state;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_b_state = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_mb_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_state),
    &c20_thisId);
  sf_mex_destroy(&c20_b_state);
  *(real_T *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

static void c20_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_useExtArmForces;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  boolean_T c20_y;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_useExtArmForces = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_useExtArmForces),
    &c20_thisId);
  sf_mex_destroy(&c20_useExtArmForces);
  *(boolean_T *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

static void c20_nb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId,
  real_T c20_y[6])
{
  real_T c20_dv30[6];
  int32_T c20_i357;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), c20_dv30, 1, 0, 0U, 1, 0U, 1, 6);
  for (c20_i357 = 0; c20_i357 < 6; c20_i357++) {
    c20_y[c20_i357] = c20_dv30[c20_i357];
  }

  sf_mex_destroy(&c20_u);
}

static void c20_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_RArmWrench;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  real_T c20_y[6];
  int32_T c20_i358;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_RArmWrench = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_nb_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_RArmWrench), &c20_thisId,
    c20_y);
  sf_mex_destroy(&c20_RArmWrench);
  for (c20_i358 = 0; c20_i358 < 6; c20_i358++) {
    (*(real_T (*)[6])c20_outData)[c20_i358] = c20_y[c20_i358];
  }

  sf_mex_destroy(&c20_mxArrayInData);
}

const mxArray *sf_c20_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c20_nameCaptureInfo;
  c20_ResolvedFunctionInfo c20_info[127];
  const mxArray *c20_m6 = NULL;
  int32_T c20_i359;
  c20_ResolvedFunctionInfo *c20_r2;
  c20_nameCaptureInfo = NULL;
  c20_nameCaptureInfo = NULL;
  c20_info_helper(c20_info);
  c20_b_info_helper(c20_info);
  sf_mex_assign(&c20_m6, sf_mex_createstruct("nameCaptureInfo", 1, 127), FALSE);
  for (c20_i359 = 0; c20_i359 < 127; c20_i359++) {
    c20_r2 = &c20_info[c20_i359];
    sf_mex_addfield(c20_m6, sf_mex_create("nameCaptureInfo", c20_r2->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c20_r2->context)), "context", "nameCaptureInfo",
                    c20_i359);
    sf_mex_addfield(c20_m6, sf_mex_create("nameCaptureInfo", c20_r2->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c20_r2->name)), "name", "nameCaptureInfo",
                    c20_i359);
    sf_mex_addfield(c20_m6, sf_mex_create("nameCaptureInfo",
      c20_r2->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c20_r2->dominantType)),
                    "dominantType", "nameCaptureInfo", c20_i359);
    sf_mex_addfield(c20_m6, sf_mex_create("nameCaptureInfo", c20_r2->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c20_r2->resolved)), "resolved",
                    "nameCaptureInfo", c20_i359);
    sf_mex_addfield(c20_m6, sf_mex_create("nameCaptureInfo", &c20_r2->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c20_i359);
    sf_mex_addfield(c20_m6, sf_mex_create("nameCaptureInfo", &c20_r2->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c20_i359);
    sf_mex_addfield(c20_m6, sf_mex_create("nameCaptureInfo",
      &c20_r2->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c20_i359);
    sf_mex_addfield(c20_m6, sf_mex_create("nameCaptureInfo",
      &c20_r2->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c20_i359);
  }

  sf_mex_assign(&c20_nameCaptureInfo, c20_m6, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c20_nameCaptureInfo);
  return c20_nameCaptureInfo;
}

static void c20_info_helper(c20_ResolvedFunctionInfo c20_info[127])
{
  c20_info[0].context = "";
  c20_info[0].name = "stateMachineStandUp";
  c20_info[0].dominantType = "struct";
  c20_info[0].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m";
  c20_info[0].fileTimeLo = 1495631764U;
  c20_info[0].fileTimeHi = 0U;
  c20_info[0].mFileTimeLo = 0U;
  c20_info[0].mFileTimeHi = 0U;
  c20_info[1].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m";
  c20_info[1].name = "mrdivide";
  c20_info[1].dominantType = "double";
  c20_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c20_info[1].fileTimeLo = 1357951548U;
  c20_info[1].fileTimeHi = 0U;
  c20_info[1].mFileTimeLo = 1319729966U;
  c20_info[1].mFileTimeHi = 0U;
  c20_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c20_info[2].name = "mldivide";
  c20_info[2].dominantType = "double";
  c20_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c20_info[2].fileTimeLo = 1357951548U;
  c20_info[2].fileTimeHi = 0U;
  c20_info[2].mFileTimeLo = 1319729966U;
  c20_info[2].mFileTimeHi = 0U;
  c20_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c20_info[3].name = "eml_lusolve";
  c20_info[3].dominantType = "double";
  c20_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c20_info[3].fileTimeLo = 1309451196U;
  c20_info[3].fileTimeHi = 0U;
  c20_info[3].mFileTimeLo = 0U;
  c20_info[3].mFileTimeHi = 0U;
  c20_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c20_info[4].name = "eml_index_class";
  c20_info[4].dominantType = "";
  c20_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[4].fileTimeLo = 1323170578U;
  c20_info[4].fileTimeHi = 0U;
  c20_info[4].mFileTimeLo = 0U;
  c20_info[4].mFileTimeHi = 0U;
  c20_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c20_info[5].name = "eml_index_class";
  c20_info[5].dominantType = "";
  c20_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[5].fileTimeLo = 1323170578U;
  c20_info[5].fileTimeHi = 0U;
  c20_info[5].mFileTimeLo = 0U;
  c20_info[5].mFileTimeHi = 0U;
  c20_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c20_info[6].name = "eml_xgetrf";
  c20_info[6].dominantType = "double";
  c20_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c20_info[6].fileTimeLo = 1286818806U;
  c20_info[6].fileTimeHi = 0U;
  c20_info[6].mFileTimeLo = 0U;
  c20_info[6].mFileTimeHi = 0U;
  c20_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c20_info[7].name = "eml_lapack_xgetrf";
  c20_info[7].dominantType = "double";
  c20_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c20_info[7].fileTimeLo = 1286818810U;
  c20_info[7].fileTimeHi = 0U;
  c20_info[7].mFileTimeLo = 0U;
  c20_info[7].mFileTimeHi = 0U;
  c20_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c20_info[8].name = "eml_matlab_zgetrf";
  c20_info[8].dominantType = "double";
  c20_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[8].fileTimeLo = 1302688994U;
  c20_info[8].fileTimeHi = 0U;
  c20_info[8].mFileTimeLo = 0U;
  c20_info[8].mFileTimeHi = 0U;
  c20_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[9].name = "realmin";
  c20_info[9].dominantType = "char";
  c20_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c20_info[9].fileTimeLo = 1307651242U;
  c20_info[9].fileTimeHi = 0U;
  c20_info[9].mFileTimeLo = 0U;
  c20_info[9].mFileTimeHi = 0U;
  c20_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c20_info[10].name = "eml_realmin";
  c20_info[10].dominantType = "char";
  c20_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c20_info[10].fileTimeLo = 1307651244U;
  c20_info[10].fileTimeHi = 0U;
  c20_info[10].mFileTimeLo = 0U;
  c20_info[10].mFileTimeHi = 0U;
  c20_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c20_info[11].name = "eml_float_model";
  c20_info[11].dominantType = "char";
  c20_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c20_info[11].fileTimeLo = 1326727996U;
  c20_info[11].fileTimeHi = 0U;
  c20_info[11].mFileTimeLo = 0U;
  c20_info[11].mFileTimeHi = 0U;
  c20_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[12].name = "eps";
  c20_info[12].dominantType = "char";
  c20_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c20_info[12].fileTimeLo = 1326727996U;
  c20_info[12].fileTimeHi = 0U;
  c20_info[12].mFileTimeLo = 0U;
  c20_info[12].mFileTimeHi = 0U;
  c20_info[13].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c20_info[13].name = "eml_is_float_class";
  c20_info[13].dominantType = "char";
  c20_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c20_info[13].fileTimeLo = 1286818782U;
  c20_info[13].fileTimeHi = 0U;
  c20_info[13].mFileTimeLo = 0U;
  c20_info[13].mFileTimeHi = 0U;
  c20_info[14].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c20_info[14].name = "eml_eps";
  c20_info[14].dominantType = "char";
  c20_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c20_info[14].fileTimeLo = 1326727996U;
  c20_info[14].fileTimeHi = 0U;
  c20_info[14].mFileTimeLo = 0U;
  c20_info[14].mFileTimeHi = 0U;
  c20_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c20_info[15].name = "eml_float_model";
  c20_info[15].dominantType = "char";
  c20_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c20_info[15].fileTimeLo = 1326727996U;
  c20_info[15].fileTimeHi = 0U;
  c20_info[15].mFileTimeLo = 0U;
  c20_info[15].mFileTimeHi = 0U;
  c20_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[16].name = "min";
  c20_info[16].dominantType = "coder.internal.indexInt";
  c20_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c20_info[16].fileTimeLo = 1311255318U;
  c20_info[16].fileTimeHi = 0U;
  c20_info[16].mFileTimeLo = 0U;
  c20_info[16].mFileTimeHi = 0U;
  c20_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c20_info[17].name = "eml_min_or_max";
  c20_info[17].dominantType = "char";
  c20_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c20_info[17].fileTimeLo = 1334071490U;
  c20_info[17].fileTimeHi = 0U;
  c20_info[17].mFileTimeLo = 0U;
  c20_info[17].mFileTimeHi = 0U;
  c20_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c20_info[18].name = "eml_scalar_eg";
  c20_info[18].dominantType = "coder.internal.indexInt";
  c20_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[18].fileTimeLo = 1286818796U;
  c20_info[18].fileTimeHi = 0U;
  c20_info[18].mFileTimeLo = 0U;
  c20_info[18].mFileTimeHi = 0U;
  c20_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c20_info[19].name = "eml_scalexp_alloc";
  c20_info[19].dominantType = "coder.internal.indexInt";
  c20_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c20_info[19].fileTimeLo = 1352424860U;
  c20_info[19].fileTimeHi = 0U;
  c20_info[19].mFileTimeLo = 0U;
  c20_info[19].mFileTimeHi = 0U;
  c20_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c20_info[20].name = "eml_index_class";
  c20_info[20].dominantType = "";
  c20_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[20].fileTimeLo = 1323170578U;
  c20_info[20].fileTimeHi = 0U;
  c20_info[20].mFileTimeLo = 0U;
  c20_info[20].mFileTimeHi = 0U;
  c20_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c20_info[21].name = "eml_scalar_eg";
  c20_info[21].dominantType = "coder.internal.indexInt";
  c20_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[21].fileTimeLo = 1286818796U;
  c20_info[21].fileTimeHi = 0U;
  c20_info[21].mFileTimeLo = 0U;
  c20_info[21].mFileTimeHi = 0U;
  c20_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[22].name = "colon";
  c20_info[22].dominantType = "double";
  c20_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c20_info[22].fileTimeLo = 1348191928U;
  c20_info[22].fileTimeHi = 0U;
  c20_info[22].mFileTimeLo = 0U;
  c20_info[22].mFileTimeHi = 0U;
  c20_info[23].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c20_info[23].name = "colon";
  c20_info[23].dominantType = "double";
  c20_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c20_info[23].fileTimeLo = 1348191928U;
  c20_info[23].fileTimeHi = 0U;
  c20_info[23].mFileTimeLo = 0U;
  c20_info[23].mFileTimeHi = 0U;
  c20_info[24].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c20_info[24].name = "floor";
  c20_info[24].dominantType = "double";
  c20_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c20_info[24].fileTimeLo = 1343830380U;
  c20_info[24].fileTimeHi = 0U;
  c20_info[24].mFileTimeLo = 0U;
  c20_info[24].mFileTimeHi = 0U;
  c20_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c20_info[25].name = "eml_scalar_floor";
  c20_info[25].dominantType = "double";
  c20_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c20_info[25].fileTimeLo = 1286818726U;
  c20_info[25].fileTimeHi = 0U;
  c20_info[25].mFileTimeLo = 0U;
  c20_info[25].mFileTimeHi = 0U;
  c20_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c20_info[26].name = "intmin";
  c20_info[26].dominantType = "char";
  c20_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c20_info[26].fileTimeLo = 1311255318U;
  c20_info[26].fileTimeHi = 0U;
  c20_info[26].mFileTimeLo = 0U;
  c20_info[26].mFileTimeHi = 0U;
  c20_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c20_info[27].name = "intmax";
  c20_info[27].dominantType = "char";
  c20_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c20_info[27].fileTimeLo = 1311255316U;
  c20_info[27].fileTimeHi = 0U;
  c20_info[27].mFileTimeLo = 0U;
  c20_info[27].mFileTimeHi = 0U;
  c20_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c20_info[28].name = "intmin";
  c20_info[28].dominantType = "char";
  c20_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c20_info[28].fileTimeLo = 1311255318U;
  c20_info[28].fileTimeHi = 0U;
  c20_info[28].mFileTimeLo = 0U;
  c20_info[28].mFileTimeHi = 0U;
  c20_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c20_info[29].name = "intmax";
  c20_info[29].dominantType = "char";
  c20_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c20_info[29].fileTimeLo = 1311255316U;
  c20_info[29].fileTimeHi = 0U;
  c20_info[29].mFileTimeLo = 0U;
  c20_info[29].mFileTimeHi = 0U;
  c20_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c20_info[30].name = "eml_isa_uint";
  c20_info[30].dominantType = "coder.internal.indexInt";
  c20_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c20_info[30].fileTimeLo = 1286818784U;
  c20_info[30].fileTimeHi = 0U;
  c20_info[30].mFileTimeLo = 0U;
  c20_info[30].mFileTimeHi = 0U;
  c20_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c20_info[31].name = "eml_unsigned_class";
  c20_info[31].dominantType = "char";
  c20_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c20_info[31].fileTimeLo = 1323170580U;
  c20_info[31].fileTimeHi = 0U;
  c20_info[31].mFileTimeLo = 0U;
  c20_info[31].mFileTimeHi = 0U;
  c20_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c20_info[32].name = "eml_index_class";
  c20_info[32].dominantType = "";
  c20_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[32].fileTimeLo = 1323170578U;
  c20_info[32].fileTimeHi = 0U;
  c20_info[32].mFileTimeLo = 0U;
  c20_info[32].mFileTimeHi = 0U;
  c20_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c20_info[33].name = "eml_index_class";
  c20_info[33].dominantType = "";
  c20_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[33].fileTimeLo = 1323170578U;
  c20_info[33].fileTimeHi = 0U;
  c20_info[33].mFileTimeLo = 0U;
  c20_info[33].mFileTimeHi = 0U;
  c20_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c20_info[34].name = "intmax";
  c20_info[34].dominantType = "char";
  c20_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c20_info[34].fileTimeLo = 1311255316U;
  c20_info[34].fileTimeHi = 0U;
  c20_info[34].mFileTimeLo = 0U;
  c20_info[34].mFileTimeHi = 0U;
  c20_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c20_info[35].name = "eml_isa_uint";
  c20_info[35].dominantType = "coder.internal.indexInt";
  c20_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c20_info[35].fileTimeLo = 1286818784U;
  c20_info[35].fileTimeHi = 0U;
  c20_info[35].mFileTimeLo = 0U;
  c20_info[35].mFileTimeHi = 0U;
  c20_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c20_info[36].name = "eml_index_plus";
  c20_info[36].dominantType = "double";
  c20_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[36].fileTimeLo = 1286818778U;
  c20_info[36].fileTimeHi = 0U;
  c20_info[36].mFileTimeLo = 0U;
  c20_info[36].mFileTimeHi = 0U;
  c20_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[37].name = "eml_index_class";
  c20_info[37].dominantType = "";
  c20_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[37].fileTimeLo = 1323170578U;
  c20_info[37].fileTimeHi = 0U;
  c20_info[37].mFileTimeLo = 0U;
  c20_info[37].mFileTimeHi = 0U;
  c20_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c20_info[38].name = "eml_int_forloop_overflow_check";
  c20_info[38].dominantType = "";
  c20_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c20_info[38].fileTimeLo = 1346510340U;
  c20_info[38].fileTimeHi = 0U;
  c20_info[38].mFileTimeLo = 0U;
  c20_info[38].mFileTimeHi = 0U;
  c20_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c20_info[39].name = "intmax";
  c20_info[39].dominantType = "char";
  c20_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c20_info[39].fileTimeLo = 1311255316U;
  c20_info[39].fileTimeHi = 0U;
  c20_info[39].mFileTimeLo = 0U;
  c20_info[39].mFileTimeHi = 0U;
  c20_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[40].name = "eml_index_class";
  c20_info[40].dominantType = "";
  c20_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[40].fileTimeLo = 1323170578U;
  c20_info[40].fileTimeHi = 0U;
  c20_info[40].mFileTimeLo = 0U;
  c20_info[40].mFileTimeHi = 0U;
  c20_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[41].name = "eml_index_plus";
  c20_info[41].dominantType = "double";
  c20_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[41].fileTimeLo = 1286818778U;
  c20_info[41].fileTimeHi = 0U;
  c20_info[41].mFileTimeLo = 0U;
  c20_info[41].mFileTimeHi = 0U;
  c20_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[42].name = "eml_int_forloop_overflow_check";
  c20_info[42].dominantType = "";
  c20_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c20_info[42].fileTimeLo = 1346510340U;
  c20_info[42].fileTimeHi = 0U;
  c20_info[42].mFileTimeLo = 0U;
  c20_info[42].mFileTimeHi = 0U;
  c20_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[43].name = "eml_index_minus";
  c20_info[43].dominantType = "double";
  c20_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c20_info[43].fileTimeLo = 1286818778U;
  c20_info[43].fileTimeHi = 0U;
  c20_info[43].mFileTimeLo = 0U;
  c20_info[43].mFileTimeHi = 0U;
  c20_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c20_info[44].name = "eml_index_class";
  c20_info[44].dominantType = "";
  c20_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[44].fileTimeLo = 1323170578U;
  c20_info[44].fileTimeHi = 0U;
  c20_info[44].mFileTimeLo = 0U;
  c20_info[44].mFileTimeHi = 0U;
  c20_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[45].name = "eml_index_minus";
  c20_info[45].dominantType = "coder.internal.indexInt";
  c20_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c20_info[45].fileTimeLo = 1286818778U;
  c20_info[45].fileTimeHi = 0U;
  c20_info[45].mFileTimeLo = 0U;
  c20_info[45].mFileTimeHi = 0U;
  c20_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[46].name = "eml_index_times";
  c20_info[46].dominantType = "coder.internal.indexInt";
  c20_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c20_info[46].fileTimeLo = 1286818780U;
  c20_info[46].fileTimeHi = 0U;
  c20_info[46].mFileTimeLo = 0U;
  c20_info[46].mFileTimeHi = 0U;
  c20_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c20_info[47].name = "eml_index_class";
  c20_info[47].dominantType = "";
  c20_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[47].fileTimeLo = 1323170578U;
  c20_info[47].fileTimeHi = 0U;
  c20_info[47].mFileTimeLo = 0U;
  c20_info[47].mFileTimeHi = 0U;
  c20_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[48].name = "eml_index_plus";
  c20_info[48].dominantType = "coder.internal.indexInt";
  c20_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[48].fileTimeLo = 1286818778U;
  c20_info[48].fileTimeHi = 0U;
  c20_info[48].mFileTimeLo = 0U;
  c20_info[48].mFileTimeHi = 0U;
  c20_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[49].name = "eml_ixamax";
  c20_info[49].dominantType = "double";
  c20_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c20_info[49].fileTimeLo = 1299076770U;
  c20_info[49].fileTimeHi = 0U;
  c20_info[49].mFileTimeLo = 0U;
  c20_info[49].mFileTimeHi = 0U;
  c20_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c20_info[50].name = "eml_blas_inline";
  c20_info[50].dominantType = "";
  c20_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c20_info[50].fileTimeLo = 1299076768U;
  c20_info[50].fileTimeHi = 0U;
  c20_info[50].mFileTimeLo = 0U;
  c20_info[50].mFileTimeHi = 0U;
  c20_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c20_info[51].name = "length";
  c20_info[51].dominantType = "double";
  c20_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c20_info[51].fileTimeLo = 1303146206U;
  c20_info[51].fileTimeHi = 0U;
  c20_info[51].mFileTimeLo = 0U;
  c20_info[51].mFileTimeHi = 0U;
  c20_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c20_info[52].name = "eml_index_class";
  c20_info[52].dominantType = "";
  c20_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[52].fileTimeLo = 1323170578U;
  c20_info[52].fileTimeHi = 0U;
  c20_info[52].mFileTimeLo = 0U;
  c20_info[52].mFileTimeHi = 0U;
  c20_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c20_info[53].name = "eml_index_class";
  c20_info[53].dominantType = "";
  c20_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[53].fileTimeLo = 1323170578U;
  c20_info[53].fileTimeHi = 0U;
  c20_info[53].mFileTimeLo = 0U;
  c20_info[53].mFileTimeHi = 0U;
  c20_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c20_info[54].name = "eml_refblas_ixamax";
  c20_info[54].dominantType = "double";
  c20_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c20_info[54].fileTimeLo = 1299076770U;
  c20_info[54].fileTimeHi = 0U;
  c20_info[54].mFileTimeLo = 0U;
  c20_info[54].mFileTimeHi = 0U;
  c20_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c20_info[55].name = "eml_index_class";
  c20_info[55].dominantType = "";
  c20_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[55].fileTimeLo = 1323170578U;
  c20_info[55].fileTimeHi = 0U;
  c20_info[55].mFileTimeLo = 0U;
  c20_info[55].mFileTimeHi = 0U;
  c20_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c20_info[56].name = "eml_xcabs1";
  c20_info[56].dominantType = "double";
  c20_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c20_info[56].fileTimeLo = 1286818706U;
  c20_info[56].fileTimeHi = 0U;
  c20_info[56].mFileTimeLo = 0U;
  c20_info[56].mFileTimeHi = 0U;
  c20_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c20_info[57].name = "abs";
  c20_info[57].dominantType = "double";
  c20_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c20_info[57].fileTimeLo = 1343830366U;
  c20_info[57].fileTimeHi = 0U;
  c20_info[57].mFileTimeLo = 0U;
  c20_info[57].mFileTimeHi = 0U;
  c20_info[58].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c20_info[58].name = "eml_scalar_abs";
  c20_info[58].dominantType = "double";
  c20_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c20_info[58].fileTimeLo = 1286818712U;
  c20_info[58].fileTimeHi = 0U;
  c20_info[58].mFileTimeLo = 0U;
  c20_info[58].mFileTimeHi = 0U;
  c20_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c20_info[59].name = "eml_int_forloop_overflow_check";
  c20_info[59].dominantType = "";
  c20_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c20_info[59].fileTimeLo = 1346510340U;
  c20_info[59].fileTimeHi = 0U;
  c20_info[59].mFileTimeLo = 0U;
  c20_info[59].mFileTimeHi = 0U;
  c20_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c20_info[60].name = "eml_index_plus";
  c20_info[60].dominantType = "coder.internal.indexInt";
  c20_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[60].fileTimeLo = 1286818778U;
  c20_info[60].fileTimeHi = 0U;
  c20_info[60].mFileTimeLo = 0U;
  c20_info[60].mFileTimeHi = 0U;
  c20_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[61].name = "eml_xswap";
  c20_info[61].dominantType = "double";
  c20_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c20_info[61].fileTimeLo = 1299076778U;
  c20_info[61].fileTimeHi = 0U;
  c20_info[61].mFileTimeLo = 0U;
  c20_info[61].mFileTimeHi = 0U;
  c20_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c20_info[62].name = "eml_blas_inline";
  c20_info[62].dominantType = "";
  c20_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c20_info[62].fileTimeLo = 1299076768U;
  c20_info[62].fileTimeHi = 0U;
  c20_info[62].mFileTimeLo = 0U;
  c20_info[62].mFileTimeHi = 0U;
  c20_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c20_info[63].name = "eml_index_class";
  c20_info[63].dominantType = "";
  c20_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[63].fileTimeLo = 1323170578U;
  c20_info[63].fileTimeHi = 0U;
  c20_info[63].mFileTimeLo = 0U;
  c20_info[63].mFileTimeHi = 0U;
}

static void c20_b_info_helper(c20_ResolvedFunctionInfo c20_info[127])
{
  c20_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c20_info[64].name = "eml_refblas_xswap";
  c20_info[64].dominantType = "double";
  c20_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c20_info[64].fileTimeLo = 1299076786U;
  c20_info[64].fileTimeHi = 0U;
  c20_info[64].mFileTimeLo = 0U;
  c20_info[64].mFileTimeHi = 0U;
  c20_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c20_info[65].name = "eml_index_class";
  c20_info[65].dominantType = "";
  c20_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[65].fileTimeLo = 1323170578U;
  c20_info[65].fileTimeHi = 0U;
  c20_info[65].mFileTimeLo = 0U;
  c20_info[65].mFileTimeHi = 0U;
  c20_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c20_info[66].name = "abs";
  c20_info[66].dominantType = "coder.internal.indexInt";
  c20_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c20_info[66].fileTimeLo = 1343830366U;
  c20_info[66].fileTimeHi = 0U;
  c20_info[66].mFileTimeLo = 0U;
  c20_info[66].mFileTimeHi = 0U;
  c20_info[67].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c20_info[67].name = "eml_scalar_abs";
  c20_info[67].dominantType = "coder.internal.indexInt";
  c20_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c20_info[67].fileTimeLo = 1286818712U;
  c20_info[67].fileTimeHi = 0U;
  c20_info[67].mFileTimeLo = 0U;
  c20_info[67].mFileTimeHi = 0U;
  c20_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c20_info[68].name = "eml_int_forloop_overflow_check";
  c20_info[68].dominantType = "";
  c20_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c20_info[68].fileTimeLo = 1346510340U;
  c20_info[68].fileTimeHi = 0U;
  c20_info[68].mFileTimeLo = 0U;
  c20_info[68].mFileTimeHi = 0U;
  c20_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c20_info[69].name = "eml_index_plus";
  c20_info[69].dominantType = "coder.internal.indexInt";
  c20_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[69].fileTimeLo = 1286818778U;
  c20_info[69].fileTimeHi = 0U;
  c20_info[69].mFileTimeLo = 0U;
  c20_info[69].mFileTimeHi = 0U;
  c20_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[70].name = "eml_div";
  c20_info[70].dominantType = "double";
  c20_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c20_info[70].fileTimeLo = 1313347810U;
  c20_info[70].fileTimeHi = 0U;
  c20_info[70].mFileTimeLo = 0U;
  c20_info[70].mFileTimeHi = 0U;
  c20_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c20_info[71].name = "eml_xgeru";
  c20_info[71].dominantType = "double";
  c20_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c20_info[71].fileTimeLo = 1299076774U;
  c20_info[71].fileTimeHi = 0U;
  c20_info[71].mFileTimeLo = 0U;
  c20_info[71].mFileTimeHi = 0U;
  c20_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c20_info[72].name = "eml_blas_inline";
  c20_info[72].dominantType = "";
  c20_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c20_info[72].fileTimeLo = 1299076768U;
  c20_info[72].fileTimeHi = 0U;
  c20_info[72].mFileTimeLo = 0U;
  c20_info[72].mFileTimeHi = 0U;
  c20_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c20_info[73].name = "eml_xger";
  c20_info[73].dominantType = "double";
  c20_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c20_info[73].fileTimeLo = 1299076774U;
  c20_info[73].fileTimeHi = 0U;
  c20_info[73].mFileTimeLo = 0U;
  c20_info[73].mFileTimeHi = 0U;
  c20_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c20_info[74].name = "eml_blas_inline";
  c20_info[74].dominantType = "";
  c20_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c20_info[74].fileTimeLo = 1299076768U;
  c20_info[74].fileTimeHi = 0U;
  c20_info[74].mFileTimeLo = 0U;
  c20_info[74].mFileTimeHi = 0U;
  c20_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c20_info[75].name = "intmax";
  c20_info[75].dominantType = "char";
  c20_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c20_info[75].fileTimeLo = 1311255316U;
  c20_info[75].fileTimeHi = 0U;
  c20_info[75].mFileTimeLo = 0U;
  c20_info[75].mFileTimeHi = 0U;
  c20_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c20_info[76].name = "min";
  c20_info[76].dominantType = "double";
  c20_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c20_info[76].fileTimeLo = 1311255318U;
  c20_info[76].fileTimeHi = 0U;
  c20_info[76].mFileTimeLo = 0U;
  c20_info[76].mFileTimeHi = 0U;
  c20_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c20_info[77].name = "eml_scalar_eg";
  c20_info[77].dominantType = "double";
  c20_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[77].fileTimeLo = 1286818796U;
  c20_info[77].fileTimeHi = 0U;
  c20_info[77].mFileTimeLo = 0U;
  c20_info[77].mFileTimeHi = 0U;
  c20_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c20_info[78].name = "eml_scalexp_alloc";
  c20_info[78].dominantType = "double";
  c20_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c20_info[78].fileTimeLo = 1352424860U;
  c20_info[78].fileTimeHi = 0U;
  c20_info[78].mFileTimeLo = 0U;
  c20_info[78].mFileTimeHi = 0U;
  c20_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c20_info[79].name = "eml_scalar_eg";
  c20_info[79].dominantType = "double";
  c20_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[79].fileTimeLo = 1286818796U;
  c20_info[79].fileTimeHi = 0U;
  c20_info[79].mFileTimeLo = 0U;
  c20_info[79].mFileTimeHi = 0U;
  c20_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c20_info[80].name = "mtimes";
  c20_info[80].dominantType = "double";
  c20_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c20_info[80].fileTimeLo = 1289519692U;
  c20_info[80].fileTimeHi = 0U;
  c20_info[80].mFileTimeLo = 0U;
  c20_info[80].mFileTimeHi = 0U;
  c20_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c20_info[81].name = "eml_index_class";
  c20_info[81].dominantType = "";
  c20_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[81].fileTimeLo = 1323170578U;
  c20_info[81].fileTimeHi = 0U;
  c20_info[81].mFileTimeLo = 0U;
  c20_info[81].mFileTimeHi = 0U;
  c20_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c20_info[82].name = "eml_refblas_xger";
  c20_info[82].dominantType = "double";
  c20_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c20_info[82].fileTimeLo = 1299076776U;
  c20_info[82].fileTimeHi = 0U;
  c20_info[82].mFileTimeLo = 0U;
  c20_info[82].mFileTimeHi = 0U;
  c20_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c20_info[83].name = "eml_refblas_xgerx";
  c20_info[83].dominantType = "char";
  c20_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c20_info[83].fileTimeLo = 1299076778U;
  c20_info[83].fileTimeHi = 0U;
  c20_info[83].mFileTimeLo = 0U;
  c20_info[83].mFileTimeHi = 0U;
  c20_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c20_info[84].name = "eml_index_class";
  c20_info[84].dominantType = "";
  c20_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[84].fileTimeLo = 1323170578U;
  c20_info[84].fileTimeHi = 0U;
  c20_info[84].mFileTimeLo = 0U;
  c20_info[84].mFileTimeHi = 0U;
  c20_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c20_info[85].name = "abs";
  c20_info[85].dominantType = "coder.internal.indexInt";
  c20_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c20_info[85].fileTimeLo = 1343830366U;
  c20_info[85].fileTimeHi = 0U;
  c20_info[85].mFileTimeLo = 0U;
  c20_info[85].mFileTimeHi = 0U;
  c20_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c20_info[86].name = "eml_index_minus";
  c20_info[86].dominantType = "double";
  c20_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c20_info[86].fileTimeLo = 1286818778U;
  c20_info[86].fileTimeHi = 0U;
  c20_info[86].mFileTimeLo = 0U;
  c20_info[86].mFileTimeHi = 0U;
  c20_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c20_info[87].name = "eml_int_forloop_overflow_check";
  c20_info[87].dominantType = "";
  c20_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c20_info[87].fileTimeLo = 1346510340U;
  c20_info[87].fileTimeHi = 0U;
  c20_info[87].mFileTimeLo = 0U;
  c20_info[87].mFileTimeHi = 0U;
  c20_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c20_info[88].name = "eml_index_plus";
  c20_info[88].dominantType = "double";
  c20_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[88].fileTimeLo = 1286818778U;
  c20_info[88].fileTimeHi = 0U;
  c20_info[88].mFileTimeLo = 0U;
  c20_info[88].mFileTimeHi = 0U;
  c20_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c20_info[89].name = "eml_index_plus";
  c20_info[89].dominantType = "coder.internal.indexInt";
  c20_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[89].fileTimeLo = 1286818778U;
  c20_info[89].fileTimeHi = 0U;
  c20_info[89].mFileTimeLo = 0U;
  c20_info[89].mFileTimeHi = 0U;
  c20_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c20_info[90].name = "eml_warning";
  c20_info[90].dominantType = "char";
  c20_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c20_info[90].fileTimeLo = 1286818802U;
  c20_info[90].fileTimeHi = 0U;
  c20_info[90].mFileTimeLo = 0U;
  c20_info[90].mFileTimeHi = 0U;
  c20_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c20_info[91].name = "eml_scalar_eg";
  c20_info[91].dominantType = "double";
  c20_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[91].fileTimeLo = 1286818796U;
  c20_info[91].fileTimeHi = 0U;
  c20_info[91].mFileTimeLo = 0U;
  c20_info[91].mFileTimeHi = 0U;
  c20_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c20_info[92].name = "eml_int_forloop_overflow_check";
  c20_info[92].dominantType = "";
  c20_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c20_info[92].fileTimeLo = 1346510340U;
  c20_info[92].fileTimeHi = 0U;
  c20_info[92].mFileTimeLo = 0U;
  c20_info[92].mFileTimeHi = 0U;
  c20_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c20_info[93].name = "eml_xtrsm";
  c20_info[93].dominantType = "char";
  c20_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c20_info[93].fileTimeLo = 1299076778U;
  c20_info[93].fileTimeHi = 0U;
  c20_info[93].mFileTimeLo = 0U;
  c20_info[93].mFileTimeHi = 0U;
  c20_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c20_info[94].name = "eml_blas_inline";
  c20_info[94].dominantType = "";
  c20_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c20_info[94].fileTimeLo = 1299076768U;
  c20_info[94].fileTimeHi = 0U;
  c20_info[94].mFileTimeLo = 0U;
  c20_info[94].mFileTimeHi = 0U;
  c20_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c20_info[95].name = "mtimes";
  c20_info[95].dominantType = "double";
  c20_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c20_info[95].fileTimeLo = 1289519692U;
  c20_info[95].fileTimeHi = 0U;
  c20_info[95].mFileTimeLo = 0U;
  c20_info[95].mFileTimeHi = 0U;
  c20_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c20_info[96].name = "eml_index_class";
  c20_info[96].dominantType = "";
  c20_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[96].fileTimeLo = 1323170578U;
  c20_info[96].fileTimeHi = 0U;
  c20_info[96].mFileTimeLo = 0U;
  c20_info[96].mFileTimeHi = 0U;
  c20_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c20_info[97].name = "eml_scalar_eg";
  c20_info[97].dominantType = "double";
  c20_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[97].fileTimeLo = 1286818796U;
  c20_info[97].fileTimeHi = 0U;
  c20_info[97].mFileTimeLo = 0U;
  c20_info[97].mFileTimeHi = 0U;
  c20_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c20_info[98].name = "eml_refblas_xtrsm";
  c20_info[98].dominantType = "char";
  c20_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[98].fileTimeLo = 1299076786U;
  c20_info[98].fileTimeHi = 0U;
  c20_info[98].mFileTimeLo = 0U;
  c20_info[98].mFileTimeHi = 0U;
  c20_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[99].name = "eml_scalar_eg";
  c20_info[99].dominantType = "double";
  c20_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[99].fileTimeLo = 1286818796U;
  c20_info[99].fileTimeHi = 0U;
  c20_info[99].mFileTimeLo = 0U;
  c20_info[99].mFileTimeHi = 0U;
  c20_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[100].name = "eml_index_minus";
  c20_info[100].dominantType = "double";
  c20_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c20_info[100].fileTimeLo = 1286818778U;
  c20_info[100].fileTimeHi = 0U;
  c20_info[100].mFileTimeLo = 0U;
  c20_info[100].mFileTimeHi = 0U;
  c20_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[101].name = "eml_index_class";
  c20_info[101].dominantType = "";
  c20_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[101].fileTimeLo = 1323170578U;
  c20_info[101].fileTimeHi = 0U;
  c20_info[101].mFileTimeLo = 0U;
  c20_info[101].mFileTimeHi = 0U;
  c20_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[102].name = "eml_int_forloop_overflow_check";
  c20_info[102].dominantType = "";
  c20_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c20_info[102].fileTimeLo = 1346510340U;
  c20_info[102].fileTimeHi = 0U;
  c20_info[102].mFileTimeLo = 0U;
  c20_info[102].mFileTimeHi = 0U;
  c20_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[103].name = "eml_index_times";
  c20_info[103].dominantType = "coder.internal.indexInt";
  c20_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c20_info[103].fileTimeLo = 1286818780U;
  c20_info[103].fileTimeHi = 0U;
  c20_info[103].mFileTimeLo = 0U;
  c20_info[103].mFileTimeHi = 0U;
  c20_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[104].name = "eml_index_plus";
  c20_info[104].dominantType = "coder.internal.indexInt";
  c20_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[104].fileTimeLo = 1286818778U;
  c20_info[104].fileTimeHi = 0U;
  c20_info[104].mFileTimeLo = 0U;
  c20_info[104].mFileTimeHi = 0U;
  c20_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[105].name = "eml_index_plus";
  c20_info[105].dominantType = "double";
  c20_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c20_info[105].fileTimeLo = 1286818778U;
  c20_info[105].fileTimeHi = 0U;
  c20_info[105].mFileTimeLo = 0U;
  c20_info[105].mFileTimeHi = 0U;
  c20_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c20_info[106].name = "intmin";
  c20_info[106].dominantType = "char";
  c20_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c20_info[106].fileTimeLo = 1311255318U;
  c20_info[106].fileTimeHi = 0U;
  c20_info[106].mFileTimeLo = 0U;
  c20_info[106].mFileTimeHi = 0U;
  c20_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c20_info[107].name = "eml_div";
  c20_info[107].dominantType = "double";
  c20_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c20_info[107].fileTimeLo = 1313347810U;
  c20_info[107].fileTimeHi = 0U;
  c20_info[107].mFileTimeLo = 0U;
  c20_info[107].mFileTimeHi = 0U;
  c20_info[108].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m";
  c20_info[108].name = "eye";
  c20_info[108].dominantType = "double";
  c20_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c20_info[108].fileTimeLo = 1286818688U;
  c20_info[108].fileTimeHi = 0U;
  c20_info[108].mFileTimeLo = 0U;
  c20_info[108].mFileTimeHi = 0U;
  c20_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c20_info[109].name = "eml_assert_valid_size_arg";
  c20_info[109].dominantType = "double";
  c20_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c20_info[109].fileTimeLo = 1286818694U;
  c20_info[109].fileTimeHi = 0U;
  c20_info[109].mFileTimeLo = 0U;
  c20_info[109].mFileTimeHi = 0U;
  c20_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c20_info[110].name = "isinf";
  c20_info[110].dominantType = "double";
  c20_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c20_info[110].fileTimeLo = 1286818760U;
  c20_info[110].fileTimeHi = 0U;
  c20_info[110].mFileTimeLo = 0U;
  c20_info[110].mFileTimeHi = 0U;
  c20_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c20_info[111].name = "mtimes";
  c20_info[111].dominantType = "double";
  c20_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c20_info[111].fileTimeLo = 1289519692U;
  c20_info[111].fileTimeHi = 0U;
  c20_info[111].mFileTimeLo = 0U;
  c20_info[111].mFileTimeHi = 0U;
  c20_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c20_info[112].name = "eml_index_class";
  c20_info[112].dominantType = "";
  c20_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[112].fileTimeLo = 1323170578U;
  c20_info[112].fileTimeHi = 0U;
  c20_info[112].mFileTimeLo = 0U;
  c20_info[112].mFileTimeHi = 0U;
  c20_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c20_info[113].name = "intmax";
  c20_info[113].dominantType = "char";
  c20_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c20_info[113].fileTimeLo = 1311255316U;
  c20_info[113].fileTimeHi = 0U;
  c20_info[113].mFileTimeLo = 0U;
  c20_info[113].mFileTimeHi = 0U;
  c20_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c20_info[114].name = "eml_is_float_class";
  c20_info[114].dominantType = "char";
  c20_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c20_info[114].fileTimeLo = 1286818782U;
  c20_info[114].fileTimeHi = 0U;
  c20_info[114].mFileTimeLo = 0U;
  c20_info[114].mFileTimeHi = 0U;
  c20_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c20_info[115].name = "min";
  c20_info[115].dominantType = "double";
  c20_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c20_info[115].fileTimeLo = 1311255318U;
  c20_info[115].fileTimeHi = 0U;
  c20_info[115].mFileTimeLo = 0U;
  c20_info[115].mFileTimeHi = 0U;
  c20_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c20_info[116].name = "eml_index_class";
  c20_info[116].dominantType = "";
  c20_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[116].fileTimeLo = 1323170578U;
  c20_info[116].fileTimeHi = 0U;
  c20_info[116].mFileTimeLo = 0U;
  c20_info[116].mFileTimeHi = 0U;
  c20_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c20_info[117].name = "eml_int_forloop_overflow_check";
  c20_info[117].dominantType = "";
  c20_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c20_info[117].fileTimeLo = 1346510340U;
  c20_info[117].fileTimeHi = 0U;
  c20_info[117].mFileTimeLo = 0U;
  c20_info[117].mFileTimeHi = 0U;
  c20_info[118].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m";
  c20_info[118].name = "mtimes";
  c20_info[118].dominantType = "double";
  c20_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c20_info[118].fileTimeLo = 1289519692U;
  c20_info[118].fileTimeHi = 0U;
  c20_info[118].mFileTimeLo = 0U;
  c20_info[118].mFileTimeHi = 0U;
  c20_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c20_info[119].name = "eml_index_class";
  c20_info[119].dominantType = "";
  c20_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[119].fileTimeLo = 1323170578U;
  c20_info[119].fileTimeHi = 0U;
  c20_info[119].mFileTimeLo = 0U;
  c20_info[119].mFileTimeHi = 0U;
  c20_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c20_info[120].name = "eml_scalar_eg";
  c20_info[120].dominantType = "double";
  c20_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[120].fileTimeLo = 1286818796U;
  c20_info[120].fileTimeHi = 0U;
  c20_info[120].mFileTimeLo = 0U;
  c20_info[120].mFileTimeHi = 0U;
  c20_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c20_info[121].name = "eml_xgemm";
  c20_info[121].dominantType = "char";
  c20_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c20_info[121].fileTimeLo = 1299076772U;
  c20_info[121].fileTimeHi = 0U;
  c20_info[121].mFileTimeLo = 0U;
  c20_info[121].mFileTimeHi = 0U;
  c20_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c20_info[122].name = "eml_blas_inline";
  c20_info[122].dominantType = "";
  c20_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c20_info[122].fileTimeLo = 1299076768U;
  c20_info[122].fileTimeHi = 0U;
  c20_info[122].mFileTimeLo = 0U;
  c20_info[122].mFileTimeHi = 0U;
  c20_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c20_info[123].name = "mtimes";
  c20_info[123].dominantType = "double";
  c20_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c20_info[123].fileTimeLo = 1289519692U;
  c20_info[123].fileTimeHi = 0U;
  c20_info[123].mFileTimeLo = 0U;
  c20_info[123].mFileTimeHi = 0U;
  c20_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c20_info[124].name = "eml_index_class";
  c20_info[124].dominantType = "";
  c20_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c20_info[124].fileTimeLo = 1323170578U;
  c20_info[124].fileTimeHi = 0U;
  c20_info[124].mFileTimeLo = 0U;
  c20_info[124].mFileTimeHi = 0U;
  c20_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c20_info[125].name = "eml_scalar_eg";
  c20_info[125].dominantType = "double";
  c20_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c20_info[125].fileTimeLo = 1286818796U;
  c20_info[125].fileTimeHi = 0U;
  c20_info[125].mFileTimeLo = 0U;
  c20_info[125].mFileTimeHi = 0U;
  c20_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c20_info[126].name = "eml_refblas_xgemm";
  c20_info[126].dominantType = "char";
  c20_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c20_info[126].fileTimeLo = 1299076774U;
  c20_info[126].fileTimeHi = 0U;
  c20_info[126].mFileTimeLo = 0U;
  c20_info[126].mFileTimeHi = 0U;
}

static void c20_mrdivide(SFc20_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c20_A[16], real_T c20_B[16], real_T c20_y[16])
{
  int32_T c20_i360;
  int32_T c20_i361;
  int32_T c20_i362;
  int32_T c20_i363;
  real_T c20_b_A[16];
  int32_T c20_i364;
  int32_T c20_i365;
  int32_T c20_i366;
  int32_T c20_i367;
  real_T c20_b_B[16];
  int32_T c20_info;
  int32_T c20_ipiv[4];
  int32_T c20_b_info;
  int32_T c20_c_info;
  int32_T c20_d_info;
  int32_T c20_i;
  int32_T c20_b_i;
  int32_T c20_ip;
  int32_T c20_j;
  int32_T c20_b_j;
  real_T c20_temp;
  int32_T c20_i368;
  real_T c20_c_A[16];
  int32_T c20_i369;
  real_T c20_d_A[16];
  int32_T c20_i370;
  int32_T c20_i371;
  int32_T c20_i372;
  int32_T c20_i373;
  c20_i360 = 0;
  for (c20_i361 = 0; c20_i361 < 4; c20_i361++) {
    c20_i362 = 0;
    for (c20_i363 = 0; c20_i363 < 4; c20_i363++) {
      c20_b_A[c20_i363 + c20_i360] = c20_B[c20_i362 + c20_i361];
      c20_i362 += 4;
    }

    c20_i360 += 4;
  }

  c20_i364 = 0;
  for (c20_i365 = 0; c20_i365 < 4; c20_i365++) {
    c20_i366 = 0;
    for (c20_i367 = 0; c20_i367 < 4; c20_i367++) {
      c20_b_B[c20_i367 + c20_i364] = c20_A[c20_i366 + c20_i365];
      c20_i366 += 4;
    }

    c20_i364 += 4;
  }

  c20_b_eml_matlab_zgetrf(chartInstance, c20_b_A, c20_ipiv, &c20_info);
  c20_b_info = c20_info;
  c20_c_info = c20_b_info;
  c20_d_info = c20_c_info;
  if (c20_d_info > 0) {
    c20_eml_warning(chartInstance);
  }

  c20_eml_scalar_eg(chartInstance);
  for (c20_i = 1; c20_i < 5; c20_i++) {
    c20_b_i = c20_i;
    if (c20_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c20_b_i), 1, 4, 1, 0) - 1] != c20_b_i) {
      c20_ip = c20_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c20_b_i), 1, 4, 1, 0) - 1];
      for (c20_j = 1; c20_j < 5; c20_j++) {
        c20_b_j = c20_j;
        c20_temp = c20_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c20_b_i), 1, 4, 1, 0) +
                            ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c20_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c20_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c20_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK
                   ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c20_b_j), 1, 4,
                    2, 0) - 1) << 2)) - 1] = c20_b_B
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c20_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c20_b_j), 1, 4, 2, 0) - 1)
             << 2)) - 1];
        c20_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c20_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                    "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c20_b_j), 1, 4,
                    2, 0) - 1) << 2)) - 1] = c20_temp;
      }
    }
  }

  for (c20_i368 = 0; c20_i368 < 16; c20_i368++) {
    c20_c_A[c20_i368] = c20_b_A[c20_i368];
  }

  c20_c_eml_xtrsm(chartInstance, c20_c_A, c20_b_B);
  for (c20_i369 = 0; c20_i369 < 16; c20_i369++) {
    c20_d_A[c20_i369] = c20_b_A[c20_i369];
  }

  c20_d_eml_xtrsm(chartInstance, c20_d_A, c20_b_B);
  c20_i370 = 0;
  for (c20_i371 = 0; c20_i371 < 4; c20_i371++) {
    c20_i372 = 0;
    for (c20_i373 = 0; c20_i373 < 4; c20_i373++) {
      c20_y[c20_i373 + c20_i370] = c20_b_B[c20_i372 + c20_i371];
      c20_i372 += 4;
    }

    c20_i370 += 4;
  }
}

static void c20_realmin(SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c20_eps(SFc20_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c20_eml_matlab_zgetrf(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_b_A[16], int32_T c20_ipiv[4],
  int32_T *c20_info)
{
  int32_T c20_i374;
  for (c20_i374 = 0; c20_i374 < 16; c20_i374++) {
    c20_b_A[c20_i374] = c20_A[c20_i374];
  }

  c20_b_eml_matlab_zgetrf(chartInstance, c20_b_A, c20_ipiv, c20_info);
}

static void c20_check_forloop_overflow_error
  (SFc20_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c20_overflow)
{
  int32_T c20_i375;
  static char_T c20_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c20_u[34];
  const mxArray *c20_y = NULL;
  int32_T c20_i376;
  static char_T c20_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c20_b_u[23];
  const mxArray *c20_b_y = NULL;
  if (!c20_overflow) {
  } else {
    for (c20_i375 = 0; c20_i375 < 34; c20_i375++) {
      c20_u[c20_i375] = c20_cv0[c20_i375];
    }

    c20_y = NULL;
    sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c20_i376 = 0; c20_i376 < 23; c20_i376++) {
      c20_b_u[c20_i376] = c20_cv1[c20_i376];
    }

    c20_b_y = NULL;
    sf_mex_assign(&c20_b_y, sf_mex_create("y", c20_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c20_y, 14, c20_b_y));
  }
}

static void c20_eml_xger(SFc20_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c20_m, int32_T c20_n, real_T c20_alpha1, int32_T c20_ix0, int32_T
  c20_iy0, real_T c20_A[16], int32_T c20_ia0, real_T c20_b_A[16])
{
  int32_T c20_i377;
  for (c20_i377 = 0; c20_i377 < 16; c20_i377++) {
    c20_b_A[c20_i377] = c20_A[c20_i377];
  }

  c20_b_eml_xger(chartInstance, c20_m, c20_n, c20_alpha1, c20_ix0, c20_iy0,
                 c20_b_A, c20_ia0);
}

static void c20_eml_warning(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c20_i378;
  static char_T c20_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c20_u[27];
  const mxArray *c20_y = NULL;
  for (c20_i378 = 0; c20_i378 < 27; c20_i378++) {
    c20_u[c20_i378] = c20_varargin_1[c20_i378];
  }

  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", c20_u, 10, 0U, 1U, 0U, 2, 1, 27),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c20_y));
}

static void c20_eml_scalar_eg(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c20_eml_xtrsm(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_B[16], real_T c20_b_B[16])
{
  int32_T c20_i379;
  int32_T c20_i380;
  real_T c20_b_A[16];
  for (c20_i379 = 0; c20_i379 < 16; c20_i379++) {
    c20_b_B[c20_i379] = c20_B[c20_i379];
  }

  for (c20_i380 = 0; c20_i380 < 16; c20_i380++) {
    c20_b_A[c20_i380] = c20_A[c20_i380];
  }

  c20_c_eml_xtrsm(chartInstance, c20_b_A, c20_b_B);
}

static void c20_below_threshold(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c20_b_eml_scalar_eg(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c20_b_eml_xtrsm(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_B[16], real_T c20_b_B[16])
{
  int32_T c20_i381;
  int32_T c20_i382;
  real_T c20_b_A[16];
  for (c20_i381 = 0; c20_i381 < 16; c20_i381++) {
    c20_b_B[c20_i381] = c20_B[c20_i381];
  }

  for (c20_i382 = 0; c20_i382 < 16; c20_i382++) {
    c20_b_A[c20_i382] = c20_A[c20_i382];
  }

  c20_d_eml_xtrsm(chartInstance, c20_b_A, c20_b_B);
}

static void c20_eye(SFc20_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c20_I[16])
{
  int32_T c20_i383;
  int32_T c20_i;
  int32_T c20_b_i;
  for (c20_i383 = 0; c20_i383 < 16; c20_i383++) {
    c20_I[c20_i383] = 0.0;
  }

  for (c20_i = 1; c20_i < 5; c20_i++) {
    c20_b_i = c20_i;
    c20_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c20_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c20_b_i), 1, 4, 2, 0) - 1)
            << 2)) - 1] = 1.0;
  }
}

static const mxArray *c20_q_sf_marshallOut(void *chartInstanceVoid, void
  *c20_inData)
{
  const mxArray *c20_mxArrayOutData = NULL;
  int32_T c20_u;
  const mxArray *c20_y = NULL;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_mxArrayOutData = NULL;
  c20_u = *(int32_T *)c20_inData;
  c20_y = NULL;
  sf_mex_assign(&c20_y, sf_mex_create("y", &c20_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c20_mxArrayOutData, c20_y, FALSE);
  return c20_mxArrayOutData;
}

static int32_T c20_ob_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  int32_T c20_y;
  int32_T c20_i384;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_i384, 1, 6, 0U, 0, 0U, 0);
  c20_y = c20_i384;
  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c20_mxArrayInData, const char_T *c20_varName, void *c20_outData)
{
  const mxArray *c20_b_sfEvent;
  const char_T *c20_identifier;
  emlrtMsgIdentifier c20_thisId;
  int32_T c20_y;
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c20_b_sfEvent = sf_mex_dup(c20_mxArrayInData);
  c20_identifier = c20_varName;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_ob_emlrt_marshallIn(chartInstance, sf_mex_dup(c20_b_sfEvent),
    &c20_thisId);
  sf_mex_destroy(&c20_b_sfEvent);
  *(int32_T *)c20_outData = c20_y;
  sf_mex_destroy(&c20_mxArrayInData);
}

static uint8_T c20_pb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c20_b_is_active_c20_torqueBalancing2012b, const
  char_T *c20_identifier)
{
  uint8_T c20_y;
  emlrtMsgIdentifier c20_thisId;
  c20_thisId.fIdentifier = c20_identifier;
  c20_thisId.fParent = NULL;
  c20_y = c20_qb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c20_b_is_active_c20_torqueBalancing2012b), &c20_thisId);
  sf_mex_destroy(&c20_b_is_active_c20_torqueBalancing2012b);
  return c20_y;
}

static uint8_T c20_qb_emlrt_marshallIn(SFc20_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c20_u, const emlrtMsgIdentifier *c20_parentId)
{
  uint8_T c20_y;
  uint8_T c20_u0;
  sf_mex_import(c20_parentId, sf_mex_dup(c20_u), &c20_u0, 1, 3, 0U, 0, 0U, 0);
  c20_y = c20_u0;
  sf_mex_destroy(&c20_u);
  return c20_y;
}

static void c20_b_eml_matlab_zgetrf(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], int32_T c20_ipiv[4], int32_T *c20_info)
{
  int32_T c20_i385;
  int32_T c20_j;
  int32_T c20_b_j;
  int32_T c20_a;
  int32_T c20_jm1;
  int32_T c20_b;
  int32_T c20_mmj;
  int32_T c20_b_a;
  int32_T c20_c;
  int32_T c20_b_b;
  int32_T c20_jj;
  int32_T c20_c_a;
  int32_T c20_jp1j;
  int32_T c20_d_a;
  int32_T c20_b_c;
  int32_T c20_n;
  int32_T c20_ix0;
  int32_T c20_b_n;
  int32_T c20_b_ix0;
  int32_T c20_c_n;
  int32_T c20_c_ix0;
  int32_T c20_idxmax;
  int32_T c20_ix;
  real_T c20_x;
  real_T c20_b_x;
  real_T c20_c_x;
  real_T c20_y;
  real_T c20_d_x;
  real_T c20_e_x;
  real_T c20_b_y;
  real_T c20_smax;
  int32_T c20_d_n;
  int32_T c20_c_b;
  int32_T c20_d_b;
  boolean_T c20_overflow;
  int32_T c20_k;
  int32_T c20_b_k;
  int32_T c20_e_a;
  real_T c20_f_x;
  real_T c20_g_x;
  real_T c20_h_x;
  real_T c20_c_y;
  real_T c20_i_x;
  real_T c20_j_x;
  real_T c20_d_y;
  real_T c20_s;
  int32_T c20_f_a;
  int32_T c20_jpiv_offset;
  int32_T c20_g_a;
  int32_T c20_e_b;
  int32_T c20_jpiv;
  int32_T c20_h_a;
  int32_T c20_f_b;
  int32_T c20_c_c;
  int32_T c20_g_b;
  int32_T c20_jrow;
  int32_T c20_i_a;
  int32_T c20_h_b;
  int32_T c20_jprow;
  int32_T c20_d_ix0;
  int32_T c20_iy0;
  int32_T c20_e_ix0;
  int32_T c20_b_iy0;
  int32_T c20_f_ix0;
  int32_T c20_c_iy0;
  int32_T c20_b_ix;
  int32_T c20_iy;
  int32_T c20_c_k;
  real_T c20_temp;
  int32_T c20_j_a;
  int32_T c20_k_a;
  int32_T c20_b_jp1j;
  int32_T c20_l_a;
  int32_T c20_d_c;
  int32_T c20_m_a;
  int32_T c20_i_b;
  int32_T c20_i386;
  int32_T c20_n_a;
  int32_T c20_j_b;
  int32_T c20_o_a;
  int32_T c20_k_b;
  boolean_T c20_b_overflow;
  int32_T c20_i;
  int32_T c20_b_i;
  real_T c20_k_x;
  real_T c20_e_y;
  real_T c20_z;
  int32_T c20_l_b;
  int32_T c20_e_c;
  int32_T c20_p_a;
  int32_T c20_f_c;
  int32_T c20_q_a;
  int32_T c20_g_c;
  int32_T c20_m;
  int32_T c20_e_n;
  int32_T c20_g_ix0;
  int32_T c20_d_iy0;
  int32_T c20_ia0;
  real_T c20_d3;
  c20_realmin(chartInstance);
  c20_eps(chartInstance);
  for (c20_i385 = 0; c20_i385 < 4; c20_i385++) {
    c20_ipiv[c20_i385] = 1 + c20_i385;
  }

  *c20_info = 0;
  for (c20_j = 1; c20_j < 4; c20_j++) {
    c20_b_j = c20_j;
    c20_a = c20_b_j - 1;
    c20_jm1 = c20_a;
    c20_b = c20_b_j;
    c20_mmj = 4 - c20_b;
    c20_b_a = c20_jm1;
    c20_c = c20_b_a * 5;
    c20_b_b = c20_c + 1;
    c20_jj = c20_b_b;
    c20_c_a = c20_jj + 1;
    c20_jp1j = c20_c_a;
    c20_d_a = c20_mmj;
    c20_b_c = c20_d_a;
    c20_n = c20_b_c + 1;
    c20_ix0 = c20_jj;
    c20_b_n = c20_n;
    c20_b_ix0 = c20_ix0;
    c20_c_n = c20_b_n;
    c20_c_ix0 = c20_b_ix0;
    if (c20_c_n < 1) {
      c20_idxmax = 0;
    } else {
      c20_idxmax = 1;
      if (c20_c_n > 1) {
        c20_ix = c20_c_ix0;
        c20_x = c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c20_ix), 1, 16, 1, 0) - 1];
        c20_b_x = c20_x;
        c20_c_x = c20_b_x;
        c20_y = muDoubleScalarAbs(c20_c_x);
        c20_d_x = 0.0;
        c20_e_x = c20_d_x;
        c20_b_y = muDoubleScalarAbs(c20_e_x);
        c20_smax = c20_y + c20_b_y;
        c20_d_n = c20_c_n;
        c20_c_b = c20_d_n;
        c20_d_b = c20_c_b;
        if (2 > c20_d_b) {
          c20_overflow = FALSE;
        } else {
          c20_overflow = (c20_d_b > 2147483646);
        }

        if (c20_overflow) {
          c20_check_forloop_overflow_error(chartInstance, c20_overflow);
        }

        for (c20_k = 2; c20_k <= c20_d_n; c20_k++) {
          c20_b_k = c20_k;
          c20_e_a = c20_ix + 1;
          c20_ix = c20_e_a;
          c20_f_x = c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c20_ix), 1, 16, 1, 0) - 1];
          c20_g_x = c20_f_x;
          c20_h_x = c20_g_x;
          c20_c_y = muDoubleScalarAbs(c20_h_x);
          c20_i_x = 0.0;
          c20_j_x = c20_i_x;
          c20_d_y = muDoubleScalarAbs(c20_j_x);
          c20_s = c20_c_y + c20_d_y;
          if (c20_s > c20_smax) {
            c20_idxmax = c20_b_k;
            c20_smax = c20_s;
          }
        }
      }
    }

    c20_f_a = c20_idxmax - 1;
    c20_jpiv_offset = c20_f_a;
    c20_g_a = c20_jj;
    c20_e_b = c20_jpiv_offset;
    c20_jpiv = c20_g_a + c20_e_b;
    if (c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c20_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if (c20_jpiv_offset != 0) {
        c20_h_a = c20_b_j;
        c20_f_b = c20_jpiv_offset;
        c20_c_c = c20_h_a + c20_f_b;
        c20_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c20_b_j), 1, 4, 1, 0) - 1] = c20_c_c;
        c20_g_b = c20_jm1 + 1;
        c20_jrow = c20_g_b;
        c20_i_a = c20_jrow;
        c20_h_b = c20_jpiv_offset;
        c20_jprow = c20_i_a + c20_h_b;
        c20_d_ix0 = c20_jrow;
        c20_iy0 = c20_jprow;
        c20_e_ix0 = c20_d_ix0;
        c20_b_iy0 = c20_iy0;
        c20_f_ix0 = c20_e_ix0;
        c20_c_iy0 = c20_b_iy0;
        c20_b_ix = c20_f_ix0;
        c20_iy = c20_c_iy0;
        for (c20_c_k = 1; c20_c_k < 5; c20_c_k++) {
          c20_temp = c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c20_b_ix), 1, 16, 1, 0) - 1];
          c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_b_ix), 1, 16, 1, 0) - 1] =
            c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_iy), 1, 16, 1, 0) - 1];
          c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_iy), 1, 16, 1, 0) - 1] = c20_temp;
          c20_j_a = c20_b_ix + 4;
          c20_b_ix = c20_j_a;
          c20_k_a = c20_iy + 4;
          c20_iy = c20_k_a;
        }
      }

      c20_b_jp1j = c20_jp1j;
      c20_l_a = c20_mmj;
      c20_d_c = c20_l_a;
      c20_m_a = c20_jp1j;
      c20_i_b = c20_d_c - 1;
      c20_i386 = c20_m_a + c20_i_b;
      c20_n_a = c20_b_jp1j;
      c20_j_b = c20_i386;
      c20_o_a = c20_n_a;
      c20_k_b = c20_j_b;
      if (c20_o_a > c20_k_b) {
        c20_b_overflow = FALSE;
      } else {
        c20_b_overflow = (c20_k_b > 2147483646);
      }

      if (c20_b_overflow) {
        c20_check_forloop_overflow_error(chartInstance, c20_b_overflow);
      }

      for (c20_i = c20_b_jp1j; c20_i <= c20_i386; c20_i++) {
        c20_b_i = c20_i;
        c20_k_x = c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c20_b_i), 1, 16, 1, 0) - 1];
        c20_e_y = c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c20_jj), 1, 16, 1, 0) - 1];
        c20_z = c20_k_x / c20_e_y;
        c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c20_b_i), 1, 16, 1, 0) - 1] = c20_z;
      }
    } else {
      *c20_info = c20_b_j;
    }

    c20_l_b = c20_b_j;
    c20_e_c = 4 - c20_l_b;
    c20_p_a = c20_jj;
    c20_f_c = c20_p_a;
    c20_q_a = c20_jj;
    c20_g_c = c20_q_a;
    c20_m = c20_mmj;
    c20_e_n = c20_e_c;
    c20_g_ix0 = c20_jp1j;
    c20_d_iy0 = c20_f_c + 4;
    c20_ia0 = c20_g_c + 5;
    c20_d3 = -1.0;
    c20_b_eml_xger(chartInstance, c20_m, c20_e_n, c20_d3, c20_g_ix0, c20_d_iy0,
                   c20_A, c20_ia0);
  }

  if (*c20_info == 0) {
    if (!(c20_A[15] != 0.0)) {
      *c20_info = 4;
    }
  }
}

static void c20_b_eml_xger(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c20_m, int32_T c20_n, real_T c20_alpha1, int32_T
  c20_ix0, int32_T c20_iy0, real_T c20_A[16], int32_T c20_ia0)
{
  int32_T c20_b_m;
  int32_T c20_b_n;
  real_T c20_b_alpha1;
  int32_T c20_b_ix0;
  int32_T c20_b_iy0;
  int32_T c20_b_ia0;
  int32_T c20_c_m;
  int32_T c20_c_n;
  real_T c20_c_alpha1;
  int32_T c20_c_ix0;
  int32_T c20_c_iy0;
  int32_T c20_c_ia0;
  int32_T c20_d_m;
  int32_T c20_d_n;
  real_T c20_d_alpha1;
  int32_T c20_d_ix0;
  int32_T c20_d_iy0;
  int32_T c20_d_ia0;
  int32_T c20_ixstart;
  int32_T c20_a;
  int32_T c20_jA;
  int32_T c20_jy;
  int32_T c20_e_n;
  int32_T c20_b;
  int32_T c20_b_b;
  boolean_T c20_overflow;
  int32_T c20_j;
  real_T c20_yjy;
  real_T c20_temp;
  int32_T c20_ix;
  int32_T c20_c_b;
  int32_T c20_i387;
  int32_T c20_b_a;
  int32_T c20_d_b;
  int32_T c20_i388;
  int32_T c20_c_a;
  int32_T c20_e_b;
  int32_T c20_d_a;
  int32_T c20_f_b;
  boolean_T c20_b_overflow;
  int32_T c20_ijA;
  int32_T c20_b_ijA;
  int32_T c20_e_a;
  int32_T c20_f_a;
  int32_T c20_g_a;
  c20_b_m = c20_m;
  c20_b_n = c20_n;
  c20_b_alpha1 = c20_alpha1;
  c20_b_ix0 = c20_ix0;
  c20_b_iy0 = c20_iy0;
  c20_b_ia0 = c20_ia0;
  c20_c_m = c20_b_m;
  c20_c_n = c20_b_n;
  c20_c_alpha1 = c20_b_alpha1;
  c20_c_ix0 = c20_b_ix0;
  c20_c_iy0 = c20_b_iy0;
  c20_c_ia0 = c20_b_ia0;
  c20_d_m = c20_c_m;
  c20_d_n = c20_c_n;
  c20_d_alpha1 = c20_c_alpha1;
  c20_d_ix0 = c20_c_ix0;
  c20_d_iy0 = c20_c_iy0;
  c20_d_ia0 = c20_c_ia0;
  if (c20_d_alpha1 == 0.0) {
  } else {
    c20_ixstart = c20_d_ix0;
    c20_a = c20_d_ia0 - 1;
    c20_jA = c20_a;
    c20_jy = c20_d_iy0;
    c20_e_n = c20_d_n;
    c20_b = c20_e_n;
    c20_b_b = c20_b;
    if (1 > c20_b_b) {
      c20_overflow = FALSE;
    } else {
      c20_overflow = (c20_b_b > 2147483646);
    }

    if (c20_overflow) {
      c20_check_forloop_overflow_error(chartInstance, c20_overflow);
    }

    for (c20_j = 1; c20_j <= c20_e_n; c20_j++) {
      c20_yjy = c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c20_jy), 1, 16, 1, 0) - 1];
      if (c20_yjy != 0.0) {
        c20_temp = c20_yjy * c20_d_alpha1;
        c20_ix = c20_ixstart;
        c20_c_b = c20_jA + 1;
        c20_i387 = c20_c_b;
        c20_b_a = c20_d_m;
        c20_d_b = c20_jA;
        c20_i388 = c20_b_a + c20_d_b;
        c20_c_a = c20_i387;
        c20_e_b = c20_i388;
        c20_d_a = c20_c_a;
        c20_f_b = c20_e_b;
        if (c20_d_a > c20_f_b) {
          c20_b_overflow = FALSE;
        } else {
          c20_b_overflow = (c20_f_b > 2147483646);
        }

        if (c20_b_overflow) {
          c20_check_forloop_overflow_error(chartInstance, c20_b_overflow);
        }

        for (c20_ijA = c20_i387; c20_ijA <= c20_i388; c20_ijA++) {
          c20_b_ijA = c20_ijA;
          c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_b_ijA), 1, 16, 1, 0) - 1] =
            c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_b_ijA), 1, 16, 1, 0) - 1] +
            c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_ix), 1, 16, 1, 0) - 1] * c20_temp;
          c20_e_a = c20_ix + 1;
          c20_ix = c20_e_a;
        }
      }

      c20_f_a = c20_jy + 4;
      c20_jy = c20_f_a;
      c20_g_a = c20_jA + 4;
      c20_jA = c20_g_a;
    }
  }
}

static void c20_c_eml_xtrsm(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_B[16])
{
  int32_T c20_j;
  int32_T c20_b_j;
  int32_T c20_a;
  int32_T c20_c;
  int32_T c20_b;
  int32_T c20_b_c;
  int32_T c20_b_b;
  int32_T c20_jBcol;
  int32_T c20_k;
  int32_T c20_b_k;
  int32_T c20_b_a;
  int32_T c20_c_c;
  int32_T c20_c_b;
  int32_T c20_d_c;
  int32_T c20_d_b;
  int32_T c20_kAcol;
  int32_T c20_c_a;
  int32_T c20_e_b;
  int32_T c20_e_c;
  int32_T c20_d_a;
  int32_T c20_i389;
  boolean_T c20_overflow;
  int32_T c20_i;
  int32_T c20_b_i;
  int32_T c20_e_a;
  int32_T c20_f_b;
  int32_T c20_f_c;
  int32_T c20_f_a;
  int32_T c20_g_b;
  int32_T c20_g_c;
  int32_T c20_g_a;
  int32_T c20_h_b;
  int32_T c20_h_c;
  int32_T c20_h_a;
  int32_T c20_i_b;
  int32_T c20_i_c;
  c20_below_threshold(chartInstance);
  c20_b_eml_scalar_eg(chartInstance);
  for (c20_j = 1; c20_j < 5; c20_j++) {
    c20_b_j = c20_j;
    c20_a = c20_b_j;
    c20_c = c20_a;
    c20_b = c20_c - 1;
    c20_b_c = c20_b << 2;
    c20_b_b = c20_b_c;
    c20_jBcol = c20_b_b;
    for (c20_k = 1; c20_k < 5; c20_k++) {
      c20_b_k = c20_k;
      c20_b_a = c20_b_k;
      c20_c_c = c20_b_a;
      c20_c_b = c20_c_c - 1;
      c20_d_c = c20_c_b << 2;
      c20_d_b = c20_d_c;
      c20_kAcol = c20_d_b;
      c20_c_a = c20_b_k;
      c20_e_b = c20_jBcol;
      c20_e_c = c20_c_a + c20_e_b;
      if (c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c20_d_a = c20_b_k;
        c20_i389 = c20_d_a;
        c20_overflow = FALSE;
        if (c20_overflow) {
          c20_check_forloop_overflow_error(chartInstance, c20_overflow);
        }

        for (c20_i = c20_i389 + 1; c20_i < 5; c20_i++) {
          c20_b_i = c20_i;
          c20_e_a = c20_b_i;
          c20_f_b = c20_jBcol;
          c20_f_c = c20_e_a + c20_f_b;
          c20_f_a = c20_b_i;
          c20_g_b = c20_jBcol;
          c20_g_c = c20_f_a + c20_g_b;
          c20_g_a = c20_b_k;
          c20_h_b = c20_jBcol;
          c20_h_c = c20_g_a + c20_h_b;
          c20_h_a = c20_b_i;
          c20_i_b = c20_kAcol;
          c20_i_c = c20_h_a + c20_i_b;
          c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_f_c), 1, 16, 1, 0) - 1] =
            c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_g_c), 1, 16, 1, 0) - 1] -
            c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_h_c), 1, 16, 1, 0) - 1] *
            c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_i_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void c20_d_eml_xtrsm(SFc20_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c20_A[16], real_T c20_B[16])
{
  int32_T c20_j;
  int32_T c20_b_j;
  int32_T c20_a;
  int32_T c20_c;
  int32_T c20_b;
  int32_T c20_b_c;
  int32_T c20_b_b;
  int32_T c20_jBcol;
  int32_T c20_k;
  int32_T c20_b_k;
  int32_T c20_b_a;
  int32_T c20_c_c;
  int32_T c20_c_b;
  int32_T c20_d_c;
  int32_T c20_d_b;
  int32_T c20_kAcol;
  int32_T c20_c_a;
  int32_T c20_e_b;
  int32_T c20_e_c;
  int32_T c20_d_a;
  int32_T c20_f_b;
  int32_T c20_f_c;
  int32_T c20_e_a;
  int32_T c20_g_b;
  int32_T c20_g_c;
  int32_T c20_f_a;
  int32_T c20_h_b;
  int32_T c20_h_c;
  real_T c20_x;
  real_T c20_y;
  real_T c20_z;
  int32_T c20_g_a;
  int32_T c20_i390;
  int32_T c20_i_b;
  int32_T c20_j_b;
  boolean_T c20_overflow;
  int32_T c20_i;
  int32_T c20_b_i;
  int32_T c20_h_a;
  int32_T c20_k_b;
  int32_T c20_i_c;
  int32_T c20_i_a;
  int32_T c20_l_b;
  int32_T c20_j_c;
  int32_T c20_j_a;
  int32_T c20_m_b;
  int32_T c20_k_c;
  int32_T c20_k_a;
  int32_T c20_n_b;
  int32_T c20_l_c;
  c20_below_threshold(chartInstance);
  c20_b_eml_scalar_eg(chartInstance);
  for (c20_j = 1; c20_j < 5; c20_j++) {
    c20_b_j = c20_j;
    c20_a = c20_b_j;
    c20_c = c20_a;
    c20_b = c20_c - 1;
    c20_b_c = c20_b << 2;
    c20_b_b = c20_b_c;
    c20_jBcol = c20_b_b;
    for (c20_k = 4; c20_k > 0; c20_k--) {
      c20_b_k = c20_k;
      c20_b_a = c20_b_k;
      c20_c_c = c20_b_a;
      c20_c_b = c20_c_c - 1;
      c20_d_c = c20_c_b << 2;
      c20_d_b = c20_d_c;
      c20_kAcol = c20_d_b;
      c20_c_a = c20_b_k;
      c20_e_b = c20_jBcol;
      c20_e_c = c20_c_a + c20_e_b;
      if (c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c20_d_a = c20_b_k;
        c20_f_b = c20_jBcol;
        c20_f_c = c20_d_a + c20_f_b;
        c20_e_a = c20_b_k;
        c20_g_b = c20_jBcol;
        c20_g_c = c20_e_a + c20_g_b;
        c20_f_a = c20_b_k;
        c20_h_b = c20_kAcol;
        c20_h_c = c20_f_a + c20_h_b;
        c20_x = c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c20_g_c), 1, 16, 1, 0) - 1];
        c20_y = c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c20_h_c), 1, 16, 1, 0) - 1];
        c20_z = c20_x / c20_y;
        c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c20_f_c), 1, 16, 1, 0) - 1] = c20_z;
        c20_g_a = c20_b_k - 1;
        c20_i390 = c20_g_a;
        c20_i_b = c20_i390;
        c20_j_b = c20_i_b;
        if (1 > c20_j_b) {
          c20_overflow = FALSE;
        } else {
          c20_overflow = (c20_j_b > 2147483646);
        }

        if (c20_overflow) {
          c20_check_forloop_overflow_error(chartInstance, c20_overflow);
        }

        for (c20_i = 1; c20_i <= c20_i390; c20_i++) {
          c20_b_i = c20_i;
          c20_h_a = c20_b_i;
          c20_k_b = c20_jBcol;
          c20_i_c = c20_h_a + c20_k_b;
          c20_i_a = c20_b_i;
          c20_l_b = c20_jBcol;
          c20_j_c = c20_i_a + c20_l_b;
          c20_j_a = c20_b_k;
          c20_m_b = c20_jBcol;
          c20_k_c = c20_j_a + c20_m_b;
          c20_k_a = c20_b_i;
          c20_n_b = c20_kAcol;
          c20_l_c = c20_k_a + c20_n_b;
          c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_i_c), 1, 16, 1, 0) - 1] =
            c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_j_c), 1, 16, 1, 0) - 1] -
            c20_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_k_c), 1, 16, 1, 0) - 1] *
            c20_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c20_l_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void init_dsm_address_info(SFc20_torqueBalancing2012bInstanceStruct
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

void sf_c20_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1111187420U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(137472521U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1410097870U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2343570812U);
}

mxArray *sf_c20_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("xcFaCjBSgOmhoBADfFt9WF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,11,3,dataFields);

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
      pr[0] = (double)(4);
      pr[1] = (double)(4);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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
      pr[0] = (double)(6);
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
      pr[0] = (double)(6);
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
      pr[0] = (double)(6);
      pr[1] = (double)(1);
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
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
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
      pr[0] = (double)(2);
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
      pr[0] = (double)(1);
      pr[1] = (double)(23);
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
      pr[0] = (double)(1);
      pr[1] = (double)(3);
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
      pr[1] = (double)(3);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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
      pr[0] = (double)(3);
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

mxArray *sf_c20_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c20_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[88],T\"CoM_Des\",},{M[1],M[43],T\"constraints\",},{M[1],M[68],T\"currentState\",},{M[1],M[66],T\"impedances\",},{M[1],M[81],T\"jointsSmoothingTime\",},{M[1],M[83],T\"kdCom\",},{M[1],M[82],T\"kpCom\",},{M[1],M[85],T\"qjDes\",},{M[1],M[71],T\"w_H_b\",},{M[4],M[0],T\"CoMprevious\",S'l','i','p'{{M1x2[355 366],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m\"}}}}",
    "100 S1x4'type','srcId','name','auxInfo'{{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[279 284],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m\"}}},{M[4],M[0],T\"tSwitch\",S'l','i','p'{{M1x2[301 308],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m\"}}},{M[4],M[0],T\"w_H_fixedLink\",S'l','i','p'{{M1x2[325 338],M[1],T\"/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/src/stateMachineStandUp.m\"}}},{M[8],M[0],T\"is_active_c20_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 14, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c20_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           20,
           1,
           1,
           22,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"qjRef");
          _SFD_SET_DATA_PROPS(1,1,1,0,"CoM");
          _SFD_SET_DATA_PROPS(2,1,1,0,"CoM_0");
          _SFD_SET_DATA_PROPS(3,1,1,0,"l_sole_H_b");
          _SFD_SET_DATA_PROPS(4,1,1,0,"l_upper_leg_contact_H_b");
          _SFD_SET_DATA_PROPS(5,2,0,1,"w_H_b");
          _SFD_SET_DATA_PROPS(6,2,0,1,"constraints");
          _SFD_SET_DATA_PROPS(7,1,1,0,"t");
          _SFD_SET_DATA_PROPS(8,2,0,1,"impedances");
          _SFD_SET_DATA_PROPS(9,2,0,1,"kpCom");
          _SFD_SET_DATA_PROPS(10,2,0,1,"kdCom");
          _SFD_SET_DATA_PROPS(11,10,0,0,"gain");
          _SFD_SET_DATA_PROPS(12,2,0,1,"currentState");
          _SFD_SET_DATA_PROPS(13,2,0,1,"jointsSmoothingTime");
          _SFD_SET_DATA_PROPS(14,10,0,0,"sm");
          _SFD_SET_DATA_PROPS(15,2,0,1,"qjDes");
          _SFD_SET_DATA_PROPS(16,2,0,1,"CoM_Des");
          _SFD_SET_DATA_PROPS(17,1,1,0,"Lwrench");
          _SFD_SET_DATA_PROPS(18,1,1,0,"Rwrench");
          _SFD_SET_DATA_PROPS(19,1,1,0,"LHandWrench");
          _SFD_SET_DATA_PROPS(20,1,1,0,"RHandWrench");
          _SFD_SET_DATA_PROPS(21,1,1,0,"useExtArmForces");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,509);
        _SFD_CV_INIT_SCRIPT(0,1,18,0,0,0,0,0,19,7);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"stateMachineStandUp",0,-1,8874);
        _SFD_CV_INIT_SCRIPT_IF(0,0,373,460,-1,623);
        _SFD_CV_INIT_SCRIPT_IF(0,1,890,903,-1,1616);
        _SFD_CV_INIT_SCRIPT_IF(0,2,1226,1249,1465,1608);
        _SFD_CV_INIT_SCRIPT_IF(0,3,1275,1381,-1,1456);
        _SFD_CV_INIT_SCRIPT_IF(0,4,1482,1502,-1,1596);
        _SFD_CV_INIT_SCRIPT_IF(0,5,1646,1659,-1,2735);
        _SFD_CV_INIT_SCRIPT_IF(0,6,2422,2523,-1,2718);
        _SFD_CV_INIT_SCRIPT_IF(0,7,2767,2780,-1,3737);
        _SFD_CV_INIT_SCRIPT_IF(0,8,3528,3628,-1,3729);
        _SFD_CV_INIT_SCRIPT_IF(0,9,3765,3778,-1,4696);
        _SFD_CV_INIT_SCRIPT_IF(0,10,4530,4563,-1,4671);
        _SFD_CV_INIT_SCRIPT_IF(0,11,4731,4744,-1,5766);
        _SFD_CV_INIT_SCRIPT_IF(0,12,5501,5602,-1,5730);
        _SFD_CV_INIT_SCRIPT_IF(0,13,5803,5816,-1,6931);
        _SFD_CV_INIT_SCRIPT_IF(0,14,6575,6675,-1,6895);
        _SFD_CV_INIT_SCRIPT_IF(0,15,6961,6974,-1,7893);
        _SFD_CV_INIT_SCRIPT_IF(0,16,7745,7760,-1,7857);
        _SFD_CV_INIT_SCRIPT_IF(0,17,7931,7944,-1,8684);

        {
          static int condStart[] = { 376, 394, 414, 440 };

          static int condEnd[] = { 390, 410, 436, 460 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2, 3, -2 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,0,376,460,4,0,&(condStart[0]),&(condEnd[0]),
            7,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1278, 1299, 1342 };

          static int condEnd[] = { 1295, 1338, 1381 };

          static int pfixExpr[] = { 0, 1, -3, 2, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,1,1278,1381,3,4,&(condStart[0]),&(condEnd[0]),
            5,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 2425, 2512 };

          static int condEnd[] = { 2508, 2523 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,2,2425,2523,2,7,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 3531, 3575, 3618 };

          static int condEnd[] = { 3570, 3614, 3628 };

          static int pfixExpr[] = { 0, 1, -3, 2, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,3,3531,3628,3,9,&(condStart[0]),&(condEnd[0]),
            5,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 4533, 4549 };

          static int condEnd[] = { 4545, 4563 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,4,4533,4563,2,12,&(condStart[0]),&(condEnd
            [0]),3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 5504, 5520, 5563 };

          static int condEnd[] = { 5516, 5559, 5602 };

          static int pfixExpr[] = { 0, 1, -3, 2, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,5,5504,5602,3,14,&(condStart[0]),&(condEnd
            [0]),5,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 6578, 6665 };

          static int condEnd[] = { 6661, 6675 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,6,6578,6675,2,17,&(condStart[0]),&(condEnd
            [0]),3,&(pfixExpr[0]));
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
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_l_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_g_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_g_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_g_sf_marshallOut,(MexInFcnForType)
            c20_g_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_f_sf_marshallOut,(MexInFcnForType)
            c20_f_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_e_sf_marshallOut,(MexInFcnForType)
            c20_e_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_d_sf_marshallOut,(MexInFcnForType)
            c20_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_d_sf_marshallOut,(MexInFcnForType)
            c20_d_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(11,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_k_sf_marshallOut,(MexInFcnForType)
          c20_i_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_c_sf_marshallOut,(MexInFcnForType)
          c20_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_c_sf_marshallOut,(MexInFcnForType)
          c20_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_j_sf_marshallOut,(MexInFcnForType)
          c20_h_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_b_sf_marshallOut,(MexInFcnForType)
            c20_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_sf_marshallOut,(MexInFcnForType)
            c20_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_i_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_i_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_i_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c20_i_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(21,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c20_h_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c20_t;
          real_T *c20_currentState;
          real_T *c20_jointsSmoothingTime;
          boolean_T *c20_useExtArmForces;
          real_T (*c20_qjRef)[23];
          real_T (*c20_CoM)[3];
          real_T (*c20_CoM_0)[3];
          real_T (*c20_l_sole_H_b)[16];
          real_T (*c20_l_upper_leg_contact_H_b)[16];
          real_T (*c20_w_H_b)[16];
          real_T (*c20_constraints)[2];
          real_T (*c20_impedances)[23];
          real_T (*c20_kpCom)[3];
          real_T (*c20_kdCom)[3];
          real_T (*c20_qjDes)[23];
          real_T (*c20_CoM_Des)[3];
          real_T (*c20_Lwrench)[6];
          real_T (*c20_Rwrench)[6];
          real_T (*c20_LHandWrench)[6];
          real_T (*c20_RHandWrench)[6];
          c20_useExtArmForces = (boolean_T *)ssGetInputPortSignal
            (chartInstance->S, 10);
          c20_RHandWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
            9);
          c20_LHandWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
            8);
          c20_Rwrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 7);
          c20_Lwrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 6);
          c20_CoM_Des = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 9);
          c20_qjDes = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 8);
          c20_jointsSmoothingTime = (real_T *)ssGetOutputPortSignal
            (chartInstance->S, 7);
          c20_currentState = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
          c20_kdCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 5);
          c20_kpCom = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 4);
          c20_impedances = (real_T (*)[23])ssGetOutputPortSignal
            (chartInstance->S, 3);
          c20_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c20_constraints = (real_T (*)[2])ssGetOutputPortSignal
            (chartInstance->S, 2);
          c20_w_H_b = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
          c20_l_upper_leg_contact_H_b = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 4);
          c20_l_sole_H_b = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            3);
          c20_CoM_0 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
          c20_CoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c20_qjRef = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c20_qjRef);
          _SFD_SET_DATA_VALUE_PTR(1U, *c20_CoM);
          _SFD_SET_DATA_VALUE_PTR(2U, *c20_CoM_0);
          _SFD_SET_DATA_VALUE_PTR(3U, *c20_l_sole_H_b);
          _SFD_SET_DATA_VALUE_PTR(4U, *c20_l_upper_leg_contact_H_b);
          _SFD_SET_DATA_VALUE_PTR(5U, *c20_w_H_b);
          _SFD_SET_DATA_VALUE_PTR(6U, *c20_constraints);
          _SFD_SET_DATA_VALUE_PTR(7U, c20_t);
          _SFD_SET_DATA_VALUE_PTR(8U, *c20_impedances);
          _SFD_SET_DATA_VALUE_PTR(9U, *c20_kpCom);
          _SFD_SET_DATA_VALUE_PTR(10U, *c20_kdCom);
          _SFD_SET_DATA_VALUE_PTR(11U, &chartInstance->c20_gain);
          _SFD_SET_DATA_VALUE_PTR(12U, c20_currentState);
          _SFD_SET_DATA_VALUE_PTR(13U, c20_jointsSmoothingTime);
          _SFD_SET_DATA_VALUE_PTR(14U, &chartInstance->c20_sm);
          _SFD_SET_DATA_VALUE_PTR(15U, *c20_qjDes);
          _SFD_SET_DATA_VALUE_PTR(16U, *c20_CoM_Des);
          _SFD_SET_DATA_VALUE_PTR(17U, *c20_Lwrench);
          _SFD_SET_DATA_VALUE_PTR(18U, *c20_Rwrench);
          _SFD_SET_DATA_VALUE_PTR(19U, *c20_LHandWrench);
          _SFD_SET_DATA_VALUE_PTR(20U, *c20_RHandWrench);
          _SFD_SET_DATA_VALUE_PTR(21U, c20_useExtArmForces);
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
  return "6QWUhsbfRLaDTkqET5sIzD";
}

static void sf_opaque_initialize_c20_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc20_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c20_torqueBalancing2012b
    ((SFc20_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c20_torqueBalancing2012b((SFc20_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c20_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c20_torqueBalancing2012b((SFc20_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c20_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c20_torqueBalancing2012b((SFc20_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c20_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c20_torqueBalancing2012b((SFc20_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c20_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c20_torqueBalancing2012b
    ((SFc20_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c20_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c20_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c20_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c20_torqueBalancing2012b
    ((SFc20_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c20_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c20_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c20_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c20_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c20_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc20_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c20_torqueBalancing2012b((SFc20_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc20_torqueBalancing2012b
    ((SFc20_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c20_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c20_torqueBalancing2012b
      ((SFc20_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c20_torqueBalancing2012b(SimStruct *S)
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
      20);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,20,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,20,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,20);
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
        infoStruct,20,11);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,20,9);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=9; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 11; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,20);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1957563056U));
  ssSetChecksum1(S,(1884295407U));
  ssSetChecksum2(S,(1477251939U));
  ssSetChecksum3(S,(1801573084U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c20_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c20_torqueBalancing2012b(SimStruct *S)
{
  SFc20_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc20_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc20_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc20_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c20_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c20_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c20_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c20_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c20_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c20_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c20_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c20_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c20_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c20_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c20_torqueBalancing2012b;
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

void c20_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c20_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c20_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c20_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c20_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
