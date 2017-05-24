/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c17_torqueBalancing2012b.h"
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
static const char * c17_debug_family_names[50] = { "nargin", "nargout",
  "constraints", "ROBOT_DOF_FOR_SIMULINK", "ConstraintsMatrix",
  "bVectorConstraints", "q", "qDes", "v", "M", "h", "H", "intHw",
  "w_H_l_contact", "w_H_r_contact", "JL", "JR", "dJLv", "dJRv", "xcom", "J_CoM",
  "desired_x_dx_ddx_CoM", "gainsPCOM", "gainsDCOM", "impedances", "intErrorCoM",
  "ki_int_qtilde", "reg", "gain", "w_H_lArm", "w_H_rArm", "LArmWrench",
  "RArmWrench", "useExtArmForces", "tauModel", "Sigma", "NA", "fHdotDesC1C2",
  "HessianMatrixQP1Foot", "gradientQP1Foot", "ConstraintsMatrixQP1Foot",
  "bVectorConstraintsQp1Foot", "HessianMatrixQP2FeetOrLegs",
  "gradientQP2FeetOrLegs", "ConstraintsMatrixQP2FeetOrLegs",
  "bVectorConstraintsQp2FeetOrLegs", "errorCoM", "qTilde", "f",
  "correctionFromSupportForce" };

static const char * c17_b_debug_family_names[4] = { "nargin", "nargout", "w",
  "S" };

static const char * c17_c_debug_family_names[5] = { "nargin", "nargout", "A",
  "regDamp", "pinvDampA" };

static const char * c17_d_debug_family_names[102] = { "f", "pos_leftFoot",
  "w_R_l_sole", "pos_rightFoot", "w_R_r_sole", "pos_leftArm", "pos_rightArm",
  "gainsICOM", "dampings", "ROBOT_DOF", "gravAcc", "m", "Mb", "Mbj", "Mj", "St",
  "gravityWrench", "xDcom", "qD", "Pr", "Pl", "PlArm", "PrArm", "AL", "AR", "A",
  "pinvA", "A_lArm", "A_rArm", "A_arms", "fArms", "fsupport", "xDDcomStar",
  "H_desired", "H_error", "alpha", "H_errParallel", "Jc", "JcDv", "JcMinv",
  "JcMinvSt", "JcMinvJct", "JBar", "PInv_JcMinvSt", "nullJcMinvSt", "Mbar",
  "NLMbar", "constraintMatrixLeftFoot", "constraintMatrixRightFoot",
  "ConstraintsMatrix2FeetOrLegs", "bVectorConstraints2FeetOrLegs", "HDotDes",
  "SigmaNA", "A1Foot", "impedances", "nargin", "nargout", "constraints",
  "ConstraintsMatrix", "bVectorConstraints", "q", "qDes", "v", "M", "h", "H",
  "intHw", "w_H_l_contact", "w_H_r_contact", "JL", "JR", "dJLv", "dJRv", "xcom",
  "J_CoM", "desired_x_dx_ddx_CoM", "gainsPCOM", "gainsDCOM", "intErrorCoM",
  "ki_int_qtilde", "reg", "gain", "w_H_lArm", "w_H_rArm", "LArmWrench",
  "RArmWrench", "useExtArmForces", "tauModel", "Sigma", "NA", "f_HDot",
  "HessianMatrixQP1Foot", "gradientQP1Foot", "ConstraintsMatrixQP1Foot",
  "bVectorConstraintsQp1Foot", "HessianMatrixQP2FeetOrLegs",
  "gradientQP2FeetOrLegs", "ConstraintsMatrixQP2FeetOrLegs",
  "bVectorConstraintsQp2FeetOrLegs", "errorCoM", "qTilde",
  "correctionFromSupportForce" };

/* Function Declarations */
static void initialize_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void c17_update_debugger_state_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c17_st);
static void finalize_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c17_torqueBalancing2012b(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_chartstep_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void initSimStructsc17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void c17_balancingController(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_constraints[2], real_T c17_ConstraintsMatrix[114],
  real_T c17_bVectorConstraints[19], real_T c17_q[23], real_T c17_qDes[23],
  real_T c17_v[29], real_T c17_M[841], real_T c17_h[29], real_T c17_H[6], real_T
  c17_intHw[3], real_T c17_w_H_l_contact[16], real_T c17_w_H_r_contact[16],
  real_T c17_JL[174], real_T c17_JR[174], real_T c17_dJLv[6], real_T c17_dJRv[6],
  real_T c17_xcom[3], real_T c17_J_CoM[174], real_T c17_desired_x_dx_ddx_CoM[9],
  real_T c17_gainsPCOM[3], real_T c17_gainsDCOM[3], real_T c17_impedances[23],
  real_T c17_intErrorCoM[3], real_T c17_ki_int_qtilde[23],
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_b_reg, c17_struct_kzTB0QQWoOlMoMhgKf6sK *
  c17_b_gain, real_T c17_w_H_lArm[16], real_T c17_w_H_rArm[16], real_T
  c17_LArmWrench[6], real_T c17_RArmWrench[6], boolean_T c17_useExtArmForces,
  real_T c17_tauModel[23], real_T c17_Sigma[276], real_T c17_NA[144], real_T
  c17_f_HDot[12], real_T c17_HessianMatrixQP1Foot[36], real_T
  c17_gradientQP1Foot[6], real_T c17_ConstraintsMatrixQP1Foot[114], real_T
  c17_bVectorConstraintsQp1Foot[19], real_T c17_HessianMatrixQP2FeetOrLegs[144],
  real_T c17_gradientQP2FeetOrLegs[12], real_T
  c17_ConstraintsMatrixQP2FeetOrLegs[456], real_T
  c17_bVectorConstraintsQp2FeetOrLegs[38], real_T c17_errorCoM[3], real_T
  c17_qTilde[23], real_T c17_correctionFromSupportForce[6]);
static void c17_Sf(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                   real_T c17_w[3], real_T c17_S[9]);
static void c17_pinvDamped(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_regDamp, real_T c17_pinvDampA
  [276]);
static void init_script_number_translation(uint32_T c17_machineNumber, uint32_T
  c17_chartNumber);
static void c17_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_sprintf, const char_T *c17_identifier,
  char_T c17_y[14]);
static void c17_b_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  char_T c17_y[14]);
static const mxArray *c17_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_c_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_correctionFromSupportForce, const char_T
  *c17_identifier, real_T c17_y[6]);
static void c17_d_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[6]);
static void c17_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_b_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_e_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_f, const char_T *c17_identifier, real_T
  c17_y[12]);
static void c17_f_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[12]);
static void c17_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_c_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_g_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_qTilde, const char_T *c17_identifier,
  real_T c17_y[23]);
static void c17_h_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[23]);
static void c17_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_d_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_i_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_errorCoM, const char_T *c17_identifier,
  real_T c17_y[3]);
static void c17_j_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[3]);
static void c17_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_e_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_k_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_bVectorConstraintsQp2FeetOrLegs, const
  char_T *c17_identifier, real_T c17_y[38]);
static void c17_l_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[38]);
static void c17_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_f_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_m_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_ConstraintsMatrixQP2FeetOrLegs, const
  char_T *c17_identifier, real_T c17_y[456]);
static void c17_n_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[456]);
static void c17_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_g_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_o_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_HessianMatrixQP2FeetOrLegs, const char_T
  *c17_identifier, real_T c17_y[144]);
static void c17_p_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[144]);
static void c17_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_h_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_q_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_bVectorConstraintsQp1Foot, const char_T
  *c17_identifier, real_T c17_y[19]);
static void c17_r_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[19]);
static void c17_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_i_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_s_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_ConstraintsMatrixQP1Foot, const char_T
  *c17_identifier, real_T c17_y[114]);
static void c17_t_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[114]);
static void c17_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_j_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_u_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_HessianMatrixQP1Foot, const char_T
  *c17_identifier, real_T c17_y[36]);
static void c17_v_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[36]);
static void c17_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_k_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_w_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_Sigma, const char_T *c17_identifier, real_T
  c17_y[276]);
static void c17_x_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[276]);
static void c17_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_l_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_m_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_n_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_y_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  c17_struct_kzTB0QQWoOlMoMhgKf6sK *c17_y);
static real_T c17_ab_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId);
static void c17_bb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[9]);
static void c17_cb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[23]);
static void c17_db_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[4]);
static void c17_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_o_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_eb_emlrt_marshallIn
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c17_u,
   const emlrtMsgIdentifier *c17_parentId);
static void c17_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_p_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_q_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_r_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_s_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_t_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_u_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_v_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_w_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static void c17_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_x_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_fb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[276]);
static void c17_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static boolean_T c17_gb_emlrt_marshallIn
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c17_u,
   const emlrtMsgIdentifier *c17_parentId);
static void c17_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static void c17_hb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[16]);
static void c17_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static void c17_ib_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[174]);
static void c17_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static void c17_jb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[29]);
static void c17_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static void c17_kb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[841]);
static void c17_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static void c17_lb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[2]);
static void c17_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static void c17_mb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[529]);
static void c17_w_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_y_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_nb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[348]);
static void c17_x_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_ab_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_ob_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[72]);
static void c17_y_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_bb_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_pb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[72]);
static void c17_ab_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_cb_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static const mxArray *c17_db_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_qb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[138]);
static void c17_bb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static const mxArray *c17_eb_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static void c17_cb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static void c17_info_helper(c17_ResolvedFunctionInfo c17_info[302]);
static void c17_b_info_helper(c17_ResolvedFunctionInfo c17_info[302]);
static void c17_c_info_helper(c17_ResolvedFunctionInfo c17_info[302]);
static void c17_d_info_helper(c17_ResolvedFunctionInfo c17_info[302]);
static void c17_e_info_helper(c17_ResolvedFunctionInfo c17_info[302]);
static void c17_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_b_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static boolean_T c17_eml_use_refblas(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_eye(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c17_I[9]);
static void c17_pinv(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c17_A[72], real_T c17_tol, real_T c17_X[72]);
static void c17_c_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static boolean_T c17_isfinite(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x);
static void c17_eml_error(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_eml_xgesvd(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[72], real_T c17_U[72], real_T c17_S[6], real_T
  c17_V[36]);
static real_T c17_eml_xnrm2(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[72], int32_T c17_ix0);
static real_T c17_abs(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                      real_T c17_x);
static void c17_realmin(SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void c17_check_forloop_overflow_error
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c17_overflow);
static real_T c17_eml_div(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x, real_T c17_y);
static void c17_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[72], int32_T c17_ix0,
  real_T c17_b_x[72]);
static real_T c17_eml_xdotc(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[72], int32_T c17_ix0, real_T
  c17_y[72], int32_T c17_iy0);
static void c17_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[72],
  int32_T c17_iy0, real_T c17_b_y[72]);
static real_T c17_b_eml_xnrm2(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[6], int32_T c17_ix0);
static void c17_b_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[6], int32_T c17_ix0,
  real_T c17_b_x[6]);
static void c17_b_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[72], int32_T c17_ix0,
  real_T c17_y[12], int32_T c17_iy0, real_T c17_b_y[12]);
static void c17_c_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[12], int32_T c17_ix0,
  real_T c17_y[72], int32_T c17_iy0, real_T c17_b_y[72]);
static real_T c17_b_eml_xdotc(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[36], int32_T c17_ix0, real_T
  c17_y[36], int32_T c17_iy0);
static void c17_d_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_d_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[36],
  int32_T c17_iy0, real_T c17_b_y[36]);
static void c17_e_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_c_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[72], int32_T c17_ix0, real_T
  c17_b_x[72]);
static void c17_d_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[36], int32_T c17_ix0, real_T
  c17_b_x[36]);
static void c17_eps(SFc17_torqueBalancing2012bInstanceStruct *chartInstance);
static void c17_b_eml_error(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static real_T c17_sqrt(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x);
static void c17_c_eml_error(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_eml_xrotg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_b, real_T *c17_b_a, real_T *c17_b_b,
  real_T *c17_c, real_T *c17_s);
static void c17_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x[36], int32_T c17_ix0, int32_T c17_iy0, real_T c17_c, real_T c17_s,
  real_T c17_b_x[36]);
static void c17_b_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[72], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s, real_T c17_b_x[72]);
static void c17_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[36], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_b_x[36]);
static void c17_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_b_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[72], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_b_x[72]);
static void c17_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_k, real_T c17_A[36], real_T c17_B[72], real_T
  c17_C[72], real_T c17_b_C[72]);
static void c17_inv(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c17_x[36], real_T c17_y[36]);
static void c17_invNxN(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x[36], real_T c17_y[36]);
static void c17_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_b_A[36], int32_T c17_ipiv[6],
  int32_T *c17_info);
static void c17_eml_xger(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T c17_ix0, int32_T
  c17_iy0, real_T c17_A[36], int32_T c17_ia0, real_T c17_b_A[36]);
static void c17_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[36], real_T c17_b_B[36]);
static real_T c17_norm(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x[36]);
static void c17_eml_warning(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_b_eml_warning(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, char_T c17_varargin_2[14]);
static void c17_f_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_g_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static real_T c17_b_norm(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x[6]);
static void c17_b_eye(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                      real_T c17_I[144]);
static void c17_h_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_b_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[72], real_T c17_B[72], real_T c17_C[144], real_T
  c17_b_C[144]);
static void c17_mrdivide(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_A[348], real_T c17_B[841], real_T c17_y[348]);
static void c17_b_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_b_A[841], int32_T c17_ipiv[29],
  int32_T *c17_info);
static void c17_c_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[841], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_b_x[841]);
static void c17_b_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_b_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[841], int32_T c17_ia0, real_T c17_b_A
  [841]);
static void c17_i_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_b_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_B[348], real_T c17_b_B[348]);
static void c17_c_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_c_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_B[348], real_T c17_b_B[348]);
static void c17_j_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_c_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[348], real_T c17_B[667], real_T c17_C[276],
  real_T c17_b_C[276]);
static void c17_k_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_d_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[348], real_T c17_B[348], real_T c17_C[144],
  real_T c17_b_C[144]);
static void c17_b_mrdivide(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[36], real_T c17_y[138]);
static void c17_d_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[138], real_T c17_b_B[138]);
static void c17_d_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_e_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[138], real_T c17_b_B[138]);
static void c17_l_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_e_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[72], real_T c17_C[276], real_T
  c17_b_C[276]);
static void c17_m_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_f_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[276], real_T c17_C[144],
  real_T c17_b_C[144]);
static void c17_c_mrdivide(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[144], real_T c17_y[276]);
static void c17_c_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_b_A[144], int32_T c17_ipiv[12],
  int32_T *c17_info);
static void c17_c_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[144], int32_T c17_ia0, real_T c17_b_A
  [144]);
static void c17_f_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_B[276], real_T c17_b_B[276]);
static void c17_e_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_g_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_B[276], real_T c17_b_B[276]);
static void c17_c_eye(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                      real_T c17_I[529]);
static void c17_n_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_g_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[276], real_T c17_C[529],
  real_T c17_b_C[529]);
static void c17_o_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_p_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_h_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[138], real_T c17_C[529],
  real_T c17_b_C[529]);
static void c17_q_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_i_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[529], real_T c17_C[529],
  real_T c17_b_C[529]);
static void c17_diag(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c17_v[23], real_T c17_d[529]);
static void c17_b_pinv(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_A[529], real_T c17_tol, real_T c17_X[529]);
static void c17_svd(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c17_A[529], real_T c17_U[529], real_T c17_S[529],
                    real_T c17_V[529]);
static void c17_b_eml_xgesvd(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_U[529], real_T c17_S[23], real_T
  c17_V[529]);
static real_T c17_c_eml_xnrm2(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[529], int32_T c17_ix0);
static void c17_e_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[529], int32_T
  c17_ix0, real_T c17_b_x[529]);
static real_T c17_c_eml_xdotc(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[529], int32_T c17_ix0, real_T
  c17_y[529], int32_T c17_iy0);
static void c17_e_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[529],
  int32_T c17_iy0, real_T c17_b_y[529]);
static real_T c17_d_eml_xnrm2(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[23], int32_T c17_ix0);
static void c17_f_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[23], int32_T c17_ix0,
  real_T c17_b_x[23]);
static void c17_f_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[529], int32_T
  c17_ix0, real_T c17_y[23], int32_T c17_iy0, real_T c17_b_y[23]);
static void c17_g_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[23], int32_T c17_ix0,
  real_T c17_y[529], int32_T c17_iy0, real_T c17_b_y[529]);
static void c17_g_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[529], int32_T c17_ix0, real_T
  c17_b_x[529]);
static void c17_c_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[529], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s, real_T c17_b_x[529]);
static void c17_f_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_d_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[529], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_b_x[529]);
static void c17_g_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_j_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_k, real_T c17_A[529], real_T c17_B[529], real_T
  c17_C[529], real_T c17_b_C[529]);
static void c17_h_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_b_diag(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_v[23], real_T c17_d[529]);
static void c17_blkdiag(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_varargin_1[9], real_T c17_varargin_2[9], real_T c17_y[36]);
static void c17_r_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_k_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[114], real_T c17_B[36], real_T c17_C[114], real_T
  c17_b_C[114]);
static void c17_b_blkdiag(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_varargin_1[114], real_T c17_varargin_2[114], real_T
  c17_y[456]);
static void c17_s_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_t_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_u_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_v_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_l_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[23], real_T c17_C[23], real_T
  c17_b_C[23]);
static void c17_w_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_m_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[144], real_T c17_C[276],
  real_T c17_b_C[276]);
static void c17_x_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_n_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[276], real_T c17_C[276],
  real_T c17_b_C[276]);
static void c17_y_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_ab_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_o_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[456], real_T c17_B[144], real_T c17_C[456],
  real_T c17_b_C[456]);
static void c17_bb_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_cb_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c17_db_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c17_fb_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData);
static int32_T c17_rb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId);
static void c17_db_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData);
static uint8_T c17_sb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c17_b_is_active_c17_torqueBalancing2012b, const
  char_T *c17_identifier);
static uint8_T c17_tb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId);
static void c17_h_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[72], int32_T c17_ix0);
static void c17_h_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[72],
  int32_T c17_iy0);
static void c17_i_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[6], int32_T c17_ix0);
static void c17_i_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[72], int32_T c17_ix0,
  real_T c17_y[12], int32_T c17_iy0);
static void c17_j_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[12], int32_T c17_ix0,
  real_T c17_y[72], int32_T c17_iy0);
static void c17_k_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[36],
  int32_T c17_iy0);
static void c17_j_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[72], int32_T c17_ix0);
static void c17_k_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[36], int32_T c17_ix0);
static void c17_b_sqrt(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T *c17_x);
static void c17_b_eml_xrotg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T *c17_a, real_T *c17_b, real_T *c17_c, real_T *c17_s);
static void c17_d_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[36], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s);
static void c17_e_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[72], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s);
static void c17_e_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[36], int32_T c17_ix0, int32_T c17_iy0);
static void c17_f_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[72], int32_T c17_ix0, int32_T c17_iy0);
static void c17_p_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_k, real_T c17_A[36], real_T c17_B[72], real_T
  c17_C[72]);
static void c17_d_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], int32_T c17_ipiv[6], int32_T *c17_info);
static void c17_d_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[36], int32_T c17_ia0);
static void c17_h_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[36]);
static void c17_q_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[72], real_T c17_B[72], real_T c17_C[144]);
static void c17_e_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], int32_T c17_ipiv[29], int32_T *c17_info);
static void c17_g_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[841], int32_T c17_ix0, int32_T c17_iy0);
static void c17_e_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[841], int32_T c17_ia0);
static void c17_i_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_B[348]);
static void c17_j_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_B[348]);
static void c17_r_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[348], real_T c17_B[667], real_T c17_C[276]);
static void c17_s_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[348], real_T c17_B[348], real_T c17_C[144]);
static void c17_k_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[138]);
static void c17_l_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[138]);
static void c17_t_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[72], real_T c17_C[276]);
static void c17_u_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[276], real_T c17_C[144]);
static void c17_f_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], int32_T c17_ipiv[12], int32_T *c17_info);
static void c17_f_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[144], int32_T c17_ia0);
static void c17_m_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_B[276]);
static void c17_n_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_B[276]);
static void c17_v_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[276], real_T c17_C[529]);
static void c17_w_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[138], real_T c17_C[529]);
static void c17_x_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[529], real_T c17_C[529]);
static void c17_l_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[529], int32_T
  c17_ix0);
static void c17_l_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[529],
  int32_T c17_iy0);
static void c17_m_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[23], int32_T c17_ix0);
static void c17_m_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[529], int32_T
  c17_ix0, real_T c17_y[23], int32_T c17_iy0);
static void c17_n_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[23], int32_T c17_ix0,
  real_T c17_y[529], int32_T c17_iy0);
static void c17_n_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[529], int32_T c17_ix0);
static void c17_f_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[529], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s);
static void c17_h_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[529], int32_T c17_ix0, int32_T c17_iy0);
static void c17_y_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_k, real_T c17_A[529], real_T c17_B[529], real_T
  c17_C[529]);
static void c17_ab_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[114], real_T c17_B[36], real_T c17_C[114]);
static void c17_bb_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[23], real_T c17_C[23]);
static void c17_cb_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[144], real_T c17_C[276]);
static void c17_db_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[276], real_T c17_C[276]);
static void c17_eb_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[456], real_T c17_B[144], real_T c17_C[456]);
static void init_dsm_address_info(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c17_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c17_is_active_c17_torqueBalancing2012b = 0U;
}

static void initialize_params_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c17_m0 = NULL;
  const mxArray *c17_mxField;
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_r0;
  const mxArray *c17_m1 = NULL;
  const mxArray *c17_b_mxField;
  c17_struct_kzTB0QQWoOlMoMhgKf6sK c17_r1;
  sf_set_error_prefix_string(
    "Error evaluating data 'reg' in the parent workspace.\n");
  c17_m0 = sf_mex_get_sfun_param(chartInstance->S, 1, 1);
  c17_mxField = sf_mex_getfield(c17_m0, "pinvTol", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c17_mxField), &c17_r0.pinvTol, 1, 0, 0U,
                      0, 0U, 0);
  c17_mxField = sf_mex_getfield(c17_m0, "pinvDamp", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c17_mxField), &c17_r0.pinvDamp, 1, 0, 0U,
                      0, 0U, 0);
  c17_mxField = sf_mex_getfield(c17_m0, "pinvDampVb", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c17_mxField), &c17_r0.pinvDampVb, 1, 0,
                      0U, 0, 0U, 0);
  c17_mxField = sf_mex_getfield(c17_m0, "HessianQP", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c17_mxField), &c17_r0.HessianQP, 1, 0,
                      0U, 0, 0U, 0);
  c17_mxField = sf_mex_getfield(c17_m0, "impedances", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c17_mxField), &c17_r0.impedances, 1, 0,
                      0U, 0, 0U, 0);
  c17_mxField = sf_mex_getfield(c17_m0, "dampings", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c17_mxField), &c17_r0.dampings, 1, 0, 0U,
                      0, 0U, 0);
  c17_mxField = sf_mex_getfield(c17_m0, "norm_tolerance", "reg", 0);
  sf_mex_import_named("reg", sf_mex_dup(c17_mxField), &c17_r0.norm_tolerance, 1,
                      0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c17_m0);
  chartInstance->c17_reg = c17_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'gain' in the parent workspace.\n");
  c17_m1 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c17_b_mxField = sf_mex_getfield(c17_m1, "qTildeMax", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), &c17_r1.qTildeMax, 1, 0,
                      0U, 0, 0U, 0);
  c17_b_mxField = sf_mex_getfield(c17_m1, "SmoothingTimeImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField),
                      &c17_r1.SmoothingTimeImp, 1, 0, 0U, 0, 0U, 0);
  c17_b_mxField = sf_mex_getfield(c17_m1, "SmoothingTimeGainScheduling", "gain",
    0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField),
                      &c17_r1.SmoothingTimeGainScheduling, 1, 0, 0U, 0, 0U, 0);
  c17_b_mxField = sf_mex_getfield(c17_m1, "PCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), c17_r1.PCOM, 1, 0, 0U,
                      1, 0U, 2, 3, 3);
  c17_b_mxField = sf_mex_getfield(c17_m1, "ICOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), c17_r1.ICOM, 1, 0, 0U,
                      1, 0U, 2, 3, 3);
  c17_b_mxField = sf_mex_getfield(c17_m1, "DCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), c17_r1.DCOM, 1, 0, 0U,
                      1, 0U, 2, 3, 3);
  c17_b_mxField = sf_mex_getfield(c17_m1, "PAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField),
                      &c17_r1.PAngularMomentum, 1, 0, 0U, 0, 0U, 0);
  c17_b_mxField = sf_mex_getfield(c17_m1, "DAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField),
                      &c17_r1.DAngularMomentum, 1, 0, 0U, 0, 0U, 0);
  c17_b_mxField = sf_mex_getfield(c17_m1, "integral", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), c17_r1.integral, 1, 0,
                      0U, 1, 0U, 2, 1, 23);
  c17_b_mxField = sf_mex_getfield(c17_m1, "impedances", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), c17_r1.impedances, 1, 0,
                      0U, 1, 0U, 2, 1, 23);
  c17_b_mxField = sf_mex_getfield(c17_m1, "dampings", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), c17_r1.dampings, 1, 0,
                      0U, 1, 0U, 2, 1, 23);
  c17_b_mxField = sf_mex_getfield(c17_m1, "increasingRatesImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField),
                      c17_r1.increasingRatesImp, 1, 0, 0U, 1, 0U, 2, 1, 23);
  c17_b_mxField = sf_mex_getfield(c17_m1, "footSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), c17_r1.footSize, 1, 0,
                      0U, 1, 0U, 2, 2, 2);
  c17_b_mxField = sf_mex_getfield(c17_m1, "legSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c17_b_mxField), c17_r1.legSize, 1, 0,
                      0U, 1, 0U, 2, 2, 2);
  sf_mex_destroy(&c17_m1);
  chartInstance->c17_gain = c17_r1;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c17_update_debugger_state_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c17_st;
  const mxArray *c17_y = NULL;
  int32_T c17_i0;
  real_T c17_u[114];
  const mxArray *c17_b_y = NULL;
  int32_T c17_i1;
  real_T c17_b_u[456];
  const mxArray *c17_c_y = NULL;
  int32_T c17_i2;
  real_T c17_c_u[36];
  const mxArray *c17_d_y = NULL;
  int32_T c17_i3;
  real_T c17_d_u[144];
  const mxArray *c17_e_y = NULL;
  int32_T c17_i4;
  real_T c17_e_u[144];
  const mxArray *c17_f_y = NULL;
  int32_T c17_i5;
  real_T c17_f_u[276];
  const mxArray *c17_g_y = NULL;
  int32_T c17_i6;
  real_T c17_g_u[19];
  const mxArray *c17_h_y = NULL;
  int32_T c17_i7;
  real_T c17_h_u[38];
  const mxArray *c17_i_y = NULL;
  int32_T c17_i8;
  real_T c17_i_u[6];
  const mxArray *c17_j_y = NULL;
  int32_T c17_i9;
  real_T c17_j_u[3];
  const mxArray *c17_k_y = NULL;
  int32_T c17_i10;
  real_T c17_k_u[12];
  const mxArray *c17_l_y = NULL;
  int32_T c17_i11;
  real_T c17_l_u[12];
  const mxArray *c17_m_y = NULL;
  int32_T c17_i12;
  real_T c17_m_u[6];
  const mxArray *c17_n_y = NULL;
  int32_T c17_i13;
  real_T c17_n_u[12];
  const mxArray *c17_o_y = NULL;
  int32_T c17_i14;
  real_T c17_o_u[23];
  const mxArray *c17_p_y = NULL;
  int32_T c17_i15;
  real_T c17_p_u[23];
  const mxArray *c17_q_y = NULL;
  uint8_T c17_hoistedGlobal;
  uint8_T c17_q_u;
  const mxArray *c17_r_y = NULL;
  real_T (*c17_tauModel)[23];
  real_T (*c17_qTilde)[23];
  real_T (*c17_gradientQP2FeetOrLegs)[12];
  real_T (*c17_gradientQP1Foot)[6];
  real_T (*c17_fHdotDesC1C2)[12];
  real_T (*c17_f)[12];
  real_T (*c17_errorCoM)[3];
  real_T (*c17_correctionFromSupportForce)[6];
  real_T (*c17_bVectorConstraintsQp2FeetOrLegs)[38];
  real_T (*c17_bVectorConstraintsQp1Foot)[19];
  real_T (*c17_Sigma)[276];
  real_T (*c17_NA)[144];
  real_T (*c17_HessianMatrixQP2FeetOrLegs)[144];
  real_T (*c17_HessianMatrixQP1Foot)[36];
  real_T (*c17_ConstraintsMatrixQP2FeetOrLegs)[456];
  real_T (*c17_ConstraintsMatrixQP1Foot)[114];
  c17_correctionFromSupportForce = (real_T (*)[6])ssGetOutputPortSignal
    (chartInstance->S, 16);
  c17_f = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 15);
  c17_qTilde = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 14);
  c17_errorCoM = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 13);
  c17_bVectorConstraintsQp2FeetOrLegs = (real_T (*)[38])ssGetOutputPortSignal
    (chartInstance->S, 12);
  c17_ConstraintsMatrixQP2FeetOrLegs = (real_T (*)[456])ssGetOutputPortSignal
    (chartInstance->S, 11);
  c17_gradientQP2FeetOrLegs = (real_T (*)[12])ssGetOutputPortSignal
    (chartInstance->S, 10);
  c17_HessianMatrixQP2FeetOrLegs = (real_T (*)[144])ssGetOutputPortSignal
    (chartInstance->S, 9);
  c17_bVectorConstraintsQp1Foot = (real_T (*)[19])ssGetOutputPortSignal
    (chartInstance->S, 8);
  c17_ConstraintsMatrixQP1Foot = (real_T (*)[114])ssGetOutputPortSignal
    (chartInstance->S, 7);
  c17_gradientQP1Foot = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 6);
  c17_HessianMatrixQP1Foot = (real_T (*)[36])ssGetOutputPortSignal
    (chartInstance->S, 5);
  c17_fHdotDesC1C2 = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 4);
  c17_NA = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 3);
  c17_Sigma = (real_T (*)[276])ssGetOutputPortSignal(chartInstance->S, 2);
  c17_tauModel = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 1);
  c17_st = NULL;
  c17_st = NULL;
  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_createcellarray(17), FALSE);
  for (c17_i0 = 0; c17_i0 < 114; c17_i0++) {
    c17_u[c17_i0] = (*c17_ConstraintsMatrixQP1Foot)[c17_i0];
  }

  c17_b_y = NULL;
  sf_mex_assign(&c17_b_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 19, 6),
                FALSE);
  sf_mex_setcell(c17_y, 0, c17_b_y);
  for (c17_i1 = 0; c17_i1 < 456; c17_i1++) {
    c17_b_u[c17_i1] = (*c17_ConstraintsMatrixQP2FeetOrLegs)[c17_i1];
  }

  c17_c_y = NULL;
  sf_mex_assign(&c17_c_y, sf_mex_create("y", c17_b_u, 0, 0U, 1U, 0U, 2, 38, 12),
                FALSE);
  sf_mex_setcell(c17_y, 1, c17_c_y);
  for (c17_i2 = 0; c17_i2 < 36; c17_i2++) {
    c17_c_u[c17_i2] = (*c17_HessianMatrixQP1Foot)[c17_i2];
  }

  c17_d_y = NULL;
  sf_mex_assign(&c17_d_y, sf_mex_create("y", c17_c_u, 0, 0U, 1U, 0U, 2, 6, 6),
                FALSE);
  sf_mex_setcell(c17_y, 2, c17_d_y);
  for (c17_i3 = 0; c17_i3 < 144; c17_i3++) {
    c17_d_u[c17_i3] = (*c17_HessianMatrixQP2FeetOrLegs)[c17_i3];
  }

  c17_e_y = NULL;
  sf_mex_assign(&c17_e_y, sf_mex_create("y", c17_d_u, 0, 0U, 1U, 0U, 2, 12, 12),
                FALSE);
  sf_mex_setcell(c17_y, 3, c17_e_y);
  for (c17_i4 = 0; c17_i4 < 144; c17_i4++) {
    c17_e_u[c17_i4] = (*c17_NA)[c17_i4];
  }

  c17_f_y = NULL;
  sf_mex_assign(&c17_f_y, sf_mex_create("y", c17_e_u, 0, 0U, 1U, 0U, 2, 12, 12),
                FALSE);
  sf_mex_setcell(c17_y, 4, c17_f_y);
  for (c17_i5 = 0; c17_i5 < 276; c17_i5++) {
    c17_f_u[c17_i5] = (*c17_Sigma)[c17_i5];
  }

  c17_g_y = NULL;
  sf_mex_assign(&c17_g_y, sf_mex_create("y", c17_f_u, 0, 0U, 1U, 0U, 2, 23, 12),
                FALSE);
  sf_mex_setcell(c17_y, 5, c17_g_y);
  for (c17_i6 = 0; c17_i6 < 19; c17_i6++) {
    c17_g_u[c17_i6] = (*c17_bVectorConstraintsQp1Foot)[c17_i6];
  }

  c17_h_y = NULL;
  sf_mex_assign(&c17_h_y, sf_mex_create("y", c17_g_u, 0, 0U, 1U, 0U, 1, 19),
                FALSE);
  sf_mex_setcell(c17_y, 6, c17_h_y);
  for (c17_i7 = 0; c17_i7 < 38; c17_i7++) {
    c17_h_u[c17_i7] = (*c17_bVectorConstraintsQp2FeetOrLegs)[c17_i7];
  }

  c17_i_y = NULL;
  sf_mex_assign(&c17_i_y, sf_mex_create("y", c17_h_u, 0, 0U, 1U, 0U, 1, 38),
                FALSE);
  sf_mex_setcell(c17_y, 7, c17_i_y);
  for (c17_i8 = 0; c17_i8 < 6; c17_i8++) {
    c17_i_u[c17_i8] = (*c17_correctionFromSupportForce)[c17_i8];
  }

  c17_j_y = NULL;
  sf_mex_assign(&c17_j_y, sf_mex_create("y", c17_i_u, 0, 0U, 1U, 0U, 1, 6),
                FALSE);
  sf_mex_setcell(c17_y, 8, c17_j_y);
  for (c17_i9 = 0; c17_i9 < 3; c17_i9++) {
    c17_j_u[c17_i9] = (*c17_errorCoM)[c17_i9];
  }

  c17_k_y = NULL;
  sf_mex_assign(&c17_k_y, sf_mex_create("y", c17_j_u, 0, 0U, 1U, 0U, 1, 3),
                FALSE);
  sf_mex_setcell(c17_y, 9, c17_k_y);
  for (c17_i10 = 0; c17_i10 < 12; c17_i10++) {
    c17_k_u[c17_i10] = (*c17_f)[c17_i10];
  }

  c17_l_y = NULL;
  sf_mex_assign(&c17_l_y, sf_mex_create("y", c17_k_u, 0, 0U, 1U, 0U, 1, 12),
                FALSE);
  sf_mex_setcell(c17_y, 10, c17_l_y);
  for (c17_i11 = 0; c17_i11 < 12; c17_i11++) {
    c17_l_u[c17_i11] = (*c17_fHdotDesC1C2)[c17_i11];
  }

  c17_m_y = NULL;
  sf_mex_assign(&c17_m_y, sf_mex_create("y", c17_l_u, 0, 0U, 1U, 0U, 1, 12),
                FALSE);
  sf_mex_setcell(c17_y, 11, c17_m_y);
  for (c17_i12 = 0; c17_i12 < 6; c17_i12++) {
    c17_m_u[c17_i12] = (*c17_gradientQP1Foot)[c17_i12];
  }

  c17_n_y = NULL;
  sf_mex_assign(&c17_n_y, sf_mex_create("y", c17_m_u, 0, 0U, 1U, 0U, 1, 6),
                FALSE);
  sf_mex_setcell(c17_y, 12, c17_n_y);
  for (c17_i13 = 0; c17_i13 < 12; c17_i13++) {
    c17_n_u[c17_i13] = (*c17_gradientQP2FeetOrLegs)[c17_i13];
  }

  c17_o_y = NULL;
  sf_mex_assign(&c17_o_y, sf_mex_create("y", c17_n_u, 0, 0U, 1U, 0U, 1, 12),
                FALSE);
  sf_mex_setcell(c17_y, 13, c17_o_y);
  for (c17_i14 = 0; c17_i14 < 23; c17_i14++) {
    c17_o_u[c17_i14] = (*c17_qTilde)[c17_i14];
  }

  c17_p_y = NULL;
  sf_mex_assign(&c17_p_y, sf_mex_create("y", c17_o_u, 0, 0U, 1U, 0U, 1, 23),
                FALSE);
  sf_mex_setcell(c17_y, 14, c17_p_y);
  for (c17_i15 = 0; c17_i15 < 23; c17_i15++) {
    c17_p_u[c17_i15] = (*c17_tauModel)[c17_i15];
  }

  c17_q_y = NULL;
  sf_mex_assign(&c17_q_y, sf_mex_create("y", c17_p_u, 0, 0U, 1U, 0U, 1, 23),
                FALSE);
  sf_mex_setcell(c17_y, 15, c17_q_y);
  c17_hoistedGlobal = chartInstance->c17_is_active_c17_torqueBalancing2012b;
  c17_q_u = c17_hoistedGlobal;
  c17_r_y = NULL;
  sf_mex_assign(&c17_r_y, sf_mex_create("y", &c17_q_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c17_y, 16, c17_r_y);
  sf_mex_assign(&c17_st, c17_y, FALSE);
  return c17_st;
}

static void set_sim_state_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c17_st)
{
  const mxArray *c17_u;
  real_T c17_dv0[114];
  int32_T c17_i16;
  real_T c17_dv1[456];
  int32_T c17_i17;
  real_T c17_dv2[36];
  int32_T c17_i18;
  real_T c17_dv3[144];
  int32_T c17_i19;
  real_T c17_dv4[144];
  int32_T c17_i20;
  real_T c17_dv5[276];
  int32_T c17_i21;
  real_T c17_dv6[19];
  int32_T c17_i22;
  real_T c17_dv7[38];
  int32_T c17_i23;
  real_T c17_dv8[6];
  int32_T c17_i24;
  real_T c17_dv9[3];
  int32_T c17_i25;
  real_T c17_dv10[12];
  int32_T c17_i26;
  real_T c17_dv11[12];
  int32_T c17_i27;
  real_T c17_dv12[6];
  int32_T c17_i28;
  real_T c17_dv13[12];
  int32_T c17_i29;
  real_T c17_dv14[23];
  int32_T c17_i30;
  real_T c17_dv15[23];
  int32_T c17_i31;
  real_T (*c17_ConstraintsMatrixQP1Foot)[114];
  real_T (*c17_ConstraintsMatrixQP2FeetOrLegs)[456];
  real_T (*c17_HessianMatrixQP1Foot)[36];
  real_T (*c17_HessianMatrixQP2FeetOrLegs)[144];
  real_T (*c17_NA)[144];
  real_T (*c17_Sigma)[276];
  real_T (*c17_bVectorConstraintsQp1Foot)[19];
  real_T (*c17_bVectorConstraintsQp2FeetOrLegs)[38];
  real_T (*c17_correctionFromSupportForce)[6];
  real_T (*c17_errorCoM)[3];
  real_T (*c17_f)[12];
  real_T (*c17_fHdotDesC1C2)[12];
  real_T (*c17_gradientQP1Foot)[6];
  real_T (*c17_gradientQP2FeetOrLegs)[12];
  real_T (*c17_qTilde)[23];
  real_T (*c17_tauModel)[23];
  c17_correctionFromSupportForce = (real_T (*)[6])ssGetOutputPortSignal
    (chartInstance->S, 16);
  c17_f = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 15);
  c17_qTilde = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 14);
  c17_errorCoM = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 13);
  c17_bVectorConstraintsQp2FeetOrLegs = (real_T (*)[38])ssGetOutputPortSignal
    (chartInstance->S, 12);
  c17_ConstraintsMatrixQP2FeetOrLegs = (real_T (*)[456])ssGetOutputPortSignal
    (chartInstance->S, 11);
  c17_gradientQP2FeetOrLegs = (real_T (*)[12])ssGetOutputPortSignal
    (chartInstance->S, 10);
  c17_HessianMatrixQP2FeetOrLegs = (real_T (*)[144])ssGetOutputPortSignal
    (chartInstance->S, 9);
  c17_bVectorConstraintsQp1Foot = (real_T (*)[19])ssGetOutputPortSignal
    (chartInstance->S, 8);
  c17_ConstraintsMatrixQP1Foot = (real_T (*)[114])ssGetOutputPortSignal
    (chartInstance->S, 7);
  c17_gradientQP1Foot = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 6);
  c17_HessianMatrixQP1Foot = (real_T (*)[36])ssGetOutputPortSignal
    (chartInstance->S, 5);
  c17_fHdotDesC1C2 = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 4);
  c17_NA = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 3);
  c17_Sigma = (real_T (*)[276])ssGetOutputPortSignal(chartInstance->S, 2);
  c17_tauModel = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c17_doneDoubleBufferReInit = TRUE;
  c17_u = sf_mex_dup(c17_st);
  c17_s_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 0)),
    "ConstraintsMatrixQP1Foot", c17_dv0);
  for (c17_i16 = 0; c17_i16 < 114; c17_i16++) {
    (*c17_ConstraintsMatrixQP1Foot)[c17_i16] = c17_dv0[c17_i16];
  }

  c17_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 1)),
    "ConstraintsMatrixQP2FeetOrLegs", c17_dv1);
  for (c17_i17 = 0; c17_i17 < 456; c17_i17++) {
    (*c17_ConstraintsMatrixQP2FeetOrLegs)[c17_i17] = c17_dv1[c17_i17];
  }

  c17_u_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 2)),
    "HessianMatrixQP1Foot", c17_dv2);
  for (c17_i18 = 0; c17_i18 < 36; c17_i18++) {
    (*c17_HessianMatrixQP1Foot)[c17_i18] = c17_dv2[c17_i18];
  }

  c17_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 3)),
    "HessianMatrixQP2FeetOrLegs", c17_dv3);
  for (c17_i19 = 0; c17_i19 < 144; c17_i19++) {
    (*c17_HessianMatrixQP2FeetOrLegs)[c17_i19] = c17_dv3[c17_i19];
  }

  c17_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 4)),
    "NA", c17_dv4);
  for (c17_i20 = 0; c17_i20 < 144; c17_i20++) {
    (*c17_NA)[c17_i20] = c17_dv4[c17_i20];
  }

  c17_w_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 5)),
    "Sigma", c17_dv5);
  for (c17_i21 = 0; c17_i21 < 276; c17_i21++) {
    (*c17_Sigma)[c17_i21] = c17_dv5[c17_i21];
  }

  c17_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 6)),
    "bVectorConstraintsQp1Foot", c17_dv6);
  for (c17_i22 = 0; c17_i22 < 19; c17_i22++) {
    (*c17_bVectorConstraintsQp1Foot)[c17_i22] = c17_dv6[c17_i22];
  }

  c17_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 7)),
    "bVectorConstraintsQp2FeetOrLegs", c17_dv7);
  for (c17_i23 = 0; c17_i23 < 38; c17_i23++) {
    (*c17_bVectorConstraintsQp2FeetOrLegs)[c17_i23] = c17_dv7[c17_i23];
  }

  c17_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 8)),
    "correctionFromSupportForce", c17_dv8);
  for (c17_i24 = 0; c17_i24 < 6; c17_i24++) {
    (*c17_correctionFromSupportForce)[c17_i24] = c17_dv8[c17_i24];
  }

  c17_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 9)),
    "errorCoM", c17_dv9);
  for (c17_i25 = 0; c17_i25 < 3; c17_i25++) {
    (*c17_errorCoM)[c17_i25] = c17_dv9[c17_i25];
  }

  c17_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 10)),
    "f", c17_dv10);
  for (c17_i26 = 0; c17_i26 < 12; c17_i26++) {
    (*c17_f)[c17_i26] = c17_dv10[c17_i26];
  }

  c17_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 11)),
    "fHdotDesC1C2", c17_dv11);
  for (c17_i27 = 0; c17_i27 < 12; c17_i27++) {
    (*c17_fHdotDesC1C2)[c17_i27] = c17_dv11[c17_i27];
  }

  c17_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 12)),
    "gradientQP1Foot", c17_dv12);
  for (c17_i28 = 0; c17_i28 < 6; c17_i28++) {
    (*c17_gradientQP1Foot)[c17_i28] = c17_dv12[c17_i28];
  }

  c17_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 13)),
    "gradientQP2FeetOrLegs", c17_dv13);
  for (c17_i29 = 0; c17_i29 < 12; c17_i29++) {
    (*c17_gradientQP2FeetOrLegs)[c17_i29] = c17_dv13[c17_i29];
  }

  c17_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 14)),
    "qTilde", c17_dv14);
  for (c17_i30 = 0; c17_i30 < 23; c17_i30++) {
    (*c17_qTilde)[c17_i30] = c17_dv14[c17_i30];
  }

  c17_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 15)),
    "tauModel", c17_dv15);
  for (c17_i31 = 0; c17_i31 < 23; c17_i31++) {
    (*c17_tauModel)[c17_i31] = c17_dv15[c17_i31];
  }

  chartInstance->c17_is_active_c17_torqueBalancing2012b =
    c17_sb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c17_u, 16)),
    "is_active_c17_torqueBalancing2012b");
  sf_mex_destroy(&c17_u);
  c17_update_debugger_state_c17_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c17_st);
}

static void finalize_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c17_torqueBalancing2012b(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c17_i32;
  int32_T c17_i33;
  int32_T c17_i34;
  int32_T c17_i35;
  int32_T c17_i36;
  int32_T c17_i37;
  int32_T c17_i38;
  int32_T c17_i39;
  int32_T c17_i40;
  int32_T c17_i41;
  int32_T c17_i42;
  int32_T c17_i43;
  int32_T c17_i44;
  int32_T c17_i45;
  int32_T c17_i46;
  int32_T c17_i47;
  int32_T c17_i48;
  int32_T c17_i49;
  int32_T c17_i50;
  int32_T c17_i51;
  int32_T c17_i52;
  int32_T c17_i53;
  int32_T c17_i54;
  int32_T c17_i55;
  int32_T c17_i56;
  int32_T c17_i57;
  int32_T c17_i58;
  int32_T c17_i59;
  int32_T c17_i60;
  int32_T c17_i61;
  int32_T c17_i62;
  int32_T c17_i63;
  int32_T c17_i64;
  int32_T c17_i65;
  int32_T c17_i66;
  int32_T c17_i67;
  int32_T c17_i68;
  int32_T c17_i69;
  int32_T c17_i70;
  int32_T c17_i71;
  int32_T c17_i72;
  int32_T c17_i73;
  int32_T c17_i74;
  int32_T c17_i75;
  int32_T c17_i76;
  boolean_T *c17_useExtArmForces;
  real_T (*c17_correctionFromSupportForce)[6];
  real_T (*c17_RArmWrench)[6];
  real_T (*c17_LArmWrench)[6];
  real_T (*c17_w_H_rArm)[16];
  real_T (*c17_w_H_lArm)[16];
  real_T (*c17_f)[12];
  real_T (*c17_qTilde)[23];
  real_T (*c17_errorCoM)[3];
  real_T (*c17_ki_int_qtilde)[23];
  real_T (*c17_intErrorCoM)[3];
  real_T (*c17_impedances)[23];
  real_T (*c17_gainsDCOM)[3];
  real_T (*c17_gainsPCOM)[3];
  real_T (*c17_desired_x_dx_ddx_CoM)[9];
  real_T (*c17_J_CoM)[174];
  real_T (*c17_xcom)[3];
  real_T (*c17_dJRv)[6];
  real_T (*c17_dJLv)[6];
  real_T (*c17_JR)[174];
  real_T (*c17_JL)[174];
  real_T (*c17_w_H_r_contact)[16];
  real_T (*c17_w_H_l_contact)[16];
  real_T (*c17_intHw)[3];
  real_T (*c17_H)[6];
  real_T (*c17_h)[29];
  real_T (*c17_M)[841];
  real_T (*c17_v)[29];
  real_T (*c17_qDes)[23];
  real_T (*c17_q)[23];
  real_T (*c17_bVectorConstraints)[19];
  real_T (*c17_ConstraintsMatrix)[114];
  real_T (*c17_ROBOT_DOF_FOR_SIMULINK)[529];
  real_T (*c17_constraints)[2];
  real_T (*c17_bVectorConstraintsQp2FeetOrLegs)[38];
  real_T (*c17_ConstraintsMatrixQP2FeetOrLegs)[456];
  real_T (*c17_gradientQP2FeetOrLegs)[12];
  real_T (*c17_HessianMatrixQP2FeetOrLegs)[144];
  real_T (*c17_bVectorConstraintsQp1Foot)[19];
  real_T (*c17_ConstraintsMatrixQP1Foot)[114];
  real_T (*c17_gradientQP1Foot)[6];
  real_T (*c17_HessianMatrixQP1Foot)[36];
  real_T (*c17_fHdotDesC1C2)[12];
  real_T (*c17_NA)[144];
  real_T (*c17_Sigma)[276];
  real_T (*c17_tauModel)[23];
  c17_correctionFromSupportForce = (real_T (*)[6])ssGetOutputPortSignal
    (chartInstance->S, 16);
  c17_useExtArmForces = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 29);
  c17_RArmWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 28);
  c17_LArmWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 27);
  c17_w_H_rArm = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 26);
  c17_w_H_lArm = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 25);
  c17_f = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 15);
  c17_qTilde = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 14);
  c17_errorCoM = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 13);
  c17_ki_int_qtilde = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 24);
  c17_intErrorCoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 23);
  c17_impedances = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 22);
  c17_gainsDCOM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 21);
  c17_gainsPCOM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 20);
  c17_desired_x_dx_ddx_CoM = (real_T (*)[9])ssGetInputPortSignal
    (chartInstance->S, 19);
  c17_J_CoM = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 18);
  c17_xcom = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 17);
  c17_dJRv = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 16);
  c17_dJLv = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 15);
  c17_JR = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 14);
  c17_JL = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 13);
  c17_w_H_r_contact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 12);
  c17_w_H_l_contact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 11);
  c17_intHw = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 10);
  c17_H = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 9);
  c17_h = (real_T (*)[29])ssGetInputPortSignal(chartInstance->S, 8);
  c17_M = (real_T (*)[841])ssGetInputPortSignal(chartInstance->S, 7);
  c17_v = (real_T (*)[29])ssGetInputPortSignal(chartInstance->S, 6);
  c17_qDes = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 5);
  c17_q = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 4);
  c17_bVectorConstraints = (real_T (*)[19])ssGetInputPortSignal(chartInstance->S,
    3);
  c17_ConstraintsMatrix = (real_T (*)[114])ssGetInputPortSignal(chartInstance->S,
    2);
  c17_ROBOT_DOF_FOR_SIMULINK = (real_T (*)[529])ssGetInputPortSignal
    (chartInstance->S, 1);
  c17_constraints = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  c17_bVectorConstraintsQp2FeetOrLegs = (real_T (*)[38])ssGetOutputPortSignal
    (chartInstance->S, 12);
  c17_ConstraintsMatrixQP2FeetOrLegs = (real_T (*)[456])ssGetOutputPortSignal
    (chartInstance->S, 11);
  c17_gradientQP2FeetOrLegs = (real_T (*)[12])ssGetOutputPortSignal
    (chartInstance->S, 10);
  c17_HessianMatrixQP2FeetOrLegs = (real_T (*)[144])ssGetOutputPortSignal
    (chartInstance->S, 9);
  c17_bVectorConstraintsQp1Foot = (real_T (*)[19])ssGetOutputPortSignal
    (chartInstance->S, 8);
  c17_ConstraintsMatrixQP1Foot = (real_T (*)[114])ssGetOutputPortSignal
    (chartInstance->S, 7);
  c17_gradientQP1Foot = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 6);
  c17_HessianMatrixQP1Foot = (real_T (*)[36])ssGetOutputPortSignal
    (chartInstance->S, 5);
  c17_fHdotDesC1C2 = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 4);
  c17_NA = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 3);
  c17_Sigma = (real_T (*)[276])ssGetOutputPortSignal(chartInstance->S, 2);
  c17_tauModel = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 16U, chartInstance->c17_sfEvent);
  for (c17_i32 = 0; c17_i32 < 23; c17_i32++) {
    _SFD_DATA_RANGE_CHECK((*c17_tauModel)[c17_i32], 0U);
  }

  for (c17_i33 = 0; c17_i33 < 276; c17_i33++) {
    _SFD_DATA_RANGE_CHECK((*c17_Sigma)[c17_i33], 1U);
  }

  for (c17_i34 = 0; c17_i34 < 144; c17_i34++) {
    _SFD_DATA_RANGE_CHECK((*c17_NA)[c17_i34], 2U);
  }

  for (c17_i35 = 0; c17_i35 < 12; c17_i35++) {
    _SFD_DATA_RANGE_CHECK((*c17_fHdotDesC1C2)[c17_i35], 3U);
  }

  for (c17_i36 = 0; c17_i36 < 36; c17_i36++) {
    _SFD_DATA_RANGE_CHECK((*c17_HessianMatrixQP1Foot)[c17_i36], 4U);
  }

  for (c17_i37 = 0; c17_i37 < 6; c17_i37++) {
    _SFD_DATA_RANGE_CHECK((*c17_gradientQP1Foot)[c17_i37], 5U);
  }

  for (c17_i38 = 0; c17_i38 < 114; c17_i38++) {
    _SFD_DATA_RANGE_CHECK((*c17_ConstraintsMatrixQP1Foot)[c17_i38], 6U);
  }

  for (c17_i39 = 0; c17_i39 < 19; c17_i39++) {
    _SFD_DATA_RANGE_CHECK((*c17_bVectorConstraintsQp1Foot)[c17_i39], 7U);
  }

  for (c17_i40 = 0; c17_i40 < 144; c17_i40++) {
    _SFD_DATA_RANGE_CHECK((*c17_HessianMatrixQP2FeetOrLegs)[c17_i40], 8U);
  }

  for (c17_i41 = 0; c17_i41 < 12; c17_i41++) {
    _SFD_DATA_RANGE_CHECK((*c17_gradientQP2FeetOrLegs)[c17_i41], 9U);
  }

  for (c17_i42 = 0; c17_i42 < 456; c17_i42++) {
    _SFD_DATA_RANGE_CHECK((*c17_ConstraintsMatrixQP2FeetOrLegs)[c17_i42], 10U);
  }

  for (c17_i43 = 0; c17_i43 < 38; c17_i43++) {
    _SFD_DATA_RANGE_CHECK((*c17_bVectorConstraintsQp2FeetOrLegs)[c17_i43], 11U);
  }

  for (c17_i44 = 0; c17_i44 < 2; c17_i44++) {
    _SFD_DATA_RANGE_CHECK((*c17_constraints)[c17_i44], 12U);
  }

  for (c17_i45 = 0; c17_i45 < 529; c17_i45++) {
    _SFD_DATA_RANGE_CHECK((*c17_ROBOT_DOF_FOR_SIMULINK)[c17_i45], 13U);
  }

  for (c17_i46 = 0; c17_i46 < 114; c17_i46++) {
    _SFD_DATA_RANGE_CHECK((*c17_ConstraintsMatrix)[c17_i46], 14U);
  }

  for (c17_i47 = 0; c17_i47 < 19; c17_i47++) {
    _SFD_DATA_RANGE_CHECK((*c17_bVectorConstraints)[c17_i47], 15U);
  }

  for (c17_i48 = 0; c17_i48 < 23; c17_i48++) {
    _SFD_DATA_RANGE_CHECK((*c17_q)[c17_i48], 16U);
  }

  for (c17_i49 = 0; c17_i49 < 23; c17_i49++) {
    _SFD_DATA_RANGE_CHECK((*c17_qDes)[c17_i49], 17U);
  }

  for (c17_i50 = 0; c17_i50 < 29; c17_i50++) {
    _SFD_DATA_RANGE_CHECK((*c17_v)[c17_i50], 18U);
  }

  for (c17_i51 = 0; c17_i51 < 841; c17_i51++) {
    _SFD_DATA_RANGE_CHECK((*c17_M)[c17_i51], 19U);
  }

  for (c17_i52 = 0; c17_i52 < 29; c17_i52++) {
    _SFD_DATA_RANGE_CHECK((*c17_h)[c17_i52], 20U);
  }

  for (c17_i53 = 0; c17_i53 < 6; c17_i53++) {
    _SFD_DATA_RANGE_CHECK((*c17_H)[c17_i53], 21U);
  }

  for (c17_i54 = 0; c17_i54 < 3; c17_i54++) {
    _SFD_DATA_RANGE_CHECK((*c17_intHw)[c17_i54], 22U);
  }

  for (c17_i55 = 0; c17_i55 < 16; c17_i55++) {
    _SFD_DATA_RANGE_CHECK((*c17_w_H_l_contact)[c17_i55], 23U);
  }

  for (c17_i56 = 0; c17_i56 < 16; c17_i56++) {
    _SFD_DATA_RANGE_CHECK((*c17_w_H_r_contact)[c17_i56], 24U);
  }

  for (c17_i57 = 0; c17_i57 < 174; c17_i57++) {
    _SFD_DATA_RANGE_CHECK((*c17_JL)[c17_i57], 25U);
  }

  for (c17_i58 = 0; c17_i58 < 174; c17_i58++) {
    _SFD_DATA_RANGE_CHECK((*c17_JR)[c17_i58], 26U);
  }

  for (c17_i59 = 0; c17_i59 < 6; c17_i59++) {
    _SFD_DATA_RANGE_CHECK((*c17_dJLv)[c17_i59], 27U);
  }

  for (c17_i60 = 0; c17_i60 < 6; c17_i60++) {
    _SFD_DATA_RANGE_CHECK((*c17_dJRv)[c17_i60], 28U);
  }

  for (c17_i61 = 0; c17_i61 < 3; c17_i61++) {
    _SFD_DATA_RANGE_CHECK((*c17_xcom)[c17_i61], 29U);
  }

  for (c17_i62 = 0; c17_i62 < 174; c17_i62++) {
    _SFD_DATA_RANGE_CHECK((*c17_J_CoM)[c17_i62], 30U);
  }

  for (c17_i63 = 0; c17_i63 < 9; c17_i63++) {
    _SFD_DATA_RANGE_CHECK((*c17_desired_x_dx_ddx_CoM)[c17_i63], 31U);
  }

  for (c17_i64 = 0; c17_i64 < 3; c17_i64++) {
    _SFD_DATA_RANGE_CHECK((*c17_gainsPCOM)[c17_i64], 32U);
  }

  for (c17_i65 = 0; c17_i65 < 3; c17_i65++) {
    _SFD_DATA_RANGE_CHECK((*c17_gainsDCOM)[c17_i65], 33U);
  }

  for (c17_i66 = 0; c17_i66 < 23; c17_i66++) {
    _SFD_DATA_RANGE_CHECK((*c17_impedances)[c17_i66], 34U);
  }

  for (c17_i67 = 0; c17_i67 < 3; c17_i67++) {
    _SFD_DATA_RANGE_CHECK((*c17_intErrorCoM)[c17_i67], 35U);
  }

  for (c17_i68 = 0; c17_i68 < 23; c17_i68++) {
    _SFD_DATA_RANGE_CHECK((*c17_ki_int_qtilde)[c17_i68], 36U);
  }

  for (c17_i69 = 0; c17_i69 < 3; c17_i69++) {
    _SFD_DATA_RANGE_CHECK((*c17_errorCoM)[c17_i69], 39U);
  }

  for (c17_i70 = 0; c17_i70 < 23; c17_i70++) {
    _SFD_DATA_RANGE_CHECK((*c17_qTilde)[c17_i70], 40U);
  }

  for (c17_i71 = 0; c17_i71 < 12; c17_i71++) {
    _SFD_DATA_RANGE_CHECK((*c17_f)[c17_i71], 41U);
  }

  for (c17_i72 = 0; c17_i72 < 16; c17_i72++) {
    _SFD_DATA_RANGE_CHECK((*c17_w_H_lArm)[c17_i72], 42U);
  }

  for (c17_i73 = 0; c17_i73 < 16; c17_i73++) {
    _SFD_DATA_RANGE_CHECK((*c17_w_H_rArm)[c17_i73], 43U);
  }

  for (c17_i74 = 0; c17_i74 < 6; c17_i74++) {
    _SFD_DATA_RANGE_CHECK((*c17_LArmWrench)[c17_i74], 44U);
  }

  for (c17_i75 = 0; c17_i75 < 6; c17_i75++) {
    _SFD_DATA_RANGE_CHECK((*c17_RArmWrench)[c17_i75], 45U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)*c17_useExtArmForces, 46U);
  for (c17_i76 = 0; c17_i76 < 6; c17_i76++) {
    _SFD_DATA_RANGE_CHECK((*c17_correctionFromSupportForce)[c17_i76], 47U);
  }

  chartInstance->c17_sfEvent = CALL_EVENT;
  c17_chartstep_c17_torqueBalancing2012b(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c17_chartstep_c17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
  boolean_T c17_hoistedGlobal;
  int32_T c17_i77;
  real_T c17_constraints[2];
  int32_T c17_i78;
  static real_T c17_ROBOT_DOF_FOR_SIMULINK[529];
  int32_T c17_i79;
  real_T c17_ConstraintsMatrix[114];
  int32_T c17_i80;
  real_T c17_bVectorConstraints[19];
  int32_T c17_i81;
  real_T c17_q[23];
  int32_T c17_i82;
  real_T c17_qDes[23];
  int32_T c17_i83;
  real_T c17_v[29];
  int32_T c17_i84;
  static real_T c17_M[841];
  int32_T c17_i85;
  real_T c17_h[29];
  int32_T c17_i86;
  real_T c17_H[6];
  int32_T c17_i87;
  real_T c17_intHw[3];
  int32_T c17_i88;
  real_T c17_w_H_l_contact[16];
  int32_T c17_i89;
  real_T c17_w_H_r_contact[16];
  int32_T c17_i90;
  real_T c17_JL[174];
  int32_T c17_i91;
  real_T c17_JR[174];
  int32_T c17_i92;
  real_T c17_dJLv[6];
  int32_T c17_i93;
  real_T c17_dJRv[6];
  int32_T c17_i94;
  real_T c17_xcom[3];
  int32_T c17_i95;
  real_T c17_J_CoM[174];
  int32_T c17_i96;
  real_T c17_desired_x_dx_ddx_CoM[9];
  int32_T c17_i97;
  real_T c17_gainsPCOM[3];
  int32_T c17_i98;
  real_T c17_gainsDCOM[3];
  int32_T c17_i99;
  real_T c17_impedances[23];
  int32_T c17_i100;
  real_T c17_intErrorCoM[3];
  int32_T c17_i101;
  real_T c17_ki_int_qtilde[23];
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_b_reg;
  c17_struct_kzTB0QQWoOlMoMhgKf6sK c17_b_gain;
  int32_T c17_i102;
  real_T c17_w_H_lArm[16];
  int32_T c17_i103;
  real_T c17_w_H_rArm[16];
  int32_T c17_i104;
  real_T c17_LArmWrench[6];
  int32_T c17_i105;
  real_T c17_RArmWrench[6];
  boolean_T c17_useExtArmForces;
  uint32_T c17_debug_family_var_map[50];
  real_T c17_nargin = 32.0;
  real_T c17_nargout = 16.0;
  real_T c17_tauModel[23];
  real_T c17_Sigma[276];
  real_T c17_NA[144];
  real_T c17_fHdotDesC1C2[12];
  real_T c17_HessianMatrixQP1Foot[36];
  real_T c17_gradientQP1Foot[6];
  real_T c17_ConstraintsMatrixQP1Foot[114];
  real_T c17_bVectorConstraintsQp1Foot[19];
  real_T c17_HessianMatrixQP2FeetOrLegs[144];
  real_T c17_gradientQP2FeetOrLegs[12];
  static real_T c17_ConstraintsMatrixQP2FeetOrLegs[456];
  real_T c17_bVectorConstraintsQp2FeetOrLegs[38];
  real_T c17_errorCoM[3];
  real_T c17_qTilde[23];
  real_T c17_f[12];
  real_T c17_correctionFromSupportForce[6];
  int32_T c17_i106;
  real_T c17_b_constraints[2];
  int32_T c17_i107;
  real_T c17_b_ConstraintsMatrix[114];
  int32_T c17_i108;
  real_T c17_b_bVectorConstraints[19];
  int32_T c17_i109;
  real_T c17_b_q[23];
  int32_T c17_i110;
  real_T c17_b_qDes[23];
  int32_T c17_i111;
  real_T c17_b_v[29];
  int32_T c17_i112;
  static real_T c17_b_M[841];
  int32_T c17_i113;
  real_T c17_b_h[29];
  int32_T c17_i114;
  real_T c17_b_H[6];
  int32_T c17_i115;
  real_T c17_b_intHw[3];
  int32_T c17_i116;
  real_T c17_b_w_H_l_contact[16];
  int32_T c17_i117;
  real_T c17_b_w_H_r_contact[16];
  int32_T c17_i118;
  real_T c17_b_JL[174];
  int32_T c17_i119;
  real_T c17_b_JR[174];
  int32_T c17_i120;
  real_T c17_b_dJLv[6];
  int32_T c17_i121;
  real_T c17_b_dJRv[6];
  int32_T c17_i122;
  real_T c17_b_xcom[3];
  int32_T c17_i123;
  real_T c17_b_J_CoM[174];
  int32_T c17_i124;
  real_T c17_b_desired_x_dx_ddx_CoM[9];
  int32_T c17_i125;
  real_T c17_b_gainsPCOM[3];
  int32_T c17_i126;
  real_T c17_b_gainsDCOM[3];
  int32_T c17_i127;
  real_T c17_b_impedances[23];
  int32_T c17_i128;
  real_T c17_b_intErrorCoM[3];
  int32_T c17_i129;
  real_T c17_b_ki_int_qtilde[23];
  c17_struct_kzTB0QQWoOlMoMhgKf6sK c17_c_gain;
  int32_T c17_i130;
  real_T c17_b_w_H_lArm[16];
  int32_T c17_i131;
  real_T c17_b_w_H_rArm[16];
  int32_T c17_i132;
  real_T c17_b_LArmWrench[6];
  int32_T c17_i133;
  real_T c17_b_RArmWrench[6];
  real_T c17_b_correctionFromSupportForce[6];
  real_T c17_b_qTilde[23];
  real_T c17_b_errorCoM[3];
  real_T c17_b_bVectorConstraintsQp2FeetOrLegs[38];
  static real_T c17_b_ConstraintsMatrixQP2FeetOrLegs[456];
  real_T c17_b_gradientQP2FeetOrLegs[12];
  real_T c17_b_HessianMatrixQP2FeetOrLegs[144];
  real_T c17_b_bVectorConstraintsQp1Foot[19];
  real_T c17_b_ConstraintsMatrixQP1Foot[114];
  real_T c17_b_gradientQP1Foot[6];
  real_T c17_b_HessianMatrixQP1Foot[36];
  real_T c17_b_fHdotDesC1C2[12];
  real_T c17_b_NA[144];
  real_T c17_b_Sigma[276];
  real_T c17_b_tauModel[23];
  int32_T c17_i134;
  int32_T c17_i135;
  int32_T c17_i136;
  int32_T c17_i137;
  int32_T c17_i138;
  int32_T c17_i139;
  int32_T c17_i140;
  int32_T c17_i141;
  int32_T c17_i142;
  int32_T c17_i143;
  int32_T c17_i144;
  int32_T c17_i145;
  int32_T c17_i146;
  int32_T c17_i147;
  int32_T c17_i148;
  int32_T c17_i149;
  int32_T c17_i150;
  int32_T c17_i151;
  int32_T c17_i152;
  int32_T c17_i153;
  int32_T c17_i154;
  int32_T c17_i155;
  int32_T c17_i156;
  int32_T c17_i157;
  int32_T c17_i158;
  int32_T c17_i159;
  int32_T c17_i160;
  int32_T c17_i161;
  int32_T c17_i162;
  int32_T c17_i163;
  int32_T c17_i164;
  int32_T c17_i165;
  boolean_T *c17_b_useExtArmForces;
  real_T (*c17_c_tauModel)[23];
  real_T (*c17_c_Sigma)[276];
  real_T (*c17_c_NA)[144];
  real_T (*c17_c_fHdotDesC1C2)[12];
  real_T (*c17_c_HessianMatrixQP1Foot)[36];
  real_T (*c17_c_gradientQP1Foot)[6];
  real_T (*c17_c_ConstraintsMatrixQP1Foot)[114];
  real_T (*c17_c_bVectorConstraintsQp1Foot)[19];
  real_T (*c17_c_HessianMatrixQP2FeetOrLegs)[144];
  real_T (*c17_c_gradientQP2FeetOrLegs)[12];
  real_T (*c17_c_ConstraintsMatrixQP2FeetOrLegs)[456];
  real_T (*c17_c_bVectorConstraintsQp2FeetOrLegs)[38];
  real_T (*c17_c_errorCoM)[3];
  real_T (*c17_c_qTilde)[23];
  real_T (*c17_b_f)[12];
  real_T (*c17_c_correctionFromSupportForce)[6];
  real_T (*c17_c_RArmWrench)[6];
  real_T (*c17_c_LArmWrench)[6];
  real_T (*c17_c_w_H_rArm)[16];
  real_T (*c17_c_w_H_lArm)[16];
  real_T (*c17_c_ki_int_qtilde)[23];
  real_T (*c17_c_intErrorCoM)[3];
  real_T (*c17_c_impedances)[23];
  real_T (*c17_c_gainsDCOM)[3];
  real_T (*c17_c_gainsPCOM)[3];
  real_T (*c17_c_desired_x_dx_ddx_CoM)[9];
  real_T (*c17_c_J_CoM)[174];
  real_T (*c17_c_xcom)[3];
  real_T (*c17_c_dJRv)[6];
  real_T (*c17_c_dJLv)[6];
  real_T (*c17_c_JR)[174];
  real_T (*c17_c_JL)[174];
  real_T (*c17_c_w_H_r_contact)[16];
  real_T (*c17_c_w_H_l_contact)[16];
  real_T (*c17_c_intHw)[3];
  real_T (*c17_c_H)[6];
  real_T (*c17_c_h)[29];
  real_T (*c17_c_M)[841];
  real_T (*c17_c_v)[29];
  real_T (*c17_c_qDes)[23];
  real_T (*c17_c_q)[23];
  real_T (*c17_c_bVectorConstraints)[19];
  real_T (*c17_c_ConstraintsMatrix)[114];
  real_T (*c17_b_ROBOT_DOF_FOR_SIMULINK)[529];
  real_T (*c17_c_constraints)[2];
  c17_c_correctionFromSupportForce = (real_T (*)[6])ssGetOutputPortSignal
    (chartInstance->S, 16);
  c17_b_useExtArmForces = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 29);
  c17_c_RArmWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 28);
  c17_c_LArmWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 27);
  c17_c_w_H_rArm = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 26);
  c17_c_w_H_lArm = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 25);
  c17_b_f = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 15);
  c17_c_qTilde = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 14);
  c17_c_errorCoM = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 13);
  c17_c_ki_int_qtilde = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S,
    24);
  c17_c_intErrorCoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 23);
  c17_c_impedances = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 22);
  c17_c_gainsDCOM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 21);
  c17_c_gainsPCOM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 20);
  c17_c_desired_x_dx_ddx_CoM = (real_T (*)[9])ssGetInputPortSignal
    (chartInstance->S, 19);
  c17_c_J_CoM = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 18);
  c17_c_xcom = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 17);
  c17_c_dJRv = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 16);
  c17_c_dJLv = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 15);
  c17_c_JR = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 14);
  c17_c_JL = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 13);
  c17_c_w_H_r_contact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
    12);
  c17_c_w_H_l_contact = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
    11);
  c17_c_intHw = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 10);
  c17_c_H = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 9);
  c17_c_h = (real_T (*)[29])ssGetInputPortSignal(chartInstance->S, 8);
  c17_c_M = (real_T (*)[841])ssGetInputPortSignal(chartInstance->S, 7);
  c17_c_v = (real_T (*)[29])ssGetInputPortSignal(chartInstance->S, 6);
  c17_c_qDes = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 5);
  c17_c_q = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 4);
  c17_c_bVectorConstraints = (real_T (*)[19])ssGetInputPortSignal
    (chartInstance->S, 3);
  c17_c_ConstraintsMatrix = (real_T (*)[114])ssGetInputPortSignal
    (chartInstance->S, 2);
  c17_b_ROBOT_DOF_FOR_SIMULINK = (real_T (*)[529])ssGetInputPortSignal
    (chartInstance->S, 1);
  c17_c_constraints = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  c17_c_bVectorConstraintsQp2FeetOrLegs = (real_T (*)[38])ssGetOutputPortSignal
    (chartInstance->S, 12);
  c17_c_ConstraintsMatrixQP2FeetOrLegs = (real_T (*)[456])ssGetOutputPortSignal
    (chartInstance->S, 11);
  c17_c_gradientQP2FeetOrLegs = (real_T (*)[12])ssGetOutputPortSignal
    (chartInstance->S, 10);
  c17_c_HessianMatrixQP2FeetOrLegs = (real_T (*)[144])ssGetOutputPortSignal
    (chartInstance->S, 9);
  c17_c_bVectorConstraintsQp1Foot = (real_T (*)[19])ssGetOutputPortSignal
    (chartInstance->S, 8);
  c17_c_ConstraintsMatrixQP1Foot = (real_T (*)[114])ssGetOutputPortSignal
    (chartInstance->S, 7);
  c17_c_gradientQP1Foot = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S,
    6);
  c17_c_HessianMatrixQP1Foot = (real_T (*)[36])ssGetOutputPortSignal
    (chartInstance->S, 5);
  c17_c_fHdotDesC1C2 = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 4);
  c17_c_NA = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 3);
  c17_c_Sigma = (real_T (*)[276])ssGetOutputPortSignal(chartInstance->S, 2);
  c17_c_tauModel = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 16U, chartInstance->c17_sfEvent);
  c17_hoistedGlobal = *c17_b_useExtArmForces;
  for (c17_i77 = 0; c17_i77 < 2; c17_i77++) {
    c17_constraints[c17_i77] = (*c17_c_constraints)[c17_i77];
  }

  for (c17_i78 = 0; c17_i78 < 529; c17_i78++) {
    c17_ROBOT_DOF_FOR_SIMULINK[c17_i78] = (*c17_b_ROBOT_DOF_FOR_SIMULINK)
      [c17_i78];
  }

  for (c17_i79 = 0; c17_i79 < 114; c17_i79++) {
    c17_ConstraintsMatrix[c17_i79] = (*c17_c_ConstraintsMatrix)[c17_i79];
  }

  for (c17_i80 = 0; c17_i80 < 19; c17_i80++) {
    c17_bVectorConstraints[c17_i80] = (*c17_c_bVectorConstraints)[c17_i80];
  }

  for (c17_i81 = 0; c17_i81 < 23; c17_i81++) {
    c17_q[c17_i81] = (*c17_c_q)[c17_i81];
  }

  for (c17_i82 = 0; c17_i82 < 23; c17_i82++) {
    c17_qDes[c17_i82] = (*c17_c_qDes)[c17_i82];
  }

  for (c17_i83 = 0; c17_i83 < 29; c17_i83++) {
    c17_v[c17_i83] = (*c17_c_v)[c17_i83];
  }

  for (c17_i84 = 0; c17_i84 < 841; c17_i84++) {
    c17_M[c17_i84] = (*c17_c_M)[c17_i84];
  }

  for (c17_i85 = 0; c17_i85 < 29; c17_i85++) {
    c17_h[c17_i85] = (*c17_c_h)[c17_i85];
  }

  for (c17_i86 = 0; c17_i86 < 6; c17_i86++) {
    c17_H[c17_i86] = (*c17_c_H)[c17_i86];
  }

  for (c17_i87 = 0; c17_i87 < 3; c17_i87++) {
    c17_intHw[c17_i87] = (*c17_c_intHw)[c17_i87];
  }

  for (c17_i88 = 0; c17_i88 < 16; c17_i88++) {
    c17_w_H_l_contact[c17_i88] = (*c17_c_w_H_l_contact)[c17_i88];
  }

  for (c17_i89 = 0; c17_i89 < 16; c17_i89++) {
    c17_w_H_r_contact[c17_i89] = (*c17_c_w_H_r_contact)[c17_i89];
  }

  for (c17_i90 = 0; c17_i90 < 174; c17_i90++) {
    c17_JL[c17_i90] = (*c17_c_JL)[c17_i90];
  }

  for (c17_i91 = 0; c17_i91 < 174; c17_i91++) {
    c17_JR[c17_i91] = (*c17_c_JR)[c17_i91];
  }

  for (c17_i92 = 0; c17_i92 < 6; c17_i92++) {
    c17_dJLv[c17_i92] = (*c17_c_dJLv)[c17_i92];
  }

  for (c17_i93 = 0; c17_i93 < 6; c17_i93++) {
    c17_dJRv[c17_i93] = (*c17_c_dJRv)[c17_i93];
  }

  for (c17_i94 = 0; c17_i94 < 3; c17_i94++) {
    c17_xcom[c17_i94] = (*c17_c_xcom)[c17_i94];
  }

  for (c17_i95 = 0; c17_i95 < 174; c17_i95++) {
    c17_J_CoM[c17_i95] = (*c17_c_J_CoM)[c17_i95];
  }

  for (c17_i96 = 0; c17_i96 < 9; c17_i96++) {
    c17_desired_x_dx_ddx_CoM[c17_i96] = (*c17_c_desired_x_dx_ddx_CoM)[c17_i96];
  }

  for (c17_i97 = 0; c17_i97 < 3; c17_i97++) {
    c17_gainsPCOM[c17_i97] = (*c17_c_gainsPCOM)[c17_i97];
  }

  for (c17_i98 = 0; c17_i98 < 3; c17_i98++) {
    c17_gainsDCOM[c17_i98] = (*c17_c_gainsDCOM)[c17_i98];
  }

  for (c17_i99 = 0; c17_i99 < 23; c17_i99++) {
    c17_impedances[c17_i99] = (*c17_c_impedances)[c17_i99];
  }

  for (c17_i100 = 0; c17_i100 < 3; c17_i100++) {
    c17_intErrorCoM[c17_i100] = (*c17_c_intErrorCoM)[c17_i100];
  }

  for (c17_i101 = 0; c17_i101 < 23; c17_i101++) {
    c17_ki_int_qtilde[c17_i101] = (*c17_c_ki_int_qtilde)[c17_i101];
  }

  c17_b_reg = chartInstance->c17_reg;
  c17_b_gain = chartInstance->c17_gain;
  for (c17_i102 = 0; c17_i102 < 16; c17_i102++) {
    c17_w_H_lArm[c17_i102] = (*c17_c_w_H_lArm)[c17_i102];
  }

  for (c17_i103 = 0; c17_i103 < 16; c17_i103++) {
    c17_w_H_rArm[c17_i103] = (*c17_c_w_H_rArm)[c17_i103];
  }

  for (c17_i104 = 0; c17_i104 < 6; c17_i104++) {
    c17_LArmWrench[c17_i104] = (*c17_c_LArmWrench)[c17_i104];
  }

  for (c17_i105 = 0; c17_i105 < 6; c17_i105++) {
    c17_RArmWrench[c17_i105] = (*c17_c_RArmWrench)[c17_i105];
  }

  c17_useExtArmForces = c17_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 50U, 50U, c17_debug_family_names,
    c17_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_nargin, 0U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_nargout, 1U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_constraints, 2U, c17_v_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_ROBOT_DOF_FOR_SIMULINK, 3U, c17_u_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_ConstraintsMatrix, 4U, c17_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_bVectorConstraints, 5U, c17_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_q, 6U, c17_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_qDes, 7U, c17_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_v, 8U, c17_s_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_M, 9U, c17_t_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_h, 10U, c17_s_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_H, 11U, c17_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_intHw, 12U, c17_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_w_H_l_contact, 13U, c17_m_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_w_H_r_contact, 14U, c17_m_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_JL, 15U, c17_q_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_JR, 16U, c17_q_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_dJLv, 17U, c17_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_dJRv, 18U, c17_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_xcom, 19U, c17_r_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_J_CoM, 20U, c17_q_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_desired_x_dx_ddx_CoM, 21U, c17_p_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_gainsPCOM, 22U, c17_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_gainsDCOM, 23U, c17_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_impedances, 24U, c17_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_intErrorCoM, 25U, c17_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_ki_int_qtilde, 26U, c17_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_b_reg, 27U, c17_o_sf_marshallOut,
    c17_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_b_gain, 28U, c17_n_sf_marshallOut,
    c17_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_w_H_lArm, 29U, c17_m_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_w_H_rArm, 30U, c17_m_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_LArmWrench, 31U, c17_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_RArmWrench, 32U, c17_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c17_useExtArmForces, 33U, c17_l_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_tauModel, 34U, c17_c_sf_marshallOut,
    c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Sigma, 35U, c17_k_sf_marshallOut,
    c17_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_NA, 36U, c17_g_sf_marshallOut,
    c17_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_fHdotDesC1C2, 37U,
    c17_b_sf_marshallOut, c17_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_HessianMatrixQP1Foot, 38U,
    c17_j_sf_marshallOut, c17_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_gradientQP1Foot, 39U,
    c17_sf_marshallOut, c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_ConstraintsMatrixQP1Foot, 40U,
    c17_i_sf_marshallOut, c17_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_bVectorConstraintsQp1Foot, 41U,
    c17_h_sf_marshallOut, c17_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_HessianMatrixQP2FeetOrLegs, 42U,
    c17_g_sf_marshallOut, c17_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_gradientQP2FeetOrLegs, 43U,
    c17_b_sf_marshallOut, c17_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_ConstraintsMatrixQP2FeetOrLegs, 44U,
    c17_f_sf_marshallOut, c17_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_bVectorConstraintsQp2FeetOrLegs, 45U,
    c17_e_sf_marshallOut, c17_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_errorCoM, 46U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_qTilde, 47U, c17_c_sf_marshallOut,
    c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_f, 48U, c17_b_sf_marshallOut,
    c17_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_correctionFromSupportForce, 49U,
    c17_sf_marshallOut, c17_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c17_sfEvent, 12);
  for (c17_i106 = 0; c17_i106 < 2; c17_i106++) {
    c17_b_constraints[c17_i106] = c17_constraints[c17_i106];
  }

  for (c17_i107 = 0; c17_i107 < 114; c17_i107++) {
    c17_b_ConstraintsMatrix[c17_i107] = c17_ConstraintsMatrix[c17_i107];
  }

  for (c17_i108 = 0; c17_i108 < 19; c17_i108++) {
    c17_b_bVectorConstraints[c17_i108] = c17_bVectorConstraints[c17_i108];
  }

  for (c17_i109 = 0; c17_i109 < 23; c17_i109++) {
    c17_b_q[c17_i109] = c17_q[c17_i109];
  }

  for (c17_i110 = 0; c17_i110 < 23; c17_i110++) {
    c17_b_qDes[c17_i110] = c17_qDes[c17_i110];
  }

  for (c17_i111 = 0; c17_i111 < 29; c17_i111++) {
    c17_b_v[c17_i111] = c17_v[c17_i111];
  }

  for (c17_i112 = 0; c17_i112 < 841; c17_i112++) {
    c17_b_M[c17_i112] = c17_M[c17_i112];
  }

  for (c17_i113 = 0; c17_i113 < 29; c17_i113++) {
    c17_b_h[c17_i113] = c17_h[c17_i113];
  }

  for (c17_i114 = 0; c17_i114 < 6; c17_i114++) {
    c17_b_H[c17_i114] = c17_H[c17_i114];
  }

  for (c17_i115 = 0; c17_i115 < 3; c17_i115++) {
    c17_b_intHw[c17_i115] = c17_intHw[c17_i115];
  }

  for (c17_i116 = 0; c17_i116 < 16; c17_i116++) {
    c17_b_w_H_l_contact[c17_i116] = c17_w_H_l_contact[c17_i116];
  }

  for (c17_i117 = 0; c17_i117 < 16; c17_i117++) {
    c17_b_w_H_r_contact[c17_i117] = c17_w_H_r_contact[c17_i117];
  }

  for (c17_i118 = 0; c17_i118 < 174; c17_i118++) {
    c17_b_JL[c17_i118] = c17_JL[c17_i118];
  }

  for (c17_i119 = 0; c17_i119 < 174; c17_i119++) {
    c17_b_JR[c17_i119] = c17_JR[c17_i119];
  }

  for (c17_i120 = 0; c17_i120 < 6; c17_i120++) {
    c17_b_dJLv[c17_i120] = c17_dJLv[c17_i120];
  }

  for (c17_i121 = 0; c17_i121 < 6; c17_i121++) {
    c17_b_dJRv[c17_i121] = c17_dJRv[c17_i121];
  }

  for (c17_i122 = 0; c17_i122 < 3; c17_i122++) {
    c17_b_xcom[c17_i122] = c17_xcom[c17_i122];
  }

  for (c17_i123 = 0; c17_i123 < 174; c17_i123++) {
    c17_b_J_CoM[c17_i123] = c17_J_CoM[c17_i123];
  }

  for (c17_i124 = 0; c17_i124 < 9; c17_i124++) {
    c17_b_desired_x_dx_ddx_CoM[c17_i124] = c17_desired_x_dx_ddx_CoM[c17_i124];
  }

  for (c17_i125 = 0; c17_i125 < 3; c17_i125++) {
    c17_b_gainsPCOM[c17_i125] = c17_gainsPCOM[c17_i125];
  }

  for (c17_i126 = 0; c17_i126 < 3; c17_i126++) {
    c17_b_gainsDCOM[c17_i126] = c17_gainsDCOM[c17_i126];
  }

  for (c17_i127 = 0; c17_i127 < 23; c17_i127++) {
    c17_b_impedances[c17_i127] = c17_impedances[c17_i127];
  }

  for (c17_i128 = 0; c17_i128 < 3; c17_i128++) {
    c17_b_intErrorCoM[c17_i128] = c17_intErrorCoM[c17_i128];
  }

  for (c17_i129 = 0; c17_i129 < 23; c17_i129++) {
    c17_b_ki_int_qtilde[c17_i129] = c17_ki_int_qtilde[c17_i129];
  }

  c17_c_gain = c17_b_gain;
  for (c17_i130 = 0; c17_i130 < 16; c17_i130++) {
    c17_b_w_H_lArm[c17_i130] = c17_w_H_lArm[c17_i130];
  }

  for (c17_i131 = 0; c17_i131 < 16; c17_i131++) {
    c17_b_w_H_rArm[c17_i131] = c17_w_H_rArm[c17_i131];
  }

  for (c17_i132 = 0; c17_i132 < 6; c17_i132++) {
    c17_b_LArmWrench[c17_i132] = c17_LArmWrench[c17_i132];
  }

  for (c17_i133 = 0; c17_i133 < 6; c17_i133++) {
    c17_b_RArmWrench[c17_i133] = c17_RArmWrench[c17_i133];
  }

  c17_balancingController(chartInstance, c17_b_constraints,
    c17_b_ConstraintsMatrix, c17_b_bVectorConstraints, c17_b_q, c17_b_qDes,
    c17_b_v, c17_b_M, c17_b_h, c17_b_H, c17_b_intHw, c17_b_w_H_l_contact,
    c17_b_w_H_r_contact, c17_b_JL, c17_b_JR, c17_b_dJLv, c17_b_dJRv, c17_b_xcom,
    c17_b_J_CoM, c17_b_desired_x_dx_ddx_CoM, c17_b_gainsPCOM, c17_b_gainsDCOM,
    c17_b_impedances, c17_b_intErrorCoM, c17_b_ki_int_qtilde, c17_b_reg,
    &c17_c_gain, c17_b_w_H_lArm, c17_b_w_H_rArm, c17_b_LArmWrench,
    c17_b_RArmWrench, c17_useExtArmForces, c17_b_tauModel, c17_b_Sigma, c17_b_NA,
    c17_b_fHdotDesC1C2, c17_b_HessianMatrixQP1Foot, c17_b_gradientQP1Foot,
    c17_b_ConstraintsMatrixQP1Foot, c17_b_bVectorConstraintsQp1Foot,
    c17_b_HessianMatrixQP2FeetOrLegs, c17_b_gradientQP2FeetOrLegs,
    c17_b_ConstraintsMatrixQP2FeetOrLegs, c17_b_bVectorConstraintsQp2FeetOrLegs,
    c17_b_errorCoM, c17_b_qTilde, c17_b_correctionFromSupportForce);
  for (c17_i134 = 0; c17_i134 < 23; c17_i134++) {
    c17_tauModel[c17_i134] = c17_b_tauModel[c17_i134];
  }

  for (c17_i135 = 0; c17_i135 < 276; c17_i135++) {
    c17_Sigma[c17_i135] = c17_b_Sigma[c17_i135];
  }

  for (c17_i136 = 0; c17_i136 < 144; c17_i136++) {
    c17_NA[c17_i136] = c17_b_NA[c17_i136];
  }

  for (c17_i137 = 0; c17_i137 < 12; c17_i137++) {
    c17_fHdotDesC1C2[c17_i137] = c17_b_fHdotDesC1C2[c17_i137];
  }

  for (c17_i138 = 0; c17_i138 < 36; c17_i138++) {
    c17_HessianMatrixQP1Foot[c17_i138] = c17_b_HessianMatrixQP1Foot[c17_i138];
  }

  for (c17_i139 = 0; c17_i139 < 6; c17_i139++) {
    c17_gradientQP1Foot[c17_i139] = c17_b_gradientQP1Foot[c17_i139];
  }

  for (c17_i140 = 0; c17_i140 < 114; c17_i140++) {
    c17_ConstraintsMatrixQP1Foot[c17_i140] =
      c17_b_ConstraintsMatrixQP1Foot[c17_i140];
  }

  for (c17_i141 = 0; c17_i141 < 19; c17_i141++) {
    c17_bVectorConstraintsQp1Foot[c17_i141] =
      c17_b_bVectorConstraintsQp1Foot[c17_i141];
  }

  for (c17_i142 = 0; c17_i142 < 144; c17_i142++) {
    c17_HessianMatrixQP2FeetOrLegs[c17_i142] =
      c17_b_HessianMatrixQP2FeetOrLegs[c17_i142];
  }

  for (c17_i143 = 0; c17_i143 < 12; c17_i143++) {
    c17_gradientQP2FeetOrLegs[c17_i143] = c17_b_gradientQP2FeetOrLegs[c17_i143];
  }

  for (c17_i144 = 0; c17_i144 < 456; c17_i144++) {
    c17_ConstraintsMatrixQP2FeetOrLegs[c17_i144] =
      c17_b_ConstraintsMatrixQP2FeetOrLegs[c17_i144];
  }

  for (c17_i145 = 0; c17_i145 < 38; c17_i145++) {
    c17_bVectorConstraintsQp2FeetOrLegs[c17_i145] =
      c17_b_bVectorConstraintsQp2FeetOrLegs[c17_i145];
  }

  for (c17_i146 = 0; c17_i146 < 3; c17_i146++) {
    c17_errorCoM[c17_i146] = c17_b_errorCoM[c17_i146];
  }

  for (c17_i147 = 0; c17_i147 < 23; c17_i147++) {
    c17_qTilde[c17_i147] = c17_b_qTilde[c17_i147];
  }

  for (c17_i148 = 0; c17_i148 < 12; c17_i148++) {
    c17_f[c17_i148] = 0.0;
  }

  for (c17_i149 = 0; c17_i149 < 6; c17_i149++) {
    c17_correctionFromSupportForce[c17_i149] =
      c17_b_correctionFromSupportForce[c17_i149];
  }

  _SFD_EML_CALL(0U, chartInstance->c17_sfEvent, -12);
  _SFD_SYMBOL_SCOPE_POP();
  for (c17_i150 = 0; c17_i150 < 23; c17_i150++) {
    (*c17_c_tauModel)[c17_i150] = c17_tauModel[c17_i150];
  }

  for (c17_i151 = 0; c17_i151 < 276; c17_i151++) {
    (*c17_c_Sigma)[c17_i151] = c17_Sigma[c17_i151];
  }

  for (c17_i152 = 0; c17_i152 < 144; c17_i152++) {
    (*c17_c_NA)[c17_i152] = c17_NA[c17_i152];
  }

  for (c17_i153 = 0; c17_i153 < 12; c17_i153++) {
    (*c17_c_fHdotDesC1C2)[c17_i153] = c17_fHdotDesC1C2[c17_i153];
  }

  for (c17_i154 = 0; c17_i154 < 36; c17_i154++) {
    (*c17_c_HessianMatrixQP1Foot)[c17_i154] = c17_HessianMatrixQP1Foot[c17_i154];
  }

  for (c17_i155 = 0; c17_i155 < 6; c17_i155++) {
    (*c17_c_gradientQP1Foot)[c17_i155] = c17_gradientQP1Foot[c17_i155];
  }

  for (c17_i156 = 0; c17_i156 < 114; c17_i156++) {
    (*c17_c_ConstraintsMatrixQP1Foot)[c17_i156] =
      c17_ConstraintsMatrixQP1Foot[c17_i156];
  }

  for (c17_i157 = 0; c17_i157 < 19; c17_i157++) {
    (*c17_c_bVectorConstraintsQp1Foot)[c17_i157] =
      c17_bVectorConstraintsQp1Foot[c17_i157];
  }

  for (c17_i158 = 0; c17_i158 < 144; c17_i158++) {
    (*c17_c_HessianMatrixQP2FeetOrLegs)[c17_i158] =
      c17_HessianMatrixQP2FeetOrLegs[c17_i158];
  }

  for (c17_i159 = 0; c17_i159 < 12; c17_i159++) {
    (*c17_c_gradientQP2FeetOrLegs)[c17_i159] =
      c17_gradientQP2FeetOrLegs[c17_i159];
  }

  for (c17_i160 = 0; c17_i160 < 456; c17_i160++) {
    (*c17_c_ConstraintsMatrixQP2FeetOrLegs)[c17_i160] =
      c17_ConstraintsMatrixQP2FeetOrLegs[c17_i160];
  }

  for (c17_i161 = 0; c17_i161 < 38; c17_i161++) {
    (*c17_c_bVectorConstraintsQp2FeetOrLegs)[c17_i161] =
      c17_bVectorConstraintsQp2FeetOrLegs[c17_i161];
  }

  for (c17_i162 = 0; c17_i162 < 3; c17_i162++) {
    (*c17_c_errorCoM)[c17_i162] = c17_errorCoM[c17_i162];
  }

  for (c17_i163 = 0; c17_i163 < 23; c17_i163++) {
    (*c17_c_qTilde)[c17_i163] = c17_qTilde[c17_i163];
  }

  for (c17_i164 = 0; c17_i164 < 12; c17_i164++) {
    (*c17_b_f)[c17_i164] = c17_f[c17_i164];
  }

  for (c17_i165 = 0; c17_i165 < 6; c17_i165++) {
    (*c17_c_correctionFromSupportForce)[c17_i165] =
      c17_correctionFromSupportForce[c17_i165];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 16U, chartInstance->c17_sfEvent);
}

static void initSimStructsc17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc17_torqueBalancing2012b
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c17_balancingController(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_constraints[2], real_T c17_ConstraintsMatrix[114],
  real_T c17_bVectorConstraints[19], real_T c17_q[23], real_T c17_qDes[23],
  real_T c17_v[29], real_T c17_M[841], real_T c17_h[29], real_T c17_H[6], real_T
  c17_intHw[3], real_T c17_w_H_l_contact[16], real_T c17_w_H_r_contact[16],
  real_T c17_JL[174], real_T c17_JR[174], real_T c17_dJLv[6], real_T c17_dJRv[6],
  real_T c17_xcom[3], real_T c17_J_CoM[174], real_T c17_desired_x_dx_ddx_CoM[9],
  real_T c17_gainsPCOM[3], real_T c17_gainsDCOM[3], real_T c17_impedances[23],
  real_T c17_intErrorCoM[3], real_T c17_ki_int_qtilde[23],
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_b_reg, c17_struct_kzTB0QQWoOlMoMhgKf6sK *
  c17_b_gain, real_T c17_w_H_lArm[16], real_T c17_w_H_rArm[16], real_T
  c17_LArmWrench[6], real_T c17_RArmWrench[6], boolean_T c17_useExtArmForces,
  real_T c17_tauModel[23], real_T c17_Sigma[276], real_T c17_NA[144], real_T
  c17_f_HDot[12], real_T c17_HessianMatrixQP1Foot[36], real_T
  c17_gradientQP1Foot[6], real_T c17_ConstraintsMatrixQP1Foot[114], real_T
  c17_bVectorConstraintsQp1Foot[19], real_T c17_HessianMatrixQP2FeetOrLegs[144],
  real_T c17_gradientQP2FeetOrLegs[12], real_T
  c17_ConstraintsMatrixQP2FeetOrLegs[456], real_T
  c17_bVectorConstraintsQp2FeetOrLegs[38], real_T c17_errorCoM[3], real_T
  c17_qTilde[23], real_T c17_correctionFromSupportForce[6])
{
  uint32_T c17_debug_family_var_map[102];
  real_T c17_f[12];
  real_T c17_pos_leftFoot[3];
  real_T c17_w_R_l_sole[9];
  real_T c17_pos_rightFoot[3];
  real_T c17_w_R_r_sole[9];
  real_T c17_pos_leftArm[3];
  real_T c17_pos_rightArm[3];
  real_T c17_gainsICOM[3];
  real_T c17_dampings[23];
  real_T c17_ROBOT_DOF;
  real_T c17_gravAcc;
  real_T c17_m;
  real_T c17_Mb[36];
  real_T c17_Mbj[138];
  static real_T c17_Mj[529];
  static real_T c17_St[667];
  real_T c17_gravityWrench[6];
  real_T c17_xDcom[3];
  real_T c17_qD[23];
  real_T c17_Pr[3];
  real_T c17_Pl[3];
  real_T c17_PlArm[3];
  real_T c17_PrArm[3];
  real_T c17_AL[36];
  real_T c17_AR[36];
  real_T c17_A[72];
  real_T c17_pinvA[72];
  real_T c17_A_lArm[36];
  real_T c17_A_rArm[36];
  real_T c17_A_arms[72];
  real_T c17_fArms[12];
  real_T c17_fsupport[6];
  real_T c17_xDDcomStar[3];
  real_T c17_H_desired[6];
  real_T c17_H_error[6];
  real_T c17_alpha;
  real_T c17_H_errParallel[6];
  real_T c17_Jc[348];
  real_T c17_JcDv[12];
  real_T c17_JcMinv[348];
  real_T c17_JcMinvSt[276];
  real_T c17_JcMinvJct[144];
  real_T c17_JBar[276];
  real_T c17_PInv_JcMinvSt[276];
  static real_T c17_nullJcMinvSt[529];
  static real_T c17_Mbar[529];
  static real_T c17_NLMbar[529];
  real_T c17_constraintMatrixLeftFoot[114];
  real_T c17_constraintMatrixRightFoot[114];
  real_T c17_ConstraintsMatrix2FeetOrLegs[456];
  real_T c17_bVectorConstraints2FeetOrLegs[38];
  real_T c17_HDotDes[6];
  real_T c17_SigmaNA[276];
  real_T c17_A1Foot[36];
  static real_T c17_b_impedances[529];
  static real_T c17_b_dampings[529];
  real_T c17_nargin = 32.0;
  real_T c17_nargout = 16.0;
  int32_T c17_i166;
  int32_T c17_i167;
  int32_T c17_i168;
  int32_T c17_i169;
  int32_T c17_i170;
  int32_T c17_i171;
  int32_T c17_i172;
  int32_T c17_i173;
  int32_T c17_i174;
  int32_T c17_i175;
  int32_T c17_i176;
  int32_T c17_i177;
  int32_T c17_i178;
  int32_T c17_i179;
  int32_T c17_i180;
  int32_T c17_i181;
  int32_T c17_i182;
  int32_T c17_i183;
  int32_T c17_i184;
  int32_T c17_i185;
  int32_T c17_i186;
  int32_T c17_i187;
  int32_T c17_i188;
  int32_T c17_i189;
  int32_T c17_i190;
  int32_T c17_i191;
  int32_T c17_i192;
  static real_T c17_b[667] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c17_a;
  real_T c17_y;
  int32_T c17_i193;
  int32_T c17_i194;
  int32_T c17_i195;
  int32_T c17_i196;
  int32_T c17_i197;
  int32_T c17_i198;
  real_T c17_b_a[87];
  int32_T c17_i199;
  real_T c17_b_b[29];
  int32_T c17_i200;
  int32_T c17_i201;
  int32_T c17_i202;
  real_T c17_C[3];
  int32_T c17_i203;
  int32_T c17_i204;
  int32_T c17_i205;
  int32_T c17_i206;
  int32_T c17_i207;
  int32_T c17_i208;
  int32_T c17_i209;
  int32_T c17_i210;
  int32_T c17_i211;
  int32_T c17_i212;
  int32_T c17_i213;
  int32_T c17_i214;
  real_T c17_dv16[9];
  int32_T c17_i215;
  real_T c17_b_Pl[3];
  real_T c17_dv17[9];
  real_T c17_dv18[9];
  int32_T c17_i216;
  int32_T c17_i217;
  int32_T c17_i218;
  int32_T c17_i219;
  int32_T c17_i220;
  int32_T c17_i221;
  int32_T c17_i222;
  int32_T c17_i223;
  int32_T c17_i224;
  int32_T c17_i225;
  int32_T c17_i226;
  int32_T c17_i227;
  int32_T c17_i228;
  int32_T c17_i229;
  int32_T c17_i230;
  int32_T c17_i231;
  real_T c17_b_Pr[3];
  int32_T c17_i232;
  int32_T c17_i233;
  int32_T c17_i234;
  int32_T c17_i235;
  int32_T c17_i236;
  int32_T c17_i237;
  int32_T c17_i238;
  int32_T c17_i239;
  int32_T c17_i240;
  int32_T c17_i241;
  int32_T c17_i242;
  int32_T c17_i243;
  int32_T c17_i244;
  int32_T c17_i245;
  int32_T c17_i246;
  int32_T c17_i247;
  int32_T c17_i248;
  int32_T c17_i249;
  int32_T c17_i250;
  int32_T c17_i251;
  int32_T c17_i252;
  int32_T c17_i253;
  real_T c17_b_A[72];
  real_T c17_c_a[72];
  real_T c17_c_b;
  int32_T c17_i254;
  real_T c17_d_b;
  int32_T c17_i255;
  int32_T c17_i256;
  real_T c17_b_AL[36];
  real_T c17_e_b[36];
  int32_T c17_i257;
  int32_T c17_i258;
  int32_T c17_i259;
  int32_T c17_i260;
  real_T c17_d_a[72];
  int32_T c17_i261;
  int32_T c17_i262;
  int32_T c17_i263;
  real_T c17_f_b;
  int32_T c17_i264;
  real_T c17_g_b;
  int32_T c17_i265;
  int32_T c17_i266;
  real_T c17_b_AR[36];
  int32_T c17_i267;
  int32_T c17_i268;
  int32_T c17_i269;
  real_T c17_e_a[72];
  int32_T c17_i270;
  int32_T c17_i271;
  int32_T c17_i272;
  int32_T c17_i273;
  real_T c17_h_b;
  int32_T c17_i274;
  real_T c17_i_b;
  int32_T c17_i275;
  int32_T c17_i276;
  int32_T c17_i277;
  real_T c17_b_PlArm[3];
  int32_T c17_i278;
  int32_T c17_i279;
  int32_T c17_i280;
  int32_T c17_i281;
  int32_T c17_i282;
  int32_T c17_i283;
  int32_T c17_i284;
  int32_T c17_i285;
  int32_T c17_i286;
  int32_T c17_i287;
  int32_T c17_i288;
  int32_T c17_i289;
  int32_T c17_i290;
  int32_T c17_i291;
  int32_T c17_i292;
  int32_T c17_i293;
  real_T c17_b_PrArm[3];
  int32_T c17_i294;
  int32_T c17_i295;
  int32_T c17_i296;
  int32_T c17_i297;
  int32_T c17_i298;
  int32_T c17_i299;
  int32_T c17_i300;
  int32_T c17_i301;
  int32_T c17_i302;
  int32_T c17_i303;
  int32_T c17_i304;
  int32_T c17_i305;
  int32_T c17_i306;
  int32_T c17_i307;
  int32_T c17_i308;
  int32_T c17_i309;
  int32_T c17_i310;
  int32_T c17_i311;
  int32_T c17_i312;
  int32_T c17_i313;
  int32_T c17_i314;
  int32_T c17_i315;
  int32_T c17_i316;
  int32_T c17_i317;
  real_T c17_f_a[72];
  int32_T c17_i318;
  real_T c17_j_b[12];
  int32_T c17_i319;
  int32_T c17_i320;
  int32_T c17_i321;
  real_T c17_b_C[6];
  int32_T c17_i322;
  int32_T c17_i323;
  int32_T c17_i324;
  int32_T c17_i325;
  int32_T c17_i326;
  int32_T c17_i327;
  int32_T c17_i328;
  int32_T c17_i329;
  int32_T c17_i330;
  int32_T c17_i331;
  int32_T c17_i332;
  real_T c17_g_a[6];
  int32_T c17_i333;
  real_T c17_k_b[6];
  real_T c17_b_y;
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_c_A;
  int32_T c17_i334;
  real_T c17_b_H_error[6];
  real_T c17_B;
  real_T c17_x;
  real_T c17_c_y;
  real_T c17_b_x;
  real_T c17_d_y;
  int32_T c17_i335;
  int32_T c17_i336;
  real_T c17_c_H_error[6];
  real_T c17_b_B;
  real_T c17_e_y;
  real_T c17_f_y;
  int32_T c17_i337;
  real_T c17_h_a;
  int32_T c17_i338;
  int32_T c17_i339;
  int32_T c17_i340;
  int32_T c17_i341;
  int32_T c17_i342;
  int32_T c17_i343;
  real_T c17_g_y[144];
  int32_T c17_i344;
  real_T c17_i_a[72];
  int32_T c17_i345;
  real_T c17_j_a[72];
  real_T c17_k_a[144];
  int32_T c17_i346;
  real_T c17_l_b;
  int32_T c17_i347;
  real_T c17_m_b;
  int32_T c17_i348;
  int32_T c17_i349;
  real_T c17_l_a[174];
  real_T c17_n_b;
  int32_T c17_i350;
  int32_T c17_i351;
  real_T c17_m_a[174];
  real_T c17_o_b;
  int32_T c17_i352;
  int32_T c17_i353;
  int32_T c17_i354;
  int32_T c17_i355;
  int32_T c17_i356;
  int32_T c17_i357;
  int32_T c17_i358;
  int32_T c17_i359;
  int32_T c17_i360;
  int32_T c17_i361;
  real_T c17_p_b;
  int32_T c17_i362;
  int32_T c17_i363;
  real_T c17_q_b;
  int32_T c17_i364;
  int32_T c17_i365;
  int32_T c17_i366;
  int32_T c17_i367;
  real_T c17_b_Jc[348];
  int32_T c17_i368;
  static real_T c17_b_M[841];
  real_T c17_dv19[348];
  int32_T c17_i369;
  int32_T c17_i370;
  real_T c17_n_a[348];
  int32_T c17_i371;
  int32_T c17_i372;
  int32_T c17_i373;
  real_T c17_dv20[348];
  int32_T c17_i374;
  static real_T c17_dv21[667];
  int32_T c17_i375;
  real_T c17_dv22[348];
  int32_T c17_i376;
  static real_T c17_dv23[667];
  int32_T c17_i377;
  int32_T c17_i378;
  int32_T c17_i379;
  int32_T c17_i380;
  int32_T c17_i381;
  real_T c17_r_b[348];
  int32_T c17_i382;
  int32_T c17_i383;
  int32_T c17_i384;
  real_T c17_dv24[348];
  int32_T c17_i385;
  real_T c17_dv25[348];
  int32_T c17_i386;
  real_T c17_dv26[348];
  int32_T c17_i387;
  real_T c17_dv27[348];
  int32_T c17_i388;
  int32_T c17_i389;
  int32_T c17_i390;
  int32_T c17_i391;
  real_T c17_b_Mbj[138];
  int32_T c17_i392;
  real_T c17_b_Mb[36];
  real_T c17_o_a[138];
  int32_T c17_i393;
  int32_T c17_i394;
  int32_T c17_i395;
  int32_T c17_i396;
  int32_T c17_i397;
  real_T c17_h_y[276];
  int32_T c17_i398;
  real_T c17_p_a[138];
  int32_T c17_i399;
  real_T c17_q_a[72];
  int32_T c17_i400;
  int32_T c17_i401;
  int32_T c17_i402;
  int32_T c17_i403;
  int32_T c17_i404;
  real_T c17_b_JcMinvSt[276];
  real_T c17_dv28[276];
  int32_T c17_i405;
  int32_T c17_i406;
  real_T c17_r_a[276];
  int32_T c17_i407;
  real_T c17_s_b[276];
  int32_T c17_i408;
  static real_T c17_i_y[529];
  int32_T c17_i409;
  real_T c17_s_a[276];
  int32_T c17_i410;
  real_T c17_t_b[276];
  static real_T c17_t_a[529];
  int32_T c17_i411;
  int32_T c17_i412;
  int32_T c17_i413;
  int32_T c17_i414;
  int32_T c17_i415;
  real_T c17_c_Mbj[138];
  int32_T c17_i416;
  real_T c17_c_Mb[36];
  int32_T c17_i417;
  real_T c17_u_b[138];
  int32_T c17_i418;
  int32_T c17_i419;
  real_T c17_u_a[138];
  int32_T c17_i420;
  real_T c17_v_b[138];
  int32_T c17_i421;
  int32_T c17_i422;
  int32_T c17_i423;
  static real_T c17_w_b[529];
  int32_T c17_i424;
  int32_T c17_i425;
  int32_T c17_i426;
  static real_T c17_dv29[529];
  int32_T c17_i427;
  static real_T c17_dv30[529];
  int32_T c17_i428;
  static real_T c17_dv31[529];
  int32_T c17_i429;
  static real_T c17_dv32[529];
  int32_T c17_i430;
  real_T c17_c_impedances[23];
  int32_T c17_i431;
  static real_T c17_b_NLMbar[529];
  int32_T c17_i432;
  int32_T c17_i433;
  static real_T c17_v_a[529];
  int32_T c17_i434;
  static real_T c17_x_b[529];
  real_T c17_w_a;
  int32_T c17_i435;
  static real_T c17_y_b[529] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c17_i436;
  int32_T c17_i437;
  real_T c17_c_dampings[23];
  int32_T c17_i438;
  static real_T c17_c_NLMbar[529];
  int32_T c17_i439;
  int32_T c17_i440;
  static real_T c17_x_a[529];
  int32_T c17_i441;
  static real_T c17_ab_b[529];
  real_T c17_y_a;
  int32_T c17_i442;
  int32_T c17_i443;
  int32_T c17_i444;
  real_T c17_ab_a[114];
  int32_T c17_i445;
  int32_T c17_i446;
  int32_T c17_i447;
  int32_T c17_i448;
  real_T c17_b_w_R_l_sole[9];
  int32_T c17_i449;
  int32_T c17_i450;
  int32_T c17_i451;
  int32_T c17_i452;
  real_T c17_c_w_R_l_sole[9];
  int32_T c17_i453;
  int32_T c17_i454;
  int32_T c17_i455;
  real_T c17_dv33[114];
  int32_T c17_i456;
  real_T c17_dv34[36];
  int32_T c17_i457;
  real_T c17_dv35[114];
  int32_T c17_i458;
  real_T c17_dv36[36];
  int32_T c17_i459;
  int32_T c17_i460;
  int32_T c17_i461;
  int32_T c17_i462;
  int32_T c17_i463;
  real_T c17_b_w_R_r_sole[9];
  int32_T c17_i464;
  int32_T c17_i465;
  int32_T c17_i466;
  int32_T c17_i467;
  real_T c17_c_w_R_r_sole[9];
  int32_T c17_i468;
  int32_T c17_i469;
  int32_T c17_i470;
  real_T c17_dv37[114];
  int32_T c17_i471;
  real_T c17_dv38[36];
  int32_T c17_i472;
  real_T c17_dv39[114];
  int32_T c17_i473;
  real_T c17_dv40[36];
  int32_T c17_i474;
  real_T c17_b_constraintMatrixLeftFoot[114];
  int32_T c17_i475;
  real_T c17_b_constraintMatrixRightFoot[114];
  real_T c17_dv41[456];
  int32_T c17_i476;
  int32_T c17_i477;
  int32_T c17_i478;
  int32_T c17_i479;
  int32_T c17_i480;
  int32_T c17_i481;
  int32_T c17_i482;
  int32_T c17_i483;
  int32_T c17_i484;
  int32_T c17_i485;
  int32_T c17_i486;
  real_T c17_j_y[23];
  int32_T c17_i487;
  int32_T c17_i488;
  int32_T c17_i489;
  int32_T c17_i490;
  int32_T c17_i491;
  int32_T c17_i492;
  real_T c17_d_Mbj[138];
  int32_T c17_i493;
  real_T c17_d_Mb[36];
  int32_T c17_i494;
  int32_T c17_i495;
  real_T c17_k_y[23];
  int32_T c17_i496;
  int32_T c17_i497;
  int32_T c17_i498;
  int32_T c17_i499;
  int32_T c17_i500;
  int32_T c17_i501;
  static real_T c17_bb_a[529];
  int32_T c17_i502;
  static real_T c17_bb_b[529];
  int32_T c17_i503;
  real_T c17_cb_b[23];
  int32_T c17_i504;
  real_T c17_l_y[23];
  int32_T c17_i505;
  static real_T c17_m_y[529];
  int32_T c17_i506;
  real_T c17_db_b[23];
  int32_T c17_i507;
  int32_T c17_i508;
  int32_T c17_i509;
  int32_T c17_i510;
  static real_T c17_cb_a[529];
  int32_T c17_i511;
  static real_T c17_eb_b[529];
  int32_T c17_i512;
  int32_T c17_i513;
  real_T c17_n_y[23];
  int32_T c17_i514;
  static real_T c17_o_y[529];
  int32_T c17_i515;
  real_T c17_fb_b[23];
  int32_T c17_i516;
  int32_T c17_i517;
  int32_T c17_i518;
  int32_T c17_i519;
  static real_T c17_db_a[529];
  int32_T c17_i520;
  real_T c17_gb_b[23];
  int32_T c17_i521;
  int32_T c17_i522;
  int32_T c17_i523;
  int32_T c17_i524;
  int32_T c17_i525;
  real_T c17_eb_a[276];
  int32_T c17_i526;
  real_T c17_fb_a[144];
  int32_T c17_i527;
  int32_T c17_i528;
  int32_T c17_i529;
  real_T c17_p_y[276];
  int32_T c17_i530;
  static real_T c17_gb_a[529];
  int32_T c17_i531;
  real_T c17_hb_a[276];
  int32_T c17_i532;
  real_T c17_ib_a;
  int32_T c17_i533;
  int32_T c17_i534;
  real_T c17_jb_a;
  int32_T c17_i535;
  real_T c17_hb_b[3];
  int32_T c17_i536;
  real_T c17_kb_a;
  int32_T c17_i537;
  real_T c17_ib_b[3];
  int32_T c17_i538;
  int32_T c17_i539;
  real_T c17_c_C[6];
  int32_T c17_i540;
  int32_T c17_i541;
  int32_T c17_i542;
  int32_T c17_i543;
  int32_T c17_i544;
  int32_T c17_i545;
  int32_T c17_i546;
  real_T c17_jb_b;
  int32_T c17_i547;
  real_T c17_kb_b;
  int32_T c17_i548;
  int32_T c17_i549;
  int32_T c17_i550;
  int32_T c17_i551;
  int32_T c17_i552;
  int32_T c17_i553;
  real_T c17_dv42[276];
  int32_T c17_i554;
  real_T c17_dv43[144];
  int32_T c17_i555;
  real_T c17_dv44[276];
  int32_T c17_i556;
  real_T c17_dv45[144];
  int32_T c17_i557;
  real_T c17_lb_a[456];
  int32_T c17_i558;
  int32_T c17_i559;
  int32_T c17_i560;
  int32_T c17_i561;
  real_T c17_dv46[456];
  int32_T c17_i562;
  real_T c17_dv47[144];
  int32_T c17_i563;
  real_T c17_dv48[456];
  int32_T c17_i564;
  real_T c17_dv49[144];
  int32_T c17_i565;
  int32_T c17_i566;
  int32_T c17_i567;
  real_T c17_q_y[38];
  int32_T c17_i568;
  int32_T c17_i569;
  int32_T c17_i570;
  int32_T c17_i571;
  int32_T c17_i572;
  int32_T c17_i573;
  int32_T c17_i574;
  int32_T c17_i575;
  int32_T c17_i576;
  int32_T c17_i577;
  real_T c17_lb_b[276];
  int32_T c17_i578;
  real_T c17_mb_a[276];
  real_T c17_mb_b;
  int32_T c17_i579;
  static real_T c17_nb_a[144] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c17_i580;
  int32_T c17_i581;
  int32_T c17_i582;
  int32_T c17_i583;
  int32_T c17_i584;
  int32_T c17_i585;
  int32_T c17_i586;
  int32_T c17_i587;
  int32_T c17_i588;
  int32_T c17_i589;
  int32_T c17_i590;
  int32_T c17_i591;
  int32_T c17_i592;
  int32_T c17_i593;
  int32_T c17_i594;
  int32_T c17_i595;
  int32_T c17_i596;
  int32_T c17_i597;
  int32_T c17_i598;
  int32_T c17_i599;
  real_T c17_ob_a;
  real_T c17_nb_b;
  real_T c17_r_y;
  real_T c17_pb_a;
  int32_T c17_i600;
  int32_T c17_i601;
  real_T c17_qb_a;
  real_T c17_ob_b;
  real_T c17_s_y;
  real_T c17_rb_a;
  int32_T c17_i602;
  real_T c17_pb_b[114];
  int32_T c17_i603;
  int32_T c17_i604;
  int32_T c17_i605;
  int32_T c17_i606;
  real_T c17_sb_a[36];
  real_T c17_qb_b;
  int32_T c17_i607;
  real_T c17_rb_b;
  int32_T c17_i608;
  int32_T c17_i609;
  real_T c17_sb_b;
  int32_T c17_i610;
  real_T c17_tb_b;
  int32_T c17_i611;
  int32_T c17_i612;
  int32_T c17_i613;
  int32_T c17_i614;
  int32_T c17_i615;
  int32_T c17_i616;
  int32_T c17_i617;
  int32_T c17_i618;
  int32_T c17_i619;
  int32_T c17_i620;
  real_T c17_t_y[36];
  int32_T c17_i621;
  int32_T c17_i622;
  real_T c17_ub_b;
  int32_T c17_i623;
  static real_T c17_tb_a[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c17_i624;
  int32_T c17_i625;
  int32_T c17_i626;
  int32_T c17_i627;
  int32_T c17_i628;
  int32_T c17_i629;
  int32_T c17_i630;
  int32_T c17_i631;
  int32_T c17_i632;
  int32_T c17_i633;
  int32_T c17_i634;
  int32_T c17_i635;
  int32_T c17_i636;
  int32_T c17_i637;
  int32_T c17_i638;
  int32_T c17_i639;
  int32_T c17_i640;
  boolean_T guard1 = FALSE;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 102U, 104U, c17_d_debug_family_names,
    c17_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_f, 0U, c17_b_sf_marshallOut,
    c17_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_pos_leftFoot, 1U,
    c17_d_sf_marshallOut, c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_w_R_l_sole, 2U, c17_p_sf_marshallOut,
    c17_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_pos_rightFoot, 3U,
    c17_d_sf_marshallOut, c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_w_R_r_sole, 4U, c17_p_sf_marshallOut,
    c17_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_pos_leftArm, 5U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_pos_rightArm, 6U,
    c17_d_sf_marshallOut, c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_gainsICOM, 7U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_dampings, MAX_uint32_T,
    c17_eb_sf_marshallOut, c17_cb_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c17_ROBOT_DOF, 9U, c17_w_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c17_gravAcc, 10U, c17_w_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_m, 11U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Mb, 12U, c17_j_sf_marshallOut,
    c17_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Mbj, 13U, c17_db_sf_marshallOut,
    c17_bb_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Mj, 14U, c17_u_sf_marshallOut,
    c17_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c17_St, 15U, c17_cb_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_gravityWrench, 16U,
    c17_sf_marshallOut, c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_xDcom, 17U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_qD, 18U, c17_c_sf_marshallOut,
    c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Pr, 19U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Pl, 20U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_PlArm, 21U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_PrArm, 22U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_AL, 23U, c17_j_sf_marshallOut,
    c17_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_AR, 24U, c17_j_sf_marshallOut,
    c17_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_A, 25U, c17_ab_sf_marshallOut,
    c17_y_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_pinvA, 26U, c17_bb_sf_marshallOut,
    c17_ab_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_A_lArm, 27U, c17_j_sf_marshallOut,
    c17_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_A_rArm, 28U, c17_j_sf_marshallOut,
    c17_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_A_arms, 29U, c17_ab_sf_marshallOut,
    c17_y_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_fArms, 30U, c17_b_sf_marshallOut,
    c17_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_fsupport, 31U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_xDDcomStar, 32U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_H_desired, 33U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_H_error, 34U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_alpha, 35U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_H_errParallel, 36U,
    c17_sf_marshallOut, c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Jc, 37U, c17_y_sf_marshallOut,
    c17_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_JcDv, 38U, c17_b_sf_marshallOut,
    c17_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_JcMinv, 39U, c17_y_sf_marshallOut,
    c17_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_JcMinvSt, 40U, c17_x_sf_marshallOut,
    c17_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_JcMinvJct, 41U, c17_g_sf_marshallOut,
    c17_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_JBar, 42U, c17_k_sf_marshallOut,
    c17_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_PInv_JcMinvSt, 43U,
    c17_k_sf_marshallOut, c17_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_nullJcMinvSt, 44U,
    c17_u_sf_marshallOut, c17_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Mbar, 45U, c17_u_sf_marshallOut,
    c17_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_NLMbar, 46U, c17_u_sf_marshallOut,
    c17_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_constraintMatrixLeftFoot, 47U,
    c17_i_sf_marshallOut, c17_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_constraintMatrixRightFoot, 48U,
    c17_i_sf_marshallOut, c17_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_ConstraintsMatrix2FeetOrLegs, 49U,
    c17_f_sf_marshallOut, c17_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_bVectorConstraints2FeetOrLegs, 50U,
    c17_e_sf_marshallOut, c17_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_HDotDes, 51U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_SigmaNA, 52U, c17_k_sf_marshallOut,
    c17_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_A1Foot, 53U, c17_j_sf_marshallOut,
    c17_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_b_impedances, MAX_uint32_T,
    c17_u_sf_marshallOut, c17_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_b_dampings, MAX_uint32_T,
    c17_u_sf_marshallOut, c17_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_nargin, 55U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_nargout, 56U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_constraints, 57U,
    c17_v_sf_marshallOut, c17_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_ConstraintsMatrix, 58U,
    c17_i_sf_marshallOut, c17_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_bVectorConstraints, 59U,
    c17_h_sf_marshallOut, c17_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_q, 60U, c17_c_sf_marshallOut,
    c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_qDes, 61U, c17_c_sf_marshallOut,
    c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_v, 62U, c17_s_sf_marshallOut,
    c17_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_M, 63U, c17_t_sf_marshallOut,
    c17_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_h, 64U, c17_s_sf_marshallOut,
    c17_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_H, 65U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_intHw, 66U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_w_H_l_contact, 67U,
    c17_m_sf_marshallOut, c17_r_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_w_H_r_contact, 68U,
    c17_m_sf_marshallOut, c17_r_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_JL, 69U, c17_q_sf_marshallOut,
    c17_s_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_JR, 70U, c17_q_sf_marshallOut,
    c17_s_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_dJLv, 71U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_dJRv, 72U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_xcom, 73U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_J_CoM, 74U, c17_q_sf_marshallOut,
    c17_s_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_desired_x_dx_ddx_CoM, 75U,
    c17_p_sf_marshallOut, c17_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_gainsPCOM, 76U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_gainsDCOM, 77U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_impedances, 54U, c17_c_sf_marshallOut,
    c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_intErrorCoM, 78U,
    c17_d_sf_marshallOut, c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_ki_int_qtilde, 79U,
    c17_c_sf_marshallOut, c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_b_reg, 80U, c17_o_sf_marshallOut,
    c17_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_b_gain, 81U, c17_n_sf_marshallOut,
    c17_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_w_H_lArm, 82U, c17_m_sf_marshallOut,
    c17_r_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_w_H_rArm, 83U, c17_m_sf_marshallOut,
    c17_r_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_LArmWrench, 84U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_RArmWrench, 85U, c17_sf_marshallOut,
    c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_useExtArmForces, 86U,
    c17_l_sf_marshallOut, c17_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_tauModel, 87U, c17_c_sf_marshallOut,
    c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_Sigma, 88U, c17_k_sf_marshallOut,
    c17_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_NA, 89U, c17_g_sf_marshallOut,
    c17_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_f_HDot, 90U, c17_b_sf_marshallOut,
    c17_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_HessianMatrixQP1Foot, 91U,
    c17_j_sf_marshallOut, c17_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_gradientQP1Foot, 92U,
    c17_sf_marshallOut, c17_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_ConstraintsMatrixQP1Foot, 93U,
    c17_i_sf_marshallOut, c17_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_bVectorConstraintsQp1Foot, 94U,
    c17_h_sf_marshallOut, c17_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_HessianMatrixQP2FeetOrLegs, 95U,
    c17_g_sf_marshallOut, c17_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_gradientQP2FeetOrLegs, 96U,
    c17_b_sf_marshallOut, c17_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_ConstraintsMatrixQP2FeetOrLegs, 97U,
    c17_f_sf_marshallOut, c17_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_bVectorConstraintsQp2FeetOrLegs, 98U,
    c17_e_sf_marshallOut, c17_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_errorCoM, 99U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_qTilde, 100U, c17_c_sf_marshallOut,
    c17_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_correctionFromSupportForce, 101U,
    c17_sf_marshallOut, c17_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 30);
  for (c17_i166 = 0; c17_i166 < 3; c17_i166++) {
    c17_pos_leftFoot[c17_i166] = c17_w_H_l_contact[c17_i166 + 12];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 31);
  c17_i167 = 0;
  c17_i168 = 0;
  for (c17_i169 = 0; c17_i169 < 3; c17_i169++) {
    for (c17_i170 = 0; c17_i170 < 3; c17_i170++) {
      c17_w_R_l_sole[c17_i170 + c17_i167] = c17_w_H_l_contact[c17_i170 +
        c17_i168];
    }

    c17_i167 += 3;
    c17_i168 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 33);
  for (c17_i171 = 0; c17_i171 < 3; c17_i171++) {
    c17_pos_rightFoot[c17_i171] = c17_w_H_r_contact[c17_i171 + 12];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 34);
  c17_i172 = 0;
  c17_i173 = 0;
  for (c17_i174 = 0; c17_i174 < 3; c17_i174++) {
    for (c17_i175 = 0; c17_i175 < 3; c17_i175++) {
      c17_w_R_r_sole[c17_i175 + c17_i172] = c17_w_H_r_contact[c17_i175 +
        c17_i173];
    }

    c17_i172 += 3;
    c17_i173 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 37);
  for (c17_i176 = 0; c17_i176 < 3; c17_i176++) {
    c17_pos_leftArm[c17_i176] = c17_w_H_lArm[c17_i176 + 12];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 38);
  for (c17_i177 = 0; c17_i177 < 3; c17_i177++) {
    c17_pos_rightArm[c17_i177] = c17_w_H_rArm[c17_i177 + 12];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 40);
  for (c17_i178 = 0; c17_i178 < 3; c17_i178++) {
    c17_gainsICOM[c17_i178] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 41);
  for (c17_i179 = 0; c17_i179 < 23; c17_i179++) {
    c17_dampings[c17_i179] = c17_b_gain->dampings[c17_i179];
  }

  _SFD_SYMBOL_SWITCH(8U, 8U);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 43);
  c17_ROBOT_DOF = 23.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 44);
  c17_gravAcc = 9.81;
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 47);
  c17_m = c17_M[0];
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 53);
  c17_i180 = 0;
  c17_i181 = 0;
  for (c17_i182 = 0; c17_i182 < 6; c17_i182++) {
    for (c17_i183 = 0; c17_i183 < 6; c17_i183++) {
      c17_Mb[c17_i183 + c17_i180] = c17_M[c17_i183 + c17_i181];
    }

    c17_i180 += 6;
    c17_i181 += 29;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 54);
  c17_i184 = 0;
  c17_i185 = 0;
  for (c17_i186 = 0; c17_i186 < 23; c17_i186++) {
    for (c17_i187 = 0; c17_i187 < 6; c17_i187++) {
      c17_Mbj[c17_i187 + c17_i184] = c17_M[(c17_i187 + c17_i185) + 174];
    }

    c17_i184 += 6;
    c17_i185 += 29;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 55);
  c17_i188 = 0;
  c17_i189 = 0;
  for (c17_i190 = 0; c17_i190 < 23; c17_i190++) {
    for (c17_i191 = 0; c17_i191 < 23; c17_i191++) {
      c17_Mj[c17_i191 + c17_i188] = c17_M[(c17_i191 + c17_i189) + 180];
    }

    c17_i188 += 23;
    c17_i189 += 29;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 57);
  for (c17_i192 = 0; c17_i192 < 667; c17_i192++) {
    c17_St[c17_i192] = c17_b[c17_i192];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 59);
  c17_a = -c17_m;
  c17_y = c17_a * 9.81;
  for (c17_i193 = 0; c17_i193 < 2; c17_i193++) {
    c17_gravityWrench[c17_i193] = 0.0;
  }

  c17_gravityWrench[2] = c17_y;
  for (c17_i194 = 0; c17_i194 < 3; c17_i194++) {
    c17_gravityWrench[c17_i194 + 3] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 64);
  c17_i195 = 0;
  c17_i196 = 0;
  for (c17_i197 = 0; c17_i197 < 29; c17_i197++) {
    for (c17_i198 = 0; c17_i198 < 3; c17_i198++) {
      c17_b_a[c17_i198 + c17_i195] = c17_J_CoM[c17_i198 + c17_i196];
    }

    c17_i195 += 3;
    c17_i196 += 6;
  }

  for (c17_i199 = 0; c17_i199 < 29; c17_i199++) {
    c17_b_b[c17_i199] = c17_v[c17_i199];
  }

  c17_b_eml_scalar_eg(chartInstance);
  c17_b_eml_scalar_eg(chartInstance);
  for (c17_i200 = 0; c17_i200 < 3; c17_i200++) {
    c17_xDcom[c17_i200] = 0.0;
  }

  for (c17_i201 = 0; c17_i201 < 3; c17_i201++) {
    c17_xDcom[c17_i201] = 0.0;
  }

  for (c17_i202 = 0; c17_i202 < 3; c17_i202++) {
    c17_C[c17_i202] = c17_xDcom[c17_i202];
  }

  for (c17_i203 = 0; c17_i203 < 3; c17_i203++) {
    c17_xDcom[c17_i203] = c17_C[c17_i203];
  }

  for (c17_i204 = 0; c17_i204 < 3; c17_i204++) {
    c17_C[c17_i204] = c17_xDcom[c17_i204];
  }

  for (c17_i205 = 0; c17_i205 < 3; c17_i205++) {
    c17_xDcom[c17_i205] = c17_C[c17_i205];
  }

  for (c17_i206 = 0; c17_i206 < 3; c17_i206++) {
    c17_xDcom[c17_i206] = 0.0;
    c17_i207 = 0;
    for (c17_i208 = 0; c17_i208 < 29; c17_i208++) {
      c17_xDcom[c17_i206] += c17_b_a[c17_i207 + c17_i206] * c17_b_b[c17_i208];
      c17_i207 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 67);
  for (c17_i209 = 0; c17_i209 < 23; c17_i209++) {
    c17_qD[c17_i209] = c17_v[c17_i209 + 6];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 70);
  for (c17_i210 = 0; c17_i210 < 23; c17_i210++) {
    c17_qTilde[c17_i210] = c17_q[c17_i210] - c17_qDes[c17_i210];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 73);
  for (c17_i211 = 0; c17_i211 < 3; c17_i211++) {
    c17_Pr[c17_i211] = c17_pos_rightFoot[c17_i211] - c17_xcom[c17_i211];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 76);
  for (c17_i212 = 0; c17_i212 < 3; c17_i212++) {
    c17_Pl[c17_i212] = c17_pos_leftFoot[c17_i212] - c17_xcom[c17_i212];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 79);
  for (c17_i213 = 0; c17_i213 < 3; c17_i213++) {
    c17_PlArm[c17_i213] = c17_pos_leftArm[c17_i213] - c17_xcom[c17_i213];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 80);
  for (c17_i214 = 0; c17_i214 < 3; c17_i214++) {
    c17_PrArm[c17_i214] = c17_pos_rightArm[c17_i214] - c17_xcom[c17_i214];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 90);
  c17_eye(chartInstance, c17_dv16);
  for (c17_i215 = 0; c17_i215 < 3; c17_i215++) {
    c17_b_Pl[c17_i215] = c17_Pl[c17_i215];
  }

  c17_Sf(chartInstance, c17_b_Pl, c17_dv17);
  c17_eye(chartInstance, c17_dv18);
  c17_i216 = 0;
  c17_i217 = 0;
  for (c17_i218 = 0; c17_i218 < 3; c17_i218++) {
    for (c17_i219 = 0; c17_i219 < 3; c17_i219++) {
      c17_AL[c17_i219 + c17_i216] = c17_dv16[c17_i219 + c17_i217];
    }

    c17_i216 += 6;
    c17_i217 += 3;
  }

  c17_i220 = 0;
  for (c17_i221 = 0; c17_i221 < 3; c17_i221++) {
    for (c17_i222 = 0; c17_i222 < 3; c17_i222++) {
      c17_AL[(c17_i222 + c17_i220) + 18] = 0.0;
    }

    c17_i220 += 6;
  }

  c17_i223 = 0;
  c17_i224 = 0;
  for (c17_i225 = 0; c17_i225 < 3; c17_i225++) {
    for (c17_i226 = 0; c17_i226 < 3; c17_i226++) {
      c17_AL[(c17_i226 + c17_i223) + 3] = c17_dv17[c17_i226 + c17_i224];
    }

    c17_i223 += 6;
    c17_i224 += 3;
  }

  c17_i227 = 0;
  c17_i228 = 0;
  for (c17_i229 = 0; c17_i229 < 3; c17_i229++) {
    for (c17_i230 = 0; c17_i230 < 3; c17_i230++) {
      c17_AL[(c17_i230 + c17_i227) + 21] = c17_dv18[c17_i230 + c17_i228];
    }

    c17_i227 += 6;
    c17_i228 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 92);
  c17_eye(chartInstance, c17_dv16);
  for (c17_i231 = 0; c17_i231 < 3; c17_i231++) {
    c17_b_Pr[c17_i231] = c17_Pr[c17_i231];
  }

  c17_Sf(chartInstance, c17_b_Pr, c17_dv17);
  c17_eye(chartInstance, c17_dv18);
  c17_i232 = 0;
  c17_i233 = 0;
  for (c17_i234 = 0; c17_i234 < 3; c17_i234++) {
    for (c17_i235 = 0; c17_i235 < 3; c17_i235++) {
      c17_AR[c17_i235 + c17_i232] = c17_dv16[c17_i235 + c17_i233];
    }

    c17_i232 += 6;
    c17_i233 += 3;
  }

  c17_i236 = 0;
  for (c17_i237 = 0; c17_i237 < 3; c17_i237++) {
    for (c17_i238 = 0; c17_i238 < 3; c17_i238++) {
      c17_AR[(c17_i238 + c17_i236) + 18] = 0.0;
    }

    c17_i236 += 6;
  }

  c17_i239 = 0;
  c17_i240 = 0;
  for (c17_i241 = 0; c17_i241 < 3; c17_i241++) {
    for (c17_i242 = 0; c17_i242 < 3; c17_i242++) {
      c17_AR[(c17_i242 + c17_i239) + 3] = c17_dv17[c17_i242 + c17_i240];
    }

    c17_i239 += 6;
    c17_i240 += 3;
  }

  c17_i243 = 0;
  c17_i244 = 0;
  for (c17_i245 = 0; c17_i245 < 3; c17_i245++) {
    for (c17_i246 = 0; c17_i246 < 3; c17_i246++) {
      c17_AR[(c17_i246 + c17_i243) + 21] = c17_dv18[c17_i246 + c17_i244];
    }

    c17_i243 += 6;
    c17_i244 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 95);
  c17_i247 = 0;
  for (c17_i248 = 0; c17_i248 < 6; c17_i248++) {
    for (c17_i249 = 0; c17_i249 < 6; c17_i249++) {
      c17_A[c17_i249 + c17_i247] = c17_AL[c17_i249 + c17_i247];
    }

    c17_i247 += 6;
  }

  c17_i250 = 0;
  for (c17_i251 = 0; c17_i251 < 6; c17_i251++) {
    for (c17_i252 = 0; c17_i252 < 6; c17_i252++) {
      c17_A[(c17_i252 + c17_i250) + 36] = c17_AR[c17_i252 + c17_i250];
    }

    c17_i250 += 6;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 97);
  for (c17_i253 = 0; c17_i253 < 72; c17_i253++) {
    c17_b_A[c17_i253] = c17_A[c17_i253];
  }

  c17_pinv(chartInstance, c17_b_A, c17_b_reg.pinvTol, c17_c_a);
  c17_c_b = c17_constraints[0];
  for (c17_i254 = 0; c17_i254 < 72; c17_i254++) {
    c17_c_a[c17_i254] *= c17_c_b;
  }

  c17_d_b = c17_constraints[1];
  for (c17_i255 = 0; c17_i255 < 72; c17_i255++) {
    c17_c_a[c17_i255] *= c17_d_b;
  }

  for (c17_i256 = 0; c17_i256 < 36; c17_i256++) {
    c17_b_AL[c17_i256] = c17_AL[c17_i256];
  }

  c17_inv(chartInstance, c17_b_AL, c17_e_b);
  c17_i257 = 0;
  c17_i258 = 0;
  for (c17_i259 = 0; c17_i259 < 6; c17_i259++) {
    for (c17_i260 = 0; c17_i260 < 6; c17_i260++) {
      c17_d_a[c17_i260 + c17_i257] = c17_e_b[c17_i260 + c17_i258];
    }

    c17_i257 += 12;
    c17_i258 += 6;
  }

  c17_i261 = 0;
  for (c17_i262 = 0; c17_i262 < 6; c17_i262++) {
    for (c17_i263 = 0; c17_i263 < 6; c17_i263++) {
      c17_d_a[(c17_i263 + c17_i261) + 6] = 0.0;
    }

    c17_i261 += 12;
  }

  c17_f_b = c17_constraints[0];
  for (c17_i264 = 0; c17_i264 < 72; c17_i264++) {
    c17_d_a[c17_i264] *= c17_f_b;
  }

  c17_g_b = 1.0 - c17_constraints[1];
  for (c17_i265 = 0; c17_i265 < 72; c17_i265++) {
    c17_d_a[c17_i265] *= c17_g_b;
  }

  for (c17_i266 = 0; c17_i266 < 36; c17_i266++) {
    c17_b_AR[c17_i266] = c17_AR[c17_i266];
  }

  c17_inv(chartInstance, c17_b_AR, c17_e_b);
  c17_i267 = 0;
  for (c17_i268 = 0; c17_i268 < 6; c17_i268++) {
    for (c17_i269 = 0; c17_i269 < 6; c17_i269++) {
      c17_e_a[c17_i269 + c17_i267] = 0.0;
    }

    c17_i267 += 12;
  }

  c17_i270 = 0;
  c17_i271 = 0;
  for (c17_i272 = 0; c17_i272 < 6; c17_i272++) {
    for (c17_i273 = 0; c17_i273 < 6; c17_i273++) {
      c17_e_a[(c17_i273 + c17_i270) + 6] = c17_e_b[c17_i273 + c17_i271];
    }

    c17_i270 += 12;
    c17_i271 += 6;
  }

  c17_h_b = c17_constraints[1];
  for (c17_i274 = 0; c17_i274 < 72; c17_i274++) {
    c17_e_a[c17_i274] *= c17_h_b;
  }

  c17_i_b = 1.0 - c17_constraints[0];
  for (c17_i275 = 0; c17_i275 < 72; c17_i275++) {
    c17_e_a[c17_i275] *= c17_i_b;
  }

  for (c17_i276 = 0; c17_i276 < 72; c17_i276++) {
    c17_pinvA[c17_i276] = (c17_c_a[c17_i276] + c17_d_a[c17_i276]) +
      c17_e_a[c17_i276];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 102);
  c17_eye(chartInstance, c17_dv16);
  for (c17_i277 = 0; c17_i277 < 3; c17_i277++) {
    c17_b_PlArm[c17_i277] = c17_PlArm[c17_i277];
  }

  c17_Sf(chartInstance, c17_b_PlArm, c17_dv17);
  c17_eye(chartInstance, c17_dv18);
  c17_i278 = 0;
  c17_i279 = 0;
  for (c17_i280 = 0; c17_i280 < 3; c17_i280++) {
    for (c17_i281 = 0; c17_i281 < 3; c17_i281++) {
      c17_A_lArm[c17_i281 + c17_i278] = c17_dv16[c17_i281 + c17_i279];
    }

    c17_i278 += 6;
    c17_i279 += 3;
  }

  c17_i282 = 0;
  for (c17_i283 = 0; c17_i283 < 3; c17_i283++) {
    for (c17_i284 = 0; c17_i284 < 3; c17_i284++) {
      c17_A_lArm[(c17_i284 + c17_i282) + 18] = 0.0;
    }

    c17_i282 += 6;
  }

  c17_i285 = 0;
  c17_i286 = 0;
  for (c17_i287 = 0; c17_i287 < 3; c17_i287++) {
    for (c17_i288 = 0; c17_i288 < 3; c17_i288++) {
      c17_A_lArm[(c17_i288 + c17_i285) + 3] = c17_dv17[c17_i288 + c17_i286];
    }

    c17_i285 += 6;
    c17_i286 += 3;
  }

  c17_i289 = 0;
  c17_i290 = 0;
  for (c17_i291 = 0; c17_i291 < 3; c17_i291++) {
    for (c17_i292 = 0; c17_i292 < 3; c17_i292++) {
      c17_A_lArm[(c17_i292 + c17_i289) + 21] = c17_dv18[c17_i292 + c17_i290];
    }

    c17_i289 += 6;
    c17_i290 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 104);
  c17_eye(chartInstance, c17_dv16);
  for (c17_i293 = 0; c17_i293 < 3; c17_i293++) {
    c17_b_PrArm[c17_i293] = c17_PrArm[c17_i293];
  }

  c17_Sf(chartInstance, c17_b_PrArm, c17_dv17);
  c17_eye(chartInstance, c17_dv18);
  c17_i294 = 0;
  c17_i295 = 0;
  for (c17_i296 = 0; c17_i296 < 3; c17_i296++) {
    for (c17_i297 = 0; c17_i297 < 3; c17_i297++) {
      c17_A_rArm[c17_i297 + c17_i294] = c17_dv16[c17_i297 + c17_i295];
    }

    c17_i294 += 6;
    c17_i295 += 3;
  }

  c17_i298 = 0;
  for (c17_i299 = 0; c17_i299 < 3; c17_i299++) {
    for (c17_i300 = 0; c17_i300 < 3; c17_i300++) {
      c17_A_rArm[(c17_i300 + c17_i298) + 18] = 0.0;
    }

    c17_i298 += 6;
  }

  c17_i301 = 0;
  c17_i302 = 0;
  for (c17_i303 = 0; c17_i303 < 3; c17_i303++) {
    for (c17_i304 = 0; c17_i304 < 3; c17_i304++) {
      c17_A_rArm[(c17_i304 + c17_i301) + 3] = c17_dv17[c17_i304 + c17_i302];
    }

    c17_i301 += 6;
    c17_i302 += 3;
  }

  c17_i305 = 0;
  c17_i306 = 0;
  for (c17_i307 = 0; c17_i307 < 3; c17_i307++) {
    for (c17_i308 = 0; c17_i308 < 3; c17_i308++) {
      c17_A_rArm[(c17_i308 + c17_i305) + 21] = c17_dv18[c17_i308 + c17_i306];
    }

    c17_i305 += 6;
    c17_i306 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 107);
  c17_i309 = 0;
  for (c17_i310 = 0; c17_i310 < 6; c17_i310++) {
    for (c17_i311 = 0; c17_i311 < 6; c17_i311++) {
      c17_A_arms[c17_i311 + c17_i309] = c17_A_lArm[c17_i311 + c17_i309];
    }

    c17_i309 += 6;
  }

  c17_i312 = 0;
  for (c17_i313 = 0; c17_i313 < 6; c17_i313++) {
    for (c17_i314 = 0; c17_i314 < 6; c17_i314++) {
      c17_A_arms[(c17_i314 + c17_i312) + 36] = c17_A_rArm[c17_i314 + c17_i312];
    }

    c17_i312 += 6;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 114);
  for (c17_i315 = 0; c17_i315 < 6; c17_i315++) {
    c17_fArms[c17_i315] = c17_LArmWrench[c17_i315];
  }

  for (c17_i316 = 0; c17_i316 < 6; c17_i316++) {
    c17_fArms[c17_i316 + 6] = c17_RArmWrench[c17_i316];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 116);
  for (c17_i317 = 0; c17_i317 < 72; c17_i317++) {
    c17_f_a[c17_i317] = c17_A_arms[c17_i317];
  }

  for (c17_i318 = 0; c17_i318 < 12; c17_i318++) {
    c17_j_b[c17_i318] = c17_fArms[c17_i318];
  }

  c17_f_eml_scalar_eg(chartInstance);
  c17_f_eml_scalar_eg(chartInstance);
  for (c17_i319 = 0; c17_i319 < 6; c17_i319++) {
    c17_fsupport[c17_i319] = 0.0;
  }

  for (c17_i320 = 0; c17_i320 < 6; c17_i320++) {
    c17_fsupport[c17_i320] = 0.0;
  }

  for (c17_i321 = 0; c17_i321 < 6; c17_i321++) {
    c17_b_C[c17_i321] = c17_fsupport[c17_i321];
  }

  for (c17_i322 = 0; c17_i322 < 6; c17_i322++) {
    c17_fsupport[c17_i322] = c17_b_C[c17_i322];
  }

  for (c17_i323 = 0; c17_i323 < 6; c17_i323++) {
    c17_b_C[c17_i323] = c17_fsupport[c17_i323];
  }

  for (c17_i324 = 0; c17_i324 < 6; c17_i324++) {
    c17_fsupport[c17_i324] = c17_b_C[c17_i324];
  }

  for (c17_i325 = 0; c17_i325 < 6; c17_i325++) {
    c17_fsupport[c17_i325] = 0.0;
    c17_i326 = 0;
    for (c17_i327 = 0; c17_i327 < 12; c17_i327++) {
      c17_fsupport[c17_i325] += c17_f_a[c17_i326 + c17_i325] * c17_j_b[c17_i327];
      c17_i326 += 6;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 119);
  for (c17_i328 = 0; c17_i328 < 3; c17_i328++) {
    c17_xDDcomStar[c17_i328] = ((c17_desired_x_dx_ddx_CoM[c17_i328 + 6] -
      c17_gainsPCOM[c17_i328] * (c17_xcom[c17_i328] -
      c17_desired_x_dx_ddx_CoM[c17_i328])) - c17_gainsICOM[c17_i328] *
      c17_intErrorCoM[c17_i328]) - c17_gainsDCOM[c17_i328] * (c17_xDcom[c17_i328]
      - c17_desired_x_dx_ddx_CoM[c17_i328 + 3]);
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 123);
  for (c17_i329 = 0; c17_i329 < 3; c17_i329++) {
    c17_H_desired[c17_i329] = c17_m * c17_desired_x_dx_ddx_CoM[c17_i329 + 3];
  }

  for (c17_i330 = 0; c17_i330 < 3; c17_i330++) {
    c17_H_desired[c17_i330 + 3] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 126);
  for (c17_i331 = 0; c17_i331 < 6; c17_i331++) {
    c17_H_error[c17_i331] = c17_H[c17_i331] - c17_H_desired[c17_i331];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 130U);
  for (c17_i332 = 0; c17_i332 < 6; c17_i332++) {
    c17_g_a[c17_i332] = c17_H_error[c17_i332];
  }

  for (c17_i333 = 0; c17_i333 < 6; c17_i333++) {
    c17_k_b[c17_i333] = c17_fsupport[c17_i333];
  }

  c17_g_eml_scalar_eg(chartInstance);
  c17_g_eml_scalar_eg(chartInstance);
  c17_b_y = 0.0;
  for (c17_k = 1; c17_k < 7; c17_k++) {
    c17_b_k = c17_k;
    c17_b_y += c17_g_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1] *
      c17_k_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 6, 1, 0) - 1];
  }

  c17_c_A = c17_b_y;
  for (c17_i334 = 0; c17_i334 < 6; c17_i334++) {
    c17_b_H_error[c17_i334] = c17_H_error[c17_i334];
  }

  c17_B = c17_b_norm(chartInstance, c17_b_H_error) + c17_b_reg.norm_tolerance;
  c17_x = c17_c_A;
  c17_c_y = c17_B;
  c17_b_x = c17_x;
  c17_d_y = c17_c_y;
  c17_alpha = c17_b_x / c17_d_y;
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 131U);
  for (c17_i335 = 0; c17_i335 < 6; c17_i335++) {
    c17_b_C[c17_i335] = c17_H_error[c17_i335];
  }

  for (c17_i336 = 0; c17_i336 < 6; c17_i336++) {
    c17_c_H_error[c17_i336] = c17_H_error[c17_i336];
  }

  c17_b_B = c17_b_norm(chartInstance, c17_c_H_error) + c17_b_reg.norm_tolerance;
  c17_e_y = c17_b_B;
  c17_f_y = c17_e_y;
  for (c17_i337 = 0; c17_i337 < 6; c17_i337++) {
    c17_H_errParallel[c17_i337] = c17_b_C[c17_i337] / c17_f_y;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 133U);
  guard1 = FALSE;
  if (CV_SCRIPT_COND(0, 0, (real_T)c17_useExtArmForces == 1.0)) {
    if (CV_SCRIPT_COND(0, 1, c17_alpha <= 0.0)) {
      CV_SCRIPT_MCDC(0, 0, TRUE);
      CV_SCRIPT_IF(0, 0, TRUE);
      _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 135U);
      c17_h_a = c17_alpha;
      for (c17_i338 = 0; c17_i338 < 6; c17_i338++) {
        c17_k_b[c17_i338] = c17_H_errParallel[c17_i338];
      }

      for (c17_i339 = 0; c17_i339 < 6; c17_i339++) {
        c17_correctionFromSupportForce[c17_i339] = c17_h_a * c17_k_b[c17_i339];
      }
    } else {
      guard1 = TRUE;
    }
  } else {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    CV_SCRIPT_MCDC(0, 0, FALSE);
    CV_SCRIPT_IF(0, 0, FALSE);
    _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 137U);
    for (c17_i340 = 0; c17_i340 < 6; c17_i340++) {
      c17_correctionFromSupportForce[c17_i340] = 0.0;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 141U);
  for (c17_i341 = 0; c17_i341 < 72; c17_i341++) {
    c17_c_a[c17_i341] = c17_pinvA[c17_i341];
  }

  for (c17_i342 = 0; c17_i342 < 72; c17_i342++) {
    c17_f_a[c17_i342] = c17_A[c17_i342];
  }

  c17_h_eml_scalar_eg(chartInstance);
  c17_h_eml_scalar_eg(chartInstance);
  for (c17_i343 = 0; c17_i343 < 144; c17_i343++) {
    c17_g_y[c17_i343] = 0.0;
  }

  for (c17_i344 = 0; c17_i344 < 72; c17_i344++) {
    c17_i_a[c17_i344] = c17_c_a[c17_i344];
  }

  for (c17_i345 = 0; c17_i345 < 72; c17_i345++) {
    c17_j_a[c17_i345] = c17_f_a[c17_i345];
  }

  c17_q_eml_xgemm(chartInstance, c17_i_a, c17_j_a, c17_g_y);
  c17_b_eye(chartInstance, c17_k_a);
  for (c17_i346 = 0; c17_i346 < 144; c17_i346++) {
    c17_k_a[c17_i346] -= c17_g_y[c17_i346];
  }

  c17_l_b = c17_constraints[0];
  for (c17_i347 = 0; c17_i347 < 144; c17_i347++) {
    c17_k_a[c17_i347] *= c17_l_b;
  }

  c17_m_b = c17_constraints[1];
  for (c17_i348 = 0; c17_i348 < 144; c17_i348++) {
    c17_NA[c17_i348] = c17_k_a[c17_i348] * c17_m_b;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 144U);
  for (c17_i349 = 0; c17_i349 < 174; c17_i349++) {
    c17_l_a[c17_i349] = c17_JL[c17_i349];
  }

  c17_n_b = c17_constraints[0];
  for (c17_i350 = 0; c17_i350 < 174; c17_i350++) {
    c17_l_a[c17_i350] *= c17_n_b;
  }

  for (c17_i351 = 0; c17_i351 < 174; c17_i351++) {
    c17_m_a[c17_i351] = c17_JR[c17_i351];
  }

  c17_o_b = c17_constraints[1];
  for (c17_i352 = 0; c17_i352 < 174; c17_i352++) {
    c17_m_a[c17_i352] *= c17_o_b;
  }

  c17_i353 = 0;
  c17_i354 = 0;
  for (c17_i355 = 0; c17_i355 < 29; c17_i355++) {
    for (c17_i356 = 0; c17_i356 < 6; c17_i356++) {
      c17_Jc[c17_i356 + c17_i353] = c17_l_a[c17_i356 + c17_i354];
    }

    c17_i353 += 12;
    c17_i354 += 6;
  }

  c17_i357 = 0;
  c17_i358 = 0;
  for (c17_i359 = 0; c17_i359 < 29; c17_i359++) {
    for (c17_i360 = 0; c17_i360 < 6; c17_i360++) {
      c17_Jc[(c17_i360 + c17_i357) + 6] = c17_m_a[c17_i360 + c17_i358];
    }

    c17_i357 += 12;
    c17_i358 += 6;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 148U);
  for (c17_i361 = 0; c17_i361 < 6; c17_i361++) {
    c17_b_C[c17_i361] = c17_dJLv[c17_i361];
  }

  c17_p_b = c17_constraints[0];
  for (c17_i362 = 0; c17_i362 < 6; c17_i362++) {
    c17_b_C[c17_i362] *= c17_p_b;
  }

  for (c17_i363 = 0; c17_i363 < 6; c17_i363++) {
    c17_k_b[c17_i363] = c17_dJRv[c17_i363];
  }

  c17_q_b = c17_constraints[1];
  for (c17_i364 = 0; c17_i364 < 6; c17_i364++) {
    c17_k_b[c17_i364] *= c17_q_b;
  }

  for (c17_i365 = 0; c17_i365 < 6; c17_i365++) {
    c17_JcDv[c17_i365] = c17_b_C[c17_i365];
  }

  for (c17_i366 = 0; c17_i366 < 6; c17_i366++) {
    c17_JcDv[c17_i366 + 6] = c17_k_b[c17_i366];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 151U);
  for (c17_i367 = 0; c17_i367 < 348; c17_i367++) {
    c17_b_Jc[c17_i367] = c17_Jc[c17_i367];
  }

  for (c17_i368 = 0; c17_i368 < 841; c17_i368++) {
    c17_b_M[c17_i368] = c17_M[c17_i368];
  }

  c17_mrdivide(chartInstance, c17_b_Jc, c17_b_M, c17_dv19);
  for (c17_i369 = 0; c17_i369 < 348; c17_i369++) {
    c17_JcMinv[c17_i369] = c17_dv19[c17_i369];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 152U);
  for (c17_i370 = 0; c17_i370 < 348; c17_i370++) {
    c17_n_a[c17_i370] = c17_JcMinv[c17_i370];
  }

  c17_j_eml_scalar_eg(chartInstance);
  c17_j_eml_scalar_eg(chartInstance);
  for (c17_i371 = 0; c17_i371 < 276; c17_i371++) {
    c17_JcMinvSt[c17_i371] = 0.0;
  }

  for (c17_i372 = 0; c17_i372 < 276; c17_i372++) {
    c17_JcMinvSt[c17_i372] = 0.0;
  }

  for (c17_i373 = 0; c17_i373 < 348; c17_i373++) {
    c17_dv20[c17_i373] = c17_n_a[c17_i373];
  }

  for (c17_i374 = 0; c17_i374 < 667; c17_i374++) {
    c17_dv21[c17_i374] = c17_b[c17_i374];
  }

  for (c17_i375 = 0; c17_i375 < 348; c17_i375++) {
    c17_dv22[c17_i375] = c17_dv20[c17_i375];
  }

  for (c17_i376 = 0; c17_i376 < 667; c17_i376++) {
    c17_dv23[c17_i376] = c17_dv21[c17_i376];
  }

  c17_r_eml_xgemm(chartInstance, c17_dv22, c17_dv23, c17_JcMinvSt);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 153U);
  for (c17_i377 = 0; c17_i377 < 348; c17_i377++) {
    c17_n_a[c17_i377] = c17_JcMinv[c17_i377];
  }

  c17_i378 = 0;
  for (c17_i379 = 0; c17_i379 < 12; c17_i379++) {
    c17_i380 = 0;
    for (c17_i381 = 0; c17_i381 < 29; c17_i381++) {
      c17_r_b[c17_i381 + c17_i378] = c17_Jc[c17_i380 + c17_i379];
      c17_i380 += 12;
    }

    c17_i378 += 29;
  }

  c17_k_eml_scalar_eg(chartInstance);
  c17_k_eml_scalar_eg(chartInstance);
  for (c17_i382 = 0; c17_i382 < 144; c17_i382++) {
    c17_JcMinvJct[c17_i382] = 0.0;
  }

  for (c17_i383 = 0; c17_i383 < 144; c17_i383++) {
    c17_JcMinvJct[c17_i383] = 0.0;
  }

  for (c17_i384 = 0; c17_i384 < 348; c17_i384++) {
    c17_dv24[c17_i384] = c17_n_a[c17_i384];
  }

  for (c17_i385 = 0; c17_i385 < 348; c17_i385++) {
    c17_dv25[c17_i385] = c17_r_b[c17_i385];
  }

  for (c17_i386 = 0; c17_i386 < 348; c17_i386++) {
    c17_dv26[c17_i386] = c17_dv24[c17_i386];
  }

  for (c17_i387 = 0; c17_i387 < 348; c17_i387++) {
    c17_dv27[c17_i387] = c17_dv25[c17_i387];
  }

  c17_s_eml_xgemm(chartInstance, c17_dv26, c17_dv27, c17_JcMinvJct);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 154U);
  c17_i388 = 0;
  for (c17_i389 = 0; c17_i389 < 6; c17_i389++) {
    c17_i390 = 0;
    for (c17_i391 = 0; c17_i391 < 23; c17_i391++) {
      c17_b_Mbj[c17_i391 + c17_i388] = c17_Mbj[c17_i390 + c17_i389];
      c17_i390 += 6;
    }

    c17_i388 += 23;
  }

  for (c17_i392 = 0; c17_i392 < 36; c17_i392++) {
    c17_b_Mb[c17_i392] = c17_Mb[c17_i392];
  }

  c17_b_mrdivide(chartInstance, c17_b_Mbj, c17_b_Mb, c17_o_a);
  c17_i393 = 0;
  for (c17_i394 = 0; c17_i394 < 12; c17_i394++) {
    c17_i395 = 0;
    for (c17_i396 = 0; c17_i396 < 6; c17_i396++) {
      c17_f_a[c17_i396 + c17_i393] = c17_Jc[c17_i395 + c17_i394];
      c17_i395 += 12;
    }

    c17_i393 += 6;
  }

  c17_l_eml_scalar_eg(chartInstance);
  c17_l_eml_scalar_eg(chartInstance);
  for (c17_i397 = 0; c17_i397 < 276; c17_i397++) {
    c17_h_y[c17_i397] = 0.0;
  }

  for (c17_i398 = 0; c17_i398 < 138; c17_i398++) {
    c17_p_a[c17_i398] = c17_o_a[c17_i398];
  }

  for (c17_i399 = 0; c17_i399 < 72; c17_i399++) {
    c17_q_a[c17_i399] = c17_f_a[c17_i399];
  }

  c17_t_eml_xgemm(chartInstance, c17_p_a, c17_q_a, c17_h_y);
  c17_i400 = 0;
  for (c17_i401 = 0; c17_i401 < 12; c17_i401++) {
    c17_i402 = 0;
    for (c17_i403 = 0; c17_i403 < 23; c17_i403++) {
      c17_JBar[c17_i403 + c17_i400] = c17_Jc[(c17_i402 + c17_i401) + 72] -
        c17_h_y[c17_i403 + c17_i400];
      c17_i402 += 12;
    }

    c17_i400 += 23;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 156U);
  for (c17_i404 = 0; c17_i404 < 276; c17_i404++) {
    c17_b_JcMinvSt[c17_i404] = c17_JcMinvSt[c17_i404];
  }

  c17_pinvDamped(chartInstance, c17_b_JcMinvSt, c17_b_reg.pinvDamp, c17_dv28);
  for (c17_i405 = 0; c17_i405 < 276; c17_i405++) {
    c17_PInv_JcMinvSt[c17_i405] = c17_dv28[c17_i405];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 159U);
  for (c17_i406 = 0; c17_i406 < 276; c17_i406++) {
    c17_r_a[c17_i406] = c17_PInv_JcMinvSt[c17_i406];
  }

  for (c17_i407 = 0; c17_i407 < 276; c17_i407++) {
    c17_s_b[c17_i407] = c17_JcMinvSt[c17_i407];
  }

  c17_n_eml_scalar_eg(chartInstance);
  c17_n_eml_scalar_eg(chartInstance);
  for (c17_i408 = 0; c17_i408 < 529; c17_i408++) {
    c17_i_y[c17_i408] = 0.0;
  }

  for (c17_i409 = 0; c17_i409 < 276; c17_i409++) {
    c17_s_a[c17_i409] = c17_r_a[c17_i409];
  }

  for (c17_i410 = 0; c17_i410 < 276; c17_i410++) {
    c17_t_b[c17_i410] = c17_s_b[c17_i410];
  }

  c17_v_eml_xgemm(chartInstance, c17_s_a, c17_t_b, c17_i_y);
  c17_c_eye(chartInstance, c17_t_a);
  for (c17_i411 = 0; c17_i411 < 529; c17_i411++) {
    c17_nullJcMinvSt[c17_i411] = c17_t_a[c17_i411] - c17_i_y[c17_i411];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 162U);
  c17_i412 = 0;
  for (c17_i413 = 0; c17_i413 < 6; c17_i413++) {
    c17_i414 = 0;
    for (c17_i415 = 0; c17_i415 < 23; c17_i415++) {
      c17_c_Mbj[c17_i415 + c17_i412] = c17_Mbj[c17_i414 + c17_i413];
      c17_i414 += 6;
    }

    c17_i412 += 23;
  }

  for (c17_i416 = 0; c17_i416 < 36; c17_i416++) {
    c17_c_Mb[c17_i416] = c17_Mb[c17_i416];
  }

  c17_b_mrdivide(chartInstance, c17_c_Mbj, c17_c_Mb, c17_o_a);
  for (c17_i417 = 0; c17_i417 < 138; c17_i417++) {
    c17_u_b[c17_i417] = c17_Mbj[c17_i417];
  }

  c17_p_eml_scalar_eg(chartInstance);
  c17_p_eml_scalar_eg(chartInstance);
  for (c17_i418 = 0; c17_i418 < 529; c17_i418++) {
    c17_i_y[c17_i418] = 0.0;
  }

  for (c17_i419 = 0; c17_i419 < 138; c17_i419++) {
    c17_u_a[c17_i419] = c17_o_a[c17_i419];
  }

  for (c17_i420 = 0; c17_i420 < 138; c17_i420++) {
    c17_v_b[c17_i420] = c17_u_b[c17_i420];
  }

  c17_w_eml_xgemm(chartInstance, c17_u_a, c17_v_b, c17_i_y);
  for (c17_i421 = 0; c17_i421 < 529; c17_i421++) {
    c17_Mbar[c17_i421] = c17_Mj[c17_i421] - c17_i_y[c17_i421];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 163U);
  for (c17_i422 = 0; c17_i422 < 529; c17_i422++) {
    c17_t_a[c17_i422] = c17_nullJcMinvSt[c17_i422];
  }

  for (c17_i423 = 0; c17_i423 < 529; c17_i423++) {
    c17_w_b[c17_i423] = c17_Mbar[c17_i423];
  }

  c17_q_eml_scalar_eg(chartInstance);
  c17_q_eml_scalar_eg(chartInstance);
  for (c17_i424 = 0; c17_i424 < 529; c17_i424++) {
    c17_NLMbar[c17_i424] = 0.0;
  }

  for (c17_i425 = 0; c17_i425 < 529; c17_i425++) {
    c17_NLMbar[c17_i425] = 0.0;
  }

  for (c17_i426 = 0; c17_i426 < 529; c17_i426++) {
    c17_dv29[c17_i426] = c17_t_a[c17_i426];
  }

  for (c17_i427 = 0; c17_i427 < 529; c17_i427++) {
    c17_dv30[c17_i427] = c17_w_b[c17_i427];
  }

  for (c17_i428 = 0; c17_i428 < 529; c17_i428++) {
    c17_dv31[c17_i428] = c17_dv29[c17_i428];
  }

  for (c17_i429 = 0; c17_i429 < 529; c17_i429++) {
    c17_dv32[c17_i429] = c17_dv30[c17_i429];
  }

  c17_x_eml_xgemm(chartInstance, c17_dv31, c17_dv32, c17_NLMbar);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 167U);
  for (c17_i430 = 0; c17_i430 < 23; c17_i430++) {
    c17_c_impedances[c17_i430] = c17_impedances[c17_i430];
  }

  c17_diag(chartInstance, c17_c_impedances, c17_t_a);
  for (c17_i431 = 0; c17_i431 < 529; c17_i431++) {
    c17_b_NLMbar[c17_i431] = c17_NLMbar[c17_i431];
  }

  c17_b_pinv(chartInstance, c17_b_NLMbar, c17_b_reg.pinvTol, c17_w_b);
  c17_q_eml_scalar_eg(chartInstance);
  c17_q_eml_scalar_eg(chartInstance);
  for (c17_i432 = 0; c17_i432 < 529; c17_i432++) {
    c17_i_y[c17_i432] = 0.0;
  }

  for (c17_i433 = 0; c17_i433 < 529; c17_i433++) {
    c17_v_a[c17_i433] = c17_t_a[c17_i433];
  }

  for (c17_i434 = 0; c17_i434 < 529; c17_i434++) {
    c17_x_b[c17_i434] = c17_w_b[c17_i434];
  }

  c17_x_eml_xgemm(chartInstance, c17_v_a, c17_x_b, c17_i_y);
  c17_w_a = c17_b_reg.impedances;
  for (c17_i435 = 0; c17_i435 < 529; c17_i435++) {
    c17_t_a[c17_i435] = c17_w_a * c17_y_b[c17_i435];
  }

  for (c17_i436 = 0; c17_i436 < 529; c17_i436++) {
    c17_b_impedances[c17_i436] = c17_i_y[c17_i436] + c17_t_a[c17_i436];
  }

  _SFD_SYMBOL_SWITCH(54U, 54U);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 168U);
  for (c17_i437 = 0; c17_i437 < 23; c17_i437++) {
    c17_c_dampings[c17_i437] = c17_dampings[c17_i437];
  }

  c17_b_diag(chartInstance, c17_c_dampings, c17_t_a);
  for (c17_i438 = 0; c17_i438 < 529; c17_i438++) {
    c17_c_NLMbar[c17_i438] = c17_NLMbar[c17_i438];
  }

  c17_b_pinv(chartInstance, c17_c_NLMbar, c17_b_reg.pinvTol, c17_w_b);
  c17_q_eml_scalar_eg(chartInstance);
  c17_q_eml_scalar_eg(chartInstance);
  for (c17_i439 = 0; c17_i439 < 529; c17_i439++) {
    c17_i_y[c17_i439] = 0.0;
  }

  for (c17_i440 = 0; c17_i440 < 529; c17_i440++) {
    c17_x_a[c17_i440] = c17_t_a[c17_i440];
  }

  for (c17_i441 = 0; c17_i441 < 529; c17_i441++) {
    c17_ab_b[c17_i441] = c17_w_b[c17_i441];
  }

  c17_x_eml_xgemm(chartInstance, c17_x_a, c17_ab_b, c17_i_y);
  c17_y_a = c17_b_reg.dampings;
  for (c17_i442 = 0; c17_i442 < 529; c17_i442++) {
    c17_t_a[c17_i442] = c17_y_a * c17_y_b[c17_i442];
  }

  for (c17_i443 = 0; c17_i443 < 529; c17_i443++) {
    c17_b_dampings[c17_i443] = c17_i_y[c17_i443] + c17_t_a[c17_i443];
  }

  _SFD_SYMBOL_SWITCH(8U, 55U);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 202U);
  for (c17_i444 = 0; c17_i444 < 114; c17_i444++) {
    c17_ab_a[c17_i444] = c17_ConstraintsMatrix[c17_i444];
  }

  c17_i445 = 0;
  for (c17_i446 = 0; c17_i446 < 3; c17_i446++) {
    c17_i447 = 0;
    for (c17_i448 = 0; c17_i448 < 3; c17_i448++) {
      c17_b_w_R_l_sole[c17_i448 + c17_i445] = c17_w_R_l_sole[c17_i447 + c17_i446];
      c17_i447 += 3;
    }

    c17_i445 += 3;
  }

  c17_i449 = 0;
  for (c17_i450 = 0; c17_i450 < 3; c17_i450++) {
    c17_i451 = 0;
    for (c17_i452 = 0; c17_i452 < 3; c17_i452++) {
      c17_c_w_R_l_sole[c17_i452 + c17_i449] = c17_w_R_l_sole[c17_i451 + c17_i450];
      c17_i451 += 3;
    }

    c17_i449 += 3;
  }

  c17_blkdiag(chartInstance, c17_b_w_R_l_sole, c17_c_w_R_l_sole, c17_e_b);
  c17_r_eml_scalar_eg(chartInstance);
  c17_r_eml_scalar_eg(chartInstance);
  for (c17_i453 = 0; c17_i453 < 114; c17_i453++) {
    c17_constraintMatrixLeftFoot[c17_i453] = 0.0;
  }

  for (c17_i454 = 0; c17_i454 < 114; c17_i454++) {
    c17_constraintMatrixLeftFoot[c17_i454] = 0.0;
  }

  for (c17_i455 = 0; c17_i455 < 114; c17_i455++) {
    c17_dv33[c17_i455] = c17_ab_a[c17_i455];
  }

  for (c17_i456 = 0; c17_i456 < 36; c17_i456++) {
    c17_dv34[c17_i456] = c17_e_b[c17_i456];
  }

  for (c17_i457 = 0; c17_i457 < 114; c17_i457++) {
    c17_dv35[c17_i457] = c17_dv33[c17_i457];
  }

  for (c17_i458 = 0; c17_i458 < 36; c17_i458++) {
    c17_dv36[c17_i458] = c17_dv34[c17_i458];
  }

  c17_ab_eml_xgemm(chartInstance, c17_dv35, c17_dv36,
                   c17_constraintMatrixLeftFoot);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 203U);
  for (c17_i459 = 0; c17_i459 < 114; c17_i459++) {
    c17_ab_a[c17_i459] = c17_ConstraintsMatrix[c17_i459];
  }

  c17_i460 = 0;
  for (c17_i461 = 0; c17_i461 < 3; c17_i461++) {
    c17_i462 = 0;
    for (c17_i463 = 0; c17_i463 < 3; c17_i463++) {
      c17_b_w_R_r_sole[c17_i463 + c17_i460] = c17_w_R_r_sole[c17_i462 + c17_i461];
      c17_i462 += 3;
    }

    c17_i460 += 3;
  }

  c17_i464 = 0;
  for (c17_i465 = 0; c17_i465 < 3; c17_i465++) {
    c17_i466 = 0;
    for (c17_i467 = 0; c17_i467 < 3; c17_i467++) {
      c17_c_w_R_r_sole[c17_i467 + c17_i464] = c17_w_R_r_sole[c17_i466 + c17_i465];
      c17_i466 += 3;
    }

    c17_i464 += 3;
  }

  c17_blkdiag(chartInstance, c17_b_w_R_r_sole, c17_c_w_R_r_sole, c17_e_b);
  c17_r_eml_scalar_eg(chartInstance);
  c17_r_eml_scalar_eg(chartInstance);
  for (c17_i468 = 0; c17_i468 < 114; c17_i468++) {
    c17_constraintMatrixRightFoot[c17_i468] = 0.0;
  }

  for (c17_i469 = 0; c17_i469 < 114; c17_i469++) {
    c17_constraintMatrixRightFoot[c17_i469] = 0.0;
  }

  for (c17_i470 = 0; c17_i470 < 114; c17_i470++) {
    c17_dv37[c17_i470] = c17_ab_a[c17_i470];
  }

  for (c17_i471 = 0; c17_i471 < 36; c17_i471++) {
    c17_dv38[c17_i471] = c17_e_b[c17_i471];
  }

  for (c17_i472 = 0; c17_i472 < 114; c17_i472++) {
    c17_dv39[c17_i472] = c17_dv37[c17_i472];
  }

  for (c17_i473 = 0; c17_i473 < 36; c17_i473++) {
    c17_dv40[c17_i473] = c17_dv38[c17_i473];
  }

  c17_ab_eml_xgemm(chartInstance, c17_dv39, c17_dv40,
                   c17_constraintMatrixRightFoot);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 204U);
  for (c17_i474 = 0; c17_i474 < 114; c17_i474++) {
    c17_b_constraintMatrixLeftFoot[c17_i474] =
      c17_constraintMatrixLeftFoot[c17_i474];
  }

  for (c17_i475 = 0; c17_i475 < 114; c17_i475++) {
    c17_b_constraintMatrixRightFoot[c17_i475] =
      c17_constraintMatrixRightFoot[c17_i475];
  }

  c17_b_blkdiag(chartInstance, c17_b_constraintMatrixLeftFoot,
                c17_b_constraintMatrixRightFoot, c17_dv41);
  for (c17_i476 = 0; c17_i476 < 456; c17_i476++) {
    c17_ConstraintsMatrix2FeetOrLegs[c17_i476] = c17_dv41[c17_i476];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 205U);
  for (c17_i477 = 0; c17_i477 < 19; c17_i477++) {
    c17_bVectorConstraints2FeetOrLegs[c17_i477] =
      c17_bVectorConstraints[c17_i477];
  }

  for (c17_i478 = 0; c17_i478 < 19; c17_i478++) {
    c17_bVectorConstraints2FeetOrLegs[c17_i478 + 19] =
      c17_bVectorConstraints[c17_i478];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 208U);
  for (c17_i479 = 0; c17_i479 < 348; c17_i479++) {
    c17_n_a[c17_i479] = c17_JcMinv[c17_i479];
  }

  for (c17_i480 = 0; c17_i480 < 29; c17_i480++) {
    c17_b_b[c17_i480] = c17_h[c17_i480];
  }

  c17_s_eml_scalar_eg(chartInstance);
  c17_s_eml_scalar_eg(chartInstance);
  for (c17_i481 = 0; c17_i481 < 12; c17_i481++) {
    c17_j_b[c17_i481] = 0.0;
    c17_i482 = 0;
    for (c17_i483 = 0; c17_i483 < 29; c17_i483++) {
      c17_j_b[c17_i481] += c17_n_a[c17_i482 + c17_i481] * c17_b_b[c17_i483];
      c17_i482 += 12;
    }
  }

  for (c17_i484 = 0; c17_i484 < 276; c17_i484++) {
    c17_r_a[c17_i484] = c17_PInv_JcMinvSt[c17_i484];
  }

  for (c17_i485 = 0; c17_i485 < 12; c17_i485++) {
    c17_j_b[c17_i485] -= c17_JcDv[c17_i485];
  }

  c17_t_eml_scalar_eg(chartInstance);
  c17_t_eml_scalar_eg(chartInstance);
  for (c17_i486 = 0; c17_i486 < 23; c17_i486++) {
    c17_j_y[c17_i486] = 0.0;
    c17_i487 = 0;
    for (c17_i488 = 0; c17_i488 < 12; c17_i488++) {
      c17_j_y[c17_i486] += c17_r_a[c17_i487 + c17_i486] * c17_j_b[c17_i488];
      c17_i487 += 23;
    }
  }

  c17_i489 = 0;
  for (c17_i490 = 0; c17_i490 < 6; c17_i490++) {
    c17_i491 = 0;
    for (c17_i492 = 0; c17_i492 < 23; c17_i492++) {
      c17_d_Mbj[c17_i492 + c17_i489] = c17_Mbj[c17_i491 + c17_i490];
      c17_i491 += 6;
    }

    c17_i489 += 23;
  }

  for (c17_i493 = 0; c17_i493 < 36; c17_i493++) {
    c17_d_Mb[c17_i493] = c17_Mb[c17_i493];
  }

  c17_b_mrdivide(chartInstance, c17_d_Mbj, c17_d_Mb, c17_o_a);
  for (c17_i494 = 0; c17_i494 < 6; c17_i494++) {
    c17_k_b[c17_i494] = c17_h[c17_i494];
  }

  c17_u_eml_scalar_eg(chartInstance);
  c17_u_eml_scalar_eg(chartInstance);
  for (c17_i495 = 0; c17_i495 < 23; c17_i495++) {
    c17_k_y[c17_i495] = 0.0;
    c17_i496 = 0;
    for (c17_i497 = 0; c17_i497 < 6; c17_i497++) {
      c17_k_y[c17_i495] += c17_o_a[c17_i496 + c17_i495] * c17_k_b[c17_i497];
      c17_i496 += 23;
    }
  }

  for (c17_i498 = 0; c17_i498 < 529; c17_i498++) {
    c17_t_a[c17_i498] = c17_b_impedances[c17_i498];
  }

  for (c17_i499 = 0; c17_i499 < 529; c17_i499++) {
    c17_w_b[c17_i499] = c17_NLMbar[c17_i499];
  }

  c17_q_eml_scalar_eg(chartInstance);
  c17_q_eml_scalar_eg(chartInstance);
  for (c17_i500 = 0; c17_i500 < 529; c17_i500++) {
    c17_i_y[c17_i500] = 0.0;
  }

  for (c17_i501 = 0; c17_i501 < 529; c17_i501++) {
    c17_bb_a[c17_i501] = c17_t_a[c17_i501];
  }

  for (c17_i502 = 0; c17_i502 < 529; c17_i502++) {
    c17_bb_b[c17_i502] = c17_w_b[c17_i502];
  }

  c17_x_eml_xgemm(chartInstance, c17_bb_a, c17_bb_b, c17_i_y);
  for (c17_i503 = 0; c17_i503 < 23; c17_i503++) {
    c17_cb_b[c17_i503] = c17_qTilde[c17_i503];
  }

  c17_v_eml_scalar_eg(chartInstance);
  c17_v_eml_scalar_eg(chartInstance);
  for (c17_i504 = 0; c17_i504 < 23; c17_i504++) {
    c17_l_y[c17_i504] = 0.0;
  }

  for (c17_i505 = 0; c17_i505 < 529; c17_i505++) {
    c17_m_y[c17_i505] = c17_i_y[c17_i505];
  }

  for (c17_i506 = 0; c17_i506 < 23; c17_i506++) {
    c17_db_b[c17_i506] = c17_cb_b[c17_i506];
  }

  c17_bb_eml_xgemm(chartInstance, c17_m_y, c17_db_b, c17_l_y);
  for (c17_i507 = 0; c17_i507 < 529; c17_i507++) {
    c17_t_a[c17_i507] = c17_b_dampings[c17_i507];
  }

  for (c17_i508 = 0; c17_i508 < 529; c17_i508++) {
    c17_w_b[c17_i508] = c17_NLMbar[c17_i508];
  }

  c17_q_eml_scalar_eg(chartInstance);
  c17_q_eml_scalar_eg(chartInstance);
  for (c17_i509 = 0; c17_i509 < 529; c17_i509++) {
    c17_i_y[c17_i509] = 0.0;
  }

  for (c17_i510 = 0; c17_i510 < 529; c17_i510++) {
    c17_cb_a[c17_i510] = c17_t_a[c17_i510];
  }

  for (c17_i511 = 0; c17_i511 < 529; c17_i511++) {
    c17_eb_b[c17_i511] = c17_w_b[c17_i511];
  }

  c17_x_eml_xgemm(chartInstance, c17_cb_a, c17_eb_b, c17_i_y);
  for (c17_i512 = 0; c17_i512 < 23; c17_i512++) {
    c17_cb_b[c17_i512] = c17_qD[c17_i512];
  }

  c17_v_eml_scalar_eg(chartInstance);
  c17_v_eml_scalar_eg(chartInstance);
  for (c17_i513 = 0; c17_i513 < 23; c17_i513++) {
    c17_n_y[c17_i513] = 0.0;
  }

  for (c17_i514 = 0; c17_i514 < 529; c17_i514++) {
    c17_o_y[c17_i514] = c17_i_y[c17_i514];
  }

  for (c17_i515 = 0; c17_i515 < 23; c17_i515++) {
    c17_fb_b[c17_i515] = c17_cb_b[c17_i515];
  }

  c17_bb_eml_xgemm(chartInstance, c17_o_y, c17_fb_b, c17_n_y);
  for (c17_i516 = 0; c17_i516 < 529; c17_i516++) {
    c17_t_a[c17_i516] = c17_nullJcMinvSt[c17_i516];
  }

  for (c17_i517 = 0; c17_i517 < 23; c17_i517++) {
    c17_cb_b[c17_i517] = (((c17_h[c17_i517 + 6] - c17_k_y[c17_i517]) -
      c17_l_y[c17_i517]) - c17_ki_int_qtilde[c17_i517]) - c17_n_y[c17_i517];
  }

  c17_v_eml_scalar_eg(chartInstance);
  c17_v_eml_scalar_eg(chartInstance);
  for (c17_i518 = 0; c17_i518 < 23; c17_i518++) {
    c17_k_y[c17_i518] = 0.0;
  }

  for (c17_i519 = 0; c17_i519 < 529; c17_i519++) {
    c17_db_a[c17_i519] = c17_t_a[c17_i519];
  }

  for (c17_i520 = 0; c17_i520 < 23; c17_i520++) {
    c17_gb_b[c17_i520] = c17_cb_b[c17_i520];
  }

  c17_bb_eml_xgemm(chartInstance, c17_db_a, c17_gb_b, c17_k_y);
  for (c17_i521 = 0; c17_i521 < 23; c17_i521++) {
    c17_tauModel[c17_i521] = c17_j_y[c17_i521] + c17_k_y[c17_i521];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 211U);
  for (c17_i522 = 0; c17_i522 < 276; c17_i522++) {
    c17_r_a[c17_i522] = c17_PInv_JcMinvSt[c17_i522];
  }

  for (c17_i523 = 0; c17_i523 < 144; c17_i523++) {
    c17_k_a[c17_i523] = c17_JcMinvJct[c17_i523];
  }

  c17_w_eml_scalar_eg(chartInstance);
  c17_w_eml_scalar_eg(chartInstance);
  for (c17_i524 = 0; c17_i524 < 276; c17_i524++) {
    c17_h_y[c17_i524] = 0.0;
  }

  for (c17_i525 = 0; c17_i525 < 276; c17_i525++) {
    c17_eb_a[c17_i525] = c17_r_a[c17_i525];
  }

  for (c17_i526 = 0; c17_i526 < 144; c17_i526++) {
    c17_fb_a[c17_i526] = c17_k_a[c17_i526];
  }

  c17_cb_eml_xgemm(chartInstance, c17_eb_a, c17_fb_a, c17_h_y);
  for (c17_i527 = 0; c17_i527 < 529; c17_i527++) {
    c17_t_a[c17_i527] = c17_nullJcMinvSt[c17_i527];
  }

  for (c17_i528 = 0; c17_i528 < 276; c17_i528++) {
    c17_r_a[c17_i528] = c17_JBar[c17_i528];
  }

  c17_x_eml_scalar_eg(chartInstance);
  c17_x_eml_scalar_eg(chartInstance);
  for (c17_i529 = 0; c17_i529 < 276; c17_i529++) {
    c17_p_y[c17_i529] = 0.0;
  }

  for (c17_i530 = 0; c17_i530 < 529; c17_i530++) {
    c17_gb_a[c17_i530] = c17_t_a[c17_i530];
  }

  for (c17_i531 = 0; c17_i531 < 276; c17_i531++) {
    c17_hb_a[c17_i531] = c17_r_a[c17_i531];
  }

  c17_db_eml_xgemm(chartInstance, c17_gb_a, c17_hb_a, c17_p_y);
  for (c17_i532 = 0; c17_i532 < 276; c17_i532++) {
    c17_Sigma[c17_i532] = -(c17_h_y[c17_i532] + c17_p_y[c17_i532]);
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 214U);
  c17_ib_a = c17_m;
  for (c17_i533 = 0; c17_i533 < 3; c17_i533++) {
    c17_C[c17_i533] = c17_xDDcomStar[c17_i533];
  }

  for (c17_i534 = 0; c17_i534 < 3; c17_i534++) {
    c17_C[c17_i534] *= c17_ib_a;
  }

  c17_jb_a = -c17_b_gain->DAngularMomentum;
  for (c17_i535 = 0; c17_i535 < 3; c17_i535++) {
    c17_hb_b[c17_i535] = c17_H[c17_i535 + 3];
  }

  for (c17_i536 = 0; c17_i536 < 3; c17_i536++) {
    c17_hb_b[c17_i536] *= c17_jb_a;
  }

  c17_kb_a = c17_b_gain->PAngularMomentum;
  for (c17_i537 = 0; c17_i537 < 3; c17_i537++) {
    c17_ib_b[c17_i537] = c17_intHw[c17_i537];
  }

  for (c17_i538 = 0; c17_i538 < 3; c17_i538++) {
    c17_ib_b[c17_i538] *= c17_kb_a;
  }

  for (c17_i539 = 0; c17_i539 < 3; c17_i539++) {
    c17_c_C[c17_i539] = c17_C[c17_i539];
  }

  for (c17_i540 = 0; c17_i540 < 3; c17_i540++) {
    c17_c_C[c17_i540 + 3] = c17_hb_b[c17_i540] - c17_ib_b[c17_i540];
  }

  for (c17_i541 = 0; c17_i541 < 6; c17_i541++) {
    c17_HDotDes[c17_i541] = c17_c_C[c17_i541] +
      c17_correctionFromSupportForce[c17_i541];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 223U);
  for (c17_i542 = 0; c17_i542 < 72; c17_i542++) {
    c17_c_a[c17_i542] = c17_pinvA[c17_i542];
  }

  for (c17_i543 = 0; c17_i543 < 6; c17_i543++) {
    c17_k_b[c17_i543] = c17_HDotDes[c17_i543] - c17_gravityWrench[c17_i543];
  }

  c17_y_eml_scalar_eg(chartInstance);
  c17_y_eml_scalar_eg(chartInstance);
  for (c17_i544 = 0; c17_i544 < 12; c17_i544++) {
    c17_j_b[c17_i544] = 0.0;
    c17_i545 = 0;
    for (c17_i546 = 0; c17_i546 < 6; c17_i546++) {
      c17_j_b[c17_i544] += c17_c_a[c17_i545 + c17_i544] * c17_k_b[c17_i546];
      c17_i545 += 12;
    }
  }

  c17_jb_b = c17_constraints[0];
  for (c17_i547 = 0; c17_i547 < 12; c17_i547++) {
    c17_j_b[c17_i547] *= c17_jb_b;
  }

  c17_kb_b = c17_constraints[1];
  for (c17_i548 = 0; c17_i548 < 12; c17_i548++) {
    c17_f_HDot[c17_i548] = c17_j_b[c17_i548] * c17_kb_b;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 224U);
  for (c17_i549 = 0; c17_i549 < 276; c17_i549++) {
    c17_r_a[c17_i549] = c17_Sigma[c17_i549];
  }

  for (c17_i550 = 0; c17_i550 < 144; c17_i550++) {
    c17_k_a[c17_i550] = c17_NA[c17_i550];
  }

  c17_w_eml_scalar_eg(chartInstance);
  c17_w_eml_scalar_eg(chartInstance);
  for (c17_i551 = 0; c17_i551 < 276; c17_i551++) {
    c17_SigmaNA[c17_i551] = 0.0;
  }

  for (c17_i552 = 0; c17_i552 < 276; c17_i552++) {
    c17_SigmaNA[c17_i552] = 0.0;
  }

  for (c17_i553 = 0; c17_i553 < 276; c17_i553++) {
    c17_dv42[c17_i553] = c17_r_a[c17_i553];
  }

  for (c17_i554 = 0; c17_i554 < 144; c17_i554++) {
    c17_dv43[c17_i554] = c17_k_a[c17_i554];
  }

  for (c17_i555 = 0; c17_i555 < 276; c17_i555++) {
    c17_dv44[c17_i555] = c17_dv42[c17_i555];
  }

  for (c17_i556 = 0; c17_i556 < 144; c17_i556++) {
    c17_dv45[c17_i556] = c17_dv43[c17_i556];
  }

  c17_cb_eml_xgemm(chartInstance, c17_dv44, c17_dv45, c17_SigmaNA);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 239U);
  for (c17_i557 = 0; c17_i557 < 456; c17_i557++) {
    c17_lb_a[c17_i557] = c17_ConstraintsMatrix2FeetOrLegs[c17_i557];
  }

  for (c17_i558 = 0; c17_i558 < 144; c17_i558++) {
    c17_k_a[c17_i558] = c17_NA[c17_i558];
  }

  c17_ab_eml_scalar_eg(chartInstance);
  c17_ab_eml_scalar_eg(chartInstance);
  for (c17_i559 = 0; c17_i559 < 456; c17_i559++) {
    c17_ConstraintsMatrixQP2FeetOrLegs[c17_i559] = 0.0;
  }

  for (c17_i560 = 0; c17_i560 < 456; c17_i560++) {
    c17_ConstraintsMatrixQP2FeetOrLegs[c17_i560] = 0.0;
  }

  for (c17_i561 = 0; c17_i561 < 456; c17_i561++) {
    c17_dv46[c17_i561] = c17_lb_a[c17_i561];
  }

  for (c17_i562 = 0; c17_i562 < 144; c17_i562++) {
    c17_dv47[c17_i562] = c17_k_a[c17_i562];
  }

  for (c17_i563 = 0; c17_i563 < 456; c17_i563++) {
    c17_dv48[c17_i563] = c17_dv46[c17_i563];
  }

  for (c17_i564 = 0; c17_i564 < 144; c17_i564++) {
    c17_dv49[c17_i564] = c17_dv47[c17_i564];
  }

  c17_eb_eml_xgemm(chartInstance, c17_dv48, c17_dv49,
                   c17_ConstraintsMatrixQP2FeetOrLegs);
  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 240U);
  for (c17_i565 = 0; c17_i565 < 456; c17_i565++) {
    c17_lb_a[c17_i565] = c17_ConstraintsMatrix2FeetOrLegs[c17_i565];
  }

  for (c17_i566 = 0; c17_i566 < 12; c17_i566++) {
    c17_j_b[c17_i566] = c17_f_HDot[c17_i566];
  }

  c17_bb_eml_scalar_eg(chartInstance);
  c17_bb_eml_scalar_eg(chartInstance);
  for (c17_i567 = 0; c17_i567 < 38; c17_i567++) {
    c17_q_y[c17_i567] = 0.0;
    c17_i568 = 0;
    for (c17_i569 = 0; c17_i569 < 12; c17_i569++) {
      c17_q_y[c17_i567] += c17_lb_a[c17_i568 + c17_i567] * c17_j_b[c17_i569];
      c17_i568 += 38;
    }
  }

  for (c17_i570 = 0; c17_i570 < 38; c17_i570++) {
    c17_bVectorConstraintsQp2FeetOrLegs[c17_i570] =
      c17_bVectorConstraints2FeetOrLegs[c17_i570] - c17_q_y[c17_i570];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 244U);
  c17_i571 = 0;
  for (c17_i572 = 0; c17_i572 < 23; c17_i572++) {
    c17_i573 = 0;
    for (c17_i574 = 0; c17_i574 < 12; c17_i574++) {
      c17_s_b[c17_i574 + c17_i571] = c17_SigmaNA[c17_i573 + c17_i572];
      c17_i573 += 23;
    }

    c17_i571 += 12;
  }

  for (c17_i575 = 0; c17_i575 < 276; c17_i575++) {
    c17_r_a[c17_i575] = c17_SigmaNA[c17_i575];
  }

  c17_m_eml_scalar_eg(chartInstance);
  c17_m_eml_scalar_eg(chartInstance);
  for (c17_i576 = 0; c17_i576 < 144; c17_i576++) {
    c17_g_y[c17_i576] = 0.0;
  }

  for (c17_i577 = 0; c17_i577 < 276; c17_i577++) {
    c17_lb_b[c17_i577] = c17_s_b[c17_i577];
  }

  for (c17_i578 = 0; c17_i578 < 276; c17_i578++) {
    c17_mb_a[c17_i578] = c17_r_a[c17_i578];
  }

  c17_u_eml_xgemm(chartInstance, c17_lb_b, c17_mb_a, c17_g_y);
  c17_mb_b = c17_b_reg.HessianQP;
  for (c17_i579 = 0; c17_i579 < 144; c17_i579++) {
    c17_k_a[c17_i579] = c17_nb_a[c17_i579] * c17_mb_b;
  }

  for (c17_i580 = 0; c17_i580 < 144; c17_i580++) {
    c17_HessianMatrixQP2FeetOrLegs[c17_i580] = c17_g_y[c17_i580] +
      c17_k_a[c17_i580];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 245U);
  for (c17_i581 = 0; c17_i581 < 276; c17_i581++) {
    c17_r_a[c17_i581] = c17_Sigma[c17_i581];
  }

  for (c17_i582 = 0; c17_i582 < 12; c17_i582++) {
    c17_j_b[c17_i582] = c17_f_HDot[c17_i582];
  }

  c17_t_eml_scalar_eg(chartInstance);
  c17_t_eml_scalar_eg(chartInstance);
  for (c17_i583 = 0; c17_i583 < 23; c17_i583++) {
    c17_j_y[c17_i583] = 0.0;
    c17_i584 = 0;
    for (c17_i585 = 0; c17_i585 < 12; c17_i585++) {
      c17_j_y[c17_i583] += c17_r_a[c17_i584 + c17_i583] * c17_j_b[c17_i585];
      c17_i584 += 23;
    }
  }

  c17_i586 = 0;
  for (c17_i587 = 0; c17_i587 < 23; c17_i587++) {
    c17_i588 = 0;
    for (c17_i589 = 0; c17_i589 < 12; c17_i589++) {
      c17_s_b[c17_i589 + c17_i586] = c17_SigmaNA[c17_i588 + c17_i587];
      c17_i588 += 23;
    }

    c17_i586 += 12;
  }

  for (c17_i590 = 0; c17_i590 < 23; c17_i590++) {
    c17_j_y[c17_i590] += c17_tauModel[c17_i590];
  }

  c17_cb_eml_scalar_eg(chartInstance);
  c17_cb_eml_scalar_eg(chartInstance);
  for (c17_i591 = 0; c17_i591 < 12; c17_i591++) {
    c17_gradientQP2FeetOrLegs[c17_i591] = 0.0;
  }

  for (c17_i592 = 0; c17_i592 < 12; c17_i592++) {
    c17_gradientQP2FeetOrLegs[c17_i592] = 0.0;
  }

  for (c17_i593 = 0; c17_i593 < 12; c17_i593++) {
    c17_j_b[c17_i593] = c17_gradientQP2FeetOrLegs[c17_i593];
  }

  for (c17_i594 = 0; c17_i594 < 12; c17_i594++) {
    c17_gradientQP2FeetOrLegs[c17_i594] = c17_j_b[c17_i594];
  }

  for (c17_i595 = 0; c17_i595 < 12; c17_i595++) {
    c17_j_b[c17_i595] = c17_gradientQP2FeetOrLegs[c17_i595];
  }

  for (c17_i596 = 0; c17_i596 < 12; c17_i596++) {
    c17_gradientQP2FeetOrLegs[c17_i596] = c17_j_b[c17_i596];
  }

  for (c17_i597 = 0; c17_i597 < 12; c17_i597++) {
    c17_gradientQP2FeetOrLegs[c17_i597] = 0.0;
    c17_i598 = 0;
    for (c17_i599 = 0; c17_i599 < 23; c17_i599++) {
      c17_gradientQP2FeetOrLegs[c17_i597] += c17_s_b[c17_i598 + c17_i597] *
        c17_j_y[c17_i599];
      c17_i598 += 12;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 260);
  c17_ob_a = c17_constraints[0];
  c17_nb_b = 1.0 - c17_constraints[1];
  c17_r_y = c17_ob_a * c17_nb_b;
  c17_pb_a = c17_r_y;
  for (c17_i600 = 0; c17_i600 < 114; c17_i600++) {
    c17_ab_a[c17_i600] = c17_constraintMatrixLeftFoot[c17_i600];
  }

  for (c17_i601 = 0; c17_i601 < 114; c17_i601++) {
    c17_ab_a[c17_i601] *= c17_pb_a;
  }

  c17_qb_a = c17_constraints[1];
  c17_ob_b = 1.0 - c17_constraints[0];
  c17_s_y = c17_qb_a * c17_ob_b;
  c17_rb_a = c17_s_y;
  for (c17_i602 = 0; c17_i602 < 114; c17_i602++) {
    c17_pb_b[c17_i602] = c17_constraintMatrixRightFoot[c17_i602];
  }

  for (c17_i603 = 0; c17_i603 < 114; c17_i603++) {
    c17_pb_b[c17_i603] *= c17_rb_a;
  }

  for (c17_i604 = 0; c17_i604 < 114; c17_i604++) {
    c17_ConstraintsMatrixQP1Foot[c17_i604] = c17_ab_a[c17_i604] +
      c17_pb_b[c17_i604];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 262);
  for (c17_i605 = 0; c17_i605 < 19; c17_i605++) {
    c17_bVectorConstraintsQp1Foot[c17_i605] = c17_bVectorConstraints[c17_i605];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 264);
  for (c17_i606 = 0; c17_i606 < 36; c17_i606++) {
    c17_sb_a[c17_i606] = c17_AL[c17_i606];
  }

  c17_qb_b = c17_constraints[0];
  for (c17_i607 = 0; c17_i607 < 36; c17_i607++) {
    c17_sb_a[c17_i607] *= c17_qb_b;
  }

  c17_rb_b = 1.0 - c17_constraints[1];
  for (c17_i608 = 0; c17_i608 < 36; c17_i608++) {
    c17_sb_a[c17_i608] *= c17_rb_b;
  }

  for (c17_i609 = 0; c17_i609 < 36; c17_i609++) {
    c17_e_b[c17_i609] = c17_AR[c17_i609];
  }

  c17_sb_b = c17_constraints[1];
  for (c17_i610 = 0; c17_i610 < 36; c17_i610++) {
    c17_e_b[c17_i610] *= c17_sb_b;
  }

  c17_tb_b = 1.0 - c17_constraints[0];
  for (c17_i611 = 0; c17_i611 < 36; c17_i611++) {
    c17_e_b[c17_i611] *= c17_tb_b;
  }

  for (c17_i612 = 0; c17_i612 < 36; c17_i612++) {
    c17_A1Foot[c17_i612] = c17_sb_a[c17_i612] + c17_e_b[c17_i612];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 265);
  c17_i613 = 0;
  for (c17_i614 = 0; c17_i614 < 6; c17_i614++) {
    c17_i615 = 0;
    for (c17_i616 = 0; c17_i616 < 6; c17_i616++) {
      c17_sb_a[c17_i616 + c17_i613] = c17_A1Foot[c17_i615 + c17_i614];
      c17_i615 += 6;
    }

    c17_i613 += 6;
  }

  for (c17_i617 = 0; c17_i617 < 36; c17_i617++) {
    c17_e_b[c17_i617] = c17_A1Foot[c17_i617];
  }

  c17_d_eml_scalar_eg(chartInstance);
  c17_d_eml_scalar_eg(chartInstance);
  for (c17_i618 = 0; c17_i618 < 6; c17_i618++) {
    c17_i619 = 0;
    for (c17_i620 = 0; c17_i620 < 6; c17_i620++) {
      c17_t_y[c17_i619 + c17_i618] = 0.0;
      c17_i621 = 0;
      for (c17_i622 = 0; c17_i622 < 6; c17_i622++) {
        c17_t_y[c17_i619 + c17_i618] += c17_sb_a[c17_i621 + c17_i618] *
          c17_e_b[c17_i622 + c17_i619];
        c17_i621 += 6;
      }

      c17_i619 += 6;
    }
  }

  c17_ub_b = c17_b_reg.HessianQP;
  for (c17_i623 = 0; c17_i623 < 36; c17_i623++) {
    c17_e_b[c17_i623] = c17_tb_a[c17_i623] * c17_ub_b;
  }

  for (c17_i624 = 0; c17_i624 < 36; c17_i624++) {
    c17_HessianMatrixQP1Foot[c17_i624] = c17_t_y[c17_i624] + c17_e_b[c17_i624];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 266);
  c17_i625 = 0;
  for (c17_i626 = 0; c17_i626 < 6; c17_i626++) {
    c17_i627 = 0;
    for (c17_i628 = 0; c17_i628 < 6; c17_i628++) {
      c17_sb_a[c17_i628 + c17_i625] = -c17_A1Foot[c17_i627 + c17_i626];
      c17_i627 += 6;
    }

    c17_i625 += 6;
  }

  for (c17_i629 = 0; c17_i629 < 6; c17_i629++) {
    c17_k_b[c17_i629] = c17_HDotDes[c17_i629] - c17_gravityWrench[c17_i629];
  }

  c17_db_eml_scalar_eg(chartInstance);
  c17_db_eml_scalar_eg(chartInstance);
  for (c17_i630 = 0; c17_i630 < 6; c17_i630++) {
    c17_gradientQP1Foot[c17_i630] = 0.0;
  }

  for (c17_i631 = 0; c17_i631 < 6; c17_i631++) {
    c17_gradientQP1Foot[c17_i631] = 0.0;
  }

  for (c17_i632 = 0; c17_i632 < 6; c17_i632++) {
    c17_b_C[c17_i632] = c17_gradientQP1Foot[c17_i632];
  }

  for (c17_i633 = 0; c17_i633 < 6; c17_i633++) {
    c17_gradientQP1Foot[c17_i633] = c17_b_C[c17_i633];
  }

  for (c17_i634 = 0; c17_i634 < 6; c17_i634++) {
    c17_b_C[c17_i634] = c17_gradientQP1Foot[c17_i634];
  }

  for (c17_i635 = 0; c17_i635 < 6; c17_i635++) {
    c17_gradientQP1Foot[c17_i635] = c17_b_C[c17_i635];
  }

  for (c17_i636 = 0; c17_i636 < 6; c17_i636++) {
    c17_gradientQP1Foot[c17_i636] = 0.0;
    c17_i637 = 0;
    for (c17_i638 = 0; c17_i638 < 6; c17_i638++) {
      c17_gradientQP1Foot[c17_i636] += c17_sb_a[c17_i637 + c17_i636] *
        c17_k_b[c17_i638];
      c17_i637 += 6;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 272);
  for (c17_i639 = 0; c17_i639 < 12; c17_i639++) {
    c17_f[c17_i639] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, 274);
  for (c17_i640 = 0; c17_i640 < 3; c17_i640++) {
    c17_errorCoM[c17_i640] = c17_xcom[c17_i640] -
      c17_desired_x_dx_ddx_CoM[c17_i640];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c17_sfEvent, -274);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c17_Sf(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                   real_T c17_w[3], real_T c17_S[9])
{
  uint32_T c17_debug_family_var_map[4];
  real_T c17_nargin = 1.0;
  real_T c17_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c17_b_debug_family_names,
    c17_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_nargin, 0U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_nargout, 1U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_w, 2U, c17_d_sf_marshallOut,
    c17_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_S, 3U, c17_p_sf_marshallOut,
    c17_o_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c17_sfEvent, 4);
  c17_S[0] = 0.0;
  c17_S[3] = -c17_w[2];
  c17_S[6] = c17_w[1];
  c17_S[1] = c17_w[2];
  c17_S[4] = 0.0;
  c17_S[7] = -c17_w[0];
  c17_S[2] = -c17_w[1];
  c17_S[5] = c17_w[0];
  c17_S[8] = 0.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c17_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c17_pinvDamped(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_regDamp, real_T c17_pinvDampA
  [276])
{
  uint32_T c17_debug_family_var_map[5];
  real_T c17_nargin = 2.0;
  real_T c17_nargout = 1.0;
  int32_T c17_i641;
  real_T c17_a[276];
  int32_T c17_i642;
  int32_T c17_i643;
  int32_T c17_i644;
  int32_T c17_i645;
  real_T c17_b[276];
  int32_T c17_i646;
  real_T c17_y[144];
  int32_T c17_i647;
  real_T c17_b_a[276];
  int32_T c17_i648;
  real_T c17_b_b[276];
  real_T c17_c_a;
  int32_T c17_i649;
  static real_T c17_c_b[144] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c17_b_y[144];
  int32_T c17_i650;
  int32_T c17_i651;
  int32_T c17_i652;
  int32_T c17_i653;
  real_T c17_b_A[276];
  int32_T c17_i654;
  real_T c17_c_y[144];
  real_T c17_dv50[276];
  int32_T c17_i655;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c17_c_debug_family_names,
    c17_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_nargin, 0U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_nargout, 1U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_A, 2U, c17_x_sf_marshallOut,
    c17_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c17_regDamp, 3U, c17_w_sf_marshallOut,
    c17_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c17_pinvDampA, 4U, c17_k_sf_marshallOut,
    c17_k_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c17_sfEvent, 10);
  for (c17_i641 = 0; c17_i641 < 276; c17_i641++) {
    c17_a[c17_i641] = c17_A[c17_i641];
  }

  c17_i642 = 0;
  for (c17_i643 = 0; c17_i643 < 12; c17_i643++) {
    c17_i644 = 0;
    for (c17_i645 = 0; c17_i645 < 23; c17_i645++) {
      c17_b[c17_i645 + c17_i642] = c17_A[c17_i644 + c17_i643];
      c17_i644 += 12;
    }

    c17_i642 += 23;
  }

  c17_m_eml_scalar_eg(chartInstance);
  c17_m_eml_scalar_eg(chartInstance);
  for (c17_i646 = 0; c17_i646 < 144; c17_i646++) {
    c17_y[c17_i646] = 0.0;
  }

  for (c17_i647 = 0; c17_i647 < 276; c17_i647++) {
    c17_b_a[c17_i647] = c17_a[c17_i647];
  }

  for (c17_i648 = 0; c17_i648 < 276; c17_i648++) {
    c17_b_b[c17_i648] = c17_b[c17_i648];
  }

  c17_u_eml_xgemm(chartInstance, c17_b_a, c17_b_b, c17_y);
  c17_c_a = c17_regDamp;
  for (c17_i649 = 0; c17_i649 < 144; c17_i649++) {
    c17_b_y[c17_i649] = c17_c_a * c17_c_b[c17_i649];
  }

  c17_i650 = 0;
  for (c17_i651 = 0; c17_i651 < 12; c17_i651++) {
    c17_i652 = 0;
    for (c17_i653 = 0; c17_i653 < 23; c17_i653++) {
      c17_b_A[c17_i653 + c17_i650] = c17_A[c17_i652 + c17_i651];
      c17_i652 += 12;
    }

    c17_i650 += 23;
  }

  for (c17_i654 = 0; c17_i654 < 144; c17_i654++) {
    c17_c_y[c17_i654] = c17_y[c17_i654] + c17_b_y[c17_i654];
  }

  c17_c_mrdivide(chartInstance, c17_b_A, c17_c_y, c17_dv50);
  for (c17_i655 = 0; c17_i655 < 276; c17_i655++) {
    c17_pinvDampA[c17_i655] = c17_dv50[c17_i655];
  }

  _SFD_SCRIPT_CALL(2U, chartInstance->c17_sfEvent, -10);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c17_machineNumber, uint32_T
  c17_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c17_chartNumber, 0U, sf_debug_get_script_id(
    "/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m"));
  _SFD_SCRIPT_TRANSLATION(c17_chartNumber, 1U, sf_debug_get_script_id(
    "/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/Sf.m"));
  _SFD_SCRIPT_TRANSLATION(c17_chartNumber, 2U, sf_debug_get_script_id(
    "/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m"));
}

static void c17_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_sprintf, const char_T *c17_identifier,
  char_T c17_y[14])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_sprintf), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_sprintf);
}

static void c17_b_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  char_T c17_y[14])
{
  char_T c17_cv0[14];
  int32_T c17_i656;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_cv0, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c17_i656 = 0; c17_i656 < 14; c17_i656++) {
    c17_y[c17_i656] = c17_cv0[c17_i656];
  }

  sf_mex_destroy(&c17_u);
}

static const mxArray *c17_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i657;
  real_T c17_b_inData[6];
  int32_T c17_i658;
  real_T c17_u[6];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i657 = 0; c17_i657 < 6; c17_i657++) {
    c17_b_inData[c17_i657] = (*(real_T (*)[6])c17_inData)[c17_i657];
  }

  for (c17_i658 = 0; c17_i658 < 6; c17_i658++) {
    c17_u[c17_i658] = c17_b_inData[c17_i658];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_c_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_correctionFromSupportForce, const char_T
  *c17_identifier, real_T c17_y[6])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_correctionFromSupportForce), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_correctionFromSupportForce);
}

static void c17_d_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[6])
{
  real_T c17_dv51[6];
  int32_T c17_i659;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv51, 1, 0, 0U, 1, 0U, 1, 6);
  for (c17_i659 = 0; c17_i659 < 6; c17_i659++) {
    c17_y[c17_i659] = c17_dv51[c17_i659];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_correctionFromSupportForce;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[6];
  int32_T c17_i660;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_correctionFromSupportForce = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_correctionFromSupportForce), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_correctionFromSupportForce);
  for (c17_i660 = 0; c17_i660 < 6; c17_i660++) {
    (*(real_T (*)[6])c17_outData)[c17_i660] = c17_y[c17_i660];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_b_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i661;
  real_T c17_b_inData[12];
  int32_T c17_i662;
  real_T c17_u[12];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i661 = 0; c17_i661 < 12; c17_i661++) {
    c17_b_inData[c17_i661] = (*(real_T (*)[12])c17_inData)[c17_i661];
  }

  for (c17_i662 = 0; c17_i662 < 12; c17_i662++) {
    c17_u[c17_i662] = c17_b_inData[c17_i662];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_e_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_f, const char_T *c17_identifier, real_T
  c17_y[12])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_f), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_f);
}

static void c17_f_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[12])
{
  real_T c17_dv52[12];
  int32_T c17_i663;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv52, 1, 0, 0U, 1, 0U, 1,
                12);
  for (c17_i663 = 0; c17_i663 < 12; c17_i663++) {
    c17_y[c17_i663] = c17_dv52[c17_i663];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_f;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[12];
  int32_T c17_i664;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_f = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_f), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_f);
  for (c17_i664 = 0; c17_i664 < 12; c17_i664++) {
    (*(real_T (*)[12])c17_outData)[c17_i664] = c17_y[c17_i664];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_c_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i665;
  real_T c17_b_inData[23];
  int32_T c17_i666;
  real_T c17_u[23];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i665 = 0; c17_i665 < 23; c17_i665++) {
    c17_b_inData[c17_i665] = (*(real_T (*)[23])c17_inData)[c17_i665];
  }

  for (c17_i666 = 0; c17_i666 < 23; c17_i666++) {
    c17_u[c17_i666] = c17_b_inData[c17_i666];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 1, 23), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_g_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_qTilde, const char_T *c17_identifier,
  real_T c17_y[23])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_qTilde), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_qTilde);
}

static void c17_h_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[23])
{
  real_T c17_dv53[23];
  int32_T c17_i667;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv53, 1, 0, 0U, 1, 0U, 1,
                23);
  for (c17_i667 = 0; c17_i667 < 23; c17_i667++) {
    c17_y[c17_i667] = c17_dv53[c17_i667];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_qTilde;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[23];
  int32_T c17_i668;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_qTilde = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_qTilde), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_qTilde);
  for (c17_i668 = 0; c17_i668 < 23; c17_i668++) {
    (*(real_T (*)[23])c17_outData)[c17_i668] = c17_y[c17_i668];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_d_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i669;
  real_T c17_b_inData[3];
  int32_T c17_i670;
  real_T c17_u[3];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i669 = 0; c17_i669 < 3; c17_i669++) {
    c17_b_inData[c17_i669] = (*(real_T (*)[3])c17_inData)[c17_i669];
  }

  for (c17_i670 = 0; c17_i670 < 3; c17_i670++) {
    c17_u[c17_i670] = c17_b_inData[c17_i670];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_i_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_errorCoM, const char_T *c17_identifier,
  real_T c17_y[3])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_errorCoM), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_errorCoM);
}

static void c17_j_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[3])
{
  real_T c17_dv54[3];
  int32_T c17_i671;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv54, 1, 0, 0U, 1, 0U, 1, 3);
  for (c17_i671 = 0; c17_i671 < 3; c17_i671++) {
    c17_y[c17_i671] = c17_dv54[c17_i671];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_errorCoM;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[3];
  int32_T c17_i672;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_errorCoM = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_errorCoM), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_errorCoM);
  for (c17_i672 = 0; c17_i672 < 3; c17_i672++) {
    (*(real_T (*)[3])c17_outData)[c17_i672] = c17_y[c17_i672];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_e_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i673;
  real_T c17_b_inData[38];
  int32_T c17_i674;
  real_T c17_u[38];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i673 = 0; c17_i673 < 38; c17_i673++) {
    c17_b_inData[c17_i673] = (*(real_T (*)[38])c17_inData)[c17_i673];
  }

  for (c17_i674 = 0; c17_i674 < 38; c17_i674++) {
    c17_u[c17_i674] = c17_b_inData[c17_i674];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 1, 38), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_k_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_bVectorConstraintsQp2FeetOrLegs, const
  char_T *c17_identifier, real_T c17_y[38])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_l_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_bVectorConstraintsQp2FeetOrLegs), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_bVectorConstraintsQp2FeetOrLegs);
}

static void c17_l_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[38])
{
  real_T c17_dv55[38];
  int32_T c17_i675;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv55, 1, 0, 0U, 1, 0U, 1,
                38);
  for (c17_i675 = 0; c17_i675 < 38; c17_i675++) {
    c17_y[c17_i675] = c17_dv55[c17_i675];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_bVectorConstraintsQp2FeetOrLegs;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[38];
  int32_T c17_i676;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_bVectorConstraintsQp2FeetOrLegs = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_l_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_bVectorConstraintsQp2FeetOrLegs), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_bVectorConstraintsQp2FeetOrLegs);
  for (c17_i676 = 0; c17_i676 < 38; c17_i676++) {
    (*(real_T (*)[38])c17_outData)[c17_i676] = c17_y[c17_i676];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_f_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i677;
  int32_T c17_i678;
  int32_T c17_i679;
  real_T c17_b_inData[456];
  int32_T c17_i680;
  int32_T c17_i681;
  int32_T c17_i682;
  real_T c17_u[456];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i677 = 0;
  for (c17_i678 = 0; c17_i678 < 12; c17_i678++) {
    for (c17_i679 = 0; c17_i679 < 38; c17_i679++) {
      c17_b_inData[c17_i679 + c17_i677] = (*(real_T (*)[456])c17_inData)
        [c17_i679 + c17_i677];
    }

    c17_i677 += 38;
  }

  c17_i680 = 0;
  for (c17_i681 = 0; c17_i681 < 12; c17_i681++) {
    for (c17_i682 = 0; c17_i682 < 38; c17_i682++) {
      c17_u[c17_i682 + c17_i680] = c17_b_inData[c17_i682 + c17_i680];
    }

    c17_i680 += 38;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 38, 12),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_m_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_ConstraintsMatrixQP2FeetOrLegs, const
  char_T *c17_identifier, real_T c17_y[456])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_ConstraintsMatrixQP2FeetOrLegs), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_ConstraintsMatrixQP2FeetOrLegs);
}

static void c17_n_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[456])
{
  real_T c17_dv56[456];
  int32_T c17_i683;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv56, 1, 0, 0U, 1, 0U, 2,
                38, 12);
  for (c17_i683 = 0; c17_i683 < 456; c17_i683++) {
    c17_y[c17_i683] = c17_dv56[c17_i683];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_ConstraintsMatrixQP2FeetOrLegs;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[456];
  int32_T c17_i684;
  int32_T c17_i685;
  int32_T c17_i686;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_ConstraintsMatrixQP2FeetOrLegs = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_ConstraintsMatrixQP2FeetOrLegs), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_ConstraintsMatrixQP2FeetOrLegs);
  c17_i684 = 0;
  for (c17_i685 = 0; c17_i685 < 12; c17_i685++) {
    for (c17_i686 = 0; c17_i686 < 38; c17_i686++) {
      (*(real_T (*)[456])c17_outData)[c17_i686 + c17_i684] = c17_y[c17_i686 +
        c17_i684];
    }

    c17_i684 += 38;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_g_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i687;
  int32_T c17_i688;
  int32_T c17_i689;
  real_T c17_b_inData[144];
  int32_T c17_i690;
  int32_T c17_i691;
  int32_T c17_i692;
  real_T c17_u[144];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i687 = 0;
  for (c17_i688 = 0; c17_i688 < 12; c17_i688++) {
    for (c17_i689 = 0; c17_i689 < 12; c17_i689++) {
      c17_b_inData[c17_i689 + c17_i687] = (*(real_T (*)[144])c17_inData)
        [c17_i689 + c17_i687];
    }

    c17_i687 += 12;
  }

  c17_i690 = 0;
  for (c17_i691 = 0; c17_i691 < 12; c17_i691++) {
    for (c17_i692 = 0; c17_i692 < 12; c17_i692++) {
      c17_u[c17_i692 + c17_i690] = c17_b_inData[c17_i692 + c17_i690];
    }

    c17_i690 += 12;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 12, 12),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_o_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_HessianMatrixQP2FeetOrLegs, const char_T
  *c17_identifier, real_T c17_y[144])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_HessianMatrixQP2FeetOrLegs), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_HessianMatrixQP2FeetOrLegs);
}

static void c17_p_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[144])
{
  real_T c17_dv57[144];
  int32_T c17_i693;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv57, 1, 0, 0U, 1, 0U, 2,
                12, 12);
  for (c17_i693 = 0; c17_i693 < 144; c17_i693++) {
    c17_y[c17_i693] = c17_dv57[c17_i693];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_HessianMatrixQP2FeetOrLegs;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[144];
  int32_T c17_i694;
  int32_T c17_i695;
  int32_T c17_i696;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_HessianMatrixQP2FeetOrLegs = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_HessianMatrixQP2FeetOrLegs), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_HessianMatrixQP2FeetOrLegs);
  c17_i694 = 0;
  for (c17_i695 = 0; c17_i695 < 12; c17_i695++) {
    for (c17_i696 = 0; c17_i696 < 12; c17_i696++) {
      (*(real_T (*)[144])c17_outData)[c17_i696 + c17_i694] = c17_y[c17_i696 +
        c17_i694];
    }

    c17_i694 += 12;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_h_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i697;
  real_T c17_b_inData[19];
  int32_T c17_i698;
  real_T c17_u[19];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i697 = 0; c17_i697 < 19; c17_i697++) {
    c17_b_inData[c17_i697] = (*(real_T (*)[19])c17_inData)[c17_i697];
  }

  for (c17_i698 = 0; c17_i698 < 19; c17_i698++) {
    c17_u[c17_i698] = c17_b_inData[c17_i698];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 1, 19), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_q_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_bVectorConstraintsQp1Foot, const char_T
  *c17_identifier, real_T c17_y[19])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_bVectorConstraintsQp1Foot),
    &c17_thisId, c17_y);
  sf_mex_destroy(&c17_bVectorConstraintsQp1Foot);
}

static void c17_r_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[19])
{
  real_T c17_dv58[19];
  int32_T c17_i699;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv58, 1, 0, 0U, 1, 0U, 1,
                19);
  for (c17_i699 = 0; c17_i699 < 19; c17_i699++) {
    c17_y[c17_i699] = c17_dv58[c17_i699];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_bVectorConstraintsQp1Foot;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[19];
  int32_T c17_i700;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_bVectorConstraintsQp1Foot = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_bVectorConstraintsQp1Foot),
    &c17_thisId, c17_y);
  sf_mex_destroy(&c17_bVectorConstraintsQp1Foot);
  for (c17_i700 = 0; c17_i700 < 19; c17_i700++) {
    (*(real_T (*)[19])c17_outData)[c17_i700] = c17_y[c17_i700];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_i_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i701;
  int32_T c17_i702;
  int32_T c17_i703;
  real_T c17_b_inData[114];
  int32_T c17_i704;
  int32_T c17_i705;
  int32_T c17_i706;
  real_T c17_u[114];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i701 = 0;
  for (c17_i702 = 0; c17_i702 < 6; c17_i702++) {
    for (c17_i703 = 0; c17_i703 < 19; c17_i703++) {
      c17_b_inData[c17_i703 + c17_i701] = (*(real_T (*)[114])c17_inData)
        [c17_i703 + c17_i701];
    }

    c17_i701 += 19;
  }

  c17_i704 = 0;
  for (c17_i705 = 0; c17_i705 < 6; c17_i705++) {
    for (c17_i706 = 0; c17_i706 < 19; c17_i706++) {
      c17_u[c17_i706 + c17_i704] = c17_b_inData[c17_i706 + c17_i704];
    }

    c17_i704 += 19;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 19, 6),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_s_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_ConstraintsMatrixQP1Foot, const char_T
  *c17_identifier, real_T c17_y[114])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_ConstraintsMatrixQP1Foot),
    &c17_thisId, c17_y);
  sf_mex_destroy(&c17_ConstraintsMatrixQP1Foot);
}

static void c17_t_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[114])
{
  real_T c17_dv59[114];
  int32_T c17_i707;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv59, 1, 0, 0U, 1, 0U, 2,
                19, 6);
  for (c17_i707 = 0; c17_i707 < 114; c17_i707++) {
    c17_y[c17_i707] = c17_dv59[c17_i707];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_ConstraintsMatrixQP1Foot;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[114];
  int32_T c17_i708;
  int32_T c17_i709;
  int32_T c17_i710;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_ConstraintsMatrixQP1Foot = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_ConstraintsMatrixQP1Foot),
    &c17_thisId, c17_y);
  sf_mex_destroy(&c17_ConstraintsMatrixQP1Foot);
  c17_i708 = 0;
  for (c17_i709 = 0; c17_i709 < 6; c17_i709++) {
    for (c17_i710 = 0; c17_i710 < 19; c17_i710++) {
      (*(real_T (*)[114])c17_outData)[c17_i710 + c17_i708] = c17_y[c17_i710 +
        c17_i708];
    }

    c17_i708 += 19;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_j_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i711;
  int32_T c17_i712;
  int32_T c17_i713;
  real_T c17_b_inData[36];
  int32_T c17_i714;
  int32_T c17_i715;
  int32_T c17_i716;
  real_T c17_u[36];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i711 = 0;
  for (c17_i712 = 0; c17_i712 < 6; c17_i712++) {
    for (c17_i713 = 0; c17_i713 < 6; c17_i713++) {
      c17_b_inData[c17_i713 + c17_i711] = (*(real_T (*)[36])c17_inData)[c17_i713
        + c17_i711];
    }

    c17_i711 += 6;
  }

  c17_i714 = 0;
  for (c17_i715 = 0; c17_i715 < 6; c17_i715++) {
    for (c17_i716 = 0; c17_i716 < 6; c17_i716++) {
      c17_u[c17_i716 + c17_i714] = c17_b_inData[c17_i716 + c17_i714];
    }

    c17_i714 += 6;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 6, 6), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_u_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_HessianMatrixQP1Foot, const char_T
  *c17_identifier, real_T c17_y[36])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_HessianMatrixQP1Foot),
    &c17_thisId, c17_y);
  sf_mex_destroy(&c17_HessianMatrixQP1Foot);
}

static void c17_v_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[36])
{
  real_T c17_dv60[36];
  int32_T c17_i717;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv60, 1, 0, 0U, 1, 0U, 2, 6,
                6);
  for (c17_i717 = 0; c17_i717 < 36; c17_i717++) {
    c17_y[c17_i717] = c17_dv60[c17_i717];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_HessianMatrixQP1Foot;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[36];
  int32_T c17_i718;
  int32_T c17_i719;
  int32_T c17_i720;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_HessianMatrixQP1Foot = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_HessianMatrixQP1Foot),
    &c17_thisId, c17_y);
  sf_mex_destroy(&c17_HessianMatrixQP1Foot);
  c17_i718 = 0;
  for (c17_i719 = 0; c17_i719 < 6; c17_i719++) {
    for (c17_i720 = 0; c17_i720 < 6; c17_i720++) {
      (*(real_T (*)[36])c17_outData)[c17_i720 + c17_i718] = c17_y[c17_i720 +
        c17_i718];
    }

    c17_i718 += 6;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_k_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i721;
  int32_T c17_i722;
  int32_T c17_i723;
  real_T c17_b_inData[276];
  int32_T c17_i724;
  int32_T c17_i725;
  int32_T c17_i726;
  real_T c17_u[276];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i721 = 0;
  for (c17_i722 = 0; c17_i722 < 12; c17_i722++) {
    for (c17_i723 = 0; c17_i723 < 23; c17_i723++) {
      c17_b_inData[c17_i723 + c17_i721] = (*(real_T (*)[276])c17_inData)
        [c17_i723 + c17_i721];
    }

    c17_i721 += 23;
  }

  c17_i724 = 0;
  for (c17_i725 = 0; c17_i725 < 12; c17_i725++) {
    for (c17_i726 = 0; c17_i726 < 23; c17_i726++) {
      c17_u[c17_i726 + c17_i724] = c17_b_inData[c17_i726 + c17_i724];
    }

    c17_i724 += 23;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 23, 12),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_w_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_Sigma, const char_T *c17_identifier, real_T
  c17_y[276])
{
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_Sigma), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_Sigma);
}

static void c17_x_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[276])
{
  real_T c17_dv61[276];
  int32_T c17_i727;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv61, 1, 0, 0U, 1, 0U, 2,
                23, 12);
  for (c17_i727 = 0; c17_i727 < 276; c17_i727++) {
    c17_y[c17_i727] = c17_dv61[c17_i727];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_Sigma;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[276];
  int32_T c17_i728;
  int32_T c17_i729;
  int32_T c17_i730;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_Sigma = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_Sigma), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_Sigma);
  c17_i728 = 0;
  for (c17_i729 = 0; c17_i729 < 12; c17_i729++) {
    for (c17_i730 = 0; c17_i730 < 23; c17_i730++) {
      (*(real_T (*)[276])c17_outData)[c17_i730 + c17_i728] = c17_y[c17_i730 +
        c17_i728];
    }

    c17_i728 += 23;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_l_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  boolean_T c17_u;
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_u = *(boolean_T *)c17_inData;
  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", &c17_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_m_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i731;
  int32_T c17_i732;
  int32_T c17_i733;
  real_T c17_b_inData[16];
  int32_T c17_i734;
  int32_T c17_i735;
  int32_T c17_i736;
  real_T c17_u[16];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i731 = 0;
  for (c17_i732 = 0; c17_i732 < 4; c17_i732++) {
    for (c17_i733 = 0; c17_i733 < 4; c17_i733++) {
      c17_b_inData[c17_i733 + c17_i731] = (*(real_T (*)[16])c17_inData)[c17_i733
        + c17_i731];
    }

    c17_i731 += 4;
  }

  c17_i734 = 0;
  for (c17_i735 = 0; c17_i735 < 4; c17_i735++) {
    for (c17_i736 = 0; c17_i736 < 4; c17_i736++) {
      c17_u[c17_i736 + c17_i734] = c17_b_inData[c17_i736 + c17_i734];
    }

    c17_i734 += 4;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_n_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  c17_struct_kzTB0QQWoOlMoMhgKf6sK c17_u;
  const mxArray *c17_y = NULL;
  real_T c17_b_u;
  const mxArray *c17_b_y = NULL;
  real_T c17_c_u;
  const mxArray *c17_c_y = NULL;
  real_T c17_d_u;
  const mxArray *c17_d_y = NULL;
  int32_T c17_i737;
  real_T c17_e_u[9];
  const mxArray *c17_e_y = NULL;
  int32_T c17_i738;
  real_T c17_f_u[9];
  const mxArray *c17_f_y = NULL;
  int32_T c17_i739;
  real_T c17_g_u[9];
  const mxArray *c17_g_y = NULL;
  real_T c17_h_u;
  const mxArray *c17_h_y = NULL;
  real_T c17_i_u;
  const mxArray *c17_i_y = NULL;
  int32_T c17_i740;
  real_T c17_j_u[23];
  const mxArray *c17_j_y = NULL;
  int32_T c17_i741;
  real_T c17_k_u[23];
  const mxArray *c17_k_y = NULL;
  int32_T c17_i742;
  real_T c17_l_u[23];
  const mxArray *c17_l_y = NULL;
  int32_T c17_i743;
  real_T c17_m_u[23];
  const mxArray *c17_m_y = NULL;
  int32_T c17_i744;
  real_T c17_n_u[4];
  const mxArray *c17_n_y = NULL;
  int32_T c17_i745;
  real_T c17_o_u[4];
  const mxArray *c17_o_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_u = *(c17_struct_kzTB0QQWoOlMoMhgKf6sK *)c17_inData;
  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c17_b_u = c17_u.qTildeMax;
  c17_b_y = NULL;
  sf_mex_assign(&c17_b_y, sf_mex_create("y", &c17_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_b_y, "qTildeMax", "qTildeMax", 0);
  c17_c_u = c17_u.SmoothingTimeImp;
  c17_c_y = NULL;
  sf_mex_assign(&c17_c_y, sf_mex_create("y", &c17_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_c_y, "SmoothingTimeImp", "SmoothingTimeImp", 0);
  c17_d_u = c17_u.SmoothingTimeGainScheduling;
  c17_d_y = NULL;
  sf_mex_assign(&c17_d_y, sf_mex_create("y", &c17_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_d_y, "SmoothingTimeGainScheduling",
                  "SmoothingTimeGainScheduling", 0);
  for (c17_i737 = 0; c17_i737 < 9; c17_i737++) {
    c17_e_u[c17_i737] = c17_u.PCOM[c17_i737];
  }

  c17_e_y = NULL;
  sf_mex_assign(&c17_e_y, sf_mex_create("y", c17_e_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c17_y, c17_e_y, "PCOM", "PCOM", 0);
  for (c17_i738 = 0; c17_i738 < 9; c17_i738++) {
    c17_f_u[c17_i738] = c17_u.ICOM[c17_i738];
  }

  c17_f_y = NULL;
  sf_mex_assign(&c17_f_y, sf_mex_create("y", c17_f_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c17_y, c17_f_y, "ICOM", "ICOM", 0);
  for (c17_i739 = 0; c17_i739 < 9; c17_i739++) {
    c17_g_u[c17_i739] = c17_u.DCOM[c17_i739];
  }

  c17_g_y = NULL;
  sf_mex_assign(&c17_g_y, sf_mex_create("y", c17_g_u, 0, 0U, 1U, 0U, 2, 3, 3),
                FALSE);
  sf_mex_addfield(c17_y, c17_g_y, "DCOM", "DCOM", 0);
  c17_h_u = c17_u.PAngularMomentum;
  c17_h_y = NULL;
  sf_mex_assign(&c17_h_y, sf_mex_create("y", &c17_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_h_y, "PAngularMomentum", "PAngularMomentum", 0);
  c17_i_u = c17_u.DAngularMomentum;
  c17_i_y = NULL;
  sf_mex_assign(&c17_i_y, sf_mex_create("y", &c17_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_i_y, "DAngularMomentum", "DAngularMomentum", 0);
  for (c17_i740 = 0; c17_i740 < 23; c17_i740++) {
    c17_j_u[c17_i740] = c17_u.integral[c17_i740];
  }

  c17_j_y = NULL;
  sf_mex_assign(&c17_j_y, sf_mex_create("y", c17_j_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c17_y, c17_j_y, "integral", "integral", 0);
  for (c17_i741 = 0; c17_i741 < 23; c17_i741++) {
    c17_k_u[c17_i741] = c17_u.impedances[c17_i741];
  }

  c17_k_y = NULL;
  sf_mex_assign(&c17_k_y, sf_mex_create("y", c17_k_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c17_y, c17_k_y, "impedances", "impedances", 0);
  for (c17_i742 = 0; c17_i742 < 23; c17_i742++) {
    c17_l_u[c17_i742] = c17_u.dampings[c17_i742];
  }

  c17_l_y = NULL;
  sf_mex_assign(&c17_l_y, sf_mex_create("y", c17_l_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c17_y, c17_l_y, "dampings", "dampings", 0);
  for (c17_i743 = 0; c17_i743 < 23; c17_i743++) {
    c17_m_u[c17_i743] = c17_u.increasingRatesImp[c17_i743];
  }

  c17_m_y = NULL;
  sf_mex_assign(&c17_m_y, sf_mex_create("y", c17_m_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_addfield(c17_y, c17_m_y, "increasingRatesImp", "increasingRatesImp", 0);
  for (c17_i744 = 0; c17_i744 < 4; c17_i744++) {
    c17_n_u[c17_i744] = c17_u.footSize[c17_i744];
  }

  c17_n_y = NULL;
  sf_mex_assign(&c17_n_y, sf_mex_create("y", c17_n_u, 0, 0U, 1U, 0U, 2, 2, 2),
                FALSE);
  sf_mex_addfield(c17_y, c17_n_y, "footSize", "footSize", 0);
  for (c17_i745 = 0; c17_i745 < 4; c17_i745++) {
    c17_o_u[c17_i745] = c17_u.legSize[c17_i745];
  }

  c17_o_y = NULL;
  sf_mex_assign(&c17_o_y, sf_mex_create("y", c17_o_u, 0, 0U, 1U, 0U, 2, 2, 2),
                FALSE);
  sf_mex_addfield(c17_y, c17_o_y, "legSize", "legSize", 0);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_y_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  c17_struct_kzTB0QQWoOlMoMhgKf6sK *c17_y)
{
  emlrtMsgIdentifier c17_thisId;
  static const char * c17_fieldNames[14] = { "qTildeMax", "SmoothingTimeImp",
    "SmoothingTimeGainScheduling", "PCOM", "ICOM", "DCOM", "PAngularMomentum",
    "DAngularMomentum", "integral", "impedances", "dampings",
    "increasingRatesImp", "footSize", "legSize" };

  c17_thisId.fParent = c17_parentId;
  sf_mex_check_struct(c17_parentId, c17_u, 14, c17_fieldNames, 0U, 0);
  c17_thisId.fIdentifier = "qTildeMax";
  c17_y->qTildeMax = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "qTildeMax", "qTildeMax", 0)), &c17_thisId);
  c17_thisId.fIdentifier = "SmoothingTimeImp";
  c17_y->SmoothingTimeImp = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "SmoothingTimeImp", "SmoothingTimeImp", 0)),
    &c17_thisId);
  c17_thisId.fIdentifier = "SmoothingTimeGainScheduling";
  c17_y->SmoothingTimeGainScheduling = c17_ab_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c17_u, "SmoothingTimeGainScheduling",
    "SmoothingTimeGainScheduling", 0)), &c17_thisId);
  c17_thisId.fIdentifier = "PCOM";
  c17_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "PCOM", "PCOM", 0)), &c17_thisId, c17_y->PCOM);
  c17_thisId.fIdentifier = "ICOM";
  c17_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "ICOM", "ICOM", 0)), &c17_thisId, c17_y->ICOM);
  c17_thisId.fIdentifier = "DCOM";
  c17_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "DCOM", "DCOM", 0)), &c17_thisId, c17_y->DCOM);
  c17_thisId.fIdentifier = "PAngularMomentum";
  c17_y->PAngularMomentum = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "PAngularMomentum", "PAngularMomentum", 0)),
    &c17_thisId);
  c17_thisId.fIdentifier = "DAngularMomentum";
  c17_y->DAngularMomentum = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "DAngularMomentum", "DAngularMomentum", 0)),
    &c17_thisId);
  c17_thisId.fIdentifier = "integral";
  c17_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "integral", "integral", 0)), &c17_thisId, c17_y->integral);
  c17_thisId.fIdentifier = "impedances";
  c17_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "impedances", "impedances", 0)), &c17_thisId, c17_y->impedances);
  c17_thisId.fIdentifier = "dampings";
  c17_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "dampings", "dampings", 0)), &c17_thisId, c17_y->dampings);
  c17_thisId.fIdentifier = "increasingRatesImp";
  c17_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "increasingRatesImp", "increasingRatesImp", 0)), &c17_thisId,
    c17_y->increasingRatesImp);
  c17_thisId.fIdentifier = "footSize";
  c17_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "footSize", "footSize", 0)), &c17_thisId, c17_y->footSize);
  c17_thisId.fIdentifier = "legSize";
  c17_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c17_u,
    "legSize", "legSize", 0)), &c17_thisId, c17_y->legSize);
  sf_mex_destroy(&c17_u);
}

static real_T c17_ab_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId)
{
  real_T c17_y;
  real_T c17_d0;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), &c17_d0, 1, 0, 0U, 0, 0U, 0);
  c17_y = c17_d0;
  sf_mex_destroy(&c17_u);
  return c17_y;
}

static void c17_bb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[9])
{
  real_T c17_dv62[9];
  int32_T c17_i746;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv62, 1, 0, 0U, 1, 0U, 2, 3,
                3);
  for (c17_i746 = 0; c17_i746 < 9; c17_i746++) {
    c17_y[c17_i746] = c17_dv62[c17_i746];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_cb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[23])
{
  real_T c17_dv63[23];
  int32_T c17_i747;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv63, 1, 0, 0U, 1, 0U, 2, 1,
                23);
  for (c17_i747 = 0; c17_i747 < 23; c17_i747++) {
    c17_y[c17_i747] = c17_dv63[c17_i747];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_db_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[4])
{
  real_T c17_dv64[4];
  int32_T c17_i748;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv64, 1, 0, 0U, 1, 0U, 2, 2,
                2);
  for (c17_i748 = 0; c17_i748 < 4; c17_i748++) {
    c17_y[c17_i748] = c17_dv64[c17_i748];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_b_gain;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  c17_struct_kzTB0QQWoOlMoMhgKf6sK c17_y;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_b_gain = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_y_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_b_gain), &c17_thisId,
    &c17_y);
  sf_mex_destroy(&c17_b_gain);
  *(c17_struct_kzTB0QQWoOlMoMhgKf6sK *)c17_outData = c17_y;
  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_o_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_u;
  const mxArray *c17_y = NULL;
  real_T c17_b_u;
  const mxArray *c17_b_y = NULL;
  real_T c17_c_u;
  const mxArray *c17_c_y = NULL;
  real_T c17_d_u;
  const mxArray *c17_d_y = NULL;
  real_T c17_e_u;
  const mxArray *c17_e_y = NULL;
  real_T c17_f_u;
  const mxArray *c17_f_y = NULL;
  real_T c17_g_u;
  const mxArray *c17_g_y = NULL;
  real_T c17_h_u;
  const mxArray *c17_h_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_u = *(c17_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c17_inData;
  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c17_b_u = c17_u.pinvTol;
  c17_b_y = NULL;
  sf_mex_assign(&c17_b_y, sf_mex_create("y", &c17_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_b_y, "pinvTol", "pinvTol", 0);
  c17_c_u = c17_u.pinvDamp;
  c17_c_y = NULL;
  sf_mex_assign(&c17_c_y, sf_mex_create("y", &c17_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_c_y, "pinvDamp", "pinvDamp", 0);
  c17_d_u = c17_u.pinvDampVb;
  c17_d_y = NULL;
  sf_mex_assign(&c17_d_y, sf_mex_create("y", &c17_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_d_y, "pinvDampVb", "pinvDampVb", 0);
  c17_e_u = c17_u.HessianQP;
  c17_e_y = NULL;
  sf_mex_assign(&c17_e_y, sf_mex_create("y", &c17_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_e_y, "HessianQP", "HessianQP", 0);
  c17_f_u = c17_u.impedances;
  c17_f_y = NULL;
  sf_mex_assign(&c17_f_y, sf_mex_create("y", &c17_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_f_y, "impedances", "impedances", 0);
  c17_g_u = c17_u.dampings;
  c17_g_y = NULL;
  sf_mex_assign(&c17_g_y, sf_mex_create("y", &c17_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_g_y, "dampings", "dampings", 0);
  c17_h_u = c17_u.norm_tolerance;
  c17_h_y = NULL;
  sf_mex_assign(&c17_h_y, sf_mex_create("y", &c17_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c17_y, c17_h_y, "norm_tolerance", "norm_tolerance", 0);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_eb_emlrt_marshallIn
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c17_u,
   const emlrtMsgIdentifier *c17_parentId)
{
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_y;
  emlrtMsgIdentifier c17_thisId;
  static const char * c17_fieldNames[7] = { "pinvTol", "pinvDamp", "pinvDampVb",
    "HessianQP", "impedances", "dampings", "norm_tolerance" };

  c17_thisId.fParent = c17_parentId;
  sf_mex_check_struct(c17_parentId, c17_u, 7, c17_fieldNames, 0U, 0);
  c17_thisId.fIdentifier = "pinvTol";
  c17_y.pinvTol = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "pinvTol", "pinvTol", 0)), &c17_thisId);
  c17_thisId.fIdentifier = "pinvDamp";
  c17_y.pinvDamp = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "pinvDamp", "pinvDamp", 0)), &c17_thisId);
  c17_thisId.fIdentifier = "pinvDampVb";
  c17_y.pinvDampVb = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "pinvDampVb", "pinvDampVb", 0)), &c17_thisId);
  c17_thisId.fIdentifier = "HessianQP";
  c17_y.HessianQP = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "HessianQP", "HessianQP", 0)), &c17_thisId);
  c17_thisId.fIdentifier = "impedances";
  c17_y.impedances = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "impedances", "impedances", 0)), &c17_thisId);
  c17_thisId.fIdentifier = "dampings";
  c17_y.dampings = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "dampings", "dampings", 0)), &c17_thisId);
  c17_thisId.fIdentifier = "norm_tolerance";
  c17_y.norm_tolerance = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c17_u, "norm_tolerance", "norm_tolerance", 0)), &c17_thisId);
  sf_mex_destroy(&c17_u);
  return c17_y;
}

static void c17_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_b_reg;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  c17_struct_1ZGMVR6bgCMpDdXTSGnu6G c17_y;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_b_reg = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_y = c17_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_b_reg),
    &c17_thisId);
  sf_mex_destroy(&c17_b_reg);
  *(c17_struct_1ZGMVR6bgCMpDdXTSGnu6G *)c17_outData = c17_y;
  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_p_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i749;
  int32_T c17_i750;
  int32_T c17_i751;
  real_T c17_b_inData[9];
  int32_T c17_i752;
  int32_T c17_i753;
  int32_T c17_i754;
  real_T c17_u[9];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i749 = 0;
  for (c17_i750 = 0; c17_i750 < 3; c17_i750++) {
    for (c17_i751 = 0; c17_i751 < 3; c17_i751++) {
      c17_b_inData[c17_i751 + c17_i749] = (*(real_T (*)[9])c17_inData)[c17_i751
        + c17_i749];
    }

    c17_i749 += 3;
  }

  c17_i752 = 0;
  for (c17_i753 = 0; c17_i753 < 3; c17_i753++) {
    for (c17_i754 = 0; c17_i754 < 3; c17_i754++) {
      c17_u[c17_i754 + c17_i752] = c17_b_inData[c17_i754 + c17_i752];
    }

    c17_i752 += 3;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_q_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i755;
  int32_T c17_i756;
  int32_T c17_i757;
  real_T c17_b_inData[174];
  int32_T c17_i758;
  int32_T c17_i759;
  int32_T c17_i760;
  real_T c17_u[174];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i755 = 0;
  for (c17_i756 = 0; c17_i756 < 29; c17_i756++) {
    for (c17_i757 = 0; c17_i757 < 6; c17_i757++) {
      c17_b_inData[c17_i757 + c17_i755] = (*(real_T (*)[174])c17_inData)
        [c17_i757 + c17_i755];
    }

    c17_i755 += 6;
  }

  c17_i758 = 0;
  for (c17_i759 = 0; c17_i759 < 29; c17_i759++) {
    for (c17_i760 = 0; c17_i760 < 6; c17_i760++) {
      c17_u[c17_i760 + c17_i758] = c17_b_inData[c17_i760 + c17_i758];
    }

    c17_i758 += 6;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 6, 29),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_r_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i761;
  real_T c17_b_inData[3];
  int32_T c17_i762;
  real_T c17_u[3];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i761 = 0; c17_i761 < 3; c17_i761++) {
    c17_b_inData[c17_i761] = (*(real_T (*)[3])c17_inData)[c17_i761];
  }

  for (c17_i762 = 0; c17_i762 < 3; c17_i762++) {
    c17_u[c17_i762] = c17_b_inData[c17_i762];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 3, 1), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_s_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i763;
  real_T c17_b_inData[29];
  int32_T c17_i764;
  real_T c17_u[29];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i763 = 0; c17_i763 < 29; c17_i763++) {
    c17_b_inData[c17_i763] = (*(real_T (*)[29])c17_inData)[c17_i763];
  }

  for (c17_i764 = 0; c17_i764 < 29; c17_i764++) {
    c17_u[c17_i764] = c17_b_inData[c17_i764];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 1, 29), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_t_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i765;
  int32_T c17_i766;
  int32_T c17_i767;
  real_T c17_b_inData[841];
  int32_T c17_i768;
  int32_T c17_i769;
  int32_T c17_i770;
  real_T c17_u[841];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i765 = 0;
  for (c17_i766 = 0; c17_i766 < 29; c17_i766++) {
    for (c17_i767 = 0; c17_i767 < 29; c17_i767++) {
      c17_b_inData[c17_i767 + c17_i765] = (*(real_T (*)[841])c17_inData)
        [c17_i767 + c17_i765];
    }

    c17_i765 += 29;
  }

  c17_i768 = 0;
  for (c17_i769 = 0; c17_i769 < 29; c17_i769++) {
    for (c17_i770 = 0; c17_i770 < 29; c17_i770++) {
      c17_u[c17_i770 + c17_i768] = c17_b_inData[c17_i770 + c17_i768];
    }

    c17_i768 += 29;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 29, 29),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_u_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i771;
  int32_T c17_i772;
  int32_T c17_i773;
  real_T c17_b_inData[529];
  int32_T c17_i774;
  int32_T c17_i775;
  int32_T c17_i776;
  real_T c17_u[529];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i771 = 0;
  for (c17_i772 = 0; c17_i772 < 23; c17_i772++) {
    for (c17_i773 = 0; c17_i773 < 23; c17_i773++) {
      c17_b_inData[c17_i773 + c17_i771] = (*(real_T (*)[529])c17_inData)
        [c17_i773 + c17_i771];
    }

    c17_i771 += 23;
  }

  c17_i774 = 0;
  for (c17_i775 = 0; c17_i775 < 23; c17_i775++) {
    for (c17_i776 = 0; c17_i776 < 23; c17_i776++) {
      c17_u[c17_i776 + c17_i774] = c17_b_inData[c17_i776 + c17_i774];
    }

    c17_i774 += 23;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 23, 23),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_v_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i777;
  real_T c17_b_inData[2];
  int32_T c17_i778;
  real_T c17_u[2];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i777 = 0; c17_i777 < 2; c17_i777++) {
    c17_b_inData[c17_i777] = (*(real_T (*)[2])c17_inData)[c17_i777];
  }

  for (c17_i778 = 0; c17_i778 < 2; c17_i778++) {
    c17_u[c17_i778] = c17_b_inData[c17_i778];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_w_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  real_T c17_u;
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_u = *(real_T *)c17_inData;
  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", &c17_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_nargout;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_nargout = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_y = c17_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_nargout),
    &c17_thisId);
  sf_mex_destroy(&c17_nargout);
  *(real_T *)c17_outData = c17_y;
  sf_mex_destroy(&c17_mxArrayInData);
}

static void c17_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_S;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[9];
  int32_T c17_i779;
  int32_T c17_i780;
  int32_T c17_i781;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_S = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_S), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_S);
  c17_i779 = 0;
  for (c17_i780 = 0; c17_i780 < 3; c17_i780++) {
    for (c17_i781 = 0; c17_i781 < 3; c17_i781++) {
      (*(real_T (*)[9])c17_outData)[c17_i781 + c17_i779] = c17_y[c17_i781 +
        c17_i779];
    }

    c17_i779 += 3;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_x_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i782;
  int32_T c17_i783;
  int32_T c17_i784;
  real_T c17_b_inData[276];
  int32_T c17_i785;
  int32_T c17_i786;
  int32_T c17_i787;
  real_T c17_u[276];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i782 = 0;
  for (c17_i783 = 0; c17_i783 < 23; c17_i783++) {
    for (c17_i784 = 0; c17_i784 < 12; c17_i784++) {
      c17_b_inData[c17_i784 + c17_i782] = (*(real_T (*)[276])c17_inData)
        [c17_i784 + c17_i782];
    }

    c17_i782 += 12;
  }

  c17_i785 = 0;
  for (c17_i786 = 0; c17_i786 < 23; c17_i786++) {
    for (c17_i787 = 0; c17_i787 < 12; c17_i787++) {
      c17_u[c17_i787 + c17_i785] = c17_b_inData[c17_i787 + c17_i785];
    }

    c17_i785 += 12;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 12, 23),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_fb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[276])
{
  real_T c17_dv65[276];
  int32_T c17_i788;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv65, 1, 0, 0U, 1, 0U, 2,
                12, 23);
  for (c17_i788 = 0; c17_i788 < 276; c17_i788++) {
    c17_y[c17_i788] = c17_dv65[c17_i788];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_A;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[276];
  int32_T c17_i789;
  int32_T c17_i790;
  int32_T c17_i791;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_A = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_fb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_A), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_A);
  c17_i789 = 0;
  for (c17_i790 = 0; c17_i790 < 23; c17_i790++) {
    for (c17_i791 = 0; c17_i791 < 12; c17_i791++) {
      (*(real_T (*)[276])c17_outData)[c17_i791 + c17_i789] = c17_y[c17_i791 +
        c17_i789];
    }

    c17_i789 += 12;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static boolean_T c17_gb_emlrt_marshallIn
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c17_u,
   const emlrtMsgIdentifier *c17_parentId)
{
  boolean_T c17_y;
  boolean_T c17_b0;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), &c17_b0, 1, 11, 0U, 0, 0U, 0);
  c17_y = c17_b0;
  sf_mex_destroy(&c17_u);
  return c17_y;
}

static void c17_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_useExtArmForces;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  boolean_T c17_y;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_useExtArmForces = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_y = c17_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_useExtArmForces),
    &c17_thisId);
  sf_mex_destroy(&c17_useExtArmForces);
  *(boolean_T *)c17_outData = c17_y;
  sf_mex_destroy(&c17_mxArrayInData);
}

static void c17_hb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[16])
{
  real_T c17_dv66[16];
  int32_T c17_i792;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv66, 1, 0, 0U, 1, 0U, 2, 4,
                4);
  for (c17_i792 = 0; c17_i792 < 16; c17_i792++) {
    c17_y[c17_i792] = c17_dv66[c17_i792];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_w_H_rArm;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[16];
  int32_T c17_i793;
  int32_T c17_i794;
  int32_T c17_i795;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_w_H_rArm = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_hb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_w_H_rArm), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_w_H_rArm);
  c17_i793 = 0;
  for (c17_i794 = 0; c17_i794 < 4; c17_i794++) {
    for (c17_i795 = 0; c17_i795 < 4; c17_i795++) {
      (*(real_T (*)[16])c17_outData)[c17_i795 + c17_i793] = c17_y[c17_i795 +
        c17_i793];
    }

    c17_i793 += 4;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static void c17_ib_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[174])
{
  real_T c17_dv67[174];
  int32_T c17_i796;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv67, 1, 0, 0U, 1, 0U, 2, 6,
                29);
  for (c17_i796 = 0; c17_i796 < 174; c17_i796++) {
    c17_y[c17_i796] = c17_dv67[c17_i796];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_J_CoM;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[174];
  int32_T c17_i797;
  int32_T c17_i798;
  int32_T c17_i799;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_J_CoM = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_J_CoM), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_J_CoM);
  c17_i797 = 0;
  for (c17_i798 = 0; c17_i798 < 29; c17_i798++) {
    for (c17_i799 = 0; c17_i799 < 6; c17_i799++) {
      (*(real_T (*)[174])c17_outData)[c17_i799 + c17_i797] = c17_y[c17_i799 +
        c17_i797];
    }

    c17_i797 += 6;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static void c17_jb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[29])
{
  real_T c17_dv68[29];
  int32_T c17_i800;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv68, 1, 0, 0U, 1, 0U, 1,
                29);
  for (c17_i800 = 0; c17_i800 < 29; c17_i800++) {
    c17_y[c17_i800] = c17_dv68[c17_i800];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_h;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[29];
  int32_T c17_i801;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_h = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_jb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_h), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_h);
  for (c17_i801 = 0; c17_i801 < 29; c17_i801++) {
    (*(real_T (*)[29])c17_outData)[c17_i801] = c17_y[c17_i801];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static void c17_kb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[841])
{
  real_T c17_dv69[841];
  int32_T c17_i802;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv69, 1, 0, 0U, 1, 0U, 2,
                29, 29);
  for (c17_i802 = 0; c17_i802 < 841; c17_i802++) {
    c17_y[c17_i802] = c17_dv69[c17_i802];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_M;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[841];
  int32_T c17_i803;
  int32_T c17_i804;
  int32_T c17_i805;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_M = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_kb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_M), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_M);
  c17_i803 = 0;
  for (c17_i804 = 0; c17_i804 < 29; c17_i804++) {
    for (c17_i805 = 0; c17_i805 < 29; c17_i805++) {
      (*(real_T (*)[841])c17_outData)[c17_i805 + c17_i803] = c17_y[c17_i805 +
        c17_i803];
    }

    c17_i803 += 29;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static void c17_lb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[2])
{
  real_T c17_dv70[2];
  int32_T c17_i806;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv70, 1, 0, 0U, 1, 0U, 1, 2);
  for (c17_i806 = 0; c17_i806 < 2; c17_i806++) {
    c17_y[c17_i806] = c17_dv70[c17_i806];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_constraints;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[2];
  int32_T c17_i807;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_constraints = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_lb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_constraints),
    &c17_thisId, c17_y);
  sf_mex_destroy(&c17_constraints);
  for (c17_i807 = 0; c17_i807 < 2; c17_i807++) {
    (*(real_T (*)[2])c17_outData)[c17_i807] = c17_y[c17_i807];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static void c17_mb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[529])
{
  real_T c17_dv71[529];
  int32_T c17_i808;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv71, 1, 0, 0U, 1, 0U, 2,
                23, 23);
  for (c17_i808 = 0; c17_i808 < 529; c17_i808++) {
    c17_y[c17_i808] = c17_dv71[c17_i808];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_w_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_dampings;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[529];
  int32_T c17_i809;
  int32_T c17_i810;
  int32_T c17_i811;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_dampings = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_mb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_dampings), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_dampings);
  c17_i809 = 0;
  for (c17_i810 = 0; c17_i810 < 23; c17_i810++) {
    for (c17_i811 = 0; c17_i811 < 23; c17_i811++) {
      (*(real_T (*)[529])c17_outData)[c17_i811 + c17_i809] = c17_y[c17_i811 +
        c17_i809];
    }

    c17_i809 += 23;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_y_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i812;
  int32_T c17_i813;
  int32_T c17_i814;
  real_T c17_b_inData[348];
  int32_T c17_i815;
  int32_T c17_i816;
  int32_T c17_i817;
  real_T c17_u[348];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i812 = 0;
  for (c17_i813 = 0; c17_i813 < 29; c17_i813++) {
    for (c17_i814 = 0; c17_i814 < 12; c17_i814++) {
      c17_b_inData[c17_i814 + c17_i812] = (*(real_T (*)[348])c17_inData)
        [c17_i814 + c17_i812];
    }

    c17_i812 += 12;
  }

  c17_i815 = 0;
  for (c17_i816 = 0; c17_i816 < 29; c17_i816++) {
    for (c17_i817 = 0; c17_i817 < 12; c17_i817++) {
      c17_u[c17_i817 + c17_i815] = c17_b_inData[c17_i817 + c17_i815];
    }

    c17_i815 += 12;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 12, 29),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_nb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[348])
{
  real_T c17_dv72[348];
  int32_T c17_i818;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv72, 1, 0, 0U, 1, 0U, 2,
                12, 29);
  for (c17_i818 = 0; c17_i818 < 348; c17_i818++) {
    c17_y[c17_i818] = c17_dv72[c17_i818];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_x_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_JcMinv;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[348];
  int32_T c17_i819;
  int32_T c17_i820;
  int32_T c17_i821;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_JcMinv = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_nb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_JcMinv), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_JcMinv);
  c17_i819 = 0;
  for (c17_i820 = 0; c17_i820 < 29; c17_i820++) {
    for (c17_i821 = 0; c17_i821 < 12; c17_i821++) {
      (*(real_T (*)[348])c17_outData)[c17_i821 + c17_i819] = c17_y[c17_i821 +
        c17_i819];
    }

    c17_i819 += 12;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_ab_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i822;
  int32_T c17_i823;
  int32_T c17_i824;
  real_T c17_b_inData[72];
  int32_T c17_i825;
  int32_T c17_i826;
  int32_T c17_i827;
  real_T c17_u[72];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i822 = 0;
  for (c17_i823 = 0; c17_i823 < 12; c17_i823++) {
    for (c17_i824 = 0; c17_i824 < 6; c17_i824++) {
      c17_b_inData[c17_i824 + c17_i822] = (*(real_T (*)[72])c17_inData)[c17_i824
        + c17_i822];
    }

    c17_i822 += 6;
  }

  c17_i825 = 0;
  for (c17_i826 = 0; c17_i826 < 12; c17_i826++) {
    for (c17_i827 = 0; c17_i827 < 6; c17_i827++) {
      c17_u[c17_i827 + c17_i825] = c17_b_inData[c17_i827 + c17_i825];
    }

    c17_i825 += 6;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 6, 12),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_ob_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[72])
{
  real_T c17_dv73[72];
  int32_T c17_i828;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv73, 1, 0, 0U, 1, 0U, 2, 6,
                12);
  for (c17_i828 = 0; c17_i828 < 72; c17_i828++) {
    c17_y[c17_i828] = c17_dv73[c17_i828];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_y_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_A_arms;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[72];
  int32_T c17_i829;
  int32_T c17_i830;
  int32_T c17_i831;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_A_arms = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_ob_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_A_arms), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_A_arms);
  c17_i829 = 0;
  for (c17_i830 = 0; c17_i830 < 12; c17_i830++) {
    for (c17_i831 = 0; c17_i831 < 6; c17_i831++) {
      (*(real_T (*)[72])c17_outData)[c17_i831 + c17_i829] = c17_y[c17_i831 +
        c17_i829];
    }

    c17_i829 += 6;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_bb_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i832;
  int32_T c17_i833;
  int32_T c17_i834;
  real_T c17_b_inData[72];
  int32_T c17_i835;
  int32_T c17_i836;
  int32_T c17_i837;
  real_T c17_u[72];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i832 = 0;
  for (c17_i833 = 0; c17_i833 < 6; c17_i833++) {
    for (c17_i834 = 0; c17_i834 < 12; c17_i834++) {
      c17_b_inData[c17_i834 + c17_i832] = (*(real_T (*)[72])c17_inData)[c17_i834
        + c17_i832];
    }

    c17_i832 += 12;
  }

  c17_i835 = 0;
  for (c17_i836 = 0; c17_i836 < 6; c17_i836++) {
    for (c17_i837 = 0; c17_i837 < 12; c17_i837++) {
      c17_u[c17_i837 + c17_i835] = c17_b_inData[c17_i837 + c17_i835];
    }

    c17_i835 += 12;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 12, 6),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_pb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[72])
{
  real_T c17_dv74[72];
  int32_T c17_i838;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv74, 1, 0, 0U, 1, 0U, 2,
                12, 6);
  for (c17_i838 = 0; c17_i838 < 72; c17_i838++) {
    c17_y[c17_i838] = c17_dv74[c17_i838];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_ab_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_pinvA;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[72];
  int32_T c17_i839;
  int32_T c17_i840;
  int32_T c17_i841;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_pinvA = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_pb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_pinvA), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_pinvA);
  c17_i839 = 0;
  for (c17_i840 = 0; c17_i840 < 6; c17_i840++) {
    for (c17_i841 = 0; c17_i841 < 12; c17_i841++) {
      (*(real_T (*)[72])c17_outData)[c17_i841 + c17_i839] = c17_y[c17_i841 +
        c17_i839];
    }

    c17_i839 += 12;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_cb_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i842;
  int32_T c17_i843;
  int32_T c17_i844;
  real_T c17_b_inData[667];
  int32_T c17_i845;
  int32_T c17_i846;
  int32_T c17_i847;
  real_T c17_u[667];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i842 = 0;
  for (c17_i843 = 0; c17_i843 < 23; c17_i843++) {
    for (c17_i844 = 0; c17_i844 < 29; c17_i844++) {
      c17_b_inData[c17_i844 + c17_i842] = (*(real_T (*)[667])c17_inData)
        [c17_i844 + c17_i842];
    }

    c17_i842 += 29;
  }

  c17_i845 = 0;
  for (c17_i846 = 0; c17_i846 < 23; c17_i846++) {
    for (c17_i847 = 0; c17_i847 < 29; c17_i847++) {
      c17_u[c17_i847 + c17_i845] = c17_b_inData[c17_i847 + c17_i845];
    }

    c17_i845 += 29;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 29, 23),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static const mxArray *c17_db_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i848;
  int32_T c17_i849;
  int32_T c17_i850;
  real_T c17_b_inData[138];
  int32_T c17_i851;
  int32_T c17_i852;
  int32_T c17_i853;
  real_T c17_u[138];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_i848 = 0;
  for (c17_i849 = 0; c17_i849 < 23; c17_i849++) {
    for (c17_i850 = 0; c17_i850 < 6; c17_i850++) {
      c17_b_inData[c17_i850 + c17_i848] = (*(real_T (*)[138])c17_inData)
        [c17_i850 + c17_i848];
    }

    c17_i848 += 6;
  }

  c17_i851 = 0;
  for (c17_i852 = 0; c17_i852 < 23; c17_i852++) {
    for (c17_i853 = 0; c17_i853 < 6; c17_i853++) {
      c17_u[c17_i853 + c17_i851] = c17_b_inData[c17_i853 + c17_i851];
    }

    c17_i851 += 6;
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 6, 23),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_qb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId,
  real_T c17_y[138])
{
  real_T c17_dv75[138];
  int32_T c17_i854;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), c17_dv75, 1, 0, 0U, 1, 0U, 2, 6,
                23);
  for (c17_i854 = 0; c17_i854 < 138; c17_i854++) {
    c17_y[c17_i854] = c17_dv75[c17_i854];
  }

  sf_mex_destroy(&c17_u);
}

static void c17_bb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_Mbj;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[138];
  int32_T c17_i855;
  int32_T c17_i856;
  int32_T c17_i857;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_Mbj = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_qb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_Mbj), &c17_thisId, c17_y);
  sf_mex_destroy(&c17_Mbj);
  c17_i855 = 0;
  for (c17_i856 = 0; c17_i856 < 23; c17_i856++) {
    for (c17_i857 = 0; c17_i857 < 6; c17_i857++) {
      (*(real_T (*)[138])c17_outData)[c17_i857 + c17_i855] = c17_y[c17_i857 +
        c17_i855];
    }

    c17_i855 += 6;
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

static const mxArray *c17_eb_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_i858;
  real_T c17_b_inData[23];
  int32_T c17_i859;
  real_T c17_u[23];
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  for (c17_i858 = 0; c17_i858 < 23; c17_i858++) {
    c17_b_inData[c17_i858] = (*(real_T (*)[23])c17_inData)[c17_i858];
  }

  for (c17_i859 = 0; c17_i859 < 23; c17_i859++) {
    c17_u[c17_i859] = c17_b_inData[c17_i859];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 0, 0U, 1U, 0U, 2, 1, 23),
                FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static void c17_cb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_dampings;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  real_T c17_y[23];
  int32_T c17_i860;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_dampings = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_dampings), &c17_thisId,
    c17_y);
  sf_mex_destroy(&c17_dampings);
  for (c17_i860 = 0; c17_i860 < 23; c17_i860++) {
    (*(real_T (*)[23])c17_outData)[c17_i860] = c17_y[c17_i860];
  }

  sf_mex_destroy(&c17_mxArrayInData);
}

const mxArray *sf_c17_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c17_nameCaptureInfo;
  c17_ResolvedFunctionInfo c17_info[302];
  const mxArray *c17_m2 = NULL;
  int32_T c17_i861;
  c17_ResolvedFunctionInfo *c17_r2;
  c17_nameCaptureInfo = NULL;
  c17_nameCaptureInfo = NULL;
  c17_info_helper(c17_info);
  c17_b_info_helper(c17_info);
  c17_c_info_helper(c17_info);
  c17_d_info_helper(c17_info);
  c17_e_info_helper(c17_info);
  sf_mex_assign(&c17_m2, sf_mex_createstruct("nameCaptureInfo", 1, 302), FALSE);
  for (c17_i861 = 0; c17_i861 < 302; c17_i861++) {
    c17_r2 = &c17_info[c17_i861];
    sf_mex_addfield(c17_m2, sf_mex_create("nameCaptureInfo", c17_r2->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c17_r2->context)), "context", "nameCaptureInfo",
                    c17_i861);
    sf_mex_addfield(c17_m2, sf_mex_create("nameCaptureInfo", c17_r2->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c17_r2->name)), "name", "nameCaptureInfo",
                    c17_i861);
    sf_mex_addfield(c17_m2, sf_mex_create("nameCaptureInfo",
      c17_r2->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c17_r2->dominantType)),
                    "dominantType", "nameCaptureInfo", c17_i861);
    sf_mex_addfield(c17_m2, sf_mex_create("nameCaptureInfo", c17_r2->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c17_r2->resolved)), "resolved",
                    "nameCaptureInfo", c17_i861);
    sf_mex_addfield(c17_m2, sf_mex_create("nameCaptureInfo", &c17_r2->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c17_i861);
    sf_mex_addfield(c17_m2, sf_mex_create("nameCaptureInfo", &c17_r2->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c17_i861);
    sf_mex_addfield(c17_m2, sf_mex_create("nameCaptureInfo",
      &c17_r2->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c17_i861);
    sf_mex_addfield(c17_m2, sf_mex_create("nameCaptureInfo",
      &c17_r2->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c17_i861);
  }

  sf_mex_assign(&c17_nameCaptureInfo, c17_m2, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c17_nameCaptureInfo);
  return c17_nameCaptureInfo;
}

static void c17_info_helper(c17_ResolvedFunctionInfo c17_info[302])
{
  c17_info[0].context = "";
  c17_info[0].name = "balancingController";
  c17_info[0].dominantType = "struct";
  c17_info[0].resolved =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[0].fileTimeLo = 1495126737U;
  c17_info[0].fileTimeHi = 0U;
  c17_info[0].mFileTimeLo = 0U;
  c17_info[0].mFileTimeHi = 0U;
  c17_info[1].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[1].name = "eye";
  c17_info[1].dominantType = "double";
  c17_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c17_info[1].fileTimeLo = 1286818688U;
  c17_info[1].fileTimeHi = 0U;
  c17_info[1].mFileTimeLo = 0U;
  c17_info[1].mFileTimeHi = 0U;
  c17_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c17_info[2].name = "eml_assert_valid_size_arg";
  c17_info[2].dominantType = "double";
  c17_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c17_info[2].fileTimeLo = 1286818694U;
  c17_info[2].fileTimeHi = 0U;
  c17_info[2].mFileTimeLo = 0U;
  c17_info[2].mFileTimeHi = 0U;
  c17_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c17_info[3].name = "isinf";
  c17_info[3].dominantType = "double";
  c17_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c17_info[3].fileTimeLo = 1286818760U;
  c17_info[3].fileTimeHi = 0U;
  c17_info[3].mFileTimeLo = 0U;
  c17_info[3].mFileTimeHi = 0U;
  c17_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c17_info[4].name = "mtimes";
  c17_info[4].dominantType = "double";
  c17_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[4].fileTimeLo = 1289519692U;
  c17_info[4].fileTimeHi = 0U;
  c17_info[4].mFileTimeLo = 0U;
  c17_info[4].mFileTimeHi = 0U;
  c17_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c17_info[5].name = "eml_index_class";
  c17_info[5].dominantType = "";
  c17_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[5].fileTimeLo = 1323170578U;
  c17_info[5].fileTimeHi = 0U;
  c17_info[5].mFileTimeLo = 0U;
  c17_info[5].mFileTimeHi = 0U;
  c17_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c17_info[6].name = "intmax";
  c17_info[6].dominantType = "char";
  c17_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c17_info[6].fileTimeLo = 1311255316U;
  c17_info[6].fileTimeHi = 0U;
  c17_info[6].mFileTimeLo = 0U;
  c17_info[6].mFileTimeHi = 0U;
  c17_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c17_info[7].name = "eml_is_float_class";
  c17_info[7].dominantType = "char";
  c17_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c17_info[7].fileTimeLo = 1286818782U;
  c17_info[7].fileTimeHi = 0U;
  c17_info[7].mFileTimeLo = 0U;
  c17_info[7].mFileTimeHi = 0U;
  c17_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c17_info[8].name = "min";
  c17_info[8].dominantType = "double";
  c17_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c17_info[8].fileTimeLo = 1311255318U;
  c17_info[8].fileTimeHi = 0U;
  c17_info[8].mFileTimeLo = 0U;
  c17_info[8].mFileTimeHi = 0U;
  c17_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c17_info[9].name = "eml_min_or_max";
  c17_info[9].dominantType = "char";
  c17_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c17_info[9].fileTimeLo = 1334071490U;
  c17_info[9].fileTimeHi = 0U;
  c17_info[9].mFileTimeLo = 0U;
  c17_info[9].mFileTimeHi = 0U;
  c17_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c17_info[10].name = "eml_scalar_eg";
  c17_info[10].dominantType = "double";
  c17_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[10].fileTimeLo = 1286818796U;
  c17_info[10].fileTimeHi = 0U;
  c17_info[10].mFileTimeLo = 0U;
  c17_info[10].mFileTimeHi = 0U;
  c17_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c17_info[11].name = "eml_scalexp_alloc";
  c17_info[11].dominantType = "double";
  c17_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c17_info[11].fileTimeLo = 1352424860U;
  c17_info[11].fileTimeHi = 0U;
  c17_info[11].mFileTimeLo = 0U;
  c17_info[11].mFileTimeHi = 0U;
  c17_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c17_info[12].name = "eml_index_class";
  c17_info[12].dominantType = "";
  c17_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[12].fileTimeLo = 1323170578U;
  c17_info[12].fileTimeHi = 0U;
  c17_info[12].mFileTimeLo = 0U;
  c17_info[12].mFileTimeHi = 0U;
  c17_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c17_info[13].name = "eml_scalar_eg";
  c17_info[13].dominantType = "double";
  c17_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[13].fileTimeLo = 1286818796U;
  c17_info[13].fileTimeHi = 0U;
  c17_info[13].mFileTimeLo = 0U;
  c17_info[13].mFileTimeHi = 0U;
  c17_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c17_info[14].name = "eml_index_class";
  c17_info[14].dominantType = "";
  c17_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[14].fileTimeLo = 1323170578U;
  c17_info[14].fileTimeHi = 0U;
  c17_info[14].mFileTimeLo = 0U;
  c17_info[14].mFileTimeHi = 0U;
  c17_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c17_info[15].name = "eml_int_forloop_overflow_check";
  c17_info[15].dominantType = "";
  c17_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[15].fileTimeLo = 1346510340U;
  c17_info[15].fileTimeHi = 0U;
  c17_info[15].mFileTimeLo = 0U;
  c17_info[15].mFileTimeHi = 0U;
  c17_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c17_info[16].name = "intmax";
  c17_info[16].dominantType = "char";
  c17_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c17_info[16].fileTimeLo = 1311255316U;
  c17_info[16].fileTimeHi = 0U;
  c17_info[16].mFileTimeLo = 0U;
  c17_info[16].mFileTimeHi = 0U;
  c17_info[17].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[17].name = "mtimes";
  c17_info[17].dominantType = "double";
  c17_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[17].fileTimeLo = 1289519692U;
  c17_info[17].fileTimeHi = 0U;
  c17_info[17].mFileTimeLo = 0U;
  c17_info[17].mFileTimeHi = 0U;
  c17_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[18].name = "eml_index_class";
  c17_info[18].dominantType = "";
  c17_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[18].fileTimeLo = 1323170578U;
  c17_info[18].fileTimeHi = 0U;
  c17_info[18].mFileTimeLo = 0U;
  c17_info[18].mFileTimeHi = 0U;
  c17_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[19].name = "eml_scalar_eg";
  c17_info[19].dominantType = "double";
  c17_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[19].fileTimeLo = 1286818796U;
  c17_info[19].fileTimeHi = 0U;
  c17_info[19].mFileTimeLo = 0U;
  c17_info[19].mFileTimeHi = 0U;
  c17_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[20].name = "eml_xgemm";
  c17_info[20].dominantType = "char";
  c17_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c17_info[20].fileTimeLo = 1299076772U;
  c17_info[20].fileTimeHi = 0U;
  c17_info[20].mFileTimeLo = 0U;
  c17_info[20].mFileTimeHi = 0U;
  c17_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c17_info[21].name = "eml_blas_inline";
  c17_info[21].dominantType = "";
  c17_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[21].fileTimeLo = 1299076768U;
  c17_info[21].fileTimeHi = 0U;
  c17_info[21].mFileTimeLo = 0U;
  c17_info[21].mFileTimeHi = 0U;
  c17_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c17_info[22].name = "mtimes";
  c17_info[22].dominantType = "double";
  c17_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[22].fileTimeLo = 1289519692U;
  c17_info[22].fileTimeHi = 0U;
  c17_info[22].mFileTimeLo = 0U;
  c17_info[22].mFileTimeHi = 0U;
  c17_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c17_info[23].name = "eml_index_class";
  c17_info[23].dominantType = "";
  c17_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[23].fileTimeLo = 1323170578U;
  c17_info[23].fileTimeHi = 0U;
  c17_info[23].mFileTimeLo = 0U;
  c17_info[23].mFileTimeHi = 0U;
  c17_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c17_info[24].name = "eml_scalar_eg";
  c17_info[24].dominantType = "double";
  c17_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[24].fileTimeLo = 1286818796U;
  c17_info[24].fileTimeHi = 0U;
  c17_info[24].mFileTimeLo = 0U;
  c17_info[24].mFileTimeHi = 0U;
  c17_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c17_info[25].name = "eml_refblas_xgemm";
  c17_info[25].dominantType = "char";
  c17_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c17_info[25].fileTimeLo = 1299076774U;
  c17_info[25].fileTimeHi = 0U;
  c17_info[25].mFileTimeLo = 0U;
  c17_info[25].mFileTimeHi = 0U;
  c17_info[26].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[26].name = "Sf";
  c17_info[26].dominantType = "double";
  c17_info[26].resolved =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/Sf.m";
  c17_info[26].fileTimeLo = 1495096798U;
  c17_info[26].fileTimeHi = 0U;
  c17_info[26].mFileTimeLo = 0U;
  c17_info[26].mFileTimeHi = 0U;
  c17_info[27].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[27].name = "pinv";
  c17_info[27].dominantType = "double";
  c17_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m";
  c17_info[27].fileTimeLo = 1286818828U;
  c17_info[27].fileTimeHi = 0U;
  c17_info[27].mFileTimeLo = 0U;
  c17_info[27].mFileTimeHi = 0U;
  c17_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[28].name = "eml_index_class";
  c17_info[28].dominantType = "";
  c17_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[28].fileTimeLo = 1323170578U;
  c17_info[28].fileTimeHi = 0U;
  c17_info[28].mFileTimeLo = 0U;
  c17_info[28].mFileTimeHi = 0U;
  c17_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[29].name = "eml_scalar_eg";
  c17_info[29].dominantType = "double";
  c17_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[29].fileTimeLo = 1286818796U;
  c17_info[29].fileTimeHi = 0U;
  c17_info[29].mFileTimeLo = 0U;
  c17_info[29].mFileTimeHi = 0U;
  c17_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[30].name = "svd";
  c17_info[30].dominantType = "double";
  c17_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c17_info[30].fileTimeLo = 1286818832U;
  c17_info[30].fileTimeHi = 0U;
  c17_info[30].mFileTimeLo = 0U;
  c17_info[30].mFileTimeHi = 0U;
  c17_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c17_info[31].name = "eml_index_class";
  c17_info[31].dominantType = "";
  c17_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[31].fileTimeLo = 1323170578U;
  c17_info[31].fileTimeHi = 0U;
  c17_info[31].mFileTimeLo = 0U;
  c17_info[31].mFileTimeHi = 0U;
  c17_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c17_info[32].name = "eml_int_forloop_overflow_check";
  c17_info[32].dominantType = "";
  c17_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[32].fileTimeLo = 1346510340U;
  c17_info[32].fileTimeHi = 0U;
  c17_info[32].mFileTimeLo = 0U;
  c17_info[32].mFileTimeHi = 0U;
  c17_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c17_info[33].name = "isfinite";
  c17_info[33].dominantType = "double";
  c17_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c17_info[33].fileTimeLo = 1286818758U;
  c17_info[33].fileTimeHi = 0U;
  c17_info[33].mFileTimeLo = 0U;
  c17_info[33].mFileTimeHi = 0U;
  c17_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c17_info[34].name = "isinf";
  c17_info[34].dominantType = "double";
  c17_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c17_info[34].fileTimeLo = 1286818760U;
  c17_info[34].fileTimeHi = 0U;
  c17_info[34].mFileTimeLo = 0U;
  c17_info[34].mFileTimeHi = 0U;
  c17_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c17_info[35].name = "isnan";
  c17_info[35].dominantType = "double";
  c17_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c17_info[35].fileTimeLo = 1286818760U;
  c17_info[35].fileTimeHi = 0U;
  c17_info[35].mFileTimeLo = 0U;
  c17_info[35].mFileTimeHi = 0U;
  c17_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c17_info[36].name = "eml_error";
  c17_info[36].dominantType = "char";
  c17_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c17_info[36].fileTimeLo = 1343830358U;
  c17_info[36].fileTimeHi = 0U;
  c17_info[36].mFileTimeLo = 0U;
  c17_info[36].mFileTimeHi = 0U;
  c17_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m";
  c17_info[37].name = "eml_xgesvd";
  c17_info[37].dominantType = "double";
  c17_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c17_info[37].fileTimeLo = 1286818806U;
  c17_info[37].fileTimeHi = 0U;
  c17_info[37].mFileTimeLo = 0U;
  c17_info[37].mFileTimeHi = 0U;
  c17_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c17_info[38].name = "eml_lapack_xgesvd";
  c17_info[38].dominantType = "double";
  c17_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c17_info[38].fileTimeLo = 1286818810U;
  c17_info[38].fileTimeHi = 0U;
  c17_info[38].mFileTimeLo = 0U;
  c17_info[38].mFileTimeHi = 0U;
  c17_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c17_info[39].name = "eml_matlab_zsvdc";
  c17_info[39].dominantType = "double";
  c17_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[39].fileTimeLo = 1295284866U;
  c17_info[39].fileTimeHi = 0U;
  c17_info[39].mFileTimeLo = 0U;
  c17_info[39].mFileTimeHi = 0U;
  c17_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[40].name = "eml_index_class";
  c17_info[40].dominantType = "";
  c17_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[40].fileTimeLo = 1323170578U;
  c17_info[40].fileTimeHi = 0U;
  c17_info[40].mFileTimeLo = 0U;
  c17_info[40].mFileTimeHi = 0U;
  c17_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[41].name = "eml_scalar_eg";
  c17_info[41].dominantType = "double";
  c17_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[41].fileTimeLo = 1286818796U;
  c17_info[41].fileTimeHi = 0U;
  c17_info[41].mFileTimeLo = 0U;
  c17_info[41].mFileTimeHi = 0U;
  c17_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[42].name = "eml_index_plus";
  c17_info[42].dominantType = "double";
  c17_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[42].fileTimeLo = 1286818778U;
  c17_info[42].fileTimeHi = 0U;
  c17_info[42].mFileTimeLo = 0U;
  c17_info[42].mFileTimeHi = 0U;
  c17_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[43].name = "eml_index_class";
  c17_info[43].dominantType = "";
  c17_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[43].fileTimeLo = 1323170578U;
  c17_info[43].fileTimeHi = 0U;
  c17_info[43].mFileTimeLo = 0U;
  c17_info[43].mFileTimeHi = 0U;
  c17_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[44].name = "min";
  c17_info[44].dominantType = "coder.internal.indexInt";
  c17_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c17_info[44].fileTimeLo = 1311255318U;
  c17_info[44].fileTimeHi = 0U;
  c17_info[44].mFileTimeLo = 0U;
  c17_info[44].mFileTimeHi = 0U;
  c17_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c17_info[45].name = "eml_scalar_eg";
  c17_info[45].dominantType = "coder.internal.indexInt";
  c17_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[45].fileTimeLo = 1286818796U;
  c17_info[45].fileTimeHi = 0U;
  c17_info[45].mFileTimeLo = 0U;
  c17_info[45].mFileTimeHi = 0U;
  c17_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c17_info[46].name = "eml_scalexp_alloc";
  c17_info[46].dominantType = "coder.internal.indexInt";
  c17_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c17_info[46].fileTimeLo = 1352424860U;
  c17_info[46].fileTimeHi = 0U;
  c17_info[46].mFileTimeLo = 0U;
  c17_info[46].mFileTimeHi = 0U;
  c17_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c17_info[47].name = "eml_scalar_eg";
  c17_info[47].dominantType = "coder.internal.indexInt";
  c17_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[47].fileTimeLo = 1286818796U;
  c17_info[47].fileTimeHi = 0U;
  c17_info[47].mFileTimeLo = 0U;
  c17_info[47].mFileTimeHi = 0U;
  c17_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[48].name = "max";
  c17_info[48].dominantType = "double";
  c17_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c17_info[48].fileTimeLo = 1311255316U;
  c17_info[48].fileTimeHi = 0U;
  c17_info[48].mFileTimeLo = 0U;
  c17_info[48].mFileTimeHi = 0U;
  c17_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c17_info[49].name = "eml_min_or_max";
  c17_info[49].dominantType = "char";
  c17_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c17_info[49].fileTimeLo = 1334071490U;
  c17_info[49].fileTimeHi = 0U;
  c17_info[49].mFileTimeLo = 0U;
  c17_info[49].mFileTimeHi = 0U;
  c17_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c17_info[50].name = "eml_relop";
  c17_info[50].dominantType = "function_handle";
  c17_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c17_info[50].fileTimeLo = 1342451182U;
  c17_info[50].fileTimeHi = 0U;
  c17_info[50].mFileTimeLo = 0U;
  c17_info[50].mFileTimeHi = 0U;
  c17_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c17_info[51].name = "coder.internal.indexIntRelop";
  c17_info[51].dominantType = "";
  c17_info[51].resolved =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m";
  c17_info[51].fileTimeLo = 1326728322U;
  c17_info[51].fileTimeHi = 0U;
  c17_info[51].mFileTimeLo = 0U;
  c17_info[51].mFileTimeHi = 0U;
  c17_info[52].context =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass";
  c17_info[52].name = "eml_float_model";
  c17_info[52].dominantType = "char";
  c17_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c17_info[52].fileTimeLo = 1326727996U;
  c17_info[52].fileTimeHi = 0U;
  c17_info[52].mFileTimeLo = 0U;
  c17_info[52].mFileTimeHi = 0U;
  c17_info[53].context =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass";
  c17_info[53].name = "intmin";
  c17_info[53].dominantType = "char";
  c17_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c17_info[53].fileTimeLo = 1311255318U;
  c17_info[53].fileTimeHi = 0U;
  c17_info[53].mFileTimeLo = 0U;
  c17_info[53].mFileTimeHi = 0U;
  c17_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c17_info[54].name = "isnan";
  c17_info[54].dominantType = "coder.internal.indexInt";
  c17_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c17_info[54].fileTimeLo = 1286818760U;
  c17_info[54].fileTimeHi = 0U;
  c17_info[54].mFileTimeLo = 0U;
  c17_info[54].mFileTimeHi = 0U;
  c17_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[55].name = "eml_index_minus";
  c17_info[55].dominantType = "double";
  c17_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[55].fileTimeLo = 1286818778U;
  c17_info[55].fileTimeHi = 0U;
  c17_info[55].mFileTimeLo = 0U;
  c17_info[55].mFileTimeHi = 0U;
  c17_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[56].name = "eml_index_class";
  c17_info[56].dominantType = "";
  c17_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[56].fileTimeLo = 1323170578U;
  c17_info[56].fileTimeHi = 0U;
  c17_info[56].mFileTimeLo = 0U;
  c17_info[56].mFileTimeHi = 0U;
  c17_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[57].name = "max";
  c17_info[57].dominantType = "coder.internal.indexInt";
  c17_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c17_info[57].fileTimeLo = 1311255316U;
  c17_info[57].fileTimeHi = 0U;
  c17_info[57].mFileTimeLo = 0U;
  c17_info[57].mFileTimeHi = 0U;
  c17_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[58].name = "eml_int_forloop_overflow_check";
  c17_info[58].dominantType = "";
  c17_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[58].fileTimeLo = 1346510340U;
  c17_info[58].fileTimeHi = 0U;
  c17_info[58].mFileTimeLo = 0U;
  c17_info[58].mFileTimeHi = 0U;
  c17_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[59].name = "eml_index_times";
  c17_info[59].dominantType = "coder.internal.indexInt";
  c17_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c17_info[59].fileTimeLo = 1286818780U;
  c17_info[59].fileTimeHi = 0U;
  c17_info[59].mFileTimeLo = 0U;
  c17_info[59].mFileTimeHi = 0U;
  c17_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c17_info[60].name = "eml_index_class";
  c17_info[60].dominantType = "";
  c17_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[60].fileTimeLo = 1323170578U;
  c17_info[60].fileTimeHi = 0U;
  c17_info[60].mFileTimeLo = 0U;
  c17_info[60].mFileTimeHi = 0U;
  c17_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[61].name = "eml_index_plus";
  c17_info[61].dominantType = "coder.internal.indexInt";
  c17_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[61].fileTimeLo = 1286818778U;
  c17_info[61].fileTimeHi = 0U;
  c17_info[61].mFileTimeLo = 0U;
  c17_info[61].mFileTimeHi = 0U;
  c17_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[62].name = "eml_index_minus";
  c17_info[62].dominantType = "coder.internal.indexInt";
  c17_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[62].fileTimeLo = 1286818778U;
  c17_info[62].fileTimeHi = 0U;
  c17_info[62].mFileTimeLo = 0U;
  c17_info[62].mFileTimeHi = 0U;
  c17_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[63].name = "eml_xnrm2";
  c17_info[63].dominantType = "double";
  c17_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c17_info[63].fileTimeLo = 1299076776U;
  c17_info[63].fileTimeHi = 0U;
  c17_info[63].mFileTimeLo = 0U;
  c17_info[63].mFileTimeHi = 0U;
}

static void c17_b_info_helper(c17_ResolvedFunctionInfo c17_info[302])
{
  c17_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c17_info[64].name = "eml_blas_inline";
  c17_info[64].dominantType = "";
  c17_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[64].fileTimeLo = 1299076768U;
  c17_info[64].fileTimeHi = 0U;
  c17_info[64].mFileTimeLo = 0U;
  c17_info[64].mFileTimeHi = 0U;
  c17_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m!below_threshold";
  c17_info[65].name = "length";
  c17_info[65].dominantType = "double";
  c17_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c17_info[65].fileTimeLo = 1303146206U;
  c17_info[65].fileTimeHi = 0U;
  c17_info[65].mFileTimeLo = 0U;
  c17_info[65].mFileTimeHi = 0U;
  c17_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c17_info[66].name = "eml_index_class";
  c17_info[66].dominantType = "";
  c17_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[66].fileTimeLo = 1323170578U;
  c17_info[66].fileTimeHi = 0U;
  c17_info[66].mFileTimeLo = 0U;
  c17_info[66].mFileTimeHi = 0U;
  c17_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c17_info[67].name = "eml_index_class";
  c17_info[67].dominantType = "";
  c17_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[67].fileTimeLo = 1323170578U;
  c17_info[67].fileTimeHi = 0U;
  c17_info[67].mFileTimeLo = 0U;
  c17_info[67].mFileTimeHi = 0U;
  c17_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c17_info[68].name = "eml_refblas_xnrm2";
  c17_info[68].dominantType = "double";
  c17_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c17_info[68].fileTimeLo = 1299076784U;
  c17_info[68].fileTimeHi = 0U;
  c17_info[68].mFileTimeLo = 0U;
  c17_info[68].mFileTimeHi = 0U;
  c17_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c17_info[69].name = "abs";
  c17_info[69].dominantType = "double";
  c17_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[69].fileTimeLo = 1343830366U;
  c17_info[69].fileTimeHi = 0U;
  c17_info[69].mFileTimeLo = 0U;
  c17_info[69].mFileTimeHi = 0U;
  c17_info[70].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[70].name = "eml_scalar_abs";
  c17_info[70].dominantType = "double";
  c17_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c17_info[70].fileTimeLo = 1286818712U;
  c17_info[70].fileTimeHi = 0U;
  c17_info[70].mFileTimeLo = 0U;
  c17_info[70].mFileTimeHi = 0U;
  c17_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c17_info[71].name = "realmin";
  c17_info[71].dominantType = "char";
  c17_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c17_info[71].fileTimeLo = 1307651242U;
  c17_info[71].fileTimeHi = 0U;
  c17_info[71].mFileTimeLo = 0U;
  c17_info[71].mFileTimeHi = 0U;
  c17_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c17_info[72].name = "eml_realmin";
  c17_info[72].dominantType = "char";
  c17_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c17_info[72].fileTimeLo = 1307651244U;
  c17_info[72].fileTimeHi = 0U;
  c17_info[72].mFileTimeLo = 0U;
  c17_info[72].mFileTimeHi = 0U;
  c17_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c17_info[73].name = "eml_float_model";
  c17_info[73].dominantType = "char";
  c17_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c17_info[73].fileTimeLo = 1326727996U;
  c17_info[73].fileTimeHi = 0U;
  c17_info[73].mFileTimeLo = 0U;
  c17_info[73].mFileTimeHi = 0U;
  c17_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c17_info[74].name = "eml_index_class";
  c17_info[74].dominantType = "";
  c17_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[74].fileTimeLo = 1323170578U;
  c17_info[74].fileTimeHi = 0U;
  c17_info[74].mFileTimeLo = 0U;
  c17_info[74].mFileTimeHi = 0U;
  c17_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c17_info[75].name = "eml_index_minus";
  c17_info[75].dominantType = "double";
  c17_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[75].fileTimeLo = 1286818778U;
  c17_info[75].fileTimeHi = 0U;
  c17_info[75].mFileTimeLo = 0U;
  c17_info[75].mFileTimeHi = 0U;
  c17_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c17_info[76].name = "eml_index_times";
  c17_info[76].dominantType = "coder.internal.indexInt";
  c17_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c17_info[76].fileTimeLo = 1286818780U;
  c17_info[76].fileTimeHi = 0U;
  c17_info[76].mFileTimeLo = 0U;
  c17_info[76].mFileTimeHi = 0U;
  c17_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c17_info[77].name = "eml_index_plus";
  c17_info[77].dominantType = "coder.internal.indexInt";
  c17_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[77].fileTimeLo = 1286818778U;
  c17_info[77].fileTimeHi = 0U;
  c17_info[77].mFileTimeLo = 0U;
  c17_info[77].mFileTimeHi = 0U;
  c17_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c17_info[78].name = "eml_int_forloop_overflow_check";
  c17_info[78].dominantType = "";
  c17_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[78].fileTimeLo = 1346510340U;
  c17_info[78].fileTimeHi = 0U;
  c17_info[78].mFileTimeLo = 0U;
  c17_info[78].mFileTimeHi = 0U;
  c17_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[79].name = "eml_div";
  c17_info[79].dominantType = "double";
  c17_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c17_info[79].fileTimeLo = 1313347810U;
  c17_info[79].fileTimeHi = 0U;
  c17_info[79].mFileTimeLo = 0U;
  c17_info[79].mFileTimeHi = 0U;
  c17_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[80].name = "eml_xscal";
  c17_info[80].dominantType = "double";
  c17_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c17_info[80].fileTimeLo = 1299076776U;
  c17_info[80].fileTimeHi = 0U;
  c17_info[80].mFileTimeLo = 0U;
  c17_info[80].mFileTimeHi = 0U;
  c17_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c17_info[81].name = "eml_blas_inline";
  c17_info[81].dominantType = "";
  c17_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[81].fileTimeLo = 1299076768U;
  c17_info[81].fileTimeHi = 0U;
  c17_info[81].mFileTimeLo = 0U;
  c17_info[81].mFileTimeHi = 0U;
  c17_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m!below_threshold";
  c17_info[82].name = "length";
  c17_info[82].dominantType = "double";
  c17_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c17_info[82].fileTimeLo = 1303146206U;
  c17_info[82].fileTimeHi = 0U;
  c17_info[82].mFileTimeLo = 0U;
  c17_info[82].mFileTimeHi = 0U;
  c17_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c17_info[83].name = "eml_index_class";
  c17_info[83].dominantType = "";
  c17_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[83].fileTimeLo = 1323170578U;
  c17_info[83].fileTimeHi = 0U;
  c17_info[83].mFileTimeLo = 0U;
  c17_info[83].mFileTimeHi = 0U;
  c17_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c17_info[84].name = "eml_scalar_eg";
  c17_info[84].dominantType = "double";
  c17_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[84].fileTimeLo = 1286818796U;
  c17_info[84].fileTimeHi = 0U;
  c17_info[84].mFileTimeLo = 0U;
  c17_info[84].mFileTimeHi = 0U;
  c17_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c17_info[85].name = "eml_refblas_xscal";
  c17_info[85].dominantType = "double";
  c17_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c17_info[85].fileTimeLo = 1299076784U;
  c17_info[85].fileTimeHi = 0U;
  c17_info[85].mFileTimeLo = 0U;
  c17_info[85].mFileTimeHi = 0U;
  c17_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c17_info[86].name = "eml_index_class";
  c17_info[86].dominantType = "";
  c17_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[86].fileTimeLo = 1323170578U;
  c17_info[86].fileTimeHi = 0U;
  c17_info[86].mFileTimeLo = 0U;
  c17_info[86].mFileTimeHi = 0U;
  c17_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c17_info[87].name = "eml_index_minus";
  c17_info[87].dominantType = "double";
  c17_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[87].fileTimeLo = 1286818778U;
  c17_info[87].fileTimeHi = 0U;
  c17_info[87].mFileTimeLo = 0U;
  c17_info[87].mFileTimeHi = 0U;
  c17_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c17_info[88].name = "eml_index_times";
  c17_info[88].dominantType = "coder.internal.indexInt";
  c17_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c17_info[88].fileTimeLo = 1286818780U;
  c17_info[88].fileTimeHi = 0U;
  c17_info[88].mFileTimeLo = 0U;
  c17_info[88].mFileTimeHi = 0U;
  c17_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c17_info[89].name = "eml_index_plus";
  c17_info[89].dominantType = "coder.internal.indexInt";
  c17_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[89].fileTimeLo = 1286818778U;
  c17_info[89].fileTimeHi = 0U;
  c17_info[89].mFileTimeLo = 0U;
  c17_info[89].mFileTimeHi = 0U;
  c17_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c17_info[90].name = "eml_int_forloop_overflow_check";
  c17_info[90].dominantType = "";
  c17_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[90].fileTimeLo = 1346510340U;
  c17_info[90].fileTimeHi = 0U;
  c17_info[90].mFileTimeLo = 0U;
  c17_info[90].mFileTimeHi = 0U;
  c17_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[91].name = "eml_xdotc";
  c17_info[91].dominantType = "double";
  c17_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c17_info[91].fileTimeLo = 1299076772U;
  c17_info[91].fileTimeHi = 0U;
  c17_info[91].mFileTimeLo = 0U;
  c17_info[91].mFileTimeHi = 0U;
  c17_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c17_info[92].name = "eml_blas_inline";
  c17_info[92].dominantType = "";
  c17_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[92].fileTimeLo = 1299076768U;
  c17_info[92].fileTimeHi = 0U;
  c17_info[92].mFileTimeLo = 0U;
  c17_info[92].mFileTimeHi = 0U;
  c17_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c17_info[93].name = "eml_xdot";
  c17_info[93].dominantType = "double";
  c17_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c17_info[93].fileTimeLo = 1299076772U;
  c17_info[93].fileTimeHi = 0U;
  c17_info[93].mFileTimeLo = 0U;
  c17_info[93].mFileTimeHi = 0U;
  c17_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c17_info[94].name = "eml_blas_inline";
  c17_info[94].dominantType = "";
  c17_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[94].fileTimeLo = 1299076768U;
  c17_info[94].fileTimeHi = 0U;
  c17_info[94].mFileTimeLo = 0U;
  c17_info[94].mFileTimeHi = 0U;
  c17_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m!below_threshold";
  c17_info[95].name = "length";
  c17_info[95].dominantType = "double";
  c17_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c17_info[95].fileTimeLo = 1303146206U;
  c17_info[95].fileTimeHi = 0U;
  c17_info[95].mFileTimeLo = 0U;
  c17_info[95].mFileTimeHi = 0U;
  c17_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c17_info[96].name = "eml_index_class";
  c17_info[96].dominantType = "";
  c17_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[96].fileTimeLo = 1323170578U;
  c17_info[96].fileTimeHi = 0U;
  c17_info[96].mFileTimeLo = 0U;
  c17_info[96].mFileTimeHi = 0U;
  c17_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c17_info[97].name = "eml_refblas_xdot";
  c17_info[97].dominantType = "double";
  c17_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c17_info[97].fileTimeLo = 1299076772U;
  c17_info[97].fileTimeHi = 0U;
  c17_info[97].mFileTimeLo = 0U;
  c17_info[97].mFileTimeHi = 0U;
  c17_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c17_info[98].name = "eml_refblas_xdotx";
  c17_info[98].dominantType = "char";
  c17_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c17_info[98].fileTimeLo = 1299076774U;
  c17_info[98].fileTimeHi = 0U;
  c17_info[98].mFileTimeLo = 0U;
  c17_info[98].mFileTimeHi = 0U;
  c17_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c17_info[99].name = "eml_scalar_eg";
  c17_info[99].dominantType = "double";
  c17_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[99].fileTimeLo = 1286818796U;
  c17_info[99].fileTimeHi = 0U;
  c17_info[99].mFileTimeLo = 0U;
  c17_info[99].mFileTimeHi = 0U;
  c17_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c17_info[100].name = "eml_index_class";
  c17_info[100].dominantType = "";
  c17_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[100].fileTimeLo = 1323170578U;
  c17_info[100].fileTimeHi = 0U;
  c17_info[100].mFileTimeLo = 0U;
  c17_info[100].mFileTimeHi = 0U;
  c17_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c17_info[101].name = "eml_int_forloop_overflow_check";
  c17_info[101].dominantType = "";
  c17_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[101].fileTimeLo = 1346510340U;
  c17_info[101].fileTimeHi = 0U;
  c17_info[101].mFileTimeLo = 0U;
  c17_info[101].mFileTimeHi = 0U;
  c17_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c17_info[102].name = "eml_index_plus";
  c17_info[102].dominantType = "coder.internal.indexInt";
  c17_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[102].fileTimeLo = 1286818778U;
  c17_info[102].fileTimeHi = 0U;
  c17_info[102].mFileTimeLo = 0U;
  c17_info[102].mFileTimeHi = 0U;
  c17_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[103].name = "eml_xaxpy";
  c17_info[103].dominantType = "double";
  c17_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c17_info[103].fileTimeLo = 1299076770U;
  c17_info[103].fileTimeHi = 0U;
  c17_info[103].mFileTimeLo = 0U;
  c17_info[103].mFileTimeHi = 0U;
  c17_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c17_info[104].name = "eml_blas_inline";
  c17_info[104].dominantType = "";
  c17_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[104].fileTimeLo = 1299076768U;
  c17_info[104].fileTimeHi = 0U;
  c17_info[104].mFileTimeLo = 0U;
  c17_info[104].mFileTimeHi = 0U;
  c17_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m!below_threshold";
  c17_info[105].name = "length";
  c17_info[105].dominantType = "double";
  c17_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c17_info[105].fileTimeLo = 1303146206U;
  c17_info[105].fileTimeHi = 0U;
  c17_info[105].mFileTimeLo = 0U;
  c17_info[105].mFileTimeHi = 0U;
  c17_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c17_info[106].name = "eml_index_class";
  c17_info[106].dominantType = "";
  c17_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[106].fileTimeLo = 1323170578U;
  c17_info[106].fileTimeHi = 0U;
  c17_info[106].mFileTimeLo = 0U;
  c17_info[106].mFileTimeHi = 0U;
  c17_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c17_info[107].name = "eml_scalar_eg";
  c17_info[107].dominantType = "double";
  c17_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[107].fileTimeLo = 1286818796U;
  c17_info[107].fileTimeHi = 0U;
  c17_info[107].mFileTimeLo = 0U;
  c17_info[107].mFileTimeHi = 0U;
  c17_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c17_info[108].name = "eml_refblas_xaxpy";
  c17_info[108].dominantType = "double";
  c17_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c17_info[108].fileTimeLo = 1299076772U;
  c17_info[108].fileTimeHi = 0U;
  c17_info[108].mFileTimeLo = 0U;
  c17_info[108].mFileTimeHi = 0U;
  c17_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c17_info[109].name = "eml_index_class";
  c17_info[109].dominantType = "";
  c17_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[109].fileTimeLo = 1323170578U;
  c17_info[109].fileTimeHi = 0U;
  c17_info[109].mFileTimeLo = 0U;
  c17_info[109].mFileTimeHi = 0U;
  c17_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c17_info[110].name = "eml_isa_uint";
  c17_info[110].dominantType = "coder.internal.indexInt";
  c17_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c17_info[110].fileTimeLo = 1286818784U;
  c17_info[110].fileTimeHi = 0U;
  c17_info[110].mFileTimeLo = 0U;
  c17_info[110].mFileTimeHi = 0U;
  c17_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c17_info[111].name = "eml_index_minus";
  c17_info[111].dominantType = "double";
  c17_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[111].fileTimeLo = 1286818778U;
  c17_info[111].fileTimeHi = 0U;
  c17_info[111].mFileTimeLo = 0U;
  c17_info[111].mFileTimeHi = 0U;
  c17_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c17_info[112].name = "eml_int_forloop_overflow_check";
  c17_info[112].dominantType = "";
  c17_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[112].fileTimeLo = 1346510340U;
  c17_info[112].fileTimeHi = 0U;
  c17_info[112].mFileTimeLo = 0U;
  c17_info[112].mFileTimeHi = 0U;
  c17_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c17_info[113].name = "eml_index_plus";
  c17_info[113].dominantType = "double";
  c17_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[113].fileTimeLo = 1286818778U;
  c17_info[113].fileTimeHi = 0U;
  c17_info[113].mFileTimeLo = 0U;
  c17_info[113].mFileTimeHi = 0U;
  c17_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c17_info[114].name = "eml_index_plus";
  c17_info[114].dominantType = "coder.internal.indexInt";
  c17_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[114].fileTimeLo = 1286818778U;
  c17_info[114].fileTimeHi = 0U;
  c17_info[114].mFileTimeLo = 0U;
  c17_info[114].mFileTimeHi = 0U;
  c17_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c17_info[115].name = "intmin";
  c17_info[115].dominantType = "char";
  c17_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c17_info[115].fileTimeLo = 1311255318U;
  c17_info[115].fileTimeHi = 0U;
  c17_info[115].mFileTimeLo = 0U;
  c17_info[115].mFileTimeHi = 0U;
  c17_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[116].name = "abs";
  c17_info[116].dominantType = "double";
  c17_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[116].fileTimeLo = 1343830366U;
  c17_info[116].fileTimeHi = 0U;
  c17_info[116].mFileTimeLo = 0U;
  c17_info[116].mFileTimeHi = 0U;
  c17_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[117].name = "mtimes";
  c17_info[117].dominantType = "double";
  c17_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[117].fileTimeLo = 1289519692U;
  c17_info[117].fileTimeHi = 0U;
  c17_info[117].mFileTimeLo = 0U;
  c17_info[117].mFileTimeHi = 0U;
  c17_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[118].name = "realmin";
  c17_info[118].dominantType = "char";
  c17_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c17_info[118].fileTimeLo = 1307651242U;
  c17_info[118].fileTimeHi = 0U;
  c17_info[118].mFileTimeLo = 0U;
  c17_info[118].mFileTimeHi = 0U;
  c17_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[119].name = "eps";
  c17_info[119].dominantType = "char";
  c17_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c17_info[119].fileTimeLo = 1326727996U;
  c17_info[119].fileTimeHi = 0U;
  c17_info[119].mFileTimeLo = 0U;
  c17_info[119].mFileTimeHi = 0U;
  c17_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c17_info[120].name = "eml_is_float_class";
  c17_info[120].dominantType = "char";
  c17_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c17_info[120].fileTimeLo = 1286818782U;
  c17_info[120].fileTimeHi = 0U;
  c17_info[120].mFileTimeLo = 0U;
  c17_info[120].mFileTimeHi = 0U;
  c17_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c17_info[121].name = "eml_eps";
  c17_info[121].dominantType = "char";
  c17_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c17_info[121].fileTimeLo = 1326727996U;
  c17_info[121].fileTimeHi = 0U;
  c17_info[121].mFileTimeLo = 0U;
  c17_info[121].mFileTimeHi = 0U;
  c17_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c17_info[122].name = "eml_float_model";
  c17_info[122].dominantType = "char";
  c17_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c17_info[122].fileTimeLo = 1326727996U;
  c17_info[122].fileTimeHi = 0U;
  c17_info[122].mFileTimeLo = 0U;
  c17_info[122].mFileTimeHi = 0U;
  c17_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[123].name = "eml_error";
  c17_info[123].dominantType = "char";
  c17_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c17_info[123].fileTimeLo = 1343830358U;
  c17_info[123].fileTimeHi = 0U;
  c17_info[123].mFileTimeLo = 0U;
  c17_info[123].mFileTimeHi = 0U;
  c17_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c17_info[124].name = "eml_const_nonsingleton_dim";
  c17_info[124].dominantType = "double";
  c17_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c17_info[124].fileTimeLo = 1286818696U;
  c17_info[124].fileTimeHi = 0U;
  c17_info[124].mFileTimeLo = 0U;
  c17_info[124].mFileTimeHi = 0U;
  c17_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c17_info[125].name = "eml_scalar_eg";
  c17_info[125].dominantType = "double";
  c17_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[125].fileTimeLo = 1286818796U;
  c17_info[125].fileTimeHi = 0U;
  c17_info[125].mFileTimeLo = 0U;
  c17_info[125].mFileTimeHi = 0U;
  c17_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c17_info[126].name = "eml_index_class";
  c17_info[126].dominantType = "";
  c17_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[126].fileTimeLo = 1323170578U;
  c17_info[126].fileTimeHi = 0U;
  c17_info[126].mFileTimeLo = 0U;
  c17_info[126].mFileTimeHi = 0U;
  c17_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c17_info[127].name = "eml_index_class";
  c17_info[127].dominantType = "";
  c17_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[127].fileTimeLo = 1323170578U;
  c17_info[127].fileTimeHi = 0U;
  c17_info[127].mFileTimeLo = 0U;
  c17_info[127].mFileTimeHi = 0U;
}

static void c17_c_info_helper(c17_ResolvedFunctionInfo c17_info[302])
{
  c17_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c17_info[128].name = "isnan";
  c17_info[128].dominantType = "double";
  c17_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c17_info[128].fileTimeLo = 1286818760U;
  c17_info[128].fileTimeHi = 0U;
  c17_info[128].mFileTimeLo = 0U;
  c17_info[128].mFileTimeHi = 0U;
  c17_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c17_info[129].name = "eml_index_plus";
  c17_info[129].dominantType = "coder.internal.indexInt";
  c17_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[129].fileTimeLo = 1286818778U;
  c17_info[129].fileTimeHi = 0U;
  c17_info[129].mFileTimeLo = 0U;
  c17_info[129].mFileTimeHi = 0U;
  c17_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c17_info[130].name = "eml_int_forloop_overflow_check";
  c17_info[130].dominantType = "";
  c17_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[130].fileTimeLo = 1346510340U;
  c17_info[130].fileTimeHi = 0U;
  c17_info[130].mFileTimeLo = 0U;
  c17_info[130].mFileTimeHi = 0U;
  c17_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c17_info[131].name = "eml_relop";
  c17_info[131].dominantType = "function_handle";
  c17_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c17_info[131].fileTimeLo = 1342451182U;
  c17_info[131].fileTimeHi = 0U;
  c17_info[131].mFileTimeLo = 0U;
  c17_info[131].mFileTimeHi = 0U;
  c17_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[132].name = "sqrt";
  c17_info[132].dominantType = "double";
  c17_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c17_info[132].fileTimeLo = 1343830386U;
  c17_info[132].fileTimeHi = 0U;
  c17_info[132].mFileTimeLo = 0U;
  c17_info[132].mFileTimeHi = 0U;
  c17_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c17_info[133].name = "eml_error";
  c17_info[133].dominantType = "char";
  c17_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c17_info[133].fileTimeLo = 1343830358U;
  c17_info[133].fileTimeHi = 0U;
  c17_info[133].mFileTimeLo = 0U;
  c17_info[133].mFileTimeHi = 0U;
  c17_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c17_info[134].name = "eml_scalar_sqrt";
  c17_info[134].dominantType = "double";
  c17_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c17_info[134].fileTimeLo = 1286818738U;
  c17_info[134].fileTimeHi = 0U;
  c17_info[134].mFileTimeLo = 0U;
  c17_info[134].mFileTimeHi = 0U;
  c17_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[135].name = "eml_xrotg";
  c17_info[135].dominantType = "double";
  c17_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c17_info[135].fileTimeLo = 1299076776U;
  c17_info[135].fileTimeHi = 0U;
  c17_info[135].mFileTimeLo = 0U;
  c17_info[135].mFileTimeHi = 0U;
  c17_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c17_info[136].name = "eml_blas_inline";
  c17_info[136].dominantType = "";
  c17_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[136].fileTimeLo = 1299076768U;
  c17_info[136].fileTimeHi = 0U;
  c17_info[136].mFileTimeLo = 0U;
  c17_info[136].mFileTimeHi = 0U;
  c17_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m";
  c17_info[137].name = "eml_refblas_xrotg";
  c17_info[137].dominantType = "double";
  c17_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c17_info[137].fileTimeLo = 1299076784U;
  c17_info[137].fileTimeHi = 0U;
  c17_info[137].mFileTimeLo = 0U;
  c17_info[137].mFileTimeHi = 0U;
  c17_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c17_info[138].name = "abs";
  c17_info[138].dominantType = "double";
  c17_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[138].fileTimeLo = 1343830366U;
  c17_info[138].fileTimeHi = 0U;
  c17_info[138].mFileTimeLo = 0U;
  c17_info[138].mFileTimeHi = 0U;
  c17_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c17_info[139].name = "mrdivide";
  c17_info[139].dominantType = "double";
  c17_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c17_info[139].fileTimeLo = 1357951548U;
  c17_info[139].fileTimeHi = 0U;
  c17_info[139].mFileTimeLo = 1319729966U;
  c17_info[139].mFileTimeHi = 0U;
  c17_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c17_info[140].name = "rdivide";
  c17_info[140].dominantType = "double";
  c17_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c17_info[140].fileTimeLo = 1346510388U;
  c17_info[140].fileTimeHi = 0U;
  c17_info[140].mFileTimeLo = 0U;
  c17_info[140].mFileTimeHi = 0U;
  c17_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c17_info[141].name = "eml_scalexp_compatible";
  c17_info[141].dominantType = "double";
  c17_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c17_info[141].fileTimeLo = 1286818796U;
  c17_info[141].fileTimeHi = 0U;
  c17_info[141].mFileTimeLo = 0U;
  c17_info[141].mFileTimeHi = 0U;
  c17_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c17_info[142].name = "eml_div";
  c17_info[142].dominantType = "double";
  c17_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c17_info[142].fileTimeLo = 1313347810U;
  c17_info[142].fileTimeHi = 0U;
  c17_info[142].mFileTimeLo = 0U;
  c17_info[142].mFileTimeHi = 0U;
  c17_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c17_info[143].name = "sqrt";
  c17_info[143].dominantType = "double";
  c17_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c17_info[143].fileTimeLo = 1343830386U;
  c17_info[143].fileTimeHi = 0U;
  c17_info[143].mFileTimeLo = 0U;
  c17_info[143].mFileTimeHi = 0U;
  c17_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m!eml_ceval_xrotg";
  c17_info[144].name = "eml_scalar_eg";
  c17_info[144].dominantType = "double";
  c17_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[144].fileTimeLo = 1286818796U;
  c17_info[144].fileTimeHi = 0U;
  c17_info[144].mFileTimeLo = 0U;
  c17_info[144].mFileTimeHi = 0U;
  c17_info[145].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[145].name = "eml_xrot";
  c17_info[145].dominantType = "double";
  c17_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m";
  c17_info[145].fileTimeLo = 1299076776U;
  c17_info[145].fileTimeHi = 0U;
  c17_info[145].mFileTimeLo = 0U;
  c17_info[145].mFileTimeHi = 0U;
  c17_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m";
  c17_info[146].name = "eml_blas_inline";
  c17_info[146].dominantType = "";
  c17_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[146].fileTimeLo = 1299076768U;
  c17_info[146].fileTimeHi = 0U;
  c17_info[146].mFileTimeLo = 0U;
  c17_info[146].mFileTimeHi = 0U;
  c17_info[147].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c17_info[147].name = "eml_index_class";
  c17_info[147].dominantType = "";
  c17_info[147].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[147].fileTimeLo = 1323170578U;
  c17_info[147].fileTimeHi = 0U;
  c17_info[147].mFileTimeLo = 0U;
  c17_info[147].mFileTimeHi = 0U;
  c17_info[148].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c17_info[148].name = "eml_scalar_eg";
  c17_info[148].dominantType = "double";
  c17_info[148].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[148].fileTimeLo = 1286818796U;
  c17_info[148].fileTimeHi = 0U;
  c17_info[148].mFileTimeLo = 0U;
  c17_info[148].mFileTimeHi = 0U;
  c17_info[149].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c17_info[149].name = "eml_refblas_xrot";
  c17_info[149].dominantType = "double";
  c17_info[149].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c17_info[149].fileTimeLo = 1299076784U;
  c17_info[149].fileTimeHi = 0U;
  c17_info[149].mFileTimeLo = 0U;
  c17_info[149].mFileTimeHi = 0U;
  c17_info[150].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c17_info[150].name = "eml_index_class";
  c17_info[150].dominantType = "";
  c17_info[150].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[150].fileTimeLo = 1323170578U;
  c17_info[150].fileTimeHi = 0U;
  c17_info[150].mFileTimeLo = 0U;
  c17_info[150].mFileTimeHi = 0U;
  c17_info[151].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c17_info[151].name = "eml_int_forloop_overflow_check";
  c17_info[151].dominantType = "";
  c17_info[151].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[151].fileTimeLo = 1346510340U;
  c17_info[151].fileTimeHi = 0U;
  c17_info[151].mFileTimeLo = 0U;
  c17_info[151].mFileTimeHi = 0U;
  c17_info[152].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c17_info[152].name = "mtimes";
  c17_info[152].dominantType = "double";
  c17_info[152].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[152].fileTimeLo = 1289519692U;
  c17_info[152].fileTimeHi = 0U;
  c17_info[152].mFileTimeLo = 0U;
  c17_info[152].mFileTimeHi = 0U;
  c17_info[153].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c17_info[153].name = "eml_index_plus";
  c17_info[153].dominantType = "coder.internal.indexInt";
  c17_info[153].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[153].fileTimeLo = 1286818778U;
  c17_info[153].fileTimeHi = 0U;
  c17_info[153].mFileTimeLo = 0U;
  c17_info[153].mFileTimeHi = 0U;
  c17_info[154].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c17_info[154].name = "eml_xswap";
  c17_info[154].dominantType = "double";
  c17_info[154].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c17_info[154].fileTimeLo = 1299076778U;
  c17_info[154].fileTimeHi = 0U;
  c17_info[154].mFileTimeLo = 0U;
  c17_info[154].mFileTimeHi = 0U;
  c17_info[155].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c17_info[155].name = "eml_blas_inline";
  c17_info[155].dominantType = "";
  c17_info[155].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[155].fileTimeLo = 1299076768U;
  c17_info[155].fileTimeHi = 0U;
  c17_info[155].mFileTimeLo = 0U;
  c17_info[155].mFileTimeHi = 0U;
  c17_info[156].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c17_info[156].name = "eml_index_class";
  c17_info[156].dominantType = "";
  c17_info[156].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[156].fileTimeLo = 1323170578U;
  c17_info[156].fileTimeHi = 0U;
  c17_info[156].mFileTimeLo = 0U;
  c17_info[156].mFileTimeHi = 0U;
  c17_info[157].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c17_info[157].name = "eml_refblas_xswap";
  c17_info[157].dominantType = "double";
  c17_info[157].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c17_info[157].fileTimeLo = 1299076786U;
  c17_info[157].fileTimeHi = 0U;
  c17_info[157].mFileTimeLo = 0U;
  c17_info[157].mFileTimeHi = 0U;
  c17_info[158].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c17_info[158].name = "eml_index_class";
  c17_info[158].dominantType = "";
  c17_info[158].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[158].fileTimeLo = 1323170578U;
  c17_info[158].fileTimeHi = 0U;
  c17_info[158].mFileTimeLo = 0U;
  c17_info[158].mFileTimeHi = 0U;
  c17_info[159].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c17_info[159].name = "abs";
  c17_info[159].dominantType = "coder.internal.indexInt";
  c17_info[159].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[159].fileTimeLo = 1343830366U;
  c17_info[159].fileTimeHi = 0U;
  c17_info[159].mFileTimeLo = 0U;
  c17_info[159].mFileTimeHi = 0U;
  c17_info[160].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[160].name = "eml_scalar_abs";
  c17_info[160].dominantType = "coder.internal.indexInt";
  c17_info[160].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c17_info[160].fileTimeLo = 1286818712U;
  c17_info[160].fileTimeHi = 0U;
  c17_info[160].mFileTimeLo = 0U;
  c17_info[160].mFileTimeHi = 0U;
  c17_info[161].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c17_info[161].name = "eml_int_forloop_overflow_check";
  c17_info[161].dominantType = "";
  c17_info[161].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[161].fileTimeLo = 1346510340U;
  c17_info[161].fileTimeHi = 0U;
  c17_info[161].mFileTimeLo = 0U;
  c17_info[161].mFileTimeHi = 0U;
  c17_info[162].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c17_info[162].name = "eml_index_plus";
  c17_info[162].dominantType = "coder.internal.indexInt";
  c17_info[162].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[162].fileTimeLo = 1286818778U;
  c17_info[162].fileTimeHi = 0U;
  c17_info[162].mFileTimeLo = 0U;
  c17_info[162].mFileTimeHi = 0U;
  c17_info[163].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[163].name = "eml_int_forloop_overflow_check";
  c17_info[163].dominantType = "";
  c17_info[163].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[163].fileTimeLo = 1346510340U;
  c17_info[163].fileTimeHi = 0U;
  c17_info[163].mFileTimeLo = 0U;
  c17_info[163].mFileTimeHi = 0U;
  c17_info[164].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[164].name = "eml_index_plus";
  c17_info[164].dominantType = "double";
  c17_info[164].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[164].fileTimeLo = 1286818778U;
  c17_info[164].fileTimeHi = 0U;
  c17_info[164].mFileTimeLo = 0U;
  c17_info[164].mFileTimeHi = 0U;
  c17_info[165].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[165].name = "eml_div";
  c17_info[165].dominantType = "double";
  c17_info[165].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c17_info[165].fileTimeLo = 1313347810U;
  c17_info[165].fileTimeHi = 0U;
  c17_info[165].mFileTimeLo = 0U;
  c17_info[165].mFileTimeHi = 0U;
  c17_info[166].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[166].name = "eml_xscal";
  c17_info[166].dominantType = "double";
  c17_info[166].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c17_info[166].fileTimeLo = 1299076776U;
  c17_info[166].fileTimeHi = 0U;
  c17_info[166].mFileTimeLo = 0U;
  c17_info[166].mFileTimeHi = 0U;
  c17_info[167].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[167].name = "eml_index_plus";
  c17_info[167].dominantType = "coder.internal.indexInt";
  c17_info[167].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[167].fileTimeLo = 1286818778U;
  c17_info[167].fileTimeHi = 0U;
  c17_info[167].mFileTimeLo = 0U;
  c17_info[167].mFileTimeHi = 0U;
  c17_info[168].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c17_info[168].name = "eml_xgemm";
  c17_info[168].dominantType = "char";
  c17_info[168].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c17_info[168].fileTimeLo = 1299076772U;
  c17_info[168].fileTimeHi = 0U;
  c17_info[168].mFileTimeLo = 0U;
  c17_info[168].mFileTimeHi = 0U;
  c17_info[169].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c17_info[169].name = "min";
  c17_info[169].dominantType = "double";
  c17_info[169].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c17_info[169].fileTimeLo = 1311255318U;
  c17_info[169].fileTimeHi = 0U;
  c17_info[169].mFileTimeLo = 0U;
  c17_info[169].mFileTimeHi = 0U;
  c17_info[170].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c17_info[170].name = "eml_index_minus";
  c17_info[170].dominantType = "double";
  c17_info[170].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[170].fileTimeLo = 1286818778U;
  c17_info[170].fileTimeHi = 0U;
  c17_info[170].mFileTimeLo = 0U;
  c17_info[170].mFileTimeHi = 0U;
  c17_info[171].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c17_info[171].name = "eml_index_class";
  c17_info[171].dominantType = "";
  c17_info[171].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[171].fileTimeLo = 1323170578U;
  c17_info[171].fileTimeHi = 0U;
  c17_info[171].mFileTimeLo = 0U;
  c17_info[171].mFileTimeHi = 0U;
  c17_info[172].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c17_info[172].name = "eml_scalar_eg";
  c17_info[172].dominantType = "double";
  c17_info[172].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[172].fileTimeLo = 1286818796U;
  c17_info[172].fileTimeHi = 0U;
  c17_info[172].mFileTimeLo = 0U;
  c17_info[172].mFileTimeHi = 0U;
  c17_info[173].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c17_info[173].name = "eml_index_times";
  c17_info[173].dominantType = "coder.internal.indexInt";
  c17_info[173].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c17_info[173].fileTimeLo = 1286818780U;
  c17_info[173].fileTimeHi = 0U;
  c17_info[173].mFileTimeLo = 0U;
  c17_info[173].mFileTimeHi = 0U;
  c17_info[174].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c17_info[174].name = "eml_index_plus";
  c17_info[174].dominantType = "coder.internal.indexInt";
  c17_info[174].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[174].fileTimeLo = 1286818778U;
  c17_info[174].fileTimeHi = 0U;
  c17_info[174].mFileTimeLo = 0U;
  c17_info[174].mFileTimeHi = 0U;
  c17_info[175].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c17_info[175].name = "eml_int_forloop_overflow_check";
  c17_info[175].dominantType = "";
  c17_info[175].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[175].fileTimeLo = 1346510340U;
  c17_info[175].fileTimeHi = 0U;
  c17_info[175].mFileTimeLo = 0U;
  c17_info[175].mFileTimeHi = 0U;
  c17_info[176].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c17_info[176].name = "eml_index_plus";
  c17_info[176].dominantType = "double";
  c17_info[176].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[176].fileTimeLo = 1286818778U;
  c17_info[176].fileTimeHi = 0U;
  c17_info[176].mFileTimeLo = 0U;
  c17_info[176].mFileTimeHi = 0U;
  c17_info[177].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[177].name = "inv";
  c17_info[177].dominantType = "double";
  c17_info[177].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m";
  c17_info[177].fileTimeLo = 1305318000U;
  c17_info[177].fileTimeHi = 0U;
  c17_info[177].mFileTimeLo = 0U;
  c17_info[177].mFileTimeHi = 0U;
  c17_info[178].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c17_info[178].name = "eml_index_class";
  c17_info[178].dominantType = "";
  c17_info[178].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[178].fileTimeLo = 1323170578U;
  c17_info[178].fileTimeHi = 0U;
  c17_info[178].mFileTimeLo = 0U;
  c17_info[178].mFileTimeHi = 0U;
  c17_info[179].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c17_info[179].name = "eml_xgetrf";
  c17_info[179].dominantType = "double";
  c17_info[179].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c17_info[179].fileTimeLo = 1286818806U;
  c17_info[179].fileTimeHi = 0U;
  c17_info[179].mFileTimeLo = 0U;
  c17_info[179].mFileTimeHi = 0U;
  c17_info[180].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c17_info[180].name = "eml_lapack_xgetrf";
  c17_info[180].dominantType = "double";
  c17_info[180].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c17_info[180].fileTimeLo = 1286818810U;
  c17_info[180].fileTimeHi = 0U;
  c17_info[180].mFileTimeLo = 0U;
  c17_info[180].mFileTimeHi = 0U;
  c17_info[181].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c17_info[181].name = "eml_matlab_zgetrf";
  c17_info[181].dominantType = "double";
  c17_info[181].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[181].fileTimeLo = 1302688994U;
  c17_info[181].fileTimeHi = 0U;
  c17_info[181].mFileTimeLo = 0U;
  c17_info[181].mFileTimeHi = 0U;
  c17_info[182].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[182].name = "realmin";
  c17_info[182].dominantType = "char";
  c17_info[182].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c17_info[182].fileTimeLo = 1307651242U;
  c17_info[182].fileTimeHi = 0U;
  c17_info[182].mFileTimeLo = 0U;
  c17_info[182].mFileTimeHi = 0U;
  c17_info[183].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[183].name = "eps";
  c17_info[183].dominantType = "char";
  c17_info[183].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c17_info[183].fileTimeLo = 1326727996U;
  c17_info[183].fileTimeHi = 0U;
  c17_info[183].mFileTimeLo = 0U;
  c17_info[183].mFileTimeHi = 0U;
  c17_info[184].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[184].name = "min";
  c17_info[184].dominantType = "coder.internal.indexInt";
  c17_info[184].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c17_info[184].fileTimeLo = 1311255318U;
  c17_info[184].fileTimeHi = 0U;
  c17_info[184].mFileTimeLo = 0U;
  c17_info[184].mFileTimeHi = 0U;
  c17_info[185].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[185].name = "colon";
  c17_info[185].dominantType = "double";
  c17_info[185].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c17_info[185].fileTimeLo = 1348191928U;
  c17_info[185].fileTimeHi = 0U;
  c17_info[185].mFileTimeLo = 0U;
  c17_info[185].mFileTimeHi = 0U;
  c17_info[186].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c17_info[186].name = "colon";
  c17_info[186].dominantType = "double";
  c17_info[186].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c17_info[186].fileTimeLo = 1348191928U;
  c17_info[186].fileTimeHi = 0U;
  c17_info[186].mFileTimeLo = 0U;
  c17_info[186].mFileTimeHi = 0U;
  c17_info[187].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c17_info[187].name = "floor";
  c17_info[187].dominantType = "double";
  c17_info[187].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c17_info[187].fileTimeLo = 1343830380U;
  c17_info[187].fileTimeHi = 0U;
  c17_info[187].mFileTimeLo = 0U;
  c17_info[187].mFileTimeHi = 0U;
  c17_info[188].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c17_info[188].name = "eml_scalar_floor";
  c17_info[188].dominantType = "double";
  c17_info[188].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c17_info[188].fileTimeLo = 1286818726U;
  c17_info[188].fileTimeHi = 0U;
  c17_info[188].mFileTimeLo = 0U;
  c17_info[188].mFileTimeHi = 0U;
  c17_info[189].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c17_info[189].name = "intmin";
  c17_info[189].dominantType = "char";
  c17_info[189].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c17_info[189].fileTimeLo = 1311255318U;
  c17_info[189].fileTimeHi = 0U;
  c17_info[189].mFileTimeLo = 0U;
  c17_info[189].mFileTimeHi = 0U;
  c17_info[190].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c17_info[190].name = "intmax";
  c17_info[190].dominantType = "char";
  c17_info[190].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c17_info[190].fileTimeLo = 1311255316U;
  c17_info[190].fileTimeHi = 0U;
  c17_info[190].mFileTimeLo = 0U;
  c17_info[190].mFileTimeHi = 0U;
  c17_info[191].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c17_info[191].name = "intmin";
  c17_info[191].dominantType = "char";
  c17_info[191].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c17_info[191].fileTimeLo = 1311255318U;
  c17_info[191].fileTimeHi = 0U;
  c17_info[191].mFileTimeLo = 0U;
  c17_info[191].mFileTimeHi = 0U;
}

static void c17_d_info_helper(c17_ResolvedFunctionInfo c17_info[302])
{
  c17_info[192].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c17_info[192].name = "intmax";
  c17_info[192].dominantType = "char";
  c17_info[192].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c17_info[192].fileTimeLo = 1311255316U;
  c17_info[192].fileTimeHi = 0U;
  c17_info[192].mFileTimeLo = 0U;
  c17_info[192].mFileTimeHi = 0U;
  c17_info[193].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c17_info[193].name = "eml_isa_uint";
  c17_info[193].dominantType = "coder.internal.indexInt";
  c17_info[193].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c17_info[193].fileTimeLo = 1286818784U;
  c17_info[193].fileTimeHi = 0U;
  c17_info[193].mFileTimeLo = 0U;
  c17_info[193].mFileTimeHi = 0U;
  c17_info[194].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c17_info[194].name = "eml_unsigned_class";
  c17_info[194].dominantType = "char";
  c17_info[194].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c17_info[194].fileTimeLo = 1323170580U;
  c17_info[194].fileTimeHi = 0U;
  c17_info[194].mFileTimeLo = 0U;
  c17_info[194].mFileTimeHi = 0U;
  c17_info[195].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c17_info[195].name = "eml_index_class";
  c17_info[195].dominantType = "";
  c17_info[195].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[195].fileTimeLo = 1323170578U;
  c17_info[195].fileTimeHi = 0U;
  c17_info[195].mFileTimeLo = 0U;
  c17_info[195].mFileTimeHi = 0U;
  c17_info[196].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c17_info[196].name = "eml_index_class";
  c17_info[196].dominantType = "";
  c17_info[196].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[196].fileTimeLo = 1323170578U;
  c17_info[196].fileTimeHi = 0U;
  c17_info[196].mFileTimeLo = 0U;
  c17_info[196].mFileTimeHi = 0U;
  c17_info[197].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c17_info[197].name = "intmax";
  c17_info[197].dominantType = "char";
  c17_info[197].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c17_info[197].fileTimeLo = 1311255316U;
  c17_info[197].fileTimeHi = 0U;
  c17_info[197].mFileTimeLo = 0U;
  c17_info[197].mFileTimeHi = 0U;
  c17_info[198].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c17_info[198].name = "eml_isa_uint";
  c17_info[198].dominantType = "coder.internal.indexInt";
  c17_info[198].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c17_info[198].fileTimeLo = 1286818784U;
  c17_info[198].fileTimeHi = 0U;
  c17_info[198].mFileTimeLo = 0U;
  c17_info[198].mFileTimeHi = 0U;
  c17_info[199].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c17_info[199].name = "eml_index_plus";
  c17_info[199].dominantType = "double";
  c17_info[199].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[199].fileTimeLo = 1286818778U;
  c17_info[199].fileTimeHi = 0U;
  c17_info[199].mFileTimeLo = 0U;
  c17_info[199].mFileTimeHi = 0U;
  c17_info[200].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c17_info[200].name = "eml_int_forloop_overflow_check";
  c17_info[200].dominantType = "";
  c17_info[200].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[200].fileTimeLo = 1346510340U;
  c17_info[200].fileTimeHi = 0U;
  c17_info[200].mFileTimeLo = 0U;
  c17_info[200].mFileTimeHi = 0U;
  c17_info[201].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[201].name = "eml_index_class";
  c17_info[201].dominantType = "";
  c17_info[201].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[201].fileTimeLo = 1323170578U;
  c17_info[201].fileTimeHi = 0U;
  c17_info[201].mFileTimeLo = 0U;
  c17_info[201].mFileTimeHi = 0U;
  c17_info[202].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[202].name = "eml_index_plus";
  c17_info[202].dominantType = "double";
  c17_info[202].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[202].fileTimeLo = 1286818778U;
  c17_info[202].fileTimeHi = 0U;
  c17_info[202].mFileTimeLo = 0U;
  c17_info[202].mFileTimeHi = 0U;
  c17_info[203].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[203].name = "eml_int_forloop_overflow_check";
  c17_info[203].dominantType = "";
  c17_info[203].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[203].fileTimeLo = 1346510340U;
  c17_info[203].fileTimeHi = 0U;
  c17_info[203].mFileTimeLo = 0U;
  c17_info[203].mFileTimeHi = 0U;
  c17_info[204].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[204].name = "eml_index_minus";
  c17_info[204].dominantType = "double";
  c17_info[204].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[204].fileTimeLo = 1286818778U;
  c17_info[204].fileTimeHi = 0U;
  c17_info[204].mFileTimeLo = 0U;
  c17_info[204].mFileTimeHi = 0U;
  c17_info[205].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[205].name = "eml_index_minus";
  c17_info[205].dominantType = "coder.internal.indexInt";
  c17_info[205].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[205].fileTimeLo = 1286818778U;
  c17_info[205].fileTimeHi = 0U;
  c17_info[205].mFileTimeLo = 0U;
  c17_info[205].mFileTimeHi = 0U;
  c17_info[206].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[206].name = "eml_index_times";
  c17_info[206].dominantType = "coder.internal.indexInt";
  c17_info[206].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c17_info[206].fileTimeLo = 1286818780U;
  c17_info[206].fileTimeHi = 0U;
  c17_info[206].mFileTimeLo = 0U;
  c17_info[206].mFileTimeHi = 0U;
  c17_info[207].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[207].name = "eml_index_plus";
  c17_info[207].dominantType = "coder.internal.indexInt";
  c17_info[207].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[207].fileTimeLo = 1286818778U;
  c17_info[207].fileTimeHi = 0U;
  c17_info[207].mFileTimeLo = 0U;
  c17_info[207].mFileTimeHi = 0U;
  c17_info[208].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[208].name = "eml_ixamax";
  c17_info[208].dominantType = "double";
  c17_info[208].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c17_info[208].fileTimeLo = 1299076770U;
  c17_info[208].fileTimeHi = 0U;
  c17_info[208].mFileTimeLo = 0U;
  c17_info[208].mFileTimeHi = 0U;
  c17_info[209].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c17_info[209].name = "eml_blas_inline";
  c17_info[209].dominantType = "";
  c17_info[209].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[209].fileTimeLo = 1299076768U;
  c17_info[209].fileTimeHi = 0U;
  c17_info[209].mFileTimeLo = 0U;
  c17_info[209].mFileTimeHi = 0U;
  c17_info[210].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c17_info[210].name = "length";
  c17_info[210].dominantType = "double";
  c17_info[210].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c17_info[210].fileTimeLo = 1303146206U;
  c17_info[210].fileTimeHi = 0U;
  c17_info[210].mFileTimeLo = 0U;
  c17_info[210].mFileTimeHi = 0U;
  c17_info[211].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c17_info[211].name = "eml_index_class";
  c17_info[211].dominantType = "";
  c17_info[211].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[211].fileTimeLo = 1323170578U;
  c17_info[211].fileTimeHi = 0U;
  c17_info[211].mFileTimeLo = 0U;
  c17_info[211].mFileTimeHi = 0U;
  c17_info[212].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c17_info[212].name = "eml_refblas_ixamax";
  c17_info[212].dominantType = "double";
  c17_info[212].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c17_info[212].fileTimeLo = 1299076770U;
  c17_info[212].fileTimeHi = 0U;
  c17_info[212].mFileTimeLo = 0U;
  c17_info[212].mFileTimeHi = 0U;
  c17_info[213].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c17_info[213].name = "eml_index_class";
  c17_info[213].dominantType = "";
  c17_info[213].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[213].fileTimeLo = 1323170578U;
  c17_info[213].fileTimeHi = 0U;
  c17_info[213].mFileTimeLo = 0U;
  c17_info[213].mFileTimeHi = 0U;
  c17_info[214].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c17_info[214].name = "eml_xcabs1";
  c17_info[214].dominantType = "double";
  c17_info[214].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c17_info[214].fileTimeLo = 1286818706U;
  c17_info[214].fileTimeHi = 0U;
  c17_info[214].mFileTimeLo = 0U;
  c17_info[214].mFileTimeHi = 0U;
  c17_info[215].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c17_info[215].name = "abs";
  c17_info[215].dominantType = "double";
  c17_info[215].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[215].fileTimeLo = 1343830366U;
  c17_info[215].fileTimeHi = 0U;
  c17_info[215].mFileTimeLo = 0U;
  c17_info[215].mFileTimeHi = 0U;
  c17_info[216].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c17_info[216].name = "eml_int_forloop_overflow_check";
  c17_info[216].dominantType = "";
  c17_info[216].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[216].fileTimeLo = 1346510340U;
  c17_info[216].fileTimeHi = 0U;
  c17_info[216].mFileTimeLo = 0U;
  c17_info[216].mFileTimeHi = 0U;
  c17_info[217].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c17_info[217].name = "eml_index_plus";
  c17_info[217].dominantType = "coder.internal.indexInt";
  c17_info[217].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[217].fileTimeLo = 1286818778U;
  c17_info[217].fileTimeHi = 0U;
  c17_info[217].mFileTimeLo = 0U;
  c17_info[217].mFileTimeHi = 0U;
  c17_info[218].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[218].name = "eml_xswap";
  c17_info[218].dominantType = "double";
  c17_info[218].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c17_info[218].fileTimeLo = 1299076778U;
  c17_info[218].fileTimeHi = 0U;
  c17_info[218].mFileTimeLo = 0U;
  c17_info[218].mFileTimeHi = 0U;
  c17_info[219].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[219].name = "eml_div";
  c17_info[219].dominantType = "double";
  c17_info[219].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c17_info[219].fileTimeLo = 1313347810U;
  c17_info[219].fileTimeHi = 0U;
  c17_info[219].mFileTimeLo = 0U;
  c17_info[219].mFileTimeHi = 0U;
  c17_info[220].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c17_info[220].name = "eml_xgeru";
  c17_info[220].dominantType = "double";
  c17_info[220].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c17_info[220].fileTimeLo = 1299076774U;
  c17_info[220].fileTimeHi = 0U;
  c17_info[220].mFileTimeLo = 0U;
  c17_info[220].mFileTimeHi = 0U;
  c17_info[221].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c17_info[221].name = "eml_blas_inline";
  c17_info[221].dominantType = "";
  c17_info[221].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[221].fileTimeLo = 1299076768U;
  c17_info[221].fileTimeHi = 0U;
  c17_info[221].mFileTimeLo = 0U;
  c17_info[221].mFileTimeHi = 0U;
  c17_info[222].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c17_info[222].name = "eml_xger";
  c17_info[222].dominantType = "double";
  c17_info[222].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c17_info[222].fileTimeLo = 1299076774U;
  c17_info[222].fileTimeHi = 0U;
  c17_info[222].mFileTimeLo = 0U;
  c17_info[222].mFileTimeHi = 0U;
  c17_info[223].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c17_info[223].name = "eml_blas_inline";
  c17_info[223].dominantType = "";
  c17_info[223].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[223].fileTimeLo = 1299076768U;
  c17_info[223].fileTimeHi = 0U;
  c17_info[223].mFileTimeLo = 0U;
  c17_info[223].mFileTimeHi = 0U;
  c17_info[224].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c17_info[224].name = "intmax";
  c17_info[224].dominantType = "char";
  c17_info[224].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c17_info[224].fileTimeLo = 1311255316U;
  c17_info[224].fileTimeHi = 0U;
  c17_info[224].mFileTimeLo = 0U;
  c17_info[224].mFileTimeHi = 0U;
  c17_info[225].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c17_info[225].name = "min";
  c17_info[225].dominantType = "double";
  c17_info[225].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c17_info[225].fileTimeLo = 1311255318U;
  c17_info[225].fileTimeHi = 0U;
  c17_info[225].mFileTimeLo = 0U;
  c17_info[225].mFileTimeHi = 0U;
  c17_info[226].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c17_info[226].name = "mtimes";
  c17_info[226].dominantType = "double";
  c17_info[226].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[226].fileTimeLo = 1289519692U;
  c17_info[226].fileTimeHi = 0U;
  c17_info[226].mFileTimeLo = 0U;
  c17_info[226].mFileTimeHi = 0U;
  c17_info[227].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c17_info[227].name = "eml_index_class";
  c17_info[227].dominantType = "";
  c17_info[227].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[227].fileTimeLo = 1323170578U;
  c17_info[227].fileTimeHi = 0U;
  c17_info[227].mFileTimeLo = 0U;
  c17_info[227].mFileTimeHi = 0U;
  c17_info[228].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c17_info[228].name = "eml_refblas_xger";
  c17_info[228].dominantType = "double";
  c17_info[228].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c17_info[228].fileTimeLo = 1299076776U;
  c17_info[228].fileTimeHi = 0U;
  c17_info[228].mFileTimeLo = 0U;
  c17_info[228].mFileTimeHi = 0U;
  c17_info[229].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c17_info[229].name = "eml_refblas_xgerx";
  c17_info[229].dominantType = "char";
  c17_info[229].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c17_info[229].fileTimeLo = 1299076778U;
  c17_info[229].fileTimeHi = 0U;
  c17_info[229].mFileTimeLo = 0U;
  c17_info[229].mFileTimeHi = 0U;
  c17_info[230].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c17_info[230].name = "eml_index_class";
  c17_info[230].dominantType = "";
  c17_info[230].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[230].fileTimeLo = 1323170578U;
  c17_info[230].fileTimeHi = 0U;
  c17_info[230].mFileTimeLo = 0U;
  c17_info[230].mFileTimeHi = 0U;
  c17_info[231].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c17_info[231].name = "abs";
  c17_info[231].dominantType = "coder.internal.indexInt";
  c17_info[231].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[231].fileTimeLo = 1343830366U;
  c17_info[231].fileTimeHi = 0U;
  c17_info[231].mFileTimeLo = 0U;
  c17_info[231].mFileTimeHi = 0U;
  c17_info[232].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c17_info[232].name = "eml_index_minus";
  c17_info[232].dominantType = "double";
  c17_info[232].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[232].fileTimeLo = 1286818778U;
  c17_info[232].fileTimeHi = 0U;
  c17_info[232].mFileTimeLo = 0U;
  c17_info[232].mFileTimeHi = 0U;
  c17_info[233].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c17_info[233].name = "eml_int_forloop_overflow_check";
  c17_info[233].dominantType = "";
  c17_info[233].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[233].fileTimeLo = 1346510340U;
  c17_info[233].fileTimeHi = 0U;
  c17_info[233].mFileTimeLo = 0U;
  c17_info[233].mFileTimeHi = 0U;
  c17_info[234].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c17_info[234].name = "eml_index_plus";
  c17_info[234].dominantType = "double";
  c17_info[234].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[234].fileTimeLo = 1286818778U;
  c17_info[234].fileTimeHi = 0U;
  c17_info[234].mFileTimeLo = 0U;
  c17_info[234].mFileTimeHi = 0U;
  c17_info[235].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c17_info[235].name = "eml_index_plus";
  c17_info[235].dominantType = "coder.internal.indexInt";
  c17_info[235].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[235].fileTimeLo = 1286818778U;
  c17_info[235].fileTimeHi = 0U;
  c17_info[235].mFileTimeLo = 0U;
  c17_info[235].mFileTimeHi = 0U;
  c17_info[236].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c17_info[236].name = "eml_ipiv2perm";
  c17_info[236].dominantType = "coder.internal.indexInt";
  c17_info[236].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c17_info[236].fileTimeLo = 1286818782U;
  c17_info[236].fileTimeHi = 0U;
  c17_info[236].mFileTimeLo = 0U;
  c17_info[236].mFileTimeHi = 0U;
  c17_info[237].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c17_info[237].name = "colon";
  c17_info[237].dominantType = "double";
  c17_info[237].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c17_info[237].fileTimeLo = 1348191928U;
  c17_info[237].fileTimeHi = 0U;
  c17_info[237].mFileTimeLo = 0U;
  c17_info[237].mFileTimeHi = 0U;
  c17_info[238].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c17_info[238].name = "eml_index_class";
  c17_info[238].dominantType = "";
  c17_info[238].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[238].fileTimeLo = 1323170578U;
  c17_info[238].fileTimeHi = 0U;
  c17_info[238].mFileTimeLo = 0U;
  c17_info[238].mFileTimeHi = 0U;
  c17_info[239].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c17_info[239].name = "coder.internal.indexIntRelop";
  c17_info[239].dominantType = "";
  c17_info[239].resolved =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m";
  c17_info[239].fileTimeLo = 1326728322U;
  c17_info[239].fileTimeHi = 0U;
  c17_info[239].mFileTimeLo = 0U;
  c17_info[239].mFileTimeHi = 0U;
  c17_info[240].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c17_info[240].name = "eml_int_forloop_overflow_check";
  c17_info[240].dominantType = "";
  c17_info[240].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[240].fileTimeLo = 1346510340U;
  c17_info[240].fileTimeHi = 0U;
  c17_info[240].mFileTimeLo = 0U;
  c17_info[240].mFileTimeHi = 0U;
  c17_info[241].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c17_info[241].name = "eml_index_plus";
  c17_info[241].dominantType = "double";
  c17_info[241].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[241].fileTimeLo = 1286818778U;
  c17_info[241].fileTimeHi = 0U;
  c17_info[241].mFileTimeLo = 0U;
  c17_info[241].mFileTimeHi = 0U;
  c17_info[242].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c17_info[242].name = "mtimes";
  c17_info[242].dominantType = "double";
  c17_info[242].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[242].fileTimeLo = 1289519692U;
  c17_info[242].fileTimeHi = 0U;
  c17_info[242].mFileTimeLo = 0U;
  c17_info[242].mFileTimeHi = 0U;
  c17_info[243].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c17_info[243].name = "eml_scalar_eg";
  c17_info[243].dominantType = "double";
  c17_info[243].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[243].fileTimeLo = 1286818796U;
  c17_info[243].fileTimeHi = 0U;
  c17_info[243].mFileTimeLo = 0U;
  c17_info[243].mFileTimeHi = 0U;
  c17_info[244].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c17_info[244].name = "eml_xtrsm";
  c17_info[244].dominantType = "char";
  c17_info[244].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c17_info[244].fileTimeLo = 1299076778U;
  c17_info[244].fileTimeHi = 0U;
  c17_info[244].mFileTimeLo = 0U;
  c17_info[244].mFileTimeHi = 0U;
  c17_info[245].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c17_info[245].name = "eml_blas_inline";
  c17_info[245].dominantType = "";
  c17_info[245].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[245].fileTimeLo = 1299076768U;
  c17_info[245].fileTimeHi = 0U;
  c17_info[245].mFileTimeLo = 0U;
  c17_info[245].mFileTimeHi = 0U;
  c17_info[246].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c17_info[246].name = "mtimes";
  c17_info[246].dominantType = "double";
  c17_info[246].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[246].fileTimeLo = 1289519692U;
  c17_info[246].fileTimeHi = 0U;
  c17_info[246].mFileTimeLo = 0U;
  c17_info[246].mFileTimeHi = 0U;
  c17_info[247].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c17_info[247].name = "eml_index_class";
  c17_info[247].dominantType = "";
  c17_info[247].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[247].fileTimeLo = 1323170578U;
  c17_info[247].fileTimeHi = 0U;
  c17_info[247].mFileTimeLo = 0U;
  c17_info[247].mFileTimeHi = 0U;
  c17_info[248].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c17_info[248].name = "eml_scalar_eg";
  c17_info[248].dominantType = "double";
  c17_info[248].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[248].fileTimeLo = 1286818796U;
  c17_info[248].fileTimeHi = 0U;
  c17_info[248].mFileTimeLo = 0U;
  c17_info[248].mFileTimeHi = 0U;
  c17_info[249].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c17_info[249].name = "eml_refblas_xtrsm";
  c17_info[249].dominantType = "char";
  c17_info[249].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[249].fileTimeLo = 1299076786U;
  c17_info[249].fileTimeHi = 0U;
  c17_info[249].mFileTimeLo = 0U;
  c17_info[249].mFileTimeHi = 0U;
  c17_info[250].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[250].name = "eml_scalar_eg";
  c17_info[250].dominantType = "double";
  c17_info[250].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[250].fileTimeLo = 1286818796U;
  c17_info[250].fileTimeHi = 0U;
  c17_info[250].mFileTimeLo = 0U;
  c17_info[250].mFileTimeHi = 0U;
  c17_info[251].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[251].name = "eml_index_minus";
  c17_info[251].dominantType = "double";
  c17_info[251].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[251].fileTimeLo = 1286818778U;
  c17_info[251].fileTimeHi = 0U;
  c17_info[251].mFileTimeLo = 0U;
  c17_info[251].mFileTimeHi = 0U;
  c17_info[252].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[252].name = "eml_index_class";
  c17_info[252].dominantType = "";
  c17_info[252].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[252].fileTimeLo = 1323170578U;
  c17_info[252].fileTimeHi = 0U;
  c17_info[252].mFileTimeLo = 0U;
  c17_info[252].mFileTimeHi = 0U;
  c17_info[253].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[253].name = "eml_int_forloop_overflow_check";
  c17_info[253].dominantType = "";
  c17_info[253].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[253].fileTimeLo = 1346510340U;
  c17_info[253].fileTimeHi = 0U;
  c17_info[253].mFileTimeLo = 0U;
  c17_info[253].mFileTimeHi = 0U;
  c17_info[254].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[254].name = "eml_index_times";
  c17_info[254].dominantType = "coder.internal.indexInt";
  c17_info[254].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c17_info[254].fileTimeLo = 1286818780U;
  c17_info[254].fileTimeHi = 0U;
  c17_info[254].mFileTimeLo = 0U;
  c17_info[254].mFileTimeHi = 0U;
  c17_info[255].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[255].name = "eml_index_plus";
  c17_info[255].dominantType = "coder.internal.indexInt";
  c17_info[255].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[255].fileTimeLo = 1286818778U;
  c17_info[255].fileTimeHi = 0U;
  c17_info[255].mFileTimeLo = 0U;
  c17_info[255].mFileTimeHi = 0U;
}

static void c17_e_info_helper(c17_ResolvedFunctionInfo c17_info[302])
{
  c17_info[256].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[256].name = "eml_div";
  c17_info[256].dominantType = "double";
  c17_info[256].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c17_info[256].fileTimeLo = 1313347810U;
  c17_info[256].fileTimeHi = 0U;
  c17_info[256].mFileTimeLo = 0U;
  c17_info[256].mFileTimeHi = 0U;
  c17_info[257].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c17_info[257].name = "norm";
  c17_info[257].dominantType = "double";
  c17_info[257].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c17_info[257].fileTimeLo = 1336522094U;
  c17_info[257].fileTimeHi = 0U;
  c17_info[257].mFileTimeLo = 0U;
  c17_info[257].mFileTimeHi = 0U;
  c17_info[258].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c17_info[258].name = "abs";
  c17_info[258].dominantType = "double";
  c17_info[258].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c17_info[258].fileTimeLo = 1343830366U;
  c17_info[258].fileTimeHi = 0U;
  c17_info[258].mFileTimeLo = 0U;
  c17_info[258].mFileTimeHi = 0U;
  c17_info[259].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c17_info[259].name = "isnan";
  c17_info[259].dominantType = "double";
  c17_info[259].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c17_info[259].fileTimeLo = 1286818760U;
  c17_info[259].fileTimeHi = 0U;
  c17_info[259].mFileTimeLo = 0U;
  c17_info[259].mFileTimeHi = 0U;
  c17_info[260].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c17_info[260].name = "eml_guarded_nan";
  c17_info[260].dominantType = "char";
  c17_info[260].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c17_info[260].fileTimeLo = 1286818776U;
  c17_info[260].fileTimeHi = 0U;
  c17_info[260].mFileTimeLo = 0U;
  c17_info[260].mFileTimeHi = 0U;
  c17_info[261].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c17_info[261].name = "eml_is_float_class";
  c17_info[261].dominantType = "char";
  c17_info[261].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c17_info[261].fileTimeLo = 1286818782U;
  c17_info[261].fileTimeHi = 0U;
  c17_info[261].mFileTimeLo = 0U;
  c17_info[261].mFileTimeHi = 0U;
  c17_info[262].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c17_info[262].name = "mtimes";
  c17_info[262].dominantType = "double";
  c17_info[262].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[262].fileTimeLo = 1289519692U;
  c17_info[262].fileTimeHi = 0U;
  c17_info[262].mFileTimeLo = 0U;
  c17_info[262].mFileTimeHi = 0U;
  c17_info[263].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c17_info[263].name = "eml_warning";
  c17_info[263].dominantType = "char";
  c17_info[263].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c17_info[263].fileTimeLo = 1286818802U;
  c17_info[263].fileTimeHi = 0U;
  c17_info[263].mFileTimeLo = 0U;
  c17_info[263].mFileTimeHi = 0U;
  c17_info[264].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c17_info[264].name = "isnan";
  c17_info[264].dominantType = "double";
  c17_info[264].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c17_info[264].fileTimeLo = 1286818760U;
  c17_info[264].fileTimeHi = 0U;
  c17_info[264].mFileTimeLo = 0U;
  c17_info[264].mFileTimeHi = 0U;
  c17_info[265].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c17_info[265].name = "eps";
  c17_info[265].dominantType = "char";
  c17_info[265].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c17_info[265].fileTimeLo = 1326727996U;
  c17_info[265].fileTimeHi = 0U;
  c17_info[265].mFileTimeLo = 0U;
  c17_info[265].mFileTimeHi = 0U;
  c17_info[266].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c17_info[266].name = "eml_flt2str";
  c17_info[266].dominantType = "double";
  c17_info[266].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c17_info[266].fileTimeLo = 1309451196U;
  c17_info[266].fileTimeHi = 0U;
  c17_info[266].mFileTimeLo = 0U;
  c17_info[266].mFileTimeHi = 0U;
  c17_info[267].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c17_info[267].name = "char";
  c17_info[267].dominantType = "double";
  c17_info[267].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m";
  c17_info[267].fileTimeLo = 1319729968U;
  c17_info[267].fileTimeHi = 0U;
  c17_info[267].mFileTimeLo = 0U;
  c17_info[267].mFileTimeHi = 0U;
  c17_info[268].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[268].name = "eml_xdotu";
  c17_info[268].dominantType = "double";
  c17_info[268].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c17_info[268].fileTimeLo = 1299076772U;
  c17_info[268].fileTimeHi = 0U;
  c17_info[268].mFileTimeLo = 0U;
  c17_info[268].mFileTimeHi = 0U;
  c17_info[269].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c17_info[269].name = "eml_blas_inline";
  c17_info[269].dominantType = "";
  c17_info[269].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c17_info[269].fileTimeLo = 1299076768U;
  c17_info[269].fileTimeHi = 0U;
  c17_info[269].mFileTimeLo = 0U;
  c17_info[269].mFileTimeHi = 0U;
  c17_info[270].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c17_info[270].name = "eml_xdot";
  c17_info[270].dominantType = "double";
  c17_info[270].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c17_info[270].fileTimeLo = 1299076772U;
  c17_info[270].fileTimeHi = 0U;
  c17_info[270].mFileTimeLo = 0U;
  c17_info[270].mFileTimeHi = 0U;
  c17_info[271].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c17_info[271].name = "eml_index_minus";
  c17_info[271].dominantType = "double";
  c17_info[271].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c17_info[271].fileTimeLo = 1286818778U;
  c17_info[271].fileTimeHi = 0U;
  c17_info[271].mFileTimeLo = 0U;
  c17_info[271].mFileTimeHi = 0U;
  c17_info[272].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c17_info[272].name = "eml_index_times";
  c17_info[272].dominantType = "coder.internal.indexInt";
  c17_info[272].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c17_info[272].fileTimeLo = 1286818780U;
  c17_info[272].fileTimeHi = 0U;
  c17_info[272].mFileTimeLo = 0U;
  c17_info[272].mFileTimeHi = 0U;
  c17_info[273].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[273].name = "norm";
  c17_info[273].dominantType = "double";
  c17_info[273].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c17_info[273].fileTimeLo = 1336522094U;
  c17_info[273].fileTimeHi = 0U;
  c17_info[273].mFileTimeLo = 0U;
  c17_info[273].mFileTimeHi = 0U;
  c17_info[274].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c17_info[274].name = "eml_index_class";
  c17_info[274].dominantType = "";
  c17_info[274].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[274].fileTimeLo = 1323170578U;
  c17_info[274].fileTimeHi = 0U;
  c17_info[274].mFileTimeLo = 0U;
  c17_info[274].mFileTimeHi = 0U;
  c17_info[275].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c17_info[275].name = "eml_xnrm2";
  c17_info[275].dominantType = "double";
  c17_info[275].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c17_info[275].fileTimeLo = 1299076776U;
  c17_info[275].fileTimeHi = 0U;
  c17_info[275].mFileTimeLo = 0U;
  c17_info[275].mFileTimeHi = 0U;
  c17_info[276].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[276].name = "mrdivide";
  c17_info[276].dominantType = "double";
  c17_info[276].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c17_info[276].fileTimeLo = 1357951548U;
  c17_info[276].fileTimeHi = 0U;
  c17_info[276].mFileTimeLo = 1319729966U;
  c17_info[276].mFileTimeHi = 0U;
  c17_info[277].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c17_info[277].name = "mldivide";
  c17_info[277].dominantType = "double";
  c17_info[277].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c17_info[277].fileTimeLo = 1357951548U;
  c17_info[277].fileTimeHi = 0U;
  c17_info[277].mFileTimeLo = 1319729966U;
  c17_info[277].mFileTimeHi = 0U;
  c17_info[278].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c17_info[278].name = "eml_lusolve";
  c17_info[278].dominantType = "double";
  c17_info[278].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c17_info[278].fileTimeLo = 1309451196U;
  c17_info[278].fileTimeHi = 0U;
  c17_info[278].mFileTimeLo = 0U;
  c17_info[278].mFileTimeHi = 0U;
  c17_info[279].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c17_info[279].name = "eml_index_class";
  c17_info[279].dominantType = "";
  c17_info[279].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[279].fileTimeLo = 1323170578U;
  c17_info[279].fileTimeHi = 0U;
  c17_info[279].mFileTimeLo = 0U;
  c17_info[279].mFileTimeHi = 0U;
  c17_info[280].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c17_info[280].name = "eml_index_class";
  c17_info[280].dominantType = "";
  c17_info[280].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[280].fileTimeLo = 1323170578U;
  c17_info[280].fileTimeHi = 0U;
  c17_info[280].mFileTimeLo = 0U;
  c17_info[280].mFileTimeHi = 0U;
  c17_info[281].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c17_info[281].name = "eml_xgetrf";
  c17_info[281].dominantType = "double";
  c17_info[281].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c17_info[281].fileTimeLo = 1286818806U;
  c17_info[281].fileTimeHi = 0U;
  c17_info[281].mFileTimeLo = 0U;
  c17_info[281].mFileTimeHi = 0U;
  c17_info[282].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c17_info[282].name = "eml_warning";
  c17_info[282].dominantType = "char";
  c17_info[282].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c17_info[282].fileTimeLo = 1286818802U;
  c17_info[282].fileTimeHi = 0U;
  c17_info[282].mFileTimeLo = 0U;
  c17_info[282].mFileTimeHi = 0U;
  c17_info[283].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c17_info[283].name = "eml_scalar_eg";
  c17_info[283].dominantType = "double";
  c17_info[283].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[283].fileTimeLo = 1286818796U;
  c17_info[283].fileTimeHi = 0U;
  c17_info[283].mFileTimeLo = 0U;
  c17_info[283].mFileTimeHi = 0U;
  c17_info[284].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c17_info[284].name = "eml_int_forloop_overflow_check";
  c17_info[284].dominantType = "";
  c17_info[284].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[284].fileTimeLo = 1346510340U;
  c17_info[284].fileTimeHi = 0U;
  c17_info[284].mFileTimeLo = 0U;
  c17_info[284].mFileTimeHi = 0U;
  c17_info[285].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c17_info[285].name = "eml_xtrsm";
  c17_info[285].dominantType = "char";
  c17_info[285].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c17_info[285].fileTimeLo = 1299076778U;
  c17_info[285].fileTimeHi = 0U;
  c17_info[285].mFileTimeLo = 0U;
  c17_info[285].mFileTimeHi = 0U;
  c17_info[286].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c17_info[286].name = "eml_index_plus";
  c17_info[286].dominantType = "double";
  c17_info[286].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[286].fileTimeLo = 1286818778U;
  c17_info[286].fileTimeHi = 0U;
  c17_info[286].mFileTimeLo = 0U;
  c17_info[286].mFileTimeHi = 0U;
  c17_info[287].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[287].name = "pinvDamped";
  c17_info[287].dominantType = "double";
  c17_info[287].resolved =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c17_info[287].fileTimeLo = 1495126739U;
  c17_info[287].fileTimeHi = 0U;
  c17_info[287].mFileTimeLo = 0U;
  c17_info[287].mFileTimeHi = 0U;
  c17_info[288].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c17_info[288].name = "mtimes";
  c17_info[288].dominantType = "double";
  c17_info[288].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c17_info[288].fileTimeLo = 1289519692U;
  c17_info[288].fileTimeHi = 0U;
  c17_info[288].mFileTimeLo = 0U;
  c17_info[288].mFileTimeHi = 0U;
  c17_info[289].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c17_info[289].name = "eye";
  c17_info[289].dominantType = "double";
  c17_info[289].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c17_info[289].fileTimeLo = 1286818688U;
  c17_info[289].fileTimeHi = 0U;
  c17_info[289].mFileTimeLo = 0U;
  c17_info[289].mFileTimeHi = 0U;
  c17_info[290].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/pinvDamped.m";
  c17_info[290].name = "mrdivide";
  c17_info[290].dominantType = "double";
  c17_info[290].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c17_info[290].fileTimeLo = 1357951548U;
  c17_info[290].fileTimeHi = 0U;
  c17_info[290].mFileTimeLo = 1319729966U;
  c17_info[290].mFileTimeHi = 0U;
  c17_info[291].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[291].name = "diag";
  c17_info[291].dominantType = "double";
  c17_info[291].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c17_info[291].fileTimeLo = 1286818686U;
  c17_info[291].fileTimeHi = 0U;
  c17_info[291].mFileTimeLo = 0U;
  c17_info[291].mFileTimeHi = 0U;
  c17_info[292].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c17_info[292].name = "eml_index_class";
  c17_info[292].dominantType = "";
  c17_info[292].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[292].fileTimeLo = 1323170578U;
  c17_info[292].fileTimeHi = 0U;
  c17_info[292].mFileTimeLo = 0U;
  c17_info[292].mFileTimeHi = 0U;
  c17_info[293].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c17_info[293].name = "eml_index_plus";
  c17_info[293].dominantType = "coder.internal.indexInt";
  c17_info[293].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[293].fileTimeLo = 1286818778U;
  c17_info[293].fileTimeHi = 0U;
  c17_info[293].mFileTimeLo = 0U;
  c17_info[293].mFileTimeHi = 0U;
  c17_info[294].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c17_info[294].name = "eml_scalar_eg";
  c17_info[294].dominantType = "double";
  c17_info[294].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[294].fileTimeLo = 1286818796U;
  c17_info[294].fileTimeHi = 0U;
  c17_info[294].mFileTimeLo = 0U;
  c17_info[294].mFileTimeHi = 0U;
  c17_info[295].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c17_info[295].name = "eml_int_forloop_overflow_check";
  c17_info[295].dominantType = "";
  c17_info[295].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c17_info[295].fileTimeLo = 1346510340U;
  c17_info[295].fileTimeHi = 0U;
  c17_info[295].mFileTimeLo = 0U;
  c17_info[295].mFileTimeHi = 0U;
  c17_info[296].context =
    "[E]/home/vale/git/learnOptimWBC/matlab/Common/TB_StandUp/src/balancingController.m";
  c17_info[296].name = "blkdiag";
  c17_info[296].dominantType = "double";
  c17_info[296].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/blkdiag.m";
  c17_info[296].fileTimeLo = 1307651238U;
  c17_info[296].fileTimeHi = 0U;
  c17_info[296].mFileTimeLo = 0U;
  c17_info[296].mFileTimeHi = 0U;
  c17_info[297].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/blkdiag.m!output_size";
  c17_info[297].name = "eml_index_class";
  c17_info[297].dominantType = "";
  c17_info[297].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[297].fileTimeLo = 1323170578U;
  c17_info[297].fileTimeHi = 0U;
  c17_info[297].mFileTimeLo = 0U;
  c17_info[297].mFileTimeHi = 0U;
  c17_info[298].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/blkdiag.m!output_size";
  c17_info[298].name = "eml_index_plus";
  c17_info[298].dominantType = "double";
  c17_info[298].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[298].fileTimeLo = 1286818778U;
  c17_info[298].fileTimeHi = 0U;
  c17_info[298].mFileTimeLo = 0U;
  c17_info[298].mFileTimeHi = 0U;
  c17_info[299].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/blkdiag.m";
  c17_info[299].name = "eml_scalar_eg";
  c17_info[299].dominantType = "double";
  c17_info[299].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c17_info[299].fileTimeLo = 1286818796U;
  c17_info[299].fileTimeHi = 0U;
  c17_info[299].mFileTimeLo = 0U;
  c17_info[299].mFileTimeHi = 0U;
  c17_info[300].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/blkdiag.m";
  c17_info[300].name = "eml_index_class";
  c17_info[300].dominantType = "";
  c17_info[300].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c17_info[300].fileTimeLo = 1323170578U;
  c17_info[300].fileTimeHi = 0U;
  c17_info[300].mFileTimeLo = 0U;
  c17_info[300].mFileTimeHi = 0U;
  c17_info[301].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/blkdiag.m";
  c17_info[301].name = "eml_index_plus";
  c17_info[301].dominantType = "double";
  c17_info[301].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c17_info[301].fileTimeLo = 1286818778U;
  c17_info[301].fileTimeHi = 0U;
  c17_info[301].mFileTimeLo = 0U;
  c17_info[301].mFileTimeHi = 0U;
}

static void c17_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_b_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static boolean_T c17_eml_use_refblas(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  return FALSE;
}

static void c17_eye(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c17_I[9])
{
  int32_T c17_i862;
  int32_T c17_i;
  int32_T c17_b_i;
  for (c17_i862 = 0; c17_i862 < 9; c17_i862++) {
    c17_I[c17_i862] = 0.0;
  }

  for (c17_i = 1; c17_i < 4; c17_i++) {
    c17_b_i = c17_i;
    c17_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c17_b_i), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 3, 2, 0) - 1))
      - 1] = 1.0;
  }
}

static void c17_pinv(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c17_A[72], real_T c17_tol, real_T c17_X[72])
{
  int32_T c17_i863;
  int32_T c17_i864;
  int32_T c17_i865;
  int32_T c17_i866;
  real_T c17_U[72];
  real_T c17_b_tol;
  int32_T c17_i867;
  real_T c17_b_X[72];
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_x;
  real_T c17_b_x;
  boolean_T c17_b;
  boolean_T c17_b1;
  real_T c17_c_x;
  boolean_T c17_b_b;
  boolean_T c17_b2;
  boolean_T c17_c_b;
  int32_T c17_i868;
  real_T c17_b_U[72];
  real_T c17_V[36];
  real_T c17_s[6];
  int32_T c17_i869;
  real_T c17_S[36];
  int32_T c17_c_k;
  real_T c17_d_k;
  int32_T c17_r;
  int32_T c17_e_k;
  int32_T c17_f_k;
  int32_T c17_a;
  int32_T c17_vcol;
  int32_T c17_b_r;
  int32_T c17_d_b;
  int32_T c17_e_b;
  boolean_T c17_overflow;
  int32_T c17_j;
  int32_T c17_b_j;
  real_T c17_y;
  real_T c17_z;
  int32_T c17_b_a;
  int32_T c17_i870;
  real_T c17_b_V[36];
  int32_T c17_i871;
  real_T c17_c_U[72];
  int32_T c17_i872;
  int32_T c17_i873;
  int32_T c17_i874;
  int32_T c17_i875;
  boolean_T exitg1;
  c17_i863 = 0;
  for (c17_i864 = 0; c17_i864 < 6; c17_i864++) {
    c17_i865 = 0;
    for (c17_i866 = 0; c17_i866 < 12; c17_i866++) {
      c17_U[c17_i866 + c17_i863] = c17_A[c17_i865 + c17_i864];
      c17_i865 += 6;
    }

    c17_i863 += 12;
  }

  c17_b_tol = c17_tol;
  for (c17_i867 = 0; c17_i867 < 72; c17_i867++) {
    c17_b_X[c17_i867] = 0.0;
  }

  for (c17_k = 1; c17_k < 73; c17_k++) {
    c17_b_k = c17_k;
    c17_x = c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 72, 1, 0) - 1];
    c17_b_x = c17_x;
    c17_b = muDoubleScalarIsInf(c17_b_x);
    c17_b1 = !c17_b;
    c17_c_x = c17_x;
    c17_b_b = muDoubleScalarIsNaN(c17_c_x);
    c17_b2 = !c17_b_b;
    c17_c_b = (c17_b1 && c17_b2);
    if (!c17_c_b) {
      c17_eml_error(chartInstance);
    }
  }

  for (c17_i868 = 0; c17_i868 < 72; c17_i868++) {
    c17_b_U[c17_i868] = c17_U[c17_i868];
  }

  c17_eml_xgesvd(chartInstance, c17_b_U, c17_U, c17_s, c17_V);
  for (c17_i869 = 0; c17_i869 < 36; c17_i869++) {
    c17_S[c17_i869] = 0.0;
  }

  for (c17_c_k = 0; c17_c_k < 6; c17_c_k++) {
    c17_d_k = 1.0 + (real_T)c17_c_k;
    c17_S[((int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c17_d_k), 1, 6, 1, 0) + 6 * ((int32_T)(real_T)
            _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              c17_d_k), 1, 6, 2, 0) - 1)) - 1] = c17_s[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c17_d_k),
      1, 6, 1, 0) - 1];
  }

  c17_r = 0;
  c17_e_k = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c17_e_k < 7)) {
    c17_f_k = c17_e_k;
    if (!(c17_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_f_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_f_k), 1, 6, 2, 0) - 1))
          - 1] > c17_b_tol)) {
      exitg1 = TRUE;
    } else {
      c17_a = c17_r + 1;
      c17_r = c17_a;
      c17_e_k++;
    }
  }

  if (c17_r > 0) {
    c17_vcol = 1;
    c17_b_r = c17_r;
    c17_d_b = c17_b_r;
    c17_e_b = c17_d_b;
    if (1 > c17_e_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_e_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_j = 1; c17_j <= c17_b_r; c17_j++) {
      c17_b_j = c17_j;
      c17_y = c17_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c17_b_j), 1, 6, 1, 0) + 6 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 6, 2, 0) - 1)) - 1];
      c17_z = 1.0 / c17_y;
      c17_k_eml_xscal(chartInstance, c17_z, c17_V, c17_vcol);
      c17_b_a = c17_vcol + 6;
      c17_vcol = c17_b_a;
    }

    for (c17_i870 = 0; c17_i870 < 36; c17_i870++) {
      c17_b_V[c17_i870] = c17_V[c17_i870];
    }

    for (c17_i871 = 0; c17_i871 < 72; c17_i871++) {
      c17_c_U[c17_i871] = c17_U[c17_i871];
    }

    c17_p_eml_xgemm(chartInstance, c17_r, c17_b_V, c17_c_U, c17_b_X);
  }

  c17_i872 = 0;
  for (c17_i873 = 0; c17_i873 < 6; c17_i873++) {
    c17_i874 = 0;
    for (c17_i875 = 0; c17_i875 < 12; c17_i875++) {
      c17_X[c17_i875 + c17_i872] = c17_b_X[c17_i874 + c17_i873];
      c17_i874 += 6;
    }

    c17_i872 += 12;
  }
}

static void c17_c_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static boolean_T c17_isfinite(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x)
{
  real_T c17_b_x;
  boolean_T c17_b_b;
  boolean_T c17_b3;
  real_T c17_c_x;
  boolean_T c17_c_b;
  boolean_T c17_b4;
  c17_b_x = c17_x;
  c17_b_b = muDoubleScalarIsInf(c17_b_x);
  c17_b3 = !c17_b_b;
  c17_c_x = c17_x;
  c17_c_b = muDoubleScalarIsNaN(c17_c_x);
  c17_b4 = !c17_c_b;
  return c17_b3 && c17_b4;
}

static void c17_eml_error(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c17_i876;
  static char_T c17_cv1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x', 'W', 'i',
    't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c17_u[33];
  const mxArray *c17_y = NULL;
  for (c17_i876 = 0; c17_i876 < 33; c17_i876++) {
    c17_u[c17_i876] = c17_cv1[c17_i876];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 10, 0U, 1U, 0U, 2, 1, 33),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c17_y));
}

static void c17_eml_xgesvd(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[72], real_T c17_U[72], real_T c17_S[6], real_T
  c17_V[36])
{
  int32_T c17_i877;
  real_T c17_b_A[72];
  int32_T c17_i878;
  real_T c17_s[6];
  int32_T c17_i879;
  real_T c17_e[6];
  int32_T c17_i880;
  real_T c17_work[12];
  int32_T c17_i881;
  int32_T c17_i882;
  real_T c17_Vf[36];
  int32_T c17_q;
  int32_T c17_b_q;
  int32_T c17_a;
  int32_T c17_qp1;
  int32_T c17_b_a;
  int32_T c17_qm1;
  int32_T c17_b;
  int32_T c17_c;
  int32_T c17_c_a;
  int32_T c17_b_b;
  int32_T c17_qq;
  int32_T c17_c_b;
  int32_T c17_nmq;
  int32_T c17_d_a;
  int32_T c17_nmqp1;
  int32_T c17_i883;
  real_T c17_c_A[72];
  real_T c17_nrm;
  real_T c17_absx;
  real_T c17_d;
  real_T c17_y;
  real_T c17_d1;
  int32_T c17_b_qp1;
  boolean_T c17_overflow;
  int32_T c17_jj;
  int32_T c17_b_jj;
  int32_T c17_e_a;
  int32_T c17_b_c;
  int32_T c17_d_b;
  int32_T c17_c_c;
  int32_T c17_f_a;
  int32_T c17_e_b;
  int32_T c17_qjj;
  int32_T c17_i884;
  real_T c17_d_A[72];
  int32_T c17_i885;
  real_T c17_e_A[72];
  real_T c17_t;
  int32_T c17_c_q;
  boolean_T c17_b_overflow;
  int32_T c17_ii;
  int32_T c17_b_ii;
  int32_T c17_f_b;
  int32_T c17_pmq;
  int32_T c17_i886;
  real_T c17_b_e[6];
  real_T c17_b_absx;
  real_T c17_b_d;
  real_T c17_b_y;
  real_T c17_d2;
  int32_T c17_c_qp1;
  boolean_T c17_c_overflow;
  int32_T c17_c_ii;
  int32_T c17_d_qp1;
  boolean_T c17_d_overflow;
  int32_T c17_c_jj;
  int32_T c17_g_a;
  int32_T c17_d_c;
  int32_T c17_g_b;
  int32_T c17_e_c;
  int32_T c17_h_a;
  int32_T c17_h_b;
  int32_T c17_qp1jj;
  int32_T c17_i887;
  real_T c17_f_A[72];
  int32_T c17_e_qp1;
  boolean_T c17_e_overflow;
  int32_T c17_d_jj;
  int32_T c17_i_a;
  int32_T c17_f_c;
  int32_T c17_i_b;
  int32_T c17_g_c;
  int32_T c17_j_a;
  int32_T c17_j_b;
  int32_T c17_i888;
  real_T c17_b_work[12];
  int32_T c17_f_qp1;
  boolean_T c17_f_overflow;
  int32_T c17_d_ii;
  int32_T c17_m;
  int32_T c17_d_q;
  int32_T c17_k_a;
  int32_T c17_k_b;
  int32_T c17_l_a;
  int32_T c17_m_a;
  int32_T c17_h_c;
  int32_T c17_l_b;
  int32_T c17_i_c;
  int32_T c17_n_a;
  int32_T c17_m_b;
  int32_T c17_g_qp1;
  boolean_T c17_g_overflow;
  int32_T c17_e_jj;
  int32_T c17_o_a;
  int32_T c17_j_c;
  int32_T c17_n_b;
  int32_T c17_k_c;
  int32_T c17_p_a;
  int32_T c17_o_b;
  int32_T c17_i889;
  real_T c17_b_U[72];
  int32_T c17_i890;
  real_T c17_c_U[72];
  int32_T c17_e_q;
  boolean_T c17_h_overflow;
  int32_T c17_e_ii;
  int32_T c17_q_a;
  int32_T c17_i891;
  int32_T c17_p_b;
  int32_T c17_q_b;
  boolean_T c17_i_overflow;
  int32_T c17_f_ii;
  int32_T c17_g_ii;
  int32_T c17_f_q;
  int32_T c17_r_a;
  int32_T c17_r_b;
  int32_T c17_s_a;
  int32_T c17_l_c;
  int32_T c17_s_b;
  int32_T c17_m_c;
  int32_T c17_t_a;
  int32_T c17_t_b;
  int32_T c17_qp1q;
  int32_T c17_h_qp1;
  boolean_T c17_j_overflow;
  int32_T c17_f_jj;
  int32_T c17_u_a;
  int32_T c17_n_c;
  int32_T c17_u_b;
  int32_T c17_o_c;
  int32_T c17_v_a;
  int32_T c17_v_b;
  int32_T c17_i892;
  real_T c17_b_Vf[36];
  int32_T c17_i893;
  real_T c17_c_Vf[36];
  int32_T c17_h_ii;
  int32_T c17_g_q;
  real_T c17_rt;
  real_T c17_r;
  int32_T c17_w_a;
  int32_T c17_p_c;
  int32_T c17_w_b;
  int32_T c17_q_c;
  int32_T c17_x_b;
  int32_T c17_colq;
  int32_T c17_x_a;
  int32_T c17_r_c;
  int32_T c17_y_a;
  int32_T c17_s_c;
  real_T c17_ab_a;
  real_T c17_y_b;
  real_T c17_c_y;
  int32_T c17_ab_b;
  int32_T c17_t_c;
  int32_T c17_bb_b;
  int32_T c17_colqp1;
  real_T c17_iter;
  real_T c17_tiny;
  real_T c17_snorm;
  int32_T c17_i_ii;
  real_T c17_varargin_1;
  real_T c17_varargin_2;
  real_T c17_b_varargin_2;
  real_T c17_varargin_3;
  real_T c17_x;
  real_T c17_d_y;
  real_T c17_b_x;
  real_T c17_e_y;
  real_T c17_xk;
  real_T c17_yk;
  real_T c17_c_x;
  real_T c17_f_y;
  real_T c17_maxval;
  real_T c17_b_varargin_1;
  real_T c17_c_varargin_2;
  real_T c17_d_varargin_2;
  real_T c17_b_varargin_3;
  real_T c17_d_x;
  real_T c17_g_y;
  real_T c17_e_x;
  real_T c17_h_y;
  real_T c17_b_xk;
  real_T c17_b_yk;
  real_T c17_f_x;
  real_T c17_i_y;
  int32_T c17_bb_a;
  int32_T c17_cb_a;
  int32_T c17_i894;
  boolean_T c17_k_overflow;
  int32_T c17_j_ii;
  int32_T c17_db_a;
  int32_T c17_u_c;
  real_T c17_test0;
  real_T c17_ztest0;
  real_T c17_cb_b;
  real_T c17_j_y;
  real_T c17_db_b;
  real_T c17_k_y;
  int32_T c17_eb_a;
  int32_T c17_v_c;
  real_T c17_kase;
  int32_T c17_qs;
  int32_T c17_b_m;
  int32_T c17_h_q;
  int32_T c17_fb_a;
  int32_T c17_eb_b;
  int32_T c17_gb_a;
  int32_T c17_fb_b;
  boolean_T c17_l_overflow;
  int32_T c17_k_ii;
  real_T c17_test;
  int32_T c17_hb_a;
  int32_T c17_w_c;
  int32_T c17_ib_a;
  int32_T c17_x_c;
  real_T c17_ztest;
  real_T c17_gb_b;
  real_T c17_l_y;
  int32_T c17_jb_a;
  int32_T c17_kb_a;
  int32_T c17_y_c;
  real_T c17_f;
  int32_T c17_lb_a;
  int32_T c17_ab_c;
  int32_T c17_mb_a;
  int32_T c17_i895;
  int32_T c17_i_q;
  int32_T c17_nb_a;
  int32_T c17_hb_b;
  int32_T c17_ob_a;
  int32_T c17_ib_b;
  boolean_T c17_m_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_t1;
  real_T c17_b_t1;
  real_T c17_b_f;
  real_T c17_sn;
  real_T c17_cs;
  real_T c17_b_cs;
  real_T c17_b_sn;
  int32_T c17_pb_a;
  int32_T c17_km1;
  real_T c17_qb_a;
  real_T c17_jb_b;
  real_T c17_rb_a;
  real_T c17_kb_b;
  real_T c17_m_y;
  int32_T c17_sb_a;
  int32_T c17_bb_c;
  int32_T c17_lb_b;
  int32_T c17_cb_c;
  int32_T c17_mb_b;
  int32_T c17_colk;
  int32_T c17_tb_a;
  int32_T c17_db_c;
  int32_T c17_nb_b;
  int32_T c17_eb_c;
  int32_T c17_ob_b;
  int32_T c17_colm;
  int32_T c17_ub_a;
  int32_T c17_j_q;
  int32_T c17_c_m;
  int32_T c17_vb_a;
  int32_T c17_pb_b;
  int32_T c17_wb_a;
  int32_T c17_qb_b;
  boolean_T c17_n_overflow;
  int32_T c17_c_k;
  real_T c17_c_t1;
  real_T c17_unusedU0;
  real_T c17_c_sn;
  real_T c17_c_cs;
  real_T c17_xb_a;
  real_T c17_rb_b;
  real_T c17_yb_a;
  real_T c17_sb_b;
  real_T c17_n_y;
  int32_T c17_ac_a;
  int32_T c17_fb_c;
  int32_T c17_tb_b;
  int32_T c17_gb_c;
  int32_T c17_ub_b;
  int32_T c17_bc_a;
  int32_T c17_hb_c;
  int32_T c17_vb_b;
  int32_T c17_ib_c;
  int32_T c17_wb_b;
  int32_T c17_colqm1;
  int32_T c17_cc_a;
  int32_T c17_mm1;
  real_T c17_d3;
  real_T c17_d4;
  real_T c17_d5;
  real_T c17_d6;
  real_T c17_d7;
  real_T c17_c_varargin_1[5];
  int32_T c17_ixstart;
  real_T c17_mtmp;
  real_T c17_g_x;
  boolean_T c17_xb_b;
  int32_T c17_ix;
  int32_T c17_b_ix;
  real_T c17_h_x;
  boolean_T c17_yb_b;
  int32_T c17_dc_a;
  int32_T c17_i896;
  boolean_T c17_o_overflow;
  int32_T c17_c_ix;
  real_T c17_ec_a;
  real_T c17_ac_b;
  boolean_T c17_p;
  real_T c17_b_mtmp;
  real_T c17_scale;
  real_T c17_sm;
  real_T c17_smm1;
  real_T c17_emm1;
  real_T c17_sqds;
  real_T c17_eqds;
  real_T c17_fc_a;
  real_T c17_bc_b;
  real_T c17_o_y;
  real_T c17_gc_a;
  real_T c17_cc_b;
  real_T c17_p_y;
  real_T c17_dc_b;
  real_T c17_hc_a;
  real_T c17_ec_b;
  real_T c17_jb_c;
  real_T c17_ic_a;
  real_T c17_fc_b;
  real_T c17_shift;
  real_T c17_jc_a;
  real_T c17_gc_b;
  real_T c17_q_y;
  real_T c17_kc_a;
  real_T c17_hc_b;
  real_T c17_r_y;
  real_T c17_lc_a;
  real_T c17_ic_b;
  real_T c17_g;
  int32_T c17_k_q;
  int32_T c17_b_mm1;
  int32_T c17_mc_a;
  int32_T c17_jc_b;
  int32_T c17_nc_a;
  int32_T c17_kc_b;
  boolean_T c17_p_overflow;
  int32_T c17_d_k;
  int32_T c17_oc_a;
  int32_T c17_pc_a;
  int32_T c17_kp1;
  real_T c17_c_f;
  real_T c17_unusedU1;
  real_T c17_d_sn;
  real_T c17_d_cs;
  real_T c17_qc_a;
  real_T c17_lc_b;
  real_T c17_s_y;
  real_T c17_rc_a;
  real_T c17_mc_b;
  real_T c17_t_y;
  real_T c17_sc_a;
  real_T c17_nc_b;
  real_T c17_u_y;
  real_T c17_tc_a;
  real_T c17_oc_b;
  real_T c17_v_y;
  real_T c17_uc_a;
  real_T c17_pc_b;
  real_T c17_vc_a;
  real_T c17_qc_b;
  real_T c17_w_y;
  int32_T c17_wc_a;
  int32_T c17_kb_c;
  int32_T c17_rc_b;
  int32_T c17_lb_c;
  int32_T c17_sc_b;
  int32_T c17_tc_b;
  int32_T c17_mb_c;
  int32_T c17_uc_b;
  int32_T c17_colkp1;
  real_T c17_d_f;
  real_T c17_unusedU2;
  real_T c17_e_sn;
  real_T c17_e_cs;
  real_T c17_xc_a;
  real_T c17_vc_b;
  real_T c17_x_y;
  real_T c17_yc_a;
  real_T c17_wc_b;
  real_T c17_y_y;
  real_T c17_ad_a;
  real_T c17_xc_b;
  real_T c17_ab_y;
  real_T c17_bd_a;
  real_T c17_yc_b;
  real_T c17_bb_y;
  real_T c17_cd_a;
  real_T c17_ad_b;
  real_T c17_dd_a;
  real_T c17_bd_b;
  real_T c17_cb_y;
  int32_T c17_ed_a;
  int32_T c17_nb_c;
  int32_T c17_cd_b;
  int32_T c17_ob_c;
  int32_T c17_dd_b;
  int32_T c17_ed_b;
  int32_T c17_pb_c;
  int32_T c17_fd_b;
  int32_T c17_fd_a;
  int32_T c17_qb_c;
  int32_T c17_e_k;
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_i;
  int32_T c17_b_i;
  int32_T c17_rb_c;
  int32_T c17_gd_a;
  int32_T c17_sb_c;
  int32_T c17_gd_b;
  int32_T c17_hd_b;
  int32_T c17_hd_a;
  int32_T c17_id_a;
  int32_T c17_tb_c;
  int32_T c17_jd_a;
  int32_T c17_ub_c;
  int32_T c17_id_b;
  int32_T c17_jd_b;
  int32_T c17_vb_c;
  int32_T c17_kd_b;
  int32_T c17_ld_b;
  int32_T c17_wb_c;
  int32_T c17_kd_a;
  int32_T c17_xb_c;
  int32_T c17_md_b;
  int32_T c17_nd_b;
  int32_T c17_yb_c;
  int32_T c17_od_b;
  int32_T c17_pd_b;
  int32_T c17_ld_a;
  real_T c17_d8;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard11 = FALSE;
  for (c17_i877 = 0; c17_i877 < 72; c17_i877++) {
    c17_b_A[c17_i877] = c17_A[c17_i877];
  }

  c17_c_eml_scalar_eg(chartInstance);
  for (c17_i878 = 0; c17_i878 < 6; c17_i878++) {
    c17_s[c17_i878] = 0.0;
  }

  for (c17_i879 = 0; c17_i879 < 6; c17_i879++) {
    c17_e[c17_i879] = 0.0;
  }

  for (c17_i880 = 0; c17_i880 < 12; c17_i880++) {
    c17_work[c17_i880] = 0.0;
  }

  for (c17_i881 = 0; c17_i881 < 72; c17_i881++) {
    c17_U[c17_i881] = 0.0;
  }

  for (c17_i882 = 0; c17_i882 < 36; c17_i882++) {
    c17_Vf[c17_i882] = 0.0;
  }

  for (c17_q = 1; c17_q < 7; c17_q++) {
    c17_b_q = c17_q;
    c17_a = c17_b_q + 1;
    c17_qp1 = c17_a;
    c17_b_a = c17_b_q;
    c17_qm1 = c17_b_a;
    c17_b = c17_qm1 - 1;
    c17_c = 12 * c17_b;
    c17_c_a = c17_b_q;
    c17_b_b = c17_c;
    c17_qq = c17_c_a + c17_b_b;
    c17_c_b = c17_b_q;
    c17_nmq = 12 - c17_c_b;
    c17_d_a = c17_nmq + 1;
    c17_nmqp1 = c17_d_a;
    if (c17_b_q <= 6) {
      for (c17_i883 = 0; c17_i883 < 72; c17_i883++) {
        c17_c_A[c17_i883] = c17_b_A[c17_i883];
      }

      c17_nrm = c17_eml_xnrm2(chartInstance, c17_nmqp1, c17_c_A, c17_qq);
      if (c17_nrm > 0.0) {
        c17_absx = c17_nrm;
        c17_d = c17_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_qq), 1, 72, 1, 0) - 1];
        if (c17_d < 0.0) {
          c17_y = -c17_absx;
        } else {
          c17_y = c17_absx;
        }

        c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] = c17_y;
        c17_d1 = c17_eml_div(chartInstance, 1.0,
                             c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1]);
        c17_h_eml_xscal(chartInstance, c17_nmqp1, c17_d1, c17_b_A, c17_qq);
        c17_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qq), 1, 72, 1, 0) - 1] =
          c17_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qq), 1, 72, 1, 0) - 1] + 1.0;
        c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] =
          -c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1];
      } else {
        c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] = 0.0;
      }
    }

    c17_b_qp1 = c17_qp1;
    c17_overflow = FALSE;
    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_jj = c17_b_qp1; c17_jj < 7; c17_jj++) {
      c17_b_jj = c17_jj;
      c17_e_a = c17_b_jj;
      c17_b_c = c17_e_a;
      c17_d_b = c17_b_c - 1;
      c17_c_c = 12 * c17_d_b;
      c17_f_a = c17_b_q;
      c17_e_b = c17_c_c;
      c17_qjj = c17_f_a + c17_e_b;
      if (c17_b_q <= 6) {
        if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 6, 1, 0) - 1] != 0.0) {
          for (c17_i884 = 0; c17_i884 < 72; c17_i884++) {
            c17_d_A[c17_i884] = c17_b_A[c17_i884];
          }

          for (c17_i885 = 0; c17_i885 < 72; c17_i885++) {
            c17_e_A[c17_i885] = c17_b_A[c17_i885];
          }

          c17_t = c17_eml_xdotc(chartInstance, c17_nmqp1, c17_d_A, c17_qq,
                                c17_e_A, c17_qjj);
          c17_t = -c17_eml_div(chartInstance, c17_t, c17_b_A
                               [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 12, 1, 0) + 12 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1]);
          c17_h_eml_xaxpy(chartInstance, c17_nmqp1, c17_t, c17_qq, c17_b_A,
                          c17_qjj);
        }
      }

      c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_b_jj), 1, 6, 1, 0) - 1] =
        c17_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_qjj), 1, 72, 1, 0) - 1];
    }

    if (c17_b_q <= 6) {
      c17_c_q = c17_b_q;
      c17_b_overflow = FALSE;
      if (c17_b_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
      }

      for (c17_ii = c17_c_q; c17_ii < 13; c17_ii++) {
        c17_b_ii = c17_ii;
        c17_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_ii), 1, 12, 1, 0) + 12 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1] = c17_b_A
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_ii), 1, 12, 1, 0) + 12 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1];
      }
    }

    if (c17_b_q <= 4) {
      c17_f_b = c17_b_q;
      c17_pmq = 6 - c17_f_b;
      for (c17_i886 = 0; c17_i886 < 6; c17_i886++) {
        c17_b_e[c17_i886] = c17_e[c17_i886];
      }

      c17_nrm = c17_b_eml_xnrm2(chartInstance, c17_pmq, c17_b_e, c17_qp1);
      if (c17_nrm == 0.0) {
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] = 0.0;
      } else {
        c17_b_absx = c17_nrm;
        c17_b_d = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_qp1), 1, 6, 1, 0) - 1];
        if (c17_b_d < 0.0) {
          c17_b_y = -c17_b_absx;
        } else {
          c17_b_y = c17_b_absx;
        }

        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] = c17_b_y;
        c17_d2 = c17_eml_div(chartInstance, 1.0,
                             c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1]);
        c17_i_eml_xscal(chartInstance, c17_pmq, c17_d2, c17_e, c17_qp1);
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qp1), 1, 6, 1, 0) - 1] = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_qp1), 1, 6, 1, 0) - 1]
          + 1.0;
      }

      c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_b_q), 1, 6, 1, 0) - 1] = -c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1];
      if (c17_qp1 <= 12) {
        if (c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 6, 1, 0) - 1] != 0.0) {
          c17_c_qp1 = c17_qp1;
          c17_c_overflow = FALSE;
          if (c17_c_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_c_overflow);
          }

          for (c17_c_ii = c17_c_qp1; c17_c_ii < 13; c17_c_ii++) {
            c17_b_ii = c17_c_ii;
            c17_work[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
              ("", (real_T)c17_b_ii), 1, 12, 1, 0) - 1] = 0.0;
          }

          c17_d_qp1 = c17_qp1;
          c17_d_overflow = FALSE;
          if (c17_d_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_d_overflow);
          }

          for (c17_c_jj = c17_d_qp1; c17_c_jj < 7; c17_c_jj++) {
            c17_b_jj = c17_c_jj;
            c17_g_a = c17_b_jj;
            c17_d_c = c17_g_a;
            c17_g_b = c17_d_c - 1;
            c17_e_c = 12 * c17_g_b;
            c17_h_a = c17_qp1;
            c17_h_b = c17_e_c;
            c17_qp1jj = c17_h_a + c17_h_b;
            for (c17_i887 = 0; c17_i887 < 72; c17_i887++) {
              c17_f_A[c17_i887] = c17_b_A[c17_i887];
            }

            c17_i_eml_xaxpy(chartInstance, c17_nmq,
                            c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_jj), 1, 6, 1, 0) - 1],
                            c17_f_A, c17_qp1jj, c17_work, c17_qp1);
          }

          c17_e_qp1 = c17_qp1;
          c17_e_overflow = FALSE;
          if (c17_e_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_e_overflow);
          }

          for (c17_d_jj = c17_e_qp1; c17_d_jj < 7; c17_d_jj++) {
            c17_b_jj = c17_d_jj;
            c17_t = c17_eml_div(chartInstance,
                                -c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_jj), 1, 6, 1, 0) - 1],
                                c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_qp1), 1, 6, 1, 0) - 1]);
            c17_i_a = c17_b_jj;
            c17_f_c = c17_i_a;
            c17_i_b = c17_f_c - 1;
            c17_g_c = 12 * c17_i_b;
            c17_j_a = c17_qp1;
            c17_j_b = c17_g_c;
            c17_qp1jj = c17_j_a + c17_j_b;
            for (c17_i888 = 0; c17_i888 < 12; c17_i888++) {
              c17_b_work[c17_i888] = c17_work[c17_i888];
            }

            c17_j_eml_xaxpy(chartInstance, c17_nmq, c17_t, c17_b_work, c17_qp1,
                            c17_b_A, c17_qp1jj);
          }
        }
      }

      c17_f_qp1 = c17_qp1;
      c17_f_overflow = FALSE;
      if (c17_f_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_f_overflow);
      }

      for (c17_d_ii = c17_f_qp1; c17_d_ii < 7; c17_d_ii++) {
        c17_b_ii = c17_d_ii;
        c17_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_ii), 1, 6, 1, 0) + 6 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1] =
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_ii), 1, 6, 1, 0) - 1];
      }
    }
  }

  c17_m = 6;
  c17_e[4] = c17_b_A[64];
  c17_e[5] = 0.0;
  for (c17_d_q = 6; c17_d_q > 0; c17_d_q--) {
    c17_b_q = c17_d_q;
    c17_k_a = c17_b_q;
    c17_qp1 = c17_k_a;
    c17_k_b = c17_b_q;
    c17_nmq = 12 - c17_k_b;
    c17_l_a = c17_nmq + 1;
    c17_nmqp1 = c17_l_a;
    c17_m_a = c17_b_q;
    c17_h_c = c17_m_a;
    c17_l_b = c17_h_c - 1;
    c17_i_c = 12 * c17_l_b;
    c17_n_a = c17_b_q;
    c17_m_b = c17_i_c;
    c17_qq = c17_n_a + c17_m_b;
    if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] != 0.0) {
      c17_g_qp1 = c17_qp1 + 1;
      c17_g_overflow = FALSE;
      if (c17_g_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_g_overflow);
      }

      for (c17_e_jj = c17_g_qp1; c17_e_jj < 7; c17_e_jj++) {
        c17_b_jj = c17_e_jj;
        c17_o_a = c17_b_jj;
        c17_j_c = c17_o_a;
        c17_n_b = c17_j_c - 1;
        c17_k_c = 12 * c17_n_b;
        c17_p_a = c17_b_q;
        c17_o_b = c17_k_c;
        c17_qjj = c17_p_a + c17_o_b;
        for (c17_i889 = 0; c17_i889 < 72; c17_i889++) {
          c17_b_U[c17_i889] = c17_U[c17_i889];
        }

        for (c17_i890 = 0; c17_i890 < 72; c17_i890++) {
          c17_c_U[c17_i890] = c17_U[c17_i890];
        }

        c17_t = c17_eml_xdotc(chartInstance, c17_nmqp1, c17_b_U, c17_qq, c17_c_U,
                              c17_qjj);
        c17_t = -c17_eml_div(chartInstance, c17_t,
                             c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_qq), 1, 72, 1, 0) - 1]);
        c17_h_eml_xaxpy(chartInstance, c17_nmqp1, c17_t, c17_qq, c17_U, c17_qjj);
      }

      c17_e_q = c17_b_q;
      c17_h_overflow = FALSE;
      if (c17_h_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_h_overflow);
      }

      for (c17_e_ii = c17_e_q; c17_e_ii < 13; c17_e_ii++) {
        c17_b_ii = c17_e_ii;
        c17_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_ii), 1, 12, 1, 0) + 12 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1] = -c17_U
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_ii), 1, 12, 1, 0) + 12 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1];
      }

      c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_qq), 1, 72, 1, 0) - 1] = c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_qq), 1, 72, 1, 0) - 1] +
        1.0;
      c17_q_a = c17_b_q - 1;
      c17_i891 = c17_q_a;
      c17_p_b = c17_i891;
      c17_q_b = c17_p_b;
      if (1 > c17_q_b) {
        c17_i_overflow = FALSE;
      } else {
        c17_i_overflow = (c17_q_b > 2147483646);
      }

      if (c17_i_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_i_overflow);
      }

      for (c17_f_ii = 1; c17_f_ii <= c17_i891; c17_f_ii++) {
        c17_b_ii = c17_f_ii;
        c17_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_ii), 1, 12, 1, 0) + 12 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
      }
    } else {
      for (c17_g_ii = 1; c17_g_ii < 13; c17_g_ii++) {
        c17_b_ii = c17_g_ii;
        c17_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_ii), 1, 12, 1, 0) + 12 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
      }

      c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_qq), 1, 72, 1, 0) - 1] = 1.0;
    }
  }

  for (c17_f_q = 6; c17_f_q > 0; c17_f_q--) {
    c17_b_q = c17_f_q;
    if (c17_b_q <= 4) {
      if (c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_q), 1, 6, 1, 0) - 1] != 0.0) {
        c17_r_a = c17_b_q + 1;
        c17_qp1 = c17_r_a;
        c17_r_b = c17_b_q;
        c17_pmq = 6 - c17_r_b;
        c17_s_a = c17_b_q;
        c17_l_c = c17_s_a;
        c17_s_b = c17_l_c - 1;
        c17_m_c = 6 * c17_s_b;
        c17_t_a = c17_qp1;
        c17_t_b = c17_m_c;
        c17_qp1q = c17_t_a + c17_t_b;
        c17_h_qp1 = c17_qp1;
        c17_j_overflow = FALSE;
        if (c17_j_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_j_overflow);
        }

        for (c17_f_jj = c17_h_qp1; c17_f_jj < 7; c17_f_jj++) {
          c17_b_jj = c17_f_jj;
          c17_u_a = c17_b_jj;
          c17_n_c = c17_u_a;
          c17_u_b = c17_n_c - 1;
          c17_o_c = 6 * c17_u_b;
          c17_v_a = c17_qp1;
          c17_v_b = c17_o_c;
          c17_qp1jj = c17_v_a + c17_v_b;
          for (c17_i892 = 0; c17_i892 < 36; c17_i892++) {
            c17_b_Vf[c17_i892] = c17_Vf[c17_i892];
          }

          for (c17_i893 = 0; c17_i893 < 36; c17_i893++) {
            c17_c_Vf[c17_i893] = c17_Vf[c17_i893];
          }

          c17_t = c17_b_eml_xdotc(chartInstance, c17_pmq, c17_b_Vf, c17_qp1q,
            c17_c_Vf, c17_qp1jj);
          c17_t = -c17_eml_div(chartInstance, c17_t,
                               c17_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_qp1q), 1, 36, 1, 0) - 1]);
          c17_k_eml_xaxpy(chartInstance, c17_pmq, c17_t, c17_qp1q, c17_Vf,
                          c17_qp1jj);
        }
      }
    }

    for (c17_h_ii = 1; c17_h_ii < 7; c17_h_ii++) {
      c17_b_ii = c17_h_ii;
      c17_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c17_b_ii), 1, 6, 1, 0) + 6 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
    }

    c17_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 2, 0)
             - 1)) - 1] = 1.0;
  }

  for (c17_g_q = 1; c17_g_q < 7; c17_g_q++) {
    c17_b_q = c17_g_q;
    if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] != 0.0) {
      c17_rt = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1]);
      c17_r = c17_eml_div(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1],
                          c17_rt);
      c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_b_q), 1, 6, 1, 0) - 1] = c17_rt;
      if (c17_b_q < 6) {
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] = c17_eml_div(chartInstance,
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1], c17_r);
      }

      if (c17_b_q <= 12) {
        c17_w_a = c17_b_q;
        c17_p_c = c17_w_a;
        c17_w_b = c17_p_c - 1;
        c17_q_c = 12 * c17_w_b;
        c17_x_b = c17_q_c;
        c17_colq = c17_x_b;
        c17_j_eml_xscal(chartInstance, c17_r, c17_U, c17_colq + 1);
      }
    }

    if (c17_b_q < 6) {
      if (c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_q), 1, 6, 1, 0) - 1] != 0.0) {
        c17_rt = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1]);
        c17_r = c17_eml_div(chartInstance, c17_rt,
                            c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1]);
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 6, 1, 0) - 1] = c17_rt;
        c17_x_a = c17_b_q;
        c17_r_c = c17_x_a;
        c17_y_a = c17_b_q;
        c17_s_c = c17_y_a;
        c17_ab_a = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)(c17_s_c + 1)), 1, 6, 1, 0) - 1];
        c17_y_b = c17_r;
        c17_c_y = c17_ab_a * c17_y_b;
        c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c17_r_c + 1)), 1, 6, 1, 0) - 1] = c17_c_y;
        c17_ab_b = c17_b_q;
        c17_t_c = 6 * c17_ab_b;
        c17_bb_b = c17_t_c;
        c17_colqp1 = c17_bb_b;
        c17_k_eml_xscal(chartInstance, c17_r, c17_Vf, c17_colqp1 + 1);
      }
    }
  }

  c17_iter = 0.0;
  c17_realmin(chartInstance);
  c17_eps(chartInstance);
  c17_tiny = c17_eml_div(chartInstance, 2.2250738585072014E-308,
    2.2204460492503131E-16);
  c17_snorm = 0.0;
  for (c17_i_ii = 1; c17_i_ii < 7; c17_i_ii++) {
    c17_b_ii = c17_i_ii;
    c17_varargin_1 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ii),
      1, 6, 1, 0) - 1]);
    c17_varargin_2 = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ii),
      1, 6, 1, 0) - 1]);
    c17_b_varargin_2 = c17_varargin_1;
    c17_varargin_3 = c17_varargin_2;
    c17_x = c17_b_varargin_2;
    c17_d_y = c17_varargin_3;
    c17_b_x = c17_x;
    c17_e_y = c17_d_y;
    c17_eml_scalar_eg(chartInstance);
    c17_xk = c17_b_x;
    c17_yk = c17_e_y;
    c17_c_x = c17_xk;
    c17_f_y = c17_yk;
    c17_eml_scalar_eg(chartInstance);
    c17_maxval = muDoubleScalarMax(c17_c_x, c17_f_y);
    c17_b_varargin_1 = c17_snorm;
    c17_c_varargin_2 = c17_maxval;
    c17_d_varargin_2 = c17_b_varargin_1;
    c17_b_varargin_3 = c17_c_varargin_2;
    c17_d_x = c17_d_varargin_2;
    c17_g_y = c17_b_varargin_3;
    c17_e_x = c17_d_x;
    c17_h_y = c17_g_y;
    c17_eml_scalar_eg(chartInstance);
    c17_b_xk = c17_e_x;
    c17_b_yk = c17_h_y;
    c17_f_x = c17_b_xk;
    c17_i_y = c17_b_yk;
    c17_eml_scalar_eg(chartInstance);
    c17_snorm = muDoubleScalarMax(c17_f_x, c17_i_y);
  }

  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c17_m > 0)) {
    if (c17_iter >= 75.0) {
      c17_b_eml_error(chartInstance);
      exitg1 = TRUE;
    } else {
      c17_bb_a = c17_m - 1;
      c17_b_q = c17_bb_a;
      c17_cb_a = c17_m;
      c17_i894 = c17_cb_a;
      c17_k_overflow = FALSE;
      if (c17_k_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_k_overflow);
      }

      c17_j_ii = c17_i894 - 1;
      guard3 = FALSE;
      guard4 = FALSE;
      exitg5 = FALSE;
      while ((exitg5 == FALSE) && (c17_j_ii > -1)) {
        c17_b_ii = c17_j_ii;
        c17_b_q = c17_b_ii;
        if (c17_b_ii == 0) {
          exitg5 = TRUE;
        } else {
          c17_db_a = c17_b_ii;
          c17_u_c = c17_db_a;
          c17_test0 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ii), 1, 6, 1, 0) -
                              1]) + c17_abs(chartInstance,
            c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c17_u_c + 1)), 1, 6, 1, 0) - 1]);
          c17_ztest0 = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ii), 1, 6, 1, 0) -
                               1]);
          c17_eps(chartInstance);
          c17_cb_b = c17_test0;
          c17_j_y = 2.2204460492503131E-16 * c17_cb_b;
          if (c17_ztest0 <= c17_j_y) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else if (c17_ztest0 <= c17_tiny) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else {
            guard11 = FALSE;
            if (c17_iter > 20.0) {
              c17_eps(chartInstance);
              c17_db_b = c17_snorm;
              c17_k_y = 2.2204460492503131E-16 * c17_db_b;
              if (c17_ztest0 <= c17_k_y) {
                guard3 = TRUE;
                exitg5 = TRUE;
              } else {
                guard11 = TRUE;
              }
            } else {
              guard11 = TRUE;
            }

            if (guard11 == TRUE) {
              c17_j_ii--;
              guard3 = FALSE;
              guard4 = FALSE;
            }
          }
        }
      }

      if (guard4 == TRUE) {
        guard3 = TRUE;
      }

      if (guard3 == TRUE) {
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_ii), 1, 6, 1, 0) - 1] = 0.0;
      }

      c17_eb_a = c17_m;
      c17_v_c = c17_eb_a;
      if (c17_b_q == c17_v_c - 1) {
        c17_kase = 4.0;
      } else {
        c17_qs = c17_m;
        c17_b_m = c17_m;
        c17_h_q = c17_b_q;
        c17_fb_a = c17_b_m;
        c17_eb_b = c17_h_q;
        c17_gb_a = c17_fb_a;
        c17_fb_b = c17_eb_b;
        if (c17_gb_a < c17_fb_b) {
          c17_l_overflow = FALSE;
        } else {
          c17_l_overflow = (c17_fb_b < -2147483647);
        }

        if (c17_l_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_l_overflow);
        }

        c17_k_ii = c17_b_m;
        guard2 = FALSE;
        exitg4 = FALSE;
        while ((exitg4 == FALSE) && (c17_k_ii >= c17_h_q)) {
          c17_b_ii = c17_k_ii;
          c17_qs = c17_b_ii;
          if (c17_b_ii == c17_b_q) {
            exitg4 = TRUE;
          } else {
            c17_test = 0.0;
            if (c17_b_ii < c17_m) {
              c17_test = c17_abs(chartInstance,
                                 c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c17_b_ii), 1, 6, 1, 0) - 1]);
            }

            c17_hb_a = c17_b_q;
            c17_w_c = c17_hb_a;
            if (c17_b_ii > c17_w_c + 1) {
              c17_ib_a = c17_b_ii;
              c17_x_c = c17_ib_a;
              c17_test += c17_abs(chartInstance,
                                  c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)(c17_x_c - 1)), 1, 6, 1, 0) - 1]);
            }

            c17_ztest = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK
                                ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
              c17_b_ii), 1, 6, 1, 0) - 1]);
            c17_eps(chartInstance);
            c17_gb_b = c17_test;
            c17_l_y = 2.2204460492503131E-16 * c17_gb_b;
            if (c17_ztest <= c17_l_y) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else if (c17_ztest <= c17_tiny) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else {
              c17_k_ii--;
              guard2 = FALSE;
            }
          }
        }

        if (guard2 == TRUE) {
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ii), 1, 6, 1, 0) - 1] = 0.0;
        }

        if (c17_qs == c17_b_q) {
          c17_kase = 3.0;
        } else if (c17_qs == c17_m) {
          c17_kase = 1.0;
        } else {
          c17_kase = 2.0;
          c17_b_q = c17_qs;
        }
      }

      c17_jb_a = c17_b_q + 1;
      c17_b_q = c17_jb_a;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c17_kase)) {
       case 1:
        c17_kb_a = c17_m;
        c17_y_c = c17_kb_a;
        c17_f = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)(c17_y_c - 1)), 1, 6, 1, 0) - 1];
        c17_lb_a = c17_m;
        c17_ab_c = c17_lb_a;
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c17_ab_c - 1)), 1, 6, 1, 0) - 1] = 0.0;
        c17_mb_a = c17_m - 1;
        c17_i895 = c17_mb_a;
        c17_i_q = c17_b_q;
        c17_nb_a = c17_i895;
        c17_hb_b = c17_i_q;
        c17_ob_a = c17_nb_a;
        c17_ib_b = c17_hb_b;
        if (c17_ob_a < c17_ib_b) {
          c17_m_overflow = FALSE;
        } else {
          c17_m_overflow = (c17_ib_b < -2147483647);
        }

        if (c17_m_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_m_overflow);
        }

        for (c17_k = c17_i895; c17_k >= c17_i_q; c17_k--) {
          c17_b_k = c17_k;
          c17_t1 = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_b_t1 = c17_t1;
          c17_b_f = c17_f;
          c17_b_eml_xrotg(chartInstance, &c17_b_t1, &c17_b_f, &c17_cs, &c17_sn);
          c17_t1 = c17_b_t1;
          c17_f = c17_b_f;
          c17_b_cs = c17_cs;
          c17_b_sn = c17_sn;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 6, 1, 0) - 1] = c17_t1;
          if (c17_b_k > c17_b_q) {
            c17_pb_a = c17_b_k - 1;
            c17_km1 = c17_pb_a;
            c17_qb_a = -c17_b_sn;
            c17_jb_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_km1), 1, 6, 1, 0) - 1];
            c17_f = c17_qb_a * c17_jb_b;
            c17_rb_a = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_km1), 1, 6, 1, 0) - 1];
            c17_kb_b = c17_b_cs;
            c17_m_y = c17_rb_a * c17_kb_b;
            c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_km1), 1, 6, 1, 0) - 1] = c17_m_y;
          }

          c17_sb_a = c17_b_k;
          c17_bb_c = c17_sb_a;
          c17_lb_b = c17_bb_c - 1;
          c17_cb_c = 6 * c17_lb_b;
          c17_mb_b = c17_cb_c;
          c17_colk = c17_mb_b;
          c17_tb_a = c17_m;
          c17_db_c = c17_tb_a;
          c17_nb_b = c17_db_c - 1;
          c17_eb_c = 6 * c17_nb_b;
          c17_ob_b = c17_eb_c;
          c17_colm = c17_ob_b;
          c17_d_eml_xrot(chartInstance, c17_Vf, c17_colk + 1, c17_colm + 1,
                         c17_b_cs, c17_b_sn);
        }
        break;

       case 2:
        c17_ub_a = c17_b_q - 1;
        c17_qm1 = c17_ub_a;
        c17_f = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_qm1), 1, 6, 1, 0) - 1];
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qm1), 1, 6, 1, 0) - 1] = 0.0;
        c17_j_q = c17_b_q;
        c17_c_m = c17_m;
        c17_vb_a = c17_j_q;
        c17_pb_b = c17_c_m;
        c17_wb_a = c17_vb_a;
        c17_qb_b = c17_pb_b;
        if (c17_wb_a > c17_qb_b) {
          c17_n_overflow = FALSE;
        } else {
          c17_n_overflow = (c17_qb_b > 2147483646);
        }

        if (c17_n_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_n_overflow);
        }

        for (c17_c_k = c17_j_q; c17_c_k <= c17_c_m; c17_c_k++) {
          c17_b_k = c17_c_k;
          c17_t1 = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_c_t1 = c17_t1;
          c17_unusedU0 = c17_f;
          c17_b_eml_xrotg(chartInstance, &c17_c_t1, &c17_unusedU0, &c17_c_cs,
                          &c17_c_sn);
          c17_t1 = c17_c_t1;
          c17_b_cs = c17_c_cs;
          c17_b_sn = c17_c_sn;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 6, 1, 0) - 1] = c17_t1;
          c17_xb_a = -c17_b_sn;
          c17_rb_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_f = c17_xb_a * c17_rb_b;
          c17_yb_a = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_sb_b = c17_b_cs;
          c17_n_y = c17_yb_a * c17_sb_b;
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 6, 1, 0) - 1] = c17_n_y;
          c17_ac_a = c17_b_k;
          c17_fb_c = c17_ac_a;
          c17_tb_b = c17_fb_c - 1;
          c17_gb_c = 12 * c17_tb_b;
          c17_ub_b = c17_gb_c;
          c17_colk = c17_ub_b;
          c17_bc_a = c17_qm1;
          c17_hb_c = c17_bc_a;
          c17_vb_b = c17_hb_c - 1;
          c17_ib_c = 12 * c17_vb_b;
          c17_wb_b = c17_ib_c;
          c17_colqm1 = c17_wb_b;
          c17_e_eml_xrot(chartInstance, c17_U, c17_colk + 1, c17_colqm1 + 1,
                         c17_b_cs, c17_b_sn);
        }
        break;

       case 3:
        c17_cc_a = c17_m - 1;
        c17_mm1 = c17_cc_a;
        c17_d3 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_m), 1, 6, 1, 0) - 1]);
        c17_d4 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_mm1), 1, 6, 1, 0) - 1]);
        c17_d5 = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_mm1), 1, 6, 1, 0) - 1]);
        c17_d6 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1]);
        c17_d7 = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1]);
        c17_c_varargin_1[0] = c17_d3;
        c17_c_varargin_1[1] = c17_d4;
        c17_c_varargin_1[2] = c17_d5;
        c17_c_varargin_1[3] = c17_d6;
        c17_c_varargin_1[4] = c17_d7;
        c17_ixstart = 1;
        c17_mtmp = c17_c_varargin_1[0];
        c17_g_x = c17_mtmp;
        c17_xb_b = muDoubleScalarIsNaN(c17_g_x);
        if (c17_xb_b) {
          c17_ix = 2;
          exitg2 = FALSE;
          while ((exitg2 == FALSE) && (c17_ix < 6)) {
            c17_b_ix = c17_ix;
            c17_ixstart = c17_b_ix;
            c17_h_x = c17_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 5, 1, 0) - 1];
            c17_yb_b = muDoubleScalarIsNaN(c17_h_x);
            if (!c17_yb_b) {
              c17_mtmp = c17_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 5, 1, 0) -
                1];
              exitg2 = TRUE;
            } else {
              c17_ix++;
            }
          }
        }

        if (c17_ixstart < 5) {
          c17_dc_a = c17_ixstart;
          c17_i896 = c17_dc_a;
          c17_o_overflow = FALSE;
          if (c17_o_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_o_overflow);
          }

          for (c17_c_ix = c17_i896 + 1; c17_c_ix < 6; c17_c_ix++) {
            c17_b_ix = c17_c_ix;
            c17_ec_a = c17_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 5, 1, 0) - 1];
            c17_ac_b = c17_mtmp;
            c17_p = (c17_ec_a > c17_ac_b);
            if (c17_p) {
              c17_mtmp = c17_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 5, 1, 0) -
                1];
            }
          }
        }

        c17_b_mtmp = c17_mtmp;
        c17_scale = c17_b_mtmp;
        c17_sm = c17_eml_div(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_m), 1,
          6, 1, 0) - 1], c17_scale);
        c17_smm1 = c17_eml_div(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_mm1), 1, 6, 1, 0) - 1],
          c17_scale);
        c17_emm1 = c17_eml_div(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_mm1), 1, 6, 1, 0) - 1],
          c17_scale);
        c17_sqds = c17_eml_div(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1],
          c17_scale);
        c17_eqds = c17_eml_div(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1],
          c17_scale);
        c17_fc_a = c17_smm1 + c17_sm;
        c17_bc_b = c17_smm1 - c17_sm;
        c17_o_y = c17_fc_a * c17_bc_b;
        c17_gc_a = c17_emm1;
        c17_cc_b = c17_emm1;
        c17_p_y = c17_gc_a * c17_cc_b;
        c17_dc_b = c17_eml_div(chartInstance, c17_o_y + c17_p_y, 2.0);
        c17_hc_a = c17_sm;
        c17_ec_b = c17_emm1;
        c17_jb_c = c17_hc_a * c17_ec_b;
        c17_ic_a = c17_jb_c;
        c17_fc_b = c17_jb_c;
        c17_jb_c = c17_ic_a * c17_fc_b;
        c17_shift = 0.0;
        guard1 = FALSE;
        if (c17_dc_b != 0.0) {
          guard1 = TRUE;
        } else {
          if (c17_jb_c != 0.0) {
            guard1 = TRUE;
          }
        }

        if (guard1 == TRUE) {
          c17_jc_a = c17_dc_b;
          c17_gc_b = c17_dc_b;
          c17_q_y = c17_jc_a * c17_gc_b;
          c17_shift = c17_q_y + c17_jb_c;
          c17_b_sqrt(chartInstance, &c17_shift);
          if (c17_dc_b < 0.0) {
            c17_shift = -c17_shift;
          }

          c17_shift = c17_eml_div(chartInstance, c17_jb_c, c17_dc_b + c17_shift);
        }

        c17_kc_a = c17_sqds + c17_sm;
        c17_hc_b = c17_sqds - c17_sm;
        c17_r_y = c17_kc_a * c17_hc_b;
        c17_f = c17_r_y + c17_shift;
        c17_lc_a = c17_sqds;
        c17_ic_b = c17_eqds;
        c17_g = c17_lc_a * c17_ic_b;
        c17_k_q = c17_b_q;
        c17_b_mm1 = c17_mm1;
        c17_mc_a = c17_k_q;
        c17_jc_b = c17_b_mm1;
        c17_nc_a = c17_mc_a;
        c17_kc_b = c17_jc_b;
        if (c17_nc_a > c17_kc_b) {
          c17_p_overflow = FALSE;
        } else {
          c17_p_overflow = (c17_kc_b > 2147483646);
        }

        if (c17_p_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_p_overflow);
        }

        for (c17_d_k = c17_k_q; c17_d_k <= c17_b_mm1; c17_d_k++) {
          c17_b_k = c17_d_k;
          c17_oc_a = c17_b_k;
          c17_km1 = c17_oc_a;
          c17_pc_a = c17_b_k + 1;
          c17_kp1 = c17_pc_a;
          c17_c_f = c17_f;
          c17_unusedU1 = c17_g;
          c17_b_eml_xrotg(chartInstance, &c17_c_f, &c17_unusedU1, &c17_d_cs,
                          &c17_d_sn);
          c17_f = c17_c_f;
          c17_b_cs = c17_d_cs;
          c17_b_sn = c17_d_sn;
          if (c17_b_k > c17_b_q) {
            c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)(c17_km1 - 1)), 1, 6, 1, 0) - 1] = c17_f;
          }

          c17_qc_a = c17_b_cs;
          c17_lc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_s_y = c17_qc_a * c17_lc_b;
          c17_rc_a = c17_b_sn;
          c17_mc_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_t_y = c17_rc_a * c17_mc_b;
          c17_f = c17_s_y + c17_t_y;
          c17_sc_a = c17_b_cs;
          c17_nc_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_u_y = c17_sc_a * c17_nc_b;
          c17_tc_a = c17_b_sn;
          c17_oc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_v_y = c17_tc_a * c17_oc_b;
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 6, 1, 0) - 1] = c17_u_y - c17_v_y;
          c17_uc_a = c17_b_sn;
          c17_pc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 6, 1, 0) - 1];
          c17_g = c17_uc_a * c17_pc_b;
          c17_vc_a = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 6, 1, 0) - 1];
          c17_qc_b = c17_b_cs;
          c17_w_y = c17_vc_a * c17_qc_b;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_kp1), 1, 6, 1, 0) - 1] = c17_w_y;
          c17_wc_a = c17_b_k;
          c17_kb_c = c17_wc_a;
          c17_rc_b = c17_kb_c - 1;
          c17_lb_c = 6 * c17_rc_b;
          c17_sc_b = c17_lb_c;
          c17_colk = c17_sc_b;
          c17_tc_b = c17_b_k;
          c17_mb_c = 6 * c17_tc_b;
          c17_uc_b = c17_mb_c;
          c17_colkp1 = c17_uc_b;
          c17_d_eml_xrot(chartInstance, c17_Vf, c17_colk + 1, c17_colkp1 + 1,
                         c17_b_cs, c17_b_sn);
          c17_d_f = c17_f;
          c17_unusedU2 = c17_g;
          c17_b_eml_xrotg(chartInstance, &c17_d_f, &c17_unusedU2, &c17_e_cs,
                          &c17_e_sn);
          c17_f = c17_d_f;
          c17_b_cs = c17_e_cs;
          c17_b_sn = c17_e_sn;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 6, 1, 0) - 1] = c17_f;
          c17_xc_a = c17_b_cs;
          c17_vc_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_x_y = c17_xc_a * c17_vc_b;
          c17_yc_a = c17_b_sn;
          c17_wc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 6, 1, 0) - 1];
          c17_y_y = c17_yc_a * c17_wc_b;
          c17_f = c17_x_y + c17_y_y;
          c17_ad_a = -c17_b_sn;
          c17_xc_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
          c17_ab_y = c17_ad_a * c17_xc_b;
          c17_bd_a = c17_b_cs;
          c17_yc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 6, 1, 0) - 1];
          c17_bb_y = c17_bd_a * c17_yc_b;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_kp1), 1, 6, 1, 0) - 1] = c17_ab_y + c17_bb_y;
          c17_cd_a = c17_b_sn;
          c17_ad_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 6, 1, 0) - 1];
          c17_g = c17_cd_a * c17_ad_b;
          c17_dd_a = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 6, 1, 0) - 1];
          c17_bd_b = c17_b_cs;
          c17_cb_y = c17_dd_a * c17_bd_b;
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_kp1), 1, 6, 1, 0) - 1] = c17_cb_y;
          if (c17_b_k < 12) {
            c17_ed_a = c17_b_k;
            c17_nb_c = c17_ed_a;
            c17_cd_b = c17_nb_c - 1;
            c17_ob_c = 12 * c17_cd_b;
            c17_dd_b = c17_ob_c;
            c17_colk = c17_dd_b;
            c17_ed_b = c17_b_k;
            c17_pb_c = 12 * c17_ed_b;
            c17_fd_b = c17_pb_c;
            c17_colkp1 = c17_fd_b;
            c17_e_eml_xrot(chartInstance, c17_U, c17_colk + 1, c17_colkp1 + 1,
                           c17_b_cs, c17_b_sn);
          }
        }

        c17_fd_a = c17_m;
        c17_qb_c = c17_fd_a;
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c17_qb_c - 1)), 1, 6, 1, 0) - 1] = c17_f;
        c17_iter++;
        break;

       default:
        if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 6, 1, 0) - 1] < 0.0) {
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_q), 1, 6, 1, 0) - 1] =
            -c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c17_b_q), 1, 6, 1, 0) - 1];
          c17_gd_a = c17_b_q;
          c17_rb_c = c17_gd_a;
          c17_gd_b = c17_rb_c - 1;
          c17_sb_c = 6 * c17_gd_b;
          c17_hd_b = c17_sb_c;
          c17_colq = c17_hd_b;
          c17_e_eml_scalar_eg(chartInstance);
          c17_d8 = -1.0;
          c17_k_eml_xscal(chartInstance, c17_d8, c17_Vf, c17_colq + 1);
        }

        c17_hd_a = c17_b_q + 1;
        c17_qp1 = c17_hd_a;
        exitg3 = FALSE;
        while ((exitg3 == FALSE) && (c17_b_q < 6)) {
          if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c17_b_q), 1, 6, 1, 0) - 1] <
              c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c17_qp1), 1, 6, 1, 0) - 1]) {
            c17_rt = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 6, 1, 0) - 1];
            c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 6, 1, 0) - 1] =
              c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c17_qp1), 1, 6, 1, 0) - 1];
            c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_qp1), 1, 6, 1, 0) - 1] = c17_rt;
            if (c17_b_q < 6) {
              c17_jd_a = c17_b_q;
              c17_tb_c = c17_jd_a;
              c17_id_b = c17_tb_c - 1;
              c17_ub_c = 6 * c17_id_b;
              c17_jd_b = c17_ub_c;
              c17_colq = c17_jd_b;
              c17_kd_b = c17_b_q;
              c17_vb_c = 6 * c17_kd_b;
              c17_ld_b = c17_vb_c;
              c17_colqp1 = c17_ld_b;
              c17_e_eml_xswap(chartInstance, c17_Vf, c17_colq + 1, c17_colqp1 +
                              1);
            }

            if (c17_b_q < 12) {
              c17_kd_a = c17_b_q;
              c17_wb_c = c17_kd_a;
              c17_md_b = c17_wb_c - 1;
              c17_xb_c = 12 * c17_md_b;
              c17_nd_b = c17_xb_c;
              c17_colq = c17_nd_b;
              c17_od_b = c17_b_q;
              c17_yb_c = 12 * c17_od_b;
              c17_pd_b = c17_yb_c;
              c17_colqp1 = c17_pd_b;
              c17_f_eml_xswap(chartInstance, c17_U, c17_colq + 1, c17_colqp1 + 1);
            }

            c17_b_q = c17_qp1;
            c17_ld_a = c17_b_q + 1;
            c17_qp1 = c17_ld_a;
          } else {
            exitg3 = TRUE;
          }
        }

        c17_iter = 0.0;
        c17_id_a = c17_m - 1;
        c17_m = c17_id_a;
        break;
      }
    }
  }

  for (c17_e_k = 1; c17_e_k < 7; c17_e_k++) {
    c17_b_k = c17_e_k;
    c17_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 6, 1, 0) - 1] = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
  }

  for (c17_j = 1; c17_j < 7; c17_j++) {
    c17_b_j = c17_j;
    for (c17_i = 1; c17_i < 7; c17_i++) {
      c17_b_i = c17_i;
      c17_V[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c17_b_i), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 6, 2, 0)
              - 1)) - 1] = c17_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 6, 1, 0) + 6 *
        (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
        c17_b_j), 1, 6, 2, 0) - 1)) - 1];
    }
  }
}

static real_T c17_eml_xnrm2(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[72], int32_T c17_ix0)
{
  real_T c17_y;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_scale;
  int32_T c17_kstart;
  int32_T c17_a;
  int32_T c17_c;
  int32_T c17_b_a;
  int32_T c17_b_c;
  int32_T c17_c_a;
  int32_T c17_b;
  int32_T c17_kend;
  int32_T c17_b_kstart;
  int32_T c17_b_kend;
  int32_T c17_d_a;
  int32_T c17_b_b;
  int32_T c17_e_a;
  int32_T c17_c_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_d_x;
  real_T c17_e_x;
  real_T c17_absxk;
  real_T c17_t;
  c17_b_n = c17_n;
  c17_b_ix0 = c17_ix0;
  c17_c_n = c17_b_n;
  c17_c_ix0 = c17_b_ix0;
  c17_y = 0.0;
  if (c17_c_n < 1) {
  } else if (c17_c_n == 1) {
    c17_b_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_c_ix0), 1, 72, 1, 0) - 1];
    c17_c_x = c17_b_x;
    c17_y = muDoubleScalarAbs(c17_c_x);
  } else {
    c17_realmin(chartInstance);
    c17_scale = 2.2250738585072014E-308;
    c17_kstart = c17_c_ix0;
    c17_a = c17_c_n;
    c17_c = c17_a;
    c17_b_a = c17_c - 1;
    c17_b_c = c17_b_a;
    c17_c_a = c17_kstart;
    c17_b = c17_b_c;
    c17_kend = c17_c_a + c17_b;
    c17_b_kstart = c17_kstart;
    c17_b_kend = c17_kend;
    c17_d_a = c17_b_kstart;
    c17_b_b = c17_b_kend;
    c17_e_a = c17_d_a;
    c17_c_b = c17_b_b;
    if (c17_e_a > c17_c_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_c_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = c17_b_kstart; c17_k <= c17_b_kend; c17_k++) {
      c17_b_k = c17_k;
      c17_d_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 72, 1, 0) - 1];
      c17_e_x = c17_d_x;
      c17_absxk = muDoubleScalarAbs(c17_e_x);
      if (c17_absxk > c17_scale) {
        c17_t = c17_scale / c17_absxk;
        c17_y = 1.0 + c17_y * c17_t * c17_t;
        c17_scale = c17_absxk;
      } else {
        c17_t = c17_absxk / c17_scale;
        c17_y += c17_t * c17_t;
      }
    }

    c17_y = c17_scale * muDoubleScalarSqrt(c17_y);
  }

  return c17_y;
}

static real_T c17_abs(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                      real_T c17_x)
{
  real_T c17_b_x;
  c17_b_x = c17_x;
  return muDoubleScalarAbs(c17_b_x);
}

static void c17_realmin(SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c17_check_forloop_overflow_error
  (SFc17_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c17_overflow)
{
  int32_T c17_i897;
  static char_T c17_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c17_u[34];
  const mxArray *c17_y = NULL;
  int32_T c17_i898;
  static char_T c17_cv3[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c17_b_u[23];
  const mxArray *c17_b_y = NULL;
  if (!c17_overflow) {
  } else {
    for (c17_i897 = 0; c17_i897 < 34; c17_i897++) {
      c17_u[c17_i897] = c17_cv2[c17_i897];
    }

    c17_y = NULL;
    sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c17_i898 = 0; c17_i898 < 23; c17_i898++) {
      c17_b_u[c17_i898] = c17_cv3[c17_i898];
    }

    c17_b_y = NULL;
    sf_mex_assign(&c17_b_y, sf_mex_create("y", c17_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c17_y, 14, c17_b_y));
  }
}

static real_T c17_eml_div(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x, real_T c17_y)
{
  return c17_x / c17_y;
}

static void c17_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[72], int32_T c17_ix0,
  real_T c17_b_x[72])
{
  int32_T c17_i899;
  for (c17_i899 = 0; c17_i899 < 72; c17_i899++) {
    c17_b_x[c17_i899] = c17_x[c17_i899];
  }

  c17_h_eml_xscal(chartInstance, c17_n, c17_a, c17_b_x, c17_ix0);
}

static real_T c17_eml_xdotc(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[72], int32_T c17_ix0, real_T
  c17_y[72], int32_T c17_iy0)
{
  real_T c17_d;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_n;
  int32_T c17_d_ix0;
  int32_T c17_d_iy0;
  int32_T c17_e_n;
  int32_T c17_e_ix0;
  int32_T c17_e_iy0;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_f_n;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_a;
  int32_T c17_b_a;
  c17_b_n = c17_n;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_d_n = c17_c_n;
  c17_d_ix0 = c17_c_ix0;
  c17_d_iy0 = c17_c_iy0;
  c17_e_n = c17_d_n;
  c17_e_ix0 = c17_d_ix0;
  c17_e_iy0 = c17_d_iy0;
  c17_d = 0.0;
  if (c17_e_n < 1) {
  } else {
    c17_ix = c17_e_ix0;
    c17_iy = c17_e_iy0;
    c17_f_n = c17_e_n;
    c17_b = c17_f_n;
    c17_b_b = c17_b;
    if (1 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 1; c17_k <= c17_f_n; c17_k++) {
      c17_d += c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c17_ix), 1, 72, 1, 0) - 1] *
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_iy), 1, 72, 1, 0) - 1];
      c17_a = c17_ix + 1;
      c17_ix = c17_a;
      c17_b_a = c17_iy + 1;
      c17_iy = c17_b_a;
    }
  }

  return c17_d;
}

static void c17_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[72],
  int32_T c17_iy0, real_T c17_b_y[72])
{
  int32_T c17_i900;
  for (c17_i900 = 0; c17_i900 < 72; c17_i900++) {
    c17_b_y[c17_i900] = c17_y[c17_i900];
  }

  c17_h_eml_xaxpy(chartInstance, c17_n, c17_a, c17_ix0, c17_b_y, c17_iy0);
}

static real_T c17_b_eml_xnrm2(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[6], int32_T c17_ix0)
{
  real_T c17_y;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_scale;
  int32_T c17_kstart;
  int32_T c17_a;
  int32_T c17_c;
  int32_T c17_b_a;
  int32_T c17_b_c;
  int32_T c17_c_a;
  int32_T c17_b;
  int32_T c17_kend;
  int32_T c17_b_kstart;
  int32_T c17_b_kend;
  int32_T c17_d_a;
  int32_T c17_b_b;
  int32_T c17_e_a;
  int32_T c17_c_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_d_x;
  real_T c17_e_x;
  real_T c17_absxk;
  real_T c17_t;
  c17_b_n = c17_n;
  c17_b_ix0 = c17_ix0;
  c17_c_n = c17_b_n;
  c17_c_ix0 = c17_b_ix0;
  c17_y = 0.0;
  if (c17_c_n < 1) {
  } else if (c17_c_n == 1) {
    c17_b_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_c_ix0), 1, 6, 1, 0) - 1];
    c17_c_x = c17_b_x;
    c17_y = muDoubleScalarAbs(c17_c_x);
  } else {
    c17_realmin(chartInstance);
    c17_scale = 2.2250738585072014E-308;
    c17_kstart = c17_c_ix0;
    c17_a = c17_c_n;
    c17_c = c17_a;
    c17_b_a = c17_c - 1;
    c17_b_c = c17_b_a;
    c17_c_a = c17_kstart;
    c17_b = c17_b_c;
    c17_kend = c17_c_a + c17_b;
    c17_b_kstart = c17_kstart;
    c17_b_kend = c17_kend;
    c17_d_a = c17_b_kstart;
    c17_b_b = c17_b_kend;
    c17_e_a = c17_d_a;
    c17_c_b = c17_b_b;
    if (c17_e_a > c17_c_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_c_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = c17_b_kstart; c17_k <= c17_b_kend; c17_k++) {
      c17_b_k = c17_k;
      c17_d_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
      c17_e_x = c17_d_x;
      c17_absxk = muDoubleScalarAbs(c17_e_x);
      if (c17_absxk > c17_scale) {
        c17_t = c17_scale / c17_absxk;
        c17_y = 1.0 + c17_y * c17_t * c17_t;
        c17_scale = c17_absxk;
      } else {
        c17_t = c17_absxk / c17_scale;
        c17_y += c17_t * c17_t;
      }
    }

    c17_y = c17_scale * muDoubleScalarSqrt(c17_y);
  }

  return c17_y;
}

static void c17_b_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[6], int32_T c17_ix0,
  real_T c17_b_x[6])
{
  int32_T c17_i901;
  for (c17_i901 = 0; c17_i901 < 6; c17_i901++) {
    c17_b_x[c17_i901] = c17_x[c17_i901];
  }

  c17_i_eml_xscal(chartInstance, c17_n, c17_a, c17_b_x, c17_ix0);
}

static void c17_b_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[72], int32_T c17_ix0,
  real_T c17_y[12], int32_T c17_iy0, real_T c17_b_y[12])
{
  int32_T c17_i902;
  int32_T c17_i903;
  real_T c17_b_x[72];
  for (c17_i902 = 0; c17_i902 < 12; c17_i902++) {
    c17_b_y[c17_i902] = c17_y[c17_i902];
  }

  for (c17_i903 = 0; c17_i903 < 72; c17_i903++) {
    c17_b_x[c17_i903] = c17_x[c17_i903];
  }

  c17_i_eml_xaxpy(chartInstance, c17_n, c17_a, c17_b_x, c17_ix0, c17_b_y,
                  c17_iy0);
}

static void c17_c_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[12], int32_T c17_ix0,
  real_T c17_y[72], int32_T c17_iy0, real_T c17_b_y[72])
{
  int32_T c17_i904;
  int32_T c17_i905;
  real_T c17_b_x[12];
  for (c17_i904 = 0; c17_i904 < 72; c17_i904++) {
    c17_b_y[c17_i904] = c17_y[c17_i904];
  }

  for (c17_i905 = 0; c17_i905 < 12; c17_i905++) {
    c17_b_x[c17_i905] = c17_x[c17_i905];
  }

  c17_j_eml_xaxpy(chartInstance, c17_n, c17_a, c17_b_x, c17_ix0, c17_b_y,
                  c17_iy0);
}

static real_T c17_b_eml_xdotc(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[36], int32_T c17_ix0, real_T
  c17_y[36], int32_T c17_iy0)
{
  real_T c17_d;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_n;
  int32_T c17_d_ix0;
  int32_T c17_d_iy0;
  int32_T c17_e_n;
  int32_T c17_e_ix0;
  int32_T c17_e_iy0;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_f_n;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_a;
  int32_T c17_b_a;
  c17_b_n = c17_n;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_d_n = c17_c_n;
  c17_d_ix0 = c17_c_ix0;
  c17_d_iy0 = c17_c_iy0;
  c17_e_n = c17_d_n;
  c17_e_ix0 = c17_d_ix0;
  c17_e_iy0 = c17_d_iy0;
  c17_d_eml_scalar_eg(chartInstance);
  c17_d = 0.0;
  if (c17_e_n < 1) {
  } else {
    c17_ix = c17_e_ix0;
    c17_iy = c17_e_iy0;
    c17_f_n = c17_e_n;
    c17_b = c17_f_n;
    c17_b_b = c17_b;
    if (1 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 1; c17_k <= c17_f_n; c17_k++) {
      c17_d += c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c17_ix), 1, 36, 1, 0) - 1] *
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_iy), 1, 36, 1, 0) - 1];
      c17_a = c17_ix + 1;
      c17_ix = c17_a;
      c17_b_a = c17_iy + 1;
      c17_iy = c17_b_a;
    }
  }

  return c17_d;
}

static void c17_d_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_d_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[36],
  int32_T c17_iy0, real_T c17_b_y[36])
{
  int32_T c17_i906;
  for (c17_i906 = 0; c17_i906 < 36; c17_i906++) {
    c17_b_y[c17_i906] = c17_y[c17_i906];
  }

  c17_k_eml_xaxpy(chartInstance, c17_n, c17_a, c17_ix0, c17_b_y, c17_iy0);
}

static void c17_e_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_c_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[72], int32_T c17_ix0, real_T
  c17_b_x[72])
{
  int32_T c17_i907;
  for (c17_i907 = 0; c17_i907 < 72; c17_i907++) {
    c17_b_x[c17_i907] = c17_x[c17_i907];
  }

  c17_j_eml_xscal(chartInstance, c17_a, c17_b_x, c17_ix0);
}

static void c17_d_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[36], int32_T c17_ix0, real_T
  c17_b_x[36])
{
  int32_T c17_i908;
  for (c17_i908 = 0; c17_i908 < 36; c17_i908++) {
    c17_b_x[c17_i908] = c17_x[c17_i908];
  }

  c17_k_eml_xscal(chartInstance, c17_a, c17_b_x, c17_ix0);
}

static void c17_eps(SFc17_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c17_b_eml_error(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c17_i909;
  static char_T c17_cv4[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v', 'e', 'r',
    'g', 'e', 'n', 'c', 'e' };

  char_T c17_u[30];
  const mxArray *c17_y = NULL;
  for (c17_i909 = 0; c17_i909 < 30; c17_i909++) {
    c17_u[c17_i909] = c17_cv4[c17_i909];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 10, 0U, 1U, 0U, 2, 1, 30),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c17_y));
}

static real_T c17_sqrt(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x)
{
  real_T c17_b_x;
  c17_b_x = c17_x;
  c17_b_sqrt(chartInstance, &c17_b_x);
  return c17_b_x;
}

static void c17_c_eml_error(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c17_i910;
  static char_T c17_cv5[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c17_u[30];
  const mxArray *c17_y = NULL;
  int32_T c17_i911;
  static char_T c17_cv6[4] = { 's', 'q', 'r', 't' };

  char_T c17_b_u[4];
  const mxArray *c17_b_y = NULL;
  for (c17_i910 = 0; c17_i910 < 30; c17_i910++) {
    c17_u[c17_i910] = c17_cv5[c17_i910];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 10, 0U, 1U, 0U, 2, 1, 30),
                FALSE);
  for (c17_i911 = 0; c17_i911 < 4; c17_i911++) {
    c17_b_u[c17_i911] = c17_cv6[c17_i911];
  }

  c17_b_y = NULL;
  sf_mex_assign(&c17_b_y, sf_mex_create("y", c17_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c17_y, 14, c17_b_y));
}

static void c17_eml_xrotg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_b, real_T *c17_b_a, real_T *c17_b_b,
  real_T *c17_c, real_T *c17_s)
{
  *c17_b_a = c17_a;
  *c17_b_b = c17_b;
  c17_b_eml_xrotg(chartInstance, c17_b_a, c17_b_b, c17_c, c17_s);
}

static void c17_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x[36], int32_T c17_ix0, int32_T c17_iy0, real_T c17_c, real_T c17_s,
  real_T c17_b_x[36])
{
  int32_T c17_i912;
  for (c17_i912 = 0; c17_i912 < 36; c17_i912++) {
    c17_b_x[c17_i912] = c17_x[c17_i912];
  }

  c17_d_eml_xrot(chartInstance, c17_b_x, c17_ix0, c17_iy0, c17_c, c17_s);
}

static void c17_b_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[72], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s, real_T c17_b_x[72])
{
  int32_T c17_i913;
  for (c17_i913 = 0; c17_i913 < 72; c17_i913++) {
    c17_b_x[c17_i913] = c17_x[c17_i913];
  }

  c17_e_eml_xrot(chartInstance, c17_b_x, c17_ix0, c17_iy0, c17_c, c17_s);
}

static void c17_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[36], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_b_x[36])
{
  int32_T c17_i914;
  for (c17_i914 = 0; c17_i914 < 36; c17_i914++) {
    c17_b_x[c17_i914] = c17_x[c17_i914];
  }

  c17_e_eml_xswap(chartInstance, c17_b_x, c17_ix0, c17_iy0);
}

static void c17_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_b_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[72], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_b_x[72])
{
  int32_T c17_i915;
  for (c17_i915 = 0; c17_i915 < 72; c17_i915++) {
    c17_b_x[c17_i915] = c17_x[c17_i915];
  }

  c17_f_eml_xswap(chartInstance, c17_b_x, c17_ix0, c17_iy0);
}

static void c17_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_k, real_T c17_A[36], real_T c17_B[72], real_T
  c17_C[72], real_T c17_b_C[72])
{
  int32_T c17_i916;
  int32_T c17_i917;
  real_T c17_b_A[36];
  int32_T c17_i918;
  real_T c17_b_B[72];
  for (c17_i916 = 0; c17_i916 < 72; c17_i916++) {
    c17_b_C[c17_i916] = c17_C[c17_i916];
  }

  for (c17_i917 = 0; c17_i917 < 36; c17_i917++) {
    c17_b_A[c17_i917] = c17_A[c17_i917];
  }

  for (c17_i918 = 0; c17_i918 < 72; c17_i918++) {
    c17_b_B[c17_i918] = c17_B[c17_i918];
  }

  c17_p_eml_xgemm(chartInstance, c17_k, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_inv(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c17_x[36], real_T c17_y[36])
{
  int32_T c17_i919;
  real_T c17_b_x[36];
  int32_T c17_i920;
  real_T c17_c_x[36];
  real_T c17_n1x;
  int32_T c17_i921;
  real_T c17_b_y[36];
  real_T c17_n1xinv;
  real_T c17_a;
  real_T c17_b;
  real_T c17_c_y;
  real_T c17_rc;
  real_T c17_d_x;
  boolean_T c17_b_b;
  real_T c17_e_x;
  int32_T c17_i922;
  static char_T c17_cv7[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c17_u[8];
  const mxArray *c17_d_y = NULL;
  real_T c17_b_u;
  const mxArray *c17_e_y = NULL;
  real_T c17_c_u;
  const mxArray *c17_f_y = NULL;
  real_T c17_d_u;
  const mxArray *c17_g_y = NULL;
  char_T c17_str[14];
  int32_T c17_i923;
  char_T c17_b_str[14];
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  for (c17_i919 = 0; c17_i919 < 36; c17_i919++) {
    c17_b_x[c17_i919] = c17_x[c17_i919];
  }

  c17_invNxN(chartInstance, c17_b_x, c17_y);
  for (c17_i920 = 0; c17_i920 < 36; c17_i920++) {
    c17_c_x[c17_i920] = c17_x[c17_i920];
  }

  c17_n1x = c17_norm(chartInstance, c17_c_x);
  for (c17_i921 = 0; c17_i921 < 36; c17_i921++) {
    c17_b_y[c17_i921] = c17_y[c17_i921];
  }

  c17_n1xinv = c17_norm(chartInstance, c17_b_y);
  c17_a = c17_n1x;
  c17_b = c17_n1xinv;
  c17_c_y = c17_a * c17_b;
  c17_rc = 1.0 / c17_c_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c17_n1x == 0.0) {
    guard2 = TRUE;
  } else if (c17_n1xinv == 0.0) {
    guard2 = TRUE;
  } else if (c17_rc == 0.0) {
    guard1 = TRUE;
  } else {
    c17_d_x = c17_rc;
    c17_b_b = muDoubleScalarIsNaN(c17_d_x);
    guard3 = FALSE;
    if (c17_b_b) {
      guard3 = TRUE;
    } else {
      c17_eps(chartInstance);
      if (c17_rc < 2.2204460492503131E-16) {
        guard3 = TRUE;
      }
    }

    if (guard3 == TRUE) {
      c17_e_x = c17_rc;
      for (c17_i922 = 0; c17_i922 < 8; c17_i922++) {
        c17_u[c17_i922] = c17_cv7[c17_i922];
      }

      c17_d_y = NULL;
      sf_mex_assign(&c17_d_y, sf_mex_create("y", c17_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    FALSE);
      c17_b_u = 14.0;
      c17_e_y = NULL;
      sf_mex_assign(&c17_e_y, sf_mex_create("y", &c17_b_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c17_c_u = 6.0;
      c17_f_y = NULL;
      sf_mex_assign(&c17_f_y, sf_mex_create("y", &c17_c_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c17_d_u = c17_e_x;
      c17_g_y = NULL;
      sf_mex_assign(&c17_g_y, sf_mex_create("y", &c17_d_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c17_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U, 2U,
        14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c17_d_y, 14, c17_e_y, 14,
        c17_f_y), 14, c17_g_y), "sprintf", c17_str);
      for (c17_i923 = 0; c17_i923 < 14; c17_i923++) {
        c17_b_str[c17_i923] = c17_str[c17_i923];
      }

      c17_b_eml_warning(chartInstance, c17_b_str);
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c17_eml_warning(chartInstance);
  }
}

static void c17_invNxN(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x[36], real_T c17_y[36])
{
  int32_T c17_i924;
  int32_T c17_info;
  int32_T c17_ipiv[6];
  int32_T c17_i925;
  int32_T c17_p[6];
  int32_T c17_k;
  real_T c17_b_k;
  int32_T c17_ipk;
  int32_T c17_a;
  real_T c17_b;
  int32_T c17_b_a;
  real_T c17_b_b;
  int32_T c17_idx;
  real_T c17_flt;
  boolean_T c17_b_p;
  int32_T c17_pipk;
  int32_T c17_c_k;
  int32_T c17_d_k;
  int32_T c17_c;
  int32_T c17_e_k;
  boolean_T c17_overflow;
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_c_a;
  int32_T c17_i926;
  boolean_T c17_b_overflow;
  int32_T c17_i;
  int32_T c17_b_i;
  real_T c17_d_a;
  real_T c17_c_b;
  real_T c17_b_y;
  int32_T c17_i927;
  real_T c17_b_x[36];
  for (c17_i924 = 0; c17_i924 < 36; c17_i924++) {
    c17_y[c17_i924] = 0.0;
  }

  c17_d_eml_matlab_zgetrf(chartInstance, c17_x, c17_ipiv, &c17_info);
  for (c17_i925 = 0; c17_i925 < 6; c17_i925++) {
    c17_p[c17_i925] = 1 + c17_i925;
  }

  for (c17_k = 0; c17_k < 5; c17_k++) {
    c17_b_k = 1.0 + (real_T)c17_k;
    c17_ipk = c17_ipiv[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", c17_b_k), 1, 6, 1, 0) - 1];
    c17_a = c17_ipk;
    c17_b = c17_b_k;
    c17_b_a = c17_a;
    c17_b_b = c17_b;
    c17_idx = c17_b_a;
    c17_flt = c17_b_b;
    c17_b_p = ((real_T)c17_idx > c17_flt);
    if (c17_b_p) {
      c17_pipk = c17_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_ipk), 1, 6, 1, 0) - 1];
      c17_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_ipk), 1, 6, 1, 0) - 1] = c17_p[(int32_T)(real_T)
        _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c17_b_k),
        1, 6, 1, 0) - 1];
      c17_p[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c17_b_k), 1, 6, 1, 0) - 1] = c17_pipk;
    }
  }

  for (c17_c_k = 1; c17_c_k < 7; c17_c_k++) {
    c17_d_k = c17_c_k;
    c17_c = c17_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_d_k), 1, 6, 1, 0) - 1];
    c17_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c17_d_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_c), 1, 6, 2, 0) - 1)) -
      1] = 1.0;
    c17_e_k = c17_d_k;
    c17_overflow = FALSE;
    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_j = c17_e_k; c17_j < 7; c17_j++) {
      c17_b_j = c17_j;
      if (c17_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c17_b_j), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_c), 1, 6, 2, 0) - 1)) -
          1] != 0.0) {
        c17_c_a = c17_b_j;
        c17_i926 = c17_c_a;
        c17_b_overflow = FALSE;
        if (c17_b_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
        }

        for (c17_i = c17_i926 + 1; c17_i < 7; c17_i++) {
          c17_b_i = c17_i;
          c17_d_a = c17_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 6, 1, 0) + 6 *
                           (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_c), 1, 6, 2, 0) - 1)) - 1];
          c17_c_b = c17_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 6, 1, 0) + 6 *
                           (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 6, 2, 0) - 1)) - 1];
          c17_b_y = c17_d_a * c17_c_b;
          c17_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_b_i), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c17_c), 1, 6, 2, 0) - 1)) - 1] = c17_y
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c17_b_i), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK
               ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_c), 1, 6, 2, 0)
               - 1)) - 1] - c17_b_y;
        }
      }
    }
  }

  for (c17_i927 = 0; c17_i927 < 36; c17_i927++) {
    c17_b_x[c17_i927] = c17_x[c17_i927];
  }

  c17_h_eml_xtrsm(chartInstance, c17_b_x, c17_y);
}

static void c17_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_b_A[36], int32_T c17_ipiv[6],
  int32_T *c17_info)
{
  int32_T c17_i928;
  for (c17_i928 = 0; c17_i928 < 36; c17_i928++) {
    c17_b_A[c17_i928] = c17_A[c17_i928];
  }

  c17_d_eml_matlab_zgetrf(chartInstance, c17_b_A, c17_ipiv, c17_info);
}

static void c17_eml_xger(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T c17_ix0, int32_T
  c17_iy0, real_T c17_A[36], int32_T c17_ia0, real_T c17_b_A[36])
{
  int32_T c17_i929;
  for (c17_i929 = 0; c17_i929 < 36; c17_i929++) {
    c17_b_A[c17_i929] = c17_A[c17_i929];
  }

  c17_d_eml_xger(chartInstance, c17_m, c17_n, c17_alpha1, c17_ix0, c17_iy0,
                 c17_b_A, c17_ia0);
}

static void c17_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[36], real_T c17_b_B[36])
{
  int32_T c17_i930;
  int32_T c17_i931;
  real_T c17_b_A[36];
  for (c17_i930 = 0; c17_i930 < 36; c17_i930++) {
    c17_b_B[c17_i930] = c17_B[c17_i930];
  }

  for (c17_i931 = 0; c17_i931 < 36; c17_i931++) {
    c17_b_A[c17_i931] = c17_A[c17_i931];
  }

  c17_h_eml_xtrsm(chartInstance, c17_b_A, c17_b_B);
}

static real_T c17_norm(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x[36])
{
  real_T c17_y;
  int32_T c17_j;
  real_T c17_b_j;
  real_T c17_s;
  int32_T c17_i;
  real_T c17_b_i;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_b_y;
  real_T c17_d_x;
  boolean_T c17_b;
  boolean_T exitg1;
  c17_y = 0.0;
  c17_j = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c17_j < 6)) {
    c17_b_j = 1.0 + (real_T)c17_j;
    c17_s = 0.0;
    for (c17_i = 0; c17_i < 6; c17_i++) {
      c17_b_i = 1.0 + (real_T)c17_i;
      c17_b_x = c17_x[((int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c17_b_i), 1, 6, 1, 0) + 6 * ((int32_T)(real_T)
        _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c17_b_j),
        1, 6, 2, 0) - 1)) - 1];
      c17_c_x = c17_b_x;
      c17_b_y = muDoubleScalarAbs(c17_c_x);
      c17_s += c17_b_y;
    }

    c17_d_x = c17_s;
    c17_b = muDoubleScalarIsNaN(c17_d_x);
    if (c17_b) {
      c17_y = rtNaN;
      exitg1 = TRUE;
    } else {
      if (c17_s > c17_y) {
        c17_y = c17_s;
      }

      c17_j++;
    }
  }

  return c17_y;
}

static void c17_eml_warning(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c17_i932;
  static char_T c17_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c17_u[27];
  const mxArray *c17_y = NULL;
  for (c17_i932 = 0; c17_i932 < 27; c17_i932++) {
    c17_u[c17_i932] = c17_varargin_1[c17_i932];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 10, 0U, 1U, 0U, 2, 1, 27),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c17_y));
}

static void c17_b_eml_warning(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, char_T c17_varargin_2[14])
{
  int32_T c17_i933;
  static char_T c17_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c17_u[33];
  const mxArray *c17_y = NULL;
  int32_T c17_i934;
  char_T c17_b_u[14];
  const mxArray *c17_b_y = NULL;
  for (c17_i933 = 0; c17_i933 < 33; c17_i933++) {
    c17_u[c17_i933] = c17_varargin_1[c17_i933];
  }

  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", c17_u, 10, 0U, 1U, 0U, 2, 1, 33),
                FALSE);
  for (c17_i934 = 0; c17_i934 < 14; c17_i934++) {
    c17_b_u[c17_i934] = c17_varargin_2[c17_i934];
  }

  c17_b_y = NULL;
  sf_mex_assign(&c17_b_y, sf_mex_create("y", c17_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c17_y, 14, c17_b_y));
}

static void c17_f_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_g_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static real_T c17_b_norm(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_x[6])
{
  real_T c17_y;
  real_T c17_scale;
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_absxk;
  real_T c17_t;
  c17_y = 0.0;
  c17_realmin(chartInstance);
  c17_scale = 2.2250738585072014E-308;
  for (c17_k = 1; c17_k < 7; c17_k++) {
    c17_b_k = c17_k;
    c17_b_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_b_k), 1, 6, 1, 0) - 1];
    c17_c_x = c17_b_x;
    c17_absxk = muDoubleScalarAbs(c17_c_x);
    if (c17_absxk > c17_scale) {
      c17_t = c17_scale / c17_absxk;
      c17_y = 1.0 + c17_y * c17_t * c17_t;
      c17_scale = c17_absxk;
    } else {
      c17_t = c17_absxk / c17_scale;
      c17_y += c17_t * c17_t;
    }
  }

  return c17_scale * muDoubleScalarSqrt(c17_y);
}

static void c17_b_eye(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                      real_T c17_I[144])
{
  int32_T c17_i935;
  int32_T c17_i;
  int32_T c17_b_i;
  for (c17_i935 = 0; c17_i935 < 144; c17_i935++) {
    c17_I[c17_i935] = 0.0;
  }

  for (c17_i = 1; c17_i < 13; c17_i++) {
    c17_b_i = c17_i;
    c17_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c17_b_i), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
             "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 12, 2, 0)
            - 1)) - 1] = 1.0;
  }
}

static void c17_h_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_b_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[72], real_T c17_B[72], real_T c17_C[144], real_T
  c17_b_C[144])
{
  int32_T c17_i936;
  int32_T c17_i937;
  real_T c17_b_A[72];
  int32_T c17_i938;
  real_T c17_b_B[72];
  for (c17_i936 = 0; c17_i936 < 144; c17_i936++) {
    c17_b_C[c17_i936] = c17_C[c17_i936];
  }

  for (c17_i937 = 0; c17_i937 < 72; c17_i937++) {
    c17_b_A[c17_i937] = c17_A[c17_i937];
  }

  for (c17_i938 = 0; c17_i938 < 72; c17_i938++) {
    c17_b_B[c17_i938] = c17_B[c17_i938];
  }

  c17_q_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_mrdivide(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_A[348], real_T c17_B[841], real_T c17_y[348])
{
  int32_T c17_i939;
  int32_T c17_i940;
  int32_T c17_i941;
  int32_T c17_i942;
  static real_T c17_b_A[841];
  int32_T c17_i943;
  int32_T c17_i944;
  int32_T c17_i945;
  int32_T c17_i946;
  real_T c17_b_B[348];
  int32_T c17_info;
  int32_T c17_ipiv[29];
  int32_T c17_b_info;
  int32_T c17_c_info;
  int32_T c17_d_info;
  int32_T c17_i;
  int32_T c17_b_i;
  int32_T c17_ip;
  int32_T c17_j;
  int32_T c17_b_j;
  real_T c17_temp;
  int32_T c17_i947;
  real_T c17_c_A[841];
  int32_T c17_i948;
  real_T c17_d_A[841];
  int32_T c17_i949;
  int32_T c17_i950;
  int32_T c17_i951;
  int32_T c17_i952;
  c17_i939 = 0;
  for (c17_i940 = 0; c17_i940 < 29; c17_i940++) {
    c17_i941 = 0;
    for (c17_i942 = 0; c17_i942 < 29; c17_i942++) {
      c17_b_A[c17_i942 + c17_i939] = c17_B[c17_i941 + c17_i940];
      c17_i941 += 29;
    }

    c17_i939 += 29;
  }

  c17_i943 = 0;
  for (c17_i944 = 0; c17_i944 < 12; c17_i944++) {
    c17_i945 = 0;
    for (c17_i946 = 0; c17_i946 < 29; c17_i946++) {
      c17_b_B[c17_i946 + c17_i943] = c17_A[c17_i945 + c17_i944];
      c17_i945 += 12;
    }

    c17_i943 += 29;
  }

  c17_e_eml_matlab_zgetrf(chartInstance, c17_b_A, c17_ipiv, &c17_info);
  c17_b_info = c17_info;
  c17_c_info = c17_b_info;
  c17_d_info = c17_c_info;
  if (c17_d_info > 0) {
    c17_eml_warning(chartInstance);
  }

  c17_i_eml_scalar_eg(chartInstance);
  for (c17_i = 1; c17_i < 30; c17_i++) {
    c17_b_i = c17_i;
    if (c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_i), 1, 29, 1, 0) - 1] != c17_b_i) {
      c17_ip = c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 29, 1, 0) - 1];
      for (c17_j = 1; c17_j < 13; c17_j++) {
        c17_b_j = c17_j;
        c17_temp = c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 29, 1, 0) + 29 *
                            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 12, 2, 0) - 1)) - 1];
        c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_b_i), 1, 29, 1, 0) + 29 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c17_b_j), 1, 12, 2, 0) - 1)) - 1] = c17_b_B
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_ip), 1, 29, 1, 0) + 29 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 12, 2, 0)
             - 1)) - 1];
        c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_ip), 1, 29, 1, 0) + 29 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c17_b_j), 1, 12, 2, 0) - 1)) - 1] = c17_temp;
      }
    }
  }

  for (c17_i947 = 0; c17_i947 < 841; c17_i947++) {
    c17_c_A[c17_i947] = c17_b_A[c17_i947];
  }

  c17_i_eml_xtrsm(chartInstance, c17_c_A, c17_b_B);
  for (c17_i948 = 0; c17_i948 < 841; c17_i948++) {
    c17_d_A[c17_i948] = c17_b_A[c17_i948];
  }

  c17_j_eml_xtrsm(chartInstance, c17_d_A, c17_b_B);
  c17_i949 = 0;
  for (c17_i950 = 0; c17_i950 < 29; c17_i950++) {
    c17_i951 = 0;
    for (c17_i952 = 0; c17_i952 < 12; c17_i952++) {
      c17_y[c17_i952 + c17_i949] = c17_b_B[c17_i951 + c17_i950];
      c17_i951 += 29;
    }

    c17_i949 += 12;
  }
}

static void c17_b_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_b_A[841], int32_T c17_ipiv[29],
  int32_T *c17_info)
{
  int32_T c17_i953;
  for (c17_i953 = 0; c17_i953 < 841; c17_i953++) {
    c17_b_A[c17_i953] = c17_A[c17_i953];
  }

  c17_e_eml_matlab_zgetrf(chartInstance, c17_b_A, c17_ipiv, c17_info);
}

static void c17_c_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[841], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_b_x[841])
{
  int32_T c17_i954;
  for (c17_i954 = 0; c17_i954 < 841; c17_i954++) {
    c17_b_x[c17_i954] = c17_x[c17_i954];
  }

  c17_g_eml_xswap(chartInstance, c17_b_x, c17_ix0, c17_iy0);
}

static void c17_b_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_b_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[841], int32_T c17_ia0, real_T c17_b_A
  [841])
{
  int32_T c17_i955;
  for (c17_i955 = 0; c17_i955 < 841; c17_i955++) {
    c17_b_A[c17_i955] = c17_A[c17_i955];
  }

  c17_e_eml_xger(chartInstance, c17_m, c17_n, c17_alpha1, c17_ix0, c17_iy0,
                 c17_b_A, c17_ia0);
}

static void c17_i_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_b_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_B[348], real_T c17_b_B[348])
{
  int32_T c17_i956;
  int32_T c17_i957;
  real_T c17_b_A[841];
  for (c17_i956 = 0; c17_i956 < 348; c17_i956++) {
    c17_b_B[c17_i956] = c17_B[c17_i956];
  }

  for (c17_i957 = 0; c17_i957 < 841; c17_i957++) {
    c17_b_A[c17_i957] = c17_A[c17_i957];
  }

  c17_i_eml_xtrsm(chartInstance, c17_b_A, c17_b_B);
}

static void c17_c_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_c_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_B[348], real_T c17_b_B[348])
{
  int32_T c17_i958;
  int32_T c17_i959;
  real_T c17_b_A[841];
  for (c17_i958 = 0; c17_i958 < 348; c17_i958++) {
    c17_b_B[c17_i958] = c17_B[c17_i958];
  }

  for (c17_i959 = 0; c17_i959 < 841; c17_i959++) {
    c17_b_A[c17_i959] = c17_A[c17_i959];
  }

  c17_j_eml_xtrsm(chartInstance, c17_b_A, c17_b_B);
}

static void c17_j_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_c_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[348], real_T c17_B[667], real_T c17_C[276],
  real_T c17_b_C[276])
{
  int32_T c17_i960;
  int32_T c17_i961;
  real_T c17_b_A[348];
  int32_T c17_i962;
  real_T c17_b_B[667];
  for (c17_i960 = 0; c17_i960 < 276; c17_i960++) {
    c17_b_C[c17_i960] = c17_C[c17_i960];
  }

  for (c17_i961 = 0; c17_i961 < 348; c17_i961++) {
    c17_b_A[c17_i961] = c17_A[c17_i961];
  }

  for (c17_i962 = 0; c17_i962 < 667; c17_i962++) {
    c17_b_B[c17_i962] = c17_B[c17_i962];
  }

  c17_r_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_k_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_d_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[348], real_T c17_B[348], real_T c17_C[144],
  real_T c17_b_C[144])
{
  int32_T c17_i963;
  int32_T c17_i964;
  real_T c17_b_A[348];
  int32_T c17_i965;
  real_T c17_b_B[348];
  for (c17_i963 = 0; c17_i963 < 144; c17_i963++) {
    c17_b_C[c17_i963] = c17_C[c17_i963];
  }

  for (c17_i964 = 0; c17_i964 < 348; c17_i964++) {
    c17_b_A[c17_i964] = c17_A[c17_i964];
  }

  for (c17_i965 = 0; c17_i965 < 348; c17_i965++) {
    c17_b_B[c17_i965] = c17_B[c17_i965];
  }

  c17_s_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_b_mrdivide(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[36], real_T c17_y[138])
{
  int32_T c17_i966;
  int32_T c17_i967;
  int32_T c17_i968;
  int32_T c17_i969;
  real_T c17_b_A[36];
  int32_T c17_i970;
  int32_T c17_i971;
  int32_T c17_i972;
  int32_T c17_i973;
  real_T c17_b_B[138];
  int32_T c17_info;
  int32_T c17_ipiv[6];
  int32_T c17_b_info;
  int32_T c17_c_info;
  int32_T c17_d_info;
  int32_T c17_i;
  int32_T c17_b_i;
  int32_T c17_ip;
  int32_T c17_j;
  int32_T c17_b_j;
  real_T c17_temp;
  int32_T c17_i974;
  real_T c17_c_A[36];
  int32_T c17_i975;
  real_T c17_d_A[36];
  int32_T c17_i976;
  int32_T c17_i977;
  int32_T c17_i978;
  int32_T c17_i979;
  c17_i966 = 0;
  for (c17_i967 = 0; c17_i967 < 6; c17_i967++) {
    c17_i968 = 0;
    for (c17_i969 = 0; c17_i969 < 6; c17_i969++) {
      c17_b_A[c17_i969 + c17_i966] = c17_B[c17_i968 + c17_i967];
      c17_i968 += 6;
    }

    c17_i966 += 6;
  }

  c17_i970 = 0;
  for (c17_i971 = 0; c17_i971 < 23; c17_i971++) {
    c17_i972 = 0;
    for (c17_i973 = 0; c17_i973 < 6; c17_i973++) {
      c17_b_B[c17_i973 + c17_i970] = c17_A[c17_i972 + c17_i971];
      c17_i972 += 23;
    }

    c17_i970 += 6;
  }

  c17_d_eml_matlab_zgetrf(chartInstance, c17_b_A, c17_ipiv, &c17_info);
  c17_b_info = c17_info;
  c17_c_info = c17_b_info;
  c17_d_info = c17_c_info;
  if (c17_d_info > 0) {
    c17_eml_warning(chartInstance);
  }

  for (c17_i = 1; c17_i < 7; c17_i++) {
    c17_b_i = c17_i;
    if (c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_i), 1, 6, 1, 0) - 1] != c17_b_i) {
      c17_ip = c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 6, 1, 0) - 1];
      for (c17_j = 1; c17_j < 24; c17_j++) {
        c17_b_j = c17_j;
        c17_temp = c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 6, 1, 0) + 6 *
                            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 23, 2, 0) - 1)) - 1];
        c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_b_i), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c17_b_j), 1, 23, 2, 0) - 1)) - 1] = c17_b_B
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_ip), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 23, 2, 0) - 1))
          - 1];
        c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_ip), 1, 6, 1, 0) + 6 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c17_b_j), 1, 23, 2, 0) - 1)) - 1] = c17_temp;
      }
    }
  }

  for (c17_i974 = 0; c17_i974 < 36; c17_i974++) {
    c17_c_A[c17_i974] = c17_b_A[c17_i974];
  }

  c17_k_eml_xtrsm(chartInstance, c17_c_A, c17_b_B);
  for (c17_i975 = 0; c17_i975 < 36; c17_i975++) {
    c17_d_A[c17_i975] = c17_b_A[c17_i975];
  }

  c17_l_eml_xtrsm(chartInstance, c17_d_A, c17_b_B);
  c17_i976 = 0;
  for (c17_i977 = 0; c17_i977 < 6; c17_i977++) {
    c17_i978 = 0;
    for (c17_i979 = 0; c17_i979 < 23; c17_i979++) {
      c17_y[c17_i979 + c17_i976] = c17_b_B[c17_i978 + c17_i977];
      c17_i978 += 6;
    }

    c17_i976 += 23;
  }
}

static void c17_d_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[138], real_T c17_b_B[138])
{
  int32_T c17_i980;
  int32_T c17_i981;
  real_T c17_b_A[36];
  for (c17_i980 = 0; c17_i980 < 138; c17_i980++) {
    c17_b_B[c17_i980] = c17_B[c17_i980];
  }

  for (c17_i981 = 0; c17_i981 < 36; c17_i981++) {
    c17_b_A[c17_i981] = c17_A[c17_i981];
  }

  c17_k_eml_xtrsm(chartInstance, c17_b_A, c17_b_B);
}

static void c17_d_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_e_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[138], real_T c17_b_B[138])
{
  int32_T c17_i982;
  int32_T c17_i983;
  real_T c17_b_A[36];
  for (c17_i982 = 0; c17_i982 < 138; c17_i982++) {
    c17_b_B[c17_i982] = c17_B[c17_i982];
  }

  for (c17_i983 = 0; c17_i983 < 36; c17_i983++) {
    c17_b_A[c17_i983] = c17_A[c17_i983];
  }

  c17_l_eml_xtrsm(chartInstance, c17_b_A, c17_b_B);
}

static void c17_l_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_e_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[72], real_T c17_C[276], real_T
  c17_b_C[276])
{
  int32_T c17_i984;
  int32_T c17_i985;
  real_T c17_b_A[138];
  int32_T c17_i986;
  real_T c17_b_B[72];
  for (c17_i984 = 0; c17_i984 < 276; c17_i984++) {
    c17_b_C[c17_i984] = c17_C[c17_i984];
  }

  for (c17_i985 = 0; c17_i985 < 138; c17_i985++) {
    c17_b_A[c17_i985] = c17_A[c17_i985];
  }

  for (c17_i986 = 0; c17_i986 < 72; c17_i986++) {
    c17_b_B[c17_i986] = c17_B[c17_i986];
  }

  c17_t_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_m_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_f_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[276], real_T c17_C[144],
  real_T c17_b_C[144])
{
  int32_T c17_i987;
  int32_T c17_i988;
  real_T c17_b_A[276];
  int32_T c17_i989;
  real_T c17_b_B[276];
  for (c17_i987 = 0; c17_i987 < 144; c17_i987++) {
    c17_b_C[c17_i987] = c17_C[c17_i987];
  }

  for (c17_i988 = 0; c17_i988 < 276; c17_i988++) {
    c17_b_A[c17_i988] = c17_A[c17_i988];
  }

  for (c17_i989 = 0; c17_i989 < 276; c17_i989++) {
    c17_b_B[c17_i989] = c17_B[c17_i989];
  }

  c17_u_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_c_mrdivide(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[144], real_T c17_y[276])
{
  int32_T c17_i990;
  int32_T c17_i991;
  int32_T c17_i992;
  int32_T c17_i993;
  real_T c17_b_A[144];
  int32_T c17_i994;
  int32_T c17_i995;
  int32_T c17_i996;
  int32_T c17_i997;
  real_T c17_b_B[276];
  int32_T c17_info;
  int32_T c17_ipiv[12];
  int32_T c17_b_info;
  int32_T c17_c_info;
  int32_T c17_d_info;
  int32_T c17_i;
  int32_T c17_b_i;
  int32_T c17_ip;
  int32_T c17_j;
  int32_T c17_b_j;
  real_T c17_temp;
  int32_T c17_i998;
  real_T c17_c_A[144];
  int32_T c17_i999;
  real_T c17_d_A[144];
  int32_T c17_i1000;
  int32_T c17_i1001;
  int32_T c17_i1002;
  int32_T c17_i1003;
  c17_i990 = 0;
  for (c17_i991 = 0; c17_i991 < 12; c17_i991++) {
    c17_i992 = 0;
    for (c17_i993 = 0; c17_i993 < 12; c17_i993++) {
      c17_b_A[c17_i993 + c17_i990] = c17_B[c17_i992 + c17_i991];
      c17_i992 += 12;
    }

    c17_i990 += 12;
  }

  c17_i994 = 0;
  for (c17_i995 = 0; c17_i995 < 23; c17_i995++) {
    c17_i996 = 0;
    for (c17_i997 = 0; c17_i997 < 12; c17_i997++) {
      c17_b_B[c17_i997 + c17_i994] = c17_A[c17_i996 + c17_i995];
      c17_i996 += 23;
    }

    c17_i994 += 12;
  }

  c17_f_eml_matlab_zgetrf(chartInstance, c17_b_A, c17_ipiv, &c17_info);
  c17_b_info = c17_info;
  c17_c_info = c17_b_info;
  c17_d_info = c17_c_info;
  if (c17_d_info > 0) {
    c17_eml_warning(chartInstance);
  }

  for (c17_i = 1; c17_i < 13; c17_i++) {
    c17_b_i = c17_i;
    if (c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_i), 1, 12, 1, 0) - 1] != c17_b_i) {
      c17_ip = c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 12, 1, 0) - 1];
      for (c17_j = 1; c17_j < 24; c17_j++) {
        c17_b_j = c17_j;
        c17_temp = c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 12, 1, 0) + 12 *
                            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 23, 2, 0) - 1)) - 1];
        c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_b_i), 1, 12, 1, 0) + 12 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c17_b_j), 1, 23, 2, 0) - 1)) - 1] = c17_b_B
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_ip), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 23, 2, 0)
             - 1)) - 1];
        c17_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_ip), 1, 12, 1, 0) + 12 *
                 (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                    (real_T)c17_b_j), 1, 23, 2, 0) - 1)) - 1] = c17_temp;
      }
    }
  }

  for (c17_i998 = 0; c17_i998 < 144; c17_i998++) {
    c17_c_A[c17_i998] = c17_b_A[c17_i998];
  }

  c17_m_eml_xtrsm(chartInstance, c17_c_A, c17_b_B);
  for (c17_i999 = 0; c17_i999 < 144; c17_i999++) {
    c17_d_A[c17_i999] = c17_b_A[c17_i999];
  }

  c17_n_eml_xtrsm(chartInstance, c17_d_A, c17_b_B);
  c17_i1000 = 0;
  for (c17_i1001 = 0; c17_i1001 < 12; c17_i1001++) {
    c17_i1002 = 0;
    for (c17_i1003 = 0; c17_i1003 < 23; c17_i1003++) {
      c17_y[c17_i1003 + c17_i1000] = c17_b_B[c17_i1002 + c17_i1001];
      c17_i1002 += 12;
    }

    c17_i1000 += 23;
  }
}

static void c17_c_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_b_A[144], int32_T c17_ipiv[12],
  int32_T *c17_info)
{
  int32_T c17_i1004;
  for (c17_i1004 = 0; c17_i1004 < 144; c17_i1004++) {
    c17_b_A[c17_i1004] = c17_A[c17_i1004];
  }

  c17_f_eml_matlab_zgetrf(chartInstance, c17_b_A, c17_ipiv, c17_info);
}

static void c17_c_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[144], int32_T c17_ia0, real_T c17_b_A
  [144])
{
  int32_T c17_i1005;
  for (c17_i1005 = 0; c17_i1005 < 144; c17_i1005++) {
    c17_b_A[c17_i1005] = c17_A[c17_i1005];
  }

  c17_f_eml_xger(chartInstance, c17_m, c17_n, c17_alpha1, c17_ix0, c17_iy0,
                 c17_b_A, c17_ia0);
}

static void c17_f_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_B[276], real_T c17_b_B[276])
{
  int32_T c17_i1006;
  int32_T c17_i1007;
  real_T c17_b_A[144];
  for (c17_i1006 = 0; c17_i1006 < 276; c17_i1006++) {
    c17_b_B[c17_i1006] = c17_B[c17_i1006];
  }

  for (c17_i1007 = 0; c17_i1007 < 144; c17_i1007++) {
    c17_b_A[c17_i1007] = c17_A[c17_i1007];
  }

  c17_m_eml_xtrsm(chartInstance, c17_b_A, c17_b_B);
}

static void c17_e_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_g_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_B[276], real_T c17_b_B[276])
{
  int32_T c17_i1008;
  int32_T c17_i1009;
  real_T c17_b_A[144];
  for (c17_i1008 = 0; c17_i1008 < 276; c17_i1008++) {
    c17_b_B[c17_i1008] = c17_B[c17_i1008];
  }

  for (c17_i1009 = 0; c17_i1009 < 144; c17_i1009++) {
    c17_b_A[c17_i1009] = c17_A[c17_i1009];
  }

  c17_n_eml_xtrsm(chartInstance, c17_b_A, c17_b_B);
}

static void c17_c_eye(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                      real_T c17_I[529])
{
  int32_T c17_i1010;
  int32_T c17_i;
  int32_T c17_b_i;
  for (c17_i1010 = 0; c17_i1010 < 529; c17_i1010++) {
    c17_I[c17_i1010] = 0.0;
  }

  for (c17_i = 1; c17_i < 24; c17_i++) {
    c17_b_i = c17_i;
    c17_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c17_b_i), 1, 23, 1, 0) + 23 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
             "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 23, 2, 0)
            - 1)) - 1] = 1.0;
  }
}

static void c17_n_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_g_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[276], real_T c17_C[529],
  real_T c17_b_C[529])
{
  int32_T c17_i1011;
  int32_T c17_i1012;
  real_T c17_b_A[276];
  int32_T c17_i1013;
  real_T c17_b_B[276];
  for (c17_i1011 = 0; c17_i1011 < 529; c17_i1011++) {
    c17_b_C[c17_i1011] = c17_C[c17_i1011];
  }

  for (c17_i1012 = 0; c17_i1012 < 276; c17_i1012++) {
    c17_b_A[c17_i1012] = c17_A[c17_i1012];
  }

  for (c17_i1013 = 0; c17_i1013 < 276; c17_i1013++) {
    c17_b_B[c17_i1013] = c17_B[c17_i1013];
  }

  c17_v_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_o_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_p_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_h_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[138], real_T c17_C[529],
  real_T c17_b_C[529])
{
  int32_T c17_i1014;
  int32_T c17_i1015;
  real_T c17_b_A[138];
  int32_T c17_i1016;
  real_T c17_b_B[138];
  for (c17_i1014 = 0; c17_i1014 < 529; c17_i1014++) {
    c17_b_C[c17_i1014] = c17_C[c17_i1014];
  }

  for (c17_i1015 = 0; c17_i1015 < 138; c17_i1015++) {
    c17_b_A[c17_i1015] = c17_A[c17_i1015];
  }

  for (c17_i1016 = 0; c17_i1016 < 138; c17_i1016++) {
    c17_b_B[c17_i1016] = c17_B[c17_i1016];
  }

  c17_w_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_q_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_i_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[529], real_T c17_C[529],
  real_T c17_b_C[529])
{
  int32_T c17_i1017;
  int32_T c17_i1018;
  real_T c17_b_A[529];
  int32_T c17_i1019;
  real_T c17_b_B[529];
  for (c17_i1017 = 0; c17_i1017 < 529; c17_i1017++) {
    c17_b_C[c17_i1017] = c17_C[c17_i1017];
  }

  for (c17_i1018 = 0; c17_i1018 < 529; c17_i1018++) {
    c17_b_A[c17_i1018] = c17_A[c17_i1018];
  }

  for (c17_i1019 = 0; c17_i1019 < 529; c17_i1019++) {
    c17_b_B[c17_i1019] = c17_B[c17_i1019];
  }

  c17_x_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_diag(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c17_v[23], real_T c17_d[529])
{
  int32_T c17_i1020;
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_a;
  int32_T c17_c;
  for (c17_i1020 = 0; c17_i1020 < 529; c17_i1020++) {
    c17_d[c17_i1020] = 0.0;
  }

  for (c17_j = 1; c17_j < 24; c17_j++) {
    c17_b_j = c17_j;
    c17_a = c17_b_j;
    c17_c = c17_a;
    c17_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c17_b_j), 1, 23, 1, 0) + 23 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
             "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_c), 1, 23, 2, 0) -
            1)) - 1] = c17_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 23, 1, 0) - 1];
  }
}

static void c17_b_pinv(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_A[529], real_T c17_tol, real_T c17_X[529])
{
  real_T c17_b_tol;
  int32_T c17_i1021;
  int32_T c17_i1022;
  static real_T c17_b_A[529];
  static real_T c17_V[529];
  static real_T c17_S[529];
  static real_T c17_U[529];
  int32_T c17_r;
  int32_T c17_k;
  int32_T c17_b_k;
  int32_T c17_a;
  int32_T c17_vcol;
  int32_T c17_b_r;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_j;
  int32_T c17_b_j;
  real_T c17_d9;
  int32_T c17_b_a;
  int32_T c17_i1023;
  static real_T c17_b_V[529];
  int32_T c17_i1024;
  static real_T c17_b_U[529];
  boolean_T exitg1;
  c17_b_tol = c17_tol;
  c17_o_eml_scalar_eg(chartInstance);
  for (c17_i1021 = 0; c17_i1021 < 529; c17_i1021++) {
    c17_X[c17_i1021] = 0.0;
  }

  for (c17_i1022 = 0; c17_i1022 < 529; c17_i1022++) {
    c17_b_A[c17_i1022] = c17_A[c17_i1022];
  }

  c17_svd(chartInstance, c17_b_A, c17_U, c17_S, c17_V);
  c17_r = 0;
  c17_k = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c17_k < 24)) {
    c17_b_k = c17_k;
    if (!(c17_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 23, 1, 0) + 23 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 2, 0) -
           1)) - 1] > c17_b_tol)) {
      exitg1 = TRUE;
    } else {
      c17_a = c17_r + 1;
      c17_r = c17_a;
      c17_k++;
    }
  }

  if (c17_r > 0) {
    c17_vcol = 1;
    c17_b_r = c17_r;
    c17_b = c17_b_r;
    c17_b_b = c17_b;
    if (1 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_j = 1; c17_j <= c17_b_r; c17_j++) {
      c17_b_j = c17_j;
      c17_d9 = c17_eml_div(chartInstance, 1.0, c17_S
                           [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 23, 1, 0) + 23 *
        (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
        c17_b_j), 1, 23, 2, 0) - 1)) - 1]);
      c17_n_eml_xscal(chartInstance, c17_d9, c17_V, c17_vcol);
      c17_b_a = c17_vcol + 23;
      c17_vcol = c17_b_a;
    }

    for (c17_i1023 = 0; c17_i1023 < 529; c17_i1023++) {
      c17_b_V[c17_i1023] = c17_V[c17_i1023];
    }

    for (c17_i1024 = 0; c17_i1024 < 529; c17_i1024++) {
      c17_b_U[c17_i1024] = c17_U[c17_i1024];
    }

    c17_y_eml_xgemm(chartInstance, c17_r, c17_b_V, c17_b_U, c17_X);
  }
}

static void c17_svd(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c17_A[529], real_T c17_U[529], real_T c17_S[529],
                    real_T c17_V[529])
{
  int32_T c17_k;
  int32_T c17_b_k;
  int32_T c17_i1025;
  static real_T c17_b_A[529];
  real_T c17_s[23];
  int32_T c17_i1026;
  int32_T c17_c_k;
  real_T c17_d_k;
  for (c17_k = 1; c17_k < 530; c17_k++) {
    c17_b_k = c17_k;
    if (!c17_isfinite(chartInstance, c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 529, 1, 0) - 1]))
    {
      c17_eml_error(chartInstance);
    }
  }

  for (c17_i1025 = 0; c17_i1025 < 529; c17_i1025++) {
    c17_b_A[c17_i1025] = c17_A[c17_i1025];
  }

  c17_b_eml_xgesvd(chartInstance, c17_b_A, c17_U, c17_s, c17_V);
  for (c17_i1026 = 0; c17_i1026 < 529; c17_i1026++) {
    c17_S[c17_i1026] = 0.0;
  }

  for (c17_c_k = 0; c17_c_k < 23; c17_c_k++) {
    c17_d_k = 1.0 + (real_T)c17_c_k;
    c17_S[((int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c17_d_k), 1, 23, 1, 0) + 23 * ((int32_T)
            (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", c17_d_k), 1, 23, 2, 0) - 1)) - 1] = c17_s[(int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c17_d_k),
      1, 23, 1, 0) - 1];
  }
}

static void c17_b_eml_xgesvd(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_U[529], real_T c17_S[23], real_T
  c17_V[529])
{
  int32_T c17_i1027;
  static real_T c17_b_A[529];
  int32_T c17_i1028;
  real_T c17_s[23];
  int32_T c17_i1029;
  real_T c17_e[23];
  int32_T c17_i1030;
  real_T c17_work[23];
  int32_T c17_i1031;
  int32_T c17_i1032;
  static real_T c17_Vf[529];
  int32_T c17_q;
  int32_T c17_b_q;
  int32_T c17_a;
  int32_T c17_qp1;
  int32_T c17_b_a;
  int32_T c17_qm1;
  int32_T c17_b;
  int32_T c17_c;
  int32_T c17_c_a;
  int32_T c17_b_b;
  int32_T c17_qq;
  int32_T c17_c_b;
  int32_T c17_nmq;
  int32_T c17_d_a;
  int32_T c17_nmqp1;
  int32_T c17_i1033;
  static real_T c17_c_A[529];
  real_T c17_nrm;
  real_T c17_absx;
  real_T c17_d;
  real_T c17_y;
  real_T c17_d10;
  int32_T c17_b_qp1;
  boolean_T c17_overflow;
  int32_T c17_jj;
  int32_T c17_b_jj;
  int32_T c17_e_a;
  int32_T c17_b_c;
  int32_T c17_d_b;
  int32_T c17_c_c;
  int32_T c17_f_a;
  int32_T c17_e_b;
  int32_T c17_qjj;
  int32_T c17_i1034;
  static real_T c17_d_A[529];
  int32_T c17_i1035;
  static real_T c17_e_A[529];
  real_T c17_t;
  int32_T c17_c_q;
  boolean_T c17_b_overflow;
  int32_T c17_ii;
  int32_T c17_b_ii;
  int32_T c17_f_b;
  int32_T c17_pmq;
  int32_T c17_i1036;
  real_T c17_b_e[23];
  real_T c17_b_absx;
  real_T c17_b_d;
  real_T c17_b_y;
  real_T c17_d11;
  int32_T c17_c_qp1;
  boolean_T c17_c_overflow;
  int32_T c17_c_ii;
  int32_T c17_d_qp1;
  boolean_T c17_d_overflow;
  int32_T c17_c_jj;
  int32_T c17_g_a;
  int32_T c17_d_c;
  int32_T c17_g_b;
  int32_T c17_e_c;
  int32_T c17_h_a;
  int32_T c17_h_b;
  int32_T c17_qp1jj;
  int32_T c17_i1037;
  static real_T c17_f_A[529];
  int32_T c17_e_qp1;
  boolean_T c17_e_overflow;
  int32_T c17_d_jj;
  int32_T c17_i_a;
  int32_T c17_f_c;
  int32_T c17_i_b;
  int32_T c17_g_c;
  int32_T c17_j_a;
  int32_T c17_j_b;
  int32_T c17_i1038;
  real_T c17_b_work[23];
  int32_T c17_f_qp1;
  boolean_T c17_f_overflow;
  int32_T c17_d_ii;
  int32_T c17_m;
  int32_T c17_e_ii;
  int32_T c17_d_q;
  int32_T c17_k_a;
  int32_T c17_k_b;
  int32_T c17_l_a;
  int32_T c17_m_a;
  int32_T c17_h_c;
  int32_T c17_l_b;
  int32_T c17_i_c;
  int32_T c17_n_a;
  int32_T c17_m_b;
  int32_T c17_g_qp1;
  boolean_T c17_g_overflow;
  int32_T c17_e_jj;
  int32_T c17_o_a;
  int32_T c17_j_c;
  int32_T c17_n_b;
  int32_T c17_k_c;
  int32_T c17_p_a;
  int32_T c17_o_b;
  int32_T c17_i1039;
  static real_T c17_b_U[529];
  int32_T c17_i1040;
  real_T c17_c_U[529];
  int32_T c17_e_q;
  boolean_T c17_h_overflow;
  int32_T c17_f_ii;
  int32_T c17_q_a;
  int32_T c17_i1041;
  int32_T c17_p_b;
  int32_T c17_q_b;
  boolean_T c17_i_overflow;
  int32_T c17_g_ii;
  int32_T c17_h_ii;
  int32_T c17_f_q;
  int32_T c17_r_a;
  int32_T c17_r_b;
  int32_T c17_s_a;
  int32_T c17_l_c;
  int32_T c17_s_b;
  int32_T c17_m_c;
  int32_T c17_t_a;
  int32_T c17_t_b;
  int32_T c17_qp1q;
  int32_T c17_h_qp1;
  boolean_T c17_j_overflow;
  int32_T c17_f_jj;
  int32_T c17_u_a;
  int32_T c17_n_c;
  int32_T c17_u_b;
  int32_T c17_o_c;
  int32_T c17_v_a;
  int32_T c17_v_b;
  int32_T c17_i1042;
  real_T c17_b_Vf[529];
  int32_T c17_i1043;
  real_T c17_c_Vf[529];
  int32_T c17_i_ii;
  int32_T c17_g_q;
  real_T c17_rt;
  real_T c17_r;
  int32_T c17_w_a;
  int32_T c17_p_c;
  int32_T c17_w_b;
  int32_T c17_q_c;
  int32_T c17_x_b;
  int32_T c17_colq;
  int32_T c17_x_a;
  int32_T c17_r_c;
  int32_T c17_y_a;
  int32_T c17_s_c;
  real_T c17_ab_a;
  real_T c17_y_b;
  real_T c17_c_y;
  int32_T c17_ab_b;
  int32_T c17_t_c;
  int32_T c17_bb_b;
  int32_T c17_colqp1;
  real_T c17_iter;
  real_T c17_tiny;
  real_T c17_snorm;
  int32_T c17_j_ii;
  real_T c17_varargin_1;
  real_T c17_varargin_2;
  real_T c17_b_varargin_2;
  real_T c17_varargin_3;
  real_T c17_x;
  real_T c17_d_y;
  real_T c17_b_x;
  real_T c17_e_y;
  real_T c17_xk;
  real_T c17_yk;
  real_T c17_c_x;
  real_T c17_f_y;
  real_T c17_maxval;
  real_T c17_b_varargin_1;
  real_T c17_c_varargin_2;
  real_T c17_d_varargin_2;
  real_T c17_b_varargin_3;
  real_T c17_d_x;
  real_T c17_g_y;
  real_T c17_e_x;
  real_T c17_h_y;
  real_T c17_b_xk;
  real_T c17_b_yk;
  real_T c17_f_x;
  real_T c17_i_y;
  int32_T c17_bb_a;
  int32_T c17_cb_a;
  int32_T c17_i1044;
  boolean_T c17_k_overflow;
  int32_T c17_k_ii;
  int32_T c17_db_a;
  int32_T c17_u_c;
  real_T c17_test0;
  real_T c17_ztest0;
  real_T c17_cb_b;
  real_T c17_j_y;
  real_T c17_db_b;
  real_T c17_k_y;
  int32_T c17_eb_a;
  int32_T c17_v_c;
  real_T c17_kase;
  int32_T c17_qs;
  int32_T c17_b_m;
  int32_T c17_h_q;
  int32_T c17_fb_a;
  int32_T c17_eb_b;
  int32_T c17_gb_a;
  int32_T c17_fb_b;
  boolean_T c17_l_overflow;
  int32_T c17_l_ii;
  real_T c17_test;
  int32_T c17_hb_a;
  int32_T c17_w_c;
  int32_T c17_ib_a;
  int32_T c17_x_c;
  real_T c17_ztest;
  real_T c17_gb_b;
  real_T c17_l_y;
  int32_T c17_jb_a;
  int32_T c17_kb_a;
  int32_T c17_y_c;
  real_T c17_f;
  int32_T c17_lb_a;
  int32_T c17_ab_c;
  int32_T c17_mb_a;
  int32_T c17_i1045;
  int32_T c17_i_q;
  int32_T c17_nb_a;
  int32_T c17_hb_b;
  int32_T c17_ob_a;
  int32_T c17_ib_b;
  boolean_T c17_m_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_t1;
  real_T c17_b_t1;
  real_T c17_b_f;
  real_T c17_sn;
  real_T c17_cs;
  real_T c17_b_cs;
  real_T c17_b_sn;
  int32_T c17_pb_a;
  int32_T c17_km1;
  real_T c17_qb_a;
  real_T c17_jb_b;
  real_T c17_rb_a;
  real_T c17_kb_b;
  real_T c17_m_y;
  int32_T c17_sb_a;
  int32_T c17_bb_c;
  int32_T c17_lb_b;
  int32_T c17_cb_c;
  int32_T c17_mb_b;
  int32_T c17_colk;
  int32_T c17_tb_a;
  int32_T c17_db_c;
  int32_T c17_nb_b;
  int32_T c17_eb_c;
  int32_T c17_ob_b;
  int32_T c17_colm;
  int32_T c17_ub_a;
  int32_T c17_j_q;
  int32_T c17_c_m;
  int32_T c17_vb_a;
  int32_T c17_pb_b;
  int32_T c17_wb_a;
  int32_T c17_qb_b;
  boolean_T c17_n_overflow;
  int32_T c17_c_k;
  real_T c17_c_t1;
  real_T c17_unusedU0;
  real_T c17_c_sn;
  real_T c17_c_cs;
  real_T c17_xb_a;
  real_T c17_rb_b;
  real_T c17_yb_a;
  real_T c17_sb_b;
  real_T c17_n_y;
  int32_T c17_ac_a;
  int32_T c17_fb_c;
  int32_T c17_tb_b;
  int32_T c17_gb_c;
  int32_T c17_ub_b;
  int32_T c17_bc_a;
  int32_T c17_hb_c;
  int32_T c17_vb_b;
  int32_T c17_ib_c;
  int32_T c17_wb_b;
  int32_T c17_colqm1;
  int32_T c17_cc_a;
  int32_T c17_mm1;
  real_T c17_d12;
  real_T c17_d13;
  real_T c17_d14;
  real_T c17_d15;
  real_T c17_d16;
  real_T c17_c_varargin_1[5];
  int32_T c17_ixstart;
  real_T c17_mtmp;
  real_T c17_g_x;
  boolean_T c17_xb_b;
  int32_T c17_ix;
  int32_T c17_b_ix;
  real_T c17_h_x;
  boolean_T c17_yb_b;
  int32_T c17_dc_a;
  int32_T c17_i1046;
  boolean_T c17_o_overflow;
  int32_T c17_c_ix;
  real_T c17_ec_a;
  real_T c17_ac_b;
  boolean_T c17_p;
  real_T c17_b_mtmp;
  real_T c17_scale;
  real_T c17_sm;
  real_T c17_smm1;
  real_T c17_emm1;
  real_T c17_sqds;
  real_T c17_eqds;
  real_T c17_fc_a;
  real_T c17_bc_b;
  real_T c17_o_y;
  real_T c17_gc_a;
  real_T c17_cc_b;
  real_T c17_p_y;
  real_T c17_dc_b;
  real_T c17_hc_a;
  real_T c17_ec_b;
  real_T c17_jb_c;
  real_T c17_ic_a;
  real_T c17_fc_b;
  real_T c17_shift;
  real_T c17_jc_a;
  real_T c17_gc_b;
  real_T c17_q_y;
  real_T c17_kc_a;
  real_T c17_hc_b;
  real_T c17_r_y;
  real_T c17_lc_a;
  real_T c17_ic_b;
  real_T c17_g;
  int32_T c17_k_q;
  int32_T c17_b_mm1;
  int32_T c17_mc_a;
  int32_T c17_jc_b;
  int32_T c17_nc_a;
  int32_T c17_kc_b;
  boolean_T c17_p_overflow;
  int32_T c17_d_k;
  int32_T c17_oc_a;
  int32_T c17_pc_a;
  int32_T c17_kp1;
  real_T c17_c_f;
  real_T c17_unusedU1;
  real_T c17_d_sn;
  real_T c17_d_cs;
  real_T c17_qc_a;
  real_T c17_lc_b;
  real_T c17_s_y;
  real_T c17_rc_a;
  real_T c17_mc_b;
  real_T c17_t_y;
  real_T c17_sc_a;
  real_T c17_nc_b;
  real_T c17_u_y;
  real_T c17_tc_a;
  real_T c17_oc_b;
  real_T c17_v_y;
  real_T c17_uc_a;
  real_T c17_pc_b;
  real_T c17_vc_a;
  real_T c17_qc_b;
  real_T c17_w_y;
  int32_T c17_wc_a;
  int32_T c17_kb_c;
  int32_T c17_rc_b;
  int32_T c17_lb_c;
  int32_T c17_sc_b;
  int32_T c17_tc_b;
  int32_T c17_mb_c;
  int32_T c17_uc_b;
  int32_T c17_colkp1;
  real_T c17_d_f;
  real_T c17_unusedU2;
  real_T c17_e_sn;
  real_T c17_e_cs;
  real_T c17_xc_a;
  real_T c17_vc_b;
  real_T c17_x_y;
  real_T c17_yc_a;
  real_T c17_wc_b;
  real_T c17_y_y;
  real_T c17_ad_a;
  real_T c17_xc_b;
  real_T c17_ab_y;
  real_T c17_bd_a;
  real_T c17_yc_b;
  real_T c17_bb_y;
  real_T c17_cd_a;
  real_T c17_ad_b;
  real_T c17_dd_a;
  real_T c17_bd_b;
  real_T c17_cb_y;
  int32_T c17_ed_a;
  int32_T c17_nb_c;
  int32_T c17_cd_b;
  int32_T c17_ob_c;
  int32_T c17_dd_b;
  int32_T c17_ed_b;
  int32_T c17_pb_c;
  int32_T c17_fd_b;
  int32_T c17_fd_a;
  int32_T c17_qb_c;
  int32_T c17_e_k;
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_i;
  int32_T c17_b_i;
  int32_T c17_rb_c;
  int32_T c17_gd_a;
  int32_T c17_sb_c;
  int32_T c17_gd_b;
  int32_T c17_hd_b;
  int32_T c17_hd_a;
  int32_T c17_id_a;
  int32_T c17_tb_c;
  int32_T c17_jd_a;
  int32_T c17_ub_c;
  int32_T c17_id_b;
  int32_T c17_jd_b;
  int32_T c17_vb_c;
  int32_T c17_kd_b;
  int32_T c17_ld_b;
  int32_T c17_wb_c;
  int32_T c17_kd_a;
  int32_T c17_xb_c;
  int32_T c17_md_b;
  int32_T c17_nd_b;
  int32_T c17_yb_c;
  int32_T c17_od_b;
  int32_T c17_pd_b;
  int32_T c17_ld_a;
  real_T c17_d17;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard11 = FALSE;
  for (c17_i1027 = 0; c17_i1027 < 529; c17_i1027++) {
    c17_b_A[c17_i1027] = c17_A[c17_i1027];
  }

  c17_o_eml_scalar_eg(chartInstance);
  for (c17_i1028 = 0; c17_i1028 < 23; c17_i1028++) {
    c17_s[c17_i1028] = 0.0;
  }

  for (c17_i1029 = 0; c17_i1029 < 23; c17_i1029++) {
    c17_e[c17_i1029] = 0.0;
  }

  for (c17_i1030 = 0; c17_i1030 < 23; c17_i1030++) {
    c17_work[c17_i1030] = 0.0;
  }

  for (c17_i1031 = 0; c17_i1031 < 529; c17_i1031++) {
    c17_U[c17_i1031] = 0.0;
  }

  for (c17_i1032 = 0; c17_i1032 < 529; c17_i1032++) {
    c17_Vf[c17_i1032] = 0.0;
  }

  for (c17_q = 1; c17_q < 23; c17_q++) {
    c17_b_q = c17_q;
    c17_a = c17_b_q + 1;
    c17_qp1 = c17_a;
    c17_b_a = c17_b_q;
    c17_qm1 = c17_b_a;
    c17_b = c17_qm1 - 1;
    c17_c = 23 * c17_b;
    c17_c_a = c17_b_q;
    c17_b_b = c17_c;
    c17_qq = c17_c_a + c17_b_b;
    c17_c_b = c17_b_q;
    c17_nmq = 23 - c17_c_b;
    c17_d_a = c17_nmq + 1;
    c17_nmqp1 = c17_d_a;
    if (c17_b_q <= 22) {
      for (c17_i1033 = 0; c17_i1033 < 529; c17_i1033++) {
        c17_c_A[c17_i1033] = c17_b_A[c17_i1033];
      }

      c17_nrm = c17_c_eml_xnrm2(chartInstance, c17_nmqp1, c17_c_A, c17_qq);
      if (c17_nrm > 0.0) {
        c17_absx = c17_nrm;
        c17_d = c17_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_qq), 1, 529, 1, 0) - 1];
        if (c17_d < 0.0) {
          c17_y = -c17_absx;
        } else {
          c17_y = c17_absx;
        }

        c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] = c17_y;
        c17_d10 = c17_eml_div(chartInstance, 1.0,
                              c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1]);
        c17_l_eml_xscal(chartInstance, c17_nmqp1, c17_d10, c17_b_A, c17_qq);
        c17_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qq), 1, 529, 1, 0) - 1] =
          c17_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qq), 1, 529, 1, 0) - 1] + 1.0;
        c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] =
          -c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1];
      } else {
        c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] = 0.0;
      }
    }

    c17_b_qp1 = c17_qp1;
    c17_overflow = FALSE;
    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_jj = c17_b_qp1; c17_jj < 24; c17_jj++) {
      c17_b_jj = c17_jj;
      c17_e_a = c17_b_jj;
      c17_b_c = c17_e_a;
      c17_d_b = c17_b_c - 1;
      c17_c_c = 23 * c17_d_b;
      c17_f_a = c17_b_q;
      c17_e_b = c17_c_c;
      c17_qjj = c17_f_a + c17_e_b;
      if (c17_b_q <= 22) {
        if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 23, 1, 0) - 1] != 0.0) {
          for (c17_i1034 = 0; c17_i1034 < 529; c17_i1034++) {
            c17_d_A[c17_i1034] = c17_b_A[c17_i1034];
          }

          for (c17_i1035 = 0; c17_i1035 < 529; c17_i1035++) {
            c17_e_A[c17_i1035] = c17_b_A[c17_i1035];
          }

          c17_t = c17_c_eml_xdotc(chartInstance, c17_nmqp1, c17_d_A, c17_qq,
            c17_e_A, c17_qjj);
          c17_t = -c17_eml_div(chartInstance, c17_t, c17_b_A
                               [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) + 23 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1]);
          c17_l_eml_xaxpy(chartInstance, c17_nmqp1, c17_t, c17_qq, c17_b_A,
                          c17_qjj);
        }
      }

      c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_b_jj), 1, 23, 1, 0) - 1] =
        c17_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_qjj), 1, 529, 1, 0) - 1];
    }

    if (c17_b_q <= 22) {
      c17_c_q = c17_b_q;
      c17_b_overflow = FALSE;
      if (c17_b_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
      }

      for (c17_ii = c17_c_q; c17_ii < 24; c17_ii++) {
        c17_b_ii = c17_ii;
        c17_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_ii), 1, 23, 1, 0) + 23 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1] = c17_b_A
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_ii), 1, 23, 1, 0) + 23 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1];
      }
    }

    if (c17_b_q <= 21) {
      c17_f_b = c17_b_q;
      c17_pmq = 23 - c17_f_b;
      for (c17_i1036 = 0; c17_i1036 < 23; c17_i1036++) {
        c17_b_e[c17_i1036] = c17_e[c17_i1036];
      }

      c17_nrm = c17_d_eml_xnrm2(chartInstance, c17_pmq, c17_b_e, c17_qp1);
      if (c17_nrm == 0.0) {
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] = 0.0;
      } else {
        c17_b_absx = c17_nrm;
        c17_b_d = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_qp1), 1, 23, 1, 0) - 1];
        if (c17_b_d < 0.0) {
          c17_b_y = -c17_b_absx;
        } else {
          c17_b_y = c17_b_absx;
        }

        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] = c17_b_y;
        c17_d11 = c17_eml_div(chartInstance, 1.0,
                              c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1]);
        c17_m_eml_xscal(chartInstance, c17_pmq, c17_d11, c17_e, c17_qp1);
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qp1), 1, 23, 1, 0) - 1] =
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qp1), 1, 23, 1, 0) - 1] + 1.0;
      }

      c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_b_q), 1, 23, 1, 0) - 1] = -c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1];
      if (c17_qp1 <= 23) {
        if (c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 23, 1, 0) - 1] != 0.0) {
          c17_c_qp1 = c17_qp1;
          c17_c_overflow = FALSE;
          if (c17_c_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_c_overflow);
          }

          for (c17_c_ii = c17_c_qp1; c17_c_ii < 24; c17_c_ii++) {
            c17_b_ii = c17_c_ii;
            c17_work[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
              ("", (real_T)c17_b_ii), 1, 23, 1, 0) - 1] = 0.0;
          }

          c17_d_qp1 = c17_qp1;
          c17_d_overflow = FALSE;
          if (c17_d_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_d_overflow);
          }

          for (c17_c_jj = c17_d_qp1; c17_c_jj < 24; c17_c_jj++) {
            c17_b_jj = c17_c_jj;
            c17_g_a = c17_b_jj;
            c17_d_c = c17_g_a;
            c17_g_b = c17_d_c - 1;
            c17_e_c = 23 * c17_g_b;
            c17_h_a = c17_qp1;
            c17_h_b = c17_e_c;
            c17_qp1jj = c17_h_a + c17_h_b;
            for (c17_i1037 = 0; c17_i1037 < 529; c17_i1037++) {
              c17_f_A[c17_i1037] = c17_b_A[c17_i1037];
            }

            c17_m_eml_xaxpy(chartInstance, c17_nmq,
                            c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_jj), 1, 23, 1, 0) - 1],
                            c17_f_A, c17_qp1jj, c17_work, c17_qp1);
          }

          c17_e_qp1 = c17_qp1;
          c17_e_overflow = FALSE;
          if (c17_e_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_e_overflow);
          }

          for (c17_d_jj = c17_e_qp1; c17_d_jj < 24; c17_d_jj++) {
            c17_b_jj = c17_d_jj;
            c17_t = c17_eml_div(chartInstance,
                                -c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_jj), 1, 23, 1, 0) - 1],
                                c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_qp1), 1, 23, 1, 0) - 1]);
            c17_i_a = c17_b_jj;
            c17_f_c = c17_i_a;
            c17_i_b = c17_f_c - 1;
            c17_g_c = 23 * c17_i_b;
            c17_j_a = c17_qp1;
            c17_j_b = c17_g_c;
            c17_qp1jj = c17_j_a + c17_j_b;
            for (c17_i1038 = 0; c17_i1038 < 23; c17_i1038++) {
              c17_b_work[c17_i1038] = c17_work[c17_i1038];
            }

            c17_n_eml_xaxpy(chartInstance, c17_nmq, c17_t, c17_b_work, c17_qp1,
                            c17_b_A, c17_qp1jj);
          }
        }
      }

      c17_f_qp1 = c17_qp1;
      c17_f_overflow = FALSE;
      if (c17_f_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_f_overflow);
      }

      for (c17_d_ii = c17_f_qp1; c17_d_ii < 24; c17_d_ii++) {
        c17_b_ii = c17_d_ii;
        c17_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_ii), 1, 23, 1, 0) + 23 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1] =
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_ii), 1, 23, 1, 0) - 1];
      }
    }
  }

  c17_m = 23;
  c17_s[22] = c17_b_A[528];
  c17_e[21] = c17_b_A[527];
  c17_e[22] = 0.0;
  for (c17_e_ii = 1; c17_e_ii < 24; c17_e_ii++) {
    c17_b_ii = c17_e_ii;
    c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_ii), 1, 23, 1, 0) + 505] = 0.0;
  }

  c17_U[528] = 1.0;
  for (c17_d_q = 22; c17_d_q > 0; c17_d_q--) {
    c17_b_q = c17_d_q;
    c17_k_a = c17_b_q;
    c17_qp1 = c17_k_a;
    c17_k_b = c17_b_q;
    c17_nmq = 23 - c17_k_b;
    c17_l_a = c17_nmq + 1;
    c17_nmqp1 = c17_l_a;
    c17_m_a = c17_b_q;
    c17_h_c = c17_m_a;
    c17_l_b = c17_h_c - 1;
    c17_i_c = 23 * c17_l_b;
    c17_n_a = c17_b_q;
    c17_m_b = c17_i_c;
    c17_qq = c17_n_a + c17_m_b;
    if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] != 0.0) {
      c17_g_qp1 = c17_qp1 + 1;
      c17_g_overflow = FALSE;
      if (c17_g_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_g_overflow);
      }

      for (c17_e_jj = c17_g_qp1; c17_e_jj < 24; c17_e_jj++) {
        c17_b_jj = c17_e_jj;
        c17_o_a = c17_b_jj;
        c17_j_c = c17_o_a;
        c17_n_b = c17_j_c - 1;
        c17_k_c = 23 * c17_n_b;
        c17_p_a = c17_b_q;
        c17_o_b = c17_k_c;
        c17_qjj = c17_p_a + c17_o_b;
        for (c17_i1039 = 0; c17_i1039 < 529; c17_i1039++) {
          c17_b_U[c17_i1039] = c17_U[c17_i1039];
        }

        for (c17_i1040 = 0; c17_i1040 < 529; c17_i1040++) {
          c17_c_U[c17_i1040] = c17_U[c17_i1040];
        }

        c17_t = c17_c_eml_xdotc(chartInstance, c17_nmqp1, c17_b_U, c17_qq,
          c17_c_U, c17_qjj);
        c17_t = -c17_eml_div(chartInstance, c17_t,
                             c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_qq), 1, 529, 1, 0) - 1]);
        c17_l_eml_xaxpy(chartInstance, c17_nmqp1, c17_t, c17_qq, c17_U, c17_qjj);
      }

      c17_e_q = c17_b_q;
      c17_h_overflow = FALSE;
      if (c17_h_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_h_overflow);
      }

      for (c17_f_ii = c17_e_q; c17_f_ii < 24; c17_f_ii++) {
        c17_b_ii = c17_f_ii;
        c17_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_ii), 1, 23, 1, 0) + 23 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1] = -c17_U
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_ii), 1, 23, 1, 0) + 23 *
            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1];
      }

      c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_qq), 1, 529, 1, 0) - 1] = c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_qq), 1, 529, 1, 0) - 1]
        + 1.0;
      c17_q_a = c17_b_q - 1;
      c17_i1041 = c17_q_a;
      c17_p_b = c17_i1041;
      c17_q_b = c17_p_b;
      if (1 > c17_q_b) {
        c17_i_overflow = FALSE;
      } else {
        c17_i_overflow = (c17_q_b > 2147483646);
      }

      if (c17_i_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_i_overflow);
      }

      for (c17_g_ii = 1; c17_g_ii <= c17_i1041; c17_g_ii++) {
        c17_b_ii = c17_g_ii;
        c17_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_ii), 1, 23, 1, 0) + 23 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1] = 0.0;
      }
    } else {
      for (c17_h_ii = 1; c17_h_ii < 24; c17_h_ii++) {
        c17_b_ii = c17_h_ii;
        c17_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_ii), 1, 23, 1, 0) + 23 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1] = 0.0;
      }

      c17_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_qq), 1, 529, 1, 0) - 1] = 1.0;
    }
  }

  for (c17_f_q = 23; c17_f_q > 0; c17_f_q--) {
    c17_b_q = c17_f_q;
    if (c17_b_q <= 21) {
      if (c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_q), 1, 23, 1, 0) - 1] != 0.0) {
        c17_r_a = c17_b_q + 1;
        c17_qp1 = c17_r_a;
        c17_r_b = c17_b_q;
        c17_pmq = 23 - c17_r_b;
        c17_s_a = c17_b_q;
        c17_l_c = c17_s_a;
        c17_s_b = c17_l_c - 1;
        c17_m_c = 23 * c17_s_b;
        c17_t_a = c17_qp1;
        c17_t_b = c17_m_c;
        c17_qp1q = c17_t_a + c17_t_b;
        c17_h_qp1 = c17_qp1;
        c17_j_overflow = FALSE;
        if (c17_j_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_j_overflow);
        }

        for (c17_f_jj = c17_h_qp1; c17_f_jj < 24; c17_f_jj++) {
          c17_b_jj = c17_f_jj;
          c17_u_a = c17_b_jj;
          c17_n_c = c17_u_a;
          c17_u_b = c17_n_c - 1;
          c17_o_c = 23 * c17_u_b;
          c17_v_a = c17_qp1;
          c17_v_b = c17_o_c;
          c17_qp1jj = c17_v_a + c17_v_b;
          for (c17_i1042 = 0; c17_i1042 < 529; c17_i1042++) {
            c17_b_Vf[c17_i1042] = c17_Vf[c17_i1042];
          }

          for (c17_i1043 = 0; c17_i1043 < 529; c17_i1043++) {
            c17_c_Vf[c17_i1043] = c17_Vf[c17_i1043];
          }

          c17_t = c17_c_eml_xdotc(chartInstance, c17_pmq, c17_b_Vf, c17_qp1q,
            c17_c_Vf, c17_qp1jj);
          c17_t = -c17_eml_div(chartInstance, c17_t,
                               c17_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_qp1q), 1, 529, 1, 0) - 1]);
          c17_l_eml_xaxpy(chartInstance, c17_pmq, c17_t, c17_qp1q, c17_Vf,
                          c17_qp1jj);
        }
      }
    }

    for (c17_i_ii = 1; c17_i_ii < 24; c17_i_ii++) {
      c17_b_ii = c17_i_ii;
      c17_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c17_b_ii), 1, 23, 1, 0) + 23 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c17_b_q), 1, 23, 2, 0) - 1)) - 1] = 0.0;
    }

    c17_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 23, 1, 0) + 23 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 2, 0)
             - 1)) - 1] = 1.0;
  }

  for (c17_g_q = 1; c17_g_q < 24; c17_g_q++) {
    c17_b_q = c17_g_q;
    if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] != 0.0) {
      c17_rt = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1]);
      c17_r = c17_eml_div(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1],
                          c17_rt);
      c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_b_q), 1, 23, 1, 0) - 1] = c17_rt;
      if (c17_b_q < 23) {
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] = c17_eml_div(chartInstance,
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1], c17_r);
      }

      if (c17_b_q <= 23) {
        c17_w_a = c17_b_q;
        c17_p_c = c17_w_a;
        c17_w_b = c17_p_c - 1;
        c17_q_c = 23 * c17_w_b;
        c17_x_b = c17_q_c;
        c17_colq = c17_x_b;
        c17_n_eml_xscal(chartInstance, c17_r, c17_U, c17_colq + 1);
      }
    }

    if (c17_b_q < 23) {
      if (c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_q), 1, 23, 1, 0) - 1] != 0.0) {
        c17_rt = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1]);
        c17_r = c17_eml_div(chartInstance, c17_rt,
                            c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1]);
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_q), 1, 23, 1, 0) - 1] = c17_rt;
        c17_x_a = c17_b_q;
        c17_r_c = c17_x_a;
        c17_y_a = c17_b_q;
        c17_s_c = c17_y_a;
        c17_ab_a = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)(c17_s_c + 1)), 1, 23, 1, 0) - 1];
        c17_y_b = c17_r;
        c17_c_y = c17_ab_a * c17_y_b;
        c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c17_r_c + 1)), 1, 23, 1, 0) - 1] = c17_c_y;
        c17_ab_b = c17_b_q;
        c17_t_c = 23 * c17_ab_b;
        c17_bb_b = c17_t_c;
        c17_colqp1 = c17_bb_b;
        c17_n_eml_xscal(chartInstance, c17_r, c17_Vf, c17_colqp1 + 1);
      }
    }
  }

  c17_iter = 0.0;
  c17_realmin(chartInstance);
  c17_eps(chartInstance);
  c17_tiny = c17_eml_div(chartInstance, 2.2250738585072014E-308,
    2.2204460492503131E-16);
  c17_snorm = 0.0;
  for (c17_j_ii = 1; c17_j_ii < 24; c17_j_ii++) {
    c17_b_ii = c17_j_ii;
    c17_varargin_1 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ii),
      1, 23, 1, 0) - 1]);
    c17_varargin_2 = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ii),
      1, 23, 1, 0) - 1]);
    c17_b_varargin_2 = c17_varargin_1;
    c17_varargin_3 = c17_varargin_2;
    c17_x = c17_b_varargin_2;
    c17_d_y = c17_varargin_3;
    c17_b_x = c17_x;
    c17_e_y = c17_d_y;
    c17_eml_scalar_eg(chartInstance);
    c17_xk = c17_b_x;
    c17_yk = c17_e_y;
    c17_c_x = c17_xk;
    c17_f_y = c17_yk;
    c17_eml_scalar_eg(chartInstance);
    c17_maxval = muDoubleScalarMax(c17_c_x, c17_f_y);
    c17_b_varargin_1 = c17_snorm;
    c17_c_varargin_2 = c17_maxval;
    c17_d_varargin_2 = c17_b_varargin_1;
    c17_b_varargin_3 = c17_c_varargin_2;
    c17_d_x = c17_d_varargin_2;
    c17_g_y = c17_b_varargin_3;
    c17_e_x = c17_d_x;
    c17_h_y = c17_g_y;
    c17_eml_scalar_eg(chartInstance);
    c17_b_xk = c17_e_x;
    c17_b_yk = c17_h_y;
    c17_f_x = c17_b_xk;
    c17_i_y = c17_b_yk;
    c17_eml_scalar_eg(chartInstance);
    c17_snorm = muDoubleScalarMax(c17_f_x, c17_i_y);
  }

  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c17_m > 0)) {
    if (c17_iter >= 75.0) {
      c17_b_eml_error(chartInstance);
      exitg1 = TRUE;
    } else {
      c17_bb_a = c17_m - 1;
      c17_b_q = c17_bb_a;
      c17_cb_a = c17_m;
      c17_i1044 = c17_cb_a;
      c17_k_overflow = FALSE;
      if (c17_k_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_k_overflow);
      }

      c17_k_ii = c17_i1044 - 1;
      guard3 = FALSE;
      guard4 = FALSE;
      exitg5 = FALSE;
      while ((exitg5 == FALSE) && (c17_k_ii > -1)) {
        c17_b_ii = c17_k_ii;
        c17_b_q = c17_b_ii;
        if (c17_b_ii == 0) {
          exitg5 = TRUE;
        } else {
          c17_db_a = c17_b_ii;
          c17_u_c = c17_db_a;
          c17_test0 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ii), 1, 23, 1, 0)
                              - 1]) + c17_abs(chartInstance,
            c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c17_u_c + 1)), 1, 23, 1, 0) - 1]);
          c17_ztest0 = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ii), 1, 23, 1, 0)
                               - 1]);
          c17_eps(chartInstance);
          c17_cb_b = c17_test0;
          c17_j_y = 2.2204460492503131E-16 * c17_cb_b;
          if (c17_ztest0 <= c17_j_y) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else if (c17_ztest0 <= c17_tiny) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else {
            guard11 = FALSE;
            if (c17_iter > 20.0) {
              c17_eps(chartInstance);
              c17_db_b = c17_snorm;
              c17_k_y = 2.2204460492503131E-16 * c17_db_b;
              if (c17_ztest0 <= c17_k_y) {
                guard3 = TRUE;
                exitg5 = TRUE;
              } else {
                guard11 = TRUE;
              }
            } else {
              guard11 = TRUE;
            }

            if (guard11 == TRUE) {
              c17_k_ii--;
              guard3 = FALSE;
              guard4 = FALSE;
            }
          }
        }
      }

      if (guard4 == TRUE) {
        guard3 = TRUE;
      }

      if (guard3 == TRUE) {
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_ii), 1, 23, 1, 0) - 1] = 0.0;
      }

      c17_eb_a = c17_m;
      c17_v_c = c17_eb_a;
      if (c17_b_q == c17_v_c - 1) {
        c17_kase = 4.0;
      } else {
        c17_qs = c17_m;
        c17_b_m = c17_m;
        c17_h_q = c17_b_q;
        c17_fb_a = c17_b_m;
        c17_eb_b = c17_h_q;
        c17_gb_a = c17_fb_a;
        c17_fb_b = c17_eb_b;
        if (c17_gb_a < c17_fb_b) {
          c17_l_overflow = FALSE;
        } else {
          c17_l_overflow = (c17_fb_b < -2147483647);
        }

        if (c17_l_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_l_overflow);
        }

        c17_l_ii = c17_b_m;
        guard2 = FALSE;
        exitg4 = FALSE;
        while ((exitg4 == FALSE) && (c17_l_ii >= c17_h_q)) {
          c17_b_ii = c17_l_ii;
          c17_qs = c17_b_ii;
          if (c17_b_ii == c17_b_q) {
            exitg4 = TRUE;
          } else {
            c17_test = 0.0;
            if (c17_b_ii < c17_m) {
              c17_test = c17_abs(chartInstance,
                                 c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c17_b_ii), 1, 23, 1, 0) - 1]);
            }

            c17_hb_a = c17_b_q;
            c17_w_c = c17_hb_a;
            if (c17_b_ii > c17_w_c + 1) {
              c17_ib_a = c17_b_ii;
              c17_x_c = c17_ib_a;
              c17_test += c17_abs(chartInstance,
                                  c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)(c17_x_c - 1)), 1, 23, 1, 0) - 1]);
            }

            c17_ztest = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK
                                ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
              c17_b_ii), 1, 23, 1, 0) - 1]);
            c17_eps(chartInstance);
            c17_gb_b = c17_test;
            c17_l_y = 2.2204460492503131E-16 * c17_gb_b;
            if (c17_ztest <= c17_l_y) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else if (c17_ztest <= c17_tiny) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else {
              c17_l_ii--;
              guard2 = FALSE;
            }
          }
        }

        if (guard2 == TRUE) {
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ii), 1, 23, 1, 0) - 1] = 0.0;
        }

        if (c17_qs == c17_b_q) {
          c17_kase = 3.0;
        } else if (c17_qs == c17_m) {
          c17_kase = 1.0;
        } else {
          c17_kase = 2.0;
          c17_b_q = c17_qs;
        }
      }

      c17_jb_a = c17_b_q + 1;
      c17_b_q = c17_jb_a;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c17_kase)) {
       case 1:
        c17_kb_a = c17_m;
        c17_y_c = c17_kb_a;
        c17_f = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)(c17_y_c - 1)), 1, 23, 1, 0) - 1];
        c17_lb_a = c17_m;
        c17_ab_c = c17_lb_a;
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c17_ab_c - 1)), 1, 23, 1, 0) - 1] = 0.0;
        c17_mb_a = c17_m - 1;
        c17_i1045 = c17_mb_a;
        c17_i_q = c17_b_q;
        c17_nb_a = c17_i1045;
        c17_hb_b = c17_i_q;
        c17_ob_a = c17_nb_a;
        c17_ib_b = c17_hb_b;
        if (c17_ob_a < c17_ib_b) {
          c17_m_overflow = FALSE;
        } else {
          c17_m_overflow = (c17_ib_b < -2147483647);
        }

        if (c17_m_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_m_overflow);
        }

        for (c17_k = c17_i1045; c17_k >= c17_i_q; c17_k--) {
          c17_b_k = c17_k;
          c17_t1 = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_b_t1 = c17_t1;
          c17_b_f = c17_f;
          c17_b_eml_xrotg(chartInstance, &c17_b_t1, &c17_b_f, &c17_cs, &c17_sn);
          c17_t1 = c17_b_t1;
          c17_f = c17_b_f;
          c17_b_cs = c17_cs;
          c17_b_sn = c17_sn;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 23, 1, 0) - 1] = c17_t1;
          if (c17_b_k > c17_b_q) {
            c17_pb_a = c17_b_k - 1;
            c17_km1 = c17_pb_a;
            c17_qb_a = -c17_b_sn;
            c17_jb_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_km1), 1, 23, 1, 0) - 1];
            c17_f = c17_qb_a * c17_jb_b;
            c17_rb_a = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_km1), 1, 23, 1, 0) - 1];
            c17_kb_b = c17_b_cs;
            c17_m_y = c17_rb_a * c17_kb_b;
            c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_km1), 1, 23, 1, 0) - 1] = c17_m_y;
          }

          c17_sb_a = c17_b_k;
          c17_bb_c = c17_sb_a;
          c17_lb_b = c17_bb_c - 1;
          c17_cb_c = 23 * c17_lb_b;
          c17_mb_b = c17_cb_c;
          c17_colk = c17_mb_b;
          c17_tb_a = c17_m;
          c17_db_c = c17_tb_a;
          c17_nb_b = c17_db_c - 1;
          c17_eb_c = 23 * c17_nb_b;
          c17_ob_b = c17_eb_c;
          c17_colm = c17_ob_b;
          c17_f_eml_xrot(chartInstance, c17_Vf, c17_colk + 1, c17_colm + 1,
                         c17_b_cs, c17_b_sn);
        }
        break;

       case 2:
        c17_ub_a = c17_b_q - 1;
        c17_qm1 = c17_ub_a;
        c17_f = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_qm1), 1, 23, 1, 0) - 1];
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_qm1), 1, 23, 1, 0) - 1] = 0.0;
        c17_j_q = c17_b_q;
        c17_c_m = c17_m;
        c17_vb_a = c17_j_q;
        c17_pb_b = c17_c_m;
        c17_wb_a = c17_vb_a;
        c17_qb_b = c17_pb_b;
        if (c17_wb_a > c17_qb_b) {
          c17_n_overflow = FALSE;
        } else {
          c17_n_overflow = (c17_qb_b > 2147483646);
        }

        if (c17_n_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_n_overflow);
        }

        for (c17_c_k = c17_j_q; c17_c_k <= c17_c_m; c17_c_k++) {
          c17_b_k = c17_c_k;
          c17_t1 = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_c_t1 = c17_t1;
          c17_unusedU0 = c17_f;
          c17_b_eml_xrotg(chartInstance, &c17_c_t1, &c17_unusedU0, &c17_c_cs,
                          &c17_c_sn);
          c17_t1 = c17_c_t1;
          c17_b_cs = c17_c_cs;
          c17_b_sn = c17_c_sn;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 23, 1, 0) - 1] = c17_t1;
          c17_xb_a = -c17_b_sn;
          c17_rb_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_f = c17_xb_a * c17_rb_b;
          c17_yb_a = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_sb_b = c17_b_cs;
          c17_n_y = c17_yb_a * c17_sb_b;
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 23, 1, 0) - 1] = c17_n_y;
          c17_ac_a = c17_b_k;
          c17_fb_c = c17_ac_a;
          c17_tb_b = c17_fb_c - 1;
          c17_gb_c = 23 * c17_tb_b;
          c17_ub_b = c17_gb_c;
          c17_colk = c17_ub_b;
          c17_bc_a = c17_qm1;
          c17_hb_c = c17_bc_a;
          c17_vb_b = c17_hb_c - 1;
          c17_ib_c = 23 * c17_vb_b;
          c17_wb_b = c17_ib_c;
          c17_colqm1 = c17_wb_b;
          c17_f_eml_xrot(chartInstance, c17_U, c17_colk + 1, c17_colqm1 + 1,
                         c17_b_cs, c17_b_sn);
        }
        break;

       case 3:
        c17_cc_a = c17_m - 1;
        c17_mm1 = c17_cc_a;
        c17_d12 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_m), 1, 23, 1, 0) - 1]);
        c17_d13 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_mm1), 1, 23, 1, 0) - 1]);
        c17_d14 = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_mm1), 1, 23, 1, 0) - 1]);
        c17_d15 = c17_abs(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1]);
        c17_d16 = c17_abs(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1]);
        c17_c_varargin_1[0] = c17_d12;
        c17_c_varargin_1[1] = c17_d13;
        c17_c_varargin_1[2] = c17_d14;
        c17_c_varargin_1[3] = c17_d15;
        c17_c_varargin_1[4] = c17_d16;
        c17_ixstart = 1;
        c17_mtmp = c17_c_varargin_1[0];
        c17_g_x = c17_mtmp;
        c17_xb_b = muDoubleScalarIsNaN(c17_g_x);
        if (c17_xb_b) {
          c17_ix = 2;
          exitg2 = FALSE;
          while ((exitg2 == FALSE) && (c17_ix < 6)) {
            c17_b_ix = c17_ix;
            c17_ixstart = c17_b_ix;
            c17_h_x = c17_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 5, 1, 0) - 1];
            c17_yb_b = muDoubleScalarIsNaN(c17_h_x);
            if (!c17_yb_b) {
              c17_mtmp = c17_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 5, 1, 0) -
                1];
              exitg2 = TRUE;
            } else {
              c17_ix++;
            }
          }
        }

        if (c17_ixstart < 5) {
          c17_dc_a = c17_ixstart;
          c17_i1046 = c17_dc_a;
          c17_o_overflow = FALSE;
          if (c17_o_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_o_overflow);
          }

          for (c17_c_ix = c17_i1046 + 1; c17_c_ix < 6; c17_c_ix++) {
            c17_b_ix = c17_c_ix;
            c17_ec_a = c17_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 5, 1, 0) - 1];
            c17_ac_b = c17_mtmp;
            c17_p = (c17_ec_a > c17_ac_b);
            if (c17_p) {
              c17_mtmp = c17_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 5, 1, 0) -
                1];
            }
          }
        }

        c17_b_mtmp = c17_mtmp;
        c17_scale = c17_b_mtmp;
        c17_sm = c17_eml_div(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_m), 1,
          23, 1, 0) - 1], c17_scale);
        c17_smm1 = c17_eml_div(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_mm1), 1, 23, 1, 0) - 1],
          c17_scale);
        c17_emm1 = c17_eml_div(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_mm1), 1, 23, 1, 0) - 1],
          c17_scale);
        c17_sqds = c17_eml_div(chartInstance, c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1],
          c17_scale);
        c17_eqds = c17_eml_div(chartInstance, c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1],
          c17_scale);
        c17_fc_a = c17_smm1 + c17_sm;
        c17_bc_b = c17_smm1 - c17_sm;
        c17_o_y = c17_fc_a * c17_bc_b;
        c17_gc_a = c17_emm1;
        c17_cc_b = c17_emm1;
        c17_p_y = c17_gc_a * c17_cc_b;
        c17_dc_b = c17_eml_div(chartInstance, c17_o_y + c17_p_y, 2.0);
        c17_hc_a = c17_sm;
        c17_ec_b = c17_emm1;
        c17_jb_c = c17_hc_a * c17_ec_b;
        c17_ic_a = c17_jb_c;
        c17_fc_b = c17_jb_c;
        c17_jb_c = c17_ic_a * c17_fc_b;
        c17_shift = 0.0;
        guard1 = FALSE;
        if (c17_dc_b != 0.0) {
          guard1 = TRUE;
        } else {
          if (c17_jb_c != 0.0) {
            guard1 = TRUE;
          }
        }

        if (guard1 == TRUE) {
          c17_jc_a = c17_dc_b;
          c17_gc_b = c17_dc_b;
          c17_q_y = c17_jc_a * c17_gc_b;
          c17_shift = c17_q_y + c17_jb_c;
          c17_b_sqrt(chartInstance, &c17_shift);
          if (c17_dc_b < 0.0) {
            c17_shift = -c17_shift;
          }

          c17_shift = c17_eml_div(chartInstance, c17_jb_c, c17_dc_b + c17_shift);
        }

        c17_kc_a = c17_sqds + c17_sm;
        c17_hc_b = c17_sqds - c17_sm;
        c17_r_y = c17_kc_a * c17_hc_b;
        c17_f = c17_r_y + c17_shift;
        c17_lc_a = c17_sqds;
        c17_ic_b = c17_eqds;
        c17_g = c17_lc_a * c17_ic_b;
        c17_k_q = c17_b_q;
        c17_b_mm1 = c17_mm1;
        c17_mc_a = c17_k_q;
        c17_jc_b = c17_b_mm1;
        c17_nc_a = c17_mc_a;
        c17_kc_b = c17_jc_b;
        if (c17_nc_a > c17_kc_b) {
          c17_p_overflow = FALSE;
        } else {
          c17_p_overflow = (c17_kc_b > 2147483646);
        }

        if (c17_p_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_p_overflow);
        }

        for (c17_d_k = c17_k_q; c17_d_k <= c17_b_mm1; c17_d_k++) {
          c17_b_k = c17_d_k;
          c17_oc_a = c17_b_k;
          c17_km1 = c17_oc_a;
          c17_pc_a = c17_b_k + 1;
          c17_kp1 = c17_pc_a;
          c17_c_f = c17_f;
          c17_unusedU1 = c17_g;
          c17_b_eml_xrotg(chartInstance, &c17_c_f, &c17_unusedU1, &c17_d_cs,
                          &c17_d_sn);
          c17_f = c17_c_f;
          c17_b_cs = c17_d_cs;
          c17_b_sn = c17_d_sn;
          if (c17_b_k > c17_b_q) {
            c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)(c17_km1 - 1)), 1, 23, 1, 0) - 1] = c17_f;
          }

          c17_qc_a = c17_b_cs;
          c17_lc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_s_y = c17_qc_a * c17_lc_b;
          c17_rc_a = c17_b_sn;
          c17_mc_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_t_y = c17_rc_a * c17_mc_b;
          c17_f = c17_s_y + c17_t_y;
          c17_sc_a = c17_b_cs;
          c17_nc_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_u_y = c17_sc_a * c17_nc_b;
          c17_tc_a = c17_b_sn;
          c17_oc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_v_y = c17_tc_a * c17_oc_b;
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 23, 1, 0) - 1] = c17_u_y - c17_v_y;
          c17_uc_a = c17_b_sn;
          c17_pc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 23, 1, 0) - 1];
          c17_g = c17_uc_a * c17_pc_b;
          c17_vc_a = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 23, 1, 0) - 1];
          c17_qc_b = c17_b_cs;
          c17_w_y = c17_vc_a * c17_qc_b;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_kp1), 1, 23, 1, 0) - 1] = c17_w_y;
          c17_wc_a = c17_b_k;
          c17_kb_c = c17_wc_a;
          c17_rc_b = c17_kb_c - 1;
          c17_lb_c = 23 * c17_rc_b;
          c17_sc_b = c17_lb_c;
          c17_colk = c17_sc_b;
          c17_tc_b = c17_b_k;
          c17_mb_c = 23 * c17_tc_b;
          c17_uc_b = c17_mb_c;
          c17_colkp1 = c17_uc_b;
          c17_f_eml_xrot(chartInstance, c17_Vf, c17_colk + 1, c17_colkp1 + 1,
                         c17_b_cs, c17_b_sn);
          c17_d_f = c17_f;
          c17_unusedU2 = c17_g;
          c17_b_eml_xrotg(chartInstance, &c17_d_f, &c17_unusedU2, &c17_e_cs,
                          &c17_e_sn);
          c17_f = c17_d_f;
          c17_b_cs = c17_e_cs;
          c17_b_sn = c17_e_sn;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_k), 1, 23, 1, 0) - 1] = c17_f;
          c17_xc_a = c17_b_cs;
          c17_vc_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_x_y = c17_xc_a * c17_vc_b;
          c17_yc_a = c17_b_sn;
          c17_wc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 23, 1, 0) - 1];
          c17_y_y = c17_yc_a * c17_wc_b;
          c17_f = c17_x_y + c17_y_y;
          c17_ad_a = -c17_b_sn;
          c17_xc_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
          c17_ab_y = c17_ad_a * c17_xc_b;
          c17_bd_a = c17_b_cs;
          c17_yc_b = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 23, 1, 0) - 1];
          c17_bb_y = c17_bd_a * c17_yc_b;
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_kp1), 1, 23, 1, 0) - 1] = c17_ab_y + c17_bb_y;
          c17_cd_a = c17_b_sn;
          c17_ad_b = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 23, 1, 0) - 1];
          c17_g = c17_cd_a * c17_ad_b;
          c17_dd_a = c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_kp1), 1, 23, 1, 0) - 1];
          c17_bd_b = c17_b_cs;
          c17_cb_y = c17_dd_a * c17_bd_b;
          c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_kp1), 1, 23, 1, 0) - 1] = c17_cb_y;
          if (c17_b_k < 23) {
            c17_ed_a = c17_b_k;
            c17_nb_c = c17_ed_a;
            c17_cd_b = c17_nb_c - 1;
            c17_ob_c = 23 * c17_cd_b;
            c17_dd_b = c17_ob_c;
            c17_colk = c17_dd_b;
            c17_ed_b = c17_b_k;
            c17_pb_c = 23 * c17_ed_b;
            c17_fd_b = c17_pb_c;
            c17_colkp1 = c17_fd_b;
            c17_f_eml_xrot(chartInstance, c17_U, c17_colk + 1, c17_colkp1 + 1,
                           c17_b_cs, c17_b_sn);
          }
        }

        c17_fd_a = c17_m;
        c17_qb_c = c17_fd_a;
        c17_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c17_qb_c - 1)), 1, 23, 1, 0) - 1] = c17_f;
        c17_iter++;
        break;

       default:
        if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 23, 1, 0) - 1] < 0.0) {
          c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_q), 1, 23, 1, 0) - 1] =
            -c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", (real_T)c17_b_q), 1, 23, 1, 0) - 1];
          c17_gd_a = c17_b_q;
          c17_rb_c = c17_gd_a;
          c17_gd_b = c17_rb_c - 1;
          c17_sb_c = 23 * c17_gd_b;
          c17_hd_b = c17_sb_c;
          c17_colq = c17_hd_b;
          c17_o_eml_scalar_eg(chartInstance);
          c17_d17 = -1.0;
          c17_n_eml_xscal(chartInstance, c17_d17, c17_Vf, c17_colq + 1);
        }

        c17_hd_a = c17_b_q + 1;
        c17_qp1 = c17_hd_a;
        exitg3 = FALSE;
        while ((exitg3 == FALSE) && (c17_b_q < 23)) {
          if (c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c17_b_q), 1, 23, 1, 0) - 1] <
              c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c17_qp1), 1, 23, 1, 0) - 1]) {
            c17_rt = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c17_b_q), 1, 23, 1, 0) - 1];
            c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_q), 1, 23, 1, 0) - 1] =
              c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c17_qp1), 1, 23, 1, 0) - 1];
            c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_qp1), 1, 23, 1, 0) - 1] = c17_rt;
            if (c17_b_q < 23) {
              c17_jd_a = c17_b_q;
              c17_tb_c = c17_jd_a;
              c17_id_b = c17_tb_c - 1;
              c17_ub_c = 23 * c17_id_b;
              c17_jd_b = c17_ub_c;
              c17_colq = c17_jd_b;
              c17_kd_b = c17_b_q;
              c17_vb_c = 23 * c17_kd_b;
              c17_ld_b = c17_vb_c;
              c17_colqp1 = c17_ld_b;
              c17_h_eml_xswap(chartInstance, c17_Vf, c17_colq + 1, c17_colqp1 +
                              1);
            }

            if (c17_b_q < 23) {
              c17_kd_a = c17_b_q;
              c17_wb_c = c17_kd_a;
              c17_md_b = c17_wb_c - 1;
              c17_xb_c = 23 * c17_md_b;
              c17_nd_b = c17_xb_c;
              c17_colq = c17_nd_b;
              c17_od_b = c17_b_q;
              c17_yb_c = 23 * c17_od_b;
              c17_pd_b = c17_yb_c;
              c17_colqp1 = c17_pd_b;
              c17_h_eml_xswap(chartInstance, c17_U, c17_colq + 1, c17_colqp1 + 1);
            }

            c17_b_q = c17_qp1;
            c17_ld_a = c17_b_q + 1;
            c17_qp1 = c17_ld_a;
          } else {
            exitg3 = TRUE;
          }
        }

        c17_iter = 0.0;
        c17_id_a = c17_m - 1;
        c17_m = c17_id_a;
        break;
      }
    }
  }

  for (c17_e_k = 1; c17_e_k < 24; c17_e_k++) {
    c17_b_k = c17_e_k;
    c17_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 23, 1, 0) - 1] = c17_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
  }

  for (c17_j = 1; c17_j < 24; c17_j++) {
    c17_b_j = c17_j;
    for (c17_i = 1; c17_i < 24; c17_i++) {
      c17_b_i = c17_i;
      c17_V[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c17_b_i), 1, 23, 1, 0) + 23 *
             (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c17_b_j), 1, 23, 2, 0) - 1)) - 1] = c17_Vf
        [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_i), 1, 23, 1, 0) + 23 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
            "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 23, 2, 0) -
           1)) - 1];
    }
  }
}

static real_T c17_c_eml_xnrm2(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[529], int32_T c17_ix0)
{
  real_T c17_y;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_scale;
  int32_T c17_kstart;
  int32_T c17_a;
  int32_T c17_c;
  int32_T c17_b_a;
  int32_T c17_b_c;
  int32_T c17_c_a;
  int32_T c17_b;
  int32_T c17_kend;
  int32_T c17_b_kstart;
  int32_T c17_b_kend;
  int32_T c17_d_a;
  int32_T c17_b_b;
  int32_T c17_e_a;
  int32_T c17_c_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_d_x;
  real_T c17_e_x;
  real_T c17_absxk;
  real_T c17_t;
  c17_b_n = c17_n;
  c17_b_ix0 = c17_ix0;
  c17_c_n = c17_b_n;
  c17_c_ix0 = c17_b_ix0;
  c17_y = 0.0;
  if (c17_c_n < 1) {
  } else if (c17_c_n == 1) {
    c17_b_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_c_ix0), 1, 529, 1, 0) - 1];
    c17_c_x = c17_b_x;
    c17_y = muDoubleScalarAbs(c17_c_x);
  } else {
    c17_realmin(chartInstance);
    c17_scale = 2.2250738585072014E-308;
    c17_kstart = c17_c_ix0;
    c17_a = c17_c_n;
    c17_c = c17_a;
    c17_b_a = c17_c - 1;
    c17_b_c = c17_b_a;
    c17_c_a = c17_kstart;
    c17_b = c17_b_c;
    c17_kend = c17_c_a + c17_b;
    c17_b_kstart = c17_kstart;
    c17_b_kend = c17_kend;
    c17_d_a = c17_b_kstart;
    c17_b_b = c17_b_kend;
    c17_e_a = c17_d_a;
    c17_c_b = c17_b_b;
    if (c17_e_a > c17_c_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_c_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = c17_b_kstart; c17_k <= c17_b_kend; c17_k++) {
      c17_b_k = c17_k;
      c17_d_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 529, 1, 0) - 1];
      c17_e_x = c17_d_x;
      c17_absxk = muDoubleScalarAbs(c17_e_x);
      if (c17_absxk > c17_scale) {
        c17_t = c17_scale / c17_absxk;
        c17_y = 1.0 + c17_y * c17_t * c17_t;
        c17_scale = c17_absxk;
      } else {
        c17_t = c17_absxk / c17_scale;
        c17_y += c17_t * c17_t;
      }
    }

    c17_y = c17_scale * muDoubleScalarSqrt(c17_y);
  }

  return c17_y;
}

static void c17_e_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[529], int32_T
  c17_ix0, real_T c17_b_x[529])
{
  int32_T c17_i1047;
  for (c17_i1047 = 0; c17_i1047 < 529; c17_i1047++) {
    c17_b_x[c17_i1047] = c17_x[c17_i1047];
  }

  c17_l_eml_xscal(chartInstance, c17_n, c17_a, c17_b_x, c17_ix0);
}

static real_T c17_c_eml_xdotc(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[529], int32_T c17_ix0, real_T
  c17_y[529], int32_T c17_iy0)
{
  real_T c17_d;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_n;
  int32_T c17_d_ix0;
  int32_T c17_d_iy0;
  int32_T c17_e_n;
  int32_T c17_e_ix0;
  int32_T c17_e_iy0;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_f_n;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_a;
  int32_T c17_b_a;
  c17_b_n = c17_n;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_d_n = c17_c_n;
  c17_d_ix0 = c17_c_ix0;
  c17_d_iy0 = c17_c_iy0;
  c17_e_n = c17_d_n;
  c17_e_ix0 = c17_d_ix0;
  c17_e_iy0 = c17_d_iy0;
  c17_q_eml_scalar_eg(chartInstance);
  c17_d = 0.0;
  if (c17_e_n < 1) {
  } else {
    c17_ix = c17_e_ix0;
    c17_iy = c17_e_iy0;
    c17_f_n = c17_e_n;
    c17_b = c17_f_n;
    c17_b_b = c17_b;
    if (1 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 1; c17_k <= c17_f_n; c17_k++) {
      c17_d += c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c17_ix), 1, 529, 1, 0) - 1] *
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_iy), 1, 529, 1, 0) - 1];
      c17_a = c17_ix + 1;
      c17_ix = c17_a;
      c17_b_a = c17_iy + 1;
      c17_iy = c17_b_a;
    }
  }

  return c17_d;
}

static void c17_e_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[529],
  int32_T c17_iy0, real_T c17_b_y[529])
{
  int32_T c17_i1048;
  for (c17_i1048 = 0; c17_i1048 < 529; c17_i1048++) {
    c17_b_y[c17_i1048] = c17_y[c17_i1048];
  }

  c17_l_eml_xaxpy(chartInstance, c17_n, c17_a, c17_ix0, c17_b_y, c17_iy0);
}

static real_T c17_d_eml_xnrm2(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_x[23], int32_T c17_ix0)
{
  real_T c17_y;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_scale;
  int32_T c17_kstart;
  int32_T c17_a;
  int32_T c17_c;
  int32_T c17_b_a;
  int32_T c17_b_c;
  int32_T c17_c_a;
  int32_T c17_b;
  int32_T c17_kend;
  int32_T c17_b_kstart;
  int32_T c17_b_kend;
  int32_T c17_d_a;
  int32_T c17_b_b;
  int32_T c17_e_a;
  int32_T c17_c_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  real_T c17_d_x;
  real_T c17_e_x;
  real_T c17_absxk;
  real_T c17_t;
  c17_b_n = c17_n;
  c17_b_ix0 = c17_ix0;
  c17_c_n = c17_b_n;
  c17_c_ix0 = c17_b_ix0;
  c17_y = 0.0;
  if (c17_c_n < 1) {
  } else if (c17_c_n == 1) {
    c17_b_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_c_ix0), 1, 23, 1, 0) - 1];
    c17_c_x = c17_b_x;
    c17_y = muDoubleScalarAbs(c17_c_x);
  } else {
    c17_realmin(chartInstance);
    c17_scale = 2.2250738585072014E-308;
    c17_kstart = c17_c_ix0;
    c17_a = c17_c_n;
    c17_c = c17_a;
    c17_b_a = c17_c - 1;
    c17_b_c = c17_b_a;
    c17_c_a = c17_kstart;
    c17_b = c17_b_c;
    c17_kend = c17_c_a + c17_b;
    c17_b_kstart = c17_kstart;
    c17_b_kend = c17_kend;
    c17_d_a = c17_b_kstart;
    c17_b_b = c17_b_kend;
    c17_e_a = c17_d_a;
    c17_c_b = c17_b_b;
    if (c17_e_a > c17_c_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_c_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = c17_b_kstart; c17_k <= c17_b_kend; c17_k++) {
      c17_b_k = c17_k;
      c17_d_x = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_b_k), 1, 23, 1, 0) - 1];
      c17_e_x = c17_d_x;
      c17_absxk = muDoubleScalarAbs(c17_e_x);
      if (c17_absxk > c17_scale) {
        c17_t = c17_scale / c17_absxk;
        c17_y = 1.0 + c17_y * c17_t * c17_t;
        c17_scale = c17_absxk;
      } else {
        c17_t = c17_absxk / c17_scale;
        c17_y += c17_t * c17_t;
      }
    }

    c17_y = c17_scale * muDoubleScalarSqrt(c17_y);
  }

  return c17_y;
}

static void c17_f_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[23], int32_T c17_ix0,
  real_T c17_b_x[23])
{
  int32_T c17_i1049;
  for (c17_i1049 = 0; c17_i1049 < 23; c17_i1049++) {
    c17_b_x[c17_i1049] = c17_x[c17_i1049];
  }

  c17_m_eml_xscal(chartInstance, c17_n, c17_a, c17_b_x, c17_ix0);
}

static void c17_f_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[529], int32_T
  c17_ix0, real_T c17_y[23], int32_T c17_iy0, real_T c17_b_y[23])
{
  int32_T c17_i1050;
  int32_T c17_i1051;
  real_T c17_b_x[529];
  for (c17_i1050 = 0; c17_i1050 < 23; c17_i1050++) {
    c17_b_y[c17_i1050] = c17_y[c17_i1050];
  }

  for (c17_i1051 = 0; c17_i1051 < 529; c17_i1051++) {
    c17_b_x[c17_i1051] = c17_x[c17_i1051];
  }

  c17_m_eml_xaxpy(chartInstance, c17_n, c17_a, c17_b_x, c17_ix0, c17_b_y,
                  c17_iy0);
}

static void c17_g_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[23], int32_T c17_ix0,
  real_T c17_y[529], int32_T c17_iy0, real_T c17_b_y[529])
{
  int32_T c17_i1052;
  int32_T c17_i1053;
  real_T c17_b_x[23];
  for (c17_i1052 = 0; c17_i1052 < 529; c17_i1052++) {
    c17_b_y[c17_i1052] = c17_y[c17_i1052];
  }

  for (c17_i1053 = 0; c17_i1053 < 23; c17_i1053++) {
    c17_b_x[c17_i1053] = c17_x[c17_i1053];
  }

  c17_n_eml_xaxpy(chartInstance, c17_n, c17_a, c17_b_x, c17_ix0, c17_b_y,
                  c17_iy0);
}

static void c17_g_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[529], int32_T c17_ix0, real_T
  c17_b_x[529])
{
  int32_T c17_i1054;
  for (c17_i1054 = 0; c17_i1054 < 529; c17_i1054++) {
    c17_b_x[c17_i1054] = c17_x[c17_i1054];
  }

  c17_n_eml_xscal(chartInstance, c17_a, c17_b_x, c17_ix0);
}

static void c17_c_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[529], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s, real_T c17_b_x[529])
{
  int32_T c17_i1055;
  for (c17_i1055 = 0; c17_i1055 < 529; c17_i1055++) {
    c17_b_x[c17_i1055] = c17_x[c17_i1055];
  }

  c17_f_eml_xrot(chartInstance, c17_b_x, c17_ix0, c17_iy0, c17_c, c17_s);
}

static void c17_f_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_d_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[529], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_b_x[529])
{
  int32_T c17_i1056;
  for (c17_i1056 = 0; c17_i1056 < 529; c17_i1056++) {
    c17_b_x[c17_i1056] = c17_x[c17_i1056];
  }

  c17_h_eml_xswap(chartInstance, c17_b_x, c17_ix0, c17_iy0);
}

static void c17_g_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_j_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_k, real_T c17_A[529], real_T c17_B[529], real_T
  c17_C[529], real_T c17_b_C[529])
{
  int32_T c17_i1057;
  int32_T c17_i1058;
  real_T c17_b_A[529];
  int32_T c17_i1059;
  real_T c17_b_B[529];
  for (c17_i1057 = 0; c17_i1057 < 529; c17_i1057++) {
    c17_b_C[c17_i1057] = c17_C[c17_i1057];
  }

  for (c17_i1058 = 0; c17_i1058 < 529; c17_i1058++) {
    c17_b_A[c17_i1058] = c17_A[c17_i1058];
  }

  for (c17_i1059 = 0; c17_i1059 < 529; c17_i1059++) {
    c17_b_B[c17_i1059] = c17_B[c17_i1059];
  }

  c17_y_eml_xgemm(chartInstance, c17_k, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_h_below_threshold(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_b_diag(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_v[23], real_T c17_d[529])
{
  int32_T c17_i1060;
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_a;
  int32_T c17_c;
  for (c17_i1060 = 0; c17_i1060 < 529; c17_i1060++) {
    c17_d[c17_i1060] = 0.0;
  }

  for (c17_j = 1; c17_j < 24; c17_j++) {
    c17_b_j = c17_j;
    c17_a = c17_b_j;
    c17_c = c17_a;
    c17_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c17_b_j), 1, 23, 1, 0) + 23 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
             "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_c), 1, 23, 2, 0) -
            1)) - 1] = c17_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c17_b_j), 1, 23, 1, 0) - 1];
  }
}

static void c17_blkdiag(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c17_varargin_1[9], real_T c17_varargin_2[9], real_T c17_y[36])
{
  int32_T c17_i1061;
  int32_T c17_i1062;
  int32_T c17_i1063;
  int32_T c17_i1064;
  int32_T c17_i1065;
  int32_T c17_i1066;
  int32_T c17_i1067;
  int32_T c17_i1068;
  int32_T c17_i1069;
  for (c17_i1061 = 0; c17_i1061 < 36; c17_i1061++) {
    c17_y[c17_i1061] = 0.0;
  }

  c17_i1062 = 0;
  c17_i1063 = 0;
  for (c17_i1064 = 0; c17_i1064 < 3; c17_i1064++) {
    for (c17_i1065 = 0; c17_i1065 < 3; c17_i1065++) {
      c17_y[c17_i1065 + c17_i1062] = c17_varargin_1[c17_i1065 + c17_i1063];
    }

    c17_i1062 += 6;
    c17_i1063 += 3;
  }

  c17_i1066 = 0;
  c17_i1067 = 0;
  for (c17_i1068 = 0; c17_i1068 < 3; c17_i1068++) {
    for (c17_i1069 = 0; c17_i1069 < 3; c17_i1069++) {
      c17_y[(c17_i1069 + c17_i1066) + 21] = c17_varargin_2[c17_i1069 + c17_i1067];
    }

    c17_i1066 += 6;
    c17_i1067 += 3;
  }
}

static void c17_r_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_k_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[114], real_T c17_B[36], real_T c17_C[114], real_T
  c17_b_C[114])
{
  int32_T c17_i1070;
  int32_T c17_i1071;
  real_T c17_b_A[114];
  int32_T c17_i1072;
  real_T c17_b_B[36];
  for (c17_i1070 = 0; c17_i1070 < 114; c17_i1070++) {
    c17_b_C[c17_i1070] = c17_C[c17_i1070];
  }

  for (c17_i1071 = 0; c17_i1071 < 114; c17_i1071++) {
    c17_b_A[c17_i1071] = c17_A[c17_i1071];
  }

  for (c17_i1072 = 0; c17_i1072 < 36; c17_i1072++) {
    c17_b_B[c17_i1072] = c17_B[c17_i1072];
  }

  c17_ab_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_b_blkdiag(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_varargin_1[114], real_T c17_varargin_2[114], real_T
  c17_y[456])
{
  int32_T c17_i1073;
  int32_T c17_i1074;
  int32_T c17_i1075;
  int32_T c17_i1076;
  int32_T c17_i1077;
  int32_T c17_i1078;
  int32_T c17_i1079;
  int32_T c17_i1080;
  int32_T c17_i1081;
  for (c17_i1073 = 0; c17_i1073 < 456; c17_i1073++) {
    c17_y[c17_i1073] = 0.0;
  }

  c17_i1074 = 0;
  c17_i1075 = 0;
  for (c17_i1076 = 0; c17_i1076 < 6; c17_i1076++) {
    for (c17_i1077 = 0; c17_i1077 < 19; c17_i1077++) {
      c17_y[c17_i1077 + c17_i1074] = c17_varargin_1[c17_i1077 + c17_i1075];
    }

    c17_i1074 += 38;
    c17_i1075 += 19;
  }

  c17_i1078 = 0;
  c17_i1079 = 0;
  for (c17_i1080 = 0; c17_i1080 < 6; c17_i1080++) {
    for (c17_i1081 = 0; c17_i1081 < 19; c17_i1081++) {
      c17_y[(c17_i1081 + c17_i1078) + 247] = c17_varargin_2[c17_i1081 +
        c17_i1079];
    }

    c17_i1078 += 38;
    c17_i1079 += 19;
  }
}

static void c17_s_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_t_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_u_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_v_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_l_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[23], real_T c17_C[23], real_T
  c17_b_C[23])
{
  int32_T c17_i1082;
  int32_T c17_i1083;
  real_T c17_b_A[529];
  int32_T c17_i1084;
  real_T c17_b_B[23];
  for (c17_i1082 = 0; c17_i1082 < 23; c17_i1082++) {
    c17_b_C[c17_i1082] = c17_C[c17_i1082];
  }

  for (c17_i1083 = 0; c17_i1083 < 529; c17_i1083++) {
    c17_b_A[c17_i1083] = c17_A[c17_i1083];
  }

  for (c17_i1084 = 0; c17_i1084 < 23; c17_i1084++) {
    c17_b_B[c17_i1084] = c17_B[c17_i1084];
  }

  c17_bb_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_w_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_m_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[144], real_T c17_C[276],
  real_T c17_b_C[276])
{
  int32_T c17_i1085;
  int32_T c17_i1086;
  real_T c17_b_A[276];
  int32_T c17_i1087;
  real_T c17_b_B[144];
  for (c17_i1085 = 0; c17_i1085 < 276; c17_i1085++) {
    c17_b_C[c17_i1085] = c17_C[c17_i1085];
  }

  for (c17_i1086 = 0; c17_i1086 < 276; c17_i1086++) {
    c17_b_A[c17_i1086] = c17_A[c17_i1086];
  }

  for (c17_i1087 = 0; c17_i1087 < 144; c17_i1087++) {
    c17_b_B[c17_i1087] = c17_B[c17_i1087];
  }

  c17_cb_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_x_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_n_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[276], real_T c17_C[276],
  real_T c17_b_C[276])
{
  int32_T c17_i1088;
  int32_T c17_i1089;
  real_T c17_b_A[529];
  int32_T c17_i1090;
  real_T c17_b_B[276];
  for (c17_i1088 = 0; c17_i1088 < 276; c17_i1088++) {
    c17_b_C[c17_i1088] = c17_C[c17_i1088];
  }

  for (c17_i1089 = 0; c17_i1089 < 529; c17_i1089++) {
    c17_b_A[c17_i1089] = c17_A[c17_i1089];
  }

  for (c17_i1090 = 0; c17_i1090 < 276; c17_i1090++) {
    c17_b_B[c17_i1090] = c17_B[c17_i1090];
  }

  c17_db_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_y_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_ab_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_o_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[456], real_T c17_B[144], real_T c17_C[456],
  real_T c17_b_C[456])
{
  int32_T c17_i1091;
  int32_T c17_i1092;
  real_T c17_b_A[456];
  int32_T c17_i1093;
  real_T c17_b_B[144];
  for (c17_i1091 = 0; c17_i1091 < 456; c17_i1091++) {
    c17_b_C[c17_i1091] = c17_C[c17_i1091];
  }

  for (c17_i1092 = 0; c17_i1092 < 456; c17_i1092++) {
    c17_b_A[c17_i1092] = c17_A[c17_i1092];
  }

  for (c17_i1093 = 0; c17_i1093 < 144; c17_i1093++) {
    c17_b_B[c17_i1093] = c17_B[c17_i1093];
  }

  c17_eb_eml_xgemm(chartInstance, c17_b_A, c17_b_B, c17_b_C);
}

static void c17_bb_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_cb_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c17_db_eml_scalar_eg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static const mxArray *c17_fb_sf_marshallOut(void *chartInstanceVoid, void
  *c17_inData)
{
  const mxArray *c17_mxArrayOutData = NULL;
  int32_T c17_u;
  const mxArray *c17_y = NULL;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_mxArrayOutData = NULL;
  c17_u = *(int32_T *)c17_inData;
  c17_y = NULL;
  sf_mex_assign(&c17_y, sf_mex_create("y", &c17_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c17_mxArrayOutData, c17_y, FALSE);
  return c17_mxArrayOutData;
}

static int32_T c17_rb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId)
{
  int32_T c17_y;
  int32_T c17_i1094;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), &c17_i1094, 1, 6, 0U, 0, 0U, 0);
  c17_y = c17_i1094;
  sf_mex_destroy(&c17_u);
  return c17_y;
}

static void c17_db_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c17_mxArrayInData, const char_T *c17_varName, void *c17_outData)
{
  const mxArray *c17_b_sfEvent;
  const char_T *c17_identifier;
  emlrtMsgIdentifier c17_thisId;
  int32_T c17_y;
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c17_b_sfEvent = sf_mex_dup(c17_mxArrayInData);
  c17_identifier = c17_varName;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_y = c17_rb_emlrt_marshallIn(chartInstance, sf_mex_dup(c17_b_sfEvent),
    &c17_thisId);
  sf_mex_destroy(&c17_b_sfEvent);
  *(int32_T *)c17_outData = c17_y;
  sf_mex_destroy(&c17_mxArrayInData);
}

static uint8_T c17_sb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c17_b_is_active_c17_torqueBalancing2012b, const
  char_T *c17_identifier)
{
  uint8_T c17_y;
  emlrtMsgIdentifier c17_thisId;
  c17_thisId.fIdentifier = c17_identifier;
  c17_thisId.fParent = NULL;
  c17_y = c17_tb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c17_b_is_active_c17_torqueBalancing2012b), &c17_thisId);
  sf_mex_destroy(&c17_b_is_active_c17_torqueBalancing2012b);
  return c17_y;
}

static uint8_T c17_tb_emlrt_marshallIn(SFc17_torqueBalancing2012bInstanceStruct *
  chartInstance, const mxArray *c17_u, const emlrtMsgIdentifier *c17_parentId)
{
  uint8_T c17_y;
  uint8_T c17_u0;
  sf_mex_import(c17_parentId, sf_mex_dup(c17_u), &c17_u0, 1, 3, 0U, 0, 0U, 0);
  c17_y = c17_u0;
  sf_mex_destroy(&c17_u);
  return c17_y;
}

static void c17_h_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[72], int32_T c17_ix0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_d_ix0;
  int32_T c17_d_a;
  int32_T c17_c;
  int32_T c17_b;
  int32_T c17_b_c;
  int32_T c17_e_a;
  int32_T c17_b_b;
  int32_T c17_i1095;
  int32_T c17_f_a;
  int32_T c17_c_b;
  int32_T c17_g_a;
  int32_T c17_d_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_d_ix0 = c17_c_ix0;
  c17_d_a = c17_c_n;
  c17_c = c17_d_a;
  c17_b = c17_c - 1;
  c17_b_c = c17_b;
  c17_e_a = c17_c_ix0;
  c17_b_b = c17_b_c;
  c17_i1095 = c17_e_a + c17_b_b;
  c17_f_a = c17_d_ix0;
  c17_c_b = c17_i1095;
  c17_g_a = c17_f_a;
  c17_d_b = c17_c_b;
  if (c17_g_a > c17_d_b) {
    c17_overflow = FALSE;
  } else {
    c17_overflow = (c17_d_b > 2147483646);
  }

  if (c17_overflow) {
    c17_check_forloop_overflow_error(chartInstance, c17_overflow);
  }

  for (c17_k = c17_d_ix0; c17_k <= c17_i1095; c17_k++) {
    c17_b_k = c17_k;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 72, 1, 0) - 1] = c17_c_a *
      c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 72, 1, 0) - 1];
  }
}

static void c17_h_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[72],
  int32_T c17_iy0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_a;
  int32_T c17_ix;
  int32_T c17_e_a;
  int32_T c17_iy;
  int32_T c17_f_a;
  int32_T c17_i1096;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_g_a;
  int32_T c17_c;
  int32_T c17_h_a;
  int32_T c17_b_c;
  int32_T c17_i_a;
  int32_T c17_c_c;
  int32_T c17_j_a;
  int32_T c17_k_a;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  if (c17_c_n < 1) {
  } else if (c17_c_a == 0.0) {
  } else {
    c17_d_a = c17_c_ix0 - 1;
    c17_ix = c17_d_a;
    c17_e_a = c17_c_iy0 - 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_c_n - 1;
    c17_i1096 = c17_f_a;
    c17_b = c17_i1096;
    c17_b_b = c17_b;
    if (0 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 0; c17_k <= c17_i1096; c17_k++) {
      c17_g_a = c17_iy;
      c17_c = c17_g_a;
      c17_h_a = c17_iy;
      c17_b_c = c17_h_a;
      c17_i_a = c17_ix;
      c17_c_c = c17_i_a;
      c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c + 1)), 1, 72, 1, 0) - 1] =
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_b_c + 1)), 1, 72, 1, 0) - 1] + c17_c_a *
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c_c + 1)), 1, 72, 1, 0) - 1];
      c17_j_a = c17_ix + 1;
      c17_ix = c17_j_a;
      c17_k_a = c17_iy + 1;
      c17_iy = c17_k_a;
    }
  }
}

static void c17_i_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[6], int32_T c17_ix0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_d_ix0;
  int32_T c17_d_a;
  int32_T c17_c;
  int32_T c17_b;
  int32_T c17_b_c;
  int32_T c17_e_a;
  int32_T c17_b_b;
  int32_T c17_i1097;
  int32_T c17_f_a;
  int32_T c17_c_b;
  int32_T c17_g_a;
  int32_T c17_d_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_d_ix0 = c17_c_ix0;
  c17_d_a = c17_c_n;
  c17_c = c17_d_a;
  c17_b = c17_c - 1;
  c17_b_c = c17_b;
  c17_e_a = c17_c_ix0;
  c17_b_b = c17_b_c;
  c17_i1097 = c17_e_a + c17_b_b;
  c17_f_a = c17_d_ix0;
  c17_c_b = c17_i1097;
  c17_g_a = c17_f_a;
  c17_d_b = c17_c_b;
  if (c17_g_a > c17_d_b) {
    c17_overflow = FALSE;
  } else {
    c17_overflow = (c17_d_b > 2147483646);
  }

  if (c17_overflow) {
    c17_check_forloop_overflow_error(chartInstance, c17_overflow);
  }

  for (c17_k = c17_d_ix0; c17_k <= c17_i1097; c17_k++) {
    c17_b_k = c17_k;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 6, 1, 0) - 1] = c17_c_a *
      c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 6, 1, 0) - 1];
  }
}

static void c17_i_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[72], int32_T c17_ix0,
  real_T c17_y[12], int32_T c17_iy0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_a;
  int32_T c17_ix;
  int32_T c17_e_a;
  int32_T c17_iy;
  int32_T c17_f_a;
  int32_T c17_i1098;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_g_a;
  int32_T c17_c;
  int32_T c17_h_a;
  int32_T c17_b_c;
  int32_T c17_i_a;
  int32_T c17_c_c;
  int32_T c17_j_a;
  int32_T c17_k_a;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  if (c17_c_n < 1) {
  } else if (c17_c_a == 0.0) {
  } else {
    c17_d_a = c17_c_ix0 - 1;
    c17_ix = c17_d_a;
    c17_e_a = c17_c_iy0 - 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_c_n - 1;
    c17_i1098 = c17_f_a;
    c17_b = c17_i1098;
    c17_b_b = c17_b;
    if (0 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 0; c17_k <= c17_i1098; c17_k++) {
      c17_g_a = c17_iy;
      c17_c = c17_g_a;
      c17_h_a = c17_iy;
      c17_b_c = c17_h_a;
      c17_i_a = c17_ix;
      c17_c_c = c17_i_a;
      c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c + 1)), 1, 12, 1, 0) - 1] =
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_b_c + 1)), 1, 12, 1, 0) - 1] + c17_c_a *
        c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c_c + 1)), 1, 72, 1, 0) - 1];
      c17_j_a = c17_ix + 1;
      c17_ix = c17_j_a;
      c17_k_a = c17_iy + 1;
      c17_iy = c17_k_a;
    }
  }
}

static void c17_j_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[12], int32_T c17_ix0,
  real_T c17_y[72], int32_T c17_iy0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_a;
  int32_T c17_ix;
  int32_T c17_e_a;
  int32_T c17_iy;
  int32_T c17_f_a;
  int32_T c17_i1099;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_g_a;
  int32_T c17_c;
  int32_T c17_h_a;
  int32_T c17_b_c;
  int32_T c17_i_a;
  int32_T c17_c_c;
  int32_T c17_j_a;
  int32_T c17_k_a;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  if (c17_c_n < 1) {
  } else if (c17_c_a == 0.0) {
  } else {
    c17_d_a = c17_c_ix0 - 1;
    c17_ix = c17_d_a;
    c17_e_a = c17_c_iy0 - 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_c_n - 1;
    c17_i1099 = c17_f_a;
    c17_b = c17_i1099;
    c17_b_b = c17_b;
    if (0 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 0; c17_k <= c17_i1099; c17_k++) {
      c17_g_a = c17_iy;
      c17_c = c17_g_a;
      c17_h_a = c17_iy;
      c17_b_c = c17_h_a;
      c17_i_a = c17_ix;
      c17_c_c = c17_i_a;
      c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c + 1)), 1, 72, 1, 0) - 1] =
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_b_c + 1)), 1, 72, 1, 0) - 1] + c17_c_a *
        c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c_c + 1)), 1, 12, 1, 0) - 1];
      c17_j_a = c17_ix + 1;
      c17_ix = c17_j_a;
      c17_k_a = c17_iy + 1;
      c17_iy = c17_k_a;
    }
  }
}

static void c17_k_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[36],
  int32_T c17_iy0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_a;
  int32_T c17_ix;
  int32_T c17_e_a;
  int32_T c17_iy;
  int32_T c17_f_a;
  int32_T c17_i1100;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_g_a;
  int32_T c17_c;
  int32_T c17_h_a;
  int32_T c17_b_c;
  int32_T c17_i_a;
  int32_T c17_c_c;
  int32_T c17_j_a;
  int32_T c17_k_a;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  if (c17_c_n < 1) {
  } else if (c17_c_a == 0.0) {
  } else {
    c17_d_a = c17_c_ix0 - 1;
    c17_ix = c17_d_a;
    c17_e_a = c17_c_iy0 - 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_c_n - 1;
    c17_i1100 = c17_f_a;
    c17_b = c17_i1100;
    c17_b_b = c17_b;
    if (0 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 0; c17_k <= c17_i1100; c17_k++) {
      c17_g_a = c17_iy;
      c17_c = c17_g_a;
      c17_h_a = c17_iy;
      c17_b_c = c17_h_a;
      c17_i_a = c17_ix;
      c17_c_c = c17_i_a;
      c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c + 1)), 1, 36, 1, 0) - 1] =
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_b_c + 1)), 1, 36, 1, 0) - 1] + c17_c_a *
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c_c + 1)), 1, 36, 1, 0) - 1];
      c17_j_a = c17_ix + 1;
      c17_ix = c17_j_a;
      c17_k_a = c17_iy + 1;
      c17_iy = c17_k_a;
    }
  }
}

static void c17_j_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[72], int32_T c17_ix0)
{
  real_T c17_b_a;
  int32_T c17_b_ix0;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_d_ix0;
  int32_T c17_d_a;
  int32_T c17_i1101;
  int32_T c17_e_a;
  int32_T c17_b;
  int32_T c17_f_a;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_d_ix0 = c17_c_ix0;
  c17_d_a = c17_c_ix0 + 11;
  c17_i1101 = c17_d_a;
  c17_e_a = c17_d_ix0;
  c17_b = c17_i1101;
  c17_f_a = c17_e_a;
  c17_b_b = c17_b;
  if (c17_f_a > c17_b_b) {
    c17_overflow = FALSE;
  } else {
    c17_overflow = (c17_b_b > 2147483646);
  }

  if (c17_overflow) {
    c17_check_forloop_overflow_error(chartInstance, c17_overflow);
  }

  for (c17_k = c17_d_ix0; c17_k <= c17_i1101; c17_k++) {
    c17_b_k = c17_k;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 72, 1, 0) - 1] = c17_c_a *
      c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 72, 1, 0) - 1];
  }
}

static void c17_k_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[36], int32_T c17_ix0)
{
  real_T c17_b_a;
  int32_T c17_b_ix0;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_d_ix0;
  int32_T c17_d_a;
  int32_T c17_i1102;
  int32_T c17_e_a;
  int32_T c17_b;
  int32_T c17_f_a;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_d_ix0 = c17_c_ix0;
  c17_d_a = c17_c_ix0 + 5;
  c17_i1102 = c17_d_a;
  c17_e_a = c17_d_ix0;
  c17_b = c17_i1102;
  c17_f_a = c17_e_a;
  c17_b_b = c17_b;
  if (c17_f_a > c17_b_b) {
    c17_overflow = FALSE;
  } else {
    c17_overflow = (c17_b_b > 2147483646);
  }

  if (c17_overflow) {
    c17_check_forloop_overflow_error(chartInstance, c17_overflow);
  }

  for (c17_k = c17_d_ix0; c17_k <= c17_i1102; c17_k++) {
    c17_b_k = c17_k;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 36, 1, 0) - 1] = c17_c_a *
      c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 36, 1, 0) - 1];
  }
}

static void c17_b_sqrt(SFc17_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T *c17_x)
{
  if (*c17_x < 0.0) {
    c17_c_eml_error(chartInstance);
  }

  *c17_x = muDoubleScalarSqrt(*c17_x);
}

static void c17_b_eml_xrotg(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T *c17_a, real_T *c17_b, real_T *c17_c, real_T *c17_s)
{
  real_T c17_b_a;
  real_T c17_b_b;
  real_T c17_c_b;
  real_T c17_c_a;
  real_T c17_d_a;
  real_T c17_d_b;
  real_T c17_e_b;
  real_T c17_e_a;
  real_T c17_b_c;
  real_T c17_b_s;
  double * c17_a_t;
  double * c17_b_t;
  double * c17_c_t;
  double * c17_s_t;
  real_T c17_c_c;
  real_T c17_c_s;
  c17_b_a = *c17_a;
  c17_b_b = *c17_b;
  c17_c_b = c17_b_b;
  c17_c_a = c17_b_a;
  c17_d_a = c17_c_a;
  c17_d_b = c17_c_b;
  c17_e_b = c17_d_b;
  c17_e_a = c17_d_a;
  c17_b_c = 0.0;
  c17_b_s = 0.0;
  c17_a_t = (double *)(&c17_e_a);
  c17_b_t = (double *)(&c17_e_b);
  c17_c_t = (double *)(&c17_b_c);
  c17_s_t = (double *)(&c17_b_s);
  drotg(c17_a_t, c17_b_t, c17_c_t, c17_s_t);
  c17_c_a = c17_e_a;
  c17_c_b = c17_e_b;
  c17_c_c = c17_b_c;
  c17_c_s = c17_b_s;
  *c17_a = c17_c_a;
  *c17_b = c17_c_b;
  *c17_c = c17_c_c;
  *c17_s = c17_c_s;
}

static void c17_d_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[36], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s)
{
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  real_T c17_b_c;
  real_T c17_b_s;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  real_T c17_c_c;
  real_T c17_c_s;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_k;
  real_T c17_a;
  real_T c17_b;
  real_T c17_y;
  real_T c17_b_a;
  real_T c17_b_b;
  real_T c17_b_y;
  real_T c17_temp;
  real_T c17_c_a;
  real_T c17_c_b;
  real_T c17_c_y;
  real_T c17_d_a;
  real_T c17_d_b;
  real_T c17_d_y;
  int32_T c17_e_a;
  int32_T c17_f_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_b_c = c17_c;
  c17_b_s = c17_s;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_c_c = c17_b_c;
  c17_c_s = c17_b_s;
  c17_ix = c17_c_ix0;
  c17_iy = c17_c_iy0;
  for (c17_k = 1; c17_k < 7; c17_k++) {
    c17_a = c17_c_c;
    c17_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 36, 1, 0) - 1];
    c17_y = c17_a * c17_b;
    c17_b_a = c17_c_s;
    c17_b_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_iy), 1, 36, 1, 0) - 1];
    c17_b_y = c17_b_a * c17_b_b;
    c17_temp = c17_y + c17_b_y;
    c17_c_a = c17_c_c;
    c17_c_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_iy), 1, 36, 1, 0) - 1];
    c17_c_y = c17_c_a * c17_c_b;
    c17_d_a = c17_c_s;
    c17_d_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_ix), 1, 36, 1, 0) - 1];
    c17_d_y = c17_d_a * c17_d_b;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_iy), 1, 36, 1, 0) - 1] = c17_c_y - c17_d_y;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 36, 1, 0) - 1] = c17_temp;
    c17_e_a = c17_iy + 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_ix + 1;
    c17_ix = c17_f_a;
  }
}

static void c17_e_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[72], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s)
{
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  real_T c17_b_c;
  real_T c17_b_s;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  real_T c17_c_c;
  real_T c17_c_s;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_k;
  real_T c17_a;
  real_T c17_b;
  real_T c17_y;
  real_T c17_b_a;
  real_T c17_b_b;
  real_T c17_b_y;
  real_T c17_temp;
  real_T c17_c_a;
  real_T c17_c_b;
  real_T c17_c_y;
  real_T c17_d_a;
  real_T c17_d_b;
  real_T c17_d_y;
  int32_T c17_e_a;
  int32_T c17_f_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_b_c = c17_c;
  c17_b_s = c17_s;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_c_c = c17_b_c;
  c17_c_s = c17_b_s;
  c17_ix = c17_c_ix0;
  c17_iy = c17_c_iy0;
  for (c17_k = 1; c17_k < 13; c17_k++) {
    c17_a = c17_c_c;
    c17_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 72, 1, 0) - 1];
    c17_y = c17_a * c17_b;
    c17_b_a = c17_c_s;
    c17_b_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_iy), 1, 72, 1, 0) - 1];
    c17_b_y = c17_b_a * c17_b_b;
    c17_temp = c17_y + c17_b_y;
    c17_c_a = c17_c_c;
    c17_c_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_iy), 1, 72, 1, 0) - 1];
    c17_c_y = c17_c_a * c17_c_b;
    c17_d_a = c17_c_s;
    c17_d_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_ix), 1, 72, 1, 0) - 1];
    c17_d_y = c17_d_a * c17_d_b;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_iy), 1, 72, 1, 0) - 1] = c17_c_y - c17_d_y;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 72, 1, 0) - 1] = c17_temp;
    c17_e_a = c17_iy + 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_ix + 1;
    c17_ix = c17_f_a;
  }
}

static void c17_e_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[36], int32_T c17_ix0, int32_T c17_iy0)
{
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_k;
  real_T c17_temp;
  int32_T c17_a;
  int32_T c17_b_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_below_threshold(chartInstance);
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_ix = c17_c_ix0;
  c17_iy = c17_c_iy0;
  for (c17_k = 1; c17_k < 7; c17_k++) {
    c17_temp = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c17_ix), 1, 36, 1, 0) - 1];
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 36, 1, 0) - 1] = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_iy), 1, 36, 1, 0) - 1];
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_iy), 1, 36, 1, 0) - 1] = c17_temp;
    c17_a = c17_ix + 1;
    c17_ix = c17_a;
    c17_b_a = c17_iy + 1;
    c17_iy = c17_b_a;
  }
}

static void c17_f_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[72], int32_T c17_ix0, int32_T c17_iy0)
{
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_k;
  real_T c17_temp;
  int32_T c17_a;
  int32_T c17_b_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_ix = c17_c_ix0;
  c17_iy = c17_c_iy0;
  for (c17_k = 1; c17_k < 13; c17_k++) {
    c17_temp = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c17_ix), 1, 72, 1, 0) - 1];
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 72, 1, 0) - 1] = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_iy), 1, 72, 1, 0) - 1];
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_iy), 1, 72, 1, 0) - 1] = c17_temp;
    c17_a = c17_ix + 1;
    c17_ix = c17_a;
    c17_b_a = c17_iy + 1;
    c17_iy = c17_b_a;
  }
}

static void c17_p_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_k, real_T c17_A[36], real_T c17_B[72], real_T
  c17_C[72])
{
  int32_T c17_b_k;
  int32_T c17_c_k;
  int32_T c17_a;
  int32_T c17_km1;
  int32_T c17_cr;
  int32_T c17_b_cr;
  int32_T c17_b_a;
  int32_T c17_i1103;
  int32_T c17_c_a;
  int32_T c17_i1104;
  int32_T c17_d_a;
  int32_T c17_b;
  int32_T c17_e_a;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_ic;
  int32_T c17_b_ic;
  int32_T c17_br;
  int32_T c17_c_cr;
  int32_T c17_ar;
  int32_T c17_f_a;
  int32_T c17_b_br;
  int32_T c17_c_b;
  int32_T c17_c;
  int32_T c17_g_a;
  int32_T c17_d_b;
  int32_T c17_i1105;
  int32_T c17_h_a;
  int32_T c17_e_b;
  int32_T c17_i_a;
  int32_T c17_f_b;
  boolean_T c17_b_overflow;
  int32_T c17_ib;
  int32_T c17_b_ib;
  real_T c17_temp;
  int32_T c17_ia;
  int32_T c17_j_a;
  int32_T c17_i1106;
  int32_T c17_k_a;
  int32_T c17_i1107;
  int32_T c17_l_a;
  int32_T c17_g_b;
  int32_T c17_m_a;
  int32_T c17_h_b;
  boolean_T c17_c_overflow;
  int32_T c17_c_ic;
  int32_T c17_n_a;
  int32_T c17_o_a;
  c17_b_k = c17_k;
  c17_c_k = c17_b_k;
  c17_a = c17_c_k;
  c17_km1 = c17_a;
  for (c17_cr = 0; c17_cr < 67; c17_cr += 6) {
    c17_b_cr = c17_cr;
    c17_b_a = c17_b_cr + 1;
    c17_i1103 = c17_b_a;
    c17_c_a = c17_b_cr + 6;
    c17_i1104 = c17_c_a;
    c17_d_a = c17_i1103;
    c17_b = c17_i1104;
    c17_e_a = c17_d_a;
    c17_b_b = c17_b;
    if (c17_e_a > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_ic = c17_i1103; c17_ic <= c17_i1104; c17_ic++) {
      c17_b_ic = c17_ic;
      c17_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c17_b_ic), 1, 72, 1, 0) - 1] = 0.0;
    }
  }

  c17_br = 0;
  for (c17_c_cr = 0; c17_c_cr < 67; c17_c_cr += 6) {
    c17_b_cr = c17_c_cr;
    c17_ar = 0;
    c17_f_a = c17_br + 1;
    c17_br = c17_f_a;
    c17_b_br = c17_br;
    c17_c_b = c17_km1 - 1;
    c17_c = 12 * c17_c_b;
    c17_g_a = c17_br;
    c17_d_b = c17_c;
    c17_i1105 = c17_g_a + c17_d_b;
    c17_h_a = c17_b_br;
    c17_e_b = c17_i1105;
    c17_i_a = c17_h_a;
    c17_f_b = c17_e_b;
    if (c17_i_a > c17_f_b) {
      c17_b_overflow = FALSE;
    } else {
      c17_b_overflow = (c17_f_b > 2147483635);
    }

    if (c17_b_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
    }

    for (c17_ib = c17_b_br; c17_ib <= c17_i1105; c17_ib += 12) {
      c17_b_ib = c17_ib;
      if (c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ib), 1, 72, 1, 0) - 1] != 0.0) {
        c17_temp = c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_ib), 1, 72, 1, 0) - 1];
        c17_ia = c17_ar;
        c17_j_a = c17_b_cr + 1;
        c17_i1106 = c17_j_a;
        c17_k_a = c17_b_cr + 6;
        c17_i1107 = c17_k_a;
        c17_l_a = c17_i1106;
        c17_g_b = c17_i1107;
        c17_m_a = c17_l_a;
        c17_h_b = c17_g_b;
        if (c17_m_a > c17_h_b) {
          c17_c_overflow = FALSE;
        } else {
          c17_c_overflow = (c17_h_b > 2147483646);
        }

        if (c17_c_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_c_overflow);
        }

        for (c17_c_ic = c17_i1106; c17_c_ic <= c17_i1107; c17_c_ic++) {
          c17_b_ic = c17_c_ic;
          c17_n_a = c17_ia + 1;
          c17_ia = c17_n_a;
          c17_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ic), 1, 72, 1, 0) - 1] =
            c17_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ic), 1, 72, 1, 0) - 1] + c17_temp *
            c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_ia), 1, 36, 1, 0) - 1];
        }
      }

      c17_o_a = c17_ar + 6;
      c17_ar = c17_o_a;
    }
  }
}

static void c17_d_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], int32_T c17_ipiv[6], int32_T *c17_info)
{
  int32_T c17_i1108;
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_a;
  int32_T c17_jm1;
  int32_T c17_b;
  int32_T c17_mmj;
  int32_T c17_b_a;
  int32_T c17_c;
  int32_T c17_b_b;
  int32_T c17_jj;
  int32_T c17_c_a;
  int32_T c17_jp1j;
  int32_T c17_d_a;
  int32_T c17_b_c;
  int32_T c17_n;
  int32_T c17_ix0;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  int32_T c17_idxmax;
  int32_T c17_ix;
  real_T c17_x;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_y;
  real_T c17_d_x;
  real_T c17_e_x;
  real_T c17_b_y;
  real_T c17_smax;
  int32_T c17_d_n;
  int32_T c17_c_b;
  int32_T c17_d_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  int32_T c17_e_a;
  real_T c17_f_x;
  real_T c17_g_x;
  real_T c17_h_x;
  real_T c17_c_y;
  real_T c17_i_x;
  real_T c17_j_x;
  real_T c17_d_y;
  real_T c17_s;
  int32_T c17_f_a;
  int32_T c17_jpiv_offset;
  int32_T c17_g_a;
  int32_T c17_e_b;
  int32_T c17_jpiv;
  int32_T c17_h_a;
  int32_T c17_f_b;
  int32_T c17_c_c;
  int32_T c17_g_b;
  int32_T c17_jrow;
  int32_T c17_i_a;
  int32_T c17_h_b;
  int32_T c17_jprow;
  int32_T c17_d_ix0;
  int32_T c17_iy0;
  int32_T c17_e_ix0;
  int32_T c17_b_iy0;
  int32_T c17_f_ix0;
  int32_T c17_c_iy0;
  int32_T c17_b_ix;
  int32_T c17_iy;
  int32_T c17_c_k;
  real_T c17_temp;
  int32_T c17_j_a;
  int32_T c17_k_a;
  int32_T c17_b_jp1j;
  int32_T c17_l_a;
  int32_T c17_d_c;
  int32_T c17_m_a;
  int32_T c17_i_b;
  int32_T c17_i1109;
  int32_T c17_n_a;
  int32_T c17_j_b;
  int32_T c17_o_a;
  int32_T c17_k_b;
  boolean_T c17_b_overflow;
  int32_T c17_i;
  int32_T c17_b_i;
  real_T c17_k_x;
  real_T c17_e_y;
  real_T c17_z;
  int32_T c17_l_b;
  int32_T c17_e_c;
  int32_T c17_p_a;
  int32_T c17_f_c;
  int32_T c17_q_a;
  int32_T c17_g_c;
  int32_T c17_m;
  int32_T c17_e_n;
  int32_T c17_g_ix0;
  int32_T c17_d_iy0;
  int32_T c17_ia0;
  real_T c17_d18;
  c17_realmin(chartInstance);
  c17_eps(chartInstance);
  for (c17_i1108 = 0; c17_i1108 < 6; c17_i1108++) {
    c17_ipiv[c17_i1108] = 1 + c17_i1108;
  }

  *c17_info = 0;
  for (c17_j = 1; c17_j < 6; c17_j++) {
    c17_b_j = c17_j;
    c17_a = c17_b_j - 1;
    c17_jm1 = c17_a;
    c17_b = c17_b_j;
    c17_mmj = 6 - c17_b;
    c17_b_a = c17_jm1;
    c17_c = c17_b_a * 7;
    c17_b_b = c17_c + 1;
    c17_jj = c17_b_b;
    c17_c_a = c17_jj + 1;
    c17_jp1j = c17_c_a;
    c17_d_a = c17_mmj;
    c17_b_c = c17_d_a;
    c17_n = c17_b_c + 1;
    c17_ix0 = c17_jj;
    c17_b_n = c17_n;
    c17_b_ix0 = c17_ix0;
    c17_c_n = c17_b_n;
    c17_c_ix0 = c17_b_ix0;
    if (c17_c_n < 1) {
      c17_idxmax = 0;
    } else {
      c17_idxmax = 1;
      if (c17_c_n > 1) {
        c17_ix = c17_c_ix0;
        c17_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_ix), 1, 36, 1, 0) - 1];
        c17_b_x = c17_x;
        c17_c_x = c17_b_x;
        c17_y = muDoubleScalarAbs(c17_c_x);
        c17_d_x = 0.0;
        c17_e_x = c17_d_x;
        c17_b_y = muDoubleScalarAbs(c17_e_x);
        c17_smax = c17_y + c17_b_y;
        c17_d_n = c17_c_n;
        c17_c_b = c17_d_n;
        c17_d_b = c17_c_b;
        if (2 > c17_d_b) {
          c17_overflow = FALSE;
        } else {
          c17_overflow = (c17_d_b > 2147483646);
        }

        if (c17_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_overflow);
        }

        for (c17_k = 2; c17_k <= c17_d_n; c17_k++) {
          c17_b_k = c17_k;
          c17_e_a = c17_ix + 1;
          c17_ix = c17_e_a;
          c17_f_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_ix), 1, 36, 1, 0) - 1];
          c17_g_x = c17_f_x;
          c17_h_x = c17_g_x;
          c17_c_y = muDoubleScalarAbs(c17_h_x);
          c17_i_x = 0.0;
          c17_j_x = c17_i_x;
          c17_d_y = muDoubleScalarAbs(c17_j_x);
          c17_s = c17_c_y + c17_d_y;
          if (c17_s > c17_smax) {
            c17_idxmax = c17_b_k;
            c17_smax = c17_s;
          }
        }
      }
    }

    c17_f_a = c17_idxmax - 1;
    c17_jpiv_offset = c17_f_a;
    c17_g_a = c17_jj;
    c17_e_b = c17_jpiv_offset;
    c17_jpiv = c17_g_a + c17_e_b;
    if (c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_jpiv), 1, 36, 1, 0) - 1] != 0.0) {
      if (c17_jpiv_offset != 0) {
        c17_h_a = c17_b_j;
        c17_f_b = c17_jpiv_offset;
        c17_c_c = c17_h_a + c17_f_b;
        c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_j), 1, 6, 1, 0) - 1] = c17_c_c;
        c17_g_b = c17_jm1 + 1;
        c17_jrow = c17_g_b;
        c17_i_a = c17_jrow;
        c17_h_b = c17_jpiv_offset;
        c17_jprow = c17_i_a + c17_h_b;
        c17_d_ix0 = c17_jrow;
        c17_iy0 = c17_jprow;
        c17_e_ix0 = c17_d_ix0;
        c17_b_iy0 = c17_iy0;
        c17_below_threshold(chartInstance);
        c17_f_ix0 = c17_e_ix0;
        c17_c_iy0 = c17_b_iy0;
        c17_b_ix = c17_f_ix0;
        c17_iy = c17_c_iy0;
        for (c17_c_k = 1; c17_c_k < 7; c17_c_k++) {
          c17_temp = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 36, 1, 0) - 1];
          c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ix), 1, 36, 1, 0) - 1] =
            c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_iy), 1, 36, 1, 0) - 1];
          c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_iy), 1, 36, 1, 0) - 1] = c17_temp;
          c17_j_a = c17_b_ix + 6;
          c17_b_ix = c17_j_a;
          c17_k_a = c17_iy + 6;
          c17_iy = c17_k_a;
        }
      }

      c17_b_jp1j = c17_jp1j;
      c17_l_a = c17_mmj;
      c17_d_c = c17_l_a;
      c17_m_a = c17_jp1j;
      c17_i_b = c17_d_c - 1;
      c17_i1109 = c17_m_a + c17_i_b;
      c17_n_a = c17_b_jp1j;
      c17_j_b = c17_i1109;
      c17_o_a = c17_n_a;
      c17_k_b = c17_j_b;
      if (c17_o_a > c17_k_b) {
        c17_b_overflow = FALSE;
      } else {
        c17_b_overflow = (c17_k_b > 2147483646);
      }

      if (c17_b_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
      }

      for (c17_i = c17_b_jp1j; c17_i <= c17_i1109; c17_i++) {
        c17_b_i = c17_i;
        c17_k_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 36, 1, 0) - 1];
        c17_e_y = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_jj), 1, 36, 1, 0) - 1];
        c17_z = c17_k_x / c17_e_y;
        c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_i), 1, 36, 1, 0) - 1] = c17_z;
      }
    } else {
      *c17_info = c17_b_j;
    }

    c17_l_b = c17_b_j;
    c17_e_c = 6 - c17_l_b;
    c17_p_a = c17_jj;
    c17_f_c = c17_p_a;
    c17_q_a = c17_jj;
    c17_g_c = c17_q_a;
    c17_m = c17_mmj;
    c17_e_n = c17_e_c;
    c17_g_ix0 = c17_jp1j;
    c17_d_iy0 = c17_f_c + 6;
    c17_ia0 = c17_g_c + 7;
    c17_d18 = -1.0;
    c17_d_eml_xger(chartInstance, c17_m, c17_e_n, c17_d18, c17_g_ix0, c17_d_iy0,
                   c17_A, c17_ia0);
  }

  if (*c17_info == 0) {
    if (!(c17_A[35] != 0.0)) {
      *c17_info = 6;
    }
  }
}

static void c17_d_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[36], int32_T c17_ia0)
{
  int32_T c17_b_m;
  int32_T c17_b_n;
  real_T c17_b_alpha1;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_b_ia0;
  int32_T c17_c_m;
  int32_T c17_c_n;
  real_T c17_c_alpha1;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_c_ia0;
  int32_T c17_d_m;
  int32_T c17_d_n;
  real_T c17_d_alpha1;
  int32_T c17_d_ix0;
  int32_T c17_d_iy0;
  int32_T c17_d_ia0;
  int32_T c17_ixstart;
  int32_T c17_a;
  int32_T c17_jA;
  int32_T c17_jy;
  int32_T c17_e_n;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_j;
  real_T c17_yjy;
  real_T c17_temp;
  int32_T c17_ix;
  int32_T c17_c_b;
  int32_T c17_i1110;
  int32_T c17_b_a;
  int32_T c17_d_b;
  int32_T c17_i1111;
  int32_T c17_c_a;
  int32_T c17_e_b;
  int32_T c17_d_a;
  int32_T c17_f_b;
  boolean_T c17_b_overflow;
  int32_T c17_ijA;
  int32_T c17_b_ijA;
  int32_T c17_e_a;
  int32_T c17_f_a;
  int32_T c17_g_a;
  c17_b_m = c17_m;
  c17_b_n = c17_n;
  c17_b_alpha1 = c17_alpha1;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_b_ia0 = c17_ia0;
  c17_c_m = c17_b_m;
  c17_c_n = c17_b_n;
  c17_c_alpha1 = c17_b_alpha1;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_c_ia0 = c17_b_ia0;
  c17_d_m = c17_c_m;
  c17_d_n = c17_c_n;
  c17_d_alpha1 = c17_c_alpha1;
  c17_d_ix0 = c17_c_ix0;
  c17_d_iy0 = c17_c_iy0;
  c17_d_ia0 = c17_c_ia0;
  if (c17_d_alpha1 == 0.0) {
  } else {
    c17_ixstart = c17_d_ix0;
    c17_a = c17_d_ia0 - 1;
    c17_jA = c17_a;
    c17_jy = c17_d_iy0;
    c17_e_n = c17_d_n;
    c17_b = c17_e_n;
    c17_b_b = c17_b;
    if (1 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_j = 1; c17_j <= c17_e_n; c17_j++) {
      c17_yjy = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_jy), 1, 36, 1, 0) - 1];
      if (c17_yjy != 0.0) {
        c17_temp = c17_yjy * c17_d_alpha1;
        c17_ix = c17_ixstart;
        c17_c_b = c17_jA + 1;
        c17_i1110 = c17_c_b;
        c17_b_a = c17_d_m;
        c17_d_b = c17_jA;
        c17_i1111 = c17_b_a + c17_d_b;
        c17_c_a = c17_i1110;
        c17_e_b = c17_i1111;
        c17_d_a = c17_c_a;
        c17_f_b = c17_e_b;
        if (c17_d_a > c17_f_b) {
          c17_b_overflow = FALSE;
        } else {
          c17_b_overflow = (c17_f_b > 2147483646);
        }

        if (c17_b_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
        }

        for (c17_ijA = c17_i1110; c17_ijA <= c17_i1111; c17_ijA++) {
          c17_b_ijA = c17_ijA;
          c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ijA), 1, 36, 1, 0) - 1] =
            c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ijA), 1, 36, 1, 0) - 1] +
            c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_ix), 1, 36, 1, 0) - 1] * c17_temp;
          c17_e_a = c17_ix + 1;
          c17_ix = c17_e_a;
        }
      }

      c17_f_a = c17_jy + 6;
      c17_jy = c17_f_a;
      c17_g_a = c17_jA + 6;
      c17_jA = c17_g_a;
    }
  }
}

static void c17_h_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[36])
{
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_a;
  int32_T c17_c;
  int32_T c17_b;
  int32_T c17_b_c;
  int32_T c17_b_b;
  int32_T c17_jBcol;
  int32_T c17_k;
  int32_T c17_b_k;
  int32_T c17_b_a;
  int32_T c17_c_c;
  int32_T c17_c_b;
  int32_T c17_d_c;
  int32_T c17_d_b;
  int32_T c17_kAcol;
  int32_T c17_c_a;
  int32_T c17_e_b;
  int32_T c17_e_c;
  int32_T c17_d_a;
  int32_T c17_f_b;
  int32_T c17_f_c;
  int32_T c17_e_a;
  int32_T c17_g_b;
  int32_T c17_g_c;
  int32_T c17_f_a;
  int32_T c17_h_b;
  int32_T c17_h_c;
  real_T c17_x;
  real_T c17_y;
  real_T c17_z;
  int32_T c17_g_a;
  int32_T c17_i1112;
  int32_T c17_i_b;
  int32_T c17_j_b;
  boolean_T c17_overflow;
  int32_T c17_i;
  int32_T c17_b_i;
  int32_T c17_h_a;
  int32_T c17_k_b;
  int32_T c17_i_c;
  int32_T c17_i_a;
  int32_T c17_l_b;
  int32_T c17_j_c;
  int32_T c17_j_a;
  int32_T c17_m_b;
  int32_T c17_k_c;
  int32_T c17_k_a;
  int32_T c17_n_b;
  int32_T c17_l_c;
  for (c17_j = 1; c17_j < 7; c17_j++) {
    c17_b_j = c17_j;
    c17_a = c17_b_j;
    c17_c = c17_a;
    c17_b = c17_c - 1;
    c17_b_c = 6 * c17_b;
    c17_b_b = c17_b_c;
    c17_jBcol = c17_b_b;
    for (c17_k = 6; c17_k > 0; c17_k--) {
      c17_b_k = c17_k;
      c17_b_a = c17_b_k;
      c17_c_c = c17_b_a;
      c17_c_b = c17_c_c - 1;
      c17_d_c = 6 * c17_c_b;
      c17_d_b = c17_d_c;
      c17_kAcol = c17_d_b;
      c17_c_a = c17_b_k;
      c17_e_b = c17_jBcol;
      c17_e_c = c17_c_a + c17_e_b;
      if (c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_e_c), 1, 36, 1, 0) - 1] != 0.0) {
        c17_d_a = c17_b_k;
        c17_f_b = c17_jBcol;
        c17_f_c = c17_d_a + c17_f_b;
        c17_e_a = c17_b_k;
        c17_g_b = c17_jBcol;
        c17_g_c = c17_e_a + c17_g_b;
        c17_f_a = c17_b_k;
        c17_h_b = c17_kAcol;
        c17_h_c = c17_f_a + c17_h_b;
        c17_x = c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_g_c), 1, 36, 1, 0) - 1];
        c17_y = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_h_c), 1, 36, 1, 0) - 1];
        c17_z = c17_x / c17_y;
        c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_f_c), 1, 36, 1, 0) - 1] = c17_z;
        c17_g_a = c17_b_k - 1;
        c17_i1112 = c17_g_a;
        c17_i_b = c17_i1112;
        c17_j_b = c17_i_b;
        if (1 > c17_j_b) {
          c17_overflow = FALSE;
        } else {
          c17_overflow = (c17_j_b > 2147483646);
        }

        if (c17_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_overflow);
        }

        for (c17_i = 1; c17_i <= c17_i1112; c17_i++) {
          c17_b_i = c17_i;
          c17_h_a = c17_b_i;
          c17_k_b = c17_jBcol;
          c17_i_c = c17_h_a + c17_k_b;
          c17_i_a = c17_b_i;
          c17_l_b = c17_jBcol;
          c17_j_c = c17_i_a + c17_l_b;
          c17_j_a = c17_b_k;
          c17_m_b = c17_jBcol;
          c17_k_c = c17_j_a + c17_m_b;
          c17_k_a = c17_b_i;
          c17_n_b = c17_kAcol;
          c17_l_c = c17_k_a + c17_n_b;
          c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_i_c), 1, 36, 1, 0) - 1] =
            c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_j_c), 1, 36, 1, 0) - 1] -
            c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_k_c), 1, 36, 1, 0) - 1] *
            c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_l_c), 1, 36, 1, 0) - 1];
        }
      }
    }
  }
}

static void c17_q_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[72], real_T c17_B[72], real_T c17_C[144])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(12);
  c17_n_t = (ptrdiff_t)(12);
  c17_k_t = (ptrdiff_t)(6);
  c17_lda_t = (ptrdiff_t)(12);
  c17_ldb_t = (ptrdiff_t)(6);
  c17_ldc_t = (ptrdiff_t)(12);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_e_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], int32_T c17_ipiv[29], int32_T *c17_info)
{
  int32_T c17_i1113;
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_a;
  int32_T c17_jm1;
  int32_T c17_b;
  int32_T c17_mmj;
  int32_T c17_b_a;
  int32_T c17_c;
  int32_T c17_b_b;
  int32_T c17_jj;
  int32_T c17_c_a;
  int32_T c17_jp1j;
  int32_T c17_d_a;
  int32_T c17_b_c;
  int32_T c17_n;
  int32_T c17_ix0;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  int32_T c17_idxmax;
  int32_T c17_ix;
  real_T c17_x;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_y;
  real_T c17_d_x;
  real_T c17_e_x;
  real_T c17_b_y;
  real_T c17_smax;
  int32_T c17_d_n;
  int32_T c17_c_b;
  int32_T c17_d_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  int32_T c17_e_a;
  real_T c17_f_x;
  real_T c17_g_x;
  real_T c17_h_x;
  real_T c17_c_y;
  real_T c17_i_x;
  real_T c17_j_x;
  real_T c17_d_y;
  real_T c17_s;
  int32_T c17_f_a;
  int32_T c17_jpiv_offset;
  int32_T c17_g_a;
  int32_T c17_e_b;
  int32_T c17_jpiv;
  int32_T c17_h_a;
  int32_T c17_f_b;
  int32_T c17_c_c;
  int32_T c17_g_b;
  int32_T c17_jrow;
  int32_T c17_i_a;
  int32_T c17_h_b;
  int32_T c17_jprow;
  int32_T c17_b_jp1j;
  int32_T c17_j_a;
  int32_T c17_d_c;
  int32_T c17_k_a;
  int32_T c17_i_b;
  int32_T c17_i1114;
  int32_T c17_l_a;
  int32_T c17_j_b;
  int32_T c17_m_a;
  int32_T c17_k_b;
  boolean_T c17_b_overflow;
  int32_T c17_i;
  int32_T c17_b_i;
  real_T c17_k_x;
  real_T c17_e_y;
  real_T c17_z;
  int32_T c17_l_b;
  int32_T c17_e_c;
  int32_T c17_n_a;
  int32_T c17_f_c;
  int32_T c17_o_a;
  int32_T c17_g_c;
  int32_T c17_m;
  int32_T c17_e_n;
  int32_T c17_d_ix0;
  int32_T c17_iy0;
  int32_T c17_ia0;
  real_T c17_d19;
  c17_realmin(chartInstance);
  c17_eps(chartInstance);
  for (c17_i1113 = 0; c17_i1113 < 29; c17_i1113++) {
    c17_ipiv[c17_i1113] = 1 + c17_i1113;
  }

  *c17_info = 0;
  for (c17_j = 1; c17_j < 29; c17_j++) {
    c17_b_j = c17_j;
    c17_a = c17_b_j - 1;
    c17_jm1 = c17_a;
    c17_b = c17_b_j;
    c17_mmj = 29 - c17_b;
    c17_b_a = c17_jm1;
    c17_c = c17_b_a * 30;
    c17_b_b = c17_c + 1;
    c17_jj = c17_b_b;
    c17_c_a = c17_jj + 1;
    c17_jp1j = c17_c_a;
    c17_d_a = c17_mmj;
    c17_b_c = c17_d_a;
    c17_n = c17_b_c + 1;
    c17_ix0 = c17_jj;
    c17_b_n = c17_n;
    c17_b_ix0 = c17_ix0;
    c17_c_n = c17_b_n;
    c17_c_ix0 = c17_b_ix0;
    if (c17_c_n < 1) {
      c17_idxmax = 0;
    } else {
      c17_idxmax = 1;
      if (c17_c_n > 1) {
        c17_ix = c17_c_ix0;
        c17_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_ix), 1, 841, 1, 0) - 1];
        c17_b_x = c17_x;
        c17_c_x = c17_b_x;
        c17_y = muDoubleScalarAbs(c17_c_x);
        c17_d_x = 0.0;
        c17_e_x = c17_d_x;
        c17_b_y = muDoubleScalarAbs(c17_e_x);
        c17_smax = c17_y + c17_b_y;
        c17_d_n = c17_c_n;
        c17_c_b = c17_d_n;
        c17_d_b = c17_c_b;
        if (2 > c17_d_b) {
          c17_overflow = FALSE;
        } else {
          c17_overflow = (c17_d_b > 2147483646);
        }

        if (c17_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_overflow);
        }

        for (c17_k = 2; c17_k <= c17_d_n; c17_k++) {
          c17_b_k = c17_k;
          c17_e_a = c17_ix + 1;
          c17_ix = c17_e_a;
          c17_f_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_ix), 1, 841, 1, 0) - 1];
          c17_g_x = c17_f_x;
          c17_h_x = c17_g_x;
          c17_c_y = muDoubleScalarAbs(c17_h_x);
          c17_i_x = 0.0;
          c17_j_x = c17_i_x;
          c17_d_y = muDoubleScalarAbs(c17_j_x);
          c17_s = c17_c_y + c17_d_y;
          if (c17_s > c17_smax) {
            c17_idxmax = c17_b_k;
            c17_smax = c17_s;
          }
        }
      }
    }

    c17_f_a = c17_idxmax - 1;
    c17_jpiv_offset = c17_f_a;
    c17_g_a = c17_jj;
    c17_e_b = c17_jpiv_offset;
    c17_jpiv = c17_g_a + c17_e_b;
    if (c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_jpiv), 1, 841, 1, 0) - 1] != 0.0) {
      if (c17_jpiv_offset != 0) {
        c17_h_a = c17_b_j;
        c17_f_b = c17_jpiv_offset;
        c17_c_c = c17_h_a + c17_f_b;
        c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_j), 1, 29, 1, 0) - 1] = c17_c_c;
        c17_g_b = c17_jm1 + 1;
        c17_jrow = c17_g_b;
        c17_i_a = c17_jrow;
        c17_h_b = c17_jpiv_offset;
        c17_jprow = c17_i_a + c17_h_b;
        c17_g_eml_xswap(chartInstance, c17_A, c17_jrow, c17_jprow);
      }

      c17_b_jp1j = c17_jp1j;
      c17_j_a = c17_mmj;
      c17_d_c = c17_j_a;
      c17_k_a = c17_jp1j;
      c17_i_b = c17_d_c - 1;
      c17_i1114 = c17_k_a + c17_i_b;
      c17_l_a = c17_b_jp1j;
      c17_j_b = c17_i1114;
      c17_m_a = c17_l_a;
      c17_k_b = c17_j_b;
      if (c17_m_a > c17_k_b) {
        c17_b_overflow = FALSE;
      } else {
        c17_b_overflow = (c17_k_b > 2147483646);
      }

      if (c17_b_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
      }

      for (c17_i = c17_b_jp1j; c17_i <= c17_i1114; c17_i++) {
        c17_b_i = c17_i;
        c17_k_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 841, 1, 0) - 1];
        c17_e_y = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_jj), 1, 841, 1, 0) - 1];
        c17_z = c17_k_x / c17_e_y;
        c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_i), 1, 841, 1, 0) - 1] = c17_z;
      }
    } else {
      *c17_info = c17_b_j;
    }

    c17_l_b = c17_b_j;
    c17_e_c = 29 - c17_l_b;
    c17_n_a = c17_jj;
    c17_f_c = c17_n_a;
    c17_o_a = c17_jj;
    c17_g_c = c17_o_a;
    c17_m = c17_mmj;
    c17_e_n = c17_e_c;
    c17_d_ix0 = c17_jp1j;
    c17_iy0 = c17_f_c + 29;
    c17_ia0 = c17_g_c + 30;
    c17_d19 = -1.0;
    c17_e_eml_xger(chartInstance, c17_m, c17_e_n, c17_d19, c17_d_ix0, c17_iy0,
                   c17_A, c17_ia0);
  }

  if (*c17_info == 0) {
    if (!(c17_A[840] != 0.0)) {
      *c17_info = 29;
    }
  }
}

static void c17_g_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[841], int32_T c17_ix0, int32_T c17_iy0)
{
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_k;
  real_T c17_temp;
  int32_T c17_a;
  int32_T c17_b_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  if (c17_eml_use_refblas(chartInstance)) {
  } else {
    c17_b_below_threshold(chartInstance);
  }

  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_ix = c17_c_ix0;
  c17_iy = c17_c_iy0;
  for (c17_k = 1; c17_k < 30; c17_k++) {
    c17_temp = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c17_ix), 1, 841, 1, 0) - 1];
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 841, 1, 0) - 1] = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_iy), 1, 841, 1, 0) - 1];
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_iy), 1, 841, 1, 0) - 1] = c17_temp;
    c17_a = c17_ix + 29;
    c17_ix = c17_a;
    c17_b_a = c17_iy + 29;
    c17_iy = c17_b_a;
  }
}

static void c17_e_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[841], int32_T c17_ia0)
{
  int32_T c17_b_m;
  int32_T c17_b_n;
  real_T c17_b_alpha1;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_b_ia0;
  int32_T c17_c_m;
  int32_T c17_c_n;
  real_T c17_c_alpha1;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_c_ia0;
  int32_T c17_var;
  ptrdiff_t c17_m_t;
  int32_T c17_b_var;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_incx_t;
  ptrdiff_t c17_incy_t;
  ptrdiff_t c17_lda_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Aix0_t;
  double * c17_Aiy0_t;
  c17_b_m = c17_m;
  c17_b_n = c17_n;
  c17_b_alpha1 = c17_alpha1;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_b_ia0 = c17_ia0;
  if (c17_b_m < 1) {
  } else if (c17_b_n < 1) {
  } else {
    c17_c_m = c17_b_m;
    c17_c_n = c17_b_n;
    c17_c_alpha1 = c17_b_alpha1;
    c17_c_ix0 = c17_b_ix0;
    c17_c_iy0 = c17_b_iy0;
    c17_c_ia0 = c17_b_ia0;
    c17_var = c17_c_m;
    c17_m_t = (ptrdiff_t)(c17_var);
    c17_b_var = c17_c_n;
    c17_n_t = (ptrdiff_t)(c17_b_var);
    c17_incx_t = (ptrdiff_t)(1);
    c17_incy_t = (ptrdiff_t)(29);
    c17_lda_t = (ptrdiff_t)(29);
    c17_alpha1_t = (double *)(&c17_c_alpha1);
    c17_Aia0_t = (double *)(&c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c17_c_ia0), 1, 841, 1, 0) - 1]);
    c17_Aix0_t = (double *)(&c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c17_c_ix0), 1, 841, 1, 0) - 1]);
    c17_Aiy0_t = (double *)(&c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c17_c_iy0), 1, 841, 1, 0) - 1]);
    dger(&c17_m_t, &c17_n_t, c17_alpha1_t, c17_Aix0_t, &c17_incx_t, c17_Aiy0_t,
         &c17_incy_t, c17_Aia0_t, &c17_lda_t);
  }
}

static void c17_i_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_B[348])
{
  real_T c17_alpha1;
  char_T c17_DIAGA;
  char_T c17_TRANSA;
  char_T c17_UPLO;
  char_T c17_SIDE;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_alpha1_t;
  c17_c_below_threshold(chartInstance);
  c17_alpha1 = 1.0;
  c17_DIAGA = 'U';
  c17_TRANSA = 'N';
  c17_UPLO = 'L';
  c17_SIDE = 'L';
  c17_m_t = (ptrdiff_t)(29);
  c17_n_t = (ptrdiff_t)(12);
  c17_lda_t = (ptrdiff_t)(29);
  c17_ldb_t = (ptrdiff_t)(29);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_alpha1_t = (double *)(&c17_alpha1);
  dtrsm(&c17_SIDE, &c17_UPLO, &c17_TRANSA, &c17_DIAGA, &c17_m_t, &c17_n_t,
        c17_alpha1_t, c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t);
}

static void c17_j_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[841], real_T c17_B[348])
{
  real_T c17_alpha1;
  char_T c17_DIAGA;
  char_T c17_TRANSA;
  char_T c17_UPLO;
  char_T c17_SIDE;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_alpha1_t;
  c17_c_below_threshold(chartInstance);
  c17_alpha1 = 1.0;
  c17_DIAGA = 'N';
  c17_TRANSA = 'N';
  c17_UPLO = 'U';
  c17_SIDE = 'L';
  c17_m_t = (ptrdiff_t)(29);
  c17_n_t = (ptrdiff_t)(12);
  c17_lda_t = (ptrdiff_t)(29);
  c17_ldb_t = (ptrdiff_t)(29);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_alpha1_t = (double *)(&c17_alpha1);
  dtrsm(&c17_SIDE, &c17_UPLO, &c17_TRANSA, &c17_DIAGA, &c17_m_t, &c17_n_t,
        c17_alpha1_t, c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t);
}

static void c17_r_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[348], real_T c17_B[667], real_T c17_C[276])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(12);
  c17_n_t = (ptrdiff_t)(23);
  c17_k_t = (ptrdiff_t)(29);
  c17_lda_t = (ptrdiff_t)(12);
  c17_ldb_t = (ptrdiff_t)(29);
  c17_ldc_t = (ptrdiff_t)(12);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_s_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[348], real_T c17_B[348], real_T c17_C[144])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(12);
  c17_n_t = (ptrdiff_t)(12);
  c17_k_t = (ptrdiff_t)(29);
  c17_lda_t = (ptrdiff_t)(12);
  c17_ldb_t = (ptrdiff_t)(29);
  c17_ldc_t = (ptrdiff_t)(12);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_k_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[138])
{
  real_T c17_alpha1;
  char_T c17_DIAGA;
  char_T c17_TRANSA;
  char_T c17_UPLO;
  char_T c17_SIDE;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_alpha1_t;
  c17_d_below_threshold(chartInstance);
  c17_alpha1 = 1.0;
  c17_DIAGA = 'U';
  c17_TRANSA = 'N';
  c17_UPLO = 'L';
  c17_SIDE = 'L';
  c17_m_t = (ptrdiff_t)(6);
  c17_n_t = (ptrdiff_t)(23);
  c17_lda_t = (ptrdiff_t)(6);
  c17_ldb_t = (ptrdiff_t)(6);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_alpha1_t = (double *)(&c17_alpha1);
  dtrsm(&c17_SIDE, &c17_UPLO, &c17_TRANSA, &c17_DIAGA, &c17_m_t, &c17_n_t,
        c17_alpha1_t, c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t);
}

static void c17_l_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[36], real_T c17_B[138])
{
  real_T c17_alpha1;
  char_T c17_DIAGA;
  char_T c17_TRANSA;
  char_T c17_UPLO;
  char_T c17_SIDE;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_alpha1_t;
  c17_d_below_threshold(chartInstance);
  c17_alpha1 = 1.0;
  c17_DIAGA = 'N';
  c17_TRANSA = 'N';
  c17_UPLO = 'U';
  c17_SIDE = 'L';
  c17_m_t = (ptrdiff_t)(6);
  c17_n_t = (ptrdiff_t)(23);
  c17_lda_t = (ptrdiff_t)(6);
  c17_ldb_t = (ptrdiff_t)(6);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_alpha1_t = (double *)(&c17_alpha1);
  dtrsm(&c17_SIDE, &c17_UPLO, &c17_TRANSA, &c17_DIAGA, &c17_m_t, &c17_n_t,
        c17_alpha1_t, c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t);
}

static void c17_t_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[72], real_T c17_C[276])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(23);
  c17_n_t = (ptrdiff_t)(12);
  c17_k_t = (ptrdiff_t)(6);
  c17_lda_t = (ptrdiff_t)(23);
  c17_ldb_t = (ptrdiff_t)(6);
  c17_ldc_t = (ptrdiff_t)(23);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_u_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[276], real_T c17_C[144])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(12);
  c17_n_t = (ptrdiff_t)(12);
  c17_k_t = (ptrdiff_t)(23);
  c17_lda_t = (ptrdiff_t)(12);
  c17_ldb_t = (ptrdiff_t)(23);
  c17_ldc_t = (ptrdiff_t)(12);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_f_eml_matlab_zgetrf(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], int32_T c17_ipiv[12], int32_T *c17_info)
{
  int32_T c17_i1115;
  int32_T c17_j;
  int32_T c17_b_j;
  int32_T c17_a;
  int32_T c17_jm1;
  int32_T c17_b;
  int32_T c17_mmj;
  int32_T c17_b_a;
  int32_T c17_c;
  int32_T c17_b_b;
  int32_T c17_jj;
  int32_T c17_c_a;
  int32_T c17_jp1j;
  int32_T c17_d_a;
  int32_T c17_b_c;
  int32_T c17_n;
  int32_T c17_ix0;
  int32_T c17_b_n;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  int32_T c17_c_ix0;
  int32_T c17_idxmax;
  int32_T c17_ix;
  real_T c17_x;
  real_T c17_b_x;
  real_T c17_c_x;
  real_T c17_y;
  real_T c17_d_x;
  real_T c17_e_x;
  real_T c17_b_y;
  real_T c17_smax;
  int32_T c17_d_n;
  int32_T c17_c_b;
  int32_T c17_d_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  int32_T c17_e_a;
  real_T c17_f_x;
  real_T c17_g_x;
  real_T c17_h_x;
  real_T c17_c_y;
  real_T c17_i_x;
  real_T c17_j_x;
  real_T c17_d_y;
  real_T c17_s;
  int32_T c17_f_a;
  int32_T c17_jpiv_offset;
  int32_T c17_g_a;
  int32_T c17_e_b;
  int32_T c17_jpiv;
  int32_T c17_h_a;
  int32_T c17_f_b;
  int32_T c17_c_c;
  int32_T c17_g_b;
  int32_T c17_jrow;
  int32_T c17_i_a;
  int32_T c17_h_b;
  int32_T c17_jprow;
  int32_T c17_d_ix0;
  int32_T c17_iy0;
  int32_T c17_e_ix0;
  int32_T c17_b_iy0;
  int32_T c17_f_ix0;
  int32_T c17_c_iy0;
  int32_T c17_b_ix;
  int32_T c17_iy;
  int32_T c17_c_k;
  real_T c17_temp;
  int32_T c17_j_a;
  int32_T c17_k_a;
  int32_T c17_b_jp1j;
  int32_T c17_l_a;
  int32_T c17_d_c;
  int32_T c17_m_a;
  int32_T c17_i_b;
  int32_T c17_i1116;
  int32_T c17_n_a;
  int32_T c17_j_b;
  int32_T c17_o_a;
  int32_T c17_k_b;
  boolean_T c17_b_overflow;
  int32_T c17_i;
  int32_T c17_b_i;
  real_T c17_k_x;
  real_T c17_e_y;
  real_T c17_z;
  int32_T c17_l_b;
  int32_T c17_e_c;
  int32_T c17_p_a;
  int32_T c17_f_c;
  int32_T c17_q_a;
  int32_T c17_g_c;
  int32_T c17_m;
  int32_T c17_e_n;
  int32_T c17_g_ix0;
  int32_T c17_d_iy0;
  int32_T c17_ia0;
  real_T c17_d20;
  c17_realmin(chartInstance);
  c17_eps(chartInstance);
  for (c17_i1115 = 0; c17_i1115 < 12; c17_i1115++) {
    c17_ipiv[c17_i1115] = 1 + c17_i1115;
  }

  *c17_info = 0;
  for (c17_j = 1; c17_j < 12; c17_j++) {
    c17_b_j = c17_j;
    c17_a = c17_b_j - 1;
    c17_jm1 = c17_a;
    c17_b = c17_b_j;
    c17_mmj = 12 - c17_b;
    c17_b_a = c17_jm1;
    c17_c = c17_b_a * 13;
    c17_b_b = c17_c + 1;
    c17_jj = c17_b_b;
    c17_c_a = c17_jj + 1;
    c17_jp1j = c17_c_a;
    c17_d_a = c17_mmj;
    c17_b_c = c17_d_a;
    c17_n = c17_b_c + 1;
    c17_ix0 = c17_jj;
    c17_b_n = c17_n;
    c17_b_ix0 = c17_ix0;
    c17_c_n = c17_b_n;
    c17_c_ix0 = c17_b_ix0;
    if (c17_c_n < 1) {
      c17_idxmax = 0;
    } else {
      c17_idxmax = 1;
      if (c17_c_n > 1) {
        c17_ix = c17_c_ix0;
        c17_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_ix), 1, 144, 1, 0) - 1];
        c17_b_x = c17_x;
        c17_c_x = c17_b_x;
        c17_y = muDoubleScalarAbs(c17_c_x);
        c17_d_x = 0.0;
        c17_e_x = c17_d_x;
        c17_b_y = muDoubleScalarAbs(c17_e_x);
        c17_smax = c17_y + c17_b_y;
        c17_d_n = c17_c_n;
        c17_c_b = c17_d_n;
        c17_d_b = c17_c_b;
        if (2 > c17_d_b) {
          c17_overflow = FALSE;
        } else {
          c17_overflow = (c17_d_b > 2147483646);
        }

        if (c17_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_overflow);
        }

        for (c17_k = 2; c17_k <= c17_d_n; c17_k++) {
          c17_b_k = c17_k;
          c17_e_a = c17_ix + 1;
          c17_ix = c17_e_a;
          c17_f_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_ix), 1, 144, 1, 0) - 1];
          c17_g_x = c17_f_x;
          c17_h_x = c17_g_x;
          c17_c_y = muDoubleScalarAbs(c17_h_x);
          c17_i_x = 0.0;
          c17_j_x = c17_i_x;
          c17_d_y = muDoubleScalarAbs(c17_j_x);
          c17_s = c17_c_y + c17_d_y;
          if (c17_s > c17_smax) {
            c17_idxmax = c17_b_k;
            c17_smax = c17_s;
          }
        }
      }
    }

    c17_f_a = c17_idxmax - 1;
    c17_jpiv_offset = c17_f_a;
    c17_g_a = c17_jj;
    c17_e_b = c17_jpiv_offset;
    c17_jpiv = c17_g_a + c17_e_b;
    if (c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_jpiv), 1, 144, 1, 0) - 1] != 0.0) {
      if (c17_jpiv_offset != 0) {
        c17_h_a = c17_b_j;
        c17_f_b = c17_jpiv_offset;
        c17_c_c = c17_h_a + c17_f_b;
        c17_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_j), 1, 12, 1, 0) - 1] = c17_c_c;
        c17_g_b = c17_jm1 + 1;
        c17_jrow = c17_g_b;
        c17_i_a = c17_jrow;
        c17_h_b = c17_jpiv_offset;
        c17_jprow = c17_i_a + c17_h_b;
        c17_d_ix0 = c17_jrow;
        c17_iy0 = c17_jprow;
        c17_e_ix0 = c17_d_ix0;
        c17_b_iy0 = c17_iy0;
        c17_f_ix0 = c17_e_ix0;
        c17_c_iy0 = c17_b_iy0;
        c17_b_ix = c17_f_ix0;
        c17_iy = c17_c_iy0;
        for (c17_c_k = 1; c17_c_k < 13; c17_c_k++) {
          c17_temp = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_ix), 1, 144, 1, 0) - 1];
          c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ix), 1, 144, 1, 0) - 1] =
            c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_iy), 1, 144, 1, 0) - 1];
          c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_iy), 1, 144, 1, 0) - 1] = c17_temp;
          c17_j_a = c17_b_ix + 12;
          c17_b_ix = c17_j_a;
          c17_k_a = c17_iy + 12;
          c17_iy = c17_k_a;
        }
      }

      c17_b_jp1j = c17_jp1j;
      c17_l_a = c17_mmj;
      c17_d_c = c17_l_a;
      c17_m_a = c17_jp1j;
      c17_i_b = c17_d_c - 1;
      c17_i1116 = c17_m_a + c17_i_b;
      c17_n_a = c17_b_jp1j;
      c17_j_b = c17_i1116;
      c17_o_a = c17_n_a;
      c17_k_b = c17_j_b;
      if (c17_o_a > c17_k_b) {
        c17_b_overflow = FALSE;
      } else {
        c17_b_overflow = (c17_k_b > 2147483646);
      }

      if (c17_b_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
      }

      for (c17_i = c17_b_jp1j; c17_i <= c17_i1116; c17_i++) {
        c17_b_i = c17_i;
        c17_k_x = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_b_i), 1, 144, 1, 0) - 1];
        c17_e_y = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c17_jj), 1, 144, 1, 0) - 1];
        c17_z = c17_k_x / c17_e_y;
        c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_i), 1, 144, 1, 0) - 1] = c17_z;
      }
    } else {
      *c17_info = c17_b_j;
    }

    c17_l_b = c17_b_j;
    c17_e_c = 12 - c17_l_b;
    c17_p_a = c17_jj;
    c17_f_c = c17_p_a;
    c17_q_a = c17_jj;
    c17_g_c = c17_q_a;
    c17_m = c17_mmj;
    c17_e_n = c17_e_c;
    c17_g_ix0 = c17_jp1j;
    c17_d_iy0 = c17_f_c + 12;
    c17_ia0 = c17_g_c + 13;
    c17_d20 = -1.0;
    c17_f_eml_xger(chartInstance, c17_m, c17_e_n, c17_d20, c17_g_ix0, c17_d_iy0,
                   c17_A, c17_ia0);
  }

  if (*c17_info == 0) {
    if (!(c17_A[143] != 0.0)) {
      *c17_info = 12;
    }
  }
}

static void c17_f_eml_xger(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_m, int32_T c17_n, real_T c17_alpha1, int32_T
  c17_ix0, int32_T c17_iy0, real_T c17_A[144], int32_T c17_ia0)
{
  int32_T c17_b_m;
  int32_T c17_b_n;
  real_T c17_b_alpha1;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_b_ia0;
  int32_T c17_c_m;
  int32_T c17_c_n;
  real_T c17_c_alpha1;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_c_ia0;
  int32_T c17_d_m;
  int32_T c17_d_n;
  real_T c17_d_alpha1;
  int32_T c17_d_ix0;
  int32_T c17_d_iy0;
  int32_T c17_d_ia0;
  int32_T c17_ixstart;
  int32_T c17_a;
  int32_T c17_jA;
  int32_T c17_jy;
  int32_T c17_e_n;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_j;
  real_T c17_yjy;
  real_T c17_temp;
  int32_T c17_ix;
  int32_T c17_c_b;
  int32_T c17_i1117;
  int32_T c17_b_a;
  int32_T c17_d_b;
  int32_T c17_i1118;
  int32_T c17_c_a;
  int32_T c17_e_b;
  int32_T c17_d_a;
  int32_T c17_f_b;
  boolean_T c17_b_overflow;
  int32_T c17_ijA;
  int32_T c17_b_ijA;
  int32_T c17_e_a;
  int32_T c17_f_a;
  int32_T c17_g_a;
  c17_b_m = c17_m;
  c17_b_n = c17_n;
  c17_b_alpha1 = c17_alpha1;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_b_ia0 = c17_ia0;
  c17_c_m = c17_b_m;
  c17_c_n = c17_b_n;
  c17_c_alpha1 = c17_b_alpha1;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_c_ia0 = c17_b_ia0;
  c17_d_m = c17_c_m;
  c17_d_n = c17_c_n;
  c17_d_alpha1 = c17_c_alpha1;
  c17_d_ix0 = c17_c_ix0;
  c17_d_iy0 = c17_c_iy0;
  c17_d_ia0 = c17_c_ia0;
  if (c17_d_alpha1 == 0.0) {
  } else {
    c17_ixstart = c17_d_ix0;
    c17_a = c17_d_ia0 - 1;
    c17_jA = c17_a;
    c17_jy = c17_d_iy0;
    c17_e_n = c17_d_n;
    c17_b = c17_e_n;
    c17_b_b = c17_b;
    if (1 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_j = 1; c17_j <= c17_e_n; c17_j++) {
      c17_yjy = c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c17_jy), 1, 144, 1, 0) - 1];
      if (c17_yjy != 0.0) {
        c17_temp = c17_yjy * c17_d_alpha1;
        c17_ix = c17_ixstart;
        c17_c_b = c17_jA + 1;
        c17_i1117 = c17_c_b;
        c17_b_a = c17_d_m;
        c17_d_b = c17_jA;
        c17_i1118 = c17_b_a + c17_d_b;
        c17_c_a = c17_i1117;
        c17_e_b = c17_i1118;
        c17_d_a = c17_c_a;
        c17_f_b = c17_e_b;
        if (c17_d_a > c17_f_b) {
          c17_b_overflow = FALSE;
        } else {
          c17_b_overflow = (c17_f_b > 2147483646);
        }

        if (c17_b_overflow) {
          c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
        }

        for (c17_ijA = c17_i1117; c17_ijA <= c17_i1118; c17_ijA++) {
          c17_b_ijA = c17_ijA;
          c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ijA), 1, 144, 1, 0) - 1] =
            c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_b_ijA), 1, 144, 1, 0) - 1] +
            c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c17_ix), 1, 144, 1, 0) - 1] * c17_temp;
          c17_e_a = c17_ix + 1;
          c17_ix = c17_e_a;
        }
      }

      c17_f_a = c17_jy + 12;
      c17_jy = c17_f_a;
      c17_g_a = c17_jA + 12;
      c17_jA = c17_g_a;
    }
  }
}

static void c17_m_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_B[276])
{
  real_T c17_alpha1;
  char_T c17_DIAGA;
  char_T c17_TRANSA;
  char_T c17_UPLO;
  char_T c17_SIDE;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_alpha1_t;
  c17_e_below_threshold(chartInstance);
  c17_alpha1 = 1.0;
  c17_DIAGA = 'U';
  c17_TRANSA = 'N';
  c17_UPLO = 'L';
  c17_SIDE = 'L';
  c17_m_t = (ptrdiff_t)(12);
  c17_n_t = (ptrdiff_t)(23);
  c17_lda_t = (ptrdiff_t)(12);
  c17_ldb_t = (ptrdiff_t)(12);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_alpha1_t = (double *)(&c17_alpha1);
  dtrsm(&c17_SIDE, &c17_UPLO, &c17_TRANSA, &c17_DIAGA, &c17_m_t, &c17_n_t,
        c17_alpha1_t, c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t);
}

static void c17_n_eml_xtrsm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[144], real_T c17_B[276])
{
  real_T c17_alpha1;
  char_T c17_DIAGA;
  char_T c17_TRANSA;
  char_T c17_UPLO;
  char_T c17_SIDE;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_alpha1_t;
  c17_e_below_threshold(chartInstance);
  c17_alpha1 = 1.0;
  c17_DIAGA = 'N';
  c17_TRANSA = 'N';
  c17_UPLO = 'U';
  c17_SIDE = 'L';
  c17_m_t = (ptrdiff_t)(12);
  c17_n_t = (ptrdiff_t)(23);
  c17_lda_t = (ptrdiff_t)(12);
  c17_ldb_t = (ptrdiff_t)(12);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_alpha1_t = (double *)(&c17_alpha1);
  dtrsm(&c17_SIDE, &c17_UPLO, &c17_TRANSA, &c17_DIAGA, &c17_m_t, &c17_n_t,
        c17_alpha1_t, c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t);
}

static void c17_v_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[276], real_T c17_C[529])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(23);
  c17_n_t = (ptrdiff_t)(23);
  c17_k_t = (ptrdiff_t)(12);
  c17_lda_t = (ptrdiff_t)(23);
  c17_ldb_t = (ptrdiff_t)(12);
  c17_ldc_t = (ptrdiff_t)(23);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_w_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[138], real_T c17_B[138], real_T c17_C[529])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(23);
  c17_n_t = (ptrdiff_t)(23);
  c17_k_t = (ptrdiff_t)(6);
  c17_lda_t = (ptrdiff_t)(23);
  c17_ldb_t = (ptrdiff_t)(6);
  c17_ldc_t = (ptrdiff_t)(23);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_x_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[529], real_T c17_C[529])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(23);
  c17_n_t = (ptrdiff_t)(23);
  c17_k_t = (ptrdiff_t)(23);
  c17_lda_t = (ptrdiff_t)(23);
  c17_ldb_t = (ptrdiff_t)(23);
  c17_ldc_t = (ptrdiff_t)(23);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_l_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[529], int32_T
  c17_ix0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_d_ix0;
  int32_T c17_d_a;
  int32_T c17_c;
  int32_T c17_b;
  int32_T c17_b_c;
  int32_T c17_e_a;
  int32_T c17_b_b;
  int32_T c17_i1119;
  int32_T c17_f_a;
  int32_T c17_c_b;
  int32_T c17_g_a;
  int32_T c17_d_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_d_ix0 = c17_c_ix0;
  c17_d_a = c17_c_n;
  c17_c = c17_d_a;
  c17_b = c17_c - 1;
  c17_b_c = c17_b;
  c17_e_a = c17_c_ix0;
  c17_b_b = c17_b_c;
  c17_i1119 = c17_e_a + c17_b_b;
  c17_f_a = c17_d_ix0;
  c17_c_b = c17_i1119;
  c17_g_a = c17_f_a;
  c17_d_b = c17_c_b;
  if (c17_g_a > c17_d_b) {
    c17_overflow = FALSE;
  } else {
    c17_overflow = (c17_d_b > 2147483646);
  }

  if (c17_overflow) {
    c17_check_forloop_overflow_error(chartInstance, c17_overflow);
  }

  for (c17_k = c17_d_ix0; c17_k <= c17_i1119; c17_k++) {
    c17_b_k = c17_k;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 529, 1, 0) - 1] = c17_c_a *
      c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 529, 1, 0) - 1];
  }
}

static void c17_l_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, int32_T c17_ix0, real_T c17_y[529],
  int32_T c17_iy0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_a;
  int32_T c17_ix;
  int32_T c17_e_a;
  int32_T c17_iy;
  int32_T c17_f_a;
  int32_T c17_i1120;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_g_a;
  int32_T c17_c;
  int32_T c17_h_a;
  int32_T c17_b_c;
  int32_T c17_i_a;
  int32_T c17_c_c;
  int32_T c17_j_a;
  int32_T c17_k_a;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  if (c17_c_n < 1) {
  } else if (c17_c_a == 0.0) {
  } else {
    c17_d_a = c17_c_ix0 - 1;
    c17_ix = c17_d_a;
    c17_e_a = c17_c_iy0 - 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_c_n - 1;
    c17_i1120 = c17_f_a;
    c17_b = c17_i1120;
    c17_b_b = c17_b;
    if (0 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 0; c17_k <= c17_i1120; c17_k++) {
      c17_g_a = c17_iy;
      c17_c = c17_g_a;
      c17_h_a = c17_iy;
      c17_b_c = c17_h_a;
      c17_i_a = c17_ix;
      c17_c_c = c17_i_a;
      c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c + 1)), 1, 529, 1, 0) - 1] =
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_b_c + 1)), 1, 529, 1, 0) - 1] + c17_c_a *
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c_c + 1)), 1, 529, 1, 0) - 1];
      c17_j_a = c17_ix + 1;
      c17_ix = c17_j_a;
      c17_k_a = c17_iy + 1;
      c17_iy = c17_k_a;
    }
  }
}

static void c17_m_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[23], int32_T c17_ix0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_d_ix0;
  int32_T c17_d_a;
  int32_T c17_c;
  int32_T c17_b;
  int32_T c17_b_c;
  int32_T c17_e_a;
  int32_T c17_b_b;
  int32_T c17_i1121;
  int32_T c17_f_a;
  int32_T c17_c_b;
  int32_T c17_g_a;
  int32_T c17_d_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_d_ix0 = c17_c_ix0;
  c17_d_a = c17_c_n;
  c17_c = c17_d_a;
  c17_b = c17_c - 1;
  c17_b_c = c17_b;
  c17_e_a = c17_c_ix0;
  c17_b_b = c17_b_c;
  c17_i1121 = c17_e_a + c17_b_b;
  c17_f_a = c17_d_ix0;
  c17_c_b = c17_i1121;
  c17_g_a = c17_f_a;
  c17_d_b = c17_c_b;
  if (c17_g_a > c17_d_b) {
    c17_overflow = FALSE;
  } else {
    c17_overflow = (c17_d_b > 2147483646);
  }

  if (c17_overflow) {
    c17_check_forloop_overflow_error(chartInstance, c17_overflow);
  }

  for (c17_k = c17_d_ix0; c17_k <= c17_i1121; c17_k++) {
    c17_b_k = c17_k;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 23, 1, 0) - 1] = c17_c_a *
      c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 23, 1, 0) - 1];
  }
}

static void c17_m_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[529], int32_T
  c17_ix0, real_T c17_y[23], int32_T c17_iy0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_a;
  int32_T c17_ix;
  int32_T c17_e_a;
  int32_T c17_iy;
  int32_T c17_f_a;
  int32_T c17_i1122;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_g_a;
  int32_T c17_c;
  int32_T c17_h_a;
  int32_T c17_b_c;
  int32_T c17_i_a;
  int32_T c17_c_c;
  int32_T c17_j_a;
  int32_T c17_k_a;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  if (c17_c_n < 1) {
  } else if (c17_c_a == 0.0) {
  } else {
    c17_d_a = c17_c_ix0 - 1;
    c17_ix = c17_d_a;
    c17_e_a = c17_c_iy0 - 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_c_n - 1;
    c17_i1122 = c17_f_a;
    c17_b = c17_i1122;
    c17_b_b = c17_b;
    if (0 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 0; c17_k <= c17_i1122; c17_k++) {
      c17_g_a = c17_iy;
      c17_c = c17_g_a;
      c17_h_a = c17_iy;
      c17_b_c = c17_h_a;
      c17_i_a = c17_ix;
      c17_c_c = c17_i_a;
      c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c + 1)), 1, 23, 1, 0) - 1] =
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_b_c + 1)), 1, 23, 1, 0) - 1] + c17_c_a *
        c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c_c + 1)), 1, 529, 1, 0) - 1];
      c17_j_a = c17_ix + 1;
      c17_ix = c17_j_a;
      c17_k_a = c17_iy + 1;
      c17_iy = c17_k_a;
    }
  }
}

static void c17_n_eml_xaxpy(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_n, real_T c17_a, real_T c17_x[23], int32_T c17_ix0,
  real_T c17_y[529], int32_T c17_iy0)
{
  int32_T c17_b_n;
  real_T c17_b_a;
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_n;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_d_a;
  int32_T c17_ix;
  int32_T c17_e_a;
  int32_T c17_iy;
  int32_T c17_f_a;
  int32_T c17_i1123;
  int32_T c17_b;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_g_a;
  int32_T c17_c;
  int32_T c17_h_a;
  int32_T c17_b_c;
  int32_T c17_i_a;
  int32_T c17_c_c;
  int32_T c17_j_a;
  int32_T c17_k_a;
  c17_b_n = c17_n;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_c_n = c17_b_n;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  if (c17_c_n < 1) {
  } else if (c17_c_a == 0.0) {
  } else {
    c17_d_a = c17_c_ix0 - 1;
    c17_ix = c17_d_a;
    c17_e_a = c17_c_iy0 - 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_c_n - 1;
    c17_i1123 = c17_f_a;
    c17_b = c17_i1123;
    c17_b_b = c17_b;
    if (0 > c17_b_b) {
      c17_overflow = FALSE;
    } else {
      c17_overflow = (c17_b_b > 2147483646);
    }

    if (c17_overflow) {
      c17_check_forloop_overflow_error(chartInstance, c17_overflow);
    }

    for (c17_k = 0; c17_k <= c17_i1123; c17_k++) {
      c17_g_a = c17_iy;
      c17_c = c17_g_a;
      c17_h_a = c17_iy;
      c17_b_c = c17_h_a;
      c17_i_a = c17_ix;
      c17_c_c = c17_i_a;
      c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c + 1)), 1, 529, 1, 0) - 1] =
        c17_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_b_c + 1)), 1, 529, 1, 0) - 1] + c17_c_a *
        c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c17_c_c + 1)), 1, 23, 1, 0) - 1];
      c17_j_a = c17_ix + 1;
      c17_ix = c17_j_a;
      c17_k_a = c17_iy + 1;
      c17_iy = c17_k_a;
    }
  }
}

static void c17_n_eml_xscal(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_a, real_T c17_x[529], int32_T c17_ix0)
{
  real_T c17_b_a;
  int32_T c17_b_ix0;
  real_T c17_c_a;
  int32_T c17_c_ix0;
  int32_T c17_d_ix0;
  int32_T c17_d_a;
  int32_T c17_i1124;
  int32_T c17_e_a;
  int32_T c17_b;
  int32_T c17_f_a;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_k;
  int32_T c17_b_k;
  c17_b_a = c17_a;
  c17_b_ix0 = c17_ix0;
  c17_c_a = c17_b_a;
  c17_c_ix0 = c17_b_ix0;
  c17_d_ix0 = c17_c_ix0;
  c17_d_a = c17_c_ix0 + 22;
  c17_i1124 = c17_d_a;
  c17_e_a = c17_d_ix0;
  c17_b = c17_i1124;
  c17_f_a = c17_e_a;
  c17_b_b = c17_b;
  if (c17_f_a > c17_b_b) {
    c17_overflow = FALSE;
  } else {
    c17_overflow = (c17_b_b > 2147483646);
  }

  if (c17_overflow) {
    c17_check_forloop_overflow_error(chartInstance, c17_overflow);
  }

  for (c17_k = c17_d_ix0; c17_k <= c17_i1124; c17_k++) {
    c17_b_k = c17_k;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 529, 1, 0) - 1] = c17_c_a *
      c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_b_k), 1, 529, 1, 0) - 1];
  }
}

static void c17_f_eml_xrot(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[529], int32_T c17_ix0, int32_T c17_iy0, real_T
  c17_c, real_T c17_s)
{
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  real_T c17_b_c;
  real_T c17_b_s;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  real_T c17_c_c;
  real_T c17_c_s;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_k;
  real_T c17_a;
  real_T c17_b;
  real_T c17_y;
  real_T c17_b_a;
  real_T c17_b_b;
  real_T c17_b_y;
  real_T c17_temp;
  real_T c17_c_a;
  real_T c17_c_b;
  real_T c17_c_y;
  real_T c17_d_a;
  real_T c17_d_b;
  real_T c17_d_y;
  int32_T c17_e_a;
  int32_T c17_f_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  c17_b_c = c17_c;
  c17_b_s = c17_s;
  if (c17_eml_use_refblas(chartInstance)) {
  } else {
    c17_f_below_threshold(chartInstance);
  }

  c17_o_eml_scalar_eg(chartInstance);
  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_c_c = c17_b_c;
  c17_c_s = c17_b_s;
  c17_ix = c17_c_ix0;
  c17_iy = c17_c_iy0;
  for (c17_k = 1; c17_k < 24; c17_k++) {
    c17_a = c17_c_c;
    c17_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 529, 1, 0) - 1];
    c17_y = c17_a * c17_b;
    c17_b_a = c17_c_s;
    c17_b_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_iy), 1, 529, 1, 0) - 1];
    c17_b_y = c17_b_a * c17_b_b;
    c17_temp = c17_y + c17_b_y;
    c17_c_a = c17_c_c;
    c17_c_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_iy), 1, 529, 1, 0) - 1];
    c17_c_y = c17_c_a * c17_c_b;
    c17_d_a = c17_c_s;
    c17_d_b = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c17_ix), 1, 529, 1, 0) - 1];
    c17_d_y = c17_d_a * c17_d_b;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_iy), 1, 529, 1, 0) - 1] = c17_c_y - c17_d_y;
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 529, 1, 0) - 1] = c17_temp;
    c17_e_a = c17_iy + 1;
    c17_iy = c17_e_a;
    c17_f_a = c17_ix + 1;
    c17_ix = c17_f_a;
  }
}

static void c17_h_eml_xswap(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_x[529], int32_T c17_ix0, int32_T c17_iy0)
{
  int32_T c17_b_ix0;
  int32_T c17_b_iy0;
  int32_T c17_c_ix0;
  int32_T c17_c_iy0;
  int32_T c17_ix;
  int32_T c17_iy;
  int32_T c17_k;
  real_T c17_temp;
  int32_T c17_a;
  int32_T c17_b_a;
  c17_b_ix0 = c17_ix0;
  c17_b_iy0 = c17_iy0;
  if (c17_eml_use_refblas(chartInstance)) {
  } else {
    c17_g_below_threshold(chartInstance);
  }

  c17_c_ix0 = c17_b_ix0;
  c17_c_iy0 = c17_b_iy0;
  c17_ix = c17_c_ix0;
  c17_iy = c17_c_iy0;
  for (c17_k = 1; c17_k < 24; c17_k++) {
    c17_temp = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c17_ix), 1, 529, 1, 0) - 1];
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_ix), 1, 529, 1, 0) - 1] = c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c17_iy), 1, 529, 1, 0) - 1];
    c17_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c17_iy), 1, 529, 1, 0) - 1] = c17_temp;
    c17_a = c17_ix + 1;
    c17_ix = c17_a;
    c17_b_a = c17_iy + 1;
    c17_iy = c17_b_a;
  }
}

static void c17_y_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c17_k, real_T c17_A[529], real_T c17_B[529], real_T
  c17_C[529])
{
  int32_T c17_b_k;
  int32_T c17_c_k;
  int32_T c17_a;
  int32_T c17_km1;
  int32_T c17_cr;
  int32_T c17_b_cr;
  int32_T c17_b_a;
  int32_T c17_i1125;
  int32_T c17_c_a;
  int32_T c17_i1126;
  int32_T c17_d_a;
  int32_T c17_b;
  int32_T c17_e_a;
  int32_T c17_b_b;
  boolean_T c17_overflow;
  int32_T c17_ic;
  int32_T c17_b_ic;
  int32_T c17_br;
  int32_T c17_c_cr;
  int32_T c17_ar;
  int32_T c17_f_a;
  int32_T c17_b_br;
  int32_T c17_c_b;
  int32_T c17_c;
  int32_T c17_g_a;
  int32_T c17_d_b;
  int32_T c17_i1127;
  int32_T c17_h_a;
  int32_T c17_e_b;
  int32_T c17_i_a;
  int32_T c17_f_b;
  boolean_T c17_b_overflow;
  int32_T c17_ib;
  int32_T c17_b_ib;
  real_T c17_temp;
  int32_T c17_ia;
  int32_T c17_j_a;
  int32_T c17_i1128;
  int32_T c17_k_a;
  int32_T c17_i1129;
  int32_T c17_l_a;
  int32_T c17_g_b;
  int32_T c17_m_a;
  int32_T c17_h_b;
  boolean_T c17_c_overflow;
  int32_T c17_c_ic;
  int32_T c17_n_a;
  int32_T c17_o_a;
  int32_T c17_d_k;
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  int32_T c17_var;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_b_k = c17_k;
  if (c17_eml_use_refblas(chartInstance)) {
    c17_o_eml_scalar_eg(chartInstance);
    c17_o_eml_scalar_eg(chartInstance);
    c17_c_k = c17_b_k;
    c17_a = c17_c_k;
    c17_km1 = c17_a;
    for (c17_cr = 0; c17_cr < 507; c17_cr += 23) {
      c17_b_cr = c17_cr;
      c17_b_a = c17_b_cr + 1;
      c17_i1125 = c17_b_a;
      c17_c_a = c17_b_cr + 23;
      c17_i1126 = c17_c_a;
      c17_d_a = c17_i1125;
      c17_b = c17_i1126;
      c17_e_a = c17_d_a;
      c17_b_b = c17_b;
      if (c17_e_a > c17_b_b) {
        c17_overflow = FALSE;
      } else {
        c17_overflow = (c17_b_b > 2147483646);
      }

      if (c17_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_overflow);
      }

      for (c17_ic = c17_i1125; c17_ic <= c17_i1126; c17_ic++) {
        c17_b_ic = c17_ic;
        c17_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c17_b_ic), 1, 529, 1, 0) - 1] = 0.0;
      }
    }

    c17_br = 0;
    for (c17_c_cr = 0; c17_c_cr < 507; c17_c_cr += 23) {
      c17_b_cr = c17_c_cr;
      c17_ar = 0;
      c17_f_a = c17_br + 1;
      c17_br = c17_f_a;
      c17_b_br = c17_br;
      c17_c_b = c17_km1 - 1;
      c17_c = 23 * c17_c_b;
      c17_g_a = c17_br;
      c17_d_b = c17_c;
      c17_i1127 = c17_g_a + c17_d_b;
      c17_h_a = c17_b_br;
      c17_e_b = c17_i1127;
      c17_i_a = c17_h_a;
      c17_f_b = c17_e_b;
      if (c17_i_a > c17_f_b) {
        c17_b_overflow = FALSE;
      } else {
        c17_b_overflow = (c17_f_b > 2147483624);
      }

      if (c17_b_overflow) {
        c17_check_forloop_overflow_error(chartInstance, c17_b_overflow);
      }

      for (c17_ib = c17_b_br; c17_ib <= c17_i1127; c17_ib += 23) {
        c17_b_ib = c17_ib;
        if (c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_ib), 1, 529, 1, 0) - 1] != 0.0) {
          c17_temp = c17_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c17_b_ib), 1, 529, 1, 0) - 1];
          c17_ia = c17_ar;
          c17_j_a = c17_b_cr + 1;
          c17_i1128 = c17_j_a;
          c17_k_a = c17_b_cr + 23;
          c17_i1129 = c17_k_a;
          c17_l_a = c17_i1128;
          c17_g_b = c17_i1129;
          c17_m_a = c17_l_a;
          c17_h_b = c17_g_b;
          if (c17_m_a > c17_h_b) {
            c17_c_overflow = FALSE;
          } else {
            c17_c_overflow = (c17_h_b > 2147483646);
          }

          if (c17_c_overflow) {
            c17_check_forloop_overflow_error(chartInstance, c17_c_overflow);
          }

          for (c17_c_ic = c17_i1128; c17_c_ic <= c17_i1129; c17_c_ic++) {
            c17_b_ic = c17_c_ic;
            c17_n_a = c17_ia + 1;
            c17_ia = c17_n_a;
            c17_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c17_b_ic), 1, 529, 1, 0) - 1] =
              c17_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c17_b_ic), 1, 529, 1, 0) - 1] + c17_temp *
              c17_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c17_ia), 1, 529, 1, 0) - 1];
          }
        }

        c17_o_a = c17_ar + 23;
        c17_ar = c17_o_a;
      }
    }
  } else {
    c17_h_below_threshold(chartInstance);
    if (c17_b_k < 1) {
    } else {
      c17_d_k = c17_b_k;
      c17_alpha1 = 1.0;
      c17_beta1 = 0.0;
      c17_TRANSB = 'C';
      c17_TRANSA = 'N';
      c17_m_t = (ptrdiff_t)(23);
      c17_n_t = (ptrdiff_t)(23);
      c17_var = c17_d_k;
      c17_k_t = (ptrdiff_t)(c17_var);
      c17_lda_t = (ptrdiff_t)(23);
      c17_ldb_t = (ptrdiff_t)(23);
      c17_ldc_t = (ptrdiff_t)(23);
      c17_alpha1_t = (double *)(&c17_alpha1);
      c17_Aia0_t = (double *)(&c17_A[0]);
      c17_Bib0_t = (double *)(&c17_B[0]);
      c17_beta1_t = (double *)(&c17_beta1);
      c17_Cic0_t = (double *)(&c17_C[0]);
      dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
            c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t,
            c17_Cic0_t, &c17_ldc_t);
    }
  }
}

static void c17_ab_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[114], real_T c17_B[36], real_T c17_C[114])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(19);
  c17_n_t = (ptrdiff_t)(6);
  c17_k_t = (ptrdiff_t)(6);
  c17_lda_t = (ptrdiff_t)(19);
  c17_ldb_t = (ptrdiff_t)(6);
  c17_ldc_t = (ptrdiff_t)(19);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_bb_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[23], real_T c17_C[23])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(23);
  c17_n_t = (ptrdiff_t)(1);
  c17_k_t = (ptrdiff_t)(23);
  c17_lda_t = (ptrdiff_t)(23);
  c17_ldb_t = (ptrdiff_t)(23);
  c17_ldc_t = (ptrdiff_t)(23);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_cb_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[276], real_T c17_B[144], real_T c17_C[276])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(23);
  c17_n_t = (ptrdiff_t)(12);
  c17_k_t = (ptrdiff_t)(12);
  c17_lda_t = (ptrdiff_t)(23);
  c17_ldb_t = (ptrdiff_t)(12);
  c17_ldc_t = (ptrdiff_t)(23);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_db_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[529], real_T c17_B[276], real_T c17_C[276])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(23);
  c17_n_t = (ptrdiff_t)(12);
  c17_k_t = (ptrdiff_t)(23);
  c17_lda_t = (ptrdiff_t)(23);
  c17_ldb_t = (ptrdiff_t)(23);
  c17_ldc_t = (ptrdiff_t)(23);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void c17_eb_eml_xgemm(SFc17_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c17_A[456], real_T c17_B[144], real_T c17_C[456])
{
  real_T c17_alpha1;
  real_T c17_beta1;
  char_T c17_TRANSB;
  char_T c17_TRANSA;
  ptrdiff_t c17_m_t;
  ptrdiff_t c17_n_t;
  ptrdiff_t c17_k_t;
  ptrdiff_t c17_lda_t;
  ptrdiff_t c17_ldb_t;
  ptrdiff_t c17_ldc_t;
  double * c17_alpha1_t;
  double * c17_Aia0_t;
  double * c17_Bib0_t;
  double * c17_beta1_t;
  double * c17_Cic0_t;
  c17_alpha1 = 1.0;
  c17_beta1 = 0.0;
  c17_TRANSB = 'N';
  c17_TRANSA = 'N';
  c17_m_t = (ptrdiff_t)(38);
  c17_n_t = (ptrdiff_t)(12);
  c17_k_t = (ptrdiff_t)(12);
  c17_lda_t = (ptrdiff_t)(38);
  c17_ldb_t = (ptrdiff_t)(12);
  c17_ldc_t = (ptrdiff_t)(38);
  c17_alpha1_t = (double *)(&c17_alpha1);
  c17_Aia0_t = (double *)(&c17_A[0]);
  c17_Bib0_t = (double *)(&c17_B[0]);
  c17_beta1_t = (double *)(&c17_beta1);
  c17_Cic0_t = (double *)(&c17_C[0]);
  dgemm(&c17_TRANSA, &c17_TRANSB, &c17_m_t, &c17_n_t, &c17_k_t, c17_alpha1_t,
        c17_Aia0_t, &c17_lda_t, c17_Bib0_t, &c17_ldb_t, c17_beta1_t, c17_Cic0_t,
        &c17_ldc_t);
}

static void init_dsm_address_info(SFc17_torqueBalancing2012bInstanceStruct
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

void sf_c17_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3389361944U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3873750766U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3610780448U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1374827753U);
}

mxArray *sf_c17_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("GllqcgkcvOjLpr8WJJ6qTF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,30,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(23);
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
      pr[0] = (double)(19);
      pr[1] = (double)(6);
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
      pr[0] = (double)(19);
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
      pr[0] = (double)(23);
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
      pr[0] = (double)(29);
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
      pr[0] = (double)(29);
      pr[1] = (double)(29);
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
      pr[0] = (double)(29);
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
      pr[0] = (double)(3);
      pr[1] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(29);
      mxSetField(mxData,13,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,13,"type",mxType);
    }

    mxSetField(mxData,13,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(29);
      mxSetField(mxData,14,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,14,"type",mxType);
    }

    mxSetField(mxData,14,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,15,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,15,"type",mxType);
    }

    mxSetField(mxData,15,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,16,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,16,"type",mxType);
    }

    mxSetField(mxData,16,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,17,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,17,"type",mxType);
    }

    mxSetField(mxData,17,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(29);
      mxSetField(mxData,18,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,18,"type",mxType);
    }

    mxSetField(mxData,18,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,19,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,19,"type",mxType);
    }

    mxSetField(mxData,19,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,20,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,20,"type",mxType);
    }

    mxSetField(mxData,20,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,21,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,21,"type",mxType);
    }

    mxSetField(mxData,21,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,22,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,22,"type",mxType);
    }

    mxSetField(mxData,22,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,23,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,23,"type",mxType);
    }

    mxSetField(mxData,23,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,24,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,24,"type",mxType);
    }

    mxSetField(mxData,24,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,25,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,25,"type",mxType);
    }

    mxSetField(mxData,25,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,26,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,26,"type",mxType);
    }

    mxSetField(mxData,26,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,27,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,27,"type",mxType);
    }

    mxSetField(mxData,27,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,28,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,28,"type",mxType);
    }

    mxSetField(mxData,28,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,29,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,29,"type",mxType);
    }

    mxSetField(mxData,29,"complexity",mxCreateDoubleScalar(0));
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

    mxArray *mxData = mxCreateStructMatrix(1,16,3,dataFields);

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
      pr[0] = (double)(23);
      pr[1] = (double)(12);
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
      pr[0] = (double)(12);
      pr[1] = (double)(12);
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
      pr[0] = (double)(12);
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
      pr[0] = (double)(6);
      pr[1] = (double)(6);
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
      pr[0] = (double)(6);
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
      pr[0] = (double)(19);
      pr[1] = (double)(6);
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
      pr[0] = (double)(19);
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
      pr[0] = (double)(12);
      pr[1] = (double)(12);
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
      pr[0] = (double)(12);
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
      pr[0] = (double)(38);
      pr[1] = (double)(12);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(38);
      pr[1] = (double)(1);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,13,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,13,"type",mxType);
    }

    mxSetField(mxData,13,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
      pr[1] = (double)(1);
      mxSetField(mxData,14,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,14,"type",mxType);
    }

    mxSetField(mxData,14,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,15,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,15,"type",mxType);
    }

    mxSetField(mxData,15,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c17_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c17_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[69],T\"ConstraintsMatrixQP1Foot\",},{M[1],M[62],T\"ConstraintsMatrixQP2FeetOrLegs\",},{M[1],M[5],T\"HessianMatrixQP1Foot\",},{M[1],M[53],T\"HessianMatrixQP2FeetOrLegs\",},{M[1],M[66],T\"NA\",},{M[1],M[58],T\"Sigma\",},{M[1],M[70],T\"bVectorConstraintsQp1Foot\",},{M[1],M[63],T\"bVectorConstraintsQp2FeetOrLegs\",},{M[1],M[98],T\"correctionFromSupportForce\",},{M[1],M[48],T\"errorCoM\",}}",
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[80],T\"f\",},{M[1],M[64],T\"fHdotDesC1C2\",},{M[1],M[52],T\"gradientQP1Foot\",},{M[1],M[54],T\"gradientQP2FeetOrLegs\",},{M[1],M[49],T\"qTilde\",},{M[1],M[57],T\"tauModel\",},{M[8],M[0],T\"is_active_c17_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 17, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c17_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           17,
           1,
           1,
           48,
           0,
           0,
           0,
           0,
           3,
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
          _SFD_SET_DATA_PROPS(0,2,0,1,"tauModel");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Sigma");
          _SFD_SET_DATA_PROPS(2,2,0,1,"NA");
          _SFD_SET_DATA_PROPS(3,2,0,1,"fHdotDesC1C2");
          _SFD_SET_DATA_PROPS(4,2,0,1,"HessianMatrixQP1Foot");
          _SFD_SET_DATA_PROPS(5,2,0,1,"gradientQP1Foot");
          _SFD_SET_DATA_PROPS(6,2,0,1,"ConstraintsMatrixQP1Foot");
          _SFD_SET_DATA_PROPS(7,2,0,1,"bVectorConstraintsQp1Foot");
          _SFD_SET_DATA_PROPS(8,2,0,1,"HessianMatrixQP2FeetOrLegs");
          _SFD_SET_DATA_PROPS(9,2,0,1,"gradientQP2FeetOrLegs");
          _SFD_SET_DATA_PROPS(10,2,0,1,"ConstraintsMatrixQP2FeetOrLegs");
          _SFD_SET_DATA_PROPS(11,2,0,1,"bVectorConstraintsQp2FeetOrLegs");
          _SFD_SET_DATA_PROPS(12,1,1,0,"constraints");
          _SFD_SET_DATA_PROPS(13,1,1,0,"ROBOT_DOF_FOR_SIMULINK");
          _SFD_SET_DATA_PROPS(14,1,1,0,"ConstraintsMatrix");
          _SFD_SET_DATA_PROPS(15,1,1,0,"bVectorConstraints");
          _SFD_SET_DATA_PROPS(16,1,1,0,"q");
          _SFD_SET_DATA_PROPS(17,1,1,0,"qDes");
          _SFD_SET_DATA_PROPS(18,1,1,0,"v");
          _SFD_SET_DATA_PROPS(19,1,1,0,"M");
          _SFD_SET_DATA_PROPS(20,1,1,0,"h");
          _SFD_SET_DATA_PROPS(21,1,1,0,"H");
          _SFD_SET_DATA_PROPS(22,1,1,0,"intHw");
          _SFD_SET_DATA_PROPS(23,1,1,0,"w_H_l_contact");
          _SFD_SET_DATA_PROPS(24,1,1,0,"w_H_r_contact");
          _SFD_SET_DATA_PROPS(25,1,1,0,"JL");
          _SFD_SET_DATA_PROPS(26,1,1,0,"JR");
          _SFD_SET_DATA_PROPS(27,1,1,0,"dJLv");
          _SFD_SET_DATA_PROPS(28,1,1,0,"dJRv");
          _SFD_SET_DATA_PROPS(29,1,1,0,"xcom");
          _SFD_SET_DATA_PROPS(30,1,1,0,"J_CoM");
          _SFD_SET_DATA_PROPS(31,1,1,0,"desired_x_dx_ddx_CoM");
          _SFD_SET_DATA_PROPS(32,1,1,0,"gainsPCOM");
          _SFD_SET_DATA_PROPS(33,1,1,0,"gainsDCOM");
          _SFD_SET_DATA_PROPS(34,1,1,0,"impedances");
          _SFD_SET_DATA_PROPS(35,1,1,0,"intErrorCoM");
          _SFD_SET_DATA_PROPS(36,1,1,0,"ki_int_qtilde");
          _SFD_SET_DATA_PROPS(37,10,0,0,"reg");
          _SFD_SET_DATA_PROPS(38,10,0,0,"gain");
          _SFD_SET_DATA_PROPS(39,2,0,1,"errorCoM");
          _SFD_SET_DATA_PROPS(40,2,0,1,"qTilde");
          _SFD_SET_DATA_PROPS(41,2,0,1,"f");
          _SFD_SET_DATA_PROPS(42,1,1,0,"w_H_lArm");
          _SFD_SET_DATA_PROPS(43,1,1,0,"w_H_rArm");
          _SFD_SET_DATA_PROPS(44,1,1,0,"LArmWrench");
          _SFD_SET_DATA_PROPS(45,1,1,0,"RArmWrench");
          _SFD_SET_DATA_PROPS(46,1,1,0,"useExtArmForces");
          _SFD_SET_DATA_PROPS(47,2,0,1,"correctionFromSupportForce");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1442);
        _SFD_CV_INIT_SCRIPT(0,1,1,0,0,0,0,0,2,1);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"balancingController",788,-1,12027);
        _SFD_CV_INIT_SCRIPT_IF(0,0,5388,5429,5503,5564);

        {
          static int condStart[] = { 5392, 5418 };

          static int condEnd[] = { 5412, 5428 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_SCRIPT_MCDC(0,0,5391,5429,2,0,&(condStart[0]),&(condEnd[0]),
            3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"Sf",11,-1,135);
        _SFD_CV_INIT_SCRIPT(2,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"pinvDamped",0,-1,271);
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
            1.0,0,0,(MexFcnForType)c17_c_sf_marshallOut,(MexInFcnForType)
            c17_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 23;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_k_sf_marshallOut,(MexInFcnForType)
            c17_k_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_g_sf_marshallOut,(MexInFcnForType)
            c17_g_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_b_sf_marshallOut,(MexInFcnForType)
            c17_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_j_sf_marshallOut,(MexInFcnForType)
            c17_j_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_sf_marshallOut,(MexInFcnForType)
            c17_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 19;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_i_sf_marshallOut,(MexInFcnForType)
            c17_i_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 19;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_h_sf_marshallOut,(MexInFcnForType)
            c17_h_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_g_sf_marshallOut,(MexInFcnForType)
            c17_g_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_b_sf_marshallOut,(MexInFcnForType)
            c17_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 38;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_f_sf_marshallOut,(MexInFcnForType)
            c17_f_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 38;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_e_sf_marshallOut,(MexInFcnForType)
            c17_e_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_v_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 23;
          dimVector[1]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_u_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 19;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_i_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 19;
          _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_h_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_s_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 29;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_t_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_s_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(21,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(22,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(23,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_m_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(24,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_m_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(25,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_q_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(26,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_q_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(27,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(28,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 1;
          _SFD_SET_DATA_COMPILED_PROPS(29,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_r_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(30,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_q_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(31,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_p_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(32,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(33,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(34,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(35,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(36,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(37,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c17_o_sf_marshallOut,(MexInFcnForType)
          c17_m_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(38,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c17_n_sf_marshallOut,(MexInFcnForType)
          c17_l_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(39,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_d_sf_marshallOut,(MexInFcnForType)
            c17_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(40,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_c_sf_marshallOut,(MexInFcnForType)
            c17_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(41,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_b_sf_marshallOut,(MexInFcnForType)
            c17_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(42,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_m_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(43,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_m_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(44,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(45,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(46,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c17_l_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(47,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c17_sf_marshallOut,(MexInFcnForType)
            c17_sf_marshallIn);
        }

        {
          boolean_T *c17_useExtArmForces;
          real_T (*c17_tauModel)[23];
          real_T (*c17_Sigma)[276];
          real_T (*c17_NA)[144];
          real_T (*c17_fHdotDesC1C2)[12];
          real_T (*c17_HessianMatrixQP1Foot)[36];
          real_T (*c17_gradientQP1Foot)[6];
          real_T (*c17_ConstraintsMatrixQP1Foot)[114];
          real_T (*c17_bVectorConstraintsQp1Foot)[19];
          real_T (*c17_HessianMatrixQP2FeetOrLegs)[144];
          real_T (*c17_gradientQP2FeetOrLegs)[12];
          real_T (*c17_ConstraintsMatrixQP2FeetOrLegs)[456];
          real_T (*c17_bVectorConstraintsQp2FeetOrLegs)[38];
          real_T (*c17_constraints)[2];
          real_T (*c17_ROBOT_DOF_FOR_SIMULINK)[529];
          real_T (*c17_ConstraintsMatrix)[114];
          real_T (*c17_bVectorConstraints)[19];
          real_T (*c17_q)[23];
          real_T (*c17_qDes)[23];
          real_T (*c17_v)[29];
          real_T (*c17_M)[841];
          real_T (*c17_h)[29];
          real_T (*c17_H)[6];
          real_T (*c17_intHw)[3];
          real_T (*c17_w_H_l_contact)[16];
          real_T (*c17_w_H_r_contact)[16];
          real_T (*c17_JL)[174];
          real_T (*c17_JR)[174];
          real_T (*c17_dJLv)[6];
          real_T (*c17_dJRv)[6];
          real_T (*c17_xcom)[3];
          real_T (*c17_J_CoM)[174];
          real_T (*c17_desired_x_dx_ddx_CoM)[9];
          real_T (*c17_gainsPCOM)[3];
          real_T (*c17_gainsDCOM)[3];
          real_T (*c17_impedances)[23];
          real_T (*c17_intErrorCoM)[3];
          real_T (*c17_ki_int_qtilde)[23];
          real_T (*c17_errorCoM)[3];
          real_T (*c17_qTilde)[23];
          real_T (*c17_f)[12];
          real_T (*c17_w_H_lArm)[16];
          real_T (*c17_w_H_rArm)[16];
          real_T (*c17_LArmWrench)[6];
          real_T (*c17_RArmWrench)[6];
          real_T (*c17_correctionFromSupportForce)[6];
          c17_correctionFromSupportForce = (real_T (*)[6])ssGetOutputPortSignal
            (chartInstance->S, 16);
          c17_useExtArmForces = (boolean_T *)ssGetInputPortSignal
            (chartInstance->S, 29);
          c17_RArmWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
            28);
          c17_LArmWrench = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
            27);
          c17_w_H_rArm = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            26);
          c17_w_H_lArm = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            25);
          c17_f = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 15);
          c17_qTilde = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S,
            14);
          c17_errorCoM = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S,
            13);
          c17_ki_int_qtilde = (real_T (*)[23])ssGetInputPortSignal
            (chartInstance->S, 24);
          c17_intErrorCoM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            23);
          c17_impedances = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S,
            22);
          c17_gainsDCOM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            21);
          c17_gainsPCOM = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            20);
          c17_desired_x_dx_ddx_CoM = (real_T (*)[9])ssGetInputPortSignal
            (chartInstance->S, 19);
          c17_J_CoM = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 18);
          c17_xcom = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 17);
          c17_dJRv = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 16);
          c17_dJLv = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 15);
          c17_JR = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 14);
          c17_JL = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 13);
          c17_w_H_r_contact = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 12);
          c17_w_H_l_contact = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 11);
          c17_intHw = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 10);
          c17_H = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 9);
          c17_h = (real_T (*)[29])ssGetInputPortSignal(chartInstance->S, 8);
          c17_M = (real_T (*)[841])ssGetInputPortSignal(chartInstance->S, 7);
          c17_v = (real_T (*)[29])ssGetInputPortSignal(chartInstance->S, 6);
          c17_qDes = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 5);
          c17_q = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 4);
          c17_bVectorConstraints = (real_T (*)[19])ssGetInputPortSignal
            (chartInstance->S, 3);
          c17_ConstraintsMatrix = (real_T (*)[114])ssGetInputPortSignal
            (chartInstance->S, 2);
          c17_ROBOT_DOF_FOR_SIMULINK = (real_T (*)[529])ssGetInputPortSignal
            (chartInstance->S, 1);
          c17_constraints = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S,
            0);
          c17_bVectorConstraintsQp2FeetOrLegs = (real_T (*)[38])
            ssGetOutputPortSignal(chartInstance->S, 12);
          c17_ConstraintsMatrixQP2FeetOrLegs = (real_T (*)[456])
            ssGetOutputPortSignal(chartInstance->S, 11);
          c17_gradientQP2FeetOrLegs = (real_T (*)[12])ssGetOutputPortSignal
            (chartInstance->S, 10);
          c17_HessianMatrixQP2FeetOrLegs = (real_T (*)[144])
            ssGetOutputPortSignal(chartInstance->S, 9);
          c17_bVectorConstraintsQp1Foot = (real_T (*)[19])ssGetOutputPortSignal
            (chartInstance->S, 8);
          c17_ConstraintsMatrixQP1Foot = (real_T (*)[114])ssGetOutputPortSignal
            (chartInstance->S, 7);
          c17_gradientQP1Foot = (real_T (*)[6])ssGetOutputPortSignal
            (chartInstance->S, 6);
          c17_HessianMatrixQP1Foot = (real_T (*)[36])ssGetOutputPortSignal
            (chartInstance->S, 5);
          c17_fHdotDesC1C2 = (real_T (*)[12])ssGetOutputPortSignal
            (chartInstance->S, 4);
          c17_NA = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 3);
          c17_Sigma = (real_T (*)[276])ssGetOutputPortSignal(chartInstance->S, 2);
          c17_tauModel = (real_T (*)[23])ssGetOutputPortSignal(chartInstance->S,
            1);
          _SFD_SET_DATA_VALUE_PTR(0U, *c17_tauModel);
          _SFD_SET_DATA_VALUE_PTR(1U, *c17_Sigma);
          _SFD_SET_DATA_VALUE_PTR(2U, *c17_NA);
          _SFD_SET_DATA_VALUE_PTR(3U, *c17_fHdotDesC1C2);
          _SFD_SET_DATA_VALUE_PTR(4U, *c17_HessianMatrixQP1Foot);
          _SFD_SET_DATA_VALUE_PTR(5U, *c17_gradientQP1Foot);
          _SFD_SET_DATA_VALUE_PTR(6U, *c17_ConstraintsMatrixQP1Foot);
          _SFD_SET_DATA_VALUE_PTR(7U, *c17_bVectorConstraintsQp1Foot);
          _SFD_SET_DATA_VALUE_PTR(8U, *c17_HessianMatrixQP2FeetOrLegs);
          _SFD_SET_DATA_VALUE_PTR(9U, *c17_gradientQP2FeetOrLegs);
          _SFD_SET_DATA_VALUE_PTR(10U, *c17_ConstraintsMatrixQP2FeetOrLegs);
          _SFD_SET_DATA_VALUE_PTR(11U, *c17_bVectorConstraintsQp2FeetOrLegs);
          _SFD_SET_DATA_VALUE_PTR(12U, *c17_constraints);
          _SFD_SET_DATA_VALUE_PTR(13U, *c17_ROBOT_DOF_FOR_SIMULINK);
          _SFD_SET_DATA_VALUE_PTR(14U, *c17_ConstraintsMatrix);
          _SFD_SET_DATA_VALUE_PTR(15U, *c17_bVectorConstraints);
          _SFD_SET_DATA_VALUE_PTR(16U, *c17_q);
          _SFD_SET_DATA_VALUE_PTR(17U, *c17_qDes);
          _SFD_SET_DATA_VALUE_PTR(18U, *c17_v);
          _SFD_SET_DATA_VALUE_PTR(19U, *c17_M);
          _SFD_SET_DATA_VALUE_PTR(20U, *c17_h);
          _SFD_SET_DATA_VALUE_PTR(21U, *c17_H);
          _SFD_SET_DATA_VALUE_PTR(22U, *c17_intHw);
          _SFD_SET_DATA_VALUE_PTR(23U, *c17_w_H_l_contact);
          _SFD_SET_DATA_VALUE_PTR(24U, *c17_w_H_r_contact);
          _SFD_SET_DATA_VALUE_PTR(25U, *c17_JL);
          _SFD_SET_DATA_VALUE_PTR(26U, *c17_JR);
          _SFD_SET_DATA_VALUE_PTR(27U, *c17_dJLv);
          _SFD_SET_DATA_VALUE_PTR(28U, *c17_dJRv);
          _SFD_SET_DATA_VALUE_PTR(29U, *c17_xcom);
          _SFD_SET_DATA_VALUE_PTR(30U, *c17_J_CoM);
          _SFD_SET_DATA_VALUE_PTR(31U, *c17_desired_x_dx_ddx_CoM);
          _SFD_SET_DATA_VALUE_PTR(32U, *c17_gainsPCOM);
          _SFD_SET_DATA_VALUE_PTR(33U, *c17_gainsDCOM);
          _SFD_SET_DATA_VALUE_PTR(34U, *c17_impedances);
          _SFD_SET_DATA_VALUE_PTR(35U, *c17_intErrorCoM);
          _SFD_SET_DATA_VALUE_PTR(36U, *c17_ki_int_qtilde);
          _SFD_SET_DATA_VALUE_PTR(37U, &chartInstance->c17_reg);
          _SFD_SET_DATA_VALUE_PTR(38U, &chartInstance->c17_gain);
          _SFD_SET_DATA_VALUE_PTR(39U, *c17_errorCoM);
          _SFD_SET_DATA_VALUE_PTR(40U, *c17_qTilde);
          _SFD_SET_DATA_VALUE_PTR(41U, *c17_f);
          _SFD_SET_DATA_VALUE_PTR(42U, *c17_w_H_lArm);
          _SFD_SET_DATA_VALUE_PTR(43U, *c17_w_H_rArm);
          _SFD_SET_DATA_VALUE_PTR(44U, *c17_LArmWrench);
          _SFD_SET_DATA_VALUE_PTR(45U, *c17_RArmWrench);
          _SFD_SET_DATA_VALUE_PTR(46U, c17_useExtArmForces);
          _SFD_SET_DATA_VALUE_PTR(47U, *c17_correctionFromSupportForce);
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
  return "6nv4CdtaLrs5DniBIz9ceC";
}

static void sf_opaque_initialize_c17_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc17_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c17_torqueBalancing2012b
    ((SFc17_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c17_torqueBalancing2012b((SFc17_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c17_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c17_torqueBalancing2012b((SFc17_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c17_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c17_torqueBalancing2012b((SFc17_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c17_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c17_torqueBalancing2012b((SFc17_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c17_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c17_torqueBalancing2012b
    ((SFc17_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c17_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c17_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c17_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c17_torqueBalancing2012b
    ((SFc17_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c17_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c17_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c17_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c17_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c17_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc17_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c17_torqueBalancing2012b((SFc17_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc17_torqueBalancing2012b
    ((SFc17_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c17_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c17_torqueBalancing2012b
      ((SFc17_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c17_torqueBalancing2012b(SimStruct *S)
{
  /* Actual parameters from chart:
     gain reg
   */
  const char_T *rtParamNames[] = { "gain", "reg" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0],
    sf_get_param_data_type_id(S,0));
  ssRegDlgParamAsRunTimeParam(S, 1, 1, rtParamNames[1],
    sf_get_param_data_type_id(S,1));
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      17);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,17,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,17,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,17);
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
      ssSetInputPortOptimOpts(S, 11, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 12, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 13, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 14, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 15, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 16, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 17, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 18, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 19, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 20, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 21, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 22, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 23, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 24, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 25, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 26, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 27, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 28, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 29, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,17,30);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,17,16);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=16; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 30; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,17);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(263371942U));
  ssSetChecksum1(S,(3282022333U));
  ssSetChecksum2(S,(113007278U));
  ssSetChecksum3(S,(3824697968U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c17_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c17_torqueBalancing2012b(SimStruct *S)
{
  SFc17_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc17_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc17_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc17_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c17_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c17_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c17_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c17_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c17_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c17_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c17_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c17_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c17_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c17_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c17_torqueBalancing2012b;
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

void c17_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c17_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c17_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c17_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c17_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
