/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c2_torqueBalancing2012b.h"
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
static const char * c2_debug_family_names[10] = { "nargin", "nargout",
  "imu_H_link", "imu_H_link_0", "link_H_root", "inertial_0", "inertial", "neck",
  "CONFIG", "w_H_root" };

static const char * c2_b_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c2_c_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c2_d_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c2_e_debug_family_names[4] = { "nargin", "nargout", "R",
  "rollPitchYaw" };

static const char * c2_f_debug_family_names[7] = { "nargin", "nargout", "a", "d",
  "alph", "thet", "G" };

static const char * c2_g_debug_family_names[10] = { "G_34", "G_45", "G_56",
  "G_6I", "imuAssumingNeckToZero_H_neckBase", "imu_H_neckBase", "nargin",
  "nargout", "neckJoints", "imu_H_imuAssumingNeckToZero" };

static const char * c2_h_debug_family_names[23] = { "wImu_R_imu", "wImu_R_imu_0",
  "imu_R_link", "imu_R_link_0", "wImu_R_link_0", "rollPitchYaw_link_0",
  "rollPitchYaw_link", "rollPitchYawFiltered_link", "wImu_H_link",
  "wImu_H_link_0", "wImu_H_root", "wImu_H_wImuAssumingNeckToZero", "wImu_R_link",
  "nargin", "nargout", "imu_H_link", "imu_H_link_0", "link_H_root", "inertial_0",
  "inertial", "neck", "CONFIG", "w_H_root" };

/* Function Declarations */
static void initialize_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c2_st);
static void finalize_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c2_torqueBalancing2012b(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void c2_fromBaseToWorldWithImu(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_imu_H_link[16], real_T c2_imu_H_link_0[16], real_T
  c2_link_H_root[16], real_T c2_inertial_0[12], real_T c2_inertial[12], real_T
  c2_neck[3], c2_struct_HZPQIAn1UwxVqJtlH5llbD *c2_b_CONFIG, real_T c2_w_H_root
  [16]);
static void c2_rotz(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c2_alpha, real_T c2_R[9]);
static void c2_roty(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c2_alpha, real_T c2_R[9]);
static void c2_rotx(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c2_alpha, real_T c2_R[9]);
static void c2_rollPitchYawFromRotation(SFc2_torqueBalancing2012bInstanceStruct *
  chartInstance, real_T c2_R[9], real_T c2_rollPitchYaw[3]);
static void c2_correctIMU(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c2_neckJoints[3], real_T c2_imu_H_imuAssumingNeckToZero[16]);
static void c2_evalDHMatrix(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_a, real_T c2_d, real_T c2_alph, real_T c2_thet,
  real_T c2_G[16]);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_w_H_root, const char_T *c2_identifier,
  real_T c2_y[16]);
static void c2_b_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  c2_struct_HZPQIAn1UwxVqJtlH5llbD *c2_y);
static real_T c2_d_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static c2_struct_amzdx4J7qaaMzgPI7fQ3WD c2_e_emlrt_marshallIn
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c2_u,
   const emlrtMsgIdentifier *c2_parentId);
static boolean_T c2_f_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[2]);
static void c2_h_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[23]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_i_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_j_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_k_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[12]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[147]);
static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[147]);
static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[147]);
static void c2_eml_scalar_eg(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c2_eml_error(SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static real_T c2_atan2(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c2_y, real_T c2_x);
static void c2_b_eml_scalar_eg(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c2_c_eml_scalar_eg(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c2_realmin(SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void c2_eps(SFc2_torqueBalancing2012bInstanceStruct *chartInstance);
static void c2_eml_matlab_zgetrf(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_b_A[16], int32_T c2_ipiv[4],
  int32_T *c2_info);
static void c2_check_forloop_overflow_error
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c2_overflow);
static void c2_eml_xger(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0,
  real_T c2_A[16], int32_T c2_ia0, real_T c2_b_A[16]);
static void c2_eml_xtrsm(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c2_A[16], real_T c2_B[16], real_T c2_b_B[16]);
static void c2_below_threshold(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c2_d_eml_scalar_eg(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c2_b_eml_xtrsm(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16], real_T c2_b_B[16]);
static void c2_eml_warning(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_l_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_m_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_torqueBalancing2012b, const
  char_T *c2_identifier);
static uint8_T c2_n_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_eml_matlab_zgetrf(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], int32_T c2_ipiv[4], int32_T *c2_info);
static void c2_b_eml_xger(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0,
  real_T c2_A[16], int32_T c2_ia0);
static void c2_c_eml_xtrsm(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16]);
static void c2_d_eml_xtrsm(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16]);
static void init_dsm_address_info(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_torqueBalancing2012b = 0U;
}

static void initialize_params_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c2_m0 = NULL;
  const mxArray *c2_mxField;
  c2_struct_HZPQIAn1UwxVqJtlH5llbD c2_r0;
  const mxArray *c2_m1 = NULL;
  const mxArray *c2_b_mxField;
  sf_set_error_prefix_string(
    "Error evaluating data 'CONFIG' in the parent workspace.\n");
  c2_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c2_mxField = sf_mex_getfield(c2_m0, "SIMULATION_TIME", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.SIMULATION_TIME,
                      1, 0, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "SCOPES", "CONFIG", 0);
  c2_m1 = sf_mex_dup(c2_mxField);
  c2_b_mxField = sf_mex_getfield(c2_m1, "ALL", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_b_mxField), &c2_r0.SCOPES.ALL, 1,
                      11, 0U, 0, 0U, 0);
  c2_b_mxField = sf_mex_getfield(c2_m1, "BASE_EST_IMU", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_b_mxField),
                      &c2_r0.SCOPES.BASE_EST_IMU, 1, 11, 0U, 0, 0U, 0);
  c2_b_mxField = sf_mex_getfield(c2_m1, "EXTWRENCHES", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_b_mxField),
                      &c2_r0.SCOPES.EXTWRENCHES, 1, 11, 0U, 0, 0U, 0);
  c2_b_mxField = sf_mex_getfield(c2_m1, "GAIN_SCHE_INFO", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_b_mxField),
                      &c2_r0.SCOPES.GAIN_SCHE_INFO, 1, 11, 0U, 0, 0U, 0);
  c2_b_mxField = sf_mex_getfield(c2_m1, "MAIN", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_b_mxField), &c2_r0.SCOPES.MAIN, 1,
                      11, 0U, 0, 0U, 0);
  c2_b_mxField = sf_mex_getfield(c2_m1, "QP", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_b_mxField), &c2_r0.SCOPES.QP, 1,
                      11, 0U, 0, 0U, 0);
  sf_mex_destroy(&c2_m1);
  c2_mxField = sf_mex_getfield(c2_m0, "CHECK_LIMITS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.CHECK_LIMITS, 1,
                      11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "USE_IMU4EST_BASE", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.USE_IMU4EST_BASE,
                      1, 11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "YAW_IMU_FILTER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.YAW_IMU_FILTER, 1,
                      11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "PITCH_IMU_FILTER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.PITCH_IMU_FILTER,
                      1, 11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "CORRECT_NECK_IMU", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.CORRECT_NECK_IMU,
                      1, 11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "ONSOFTCARPET", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.ONSOFTCARPET, 1,
                      11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "USE_QP_SOLVER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.USE_QP_SOLVER, 1,
                      11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "Ts", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.Ts, 1, 0, 0U, 0,
                      0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "ON_GAZEBO", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.ON_GAZEBO, 1, 11,
                      0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "LEFT_RIGHT_FOOT_IN_CONTACT", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField),
                      c2_r0.LEFT_RIGHT_FOOT_IN_CONTACT, 1, 0, 0U, 1, 0U, 2, 1, 2);
  c2_mxField = sf_mex_getfield(c2_m0, "SMOOTH_DES_COM", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.SMOOTH_DES_COM, 1,
                      0, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "SMOOTH_DES_Q", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.SMOOTH_DES_Q, 1,
                      0, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "smoothingTimeTranDynamics", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField),
                      &c2_r0.smoothingTimeTranDynamics, 1, 0, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "DEMO_MOVEMENTS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.DEMO_MOVEMENTS, 1,
                      11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "TIME_CONTROLLER_SWITCH", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField),
                      &c2_r0.TIME_CONTROLLER_SWITCH, 1, 0, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "JOINTS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), c2_r0.JOINTS, 1, 0, 0U,
                      1, 0U, 2, 23, 1);
  c2_mxField = sf_mex_getfield(c2_m0, "JOINTSITING", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), c2_r0.JOINTSITING, 1, 0,
                      0U, 1, 0U, 2, 23, 1);
  c2_mxField = sf_mex_getfield(c2_m0, "iCubStandUp", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.iCubStandUp, 1,
                      11, 0U, 0, 0U, 0);
  c2_mxField = sf_mex_getfield(c2_m0, "useExtArmForces", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c2_mxField), &c2_r0.useExtArmForces,
                      1, 11, 0U, 0, 0U, 0);
  sf_mex_destroy(&c2_m0);
  chartInstance->c2_CONFIG = c2_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[16];
  const mxArray *c2_b_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T (*c2_w_H_root)[16];
  c2_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2), FALSE);
  for (c2_i0 = 0; c2_i0 < 16; c2_i0++) {
    c2_u[c2_i0] = (*c2_w_H_root)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = chartInstance->c2_is_active_c2_torqueBalancing2012b;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[16];
  int32_T c2_i1;
  real_T (*c2_w_H_root)[16];
  c2_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                      "w_H_root", c2_dv0);
  for (c2_i1 = 0; c2_i1 < 16; c2_i1++) {
    (*c2_w_H_root)[c2_i1] = c2_dv0[c2_i1];
  }

  chartInstance->c2_is_active_c2_torqueBalancing2012b = c2_m_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
     "is_active_c2_torqueBalancing2012b");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c2_torqueBalancing2012b(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  int32_T c2_i9;
  real_T c2_imu_H_link[16];
  int32_T c2_i10;
  real_T c2_imu_H_link_0[16];
  int32_T c2_i11;
  real_T c2_link_H_root[16];
  int32_T c2_i12;
  real_T c2_inertial_0[12];
  int32_T c2_i13;
  real_T c2_inertial[12];
  int32_T c2_i14;
  real_T c2_neck[3];
  c2_struct_HZPQIAn1UwxVqJtlH5llbD c2_b_CONFIG;
  uint32_T c2_debug_family_var_map[10];
  real_T c2_nargin = 7.0;
  real_T c2_nargout = 1.0;
  real_T c2_w_H_root[16];
  int32_T c2_i15;
  real_T c2_b_imu_H_link[16];
  int32_T c2_i16;
  real_T c2_b_imu_H_link_0[16];
  int32_T c2_i17;
  real_T c2_b_link_H_root[16];
  int32_T c2_i18;
  real_T c2_b_inertial_0[12];
  int32_T c2_i19;
  real_T c2_b_inertial[12];
  int32_T c2_i20;
  real_T c2_b_neck[3];
  c2_struct_HZPQIAn1UwxVqJtlH5llbD c2_c_CONFIG;
  real_T c2_dv1[16];
  int32_T c2_i21;
  int32_T c2_i22;
  real_T (*c2_b_w_H_root)[16];
  real_T (*c2_c_neck)[3];
  real_T (*c2_c_inertial)[12];
  real_T (*c2_c_inertial_0)[12];
  real_T (*c2_c_link_H_root)[16];
  real_T (*c2_c_imu_H_link_0)[16];
  real_T (*c2_c_imu_H_link)[16];
  c2_b_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_c_neck = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c2_c_inertial = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 4);
  c2_c_inertial_0 = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 3);
  c2_c_link_H_root = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 2);
  c2_c_imu_H_link_0 = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 1);
  c2_c_imu_H_link = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i2 = 0; c2_i2 < 16; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*c2_c_imu_H_link)[c2_i2], 0U);
  }

  for (c2_i3 = 0; c2_i3 < 16; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((*c2_c_imu_H_link_0)[c2_i3], 1U);
  }

  for (c2_i4 = 0; c2_i4 < 16; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*c2_c_link_H_root)[c2_i4], 2U);
  }

  for (c2_i5 = 0; c2_i5 < 12; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((*c2_c_inertial_0)[c2_i5], 3U);
  }

  for (c2_i6 = 0; c2_i6 < 12; c2_i6++) {
    _SFD_DATA_RANGE_CHECK((*c2_c_inertial)[c2_i6], 4U);
  }

  for (c2_i7 = 0; c2_i7 < 3; c2_i7++) {
    _SFD_DATA_RANGE_CHECK((*c2_c_neck)[c2_i7], 5U);
  }

  for (c2_i8 = 0; c2_i8 < 16; c2_i8++) {
    _SFD_DATA_RANGE_CHECK((*c2_b_w_H_root)[c2_i8], 6U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i9 = 0; c2_i9 < 16; c2_i9++) {
    c2_imu_H_link[c2_i9] = (*c2_c_imu_H_link)[c2_i9];
  }

  for (c2_i10 = 0; c2_i10 < 16; c2_i10++) {
    c2_imu_H_link_0[c2_i10] = (*c2_c_imu_H_link_0)[c2_i10];
  }

  for (c2_i11 = 0; c2_i11 < 16; c2_i11++) {
    c2_link_H_root[c2_i11] = (*c2_c_link_H_root)[c2_i11];
  }

  for (c2_i12 = 0; c2_i12 < 12; c2_i12++) {
    c2_inertial_0[c2_i12] = (*c2_c_inertial_0)[c2_i12];
  }

  for (c2_i13 = 0; c2_i13 < 12; c2_i13++) {
    c2_inertial[c2_i13] = (*c2_c_inertial)[c2_i13];
  }

  for (c2_i14 = 0; c2_i14 < 3; c2_i14++) {
    c2_neck[c2_i14] = (*c2_c_neck)[c2_i14];
  }

  c2_b_CONFIG = chartInstance->c2_CONFIG;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_imu_H_link, 2U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_imu_H_link_0, 3U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_link_H_root, 4U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_inertial_0, 5U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_inertial, 6U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_neck, 7U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_CONFIG, 8U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_H_root, 9U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 3);
  for (c2_i15 = 0; c2_i15 < 16; c2_i15++) {
    c2_b_imu_H_link[c2_i15] = c2_imu_H_link[c2_i15];
  }

  for (c2_i16 = 0; c2_i16 < 16; c2_i16++) {
    c2_b_imu_H_link_0[c2_i16] = c2_imu_H_link_0[c2_i16];
  }

  for (c2_i17 = 0; c2_i17 < 16; c2_i17++) {
    c2_b_link_H_root[c2_i17] = c2_link_H_root[c2_i17];
  }

  for (c2_i18 = 0; c2_i18 < 12; c2_i18++) {
    c2_b_inertial_0[c2_i18] = c2_inertial_0[c2_i18];
  }

  for (c2_i19 = 0; c2_i19 < 12; c2_i19++) {
    c2_b_inertial[c2_i19] = c2_inertial[c2_i19];
  }

  for (c2_i20 = 0; c2_i20 < 3; c2_i20++) {
    c2_b_neck[c2_i20] = c2_neck[c2_i20];
  }

  c2_c_CONFIG = c2_b_CONFIG;
  c2_fromBaseToWorldWithImu(chartInstance, c2_b_imu_H_link, c2_b_imu_H_link_0,
    c2_b_link_H_root, c2_b_inertial_0, c2_b_inertial, c2_b_neck, &c2_c_CONFIG,
    c2_dv1);
  for (c2_i21 = 0; c2_i21 < 16; c2_i21++) {
    c2_w_H_root[c2_i21] = c2_dv1[c2_i21];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -3);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i22 = 0; c2_i22 < 16; c2_i22++) {
    (*c2_b_w_H_root)[c2_i22] = c2_w_H_root[c2_i22];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc2_torqueBalancing2012b
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c2_fromBaseToWorldWithImu(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_imu_H_link[16], real_T c2_imu_H_link_0[16], real_T
  c2_link_H_root[16], real_T c2_inertial_0[12], real_T c2_inertial[12], real_T
  c2_neck[3], c2_struct_HZPQIAn1UwxVqJtlH5llbD *c2_b_CONFIG, real_T c2_w_H_root
  [16])
{
  uint32_T c2_debug_family_var_map[23];
  real_T c2_wImu_R_imu[9];
  real_T c2_wImu_R_imu_0[9];
  real_T c2_imu_R_link[9];
  real_T c2_imu_R_link_0[9];
  real_T c2_wImu_R_link_0[9];
  real_T c2_rollPitchYaw_link_0[3];
  real_T c2_rollPitchYaw_link[3];
  real_T c2_rollPitchYawFiltered_link[3];
  real_T c2_wImu_H_link[16];
  real_T c2_wImu_H_link_0[16];
  real_T c2_wImu_H_root[16];
  real_T c2_wImu_H_wImuAssumingNeckToZero[16];
  real_T c2_wImu_R_link[9];
  real_T c2_nargin = 7.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i23;
  real_T c2_a[12];
  int32_T c2_i24;
  int32_T c2_i25;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  real_T c2_b_a[9];
  real_T c2_b[9];
  int32_T c2_i29;
  int32_T c2_i30;
  int32_T c2_i31;
  real_T c2_y[9];
  int32_T c2_i32;
  int32_T c2_i33;
  int32_T c2_i34;
  int32_T c2_i35;
  int32_T c2_i36;
  real_T c2_C[9];
  int32_T c2_i37;
  int32_T c2_i38;
  int32_T c2_i39;
  int32_T c2_i40;
  int32_T c2_i41;
  int32_T c2_i42;
  int32_T c2_i43;
  int32_T c2_i44;
  int32_T c2_i45;
  int32_T c2_i46;
  int32_T c2_i47;
  int32_T c2_i48;
  int32_T c2_i49;
  int32_T c2_i50;
  int32_T c2_i51;
  int32_T c2_i52;
  int32_T c2_i53;
  int32_T c2_i54;
  int32_T c2_i55;
  int32_T c2_i56;
  int32_T c2_i57;
  int32_T c2_i58;
  int32_T c2_i59;
  int32_T c2_i60;
  int32_T c2_i61;
  int32_T c2_i62;
  int32_T c2_i63;
  int32_T c2_i64;
  int32_T c2_i65;
  int32_T c2_i66;
  int32_T c2_i67;
  int32_T c2_i68;
  int32_T c2_i69;
  int32_T c2_i70;
  int32_T c2_i71;
  int32_T c2_i72;
  int32_T c2_i73;
  int32_T c2_i74;
  int32_T c2_i75;
  int32_T c2_i76;
  int32_T c2_i77;
  int32_T c2_i78;
  int32_T c2_i79;
  int32_T c2_i80;
  int32_T c2_i81;
  int32_T c2_i82;
  int32_T c2_i83;
  int32_T c2_i84;
  int32_T c2_i85;
  int32_T c2_i86;
  int32_T c2_i87;
  int32_T c2_i88;
  int32_T c2_i89;
  int32_T c2_i90;
  int32_T c2_i91;
  int32_T c2_i92;
  int32_T c2_i93;
  int32_T c2_i94;
  int32_T c2_i95;
  real_T c2_b_wImu_R_link_0[9];
  real_T c2_dv2[3];
  int32_T c2_i96;
  int32_T c2_i97;
  real_T c2_b_wImu_R_link[9];
  real_T c2_dv3[3];
  int32_T c2_i98;
  int32_T c2_i99;
  int32_T c2_i100;
  int32_T c2_i101;
  int32_T c2_i102;
  int32_T c2_i103;
  int32_T c2_i104;
  int32_T c2_i105;
  int32_T c2_i106;
  int32_T c2_i107;
  int32_T c2_i108;
  int32_T c2_i109;
  int32_T c2_i110;
  int32_T c2_i111;
  int32_T c2_i112;
  int32_T c2_i113;
  int32_T c2_i114;
  int32_T c2_i115;
  int32_T c2_i116;
  real_T c2_dv4[3];
  int32_T c2_i117;
  int32_T c2_i118;
  int32_T c2_i119;
  int32_T c2_i120;
  int32_T c2_i121;
  int32_T c2_i122;
  int32_T c2_i123;
  static real_T c2_dv5[4] = { 0.0, 0.0, 0.0, 1.0 };

  int32_T c2_i124;
  int32_T c2_i125;
  int32_T c2_i126;
  int32_T c2_i127;
  int32_T c2_i128;
  int32_T c2_i129;
  int32_T c2_i130;
  int32_T c2_i131;
  int32_T c2_i132;
  real_T c2_c_a[16];
  int32_T c2_i133;
  real_T c2_b_b[16];
  int32_T c2_i134;
  int32_T c2_i135;
  int32_T c2_i136;
  real_T c2_b_C[16];
  int32_T c2_i137;
  int32_T c2_i138;
  int32_T c2_i139;
  int32_T c2_i140;
  int32_T c2_i141;
  int32_T c2_i142;
  int32_T c2_i143;
  int32_T c2_i144;
  int32_T c2_i145;
  real_T c2_b_neck[3];
  real_T c2_dv6[16];
  int32_T c2_i146;
  int32_T c2_i147;
  int32_T c2_i148;
  int32_T c2_i149;
  int32_T c2_i150;
  int32_T c2_i151;
  int32_T c2_i152;
  int32_T c2_i153;
  int32_T c2_i154;
  int32_T c2_i155;
  int32_T c2_i156;
  int32_T c2_i157;
  int32_T c2_i158;
  int32_T c2_i159;
  int32_T c2_i160;
  int32_T c2_i161;
  int32_T c2_info;
  int32_T c2_ipiv[4];
  int32_T c2_b_info;
  int32_T c2_c_info;
  int32_T c2_d_info;
  int32_T c2_i162;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_ip;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_temp;
  int32_T c2_i163;
  real_T c2_dv7[16];
  int32_T c2_i164;
  real_T c2_dv8[16];
  int32_T c2_i165;
  real_T c2_dv9[16];
  int32_T c2_i166;
  real_T c2_dv10[16];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 23U, 23U, c2_h_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_wImu_R_imu, 0U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_wImu_R_imu_0, 1U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_imu_R_link, 2U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_imu_R_link_0, 3U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_wImu_R_link_0, 4U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rollPitchYaw_link_0, 5U,
    c2_c_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rollPitchYaw_link, 6U,
    c2_c_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rollPitchYawFiltered_link, 7U,
    c2_c_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_wImu_H_link, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_wImu_H_link_0, 9U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_wImu_H_root, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_wImu_H_wImuAssumingNeckToZero, 11U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_wImu_R_link, 12U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 13U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 14U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_imu_H_link, 15U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_imu_H_link_0, 16U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_link_H_root, 17U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_inertial_0, 18U, c2_d_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_inertial, 19U, c2_d_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_neck, 20U, c2_c_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_CONFIG, 21U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_H_root, 22U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 5);
  for (c2_i23 = 0; c2_i23 < 12; c2_i23++) {
    c2_a[c2_i23] = c2_inertial[c2_i23];
  }

  for (c2_i24 = 0; c2_i24 < 12; c2_i24++) {
    c2_a[c2_i24] *= 3.1415926535897931;
  }

  for (c2_i25 = 0; c2_i25 < 12; c2_i25++) {
    c2_inertial[c2_i25] = c2_a[c2_i25] / 180.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 6);
  for (c2_i26 = 0; c2_i26 < 12; c2_i26++) {
    c2_a[c2_i26] = c2_inertial_0[c2_i26];
  }

  for (c2_i27 = 0; c2_i27 < 12; c2_i27++) {
    c2_a[c2_i27] *= 3.1415926535897931;
  }

  for (c2_i28 = 0; c2_i28 < 12; c2_i28++) {
    c2_inertial_0[c2_i28] = c2_a[c2_i28] / 180.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 10);
  c2_rotz(chartInstance, c2_inertial[2], c2_b_a);
  c2_roty(chartInstance, c2_inertial[1], c2_b);
  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i29 = 0; c2_i29 < 3; c2_i29++) {
    c2_i30 = 0;
    for (c2_i31 = 0; c2_i31 < 3; c2_i31++) {
      c2_y[c2_i30 + c2_i29] = 0.0;
      c2_i32 = 0;
      for (c2_i33 = 0; c2_i33 < 3; c2_i33++) {
        c2_y[c2_i30 + c2_i29] += c2_b_a[c2_i32 + c2_i29] * c2_b[c2_i33 + c2_i30];
        c2_i32 += 3;
      }

      c2_i30 += 3;
    }
  }

  c2_rotx(chartInstance, c2_inertial[0], c2_b);
  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i34 = 0; c2_i34 < 9; c2_i34++) {
    c2_wImu_R_imu[c2_i34] = 0.0;
  }

  for (c2_i35 = 0; c2_i35 < 9; c2_i35++) {
    c2_wImu_R_imu[c2_i35] = 0.0;
  }

  for (c2_i36 = 0; c2_i36 < 9; c2_i36++) {
    c2_C[c2_i36] = c2_wImu_R_imu[c2_i36];
  }

  for (c2_i37 = 0; c2_i37 < 9; c2_i37++) {
    c2_wImu_R_imu[c2_i37] = c2_C[c2_i37];
  }

  for (c2_i38 = 0; c2_i38 < 9; c2_i38++) {
    c2_C[c2_i38] = c2_wImu_R_imu[c2_i38];
  }

  for (c2_i39 = 0; c2_i39 < 9; c2_i39++) {
    c2_wImu_R_imu[c2_i39] = c2_C[c2_i39];
  }

  for (c2_i40 = 0; c2_i40 < 3; c2_i40++) {
    c2_i41 = 0;
    for (c2_i42 = 0; c2_i42 < 3; c2_i42++) {
      c2_wImu_R_imu[c2_i41 + c2_i40] = 0.0;
      c2_i43 = 0;
      for (c2_i44 = 0; c2_i44 < 3; c2_i44++) {
        c2_wImu_R_imu[c2_i41 + c2_i40] += c2_y[c2_i43 + c2_i40] * c2_b[c2_i44 +
          c2_i41];
        c2_i43 += 3;
      }

      c2_i41 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 11);
  c2_rotz(chartInstance, c2_inertial_0[2], c2_b_a);
  c2_roty(chartInstance, c2_inertial_0[1], c2_b);
  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i45 = 0; c2_i45 < 3; c2_i45++) {
    c2_i46 = 0;
    for (c2_i47 = 0; c2_i47 < 3; c2_i47++) {
      c2_y[c2_i46 + c2_i45] = 0.0;
      c2_i48 = 0;
      for (c2_i49 = 0; c2_i49 < 3; c2_i49++) {
        c2_y[c2_i46 + c2_i45] += c2_b_a[c2_i48 + c2_i45] * c2_b[c2_i49 + c2_i46];
        c2_i48 += 3;
      }

      c2_i46 += 3;
    }
  }

  c2_rotx(chartInstance, c2_inertial_0[0], c2_b);
  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i50 = 0; c2_i50 < 9; c2_i50++) {
    c2_wImu_R_imu_0[c2_i50] = 0.0;
  }

  for (c2_i51 = 0; c2_i51 < 9; c2_i51++) {
    c2_wImu_R_imu_0[c2_i51] = 0.0;
  }

  for (c2_i52 = 0; c2_i52 < 9; c2_i52++) {
    c2_C[c2_i52] = c2_wImu_R_imu_0[c2_i52];
  }

  for (c2_i53 = 0; c2_i53 < 9; c2_i53++) {
    c2_wImu_R_imu_0[c2_i53] = c2_C[c2_i53];
  }

  for (c2_i54 = 0; c2_i54 < 9; c2_i54++) {
    c2_C[c2_i54] = c2_wImu_R_imu_0[c2_i54];
  }

  for (c2_i55 = 0; c2_i55 < 9; c2_i55++) {
    c2_wImu_R_imu_0[c2_i55] = c2_C[c2_i55];
  }

  for (c2_i56 = 0; c2_i56 < 3; c2_i56++) {
    c2_i57 = 0;
    for (c2_i58 = 0; c2_i58 < 3; c2_i58++) {
      c2_wImu_R_imu_0[c2_i57 + c2_i56] = 0.0;
      c2_i59 = 0;
      for (c2_i60 = 0; c2_i60 < 3; c2_i60++) {
        c2_wImu_R_imu_0[c2_i57 + c2_i56] += c2_y[c2_i59 + c2_i56] * c2_b[c2_i60
          + c2_i57];
        c2_i59 += 3;
      }

      c2_i57 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_i61 = 0;
  c2_i62 = 0;
  for (c2_i63 = 0; c2_i63 < 3; c2_i63++) {
    for (c2_i64 = 0; c2_i64 < 3; c2_i64++) {
      c2_imu_R_link[c2_i64 + c2_i61] = c2_imu_H_link[c2_i64 + c2_i62];
    }

    c2_i61 += 3;
    c2_i62 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 15);
  c2_i65 = 0;
  c2_i66 = 0;
  for (c2_i67 = 0; c2_i67 < 3; c2_i67++) {
    for (c2_i68 = 0; c2_i68 < 3; c2_i68++) {
      c2_imu_R_link_0[c2_i68 + c2_i65] = c2_imu_H_link_0[c2_i68 + c2_i66];
    }

    c2_i65 += 3;
    c2_i66 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 19);
  for (c2_i69 = 0; c2_i69 < 9; c2_i69++) {
    c2_b_a[c2_i69] = c2_wImu_R_imu[c2_i69];
  }

  for (c2_i70 = 0; c2_i70 < 9; c2_i70++) {
    c2_b[c2_i70] = c2_imu_R_link[c2_i70];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i71 = 0; c2_i71 < 9; c2_i71++) {
    c2_wImu_R_link[c2_i71] = 0.0;
  }

  for (c2_i72 = 0; c2_i72 < 9; c2_i72++) {
    c2_wImu_R_link[c2_i72] = 0.0;
  }

  for (c2_i73 = 0; c2_i73 < 9; c2_i73++) {
    c2_C[c2_i73] = c2_wImu_R_link[c2_i73];
  }

  for (c2_i74 = 0; c2_i74 < 9; c2_i74++) {
    c2_wImu_R_link[c2_i74] = c2_C[c2_i74];
  }

  for (c2_i75 = 0; c2_i75 < 9; c2_i75++) {
    c2_C[c2_i75] = c2_wImu_R_link[c2_i75];
  }

  for (c2_i76 = 0; c2_i76 < 9; c2_i76++) {
    c2_wImu_R_link[c2_i76] = c2_C[c2_i76];
  }

  for (c2_i77 = 0; c2_i77 < 3; c2_i77++) {
    c2_i78 = 0;
    for (c2_i79 = 0; c2_i79 < 3; c2_i79++) {
      c2_wImu_R_link[c2_i78 + c2_i77] = 0.0;
      c2_i80 = 0;
      for (c2_i81 = 0; c2_i81 < 3; c2_i81++) {
        c2_wImu_R_link[c2_i78 + c2_i77] += c2_b_a[c2_i80 + c2_i77] * c2_b[c2_i81
          + c2_i78];
        c2_i80 += 3;
      }

      c2_i78 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 20);
  for (c2_i82 = 0; c2_i82 < 9; c2_i82++) {
    c2_b_a[c2_i82] = c2_wImu_R_imu_0[c2_i82];
  }

  for (c2_i83 = 0; c2_i83 < 9; c2_i83++) {
    c2_b[c2_i83] = c2_imu_R_link_0[c2_i83];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i84 = 0; c2_i84 < 9; c2_i84++) {
    c2_wImu_R_link_0[c2_i84] = 0.0;
  }

  for (c2_i85 = 0; c2_i85 < 9; c2_i85++) {
    c2_wImu_R_link_0[c2_i85] = 0.0;
  }

  for (c2_i86 = 0; c2_i86 < 9; c2_i86++) {
    c2_C[c2_i86] = c2_wImu_R_link_0[c2_i86];
  }

  for (c2_i87 = 0; c2_i87 < 9; c2_i87++) {
    c2_wImu_R_link_0[c2_i87] = c2_C[c2_i87];
  }

  for (c2_i88 = 0; c2_i88 < 9; c2_i88++) {
    c2_C[c2_i88] = c2_wImu_R_link_0[c2_i88];
  }

  for (c2_i89 = 0; c2_i89 < 9; c2_i89++) {
    c2_wImu_R_link_0[c2_i89] = c2_C[c2_i89];
  }

  for (c2_i90 = 0; c2_i90 < 3; c2_i90++) {
    c2_i91 = 0;
    for (c2_i92 = 0; c2_i92 < 3; c2_i92++) {
      c2_wImu_R_link_0[c2_i91 + c2_i90] = 0.0;
      c2_i93 = 0;
      for (c2_i94 = 0; c2_i94 < 3; c2_i94++) {
        c2_wImu_R_link_0[c2_i91 + c2_i90] += c2_b_a[c2_i93 + c2_i90] *
          c2_b[c2_i94 + c2_i91];
        c2_i93 += 3;
      }

      c2_i91 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 23);
  for (c2_i95 = 0; c2_i95 < 9; c2_i95++) {
    c2_b_wImu_R_link_0[c2_i95] = c2_wImu_R_link_0[c2_i95];
  }

  c2_rollPitchYawFromRotation(chartInstance, c2_b_wImu_R_link_0, c2_dv2);
  for (c2_i96 = 0; c2_i96 < 3; c2_i96++) {
    c2_rollPitchYaw_link_0[c2_i96] = c2_dv2[c2_i96];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 24);
  for (c2_i97 = 0; c2_i97 < 9; c2_i97++) {
    c2_b_wImu_R_link[c2_i97] = c2_wImu_R_link[c2_i97];
  }

  c2_rollPitchYawFromRotation(chartInstance, c2_b_wImu_R_link, c2_dv3);
  for (c2_i98 = 0; c2_i98 < 3; c2_i98++) {
    c2_rollPitchYaw_link[c2_i98] = c2_dv3[c2_i98];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 26);
  for (c2_i99 = 0; c2_i99 < 3; c2_i99++) {
    c2_rollPitchYawFiltered_link[c2_i99] = c2_rollPitchYaw_link[c2_i99];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 28);
  if (CV_SCRIPT_IF(0, 0, c2_b_CONFIG->YAW_IMU_FILTER)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 29);
    c2_rollPitchYawFiltered_link[2] = c2_rollPitchYaw_link_0[2];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 31);
  if (CV_SCRIPT_IF(0, 1, c2_b_CONFIG->PITCH_IMU_FILTER)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 32);
    c2_rollPitchYawFiltered_link[1] = c2_rollPitchYaw_link_0[1];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 35);
  c2_rotz(chartInstance, c2_rollPitchYawFiltered_link[2], c2_b_a);
  c2_roty(chartInstance, c2_rollPitchYawFiltered_link[1], c2_b);
  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i100 = 0; c2_i100 < 3; c2_i100++) {
    c2_i101 = 0;
    for (c2_i102 = 0; c2_i102 < 3; c2_i102++) {
      c2_y[c2_i101 + c2_i100] = 0.0;
      c2_i103 = 0;
      for (c2_i104 = 0; c2_i104 < 3; c2_i104++) {
        c2_y[c2_i101 + c2_i100] += c2_b_a[c2_i103 + c2_i100] * c2_b[c2_i104 +
          c2_i101];
        c2_i103 += 3;
      }

      c2_i101 += 3;
    }
  }

  c2_rotx(chartInstance, c2_rollPitchYawFiltered_link[0], c2_b);
  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i105 = 0; c2_i105 < 9; c2_i105++) {
    c2_wImu_R_link[c2_i105] = 0.0;
  }

  for (c2_i106 = 0; c2_i106 < 9; c2_i106++) {
    c2_wImu_R_link[c2_i106] = 0.0;
  }

  for (c2_i107 = 0; c2_i107 < 9; c2_i107++) {
    c2_C[c2_i107] = c2_wImu_R_link[c2_i107];
  }

  for (c2_i108 = 0; c2_i108 < 9; c2_i108++) {
    c2_wImu_R_link[c2_i108] = c2_C[c2_i108];
  }

  for (c2_i109 = 0; c2_i109 < 9; c2_i109++) {
    c2_C[c2_i109] = c2_wImu_R_link[c2_i109];
  }

  for (c2_i110 = 0; c2_i110 < 9; c2_i110++) {
    c2_wImu_R_link[c2_i110] = c2_C[c2_i110];
  }

  for (c2_i111 = 0; c2_i111 < 3; c2_i111++) {
    c2_i112 = 0;
    for (c2_i113 = 0; c2_i113 < 3; c2_i113++) {
      c2_wImu_R_link[c2_i112 + c2_i111] = 0.0;
      c2_i114 = 0;
      for (c2_i115 = 0; c2_i115 < 3; c2_i115++) {
        c2_wImu_R_link[c2_i112 + c2_i111] += c2_y[c2_i114 + c2_i111] *
          c2_b[c2_i115 + c2_i112];
        c2_i114 += 3;
      }

      c2_i112 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 37);
  for (c2_i116 = 0; c2_i116 < 3; c2_i116++) {
    c2_dv4[c2_i116] = 0.0;
  }

  c2_i117 = 0;
  c2_i118 = 0;
  for (c2_i119 = 0; c2_i119 < 3; c2_i119++) {
    for (c2_i120 = 0; c2_i120 < 3; c2_i120++) {
      c2_wImu_H_link[c2_i120 + c2_i117] = c2_wImu_R_link[c2_i120 + c2_i118];
    }

    c2_i117 += 4;
    c2_i118 += 3;
  }

  for (c2_i121 = 0; c2_i121 < 3; c2_i121++) {
    c2_wImu_H_link[c2_i121 + 12] = c2_dv4[c2_i121];
  }

  c2_i122 = 0;
  for (c2_i123 = 0; c2_i123 < 4; c2_i123++) {
    c2_wImu_H_link[c2_i122 + 3] = c2_dv5[c2_i123];
    c2_i122 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 40);
  for (c2_i124 = 0; c2_i124 < 3; c2_i124++) {
    c2_dv4[c2_i124] = 0.0;
  }

  c2_i125 = 0;
  c2_i126 = 0;
  for (c2_i127 = 0; c2_i127 < 3; c2_i127++) {
    for (c2_i128 = 0; c2_i128 < 3; c2_i128++) {
      c2_wImu_H_link_0[c2_i128 + c2_i125] = c2_wImu_R_link_0[c2_i128 + c2_i126];
    }

    c2_i125 += 4;
    c2_i126 += 3;
  }

  for (c2_i129 = 0; c2_i129 < 3; c2_i129++) {
    c2_wImu_H_link_0[c2_i129 + 12] = c2_dv4[c2_i129];
  }

  c2_i130 = 0;
  for (c2_i131 = 0; c2_i131 < 4; c2_i131++) {
    c2_wImu_H_link_0[c2_i130 + 3] = c2_dv5[c2_i131];
    c2_i130 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 43);
  for (c2_i132 = 0; c2_i132 < 16; c2_i132++) {
    c2_c_a[c2_i132] = c2_wImu_H_link[c2_i132];
  }

  for (c2_i133 = 0; c2_i133 < 16; c2_i133++) {
    c2_b_b[c2_i133] = c2_link_H_root[c2_i133];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i134 = 0; c2_i134 < 16; c2_i134++) {
    c2_wImu_H_root[c2_i134] = 0.0;
  }

  for (c2_i135 = 0; c2_i135 < 16; c2_i135++) {
    c2_wImu_H_root[c2_i135] = 0.0;
  }

  for (c2_i136 = 0; c2_i136 < 16; c2_i136++) {
    c2_b_C[c2_i136] = c2_wImu_H_root[c2_i136];
  }

  for (c2_i137 = 0; c2_i137 < 16; c2_i137++) {
    c2_wImu_H_root[c2_i137] = c2_b_C[c2_i137];
  }

  for (c2_i138 = 0; c2_i138 < 16; c2_i138++) {
    c2_b_C[c2_i138] = c2_wImu_H_root[c2_i138];
  }

  for (c2_i139 = 0; c2_i139 < 16; c2_i139++) {
    c2_wImu_H_root[c2_i139] = c2_b_C[c2_i139];
  }

  for (c2_i140 = 0; c2_i140 < 4; c2_i140++) {
    c2_i141 = 0;
    for (c2_i142 = 0; c2_i142 < 4; c2_i142++) {
      c2_wImu_H_root[c2_i141 + c2_i140] = 0.0;
      c2_i143 = 0;
      for (c2_i144 = 0; c2_i144 < 4; c2_i144++) {
        c2_wImu_H_root[c2_i141 + c2_i140] += c2_c_a[c2_i143 + c2_i140] *
          c2_b_b[c2_i144 + c2_i141];
        c2_i143 += 4;
      }

      c2_i141 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 46);
  for (c2_i145 = 0; c2_i145 < 3; c2_i145++) {
    c2_b_neck[c2_i145] = c2_neck[c2_i145];
  }

  c2_correctIMU(chartInstance, c2_b_neck, c2_dv6);
  for (c2_i146 = 0; c2_i146 < 16; c2_i146++) {
    c2_wImu_H_wImuAssumingNeckToZero[c2_i146] = c2_dv6[c2_i146];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 47);
  for (c2_i147 = 0; c2_i147 < 16; c2_i147++) {
    c2_c_a[c2_i147] = c2_wImu_H_wImuAssumingNeckToZero[c2_i147];
  }

  for (c2_i148 = 0; c2_i148 < 16; c2_i148++) {
    c2_b_b[c2_i148] = c2_wImu_H_root[c2_i148];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i149 = 0; c2_i149 < 16; c2_i149++) {
    c2_wImu_H_root[c2_i149] = 0.0;
  }

  for (c2_i150 = 0; c2_i150 < 16; c2_i150++) {
    c2_wImu_H_root[c2_i150] = 0.0;
  }

  for (c2_i151 = 0; c2_i151 < 16; c2_i151++) {
    c2_b_C[c2_i151] = c2_wImu_H_root[c2_i151];
  }

  for (c2_i152 = 0; c2_i152 < 16; c2_i152++) {
    c2_wImu_H_root[c2_i152] = c2_b_C[c2_i152];
  }

  for (c2_i153 = 0; c2_i153 < 16; c2_i153++) {
    c2_b_C[c2_i153] = c2_wImu_H_root[c2_i153];
  }

  for (c2_i154 = 0; c2_i154 < 16; c2_i154++) {
    c2_wImu_H_root[c2_i154] = c2_b_C[c2_i154];
  }

  for (c2_i155 = 0; c2_i155 < 4; c2_i155++) {
    c2_i156 = 0;
    for (c2_i157 = 0; c2_i157 < 4; c2_i157++) {
      c2_wImu_H_root[c2_i156 + c2_i155] = 0.0;
      c2_i158 = 0;
      for (c2_i159 = 0; c2_i159 < 4; c2_i159++) {
        c2_wImu_H_root[c2_i156 + c2_i155] += c2_c_a[c2_i158 + c2_i155] *
          c2_b_b[c2_i159 + c2_i156];
        c2_i158 += 4;
      }

      c2_i156 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 49);
  for (c2_i160 = 0; c2_i160 < 16; c2_i160++) {
    c2_c_a[c2_i160] = c2_wImu_H_link_0[c2_i160];
  }

  for (c2_i161 = 0; c2_i161 < 16; c2_i161++) {
    c2_b_b[c2_i161] = c2_wImu_H_root[c2_i161];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_c_a, c2_ipiv, &c2_info);
  c2_b_info = c2_info;
  c2_c_info = c2_b_info;
  c2_d_info = c2_c_info;
  if (c2_d_info > 0) {
    c2_eml_warning(chartInstance);
  }

  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i162 = 0; c2_i162 < 16; c2_i162++) {
    c2_w_H_root[c2_i162] = c2_b_b[c2_i162];
  }

  for (c2_i = 1; c2_i < 5; c2_i++) {
    c2_b_i = c2_i;
    if (c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 4, 1, 0) - 1] != c2_b_i) {
      c2_ip = c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) - 1];
      for (c2_j = 1; c2_j < 5; c2_j++) {
        c2_b_j = c2_j;
        c2_temp = c2_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) +
          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c2_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c2_b_i), 1, 4, 1, 0) +
                     ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 2, 0) - 1) << 2)) - 1] =
          c2_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_ip), 1, 4, 1, 0) +
                       ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c2_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c2_ip), 1, 4, 1, 0) +
                     ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 2, 0) - 1) << 2)) - 1] =
          c2_temp;
      }
    }
  }

  for (c2_i163 = 0; c2_i163 < 16; c2_i163++) {
    c2_dv7[c2_i163] = c2_c_a[c2_i163];
  }

  for (c2_i164 = 0; c2_i164 < 16; c2_i164++) {
    c2_dv8[c2_i164] = c2_dv7[c2_i164];
  }

  c2_c_eml_xtrsm(chartInstance, c2_dv8, c2_w_H_root);
  for (c2_i165 = 0; c2_i165 < 16; c2_i165++) {
    c2_dv9[c2_i165] = c2_c_a[c2_i165];
  }

  for (c2_i166 = 0; c2_i166 < 16; c2_i166++) {
    c2_dv10[c2_i166] = c2_dv9[c2_i166];
  }

  c2_d_eml_xtrsm(chartInstance, c2_dv10, c2_w_H_root);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, -49);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_rotz(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c2_alpha, real_T c2_R[9])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i167;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_alpha, 2U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_R, 3U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 2);
  for (c2_i167 = 0; c2_i167 < 9; c2_i167++) {
    c2_R[c2_i167] = 0.0;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 3);
  c2_R[8] = 1.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 4);
  c2_x = c2_alpha;
  c2_b_x = c2_x;
  c2_b_x = muDoubleScalarCos(c2_b_x);
  c2_R[0] = c2_b_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 5);
  c2_c_x = c2_alpha;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarSin(c2_d_x);
  c2_R[3] = -c2_d_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 6);
  c2_e_x = c2_alpha;
  c2_f_x = c2_e_x;
  c2_f_x = muDoubleScalarSin(c2_f_x);
  c2_R[1] = c2_f_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 7);
  c2_g_x = c2_alpha;
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarCos(c2_h_x);
  c2_R[4] = c2_h_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_roty(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c2_alpha, real_T c2_R[9])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i168;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_alpha, 2U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_R, 3U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 2);
  for (c2_i168 = 0; c2_i168 < 9; c2_i168++) {
    c2_R[c2_i168] = 0.0;
  }

  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 3);
  c2_R[4] = 1.0;
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 4);
  c2_x = c2_alpha;
  c2_b_x = c2_x;
  c2_b_x = muDoubleScalarCos(c2_b_x);
  c2_R[0] = c2_b_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 5);
  c2_c_x = c2_alpha;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarSin(c2_d_x);
  c2_R[6] = c2_d_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 6);
  c2_e_x = c2_alpha;
  c2_f_x = c2_e_x;
  c2_f_x = muDoubleScalarSin(c2_f_x);
  c2_R[2] = -c2_f_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 7);
  c2_g_x = c2_alpha;
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarCos(c2_h_x);
  c2_R[8] = c2_h_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_rotx(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c2_alpha, real_T c2_R[9])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i169;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_d_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_alpha, 2U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_R, 3U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_SCRIPT_FCN(3, 0);
  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 2);
  for (c2_i169 = 0; c2_i169 < 9; c2_i169++) {
    c2_R[c2_i169] = 0.0;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 3);
  c2_R[0] = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 4);
  c2_x = c2_alpha;
  c2_b_x = c2_x;
  c2_b_x = muDoubleScalarCos(c2_b_x);
  c2_R[4] = c2_b_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 5);
  c2_c_x = c2_alpha;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarSin(c2_d_x);
  c2_R[7] = -c2_d_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 6);
  c2_e_x = c2_alpha;
  c2_f_x = c2_e_x;
  c2_f_x = muDoubleScalarSin(c2_f_x);
  c2_R[5] = c2_f_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, 7);
  c2_g_x = c2_alpha;
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarCos(c2_h_x);
  c2_R[8] = c2_h_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c2_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_rollPitchYawFromRotation(SFc2_torqueBalancing2012bInstanceStruct *
  chartInstance, real_T c2_R[9], real_T c2_rollPitchYaw[3])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i170;
  real_T c2_x;
  real_T c2_b_x;
  boolean_T guard1 = FALSE;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_e_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_R, 2U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rollPitchYaw, 3U, c2_c_sf_marshallOut,
    c2_e_sf_marshallIn);
  CV_SCRIPT_FCN(4, 0);
  _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 4);
  for (c2_i170 = 0; c2_i170 < 3; c2_i170++) {
    c2_rollPitchYaw[c2_i170] = 0.0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 5);
  if (CV_SCRIPT_IF(4, 0, c2_R[2] < 1.0)) {
    _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 6);
    if (CV_SCRIPT_IF(4, 1, c2_R[2] > -1.0)) {
      _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 7);
      c2_x = -c2_R[2];
      c2_b_x = c2_x;
      guard1 = FALSE;
      if (c2_b_x < -1.0) {
        guard1 = TRUE;
      } else {
        if (1.0 < c2_b_x) {
          guard1 = TRUE;
        }
      }

      if (guard1 == TRUE) {
        c2_eml_error(chartInstance);
      }

      c2_b_x = muDoubleScalarAsin(c2_b_x);
      c2_rollPitchYaw[1] = c2_b_x;
      _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 8);
      c2_rollPitchYaw[2] = c2_atan2(chartInstance, c2_R[1], c2_R[0]);
      _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 9);
      c2_rollPitchYaw[0] = c2_atan2(chartInstance, c2_R[5], c2_R[8]);
    } else {
      _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 11);
      c2_rollPitchYaw[2] = -c2_atan2(chartInstance, -c2_R[7], c2_R[4]);
      _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 12);
      c2_rollPitchYaw[0] = 0.0;
    }
  } else {
    _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 15);
    c2_rollPitchYaw[2] = c2_atan2(chartInstance, -c2_R[7], c2_R[4]);
    _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, 16);
    c2_rollPitchYaw[0] = 0.0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c2_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_correctIMU(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c2_neckJoints[3], real_T c2_imu_H_imuAssumingNeckToZero[16])
{
  uint32_T c2_debug_family_var_map[10];
  real_T c2_G_34[16];
  real_T c2_G_45[16];
  real_T c2_G_56[16];
  real_T c2_G_6I[16];
  real_T c2_imuAssumingNeckToZero_H_neckBase[16];
  real_T c2_imu_H_neckBase[16];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i171;
  static real_T c2_dv11[16] = { 6.123233995736766E-17, 1.0, 0.0, 0.0,
    -6.123233995736766E-17, 3.749399456654644E-33, 1.0, 0.0, 1.0,
    -6.123233995736766E-17, 6.123233995736766E-17, 0.0, 5.8170722959499274E-19,
    0.0095, 0.0, 1.0 };

  int32_T c2_i172;
  static real_T c2_dv12[16] = { 6.123233995736766E-17, -1.0, 0.0, 0.0,
    6.123233995736766E-17, 3.749399456654644E-33, -1.0, 0.0, 1.0,
    6.123233995736766E-17, 6.123233995736766E-17, 0.0, 0.0, -0.0, 0.0, 1.0 };

  int32_T c2_i173;
  static real_T c2_dv13[16] = { 6.123233995736766E-17, 1.0, 0.0, 0.0,
    -6.123233995736766E-17, 3.749399456654644E-33, -1.0, 0.0, -1.0,
    6.123233995736766E-17, 6.123233995736766E-17, 0.0, 1.1327982892113017E-18,
    0.0185, 0.1108, 1.0 };

  int32_T c2_i174;
  static real_T c2_b[16] = { 1.0, 0.0, 0.0, 0.0, -0.0, 6.123233995736766E-17,
    1.0, 0.0, 0.0, -1.0, 6.123233995736766E-17, 0.0, 0.0, 0.0, 0.0066, 1.0 };

  int32_T c2_i175;
  static real_T c2_dv14[16] = { -1.0, 1.2246467991473532E-16,
    -1.2246467991473532E-16, 0.0, -1.2246467991473532E-16,
    -6.1232339957367648E-17, 1.0, 0.0, 1.2246467991473532E-16, 1.0,
    6.123233995736766E-17, 0.0, -0.018499999999999985, 0.12029999999999999,
    0.0066000000000000043, 1.0 };

  real_T c2_dv15[16];
  int32_T c2_i176;
  real_T c2_dv16[16];
  int32_T c2_i177;
  real_T c2_dv17[16];
  int32_T c2_i178;
  int32_T c2_i179;
  int32_T c2_i180;
  real_T c2_a[16];
  int32_T c2_i181;
  real_T c2_b_b[16];
  int32_T c2_i182;
  int32_T c2_i183;
  int32_T c2_i184;
  real_T c2_y[16];
  int32_T c2_i185;
  int32_T c2_i186;
  int32_T c2_i187;
  int32_T c2_i188;
  int32_T c2_i189;
  int32_T c2_i190;
  real_T c2_b_y[16];
  int32_T c2_i191;
  int32_T c2_i192;
  int32_T c2_i193;
  int32_T c2_i194;
  int32_T c2_i195;
  int32_T c2_i196;
  int32_T c2_i197;
  int32_T c2_i198;
  int32_T c2_i199;
  int32_T c2_i200;
  int32_T c2_i201;
  int32_T c2_i202;
  int32_T c2_i203;
  int32_T c2_i204;
  int32_T c2_i205;
  int32_T c2_i206;
  int32_T c2_i207;
  int32_T c2_i208;
  int32_T c2_i;
  int32_T c2_b_i;
  static int32_T c2_iv0[4] = { 1, 3, 3, 4 };

  int32_T c2_ip;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_temp;
  int32_T c2_i209;
  static real_T c2_dv18[16] = { -1.0, -1.2246467991473532E-16,
    1.2246467991473532E-16, 0.018499999999999985, 1.2246467991473532E-16, 1.0,
    -6.123233995736766E-17, 0.12029999999999999, -1.2246467991473532E-16,
    6.1232339957367648E-17, 1.0, 0.0066, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_dv19[16];
  int32_T c2_i210;
  real_T c2_dv20[16];
  int32_T c2_i211;
  int32_T c2_i212;
  int32_T c2_i213;
  int32_T c2_i214;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c2_g_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_G_34, 0U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_G_45, 1U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_G_56, 2U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_G_6I, 3U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_imuAssumingNeckToZero_H_neckBase, 4U,
    c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_imu_H_neckBase, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 6U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 7U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_neckJoints, 8U, c2_c_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_imu_H_imuAssumingNeckToZero, 9U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_SCRIPT_FCN(5, 0);
  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 9);
  for (c2_i171 = 0; c2_i171 < 16; c2_i171++) {
    c2_G_34[c2_i171] = c2_dv11[c2_i171];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 10);
  for (c2_i172 = 0; c2_i172 < 16; c2_i172++) {
    c2_G_45[c2_i172] = c2_dv12[c2_i172];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 11);
  for (c2_i173 = 0; c2_i173 < 16; c2_i173++) {
    c2_G_56[c2_i173] = c2_dv13[c2_i173];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 12);
  for (c2_i174 = 0; c2_i174 < 16; c2_i174++) {
    c2_G_6I[c2_i174] = c2_b[c2_i174];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 14);
  for (c2_i175 = 0; c2_i175 < 16; c2_i175++) {
    c2_imuAssumingNeckToZero_H_neckBase[c2_i175] = c2_dv14[c2_i175];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 17);
  c2_evalDHMatrix(chartInstance, 0.0095, 0.0, 1.5707963267948966, c2_neckJoints
                  [0] + 1.5707963267948966, c2_dv15);
  for (c2_i176 = 0; c2_i176 < 16; c2_i176++) {
    c2_G_34[c2_i176] = c2_dv15[c2_i176];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 18);
  c2_evalDHMatrix(chartInstance, 0.0, 0.0, -1.5707963267948966, c2_neckJoints[1]
                  - 1.5707963267948966, c2_dv16);
  for (c2_i177 = 0; c2_i177 < 16; c2_i177++) {
    c2_G_45[c2_i177] = c2_dv16[c2_i177];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 19);
  c2_evalDHMatrix(chartInstance, 0.0185, 0.1108, -1.5707963267948966,
                  c2_neckJoints[2] + 1.5707963267948966, c2_dv17);
  for (c2_i178 = 0; c2_i178 < 16; c2_i178++) {
    c2_G_56[c2_i178] = c2_dv17[c2_i178];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 20);
  for (c2_i179 = 0; c2_i179 < 16; c2_i179++) {
    c2_G_6I[c2_i179] = c2_b[c2_i179];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 22);
  for (c2_i180 = 0; c2_i180 < 16; c2_i180++) {
    c2_a[c2_i180] = c2_G_34[c2_i180];
  }

  for (c2_i181 = 0; c2_i181 < 16; c2_i181++) {
    c2_b_b[c2_i181] = c2_G_45[c2_i181];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i182 = 0; c2_i182 < 4; c2_i182++) {
    c2_i183 = 0;
    for (c2_i184 = 0; c2_i184 < 4; c2_i184++) {
      c2_y[c2_i183 + c2_i182] = 0.0;
      c2_i185 = 0;
      for (c2_i186 = 0; c2_i186 < 4; c2_i186++) {
        c2_y[c2_i183 + c2_i182] += c2_a[c2_i185 + c2_i182] * c2_b_b[c2_i186 +
          c2_i183];
        c2_i185 += 4;
      }

      c2_i183 += 4;
    }
  }

  for (c2_i187 = 0; c2_i187 < 16; c2_i187++) {
    c2_b_b[c2_i187] = c2_G_56[c2_i187];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i188 = 0; c2_i188 < 4; c2_i188++) {
    c2_i189 = 0;
    for (c2_i190 = 0; c2_i190 < 4; c2_i190++) {
      c2_b_y[c2_i189 + c2_i188] = 0.0;
      c2_i191 = 0;
      for (c2_i192 = 0; c2_i192 < 4; c2_i192++) {
        c2_b_y[c2_i189 + c2_i188] += c2_y[c2_i191 + c2_i188] * c2_b_b[c2_i192 +
          c2_i189];
        c2_i191 += 4;
      }

      c2_i189 += 4;
    }
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i193 = 0; c2_i193 < 16; c2_i193++) {
    c2_imu_H_neckBase[c2_i193] = 0.0;
  }

  for (c2_i194 = 0; c2_i194 < 16; c2_i194++) {
    c2_imu_H_neckBase[c2_i194] = 0.0;
  }

  for (c2_i195 = 0; c2_i195 < 16; c2_i195++) {
    c2_a[c2_i195] = c2_imu_H_neckBase[c2_i195];
  }

  for (c2_i196 = 0; c2_i196 < 16; c2_i196++) {
    c2_imu_H_neckBase[c2_i196] = c2_a[c2_i196];
  }

  for (c2_i197 = 0; c2_i197 < 16; c2_i197++) {
    c2_a[c2_i197] = c2_imu_H_neckBase[c2_i197];
  }

  for (c2_i198 = 0; c2_i198 < 16; c2_i198++) {
    c2_imu_H_neckBase[c2_i198] = c2_a[c2_i198];
  }

  for (c2_i199 = 0; c2_i199 < 4; c2_i199++) {
    c2_i200 = 0;
    for (c2_i201 = 0; c2_i201 < 4; c2_i201++) {
      c2_imu_H_neckBase[c2_i200 + c2_i199] = 0.0;
      c2_i202 = 0;
      for (c2_i203 = 0; c2_i203 < 4; c2_i203++) {
        c2_imu_H_neckBase[c2_i200 + c2_i199] += c2_b_y[c2_i202 + c2_i199] *
          c2_b[c2_i203 + c2_i200];
        c2_i202 += 4;
      }

      c2_i200 += 4;
    }
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, 24);
  for (c2_i204 = 0; c2_i204 < 16; c2_i204++) {
    c2_a[c2_i204] = c2_imu_H_neckBase[c2_i204];
  }

  c2_i205 = 0;
  for (c2_i206 = 0; c2_i206 < 4; c2_i206++) {
    c2_i207 = 0;
    for (c2_i208 = 0; c2_i208 < 4; c2_i208++) {
      c2_b_b[c2_i208 + c2_i205] = c2_a[c2_i207 + c2_i206];
      c2_i207 += 4;
    }

    c2_i205 += 4;
  }

  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i = 1; c2_i < 5; c2_i++) {
    c2_b_i = c2_i;
    if (c2_iv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 4, 1, 0) - 1] != c2_b_i) {
      c2_ip = c2_iv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c2_b_i), 1, 4, 1, 0) - 1];
      for (c2_j = 1; c2_j < 5; c2_j++) {
        c2_b_j = c2_j;
        c2_temp = c2_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) +
                          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c2_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                   "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 2,
                   0) - 1) << 2)) - 1] = c2_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_ip), 1, 4, 1, 0) +
          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c2_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
                   (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 2, 0)
                  - 1) << 2)) - 1] = c2_temp;
      }
    }
  }

  for (c2_i209 = 0; c2_i209 < 16; c2_i209++) {
    c2_dv19[c2_i209] = c2_dv18[c2_i209];
  }

  c2_c_eml_xtrsm(chartInstance, c2_dv19, c2_b_b);
  for (c2_i210 = 0; c2_i210 < 16; c2_i210++) {
    c2_dv20[c2_i210] = c2_dv18[c2_i210];
  }

  c2_d_eml_xtrsm(chartInstance, c2_dv20, c2_b_b);
  c2_i211 = 0;
  for (c2_i212 = 0; c2_i212 < 4; c2_i212++) {
    c2_i213 = 0;
    for (c2_i214 = 0; c2_i214 < 4; c2_i214++) {
      c2_imu_H_imuAssumingNeckToZero[c2_i214 + c2_i211] = c2_b_b[c2_i213 +
        c2_i212];
      c2_i213 += 4;
    }

    c2_i211 += 4;
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c2_sfEvent, -24);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_evalDHMatrix(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_a, real_T c2_d, real_T c2_alph, real_T c2_thet,
  real_T c2_G[16])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 1.0;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_b_a;
  real_T c2_b;
  real_T c2_y;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_c_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_d_a;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_p_x;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_e_a;
  real_T c2_d_b;
  real_T c2_d_y;
  real_T c2_s_x;
  real_T c2_t_x;
  real_T c2_u_x;
  real_T c2_v_x;
  real_T c2_f_a;
  real_T c2_e_b;
  real_T c2_e_y;
  real_T c2_w_x;
  real_T c2_x_x;
  real_T c2_g_a;
  real_T c2_f_b;
  real_T c2_f_y;
  real_T c2_y_x;
  real_T c2_ab_x;
  real_T c2_bb_x;
  real_T c2_cb_x;
  int32_T c2_i215;
  int32_T c2_i216;
  static real_T c2_dv21[4] = { 0.0, 0.0, 0.0, 1.0 };

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c2_f_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_a, 2U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d, 3U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_alph, 4U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_thet, 5U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_G, 6U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_SCRIPT_FCN(6, 0);
  _SFD_SCRIPT_CALL(6U, chartInstance->c2_sfEvent, 3);
  c2_x = c2_thet;
  c2_b_x = c2_x;
  c2_b_x = muDoubleScalarCos(c2_b_x);
  c2_c_x = c2_thet;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarSin(c2_d_x);
  c2_e_x = c2_alph;
  c2_f_x = c2_e_x;
  c2_f_x = muDoubleScalarCos(c2_f_x);
  c2_b_a = -c2_d_x;
  c2_b = c2_f_x;
  c2_y = c2_b_a * c2_b;
  c2_g_x = c2_thet;
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarSin(c2_h_x);
  c2_i_x = c2_alph;
  c2_j_x = c2_i_x;
  c2_j_x = muDoubleScalarSin(c2_j_x);
  c2_c_a = c2_h_x;
  c2_b_b = c2_j_x;
  c2_b_y = c2_c_a * c2_b_b;
  c2_k_x = c2_thet;
  c2_l_x = c2_k_x;
  c2_l_x = muDoubleScalarCos(c2_l_x);
  c2_d_a = c2_l_x;
  c2_c_b = c2_a;
  c2_c_y = c2_d_a * c2_c_b;
  c2_m_x = c2_thet;
  c2_n_x = c2_m_x;
  c2_n_x = muDoubleScalarSin(c2_n_x);
  c2_o_x = c2_thet;
  c2_p_x = c2_o_x;
  c2_p_x = muDoubleScalarCos(c2_p_x);
  c2_q_x = c2_alph;
  c2_r_x = c2_q_x;
  c2_r_x = muDoubleScalarCos(c2_r_x);
  c2_e_a = c2_p_x;
  c2_d_b = c2_r_x;
  c2_d_y = c2_e_a * c2_d_b;
  c2_s_x = c2_thet;
  c2_t_x = c2_s_x;
  c2_t_x = muDoubleScalarCos(c2_t_x);
  c2_u_x = c2_alph;
  c2_v_x = c2_u_x;
  c2_v_x = muDoubleScalarSin(c2_v_x);
  c2_f_a = -c2_t_x;
  c2_e_b = c2_v_x;
  c2_e_y = c2_f_a * c2_e_b;
  c2_w_x = c2_thet;
  c2_x_x = c2_w_x;
  c2_x_x = muDoubleScalarSin(c2_x_x);
  c2_g_a = c2_x_x;
  c2_f_b = c2_a;
  c2_f_y = c2_g_a * c2_f_b;
  c2_y_x = c2_alph;
  c2_ab_x = c2_y_x;
  c2_ab_x = muDoubleScalarSin(c2_ab_x);
  c2_bb_x = c2_alph;
  c2_cb_x = c2_bb_x;
  c2_cb_x = muDoubleScalarCos(c2_cb_x);
  c2_G[0] = c2_b_x;
  c2_G[4] = c2_y;
  c2_G[8] = c2_b_y;
  c2_G[12] = c2_c_y;
  c2_G[1] = c2_n_x;
  c2_G[5] = c2_d_y;
  c2_G[9] = c2_e_y;
  c2_G[13] = c2_f_y;
  c2_G[2] = 0.0;
  c2_G[6] = c2_ab_x;
  c2_G[10] = c2_cb_x;
  c2_G[14] = c2_d;
  c2_i215 = 0;
  for (c2_i216 = 0; c2_i216 < 4; c2_i216++) {
    c2_G[c2_i215 + 3] = c2_dv21[c2_i216];
    c2_i215 += 4;
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c2_sfEvent, -3);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 0U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 1U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 2U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 3U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 4U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 5U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 6U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m"));
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i217;
  int32_T c2_i218;
  int32_T c2_i219;
  real_T c2_b_inData[16];
  int32_T c2_i220;
  int32_T c2_i221;
  int32_T c2_i222;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i217 = 0;
  for (c2_i218 = 0; c2_i218 < 4; c2_i218++) {
    for (c2_i219 = 0; c2_i219 < 4; c2_i219++) {
      c2_b_inData[c2_i219 + c2_i217] = (*(real_T (*)[16])c2_inData)[c2_i219 +
        c2_i217];
    }

    c2_i217 += 4;
  }

  c2_i220 = 0;
  for (c2_i221 = 0; c2_i221 < 4; c2_i221++) {
    for (c2_i222 = 0; c2_i222 < 4; c2_i222++) {
      c2_u[c2_i222 + c2_i220] = c2_b_inData[c2_i222 + c2_i220];
    }

    c2_i220 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_w_H_root, const char_T *c2_identifier,
  real_T c2_y[16])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_w_H_root), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_w_H_root);
}

static void c2_b_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16])
{
  real_T c2_dv22[16];
  int32_T c2_i223;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv22, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c2_i223 = 0; c2_i223 < 16; c2_i223++) {
    c2_y[c2_i223] = c2_dv22[c2_i223];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_w_H_root;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[16];
  int32_T c2_i224;
  int32_T c2_i225;
  int32_T c2_i226;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_w_H_root = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_w_H_root), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_w_H_root);
  c2_i224 = 0;
  for (c2_i225 = 0; c2_i225 < 4; c2_i225++) {
    for (c2_i226 = 0; c2_i226 < 4; c2_i226++) {
      (*(real_T (*)[16])c2_outData)[c2_i226 + c2_i224] = c2_y[c2_i226 + c2_i224];
    }

    c2_i224 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_struct_HZPQIAn1UwxVqJtlH5llbD c2_u;
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_struct_amzdx4J7qaaMzgPI7fQ3WD c2_c_u;
  const mxArray *c2_c_y = NULL;
  boolean_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  boolean_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  boolean_T c2_f_u;
  const mxArray *c2_f_y = NULL;
  boolean_T c2_g_u;
  const mxArray *c2_g_y = NULL;
  boolean_T c2_h_u;
  const mxArray *c2_h_y = NULL;
  boolean_T c2_i_u;
  const mxArray *c2_i_y = NULL;
  boolean_T c2_j_u;
  const mxArray *c2_j_y = NULL;
  boolean_T c2_k_u;
  const mxArray *c2_k_y = NULL;
  boolean_T c2_l_u;
  const mxArray *c2_l_y = NULL;
  boolean_T c2_m_u;
  const mxArray *c2_m_y = NULL;
  boolean_T c2_n_u;
  const mxArray *c2_n_y = NULL;
  boolean_T c2_o_u;
  const mxArray *c2_o_y = NULL;
  boolean_T c2_p_u;
  const mxArray *c2_p_y = NULL;
  real_T c2_q_u;
  const mxArray *c2_q_y = NULL;
  boolean_T c2_r_u;
  const mxArray *c2_r_y = NULL;
  int32_T c2_i227;
  real_T c2_s_u[2];
  const mxArray *c2_s_y = NULL;
  real_T c2_t_u;
  const mxArray *c2_t_y = NULL;
  real_T c2_u_u;
  const mxArray *c2_u_y = NULL;
  real_T c2_v_u;
  const mxArray *c2_v_y = NULL;
  boolean_T c2_w_u;
  const mxArray *c2_w_y = NULL;
  real_T c2_x_u;
  const mxArray *c2_x_y = NULL;
  int32_T c2_i228;
  real_T c2_y_u[23];
  const mxArray *c2_y_y = NULL;
  int32_T c2_i229;
  real_T c2_ab_u[23];
  const mxArray *c2_ab_y = NULL;
  boolean_T c2_bb_u;
  const mxArray *c2_bb_y = NULL;
  boolean_T c2_cb_u;
  const mxArray *c2_cb_y = NULL;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_struct_HZPQIAn1UwxVqJtlH5llbD *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c2_b_u = c2_u.SIMULATION_TIME;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_b_y, "SIMULATION_TIME", "SIMULATION_TIME", 0);
  c2_c_u = c2_u.SCOPES;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c2_d_u = c2_c_u.ALL;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_c_y, c2_d_y, "ALL", "ALL", 0);
  c2_e_u = c2_c_u.BASE_EST_IMU;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_c_y, c2_e_y, "BASE_EST_IMU", "BASE_EST_IMU", 0);
  c2_f_u = c2_c_u.EXTWRENCHES;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_c_y, c2_f_y, "EXTWRENCHES", "EXTWRENCHES", 0);
  c2_g_u = c2_c_u.GAIN_SCHE_INFO;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_g_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_c_y, c2_g_y, "GAIN_SCHE_INFO", "GAIN_SCHE_INFO", 0);
  c2_h_u = c2_c_u.MAIN;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_h_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_c_y, c2_h_y, "MAIN", "MAIN", 0);
  c2_i_u = c2_c_u.QP;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_i_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_c_y, c2_i_y, "QP", "QP", 0);
  sf_mex_addfield(c2_y, c2_c_y, "SCOPES", "SCOPES", 0);
  c2_j_u = c2_u.CHECK_LIMITS;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_j_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_j_y, "CHECK_LIMITS", "CHECK_LIMITS", 0);
  c2_k_u = c2_u.USE_IMU4EST_BASE;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_k_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_k_y, "USE_IMU4EST_BASE", "USE_IMU4EST_BASE", 0);
  c2_l_u = c2_u.YAW_IMU_FILTER;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_l_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_l_y, "YAW_IMU_FILTER", "YAW_IMU_FILTER", 0);
  c2_m_u = c2_u.PITCH_IMU_FILTER;
  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_m_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_m_y, "PITCH_IMU_FILTER", "PITCH_IMU_FILTER", 0);
  c2_n_u = c2_u.CORRECT_NECK_IMU;
  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_n_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_n_y, "CORRECT_NECK_IMU", "CORRECT_NECK_IMU", 0);
  c2_o_u = c2_u.ONSOFTCARPET;
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_o_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_o_y, "ONSOFTCARPET", "ONSOFTCARPET", 0);
  c2_p_u = c2_u.USE_QP_SOLVER;
  c2_p_y = NULL;
  sf_mex_assign(&c2_p_y, sf_mex_create("y", &c2_p_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_p_y, "USE_QP_SOLVER", "USE_QP_SOLVER", 0);
  c2_q_u = c2_u.Ts;
  c2_q_y = NULL;
  sf_mex_assign(&c2_q_y, sf_mex_create("y", &c2_q_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_q_y, "Ts", "Ts", 0);
  c2_r_u = c2_u.ON_GAZEBO;
  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_create("y", &c2_r_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_r_y, "ON_GAZEBO", "ON_GAZEBO", 0);
  for (c2_i227 = 0; c2_i227 < 2; c2_i227++) {
    c2_s_u[c2_i227] = c2_u.LEFT_RIGHT_FOOT_IN_CONTACT[c2_i227];
  }

  c2_s_y = NULL;
  sf_mex_assign(&c2_s_y, sf_mex_create("y", c2_s_u, 0, 0U, 1U, 0U, 2, 1, 2),
                FALSE);
  sf_mex_addfield(c2_y, c2_s_y, "LEFT_RIGHT_FOOT_IN_CONTACT",
                  "LEFT_RIGHT_FOOT_IN_CONTACT", 0);
  c2_t_u = c2_u.SMOOTH_DES_COM;
  c2_t_y = NULL;
  sf_mex_assign(&c2_t_y, sf_mex_create("y", &c2_t_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_t_y, "SMOOTH_DES_COM", "SMOOTH_DES_COM", 0);
  c2_u_u = c2_u.SMOOTH_DES_Q;
  c2_u_y = NULL;
  sf_mex_assign(&c2_u_y, sf_mex_create("y", &c2_u_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_u_y, "SMOOTH_DES_Q", "SMOOTH_DES_Q", 0);
  c2_v_u = c2_u.smoothingTimeTranDynamics;
  c2_v_y = NULL;
  sf_mex_assign(&c2_v_y, sf_mex_create("y", &c2_v_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_v_y, "smoothingTimeTranDynamics",
                  "smoothingTimeTranDynamics", 0);
  c2_w_u = c2_u.DEMO_MOVEMENTS;
  c2_w_y = NULL;
  sf_mex_assign(&c2_w_y, sf_mex_create("y", &c2_w_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_w_y, "DEMO_MOVEMENTS", "DEMO_MOVEMENTS", 0);
  c2_x_u = c2_u.TIME_CONTROLLER_SWITCH;
  c2_x_y = NULL;
  sf_mex_assign(&c2_x_y, sf_mex_create("y", &c2_x_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_x_y, "TIME_CONTROLLER_SWITCH",
                  "TIME_CONTROLLER_SWITCH", 0);
  for (c2_i228 = 0; c2_i228 < 23; c2_i228++) {
    c2_y_u[c2_i228] = c2_u.JOINTS[c2_i228];
  }

  c2_y_y = NULL;
  sf_mex_assign(&c2_y_y, sf_mex_create("y", c2_y_u, 0, 0U, 1U, 0U, 2, 23, 1),
                FALSE);
  sf_mex_addfield(c2_y, c2_y_y, "JOINTS", "JOINTS", 0);
  for (c2_i229 = 0; c2_i229 < 23; c2_i229++) {
    c2_ab_u[c2_i229] = c2_u.JOINTSITING[c2_i229];
  }

  c2_ab_y = NULL;
  sf_mex_assign(&c2_ab_y, sf_mex_create("y", c2_ab_u, 0, 0U, 1U, 0U, 2, 23, 1),
                FALSE);
  sf_mex_addfield(c2_y, c2_ab_y, "JOINTSITING", "JOINTSITING", 0);
  c2_bb_u = c2_u.iCubStandUp;
  c2_bb_y = NULL;
  sf_mex_assign(&c2_bb_y, sf_mex_create("y", &c2_bb_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_bb_y, "iCubStandUp", "iCubStandUp", 0);
  c2_cb_u = c2_u.useExtArmForces;
  c2_cb_y = NULL;
  sf_mex_assign(&c2_cb_y, sf_mex_create("y", &c2_cb_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c2_y, c2_cb_y, "useExtArmForces", "useExtArmForces", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  c2_struct_HZPQIAn1UwxVqJtlH5llbD *c2_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[21] = { "SIMULATION_TIME", "SCOPES",
    "CHECK_LIMITS", "USE_IMU4EST_BASE", "YAW_IMU_FILTER", "PITCH_IMU_FILTER",
    "CORRECT_NECK_IMU", "ONSOFTCARPET", "USE_QP_SOLVER", "Ts", "ON_GAZEBO",
    "LEFT_RIGHT_FOOT_IN_CONTACT", "SMOOTH_DES_COM", "SMOOTH_DES_Q",
    "smoothingTimeTranDynamics", "DEMO_MOVEMENTS", "TIME_CONTROLLER_SWITCH",
    "JOINTS", "JOINTSITING", "iCubStandUp", "useExtArmForces" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 21, c2_fieldNames, 0U, 0);
  c2_thisId.fIdentifier = "SIMULATION_TIME";
  c2_y->SIMULATION_TIME = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "SIMULATION_TIME", "SIMULATION_TIME", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "SCOPES";
  c2_y->SCOPES = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
                                        (c2_u, "SCOPES", "SCOPES", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "CHECK_LIMITS";
  c2_y->CHECK_LIMITS = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "CHECK_LIMITS", "CHECK_LIMITS", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "USE_IMU4EST_BASE";
  c2_y->USE_IMU4EST_BASE = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "USE_IMU4EST_BASE", "USE_IMU4EST_BASE", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "YAW_IMU_FILTER";
  c2_y->YAW_IMU_FILTER = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "YAW_IMU_FILTER", "YAW_IMU_FILTER", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "PITCH_IMU_FILTER";
  c2_y->PITCH_IMU_FILTER = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "PITCH_IMU_FILTER", "PITCH_IMU_FILTER", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "CORRECT_NECK_IMU";
  c2_y->CORRECT_NECK_IMU = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "CORRECT_NECK_IMU", "CORRECT_NECK_IMU", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "ONSOFTCARPET";
  c2_y->ONSOFTCARPET = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "ONSOFTCARPET", "ONSOFTCARPET", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "USE_QP_SOLVER";
  c2_y->USE_QP_SOLVER = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "USE_QP_SOLVER", "USE_QP_SOLVER", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Ts";
  c2_y->Ts = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Ts", "Ts", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "ON_GAZEBO";
  c2_y->ON_GAZEBO = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "ON_GAZEBO", "ON_GAZEBO", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "LEFT_RIGHT_FOOT_IN_CONTACT";
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "LEFT_RIGHT_FOOT_IN_CONTACT", "LEFT_RIGHT_FOOT_IN_CONTACT", 0)), &c2_thisId,
                        c2_y->LEFT_RIGHT_FOOT_IN_CONTACT);
  c2_thisId.fIdentifier = "SMOOTH_DES_COM";
  c2_y->SMOOTH_DES_COM = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "SMOOTH_DES_COM", "SMOOTH_DES_COM", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "SMOOTH_DES_Q";
  c2_y->SMOOTH_DES_Q = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "SMOOTH_DES_Q", "SMOOTH_DES_Q", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "smoothingTimeTranDynamics";
  c2_y->smoothingTimeTranDynamics = c2_d_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c2_u, "smoothingTimeTranDynamics",
    "smoothingTimeTranDynamics", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "DEMO_MOVEMENTS";
  c2_y->DEMO_MOVEMENTS = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "DEMO_MOVEMENTS", "DEMO_MOVEMENTS", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "TIME_CONTROLLER_SWITCH";
  c2_y->TIME_CONTROLLER_SWITCH = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "TIME_CONTROLLER_SWITCH", "TIME_CONTROLLER_SWITCH", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "JOINTS";
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u, "JOINTS",
    "JOINTS", 0)), &c2_thisId, c2_y->JOINTS);
  c2_thisId.fIdentifier = "JOINTSITING";
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "JOINTSITING", "JOINTSITING", 0)), &c2_thisId, c2_y->JOINTSITING);
  c2_thisId.fIdentifier = "iCubStandUp";
  c2_y->iCubStandUp = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "iCubStandUp", "iCubStandUp", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "useExtArmForces";
  c2_y->useExtArmForces = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "useExtArmForces", "useExtArmForces", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
}

static real_T c2_d_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static c2_struct_amzdx4J7qaaMzgPI7fQ3WD c2_e_emlrt_marshallIn
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c2_u,
   const emlrtMsgIdentifier *c2_parentId)
{
  c2_struct_amzdx4J7qaaMzgPI7fQ3WD c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[6] = { "ALL", "BASE_EST_IMU", "EXTWRENCHES",
    "GAIN_SCHE_INFO", "MAIN", "QP" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 6, c2_fieldNames, 0U, 0);
  c2_thisId.fIdentifier = "ALL";
  c2_y.ALL = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "ALL", "ALL", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "BASE_EST_IMU";
  c2_y.BASE_EST_IMU = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "BASE_EST_IMU", "BASE_EST_IMU", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "EXTWRENCHES";
  c2_y.EXTWRENCHES = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "EXTWRENCHES", "EXTWRENCHES", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "GAIN_SCHE_INFO";
  c2_y.GAIN_SCHE_INFO = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "GAIN_SCHE_INFO", "GAIN_SCHE_INFO", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "MAIN";
  c2_y.MAIN = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "MAIN", "MAIN", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "QP";
  c2_y.QP = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "QP", "QP", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static boolean_T c2_f_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  boolean_T c2_y;
  boolean_T c2_b0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_b0, 1, 11, 0U, 0, 0U, 0);
  c2_y = c2_b0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[2])
{
  real_T c2_dv23[2];
  int32_T c2_i230;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv23, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c2_i230 = 0; c2_i230 < 2; c2_i230++) {
    c2_y[c2_i230] = c2_dv23[c2_i230];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_h_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[23])
{
  real_T c2_dv24[23];
  int32_T c2_i231;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv24, 1, 0, 0U, 1, 0U, 2, 23,
                1);
  for (c2_i231 = 0; c2_i231 < 23; c2_i231++) {
    c2_y[c2_i231] = c2_dv24[c2_i231];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_CONFIG;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_struct_HZPQIAn1UwxVqJtlH5llbD c2_y;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_b_CONFIG = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_CONFIG), &c2_thisId,
                        &c2_y);
  sf_mex_destroy(&c2_b_CONFIG);
  *(c2_struct_HZPQIAn1UwxVqJtlH5llbD *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i232;
  real_T c2_b_inData[3];
  int32_T c2_i233;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i232 = 0; c2_i232 < 3; c2_i232++) {
    c2_b_inData[c2_i232] = (*(real_T (*)[3])c2_inData)[c2_i232];
  }

  for (c2_i233 = 0; c2_i233 < 3; c2_i233++) {
    c2_u[c2_i233] = c2_b_inData[c2_i233];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i234;
  real_T c2_b_inData[12];
  int32_T c2_i235;
  real_T c2_u[12];
  const mxArray *c2_y = NULL;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i234 = 0; c2_i234 < 12; c2_i234++) {
    c2_b_inData[c2_i234] = (*(real_T (*)[12])c2_inData)[c2_i234];
  }

  for (c2_i235 = 0; c2_i235 < 12; c2_i235++) {
    c2_u[c2_i235] = c2_b_inData[c2_i235];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i236;
  int32_T c2_i237;
  int32_T c2_i238;
  real_T c2_b_inData[9];
  int32_T c2_i239;
  int32_T c2_i240;
  int32_T c2_i241;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i236 = 0;
  for (c2_i237 = 0; c2_i237 < 3; c2_i237++) {
    for (c2_i238 = 0; c2_i238 < 3; c2_i238++) {
      c2_b_inData[c2_i238 + c2_i236] = (*(real_T (*)[9])c2_inData)[c2_i238 +
        c2_i236];
    }

    c2_i236 += 3;
  }

  c2_i239 = 0;
  for (c2_i240 = 0; c2_i240 < 3; c2_i240++) {
    for (c2_i241 = 0; c2_i241 < 3; c2_i241++) {
      c2_u[c2_i241 + c2_i239] = c2_b_inData[c2_i241 + c2_i239];
    }

    c2_i239 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_i_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv25[9];
  int32_T c2_i242;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv25, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i242 = 0; c2_i242 < 9; c2_i242++) {
    c2_y[c2_i242] = c2_dv25[c2_i242];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_R;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i243;
  int32_T c2_i244;
  int32_T c2_i245;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_R = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_R), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_R);
  c2_i243 = 0;
  for (c2_i244 = 0; c2_i244 < 3; c2_i244++) {
    for (c2_i245 = 0; c2_i245 < 3; c2_i245++) {
      (*(real_T (*)[9])c2_outData)[c2_i245 + c2_i243] = c2_y[c2_i245 + c2_i243];
    }

    c2_i243 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_j_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv26[3];
  int32_T c2_i246;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv26, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i246 = 0; c2_i246 < 3; c2_i246++) {
    c2_y[c2_i246] = c2_dv26[c2_i246];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_rollPitchYaw;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i247;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_rollPitchYaw = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_rollPitchYaw), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_rollPitchYaw);
  for (c2_i247 = 0; c2_i247 < 3; c2_i247++) {
    (*(real_T (*)[3])c2_outData)[c2_i247] = c2_y[c2_i247];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_k_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[12])
{
  real_T c2_dv27[12];
  int32_T c2_i248;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv27, 1, 0, 0U, 1, 0U, 1, 12);
  for (c2_i248 = 0; c2_i248 < 12; c2_i248++) {
    c2_y[c2_i248] = c2_dv27[c2_i248];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_inertial;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[12];
  int32_T c2_i249;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_inertial = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_inertial), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_inertial);
  for (c2_i249 = 0; c2_i249 < 12; c2_i249++) {
    (*(real_T (*)[12])c2_outData)[c2_i249] = c2_y[c2_i249];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[147];
  const mxArray *c2_m2 = NULL;
  int32_T c2_i250;
  c2_ResolvedFunctionInfo *c2_r1;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  c2_b_info_helper(c2_info);
  c2_c_info_helper(c2_info);
  sf_mex_assign(&c2_m2, sf_mex_createstruct("nameCaptureInfo", 1, 147), FALSE);
  for (c2_i250 = 0; c2_i250 < 147; c2_i250++) {
    c2_r1 = &c2_info[c2_i250];
    sf_mex_addfield(c2_m2, sf_mex_create("nameCaptureInfo", c2_r1->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r1->context)), "context", "nameCaptureInfo",
                    c2_i250);
    sf_mex_addfield(c2_m2, sf_mex_create("nameCaptureInfo", c2_r1->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r1->name)), "name", "nameCaptureInfo", c2_i250);
    sf_mex_addfield(c2_m2, sf_mex_create("nameCaptureInfo", c2_r1->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r1->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i250);
    sf_mex_addfield(c2_m2, sf_mex_create("nameCaptureInfo", c2_r1->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r1->resolved)), "resolved", "nameCaptureInfo",
                    c2_i250);
    sf_mex_addfield(c2_m2, sf_mex_create("nameCaptureInfo", &c2_r1->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i250);
    sf_mex_addfield(c2_m2, sf_mex_create("nameCaptureInfo", &c2_r1->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i250);
    sf_mex_addfield(c2_m2, sf_mex_create("nameCaptureInfo", &c2_r1->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i250);
    sf_mex_addfield(c2_m2, sf_mex_create("nameCaptureInfo", &c2_r1->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i250);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m2, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[147])
{
  c2_info[0].context = "";
  c2_info[0].name = "fromBaseToWorldWithImu";
  c2_info[0].dominantType = "struct";
  c2_info[0].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[0].fileTimeLo = 1498554660U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[1].name = "mtimes";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[1].fileTimeLo = 1289519692U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[2].name = "mrdivide";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[2].fileTimeLo = 1357951548U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 1319729966U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[3].name = "rdivide";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[3].fileTimeLo = 1346510388U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[4].name = "eml_scalexp_compatible";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c2_info[4].fileTimeLo = 1286818796U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[5].name = "eml_div";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[5].fileTimeLo = 1313347810U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[6].name = "rotz";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c2_info[6].fileTimeLo = 1495631764U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c2_info[7].name = "cos";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[7].fileTimeLo = 1343830372U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[8].name = "eml_scalar_cos";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c2_info[8].fileTimeLo = 1286818722U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c2_info[9].name = "sin";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[9].fileTimeLo = 1343830386U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[10].name = "eml_scalar_sin";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c2_info[10].fileTimeLo = 1286818736U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[11].name = "roty";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c2_info[11].fileTimeLo = 1495631764U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c2_info[12].name = "cos";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[12].fileTimeLo = 1343830372U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c2_info[13].name = "sin";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[13].fileTimeLo = 1343830386U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[14].name = "eml_index_class";
  c2_info[14].dominantType = "";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[14].fileTimeLo = 1323170578U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[15].name = "eml_scalar_eg";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[15].fileTimeLo = 1286818796U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[16].name = "eml_xgemm";
  c2_info[16].dominantType = "char";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[16].fileTimeLo = 1299076772U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[17].name = "eml_blas_inline";
  c2_info[17].dominantType = "";
  c2_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[17].fileTimeLo = 1299076768U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[18].name = "mtimes";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[18].fileTimeLo = 1289519692U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[19].name = "eml_index_class";
  c2_info[19].dominantType = "";
  c2_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[19].fileTimeLo = 1323170578U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[20].name = "eml_scalar_eg";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[20].fileTimeLo = 1286818796U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[21].name = "eml_refblas_xgemm";
  c2_info[21].dominantType = "char";
  c2_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[21].fileTimeLo = 1299076774U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[22].name = "rotx";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c2_info[22].fileTimeLo = 1495631764U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c2_info[23].name = "cos";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[23].fileTimeLo = 1343830372U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c2_info[24].name = "sin";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[24].fileTimeLo = 1343830386U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[25].name = "rollPitchYawFromRotation";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c2_info[25].fileTimeLo = 1495631764U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c2_info[26].name = "asin";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c2_info[26].fileTimeLo = 1343830370U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c2_info[27].name = "eml_error";
  c2_info[27].dominantType = "char";
  c2_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[27].fileTimeLo = 1343830358U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c2_info[28].name = "eml_scalar_asin";
  c2_info[28].dominantType = "double";
  c2_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_asin.m";
  c2_info[28].fileTimeLo = 1343830376U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c2_info[29].name = "atan2";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[29].fileTimeLo = 1343830372U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[30].name = "eml_scalar_eg";
  c2_info[30].dominantType = "double";
  c2_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[30].fileTimeLo = 1286818796U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[31].name = "eml_scalexp_alloc";
  c2_info[31].dominantType = "double";
  c2_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[31].fileTimeLo = 1352424860U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c2_info[32].name = "eml_scalar_atan2";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m";
  c2_info[32].fileTimeLo = 1286818720U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[33].name = "correctIMU";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c2_info[33].fileTimeLo = 1495631764U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c2_info[34].name = "mtimes";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[34].fileTimeLo = 1289519692U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 0U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c2_info[35].name = "mrdivide";
  c2_info[35].dominantType = "double";
  c2_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[35].fileTimeLo = 1357951548U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 1319729966U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c2_info[36].name = "evalDHMatrix";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c2_info[36].fileTimeLo = 1495631765U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 0U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c2_info[37].name = "cos";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[37].fileTimeLo = 1343830372U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c2_info[38].name = "sin";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[38].fileTimeLo = 1343830386U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c2_info[39].name = "mtimes";
  c2_info[39].dominantType = "double";
  c2_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[39].fileTimeLo = 1289519692U;
  c2_info[39].fileTimeHi = 0U;
  c2_info[39].mFileTimeLo = 0U;
  c2_info[39].mFileTimeHi = 0U;
  c2_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[40].name = "mldivide";
  c2_info[40].dominantType = "double";
  c2_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[40].fileTimeLo = 1357951548U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 1319729966U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[41].name = "eml_lusolve";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c2_info[41].fileTimeLo = 1309451196U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
  c2_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c2_info[42].name = "eml_index_class";
  c2_info[42].dominantType = "";
  c2_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[42].fileTimeLo = 1323170578U;
  c2_info[42].fileTimeHi = 0U;
  c2_info[42].mFileTimeLo = 0U;
  c2_info[42].mFileTimeHi = 0U;
  c2_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[43].name = "eml_index_class";
  c2_info[43].dominantType = "";
  c2_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[43].fileTimeLo = 1323170578U;
  c2_info[43].fileTimeHi = 0U;
  c2_info[43].mFileTimeLo = 0U;
  c2_info[43].mFileTimeHi = 0U;
  c2_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[44].name = "eml_xgetrf";
  c2_info[44].dominantType = "double";
  c2_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[44].fileTimeLo = 1286818806U;
  c2_info[44].fileTimeHi = 0U;
  c2_info[44].mFileTimeLo = 0U;
  c2_info[44].mFileTimeHi = 0U;
  c2_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[45].name = "eml_lapack_xgetrf";
  c2_info[45].dominantType = "double";
  c2_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[45].fileTimeLo = 1286818810U;
  c2_info[45].fileTimeHi = 0U;
  c2_info[45].mFileTimeLo = 0U;
  c2_info[45].mFileTimeHi = 0U;
  c2_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[46].name = "eml_matlab_zgetrf";
  c2_info[46].dominantType = "double";
  c2_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[46].fileTimeLo = 1302688994U;
  c2_info[46].fileTimeHi = 0U;
  c2_info[46].mFileTimeLo = 0U;
  c2_info[46].mFileTimeHi = 0U;
  c2_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[47].name = "realmin";
  c2_info[47].dominantType = "char";
  c2_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[47].fileTimeLo = 1307651242U;
  c2_info[47].fileTimeHi = 0U;
  c2_info[47].mFileTimeLo = 0U;
  c2_info[47].mFileTimeHi = 0U;
  c2_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[48].name = "eml_realmin";
  c2_info[48].dominantType = "char";
  c2_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[48].fileTimeLo = 1307651244U;
  c2_info[48].fileTimeHi = 0U;
  c2_info[48].mFileTimeLo = 0U;
  c2_info[48].mFileTimeHi = 0U;
  c2_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[49].name = "eml_float_model";
  c2_info[49].dominantType = "char";
  c2_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[49].fileTimeLo = 1326727996U;
  c2_info[49].fileTimeHi = 0U;
  c2_info[49].mFileTimeLo = 0U;
  c2_info[49].mFileTimeHi = 0U;
  c2_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[50].name = "eps";
  c2_info[50].dominantType = "char";
  c2_info[50].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[50].fileTimeLo = 1326727996U;
  c2_info[50].fileTimeHi = 0U;
  c2_info[50].mFileTimeLo = 0U;
  c2_info[50].mFileTimeHi = 0U;
  c2_info[51].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[51].name = "eml_is_float_class";
  c2_info[51].dominantType = "char";
  c2_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[51].fileTimeLo = 1286818782U;
  c2_info[51].fileTimeHi = 0U;
  c2_info[51].mFileTimeLo = 0U;
  c2_info[51].mFileTimeHi = 0U;
  c2_info[52].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[52].name = "eml_eps";
  c2_info[52].dominantType = "char";
  c2_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[52].fileTimeLo = 1326727996U;
  c2_info[52].fileTimeHi = 0U;
  c2_info[52].mFileTimeLo = 0U;
  c2_info[52].mFileTimeHi = 0U;
  c2_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[53].name = "eml_float_model";
  c2_info[53].dominantType = "char";
  c2_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[53].fileTimeLo = 1326727996U;
  c2_info[53].fileTimeHi = 0U;
  c2_info[53].mFileTimeLo = 0U;
  c2_info[53].mFileTimeHi = 0U;
  c2_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[54].name = "min";
  c2_info[54].dominantType = "coder.internal.indexInt";
  c2_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[54].fileTimeLo = 1311255318U;
  c2_info[54].fileTimeHi = 0U;
  c2_info[54].mFileTimeLo = 0U;
  c2_info[54].mFileTimeHi = 0U;
  c2_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[55].name = "eml_min_or_max";
  c2_info[55].dominantType = "char";
  c2_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[55].fileTimeLo = 1334071490U;
  c2_info[55].fileTimeHi = 0U;
  c2_info[55].mFileTimeLo = 0U;
  c2_info[55].mFileTimeHi = 0U;
  c2_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[56].name = "eml_scalar_eg";
  c2_info[56].dominantType = "coder.internal.indexInt";
  c2_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[56].fileTimeLo = 1286818796U;
  c2_info[56].fileTimeHi = 0U;
  c2_info[56].mFileTimeLo = 0U;
  c2_info[56].mFileTimeHi = 0U;
  c2_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[57].name = "eml_scalexp_alloc";
  c2_info[57].dominantType = "coder.internal.indexInt";
  c2_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[57].fileTimeLo = 1352424860U;
  c2_info[57].fileTimeHi = 0U;
  c2_info[57].mFileTimeLo = 0U;
  c2_info[57].mFileTimeHi = 0U;
  c2_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[58].name = "eml_index_class";
  c2_info[58].dominantType = "";
  c2_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[58].fileTimeLo = 1323170578U;
  c2_info[58].fileTimeHi = 0U;
  c2_info[58].mFileTimeLo = 0U;
  c2_info[58].mFileTimeHi = 0U;
  c2_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[59].name = "eml_scalar_eg";
  c2_info[59].dominantType = "coder.internal.indexInt";
  c2_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[59].fileTimeLo = 1286818796U;
  c2_info[59].fileTimeHi = 0U;
  c2_info[59].mFileTimeLo = 0U;
  c2_info[59].mFileTimeHi = 0U;
  c2_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[60].name = "colon";
  c2_info[60].dominantType = "double";
  c2_info[60].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[60].fileTimeLo = 1348191928U;
  c2_info[60].fileTimeHi = 0U;
  c2_info[60].mFileTimeLo = 0U;
  c2_info[60].mFileTimeHi = 0U;
  c2_info[61].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[61].name = "colon";
  c2_info[61].dominantType = "double";
  c2_info[61].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[61].fileTimeLo = 1348191928U;
  c2_info[61].fileTimeHi = 0U;
  c2_info[61].mFileTimeLo = 0U;
  c2_info[61].mFileTimeHi = 0U;
  c2_info[62].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[62].name = "floor";
  c2_info[62].dominantType = "double";
  c2_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[62].fileTimeLo = 1343830380U;
  c2_info[62].fileTimeHi = 0U;
  c2_info[62].mFileTimeLo = 0U;
  c2_info[62].mFileTimeHi = 0U;
  c2_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[63].name = "eml_scalar_floor";
  c2_info[63].dominantType = "double";
  c2_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[63].fileTimeLo = 1286818726U;
  c2_info[63].fileTimeHi = 0U;
  c2_info[63].mFileTimeLo = 0U;
  c2_info[63].mFileTimeHi = 0U;
}

static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[147])
{
  c2_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[64].name = "intmin";
  c2_info[64].dominantType = "char";
  c2_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[64].fileTimeLo = 1311255318U;
  c2_info[64].fileTimeHi = 0U;
  c2_info[64].mFileTimeLo = 0U;
  c2_info[64].mFileTimeHi = 0U;
  c2_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[65].name = "intmax";
  c2_info[65].dominantType = "char";
  c2_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[65].fileTimeLo = 1311255316U;
  c2_info[65].fileTimeHi = 0U;
  c2_info[65].mFileTimeLo = 0U;
  c2_info[65].mFileTimeHi = 0U;
  c2_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[66].name = "intmin";
  c2_info[66].dominantType = "char";
  c2_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[66].fileTimeLo = 1311255318U;
  c2_info[66].fileTimeHi = 0U;
  c2_info[66].mFileTimeLo = 0U;
  c2_info[66].mFileTimeHi = 0U;
  c2_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[67].name = "intmax";
  c2_info[67].dominantType = "char";
  c2_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[67].fileTimeLo = 1311255316U;
  c2_info[67].fileTimeHi = 0U;
  c2_info[67].mFileTimeLo = 0U;
  c2_info[67].mFileTimeHi = 0U;
  c2_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[68].name = "eml_isa_uint";
  c2_info[68].dominantType = "coder.internal.indexInt";
  c2_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[68].fileTimeLo = 1286818784U;
  c2_info[68].fileTimeHi = 0U;
  c2_info[68].mFileTimeLo = 0U;
  c2_info[68].mFileTimeHi = 0U;
  c2_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[69].name = "eml_unsigned_class";
  c2_info[69].dominantType = "char";
  c2_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[69].fileTimeLo = 1323170580U;
  c2_info[69].fileTimeHi = 0U;
  c2_info[69].mFileTimeLo = 0U;
  c2_info[69].mFileTimeHi = 0U;
  c2_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[70].name = "eml_index_class";
  c2_info[70].dominantType = "";
  c2_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[70].fileTimeLo = 1323170578U;
  c2_info[70].fileTimeHi = 0U;
  c2_info[70].mFileTimeLo = 0U;
  c2_info[70].mFileTimeHi = 0U;
  c2_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[71].name = "eml_index_class";
  c2_info[71].dominantType = "";
  c2_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[71].fileTimeLo = 1323170578U;
  c2_info[71].fileTimeHi = 0U;
  c2_info[71].mFileTimeLo = 0U;
  c2_info[71].mFileTimeHi = 0U;
  c2_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[72].name = "intmax";
  c2_info[72].dominantType = "char";
  c2_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[72].fileTimeLo = 1311255316U;
  c2_info[72].fileTimeHi = 0U;
  c2_info[72].mFileTimeLo = 0U;
  c2_info[72].mFileTimeHi = 0U;
  c2_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[73].name = "eml_isa_uint";
  c2_info[73].dominantType = "coder.internal.indexInt";
  c2_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[73].fileTimeLo = 1286818784U;
  c2_info[73].fileTimeHi = 0U;
  c2_info[73].mFileTimeLo = 0U;
  c2_info[73].mFileTimeHi = 0U;
  c2_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[74].name = "eml_index_plus";
  c2_info[74].dominantType = "double";
  c2_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[74].fileTimeLo = 1286818778U;
  c2_info[74].fileTimeHi = 0U;
  c2_info[74].mFileTimeLo = 0U;
  c2_info[74].mFileTimeHi = 0U;
  c2_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[75].name = "eml_index_class";
  c2_info[75].dominantType = "";
  c2_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[75].fileTimeLo = 1323170578U;
  c2_info[75].fileTimeHi = 0U;
  c2_info[75].mFileTimeLo = 0U;
  c2_info[75].mFileTimeHi = 0U;
  c2_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c2_info[76].name = "eml_int_forloop_overflow_check";
  c2_info[76].dominantType = "";
  c2_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[76].fileTimeLo = 1346510340U;
  c2_info[76].fileTimeHi = 0U;
  c2_info[76].mFileTimeLo = 0U;
  c2_info[76].mFileTimeHi = 0U;
  c2_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[77].name = "intmax";
  c2_info[77].dominantType = "char";
  c2_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[77].fileTimeLo = 1311255316U;
  c2_info[77].fileTimeHi = 0U;
  c2_info[77].mFileTimeLo = 0U;
  c2_info[77].mFileTimeHi = 0U;
  c2_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[78].name = "eml_index_class";
  c2_info[78].dominantType = "";
  c2_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[78].fileTimeLo = 1323170578U;
  c2_info[78].fileTimeHi = 0U;
  c2_info[78].mFileTimeLo = 0U;
  c2_info[78].mFileTimeHi = 0U;
  c2_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[79].name = "eml_index_plus";
  c2_info[79].dominantType = "double";
  c2_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[79].fileTimeLo = 1286818778U;
  c2_info[79].fileTimeHi = 0U;
  c2_info[79].mFileTimeLo = 0U;
  c2_info[79].mFileTimeHi = 0U;
  c2_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[80].name = "eml_int_forloop_overflow_check";
  c2_info[80].dominantType = "";
  c2_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[80].fileTimeLo = 1346510340U;
  c2_info[80].fileTimeHi = 0U;
  c2_info[80].mFileTimeLo = 0U;
  c2_info[80].mFileTimeHi = 0U;
  c2_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[81].name = "eml_index_minus";
  c2_info[81].dominantType = "double";
  c2_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[81].fileTimeLo = 1286818778U;
  c2_info[81].fileTimeHi = 0U;
  c2_info[81].mFileTimeLo = 0U;
  c2_info[81].mFileTimeHi = 0U;
  c2_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[82].name = "eml_index_class";
  c2_info[82].dominantType = "";
  c2_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[82].fileTimeLo = 1323170578U;
  c2_info[82].fileTimeHi = 0U;
  c2_info[82].mFileTimeLo = 0U;
  c2_info[82].mFileTimeHi = 0U;
  c2_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[83].name = "eml_index_minus";
  c2_info[83].dominantType = "coder.internal.indexInt";
  c2_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[83].fileTimeLo = 1286818778U;
  c2_info[83].fileTimeHi = 0U;
  c2_info[83].mFileTimeLo = 0U;
  c2_info[83].mFileTimeHi = 0U;
  c2_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[84].name = "eml_index_times";
  c2_info[84].dominantType = "coder.internal.indexInt";
  c2_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[84].fileTimeLo = 1286818780U;
  c2_info[84].fileTimeHi = 0U;
  c2_info[84].mFileTimeLo = 0U;
  c2_info[84].mFileTimeHi = 0U;
  c2_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[85].name = "eml_index_class";
  c2_info[85].dominantType = "";
  c2_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[85].fileTimeLo = 1323170578U;
  c2_info[85].fileTimeHi = 0U;
  c2_info[85].mFileTimeLo = 0U;
  c2_info[85].mFileTimeHi = 0U;
  c2_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[86].name = "eml_index_plus";
  c2_info[86].dominantType = "coder.internal.indexInt";
  c2_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[86].fileTimeLo = 1286818778U;
  c2_info[86].fileTimeHi = 0U;
  c2_info[86].mFileTimeLo = 0U;
  c2_info[86].mFileTimeHi = 0U;
  c2_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[87].name = "eml_ixamax";
  c2_info[87].dominantType = "double";
  c2_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[87].fileTimeLo = 1299076770U;
  c2_info[87].fileTimeHi = 0U;
  c2_info[87].mFileTimeLo = 0U;
  c2_info[87].mFileTimeHi = 0U;
  c2_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[88].name = "eml_blas_inline";
  c2_info[88].dominantType = "";
  c2_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[88].fileTimeLo = 1299076768U;
  c2_info[88].fileTimeHi = 0U;
  c2_info[88].mFileTimeLo = 0U;
  c2_info[88].mFileTimeHi = 0U;
  c2_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c2_info[89].name = "length";
  c2_info[89].dominantType = "double";
  c2_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[89].fileTimeLo = 1303146206U;
  c2_info[89].fileTimeHi = 0U;
  c2_info[89].mFileTimeLo = 0U;
  c2_info[89].mFileTimeHi = 0U;
  c2_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c2_info[90].name = "eml_index_class";
  c2_info[90].dominantType = "";
  c2_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[90].fileTimeLo = 1323170578U;
  c2_info[90].fileTimeHi = 0U;
  c2_info[90].mFileTimeLo = 0U;
  c2_info[90].mFileTimeHi = 0U;
  c2_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[91].name = "eml_index_class";
  c2_info[91].dominantType = "";
  c2_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[91].fileTimeLo = 1323170578U;
  c2_info[91].fileTimeHi = 0U;
  c2_info[91].mFileTimeLo = 0U;
  c2_info[91].mFileTimeHi = 0U;
  c2_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[92].name = "eml_refblas_ixamax";
  c2_info[92].dominantType = "double";
  c2_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[92].fileTimeLo = 1299076770U;
  c2_info[92].fileTimeHi = 0U;
  c2_info[92].mFileTimeLo = 0U;
  c2_info[92].mFileTimeHi = 0U;
  c2_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[93].name = "eml_index_class";
  c2_info[93].dominantType = "";
  c2_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[93].fileTimeLo = 1323170578U;
  c2_info[93].fileTimeHi = 0U;
  c2_info[93].mFileTimeLo = 0U;
  c2_info[93].mFileTimeHi = 0U;
  c2_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[94].name = "eml_xcabs1";
  c2_info[94].dominantType = "double";
  c2_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[94].fileTimeLo = 1286818706U;
  c2_info[94].fileTimeHi = 0U;
  c2_info[94].mFileTimeLo = 0U;
  c2_info[94].mFileTimeHi = 0U;
  c2_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[95].name = "abs";
  c2_info[95].dominantType = "double";
  c2_info[95].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[95].fileTimeLo = 1343830366U;
  c2_info[95].fileTimeHi = 0U;
  c2_info[95].mFileTimeLo = 0U;
  c2_info[95].mFileTimeHi = 0U;
  c2_info[96].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[96].name = "eml_scalar_abs";
  c2_info[96].dominantType = "double";
  c2_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[96].fileTimeLo = 1286818712U;
  c2_info[96].fileTimeHi = 0U;
  c2_info[96].mFileTimeLo = 0U;
  c2_info[96].mFileTimeHi = 0U;
  c2_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[97].name = "eml_int_forloop_overflow_check";
  c2_info[97].dominantType = "";
  c2_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[97].fileTimeLo = 1346510340U;
  c2_info[97].fileTimeHi = 0U;
  c2_info[97].mFileTimeLo = 0U;
  c2_info[97].mFileTimeHi = 0U;
  c2_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[98].name = "eml_index_plus";
  c2_info[98].dominantType = "coder.internal.indexInt";
  c2_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[98].fileTimeLo = 1286818778U;
  c2_info[98].fileTimeHi = 0U;
  c2_info[98].mFileTimeLo = 0U;
  c2_info[98].mFileTimeHi = 0U;
  c2_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[99].name = "eml_xswap";
  c2_info[99].dominantType = "double";
  c2_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[99].fileTimeLo = 1299076778U;
  c2_info[99].fileTimeHi = 0U;
  c2_info[99].mFileTimeLo = 0U;
  c2_info[99].mFileTimeHi = 0U;
  c2_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[100].name = "eml_blas_inline";
  c2_info[100].dominantType = "";
  c2_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[100].fileTimeLo = 1299076768U;
  c2_info[100].fileTimeHi = 0U;
  c2_info[100].mFileTimeLo = 0U;
  c2_info[100].mFileTimeHi = 0U;
  c2_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[101].name = "eml_index_class";
  c2_info[101].dominantType = "";
  c2_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[101].fileTimeLo = 1323170578U;
  c2_info[101].fileTimeHi = 0U;
  c2_info[101].mFileTimeLo = 0U;
  c2_info[101].mFileTimeHi = 0U;
  c2_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[102].name = "eml_refblas_xswap";
  c2_info[102].dominantType = "double";
  c2_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[102].fileTimeLo = 1299076786U;
  c2_info[102].fileTimeHi = 0U;
  c2_info[102].mFileTimeLo = 0U;
  c2_info[102].mFileTimeHi = 0U;
  c2_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[103].name = "eml_index_class";
  c2_info[103].dominantType = "";
  c2_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[103].fileTimeLo = 1323170578U;
  c2_info[103].fileTimeHi = 0U;
  c2_info[103].mFileTimeLo = 0U;
  c2_info[103].mFileTimeHi = 0U;
  c2_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[104].name = "abs";
  c2_info[104].dominantType = "coder.internal.indexInt";
  c2_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[104].fileTimeLo = 1343830366U;
  c2_info[104].fileTimeHi = 0U;
  c2_info[104].mFileTimeLo = 0U;
  c2_info[104].mFileTimeHi = 0U;
  c2_info[105].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[105].name = "eml_scalar_abs";
  c2_info[105].dominantType = "coder.internal.indexInt";
  c2_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[105].fileTimeLo = 1286818712U;
  c2_info[105].fileTimeHi = 0U;
  c2_info[105].mFileTimeLo = 0U;
  c2_info[105].mFileTimeHi = 0U;
  c2_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[106].name = "eml_int_forloop_overflow_check";
  c2_info[106].dominantType = "";
  c2_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[106].fileTimeLo = 1346510340U;
  c2_info[106].fileTimeHi = 0U;
  c2_info[106].mFileTimeLo = 0U;
  c2_info[106].mFileTimeHi = 0U;
  c2_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[107].name = "eml_index_plus";
  c2_info[107].dominantType = "coder.internal.indexInt";
  c2_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[107].fileTimeLo = 1286818778U;
  c2_info[107].fileTimeHi = 0U;
  c2_info[107].mFileTimeLo = 0U;
  c2_info[107].mFileTimeHi = 0U;
  c2_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[108].name = "eml_div";
  c2_info[108].dominantType = "double";
  c2_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[108].fileTimeLo = 1313347810U;
  c2_info[108].fileTimeHi = 0U;
  c2_info[108].mFileTimeLo = 0U;
  c2_info[108].mFileTimeHi = 0U;
  c2_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[109].name = "eml_xgeru";
  c2_info[109].dominantType = "double";
  c2_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[109].fileTimeLo = 1299076774U;
  c2_info[109].fileTimeHi = 0U;
  c2_info[109].mFileTimeLo = 0U;
  c2_info[109].mFileTimeHi = 0U;
  c2_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[110].name = "eml_blas_inline";
  c2_info[110].dominantType = "";
  c2_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[110].fileTimeLo = 1299076768U;
  c2_info[110].fileTimeHi = 0U;
  c2_info[110].mFileTimeLo = 0U;
  c2_info[110].mFileTimeHi = 0U;
  c2_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[111].name = "eml_xger";
  c2_info[111].dominantType = "double";
  c2_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[111].fileTimeLo = 1299076774U;
  c2_info[111].fileTimeHi = 0U;
  c2_info[111].mFileTimeLo = 0U;
  c2_info[111].mFileTimeHi = 0U;
  c2_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[112].name = "eml_blas_inline";
  c2_info[112].dominantType = "";
  c2_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[112].fileTimeLo = 1299076768U;
  c2_info[112].fileTimeHi = 0U;
  c2_info[112].mFileTimeLo = 0U;
  c2_info[112].mFileTimeHi = 0U;
  c2_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[113].name = "intmax";
  c2_info[113].dominantType = "char";
  c2_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[113].fileTimeLo = 1311255316U;
  c2_info[113].fileTimeHi = 0U;
  c2_info[113].mFileTimeLo = 0U;
  c2_info[113].mFileTimeHi = 0U;
  c2_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[114].name = "min";
  c2_info[114].dominantType = "double";
  c2_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[114].fileTimeLo = 1311255318U;
  c2_info[114].fileTimeHi = 0U;
  c2_info[114].mFileTimeLo = 0U;
  c2_info[114].mFileTimeHi = 0U;
  c2_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[115].name = "eml_scalar_eg";
  c2_info[115].dominantType = "double";
  c2_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[115].fileTimeLo = 1286818796U;
  c2_info[115].fileTimeHi = 0U;
  c2_info[115].mFileTimeLo = 0U;
  c2_info[115].mFileTimeHi = 0U;
  c2_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[116].name = "eml_scalexp_alloc";
  c2_info[116].dominantType = "double";
  c2_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[116].fileTimeLo = 1352424860U;
  c2_info[116].fileTimeHi = 0U;
  c2_info[116].mFileTimeLo = 0U;
  c2_info[116].mFileTimeHi = 0U;
  c2_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[117].name = "eml_scalar_eg";
  c2_info[117].dominantType = "double";
  c2_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[117].fileTimeLo = 1286818796U;
  c2_info[117].fileTimeHi = 0U;
  c2_info[117].mFileTimeLo = 0U;
  c2_info[117].mFileTimeHi = 0U;
  c2_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[118].name = "mtimes";
  c2_info[118].dominantType = "double";
  c2_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[118].fileTimeLo = 1289519692U;
  c2_info[118].fileTimeHi = 0U;
  c2_info[118].mFileTimeLo = 0U;
  c2_info[118].mFileTimeHi = 0U;
  c2_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[119].name = "eml_index_class";
  c2_info[119].dominantType = "";
  c2_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[119].fileTimeLo = 1323170578U;
  c2_info[119].fileTimeHi = 0U;
  c2_info[119].mFileTimeLo = 0U;
  c2_info[119].mFileTimeHi = 0U;
  c2_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[120].name = "eml_refblas_xger";
  c2_info[120].dominantType = "double";
  c2_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[120].fileTimeLo = 1299076776U;
  c2_info[120].fileTimeHi = 0U;
  c2_info[120].mFileTimeLo = 0U;
  c2_info[120].mFileTimeHi = 0U;
  c2_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[121].name = "eml_refblas_xgerx";
  c2_info[121].dominantType = "char";
  c2_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[121].fileTimeLo = 1299076778U;
  c2_info[121].fileTimeHi = 0U;
  c2_info[121].mFileTimeLo = 0U;
  c2_info[121].mFileTimeHi = 0U;
  c2_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[122].name = "eml_index_class";
  c2_info[122].dominantType = "";
  c2_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[122].fileTimeLo = 1323170578U;
  c2_info[122].fileTimeHi = 0U;
  c2_info[122].mFileTimeLo = 0U;
  c2_info[122].mFileTimeHi = 0U;
  c2_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[123].name = "abs";
  c2_info[123].dominantType = "coder.internal.indexInt";
  c2_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[123].fileTimeLo = 1343830366U;
  c2_info[123].fileTimeHi = 0U;
  c2_info[123].mFileTimeLo = 0U;
  c2_info[123].mFileTimeHi = 0U;
  c2_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[124].name = "eml_index_minus";
  c2_info[124].dominantType = "double";
  c2_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[124].fileTimeLo = 1286818778U;
  c2_info[124].fileTimeHi = 0U;
  c2_info[124].mFileTimeLo = 0U;
  c2_info[124].mFileTimeHi = 0U;
  c2_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[125].name = "eml_int_forloop_overflow_check";
  c2_info[125].dominantType = "";
  c2_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[125].fileTimeLo = 1346510340U;
  c2_info[125].fileTimeHi = 0U;
  c2_info[125].mFileTimeLo = 0U;
  c2_info[125].mFileTimeHi = 0U;
  c2_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[126].name = "eml_index_plus";
  c2_info[126].dominantType = "double";
  c2_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[126].fileTimeLo = 1286818778U;
  c2_info[126].fileTimeHi = 0U;
  c2_info[126].mFileTimeLo = 0U;
  c2_info[126].mFileTimeHi = 0U;
  c2_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[127].name = "eml_index_plus";
  c2_info[127].dominantType = "coder.internal.indexInt";
  c2_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[127].fileTimeLo = 1286818778U;
  c2_info[127].fileTimeHi = 0U;
  c2_info[127].mFileTimeLo = 0U;
  c2_info[127].mFileTimeHi = 0U;
}

static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[147])
{
  c2_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[128].name = "eml_scalar_eg";
  c2_info[128].dominantType = "double";
  c2_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[128].fileTimeLo = 1286818796U;
  c2_info[128].fileTimeHi = 0U;
  c2_info[128].mFileTimeLo = 0U;
  c2_info[128].mFileTimeHi = 0U;
  c2_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[129].name = "eml_int_forloop_overflow_check";
  c2_info[129].dominantType = "";
  c2_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[129].fileTimeLo = 1346510340U;
  c2_info[129].fileTimeHi = 0U;
  c2_info[129].mFileTimeLo = 0U;
  c2_info[129].mFileTimeHi = 0U;
  c2_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[130].name = "eml_xtrsm";
  c2_info[130].dominantType = "char";
  c2_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[130].fileTimeLo = 1299076778U;
  c2_info[130].fileTimeHi = 0U;
  c2_info[130].mFileTimeLo = 0U;
  c2_info[130].mFileTimeHi = 0U;
  c2_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[131].name = "eml_blas_inline";
  c2_info[131].dominantType = "";
  c2_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[131].fileTimeLo = 1299076768U;
  c2_info[131].fileTimeHi = 0U;
  c2_info[131].mFileTimeLo = 0U;
  c2_info[131].mFileTimeHi = 0U;
  c2_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c2_info[132].name = "mtimes";
  c2_info[132].dominantType = "double";
  c2_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[132].fileTimeLo = 1289519692U;
  c2_info[132].fileTimeHi = 0U;
  c2_info[132].mFileTimeLo = 0U;
  c2_info[132].mFileTimeHi = 0U;
  c2_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[133].name = "eml_index_class";
  c2_info[133].dominantType = "";
  c2_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[133].fileTimeLo = 1323170578U;
  c2_info[133].fileTimeHi = 0U;
  c2_info[133].mFileTimeLo = 0U;
  c2_info[133].mFileTimeHi = 0U;
  c2_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[134].name = "eml_scalar_eg";
  c2_info[134].dominantType = "double";
  c2_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[134].fileTimeLo = 1286818796U;
  c2_info[134].fileTimeHi = 0U;
  c2_info[134].mFileTimeLo = 0U;
  c2_info[134].mFileTimeHi = 0U;
  c2_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[135].name = "eml_refblas_xtrsm";
  c2_info[135].dominantType = "char";
  c2_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[135].fileTimeLo = 1299076786U;
  c2_info[135].fileTimeHi = 0U;
  c2_info[135].mFileTimeLo = 0U;
  c2_info[135].mFileTimeHi = 0U;
  c2_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[136].name = "eml_scalar_eg";
  c2_info[136].dominantType = "double";
  c2_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[136].fileTimeLo = 1286818796U;
  c2_info[136].fileTimeHi = 0U;
  c2_info[136].mFileTimeLo = 0U;
  c2_info[136].mFileTimeHi = 0U;
  c2_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[137].name = "eml_index_minus";
  c2_info[137].dominantType = "double";
  c2_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[137].fileTimeLo = 1286818778U;
  c2_info[137].fileTimeHi = 0U;
  c2_info[137].mFileTimeLo = 0U;
  c2_info[137].mFileTimeHi = 0U;
  c2_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[138].name = "eml_index_class";
  c2_info[138].dominantType = "";
  c2_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[138].fileTimeLo = 1323170578U;
  c2_info[138].fileTimeHi = 0U;
  c2_info[138].mFileTimeLo = 0U;
  c2_info[138].mFileTimeHi = 0U;
  c2_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[139].name = "eml_int_forloop_overflow_check";
  c2_info[139].dominantType = "";
  c2_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[139].fileTimeLo = 1346510340U;
  c2_info[139].fileTimeHi = 0U;
  c2_info[139].mFileTimeLo = 0U;
  c2_info[139].mFileTimeHi = 0U;
  c2_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[140].name = "eml_index_times";
  c2_info[140].dominantType = "coder.internal.indexInt";
  c2_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[140].fileTimeLo = 1286818780U;
  c2_info[140].fileTimeHi = 0U;
  c2_info[140].mFileTimeLo = 0U;
  c2_info[140].mFileTimeHi = 0U;
  c2_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[141].name = "eml_index_plus";
  c2_info[141].dominantType = "coder.internal.indexInt";
  c2_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[141].fileTimeLo = 1286818778U;
  c2_info[141].fileTimeHi = 0U;
  c2_info[141].mFileTimeLo = 0U;
  c2_info[141].mFileTimeHi = 0U;
  c2_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[142].name = "eml_index_plus";
  c2_info[142].dominantType = "double";
  c2_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[142].fileTimeLo = 1286818778U;
  c2_info[142].fileTimeHi = 0U;
  c2_info[142].mFileTimeLo = 0U;
  c2_info[142].mFileTimeHi = 0U;
  c2_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[143].name = "intmin";
  c2_info[143].dominantType = "char";
  c2_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[143].fileTimeLo = 1311255318U;
  c2_info[143].fileTimeHi = 0U;
  c2_info[143].mFileTimeLo = 0U;
  c2_info[143].mFileTimeHi = 0U;
  c2_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[144].name = "eml_div";
  c2_info[144].dominantType = "double";
  c2_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[144].fileTimeLo = 1313347810U;
  c2_info[144].fileTimeHi = 0U;
  c2_info[144].mFileTimeLo = 0U;
  c2_info[144].mFileTimeHi = 0U;
  c2_info[145].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c2_info[145].name = "mldivide";
  c2_info[145].dominantType = "double";
  c2_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[145].fileTimeLo = 1357951548U;
  c2_info[145].fileTimeHi = 0U;
  c2_info[145].mFileTimeLo = 1319729966U;
  c2_info[145].mFileTimeHi = 0U;
  c2_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c2_info[146].name = "eml_warning";
  c2_info[146].dominantType = "char";
  c2_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[146].fileTimeLo = 1286818802U;
  c2_info[146].fileTimeHi = 0U;
  c2_info[146].mFileTimeLo = 0U;
  c2_info[146].mFileTimeHi = 0U;
}

static void c2_eml_scalar_eg(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c2_eml_error(SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
  int32_T c2_i251;
  static char_T c2_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  int32_T c2_i252;
  static char_T c2_cv1[4] = { 'a', 's', 'i', 'n' };

  char_T c2_b_u[4];
  const mxArray *c2_b_y = NULL;
  for (c2_i251 = 0; c2_i251 < 30; c2_i251++) {
    c2_u[c2_i251] = c2_cv0[c2_i251];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c2_i252 = 0; c2_i252 < 4; c2_i252++) {
    c2_b_u[c2_i252] = c2_cv1[c2_i252];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c2_y, 14, c2_b_y));
}

static real_T c2_atan2(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c2_y, real_T c2_x)
{
  real_T c2_b_y;
  real_T c2_b_x;
  c2_b_eml_scalar_eg(chartInstance);
  c2_b_y = c2_y;
  c2_b_x = c2_x;
  return muDoubleScalarAtan2(c2_b_y, c2_b_x);
}

static void c2_b_eml_scalar_eg(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c2_c_eml_scalar_eg(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c2_realmin(SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c2_eps(SFc2_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c2_eml_matlab_zgetrf(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_b_A[16], int32_T c2_ipiv[4],
  int32_T *c2_info)
{
  int32_T c2_i253;
  for (c2_i253 = 0; c2_i253 < 16; c2_i253++) {
    c2_b_A[c2_i253] = c2_A[c2_i253];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_b_A, c2_ipiv, c2_info);
}

static void c2_check_forloop_overflow_error
  (SFc2_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c2_overflow)
{
  int32_T c2_i254;
  static char_T c2_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i255;
  static char_T c2_cv3[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c2_b_u[23];
  const mxArray *c2_b_y = NULL;
  if (!c2_overflow) {
  } else {
    for (c2_i254 = 0; c2_i254 < 34; c2_i254++) {
      c2_u[c2_i254] = c2_cv2[c2_i254];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c2_i255 = 0; c2_i255 < 23; c2_i255++) {
      c2_b_u[c2_i255] = c2_cv3[c2_i255];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c2_y, 14, c2_b_y));
  }
}

static void c2_eml_xger(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0,
  real_T c2_A[16], int32_T c2_ia0, real_T c2_b_A[16])
{
  int32_T c2_i256;
  for (c2_i256 = 0; c2_i256 < 16; c2_i256++) {
    c2_b_A[c2_i256] = c2_A[c2_i256];
  }

  c2_b_eml_xger(chartInstance, c2_m, c2_n, c2_alpha1, c2_ix0, c2_iy0, c2_b_A,
                c2_ia0);
}

static void c2_eml_xtrsm(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c2_A[16], real_T c2_B[16], real_T c2_b_B[16])
{
  int32_T c2_i257;
  int32_T c2_i258;
  real_T c2_b_A[16];
  for (c2_i257 = 0; c2_i257 < 16; c2_i257++) {
    c2_b_B[c2_i257] = c2_B[c2_i257];
  }

  for (c2_i258 = 0; c2_i258 < 16; c2_i258++) {
    c2_b_A[c2_i258] = c2_A[c2_i258];
  }

  c2_c_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_below_threshold(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c2_d_eml_scalar_eg(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c2_b_eml_xtrsm(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16], real_T c2_b_B[16])
{
  int32_T c2_i259;
  int32_T c2_i260;
  real_T c2_b_A[16];
  for (c2_i259 = 0; c2_i259 < 16; c2_i259++) {
    c2_b_B[c2_i259] = c2_B[c2_i259];
  }

  for (c2_i260 = 0; c2_i260 < 16; c2_i260++) {
    c2_b_A[c2_i260] = c2_A[c2_i260];
  }

  c2_d_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_eml_warning(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c2_i261;
  static char_T c2_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c2_u[27];
  const mxArray *c2_y = NULL;
  for (c2_i261 = 0; c2_i261 < 27; c2_i261++) {
    c2_u[c2_i261] = c2_varargin_1[c2_i261];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c2_y));
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_l_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i262;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i262, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i262;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_m_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_torqueBalancing2012b, const
  char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_torqueBalancing2012b), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_torqueBalancing2012b);
  return c2_y;
}

static uint8_T c2_n_emlrt_marshallIn(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_eml_matlab_zgetrf(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], int32_T c2_ipiv[4], int32_T *c2_info)
{
  int32_T c2_i263;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_jm1;
  int32_T c2_b;
  int32_T c2_mmj;
  int32_T c2_b_a;
  int32_T c2_c;
  int32_T c2_b_b;
  int32_T c2_jj;
  int32_T c2_c_a;
  int32_T c2_jp1j;
  int32_T c2_d_a;
  int32_T c2_b_c;
  int32_T c2_n;
  int32_T c2_ix0;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_idxmax;
  int32_T c2_ix;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_y;
  real_T c2_smax;
  int32_T c2_d_n;
  int32_T c2_c_b;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_e_a;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_c_y;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_d_y;
  real_T c2_s;
  int32_T c2_f_a;
  int32_T c2_jpiv_offset;
  int32_T c2_g_a;
  int32_T c2_e_b;
  int32_T c2_jpiv;
  int32_T c2_h_a;
  int32_T c2_f_b;
  int32_T c2_c_c;
  int32_T c2_g_b;
  int32_T c2_jrow;
  int32_T c2_i_a;
  int32_T c2_h_b;
  int32_T c2_jprow;
  int32_T c2_d_ix0;
  int32_T c2_iy0;
  int32_T c2_e_ix0;
  int32_T c2_b_iy0;
  int32_T c2_f_ix0;
  int32_T c2_c_iy0;
  int32_T c2_b_ix;
  int32_T c2_iy;
  int32_T c2_c_k;
  real_T c2_temp;
  int32_T c2_j_a;
  int32_T c2_k_a;
  int32_T c2_b_jp1j;
  int32_T c2_l_a;
  int32_T c2_d_c;
  int32_T c2_m_a;
  int32_T c2_i_b;
  int32_T c2_i264;
  int32_T c2_n_a;
  int32_T c2_j_b;
  int32_T c2_o_a;
  int32_T c2_k_b;
  boolean_T c2_b_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_k_x;
  real_T c2_e_y;
  real_T c2_z;
  int32_T c2_l_b;
  int32_T c2_e_c;
  int32_T c2_p_a;
  int32_T c2_f_c;
  int32_T c2_q_a;
  int32_T c2_g_c;
  int32_T c2_m;
  int32_T c2_e_n;
  int32_T c2_g_ix0;
  int32_T c2_d_iy0;
  int32_T c2_ia0;
  real_T c2_d1;
  c2_realmin(chartInstance);
  c2_eps(chartInstance);
  for (c2_i263 = 0; c2_i263 < 4; c2_i263++) {
    c2_ipiv[c2_i263] = 1 + c2_i263;
  }

  *c2_info = 0;
  for (c2_j = 1; c2_j < 4; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j - 1;
    c2_jm1 = c2_a;
    c2_b = c2_b_j;
    c2_mmj = 4 - c2_b;
    c2_b_a = c2_jm1;
    c2_c = c2_b_a * 5;
    c2_b_b = c2_c + 1;
    c2_jj = c2_b_b;
    c2_c_a = c2_jj + 1;
    c2_jp1j = c2_c_a;
    c2_d_a = c2_mmj;
    c2_b_c = c2_d_a;
    c2_n = c2_b_c + 1;
    c2_ix0 = c2_jj;
    c2_b_n = c2_n;
    c2_b_ix0 = c2_ix0;
    c2_c_n = c2_b_n;
    c2_c_ix0 = c2_b_ix0;
    if (c2_c_n < 1) {
      c2_idxmax = 0;
    } else {
      c2_idxmax = 1;
      if (c2_c_n > 1) {
        c2_ix = c2_c_ix0;
        c2_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_ix), 1, 16, 1, 0) - 1];
        c2_b_x = c2_x;
        c2_c_x = c2_b_x;
        c2_y = muDoubleScalarAbs(c2_c_x);
        c2_d_x = 0.0;
        c2_e_x = c2_d_x;
        c2_b_y = muDoubleScalarAbs(c2_e_x);
        c2_smax = c2_y + c2_b_y;
        c2_d_n = c2_c_n;
        c2_c_b = c2_d_n;
        c2_d_b = c2_c_b;
        if (2 > c2_d_b) {
          c2_overflow = FALSE;
        } else {
          c2_overflow = (c2_d_b > 2147483646);
        }

        if (c2_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_overflow);
        }

        for (c2_k = 2; c2_k <= c2_d_n; c2_k++) {
          c2_b_k = c2_k;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
          c2_f_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 16, 1, 0) - 1];
          c2_g_x = c2_f_x;
          c2_h_x = c2_g_x;
          c2_c_y = muDoubleScalarAbs(c2_h_x);
          c2_i_x = 0.0;
          c2_j_x = c2_i_x;
          c2_d_y = muDoubleScalarAbs(c2_j_x);
          c2_s = c2_c_y + c2_d_y;
          if (c2_s > c2_smax) {
            c2_idxmax = c2_b_k;
            c2_smax = c2_s;
          }
        }
      }
    }

    c2_f_a = c2_idxmax - 1;
    c2_jpiv_offset = c2_f_a;
    c2_g_a = c2_jj;
    c2_e_b = c2_jpiv_offset;
    c2_jpiv = c2_g_a + c2_e_b;
    if (c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if (c2_jpiv_offset != 0) {
        c2_h_a = c2_b_j;
        c2_f_b = c2_jpiv_offset;
        c2_c_c = c2_h_a + c2_f_b;
        c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 4, 1, 0) - 1] = c2_c_c;
        c2_g_b = c2_jm1 + 1;
        c2_jrow = c2_g_b;
        c2_i_a = c2_jrow;
        c2_h_b = c2_jpiv_offset;
        c2_jprow = c2_i_a + c2_h_b;
        c2_d_ix0 = c2_jrow;
        c2_iy0 = c2_jprow;
        c2_e_ix0 = c2_d_ix0;
        c2_b_iy0 = c2_iy0;
        c2_f_ix0 = c2_e_ix0;
        c2_c_iy0 = c2_b_iy0;
        c2_b_ix = c2_f_ix0;
        c2_iy = c2_c_iy0;
        for (c2_c_k = 1; c2_c_k < 5; c2_c_k++) {
          c2_temp = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 16, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ix), 1, 16, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 16, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 16, 1, 0) - 1] = c2_temp;
          c2_j_a = c2_b_ix + 4;
          c2_b_ix = c2_j_a;
          c2_k_a = c2_iy + 4;
          c2_iy = c2_k_a;
        }
      }

      c2_b_jp1j = c2_jp1j;
      c2_l_a = c2_mmj;
      c2_d_c = c2_l_a;
      c2_m_a = c2_jp1j;
      c2_i_b = c2_d_c - 1;
      c2_i264 = c2_m_a + c2_i_b;
      c2_n_a = c2_b_jp1j;
      c2_j_b = c2_i264;
      c2_o_a = c2_n_a;
      c2_k_b = c2_j_b;
      if (c2_o_a > c2_k_b) {
        c2_b_overflow = FALSE;
      } else {
        c2_b_overflow = (c2_k_b > 2147483646);
      }

      if (c2_b_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      }

      for (c2_i = c2_b_jp1j; c2_i <= c2_i264; c2_i++) {
        c2_b_i = c2_i;
        c2_k_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 16, 1, 0) - 1];
        c2_e_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_jj), 1, 16, 1, 0) - 1];
        c2_z = c2_k_x / c2_e_y;
        c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 16, 1, 0) - 1] = c2_z;
      }
    } else {
      *c2_info = c2_b_j;
    }

    c2_l_b = c2_b_j;
    c2_e_c = 4 - c2_l_b;
    c2_p_a = c2_jj;
    c2_f_c = c2_p_a;
    c2_q_a = c2_jj;
    c2_g_c = c2_q_a;
    c2_m = c2_mmj;
    c2_e_n = c2_e_c;
    c2_g_ix0 = c2_jp1j;
    c2_d_iy0 = c2_f_c + 4;
    c2_ia0 = c2_g_c + 5;
    c2_d1 = -1.0;
    c2_b_eml_xger(chartInstance, c2_m, c2_e_n, c2_d1, c2_g_ix0, c2_d_iy0, c2_A,
                  c2_ia0);
  }

  if (*c2_info == 0) {
    if (!(c2_A[15] != 0.0)) {
      *c2_info = 4;
    }
  }
}

static void c2_b_eml_xger(SFc2_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0,
  real_T c2_A[16], int32_T c2_ia0)
{
  int32_T c2_b_m;
  int32_T c2_b_n;
  real_T c2_b_alpha1;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_b_ia0;
  int32_T c2_c_m;
  int32_T c2_c_n;
  real_T c2_c_alpha1;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_c_ia0;
  int32_T c2_d_m;
  int32_T c2_d_n;
  real_T c2_d_alpha1;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_d_ia0;
  int32_T c2_ixstart;
  int32_T c2_a;
  int32_T c2_jA;
  int32_T c2_jy;
  int32_T c2_e_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  real_T c2_yjy;
  real_T c2_temp;
  int32_T c2_ix;
  int32_T c2_c_b;
  int32_T c2_i265;
  int32_T c2_b_a;
  int32_T c2_d_b;
  int32_T c2_i266;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_d_a;
  int32_T c2_f_b;
  boolean_T c2_b_overflow;
  int32_T c2_ijA;
  int32_T c2_b_ijA;
  int32_T c2_e_a;
  int32_T c2_f_a;
  int32_T c2_g_a;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_alpha1 = c2_alpha1;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_ia0 = c2_ia0;
  c2_c_m = c2_b_m;
  c2_c_n = c2_b_n;
  c2_c_alpha1 = c2_b_alpha1;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_c_ia0 = c2_b_ia0;
  c2_d_m = c2_c_m;
  c2_d_n = c2_c_n;
  c2_d_alpha1 = c2_c_alpha1;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_d_ia0 = c2_c_ia0;
  if (c2_d_alpha1 == 0.0) {
  } else {
    c2_ixstart = c2_d_ix0;
    c2_a = c2_d_ia0 - 1;
    c2_jA = c2_a;
    c2_jy = c2_d_iy0;
    c2_e_n = c2_d_n;
    c2_b = c2_e_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_j = 1; c2_j <= c2_e_n; c2_j++) {
      c2_yjy = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_jy), 1, 16, 1, 0) - 1];
      if (c2_yjy != 0.0) {
        c2_temp = c2_yjy * c2_d_alpha1;
        c2_ix = c2_ixstart;
        c2_c_b = c2_jA + 1;
        c2_i265 = c2_c_b;
        c2_b_a = c2_d_m;
        c2_d_b = c2_jA;
        c2_i266 = c2_b_a + c2_d_b;
        c2_c_a = c2_i265;
        c2_e_b = c2_i266;
        c2_d_a = c2_c_a;
        c2_f_b = c2_e_b;
        if (c2_d_a > c2_f_b) {
          c2_b_overflow = FALSE;
        } else {
          c2_b_overflow = (c2_f_b > 2147483646);
        }

        if (c2_b_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
        }

        for (c2_ijA = c2_i265; c2_ijA <= c2_i266; c2_ijA++) {
          c2_b_ijA = c2_ijA;
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 16, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 16, 1, 0) - 1] +
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_ix), 1, 16, 1, 0) - 1] * c2_temp;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
        }
      }

      c2_f_a = c2_jy + 4;
      c2_jy = c2_f_a;
      c2_g_a = c2_jA + 4;
      c2_jA = c2_g_a;
    }
  }
}

static void c2_c_eml_xtrsm(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16])
{
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_b_b;
  int32_T c2_jBcol;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_b_a;
  int32_T c2_c_c;
  int32_T c2_c_b;
  int32_T c2_d_c;
  int32_T c2_d_b;
  int32_T c2_kAcol;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_e_c;
  int32_T c2_d_a;
  int32_T c2_i267;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_e_a;
  int32_T c2_f_b;
  int32_T c2_f_c;
  int32_T c2_f_a;
  int32_T c2_g_b;
  int32_T c2_g_c;
  int32_T c2_g_a;
  int32_T c2_h_b;
  int32_T c2_h_c;
  int32_T c2_h_a;
  int32_T c2_i_b;
  int32_T c2_i_c;
  c2_below_threshold(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  for (c2_j = 1; c2_j < 5; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j;
    c2_c = c2_a;
    c2_b = c2_c - 1;
    c2_b_c = c2_b << 2;
    c2_b_b = c2_b_c;
    c2_jBcol = c2_b_b;
    for (c2_k = 1; c2_k < 5; c2_k++) {
      c2_b_k = c2_k;
      c2_b_a = c2_b_k;
      c2_c_c = c2_b_a;
      c2_c_b = c2_c_c - 1;
      c2_d_c = c2_c_b << 2;
      c2_d_b = c2_d_c;
      c2_kAcol = c2_d_b;
      c2_c_a = c2_b_k;
      c2_e_b = c2_jBcol;
      c2_e_c = c2_c_a + c2_e_b;
      if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c2_d_a = c2_b_k;
        c2_i267 = c2_d_a;
        c2_overflow = FALSE;
        if (c2_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_overflow);
        }

        for (c2_i = c2_i267 + 1; c2_i < 5; c2_i++) {
          c2_b_i = c2_i;
          c2_e_a = c2_b_i;
          c2_f_b = c2_jBcol;
          c2_f_c = c2_e_a + c2_f_b;
          c2_f_a = c2_b_i;
          c2_g_b = c2_jBcol;
          c2_g_c = c2_f_a + c2_g_b;
          c2_g_a = c2_b_k;
          c2_h_b = c2_jBcol;
          c2_h_c = c2_g_a + c2_h_b;
          c2_h_a = c2_b_i;
          c2_i_b = c2_kAcol;
          c2_i_c = c2_h_a + c2_i_b;
          c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_f_c), 1, 16, 1, 0) - 1] =
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_g_c), 1, 16, 1, 0) - 1] -
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_h_c), 1, 16, 1, 0) - 1] *
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_i_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void c2_d_eml_xtrsm(SFc2_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c2_A[16], real_T c2_B[16])
{
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_b_b;
  int32_T c2_jBcol;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_b_a;
  int32_T c2_c_c;
  int32_T c2_c_b;
  int32_T c2_d_c;
  int32_T c2_d_b;
  int32_T c2_kAcol;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_e_c;
  int32_T c2_d_a;
  int32_T c2_f_b;
  int32_T c2_f_c;
  int32_T c2_e_a;
  int32_T c2_g_b;
  int32_T c2_g_c;
  int32_T c2_f_a;
  int32_T c2_h_b;
  int32_T c2_h_c;
  real_T c2_x;
  real_T c2_y;
  real_T c2_z;
  int32_T c2_g_a;
  int32_T c2_i268;
  int32_T c2_i_b;
  int32_T c2_j_b;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_h_a;
  int32_T c2_k_b;
  int32_T c2_i_c;
  int32_T c2_i_a;
  int32_T c2_l_b;
  int32_T c2_j_c;
  int32_T c2_j_a;
  int32_T c2_m_b;
  int32_T c2_k_c;
  int32_T c2_k_a;
  int32_T c2_n_b;
  int32_T c2_l_c;
  c2_below_threshold(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  for (c2_j = 1; c2_j < 5; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j;
    c2_c = c2_a;
    c2_b = c2_c - 1;
    c2_b_c = c2_b << 2;
    c2_b_b = c2_b_c;
    c2_jBcol = c2_b_b;
    for (c2_k = 4; c2_k > 0; c2_k--) {
      c2_b_k = c2_k;
      c2_b_a = c2_b_k;
      c2_c_c = c2_b_a;
      c2_c_b = c2_c_c - 1;
      c2_d_c = c2_c_b << 2;
      c2_d_b = c2_d_c;
      c2_kAcol = c2_d_b;
      c2_c_a = c2_b_k;
      c2_e_b = c2_jBcol;
      c2_e_c = c2_c_a + c2_e_b;
      if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c2_d_a = c2_b_k;
        c2_f_b = c2_jBcol;
        c2_f_c = c2_d_a + c2_f_b;
        c2_e_a = c2_b_k;
        c2_g_b = c2_jBcol;
        c2_g_c = c2_e_a + c2_g_b;
        c2_f_a = c2_b_k;
        c2_h_b = c2_kAcol;
        c2_h_c = c2_f_a + c2_h_b;
        c2_x = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_g_c), 1, 16, 1, 0) - 1];
        c2_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_h_c), 1, 16, 1, 0) - 1];
        c2_z = c2_x / c2_y;
        c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_f_c), 1, 16, 1, 0) - 1] = c2_z;
        c2_g_a = c2_b_k - 1;
        c2_i268 = c2_g_a;
        c2_i_b = c2_i268;
        c2_j_b = c2_i_b;
        if (1 > c2_j_b) {
          c2_overflow = FALSE;
        } else {
          c2_overflow = (c2_j_b > 2147483646);
        }

        if (c2_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_overflow);
        }

        for (c2_i = 1; c2_i <= c2_i268; c2_i++) {
          c2_b_i = c2_i;
          c2_h_a = c2_b_i;
          c2_k_b = c2_jBcol;
          c2_i_c = c2_h_a + c2_k_b;
          c2_i_a = c2_b_i;
          c2_l_b = c2_jBcol;
          c2_j_c = c2_i_a + c2_l_b;
          c2_j_a = c2_b_k;
          c2_m_b = c2_jBcol;
          c2_k_c = c2_j_a + c2_m_b;
          c2_k_a = c2_b_i;
          c2_n_b = c2_kAcol;
          c2_l_c = c2_k_a + c2_n_b;
          c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_i_c), 1, 16, 1, 0) - 1] =
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_j_c), 1, 16, 1, 0) - 1] -
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_k_c), 1, 16, 1, 0) - 1] *
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_l_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void init_dsm_address_info(SFc2_torqueBalancing2012bInstanceStruct
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

void sf_c2_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(44106875U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(841437657U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2368952469U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3882389986U);
}

mxArray *sf_c2_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("YmIalEnk1h9C8xyxCnKh2D");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
      pr[0] = (double)(4);
      pr[1] = (double)(4);
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
      pr[0] = (double)(4);
      pr[1] = (double)(4);
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
      pr[0] = (double)(12);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c2_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[71],T\"w_H_root\",},{M[8],M[0],T\"is_active_c2_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           2,
           1,
           1,
           8,
           0,
           0,
           0,
           0,
           7,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"imu_H_link");
          _SFD_SET_DATA_PROPS(1,1,1,0,"imu_H_link_0");
          _SFD_SET_DATA_PROPS(2,1,1,0,"link_H_root");
          _SFD_SET_DATA_PROPS(3,1,1,0,"inertial_0");
          _SFD_SET_DATA_PROPS(4,1,1,0,"inertial");
          _SFD_SET_DATA_PROPS(5,1,1,0,"neck");
          _SFD_SET_DATA_PROPS(6,2,0,1,"w_H_root");
          _SFD_SET_DATA_PROPS(7,10,0,0,"CONFIG");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,229);
        _SFD_CV_INIT_SCRIPT(0,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"fromBaseToWorldWithImu",0,-1,1994);
        _SFD_CV_INIT_SCRIPT_IF(0,0,1057,1081,-1,1144);
        _SFD_CV_INIT_SCRIPT_IF(0,1,1145,1171,-1,1234);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"rotz",0,-1,165);
        _SFD_CV_INIT_SCRIPT(2,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"roty",0,-1,165);
        _SFD_CV_INIT_SCRIPT(3,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(3,0,"rotx",0,-1,165);
        _SFD_CV_INIT_SCRIPT(4,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(4,0,"rollPitchYawFromRotation",11,-1,510);
        _SFD_CV_INIT_SCRIPT_IF(4,0,157,173,432,510);
        _SFD_CV_INIT_SCRIPT_IF(4,1,179,195,341,431);
        _SFD_CV_INIT_SCRIPT(5,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(5,0,"correctIMU",0,-1,1239);
        _SFD_CV_INIT_SCRIPT(6,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(6,0,"evalDHMatrix",0,-1,404);
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
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);

        {
          real_T (*c2_imu_H_link)[16];
          real_T (*c2_imu_H_link_0)[16];
          real_T (*c2_link_H_root)[16];
          real_T (*c2_inertial_0)[12];
          real_T (*c2_inertial)[12];
          real_T (*c2_neck)[3];
          real_T (*c2_w_H_root)[16];
          c2_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S,
            1);
          c2_neck = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
          c2_inertial = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 4);
          c2_inertial_0 = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S,
            3);
          c2_link_H_root = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            2);
          c2_imu_H_link_0 = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 1);
          c2_imu_H_link = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_imu_H_link);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_imu_H_link_0);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_link_H_root);
          _SFD_SET_DATA_VALUE_PTR(3U, *c2_inertial_0);
          _SFD_SET_DATA_VALUE_PTR(4U, *c2_inertial);
          _SFD_SET_DATA_VALUE_PTR(5U, *c2_neck);
          _SFD_SET_DATA_VALUE_PTR(6U, *c2_w_H_root);
          _SFD_SET_DATA_VALUE_PTR(7U, &chartInstance->c2_CONFIG);
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
  return "mcLsq81jT0i1ePjG1oQidD";
}

static void sf_opaque_initialize_c2_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_torqueBalancing2012b
    ((SFc2_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c2_torqueBalancing2012b((SFc2_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c2_torqueBalancing2012b((SFc2_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c2_torqueBalancing2012b((SFc2_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c2_torqueBalancing2012b((SFc2_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_torqueBalancing2012b
    ((SFc2_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c2_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_torqueBalancing2012b((SFc2_torqueBalancing2012bInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c2_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c2_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c2_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_torqueBalancing2012bInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c2_torqueBalancing2012b((SFc2_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_torqueBalancing2012b((SFc2_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_torqueBalancing2012b
      ((SFc2_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_torqueBalancing2012b(SimStruct *S)
{
  /* Actual parameters from chart:
     CONFIG
   */
  const char_T *rtParamNames[] = { "CONFIG" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0],
    sf_get_param_data_type_id(S,0));
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1973617619U));
  ssSetChecksum1(S,(3415660132U));
  ssSetChecksum2(S,(2992048661U));
  ssSetChecksum3(S,(3390034151U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_torqueBalancing2012b(SimStruct *S)
{
  SFc2_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc2_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc2_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_torqueBalancing2012b;
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

void c2_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void *
  data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
