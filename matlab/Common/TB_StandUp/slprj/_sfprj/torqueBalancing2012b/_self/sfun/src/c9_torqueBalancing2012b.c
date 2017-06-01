/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c9_torqueBalancing2012b.h"
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
static const char * c9_debug_family_names[10] = { "nargin", "nargout",
  "imu_H_link", "imu_H_link_0", "link_H_root", "inertial_0", "inertial", "neck",
  "CONFIG", "w_H_root" };

static const char * c9_b_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c9_c_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c9_d_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c9_e_debug_family_names[4] = { "nargin", "nargout", "R",
  "rollPitchYaw" };

static const char * c9_f_debug_family_names[7] = { "nargin", "nargout", "a", "d",
  "alph", "thet", "G" };

static const char * c9_g_debug_family_names[10] = { "G_34", "G_45", "G_56",
  "G_6I", "imuAssumingNeckToZero_H_neckBase", "imu_H_neckBase", "nargin",
  "nargout", "neckJoints", "imu_H_imuAssumingNeckToZero" };

static const char * c9_h_debug_family_names[23] = { "wImu_R_imu", "wImu_R_imu_0",
  "imu_R_link", "imu_R_link_0", "wImu_R_link_0", "rollPitchYaw_link_0",
  "rollPitchYaw_link", "rollPitchYawFiltered_link", "wImu_H_link",
  "wImu_H_link_0", "wImu_H_root", "wImu_H_wImuAssumingNeckToZero", "wImu_R_link",
  "nargin", "nargout", "imu_H_link", "imu_H_link_0", "link_H_root", "inertial_0",
  "inertial", "neck", "CONFIG", "w_H_root" };

/* Function Declarations */
static void initialize_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void c9_update_debugger_state_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c9_st);
static void finalize_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c9_torqueBalancing2012b(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void c9_fromBaseToWorldWithImu(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_imu_H_link[16], real_T c9_imu_H_link_0[16], real_T
  c9_link_H_root[16], real_T c9_inertial_0[12], real_T c9_inertial[12], real_T
  c9_neck[3], c9_struct_HZPQIAn1UwxVqJtlH5llbD *c9_b_CONFIG, real_T c9_w_H_root
  [16]);
static void c9_rotz(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c9_alpha, real_T c9_R[9]);
static void c9_roty(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c9_alpha, real_T c9_R[9]);
static void c9_rotx(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c9_alpha, real_T c9_R[9]);
static void c9_rollPitchYawFromRotation(SFc9_torqueBalancing2012bInstanceStruct *
  chartInstance, real_T c9_R[9], real_T c9_rollPitchYaw[3]);
static void c9_correctIMU(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c9_neckJoints[3], real_T c9_imu_H_imuAssumingNeckToZero[16]);
static void c9_evalDHMatrix(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_a, real_T c9_d, real_T c9_alph, real_T c9_thet,
  real_T c9_G[16]);
static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber);
static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData);
static void c9_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_w_H_root, const char_T *c9_identifier,
  real_T c9_y[16]);
static void c9_b_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[16]);
static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static void c9_c_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  c9_struct_HZPQIAn1UwxVqJtlH5llbD *c9_y);
static real_T c9_d_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static c9_struct_amzdx4J7qaaMzgPI7fQ3WD c9_e_emlrt_marshallIn
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c9_u,
   const emlrtMsgIdentifier *c9_parentId);
static boolean_T c9_f_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_g_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[2]);
static void c9_h_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[23]);
static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static const mxArray *c9_d_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static const mxArray *c9_e_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_f_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static void c9_i_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[9]);
static void c9_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_j_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[3]);
static void c9_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_k_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[12]);
static void c9_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_info_helper(c9_ResolvedFunctionInfo c9_info[147]);
static void c9_b_info_helper(c9_ResolvedFunctionInfo c9_info[147]);
static void c9_c_info_helper(c9_ResolvedFunctionInfo c9_info[147]);
static void c9_eml_scalar_eg(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c9_eml_error(SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static real_T c9_atan2(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c9_y, real_T c9_x);
static void c9_b_eml_scalar_eg(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c9_c_eml_scalar_eg(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c9_realmin(SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void c9_eps(SFc9_torqueBalancing2012bInstanceStruct *chartInstance);
static void c9_eml_matlab_zgetrf(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], real_T c9_b_A[16], int32_T c9_ipiv[4],
  int32_T *c9_info);
static void c9_check_forloop_overflow_error
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c9_overflow);
static void c9_eml_xger(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c9_m, int32_T c9_n, real_T c9_alpha1, int32_T c9_ix0, int32_T c9_iy0,
  real_T c9_A[16], int32_T c9_ia0, real_T c9_b_A[16]);
static void c9_eml_xtrsm(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c9_A[16], real_T c9_B[16], real_T c9_b_B[16]);
static void c9_below_threshold(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c9_d_eml_scalar_eg(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c9_b_eml_xtrsm(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], real_T c9_B[16], real_T c9_b_B[16]);
static void c9_eml_warning(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c9_g_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static int32_T c9_l_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static uint8_T c9_m_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_torqueBalancing2012b, const
  char_T *c9_identifier);
static uint8_T c9_n_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_b_eml_matlab_zgetrf(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], int32_T c9_ipiv[4], int32_T *c9_info);
static void c9_b_eml_xger(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c9_m, int32_T c9_n, real_T c9_alpha1, int32_T c9_ix0, int32_T c9_iy0,
  real_T c9_A[16], int32_T c9_ia0);
static void c9_c_eml_xtrsm(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], real_T c9_B[16]);
static void c9_d_eml_xtrsm(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], real_T c9_B[16]);
static void init_dsm_address_info(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c9_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c9_is_active_c9_torqueBalancing2012b = 0U;
}

static void initialize_params_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c9_m0 = NULL;
  const mxArray *c9_mxField;
  c9_struct_HZPQIAn1UwxVqJtlH5llbD c9_r0;
  const mxArray *c9_m1 = NULL;
  const mxArray *c9_b_mxField;
  sf_set_error_prefix_string(
    "Error evaluating data 'CONFIG' in the parent workspace.\n");
  c9_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c9_mxField = sf_mex_getfield(c9_m0, "SIMULATION_TIME", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.SIMULATION_TIME,
                      1, 0, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "SCOPES", "CONFIG", 0);
  c9_m1 = sf_mex_dup(c9_mxField);
  c9_b_mxField = sf_mex_getfield(c9_m1, "ALL", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_b_mxField), &c9_r0.SCOPES.ALL, 1,
                      11, 0U, 0, 0U, 0);
  c9_b_mxField = sf_mex_getfield(c9_m1, "BASE_EST_IMU", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_b_mxField),
                      &c9_r0.SCOPES.BASE_EST_IMU, 1, 11, 0U, 0, 0U, 0);
  c9_b_mxField = sf_mex_getfield(c9_m1, "EXTWRENCHES", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_b_mxField),
                      &c9_r0.SCOPES.EXTWRENCHES, 1, 11, 0U, 0, 0U, 0);
  c9_b_mxField = sf_mex_getfield(c9_m1, "GAIN_SCHE_INFO", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_b_mxField),
                      &c9_r0.SCOPES.GAIN_SCHE_INFO, 1, 11, 0U, 0, 0U, 0);
  c9_b_mxField = sf_mex_getfield(c9_m1, "MAIN", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_b_mxField), &c9_r0.SCOPES.MAIN, 1,
                      11, 0U, 0, 0U, 0);
  c9_b_mxField = sf_mex_getfield(c9_m1, "QP", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_b_mxField), &c9_r0.SCOPES.QP, 1,
                      11, 0U, 0, 0U, 0);
  sf_mex_destroy(&c9_m1);
  c9_mxField = sf_mex_getfield(c9_m0, "CHECK_LIMITS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.CHECK_LIMITS, 1,
                      11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "USE_IMU4EST_BASE", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.USE_IMU4EST_BASE,
                      1, 11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "YAW_IMU_FILTER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.YAW_IMU_FILTER, 1,
                      11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "PITCH_IMU_FILTER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.PITCH_IMU_FILTER,
                      1, 11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "CORRECT_NECK_IMU", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.CORRECT_NECK_IMU,
                      1, 11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "ONSOFTCARPET", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.ONSOFTCARPET, 1,
                      11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "USE_QP_SOLVER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.USE_QP_SOLVER, 1,
                      11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "Ts", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.Ts, 1, 0, 0U, 0,
                      0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "ON_GAZEBO", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.ON_GAZEBO, 1, 11,
                      0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "LEFT_RIGHT_FOOT_IN_CONTACT", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField),
                      c9_r0.LEFT_RIGHT_FOOT_IN_CONTACT, 1, 0, 0U, 1, 0U, 2, 1, 2);
  c9_mxField = sf_mex_getfield(c9_m0, "SMOOTH_DES_COM", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.SMOOTH_DES_COM, 1,
                      0, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "SMOOTH_DES_Q", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.SMOOTH_DES_Q, 1,
                      0, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "smoothingTimeTranDynamics", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField),
                      &c9_r0.smoothingTimeTranDynamics, 1, 0, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "DEMO_MOVEMENTS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.DEMO_MOVEMENTS, 1,
                      11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "TIME_CONTROLLER_SWITCH", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField),
                      &c9_r0.TIME_CONTROLLER_SWITCH, 1, 0, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "JOINTS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), c9_r0.JOINTS, 1, 0, 0U,
                      1, 0U, 2, 23, 1);
  c9_mxField = sf_mex_getfield(c9_m0, "JOINTSITING", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), c9_r0.JOINTSITING, 1, 0,
                      0U, 1, 0U, 2, 23, 1);
  c9_mxField = sf_mex_getfield(c9_m0, "iCubStandUp", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.iCubStandUp, 1,
                      11, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "useExtArmForces", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c9_mxField), &c9_r0.useExtArmForces,
                      1, 11, 0U, 0, 0U, 0);
  sf_mex_destroy(&c9_m0);
  chartInstance->c9_CONFIG = c9_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c9_update_debugger_state_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c9_st;
  const mxArray *c9_y = NULL;
  int32_T c9_i0;
  real_T c9_u[16];
  const mxArray *c9_b_y = NULL;
  uint8_T c9_hoistedGlobal;
  uint8_T c9_b_u;
  const mxArray *c9_c_y = NULL;
  real_T (*c9_w_H_root)[16];
  c9_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c9_st = NULL;
  c9_st = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_createcellarray(2), FALSE);
  for (c9_i0 = 0; c9_i0 < 16; c9_i0++) {
    c9_u[c9_i0] = (*c9_w_H_root)[c9_i0];
  }

  c9_b_y = NULL;
  sf_mex_assign(&c9_b_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_setcell(c9_y, 0, c9_b_y);
  c9_hoistedGlobal = chartInstance->c9_is_active_c9_torqueBalancing2012b;
  c9_b_u = c9_hoistedGlobal;
  c9_c_y = NULL;
  sf_mex_assign(&c9_c_y, sf_mex_create("y", &c9_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c9_y, 1, c9_c_y);
  sf_mex_assign(&c9_st, c9_y, FALSE);
  return c9_st;
}

static void set_sim_state_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c9_st)
{
  const mxArray *c9_u;
  real_T c9_dv0[16];
  int32_T c9_i1;
  real_T (*c9_w_H_root)[16];
  c9_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c9_doneDoubleBufferReInit = TRUE;
  c9_u = sf_mex_dup(c9_st);
  c9_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 0)),
                      "w_H_root", c9_dv0);
  for (c9_i1 = 0; c9_i1 < 16; c9_i1++) {
    (*c9_w_H_root)[c9_i1] = c9_dv0[c9_i1];
  }

  chartInstance->c9_is_active_c9_torqueBalancing2012b = c9_m_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 1)),
     "is_active_c9_torqueBalancing2012b");
  sf_mex_destroy(&c9_u);
  c9_update_debugger_state_c9_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c9_st);
}

static void finalize_c9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c9_torqueBalancing2012b(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c9_i2;
  int32_T c9_i3;
  int32_T c9_i4;
  int32_T c9_i5;
  int32_T c9_i6;
  int32_T c9_i7;
  int32_T c9_i8;
  int32_T c9_i9;
  real_T c9_imu_H_link[16];
  int32_T c9_i10;
  real_T c9_imu_H_link_0[16];
  int32_T c9_i11;
  real_T c9_link_H_root[16];
  int32_T c9_i12;
  real_T c9_inertial_0[12];
  int32_T c9_i13;
  real_T c9_inertial[12];
  int32_T c9_i14;
  real_T c9_neck[3];
  c9_struct_HZPQIAn1UwxVqJtlH5llbD c9_b_CONFIG;
  uint32_T c9_debug_family_var_map[10];
  real_T c9_nargin = 7.0;
  real_T c9_nargout = 1.0;
  real_T c9_w_H_root[16];
  int32_T c9_i15;
  real_T c9_b_imu_H_link[16];
  int32_T c9_i16;
  real_T c9_b_imu_H_link_0[16];
  int32_T c9_i17;
  real_T c9_b_link_H_root[16];
  int32_T c9_i18;
  real_T c9_b_inertial_0[12];
  int32_T c9_i19;
  real_T c9_b_inertial[12];
  int32_T c9_i20;
  real_T c9_b_neck[3];
  c9_struct_HZPQIAn1UwxVqJtlH5llbD c9_c_CONFIG;
  real_T c9_dv1[16];
  int32_T c9_i21;
  int32_T c9_i22;
  real_T (*c9_b_w_H_root)[16];
  real_T (*c9_c_neck)[3];
  real_T (*c9_c_inertial)[12];
  real_T (*c9_c_inertial_0)[12];
  real_T (*c9_c_link_H_root)[16];
  real_T (*c9_c_imu_H_link_0)[16];
  real_T (*c9_c_imu_H_link)[16];
  c9_b_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c9_c_neck = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c9_c_inertial = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 4);
  c9_c_inertial_0 = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 3);
  c9_c_link_H_root = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 2);
  c9_c_imu_H_link_0 = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 1);
  c9_c_imu_H_link = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  for (c9_i2 = 0; c9_i2 < 16; c9_i2++) {
    _SFD_DATA_RANGE_CHECK((*c9_c_imu_H_link)[c9_i2], 0U);
  }

  for (c9_i3 = 0; c9_i3 < 16; c9_i3++) {
    _SFD_DATA_RANGE_CHECK((*c9_c_imu_H_link_0)[c9_i3], 1U);
  }

  for (c9_i4 = 0; c9_i4 < 16; c9_i4++) {
    _SFD_DATA_RANGE_CHECK((*c9_c_link_H_root)[c9_i4], 2U);
  }

  for (c9_i5 = 0; c9_i5 < 12; c9_i5++) {
    _SFD_DATA_RANGE_CHECK((*c9_c_inertial_0)[c9_i5], 3U);
  }

  for (c9_i6 = 0; c9_i6 < 12; c9_i6++) {
    _SFD_DATA_RANGE_CHECK((*c9_c_inertial)[c9_i6], 4U);
  }

  for (c9_i7 = 0; c9_i7 < 3; c9_i7++) {
    _SFD_DATA_RANGE_CHECK((*c9_c_neck)[c9_i7], 5U);
  }

  for (c9_i8 = 0; c9_i8 < 16; c9_i8++) {
    _SFD_DATA_RANGE_CHECK((*c9_b_w_H_root)[c9_i8], 6U);
  }

  chartInstance->c9_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  for (c9_i9 = 0; c9_i9 < 16; c9_i9++) {
    c9_imu_H_link[c9_i9] = (*c9_c_imu_H_link)[c9_i9];
  }

  for (c9_i10 = 0; c9_i10 < 16; c9_i10++) {
    c9_imu_H_link_0[c9_i10] = (*c9_c_imu_H_link_0)[c9_i10];
  }

  for (c9_i11 = 0; c9_i11 < 16; c9_i11++) {
    c9_link_H_root[c9_i11] = (*c9_c_link_H_root)[c9_i11];
  }

  for (c9_i12 = 0; c9_i12 < 12; c9_i12++) {
    c9_inertial_0[c9_i12] = (*c9_c_inertial_0)[c9_i12];
  }

  for (c9_i13 = 0; c9_i13 < 12; c9_i13++) {
    c9_inertial[c9_i13] = (*c9_c_inertial)[c9_i13];
  }

  for (c9_i14 = 0; c9_i14 < 3; c9_i14++) {
    c9_neck[c9_i14] = (*c9_c_neck)[c9_i14];
  }

  c9_b_CONFIG = chartInstance->c9_CONFIG;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c9_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 0U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 1U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_imu_H_link, 2U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_imu_H_link_0, 3U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_link_H_root, 4U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_inertial_0, 5U, c9_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_inertial, 6U, c9_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_neck, 7U, c9_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_b_CONFIG, 8U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_w_H_root, 9U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 3);
  for (c9_i15 = 0; c9_i15 < 16; c9_i15++) {
    c9_b_imu_H_link[c9_i15] = c9_imu_H_link[c9_i15];
  }

  for (c9_i16 = 0; c9_i16 < 16; c9_i16++) {
    c9_b_imu_H_link_0[c9_i16] = c9_imu_H_link_0[c9_i16];
  }

  for (c9_i17 = 0; c9_i17 < 16; c9_i17++) {
    c9_b_link_H_root[c9_i17] = c9_link_H_root[c9_i17];
  }

  for (c9_i18 = 0; c9_i18 < 12; c9_i18++) {
    c9_b_inertial_0[c9_i18] = c9_inertial_0[c9_i18];
  }

  for (c9_i19 = 0; c9_i19 < 12; c9_i19++) {
    c9_b_inertial[c9_i19] = c9_inertial[c9_i19];
  }

  for (c9_i20 = 0; c9_i20 < 3; c9_i20++) {
    c9_b_neck[c9_i20] = c9_neck[c9_i20];
  }

  c9_c_CONFIG = c9_b_CONFIG;
  c9_fromBaseToWorldWithImu(chartInstance, c9_b_imu_H_link, c9_b_imu_H_link_0,
    c9_b_link_H_root, c9_b_inertial_0, c9_b_inertial, c9_b_neck, &c9_c_CONFIG,
    c9_dv1);
  for (c9_i21 = 0; c9_i21 < 16; c9_i21++) {
    c9_w_H_root[c9_i21] = c9_dv1[c9_i21];
  }

  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, -3);
  _SFD_SYMBOL_SCOPE_POP();
  for (c9_i22 = 0; c9_i22 < 16; c9_i22++) {
    (*c9_b_w_H_root)[c9_i22] = c9_w_H_root[c9_i22];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc9_torqueBalancing2012b
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c9_fromBaseToWorldWithImu(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_imu_H_link[16], real_T c9_imu_H_link_0[16], real_T
  c9_link_H_root[16], real_T c9_inertial_0[12], real_T c9_inertial[12], real_T
  c9_neck[3], c9_struct_HZPQIAn1UwxVqJtlH5llbD *c9_b_CONFIG, real_T c9_w_H_root
  [16])
{
  uint32_T c9_debug_family_var_map[23];
  real_T c9_wImu_R_imu[9];
  real_T c9_wImu_R_imu_0[9];
  real_T c9_imu_R_link[9];
  real_T c9_imu_R_link_0[9];
  real_T c9_wImu_R_link_0[9];
  real_T c9_rollPitchYaw_link_0[3];
  real_T c9_rollPitchYaw_link[3];
  real_T c9_rollPitchYawFiltered_link[3];
  real_T c9_wImu_H_link[16];
  real_T c9_wImu_H_link_0[16];
  real_T c9_wImu_H_root[16];
  real_T c9_wImu_H_wImuAssumingNeckToZero[16];
  real_T c9_wImu_R_link[9];
  real_T c9_nargin = 7.0;
  real_T c9_nargout = 1.0;
  int32_T c9_i23;
  real_T c9_a[12];
  int32_T c9_i24;
  int32_T c9_i25;
  int32_T c9_i26;
  int32_T c9_i27;
  int32_T c9_i28;
  real_T c9_b_a[9];
  real_T c9_b[9];
  int32_T c9_i29;
  int32_T c9_i30;
  int32_T c9_i31;
  real_T c9_y[9];
  int32_T c9_i32;
  int32_T c9_i33;
  int32_T c9_i34;
  int32_T c9_i35;
  int32_T c9_i36;
  real_T c9_C[9];
  int32_T c9_i37;
  int32_T c9_i38;
  int32_T c9_i39;
  int32_T c9_i40;
  int32_T c9_i41;
  int32_T c9_i42;
  int32_T c9_i43;
  int32_T c9_i44;
  int32_T c9_i45;
  int32_T c9_i46;
  int32_T c9_i47;
  int32_T c9_i48;
  int32_T c9_i49;
  int32_T c9_i50;
  int32_T c9_i51;
  int32_T c9_i52;
  int32_T c9_i53;
  int32_T c9_i54;
  int32_T c9_i55;
  int32_T c9_i56;
  int32_T c9_i57;
  int32_T c9_i58;
  int32_T c9_i59;
  int32_T c9_i60;
  int32_T c9_i61;
  int32_T c9_i62;
  int32_T c9_i63;
  int32_T c9_i64;
  int32_T c9_i65;
  int32_T c9_i66;
  int32_T c9_i67;
  int32_T c9_i68;
  int32_T c9_i69;
  int32_T c9_i70;
  int32_T c9_i71;
  int32_T c9_i72;
  int32_T c9_i73;
  int32_T c9_i74;
  int32_T c9_i75;
  int32_T c9_i76;
  int32_T c9_i77;
  int32_T c9_i78;
  int32_T c9_i79;
  int32_T c9_i80;
  int32_T c9_i81;
  int32_T c9_i82;
  int32_T c9_i83;
  int32_T c9_i84;
  int32_T c9_i85;
  int32_T c9_i86;
  int32_T c9_i87;
  int32_T c9_i88;
  int32_T c9_i89;
  int32_T c9_i90;
  int32_T c9_i91;
  int32_T c9_i92;
  int32_T c9_i93;
  int32_T c9_i94;
  int32_T c9_i95;
  real_T c9_b_wImu_R_link_0[9];
  real_T c9_dv2[3];
  int32_T c9_i96;
  int32_T c9_i97;
  real_T c9_b_wImu_R_link[9];
  real_T c9_dv3[3];
  int32_T c9_i98;
  int32_T c9_i99;
  int32_T c9_i100;
  int32_T c9_i101;
  int32_T c9_i102;
  int32_T c9_i103;
  int32_T c9_i104;
  int32_T c9_i105;
  int32_T c9_i106;
  int32_T c9_i107;
  int32_T c9_i108;
  int32_T c9_i109;
  int32_T c9_i110;
  int32_T c9_i111;
  int32_T c9_i112;
  int32_T c9_i113;
  int32_T c9_i114;
  int32_T c9_i115;
  int32_T c9_i116;
  real_T c9_dv4[3];
  int32_T c9_i117;
  int32_T c9_i118;
  int32_T c9_i119;
  int32_T c9_i120;
  int32_T c9_i121;
  int32_T c9_i122;
  int32_T c9_i123;
  static real_T c9_dv5[4] = { 0.0, 0.0, 0.0, 1.0 };

  int32_T c9_i124;
  int32_T c9_i125;
  int32_T c9_i126;
  int32_T c9_i127;
  int32_T c9_i128;
  int32_T c9_i129;
  int32_T c9_i130;
  int32_T c9_i131;
  int32_T c9_i132;
  real_T c9_c_a[16];
  int32_T c9_i133;
  real_T c9_b_b[16];
  int32_T c9_i134;
  int32_T c9_i135;
  int32_T c9_i136;
  real_T c9_b_C[16];
  int32_T c9_i137;
  int32_T c9_i138;
  int32_T c9_i139;
  int32_T c9_i140;
  int32_T c9_i141;
  int32_T c9_i142;
  int32_T c9_i143;
  int32_T c9_i144;
  int32_T c9_i145;
  real_T c9_b_neck[3];
  real_T c9_dv6[16];
  int32_T c9_i146;
  int32_T c9_i147;
  int32_T c9_i148;
  int32_T c9_i149;
  int32_T c9_i150;
  int32_T c9_i151;
  int32_T c9_i152;
  int32_T c9_i153;
  int32_T c9_i154;
  int32_T c9_i155;
  int32_T c9_i156;
  int32_T c9_i157;
  int32_T c9_i158;
  int32_T c9_i159;
  int32_T c9_i160;
  int32_T c9_i161;
  int32_T c9_info;
  int32_T c9_ipiv[4];
  int32_T c9_b_info;
  int32_T c9_c_info;
  int32_T c9_d_info;
  int32_T c9_i162;
  int32_T c9_i;
  int32_T c9_b_i;
  int32_T c9_ip;
  int32_T c9_j;
  int32_T c9_b_j;
  real_T c9_temp;
  int32_T c9_i163;
  real_T c9_dv7[16];
  int32_T c9_i164;
  real_T c9_dv8[16];
  int32_T c9_i165;
  real_T c9_dv9[16];
  int32_T c9_i166;
  real_T c9_dv10[16];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 23U, 23U, c9_h_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_wImu_R_imu, 0U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_wImu_R_imu_0, 1U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_imu_R_link, 2U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_imu_R_link_0, 3U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_wImu_R_link_0, 4U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_rollPitchYaw_link_0, 5U,
    c9_c_sf_marshallOut, c9_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_rollPitchYaw_link, 6U,
    c9_c_sf_marshallOut, c9_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_rollPitchYawFiltered_link, 7U,
    c9_c_sf_marshallOut, c9_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_wImu_H_link, 8U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_wImu_H_link_0, 9U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_wImu_H_root, 10U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_wImu_H_wImuAssumingNeckToZero, 11U,
    c9_sf_marshallOut, c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_wImu_R_link, 12U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 13U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 14U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_imu_H_link, 15U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_imu_H_link_0, 16U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_link_H_root, 17U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_inertial_0, 18U, c9_d_sf_marshallOut,
    c9_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_inertial, 19U, c9_d_sf_marshallOut,
    c9_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_neck, 20U, c9_c_sf_marshallOut,
    c9_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_b_CONFIG, 21U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_w_H_root, 22U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 5);
  for (c9_i23 = 0; c9_i23 < 12; c9_i23++) {
    c9_a[c9_i23] = c9_inertial[c9_i23];
  }

  for (c9_i24 = 0; c9_i24 < 12; c9_i24++) {
    c9_a[c9_i24] *= 3.1415926535897931;
  }

  for (c9_i25 = 0; c9_i25 < 12; c9_i25++) {
    c9_inertial[c9_i25] = c9_a[c9_i25] / 180.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 6);
  for (c9_i26 = 0; c9_i26 < 12; c9_i26++) {
    c9_a[c9_i26] = c9_inertial_0[c9_i26];
  }

  for (c9_i27 = 0; c9_i27 < 12; c9_i27++) {
    c9_a[c9_i27] *= 3.1415926535897931;
  }

  for (c9_i28 = 0; c9_i28 < 12; c9_i28++) {
    c9_inertial_0[c9_i28] = c9_a[c9_i28] / 180.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 10);
  c9_rotz(chartInstance, c9_inertial[2], c9_b_a);
  c9_roty(chartInstance, c9_inertial[1], c9_b);
  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i29 = 0; c9_i29 < 3; c9_i29++) {
    c9_i30 = 0;
    for (c9_i31 = 0; c9_i31 < 3; c9_i31++) {
      c9_y[c9_i30 + c9_i29] = 0.0;
      c9_i32 = 0;
      for (c9_i33 = 0; c9_i33 < 3; c9_i33++) {
        c9_y[c9_i30 + c9_i29] += c9_b_a[c9_i32 + c9_i29] * c9_b[c9_i33 + c9_i30];
        c9_i32 += 3;
      }

      c9_i30 += 3;
    }
  }

  c9_rotx(chartInstance, c9_inertial[0], c9_b);
  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i34 = 0; c9_i34 < 9; c9_i34++) {
    c9_wImu_R_imu[c9_i34] = 0.0;
  }

  for (c9_i35 = 0; c9_i35 < 9; c9_i35++) {
    c9_wImu_R_imu[c9_i35] = 0.0;
  }

  for (c9_i36 = 0; c9_i36 < 9; c9_i36++) {
    c9_C[c9_i36] = c9_wImu_R_imu[c9_i36];
  }

  for (c9_i37 = 0; c9_i37 < 9; c9_i37++) {
    c9_wImu_R_imu[c9_i37] = c9_C[c9_i37];
  }

  for (c9_i38 = 0; c9_i38 < 9; c9_i38++) {
    c9_C[c9_i38] = c9_wImu_R_imu[c9_i38];
  }

  for (c9_i39 = 0; c9_i39 < 9; c9_i39++) {
    c9_wImu_R_imu[c9_i39] = c9_C[c9_i39];
  }

  for (c9_i40 = 0; c9_i40 < 3; c9_i40++) {
    c9_i41 = 0;
    for (c9_i42 = 0; c9_i42 < 3; c9_i42++) {
      c9_wImu_R_imu[c9_i41 + c9_i40] = 0.0;
      c9_i43 = 0;
      for (c9_i44 = 0; c9_i44 < 3; c9_i44++) {
        c9_wImu_R_imu[c9_i41 + c9_i40] += c9_y[c9_i43 + c9_i40] * c9_b[c9_i44 +
          c9_i41];
        c9_i43 += 3;
      }

      c9_i41 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 11);
  c9_rotz(chartInstance, c9_inertial_0[2], c9_b_a);
  c9_roty(chartInstance, c9_inertial_0[1], c9_b);
  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i45 = 0; c9_i45 < 3; c9_i45++) {
    c9_i46 = 0;
    for (c9_i47 = 0; c9_i47 < 3; c9_i47++) {
      c9_y[c9_i46 + c9_i45] = 0.0;
      c9_i48 = 0;
      for (c9_i49 = 0; c9_i49 < 3; c9_i49++) {
        c9_y[c9_i46 + c9_i45] += c9_b_a[c9_i48 + c9_i45] * c9_b[c9_i49 + c9_i46];
        c9_i48 += 3;
      }

      c9_i46 += 3;
    }
  }

  c9_rotx(chartInstance, c9_inertial_0[0], c9_b);
  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i50 = 0; c9_i50 < 9; c9_i50++) {
    c9_wImu_R_imu_0[c9_i50] = 0.0;
  }

  for (c9_i51 = 0; c9_i51 < 9; c9_i51++) {
    c9_wImu_R_imu_0[c9_i51] = 0.0;
  }

  for (c9_i52 = 0; c9_i52 < 9; c9_i52++) {
    c9_C[c9_i52] = c9_wImu_R_imu_0[c9_i52];
  }

  for (c9_i53 = 0; c9_i53 < 9; c9_i53++) {
    c9_wImu_R_imu_0[c9_i53] = c9_C[c9_i53];
  }

  for (c9_i54 = 0; c9_i54 < 9; c9_i54++) {
    c9_C[c9_i54] = c9_wImu_R_imu_0[c9_i54];
  }

  for (c9_i55 = 0; c9_i55 < 9; c9_i55++) {
    c9_wImu_R_imu_0[c9_i55] = c9_C[c9_i55];
  }

  for (c9_i56 = 0; c9_i56 < 3; c9_i56++) {
    c9_i57 = 0;
    for (c9_i58 = 0; c9_i58 < 3; c9_i58++) {
      c9_wImu_R_imu_0[c9_i57 + c9_i56] = 0.0;
      c9_i59 = 0;
      for (c9_i60 = 0; c9_i60 < 3; c9_i60++) {
        c9_wImu_R_imu_0[c9_i57 + c9_i56] += c9_y[c9_i59 + c9_i56] * c9_b[c9_i60
          + c9_i57];
        c9_i59 += 3;
      }

      c9_i57 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 13);
  c9_i61 = 0;
  c9_i62 = 0;
  for (c9_i63 = 0; c9_i63 < 3; c9_i63++) {
    for (c9_i64 = 0; c9_i64 < 3; c9_i64++) {
      c9_imu_R_link[c9_i64 + c9_i61] = c9_imu_H_link[c9_i64 + c9_i62];
    }

    c9_i61 += 3;
    c9_i62 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 14);
  c9_i65 = 0;
  c9_i66 = 0;
  for (c9_i67 = 0; c9_i67 < 3; c9_i67++) {
    for (c9_i68 = 0; c9_i68 < 3; c9_i68++) {
      c9_imu_R_link_0[c9_i68 + c9_i65] = c9_imu_H_link_0[c9_i68 + c9_i66];
    }

    c9_i65 += 3;
    c9_i66 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 17);
  for (c9_i69 = 0; c9_i69 < 9; c9_i69++) {
    c9_b_a[c9_i69] = c9_wImu_R_imu[c9_i69];
  }

  for (c9_i70 = 0; c9_i70 < 9; c9_i70++) {
    c9_b[c9_i70] = c9_imu_R_link[c9_i70];
  }

  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i71 = 0; c9_i71 < 9; c9_i71++) {
    c9_wImu_R_link[c9_i71] = 0.0;
  }

  for (c9_i72 = 0; c9_i72 < 9; c9_i72++) {
    c9_wImu_R_link[c9_i72] = 0.0;
  }

  for (c9_i73 = 0; c9_i73 < 9; c9_i73++) {
    c9_C[c9_i73] = c9_wImu_R_link[c9_i73];
  }

  for (c9_i74 = 0; c9_i74 < 9; c9_i74++) {
    c9_wImu_R_link[c9_i74] = c9_C[c9_i74];
  }

  for (c9_i75 = 0; c9_i75 < 9; c9_i75++) {
    c9_C[c9_i75] = c9_wImu_R_link[c9_i75];
  }

  for (c9_i76 = 0; c9_i76 < 9; c9_i76++) {
    c9_wImu_R_link[c9_i76] = c9_C[c9_i76];
  }

  for (c9_i77 = 0; c9_i77 < 3; c9_i77++) {
    c9_i78 = 0;
    for (c9_i79 = 0; c9_i79 < 3; c9_i79++) {
      c9_wImu_R_link[c9_i78 + c9_i77] = 0.0;
      c9_i80 = 0;
      for (c9_i81 = 0; c9_i81 < 3; c9_i81++) {
        c9_wImu_R_link[c9_i78 + c9_i77] += c9_b_a[c9_i80 + c9_i77] * c9_b[c9_i81
          + c9_i78];
        c9_i80 += 3;
      }

      c9_i78 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 18);
  for (c9_i82 = 0; c9_i82 < 9; c9_i82++) {
    c9_b_a[c9_i82] = c9_wImu_R_imu_0[c9_i82];
  }

  for (c9_i83 = 0; c9_i83 < 9; c9_i83++) {
    c9_b[c9_i83] = c9_imu_R_link_0[c9_i83];
  }

  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i84 = 0; c9_i84 < 9; c9_i84++) {
    c9_wImu_R_link_0[c9_i84] = 0.0;
  }

  for (c9_i85 = 0; c9_i85 < 9; c9_i85++) {
    c9_wImu_R_link_0[c9_i85] = 0.0;
  }

  for (c9_i86 = 0; c9_i86 < 9; c9_i86++) {
    c9_C[c9_i86] = c9_wImu_R_link_0[c9_i86];
  }

  for (c9_i87 = 0; c9_i87 < 9; c9_i87++) {
    c9_wImu_R_link_0[c9_i87] = c9_C[c9_i87];
  }

  for (c9_i88 = 0; c9_i88 < 9; c9_i88++) {
    c9_C[c9_i88] = c9_wImu_R_link_0[c9_i88];
  }

  for (c9_i89 = 0; c9_i89 < 9; c9_i89++) {
    c9_wImu_R_link_0[c9_i89] = c9_C[c9_i89];
  }

  for (c9_i90 = 0; c9_i90 < 3; c9_i90++) {
    c9_i91 = 0;
    for (c9_i92 = 0; c9_i92 < 3; c9_i92++) {
      c9_wImu_R_link_0[c9_i91 + c9_i90] = 0.0;
      c9_i93 = 0;
      for (c9_i94 = 0; c9_i94 < 3; c9_i94++) {
        c9_wImu_R_link_0[c9_i91 + c9_i90] += c9_b_a[c9_i93 + c9_i90] *
          c9_b[c9_i94 + c9_i91];
        c9_i93 += 3;
      }

      c9_i91 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 20);
  for (c9_i95 = 0; c9_i95 < 9; c9_i95++) {
    c9_b_wImu_R_link_0[c9_i95] = c9_wImu_R_link_0[c9_i95];
  }

  c9_rollPitchYawFromRotation(chartInstance, c9_b_wImu_R_link_0, c9_dv2);
  for (c9_i96 = 0; c9_i96 < 3; c9_i96++) {
    c9_rollPitchYaw_link_0[c9_i96] = c9_dv2[c9_i96];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 21);
  for (c9_i97 = 0; c9_i97 < 9; c9_i97++) {
    c9_b_wImu_R_link[c9_i97] = c9_wImu_R_link[c9_i97];
  }

  c9_rollPitchYawFromRotation(chartInstance, c9_b_wImu_R_link, c9_dv3);
  for (c9_i98 = 0; c9_i98 < 3; c9_i98++) {
    c9_rollPitchYaw_link[c9_i98] = c9_dv3[c9_i98];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 23);
  for (c9_i99 = 0; c9_i99 < 3; c9_i99++) {
    c9_rollPitchYawFiltered_link[c9_i99] = c9_rollPitchYaw_link[c9_i99];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 25);
  if (CV_SCRIPT_IF(0, 0, c9_b_CONFIG->YAW_IMU_FILTER)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 26);
    c9_rollPitchYawFiltered_link[2] = c9_rollPitchYaw_link_0[2];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 28);
  if (CV_SCRIPT_IF(0, 1, c9_b_CONFIG->PITCH_IMU_FILTER)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 29);
    c9_rollPitchYawFiltered_link[1] = c9_rollPitchYaw_link_0[1];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 32);
  c9_rotz(chartInstance, c9_rollPitchYawFiltered_link[2], c9_b_a);
  c9_roty(chartInstance, c9_rollPitchYawFiltered_link[1], c9_b);
  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i100 = 0; c9_i100 < 3; c9_i100++) {
    c9_i101 = 0;
    for (c9_i102 = 0; c9_i102 < 3; c9_i102++) {
      c9_y[c9_i101 + c9_i100] = 0.0;
      c9_i103 = 0;
      for (c9_i104 = 0; c9_i104 < 3; c9_i104++) {
        c9_y[c9_i101 + c9_i100] += c9_b_a[c9_i103 + c9_i100] * c9_b[c9_i104 +
          c9_i101];
        c9_i103 += 3;
      }

      c9_i101 += 3;
    }
  }

  c9_rotx(chartInstance, c9_rollPitchYawFiltered_link[0], c9_b);
  c9_eml_scalar_eg(chartInstance);
  c9_eml_scalar_eg(chartInstance);
  for (c9_i105 = 0; c9_i105 < 9; c9_i105++) {
    c9_wImu_R_link[c9_i105] = 0.0;
  }

  for (c9_i106 = 0; c9_i106 < 9; c9_i106++) {
    c9_wImu_R_link[c9_i106] = 0.0;
  }

  for (c9_i107 = 0; c9_i107 < 9; c9_i107++) {
    c9_C[c9_i107] = c9_wImu_R_link[c9_i107];
  }

  for (c9_i108 = 0; c9_i108 < 9; c9_i108++) {
    c9_wImu_R_link[c9_i108] = c9_C[c9_i108];
  }

  for (c9_i109 = 0; c9_i109 < 9; c9_i109++) {
    c9_C[c9_i109] = c9_wImu_R_link[c9_i109];
  }

  for (c9_i110 = 0; c9_i110 < 9; c9_i110++) {
    c9_wImu_R_link[c9_i110] = c9_C[c9_i110];
  }

  for (c9_i111 = 0; c9_i111 < 3; c9_i111++) {
    c9_i112 = 0;
    for (c9_i113 = 0; c9_i113 < 3; c9_i113++) {
      c9_wImu_R_link[c9_i112 + c9_i111] = 0.0;
      c9_i114 = 0;
      for (c9_i115 = 0; c9_i115 < 3; c9_i115++) {
        c9_wImu_R_link[c9_i112 + c9_i111] += c9_y[c9_i114 + c9_i111] *
          c9_b[c9_i115 + c9_i112];
        c9_i114 += 3;
      }

      c9_i112 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 34);
  for (c9_i116 = 0; c9_i116 < 3; c9_i116++) {
    c9_dv4[c9_i116] = 0.0;
  }

  c9_i117 = 0;
  c9_i118 = 0;
  for (c9_i119 = 0; c9_i119 < 3; c9_i119++) {
    for (c9_i120 = 0; c9_i120 < 3; c9_i120++) {
      c9_wImu_H_link[c9_i120 + c9_i117] = c9_wImu_R_link[c9_i120 + c9_i118];
    }

    c9_i117 += 4;
    c9_i118 += 3;
  }

  for (c9_i121 = 0; c9_i121 < 3; c9_i121++) {
    c9_wImu_H_link[c9_i121 + 12] = c9_dv4[c9_i121];
  }

  c9_i122 = 0;
  for (c9_i123 = 0; c9_i123 < 4; c9_i123++) {
    c9_wImu_H_link[c9_i122 + 3] = c9_dv5[c9_i123];
    c9_i122 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 37);
  for (c9_i124 = 0; c9_i124 < 3; c9_i124++) {
    c9_dv4[c9_i124] = 0.0;
  }

  c9_i125 = 0;
  c9_i126 = 0;
  for (c9_i127 = 0; c9_i127 < 3; c9_i127++) {
    for (c9_i128 = 0; c9_i128 < 3; c9_i128++) {
      c9_wImu_H_link_0[c9_i128 + c9_i125] = c9_wImu_R_link_0[c9_i128 + c9_i126];
    }

    c9_i125 += 4;
    c9_i126 += 3;
  }

  for (c9_i129 = 0; c9_i129 < 3; c9_i129++) {
    c9_wImu_H_link_0[c9_i129 + 12] = c9_dv4[c9_i129];
  }

  c9_i130 = 0;
  for (c9_i131 = 0; c9_i131 < 4; c9_i131++) {
    c9_wImu_H_link_0[c9_i130 + 3] = c9_dv5[c9_i131];
    c9_i130 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 40);
  for (c9_i132 = 0; c9_i132 < 16; c9_i132++) {
    c9_c_a[c9_i132] = c9_wImu_H_link[c9_i132];
  }

  for (c9_i133 = 0; c9_i133 < 16; c9_i133++) {
    c9_b_b[c9_i133] = c9_link_H_root[c9_i133];
  }

  c9_c_eml_scalar_eg(chartInstance);
  c9_c_eml_scalar_eg(chartInstance);
  for (c9_i134 = 0; c9_i134 < 16; c9_i134++) {
    c9_wImu_H_root[c9_i134] = 0.0;
  }

  for (c9_i135 = 0; c9_i135 < 16; c9_i135++) {
    c9_wImu_H_root[c9_i135] = 0.0;
  }

  for (c9_i136 = 0; c9_i136 < 16; c9_i136++) {
    c9_b_C[c9_i136] = c9_wImu_H_root[c9_i136];
  }

  for (c9_i137 = 0; c9_i137 < 16; c9_i137++) {
    c9_wImu_H_root[c9_i137] = c9_b_C[c9_i137];
  }

  for (c9_i138 = 0; c9_i138 < 16; c9_i138++) {
    c9_b_C[c9_i138] = c9_wImu_H_root[c9_i138];
  }

  for (c9_i139 = 0; c9_i139 < 16; c9_i139++) {
    c9_wImu_H_root[c9_i139] = c9_b_C[c9_i139];
  }

  for (c9_i140 = 0; c9_i140 < 4; c9_i140++) {
    c9_i141 = 0;
    for (c9_i142 = 0; c9_i142 < 4; c9_i142++) {
      c9_wImu_H_root[c9_i141 + c9_i140] = 0.0;
      c9_i143 = 0;
      for (c9_i144 = 0; c9_i144 < 4; c9_i144++) {
        c9_wImu_H_root[c9_i141 + c9_i140] += c9_c_a[c9_i143 + c9_i140] *
          c9_b_b[c9_i144 + c9_i141];
        c9_i143 += 4;
      }

      c9_i141 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 43);
  for (c9_i145 = 0; c9_i145 < 3; c9_i145++) {
    c9_b_neck[c9_i145] = c9_neck[c9_i145];
  }

  c9_correctIMU(chartInstance, c9_b_neck, c9_dv6);
  for (c9_i146 = 0; c9_i146 < 16; c9_i146++) {
    c9_wImu_H_wImuAssumingNeckToZero[c9_i146] = c9_dv6[c9_i146];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 44);
  for (c9_i147 = 0; c9_i147 < 16; c9_i147++) {
    c9_c_a[c9_i147] = c9_wImu_H_wImuAssumingNeckToZero[c9_i147];
  }

  for (c9_i148 = 0; c9_i148 < 16; c9_i148++) {
    c9_b_b[c9_i148] = c9_wImu_H_root[c9_i148];
  }

  c9_c_eml_scalar_eg(chartInstance);
  c9_c_eml_scalar_eg(chartInstance);
  for (c9_i149 = 0; c9_i149 < 16; c9_i149++) {
    c9_wImu_H_root[c9_i149] = 0.0;
  }

  for (c9_i150 = 0; c9_i150 < 16; c9_i150++) {
    c9_wImu_H_root[c9_i150] = 0.0;
  }

  for (c9_i151 = 0; c9_i151 < 16; c9_i151++) {
    c9_b_C[c9_i151] = c9_wImu_H_root[c9_i151];
  }

  for (c9_i152 = 0; c9_i152 < 16; c9_i152++) {
    c9_wImu_H_root[c9_i152] = c9_b_C[c9_i152];
  }

  for (c9_i153 = 0; c9_i153 < 16; c9_i153++) {
    c9_b_C[c9_i153] = c9_wImu_H_root[c9_i153];
  }

  for (c9_i154 = 0; c9_i154 < 16; c9_i154++) {
    c9_wImu_H_root[c9_i154] = c9_b_C[c9_i154];
  }

  for (c9_i155 = 0; c9_i155 < 4; c9_i155++) {
    c9_i156 = 0;
    for (c9_i157 = 0; c9_i157 < 4; c9_i157++) {
      c9_wImu_H_root[c9_i156 + c9_i155] = 0.0;
      c9_i158 = 0;
      for (c9_i159 = 0; c9_i159 < 4; c9_i159++) {
        c9_wImu_H_root[c9_i156 + c9_i155] += c9_c_a[c9_i158 + c9_i155] *
          c9_b_b[c9_i159 + c9_i156];
        c9_i158 += 4;
      }

      c9_i156 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 46);
  for (c9_i160 = 0; c9_i160 < 16; c9_i160++) {
    c9_c_a[c9_i160] = c9_wImu_H_link_0[c9_i160];
  }

  for (c9_i161 = 0; c9_i161 < 16; c9_i161++) {
    c9_b_b[c9_i161] = c9_wImu_H_root[c9_i161];
  }

  c9_b_eml_matlab_zgetrf(chartInstance, c9_c_a, c9_ipiv, &c9_info);
  c9_b_info = c9_info;
  c9_c_info = c9_b_info;
  c9_d_info = c9_c_info;
  if (c9_d_info > 0) {
    c9_eml_warning(chartInstance);
  }

  c9_c_eml_scalar_eg(chartInstance);
  for (c9_i162 = 0; c9_i162 < 16; c9_i162++) {
    c9_w_H_root[c9_i162] = c9_b_b[c9_i162];
  }

  for (c9_i = 1; c9_i < 5; c9_i++) {
    c9_b_i = c9_i;
    if (c9_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c9_b_i), 1, 4, 1, 0) - 1] != c9_b_i) {
      c9_ip = c9_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c9_b_i), 1, 4, 1, 0) - 1];
      for (c9_j = 1; c9_j < 5; c9_j++) {
        c9_b_j = c9_j;
        c9_temp = c9_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_b_i), 1, 4, 1, 0) +
          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c9_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c9_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c9_b_i), 1, 4, 1, 0) +
                     ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_b_j), 1, 4, 2, 0) - 1) << 2)) - 1] =
          c9_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_ip), 1, 4, 1, 0) +
                       ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c9_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c9_ip), 1, 4, 1, 0) +
                     ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_b_j), 1, 4, 2, 0) - 1) << 2)) - 1] =
          c9_temp;
      }
    }
  }

  for (c9_i163 = 0; c9_i163 < 16; c9_i163++) {
    c9_dv7[c9_i163] = c9_c_a[c9_i163];
  }

  for (c9_i164 = 0; c9_i164 < 16; c9_i164++) {
    c9_dv8[c9_i164] = c9_dv7[c9_i164];
  }

  c9_c_eml_xtrsm(chartInstance, c9_dv8, c9_w_H_root);
  for (c9_i165 = 0; c9_i165 < 16; c9_i165++) {
    c9_dv9[c9_i165] = c9_c_a[c9_i165];
  }

  for (c9_i166 = 0; c9_i166 < 16; c9_i166++) {
    c9_dv10[c9_i166] = c9_dv9[c9_i166];
  }

  c9_d_eml_xtrsm(chartInstance, c9_dv10, c9_w_H_root);
  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, -46);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c9_rotz(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c9_alpha, real_T c9_R[9])
{
  uint32_T c9_debug_family_var_map[4];
  real_T c9_nargin = 1.0;
  real_T c9_nargout = 1.0;
  int32_T c9_i167;
  real_T c9_x;
  real_T c9_b_x;
  real_T c9_c_x;
  real_T c9_d_x;
  real_T c9_e_x;
  real_T c9_f_x;
  real_T c9_g_x;
  real_T c9_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c9_b_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 0U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 1U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_alpha, 2U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_R, 3U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c9_sfEvent, 2);
  for (c9_i167 = 0; c9_i167 < 9; c9_i167++) {
    c9_R[c9_i167] = 0.0;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c9_sfEvent, 3);
  c9_R[8] = 1.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c9_sfEvent, 4);
  c9_x = c9_alpha;
  c9_b_x = c9_x;
  c9_b_x = muDoubleScalarCos(c9_b_x);
  c9_R[0] = c9_b_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c9_sfEvent, 5);
  c9_c_x = c9_alpha;
  c9_d_x = c9_c_x;
  c9_d_x = muDoubleScalarSin(c9_d_x);
  c9_R[3] = -c9_d_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c9_sfEvent, 6);
  c9_e_x = c9_alpha;
  c9_f_x = c9_e_x;
  c9_f_x = muDoubleScalarSin(c9_f_x);
  c9_R[1] = c9_f_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c9_sfEvent, 7);
  c9_g_x = c9_alpha;
  c9_h_x = c9_g_x;
  c9_h_x = muDoubleScalarCos(c9_h_x);
  c9_R[4] = c9_h_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c9_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c9_roty(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c9_alpha, real_T c9_R[9])
{
  uint32_T c9_debug_family_var_map[4];
  real_T c9_nargin = 1.0;
  real_T c9_nargout = 1.0;
  int32_T c9_i168;
  real_T c9_x;
  real_T c9_b_x;
  real_T c9_c_x;
  real_T c9_d_x;
  real_T c9_e_x;
  real_T c9_f_x;
  real_T c9_g_x;
  real_T c9_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c9_c_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 0U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 1U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_alpha, 2U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_R, 3U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c9_sfEvent, 2);
  for (c9_i168 = 0; c9_i168 < 9; c9_i168++) {
    c9_R[c9_i168] = 0.0;
  }

  _SFD_SCRIPT_CALL(2U, chartInstance->c9_sfEvent, 3);
  c9_R[4] = 1.0;
  _SFD_SCRIPT_CALL(2U, chartInstance->c9_sfEvent, 4);
  c9_x = c9_alpha;
  c9_b_x = c9_x;
  c9_b_x = muDoubleScalarCos(c9_b_x);
  c9_R[0] = c9_b_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c9_sfEvent, 5);
  c9_c_x = c9_alpha;
  c9_d_x = c9_c_x;
  c9_d_x = muDoubleScalarSin(c9_d_x);
  c9_R[6] = c9_d_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c9_sfEvent, 6);
  c9_e_x = c9_alpha;
  c9_f_x = c9_e_x;
  c9_f_x = muDoubleScalarSin(c9_f_x);
  c9_R[2] = -c9_f_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c9_sfEvent, 7);
  c9_g_x = c9_alpha;
  c9_h_x = c9_g_x;
  c9_h_x = muDoubleScalarCos(c9_h_x);
  c9_R[8] = c9_h_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c9_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c9_rotx(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
                    real_T c9_alpha, real_T c9_R[9])
{
  uint32_T c9_debug_family_var_map[4];
  real_T c9_nargin = 1.0;
  real_T c9_nargout = 1.0;
  int32_T c9_i169;
  real_T c9_x;
  real_T c9_b_x;
  real_T c9_c_x;
  real_T c9_d_x;
  real_T c9_e_x;
  real_T c9_f_x;
  real_T c9_g_x;
  real_T c9_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c9_d_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 0U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 1U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_alpha, 2U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_R, 3U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  CV_SCRIPT_FCN(3, 0);
  _SFD_SCRIPT_CALL(3U, chartInstance->c9_sfEvent, 2);
  for (c9_i169 = 0; c9_i169 < 9; c9_i169++) {
    c9_R[c9_i169] = 0.0;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c9_sfEvent, 3);
  c9_R[0] = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c9_sfEvent, 4);
  c9_x = c9_alpha;
  c9_b_x = c9_x;
  c9_b_x = muDoubleScalarCos(c9_b_x);
  c9_R[4] = c9_b_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c9_sfEvent, 5);
  c9_c_x = c9_alpha;
  c9_d_x = c9_c_x;
  c9_d_x = muDoubleScalarSin(c9_d_x);
  c9_R[7] = -c9_d_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c9_sfEvent, 6);
  c9_e_x = c9_alpha;
  c9_f_x = c9_e_x;
  c9_f_x = muDoubleScalarSin(c9_f_x);
  c9_R[5] = c9_f_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c9_sfEvent, 7);
  c9_g_x = c9_alpha;
  c9_h_x = c9_g_x;
  c9_h_x = muDoubleScalarCos(c9_h_x);
  c9_R[8] = c9_h_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c9_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c9_rollPitchYawFromRotation(SFc9_torqueBalancing2012bInstanceStruct *
  chartInstance, real_T c9_R[9], real_T c9_rollPitchYaw[3])
{
  uint32_T c9_debug_family_var_map[4];
  real_T c9_nargin = 1.0;
  real_T c9_nargout = 1.0;
  int32_T c9_i170;
  real_T c9_x;
  real_T c9_b_x;
  boolean_T guard1 = FALSE;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c9_e_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 0U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 1U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_R, 2U, c9_f_sf_marshallOut,
    c9_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_rollPitchYaw, 3U, c9_c_sf_marshallOut,
    c9_e_sf_marshallIn);
  CV_SCRIPT_FCN(4, 0);
  _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 4);
  for (c9_i170 = 0; c9_i170 < 3; c9_i170++) {
    c9_rollPitchYaw[c9_i170] = 0.0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 5);
  if (CV_SCRIPT_IF(4, 0, c9_R[2] < 1.0)) {
    _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 6);
    if (CV_SCRIPT_IF(4, 1, c9_R[2] > -1.0)) {
      _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 7);
      c9_x = -c9_R[2];
      c9_b_x = c9_x;
      guard1 = FALSE;
      if (c9_b_x < -1.0) {
        guard1 = TRUE;
      } else {
        if (1.0 < c9_b_x) {
          guard1 = TRUE;
        }
      }

      if (guard1 == TRUE) {
        c9_eml_error(chartInstance);
      }

      c9_b_x = muDoubleScalarAsin(c9_b_x);
      c9_rollPitchYaw[1] = c9_b_x;
      _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 8);
      c9_rollPitchYaw[2] = c9_atan2(chartInstance, c9_R[1], c9_R[0]);
      _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 9);
      c9_rollPitchYaw[0] = c9_atan2(chartInstance, c9_R[5], c9_R[8]);
    } else {
      _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 11);
      c9_rollPitchYaw[2] = -c9_atan2(chartInstance, -c9_R[7], c9_R[4]);
      _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 12);
      c9_rollPitchYaw[0] = 0.0;
    }
  } else {
    _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 15);
    c9_rollPitchYaw[2] = c9_atan2(chartInstance, -c9_R[7], c9_R[4]);
    _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, 16);
    c9_rollPitchYaw[0] = 0.0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c9_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c9_correctIMU(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c9_neckJoints[3], real_T c9_imu_H_imuAssumingNeckToZero[16])
{
  uint32_T c9_debug_family_var_map[10];
  real_T c9_G_34[16];
  real_T c9_G_45[16];
  real_T c9_G_56[16];
  real_T c9_G_6I[16];
  real_T c9_imuAssumingNeckToZero_H_neckBase[16];
  real_T c9_imu_H_neckBase[16];
  real_T c9_nargin = 1.0;
  real_T c9_nargout = 1.0;
  int32_T c9_i171;
  static real_T c9_dv11[16] = { 6.123233995736766E-17, 1.0, 0.0, 0.0,
    -6.123233995736766E-17, 3.749399456654644E-33, 1.0, 0.0, 1.0,
    -6.123233995736766E-17, 6.123233995736766E-17, 0.0, 5.8170722959499274E-19,
    0.0095, 0.0, 1.0 };

  int32_T c9_i172;
  static real_T c9_dv12[16] = { 6.123233995736766E-17, -1.0, 0.0, 0.0,
    6.123233995736766E-17, 3.749399456654644E-33, -1.0, 0.0, 1.0,
    6.123233995736766E-17, 6.123233995736766E-17, 0.0, 0.0, -0.0, 0.0, 1.0 };

  int32_T c9_i173;
  static real_T c9_dv13[16] = { 6.123233995736766E-17, 1.0, 0.0, 0.0,
    -6.123233995736766E-17, 3.749399456654644E-33, -1.0, 0.0, -1.0,
    6.123233995736766E-17, 6.123233995736766E-17, 0.0, 1.1327982892113017E-18,
    0.0185, 0.1108, 1.0 };

  int32_T c9_i174;
  static real_T c9_b[16] = { 1.0, 0.0, 0.0, 0.0, -0.0, 6.123233995736766E-17,
    1.0, 0.0, 0.0, -1.0, 6.123233995736766E-17, 0.0, 0.0, 0.0, 0.0066, 1.0 };

  int32_T c9_i175;
  static real_T c9_dv14[16] = { -1.0, 1.2246467991473532E-16,
    -1.2246467991473532E-16, 0.0, -1.2246467991473532E-16,
    -6.1232339957367648E-17, 1.0, 0.0, 1.2246467991473532E-16, 1.0,
    6.123233995736766E-17, 0.0, -0.018499999999999985, 0.12029999999999999,
    0.0066000000000000043, 1.0 };

  real_T c9_dv15[16];
  int32_T c9_i176;
  real_T c9_dv16[16];
  int32_T c9_i177;
  real_T c9_dv17[16];
  int32_T c9_i178;
  int32_T c9_i179;
  int32_T c9_i180;
  real_T c9_a[16];
  int32_T c9_i181;
  real_T c9_b_b[16];
  int32_T c9_i182;
  int32_T c9_i183;
  int32_T c9_i184;
  real_T c9_y[16];
  int32_T c9_i185;
  int32_T c9_i186;
  int32_T c9_i187;
  int32_T c9_i188;
  int32_T c9_i189;
  int32_T c9_i190;
  real_T c9_b_y[16];
  int32_T c9_i191;
  int32_T c9_i192;
  int32_T c9_i193;
  int32_T c9_i194;
  int32_T c9_i195;
  int32_T c9_i196;
  int32_T c9_i197;
  int32_T c9_i198;
  int32_T c9_i199;
  int32_T c9_i200;
  int32_T c9_i201;
  int32_T c9_i202;
  int32_T c9_i203;
  int32_T c9_i204;
  int32_T c9_i205;
  int32_T c9_i206;
  int32_T c9_i207;
  int32_T c9_i208;
  int32_T c9_i;
  int32_T c9_b_i;
  static int32_T c9_iv0[4] = { 1, 3, 3, 4 };

  int32_T c9_ip;
  int32_T c9_j;
  int32_T c9_b_j;
  real_T c9_temp;
  int32_T c9_i209;
  static real_T c9_dv18[16] = { -1.0, -1.2246467991473532E-16,
    1.2246467991473532E-16, 0.018499999999999985, 1.2246467991473532E-16, 1.0,
    -6.123233995736766E-17, 0.12029999999999999, -1.2246467991473532E-16,
    6.1232339957367648E-17, 1.0, 0.0066, 0.0, 0.0, 0.0, 1.0 };

  real_T c9_dv19[16];
  int32_T c9_i210;
  real_T c9_dv20[16];
  int32_T c9_i211;
  int32_T c9_i212;
  int32_T c9_i213;
  int32_T c9_i214;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c9_g_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_G_34, 0U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_G_45, 1U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_G_56, 2U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_G_6I, 3U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_imuAssumingNeckToZero_H_neckBase, 4U,
    c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_imu_H_neckBase, 5U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 6U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 7U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_neckJoints, 8U, c9_c_sf_marshallOut,
    c9_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_imu_H_imuAssumingNeckToZero, 9U,
    c9_sf_marshallOut, c9_sf_marshallIn);
  CV_SCRIPT_FCN(5, 0);
  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 9);
  for (c9_i171 = 0; c9_i171 < 16; c9_i171++) {
    c9_G_34[c9_i171] = c9_dv11[c9_i171];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 10);
  for (c9_i172 = 0; c9_i172 < 16; c9_i172++) {
    c9_G_45[c9_i172] = c9_dv12[c9_i172];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 11);
  for (c9_i173 = 0; c9_i173 < 16; c9_i173++) {
    c9_G_56[c9_i173] = c9_dv13[c9_i173];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 12);
  for (c9_i174 = 0; c9_i174 < 16; c9_i174++) {
    c9_G_6I[c9_i174] = c9_b[c9_i174];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 14);
  for (c9_i175 = 0; c9_i175 < 16; c9_i175++) {
    c9_imuAssumingNeckToZero_H_neckBase[c9_i175] = c9_dv14[c9_i175];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 17);
  c9_evalDHMatrix(chartInstance, 0.0095, 0.0, 1.5707963267948966, c9_neckJoints
                  [0] + 1.5707963267948966, c9_dv15);
  for (c9_i176 = 0; c9_i176 < 16; c9_i176++) {
    c9_G_34[c9_i176] = c9_dv15[c9_i176];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 18);
  c9_evalDHMatrix(chartInstance, 0.0, 0.0, -1.5707963267948966, c9_neckJoints[1]
                  - 1.5707963267948966, c9_dv16);
  for (c9_i177 = 0; c9_i177 < 16; c9_i177++) {
    c9_G_45[c9_i177] = c9_dv16[c9_i177];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 19);
  c9_evalDHMatrix(chartInstance, 0.0185, 0.1108, -1.5707963267948966,
                  c9_neckJoints[2] + 1.5707963267948966, c9_dv17);
  for (c9_i178 = 0; c9_i178 < 16; c9_i178++) {
    c9_G_56[c9_i178] = c9_dv17[c9_i178];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 20);
  for (c9_i179 = 0; c9_i179 < 16; c9_i179++) {
    c9_G_6I[c9_i179] = c9_b[c9_i179];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 22);
  for (c9_i180 = 0; c9_i180 < 16; c9_i180++) {
    c9_a[c9_i180] = c9_G_34[c9_i180];
  }

  for (c9_i181 = 0; c9_i181 < 16; c9_i181++) {
    c9_b_b[c9_i181] = c9_G_45[c9_i181];
  }

  c9_c_eml_scalar_eg(chartInstance);
  c9_c_eml_scalar_eg(chartInstance);
  for (c9_i182 = 0; c9_i182 < 4; c9_i182++) {
    c9_i183 = 0;
    for (c9_i184 = 0; c9_i184 < 4; c9_i184++) {
      c9_y[c9_i183 + c9_i182] = 0.0;
      c9_i185 = 0;
      for (c9_i186 = 0; c9_i186 < 4; c9_i186++) {
        c9_y[c9_i183 + c9_i182] += c9_a[c9_i185 + c9_i182] * c9_b_b[c9_i186 +
          c9_i183];
        c9_i185 += 4;
      }

      c9_i183 += 4;
    }
  }

  for (c9_i187 = 0; c9_i187 < 16; c9_i187++) {
    c9_b_b[c9_i187] = c9_G_56[c9_i187];
  }

  c9_c_eml_scalar_eg(chartInstance);
  c9_c_eml_scalar_eg(chartInstance);
  for (c9_i188 = 0; c9_i188 < 4; c9_i188++) {
    c9_i189 = 0;
    for (c9_i190 = 0; c9_i190 < 4; c9_i190++) {
      c9_b_y[c9_i189 + c9_i188] = 0.0;
      c9_i191 = 0;
      for (c9_i192 = 0; c9_i192 < 4; c9_i192++) {
        c9_b_y[c9_i189 + c9_i188] += c9_y[c9_i191 + c9_i188] * c9_b_b[c9_i192 +
          c9_i189];
        c9_i191 += 4;
      }

      c9_i189 += 4;
    }
  }

  c9_c_eml_scalar_eg(chartInstance);
  c9_c_eml_scalar_eg(chartInstance);
  for (c9_i193 = 0; c9_i193 < 16; c9_i193++) {
    c9_imu_H_neckBase[c9_i193] = 0.0;
  }

  for (c9_i194 = 0; c9_i194 < 16; c9_i194++) {
    c9_imu_H_neckBase[c9_i194] = 0.0;
  }

  for (c9_i195 = 0; c9_i195 < 16; c9_i195++) {
    c9_a[c9_i195] = c9_imu_H_neckBase[c9_i195];
  }

  for (c9_i196 = 0; c9_i196 < 16; c9_i196++) {
    c9_imu_H_neckBase[c9_i196] = c9_a[c9_i196];
  }

  for (c9_i197 = 0; c9_i197 < 16; c9_i197++) {
    c9_a[c9_i197] = c9_imu_H_neckBase[c9_i197];
  }

  for (c9_i198 = 0; c9_i198 < 16; c9_i198++) {
    c9_imu_H_neckBase[c9_i198] = c9_a[c9_i198];
  }

  for (c9_i199 = 0; c9_i199 < 4; c9_i199++) {
    c9_i200 = 0;
    for (c9_i201 = 0; c9_i201 < 4; c9_i201++) {
      c9_imu_H_neckBase[c9_i200 + c9_i199] = 0.0;
      c9_i202 = 0;
      for (c9_i203 = 0; c9_i203 < 4; c9_i203++) {
        c9_imu_H_neckBase[c9_i200 + c9_i199] += c9_b_y[c9_i202 + c9_i199] *
          c9_b[c9_i203 + c9_i200];
        c9_i202 += 4;
      }

      c9_i200 += 4;
    }
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, 24);
  for (c9_i204 = 0; c9_i204 < 16; c9_i204++) {
    c9_a[c9_i204] = c9_imu_H_neckBase[c9_i204];
  }

  c9_i205 = 0;
  for (c9_i206 = 0; c9_i206 < 4; c9_i206++) {
    c9_i207 = 0;
    for (c9_i208 = 0; c9_i208 < 4; c9_i208++) {
      c9_b_b[c9_i208 + c9_i205] = c9_a[c9_i207 + c9_i206];
      c9_i207 += 4;
    }

    c9_i205 += 4;
  }

  c9_c_eml_scalar_eg(chartInstance);
  for (c9_i = 1; c9_i < 5; c9_i++) {
    c9_b_i = c9_i;
    if (c9_iv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c9_b_i), 1, 4, 1, 0) - 1] != c9_b_i) {
      c9_ip = c9_iv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c9_b_i), 1, 4, 1, 0) - 1];
      for (c9_j = 1; c9_j < 5; c9_j++) {
        c9_b_j = c9_j;
        c9_temp = c9_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_b_i), 1, 4, 1, 0) +
                          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c9_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c9_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                   "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c9_b_j), 1, 4, 2,
                   0) - 1) << 2)) - 1] = c9_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c9_ip), 1, 4, 1, 0) +
          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c9_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c9_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c9_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
                   (int32_T)_SFD_INTEGER_CHECK("", (real_T)c9_b_j), 1, 4, 2, 0)
                  - 1) << 2)) - 1] = c9_temp;
      }
    }
  }

  for (c9_i209 = 0; c9_i209 < 16; c9_i209++) {
    c9_dv19[c9_i209] = c9_dv18[c9_i209];
  }

  c9_c_eml_xtrsm(chartInstance, c9_dv19, c9_b_b);
  for (c9_i210 = 0; c9_i210 < 16; c9_i210++) {
    c9_dv20[c9_i210] = c9_dv18[c9_i210];
  }

  c9_d_eml_xtrsm(chartInstance, c9_dv20, c9_b_b);
  c9_i211 = 0;
  for (c9_i212 = 0; c9_i212 < 4; c9_i212++) {
    c9_i213 = 0;
    for (c9_i214 = 0; c9_i214 < 4; c9_i214++) {
      c9_imu_H_imuAssumingNeckToZero[c9_i214 + c9_i211] = c9_b_b[c9_i213 +
        c9_i212];
      c9_i213 += 4;
    }

    c9_i211 += 4;
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c9_sfEvent, -24);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c9_evalDHMatrix(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_a, real_T c9_d, real_T c9_alph, real_T c9_thet,
  real_T c9_G[16])
{
  uint32_T c9_debug_family_var_map[7];
  real_T c9_nargin = 4.0;
  real_T c9_nargout = 1.0;
  real_T c9_x;
  real_T c9_b_x;
  real_T c9_c_x;
  real_T c9_d_x;
  real_T c9_e_x;
  real_T c9_f_x;
  real_T c9_b_a;
  real_T c9_b;
  real_T c9_y;
  real_T c9_g_x;
  real_T c9_h_x;
  real_T c9_i_x;
  real_T c9_j_x;
  real_T c9_c_a;
  real_T c9_b_b;
  real_T c9_b_y;
  real_T c9_k_x;
  real_T c9_l_x;
  real_T c9_d_a;
  real_T c9_c_b;
  real_T c9_c_y;
  real_T c9_m_x;
  real_T c9_n_x;
  real_T c9_o_x;
  real_T c9_p_x;
  real_T c9_q_x;
  real_T c9_r_x;
  real_T c9_e_a;
  real_T c9_d_b;
  real_T c9_d_y;
  real_T c9_s_x;
  real_T c9_t_x;
  real_T c9_u_x;
  real_T c9_v_x;
  real_T c9_f_a;
  real_T c9_e_b;
  real_T c9_e_y;
  real_T c9_w_x;
  real_T c9_x_x;
  real_T c9_g_a;
  real_T c9_f_b;
  real_T c9_f_y;
  real_T c9_y_x;
  real_T c9_ab_x;
  real_T c9_bb_x;
  real_T c9_cb_x;
  int32_T c9_i215;
  int32_T c9_i216;
  static real_T c9_dv21[4] = { 0.0, 0.0, 0.0, 1.0 };

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c9_f_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 0U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 1U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_a, 2U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_d, 3U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_alph, 4U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_thet, 5U, c9_e_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_G, 6U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  CV_SCRIPT_FCN(6, 0);
  _SFD_SCRIPT_CALL(6U, chartInstance->c9_sfEvent, 3);
  c9_x = c9_thet;
  c9_b_x = c9_x;
  c9_b_x = muDoubleScalarCos(c9_b_x);
  c9_c_x = c9_thet;
  c9_d_x = c9_c_x;
  c9_d_x = muDoubleScalarSin(c9_d_x);
  c9_e_x = c9_alph;
  c9_f_x = c9_e_x;
  c9_f_x = muDoubleScalarCos(c9_f_x);
  c9_b_a = -c9_d_x;
  c9_b = c9_f_x;
  c9_y = c9_b_a * c9_b;
  c9_g_x = c9_thet;
  c9_h_x = c9_g_x;
  c9_h_x = muDoubleScalarSin(c9_h_x);
  c9_i_x = c9_alph;
  c9_j_x = c9_i_x;
  c9_j_x = muDoubleScalarSin(c9_j_x);
  c9_c_a = c9_h_x;
  c9_b_b = c9_j_x;
  c9_b_y = c9_c_a * c9_b_b;
  c9_k_x = c9_thet;
  c9_l_x = c9_k_x;
  c9_l_x = muDoubleScalarCos(c9_l_x);
  c9_d_a = c9_l_x;
  c9_c_b = c9_a;
  c9_c_y = c9_d_a * c9_c_b;
  c9_m_x = c9_thet;
  c9_n_x = c9_m_x;
  c9_n_x = muDoubleScalarSin(c9_n_x);
  c9_o_x = c9_thet;
  c9_p_x = c9_o_x;
  c9_p_x = muDoubleScalarCos(c9_p_x);
  c9_q_x = c9_alph;
  c9_r_x = c9_q_x;
  c9_r_x = muDoubleScalarCos(c9_r_x);
  c9_e_a = c9_p_x;
  c9_d_b = c9_r_x;
  c9_d_y = c9_e_a * c9_d_b;
  c9_s_x = c9_thet;
  c9_t_x = c9_s_x;
  c9_t_x = muDoubleScalarCos(c9_t_x);
  c9_u_x = c9_alph;
  c9_v_x = c9_u_x;
  c9_v_x = muDoubleScalarSin(c9_v_x);
  c9_f_a = -c9_t_x;
  c9_e_b = c9_v_x;
  c9_e_y = c9_f_a * c9_e_b;
  c9_w_x = c9_thet;
  c9_x_x = c9_w_x;
  c9_x_x = muDoubleScalarSin(c9_x_x);
  c9_g_a = c9_x_x;
  c9_f_b = c9_a;
  c9_f_y = c9_g_a * c9_f_b;
  c9_y_x = c9_alph;
  c9_ab_x = c9_y_x;
  c9_ab_x = muDoubleScalarSin(c9_ab_x);
  c9_bb_x = c9_alph;
  c9_cb_x = c9_bb_x;
  c9_cb_x = muDoubleScalarCos(c9_cb_x);
  c9_G[0] = c9_b_x;
  c9_G[4] = c9_y;
  c9_G[8] = c9_b_y;
  c9_G[12] = c9_c_y;
  c9_G[1] = c9_n_x;
  c9_G[5] = c9_d_y;
  c9_G[9] = c9_e_y;
  c9_G[13] = c9_f_y;
  c9_G[2] = 0.0;
  c9_G[6] = c9_ab_x;
  c9_G[10] = c9_cb_x;
  c9_G[14] = c9_d;
  c9_i215 = 0;
  for (c9_i216 = 0; c9_i216 < 4; c9_i216++) {
    c9_G[c9_i215 + 3] = c9_dv21[c9_i216];
    c9_i215 += 4;
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c9_sfEvent, -3);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c9_chartNumber, 0U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m"));
  _SFD_SCRIPT_TRANSLATION(c9_chartNumber, 1U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m"));
  _SFD_SCRIPT_TRANSLATION(c9_chartNumber, 2U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m"));
  _SFD_SCRIPT_TRANSLATION(c9_chartNumber, 3U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m"));
  _SFD_SCRIPT_TRANSLATION(c9_chartNumber, 4U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m"));
  _SFD_SCRIPT_TRANSLATION(c9_chartNumber, 5U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m"));
  _SFD_SCRIPT_TRANSLATION(c9_chartNumber, 6U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m"));
}

static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i217;
  int32_T c9_i218;
  int32_T c9_i219;
  real_T c9_b_inData[16];
  int32_T c9_i220;
  int32_T c9_i221;
  int32_T c9_i222;
  real_T c9_u[16];
  const mxArray *c9_y = NULL;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_i217 = 0;
  for (c9_i218 = 0; c9_i218 < 4; c9_i218++) {
    for (c9_i219 = 0; c9_i219 < 4; c9_i219++) {
      c9_b_inData[c9_i219 + c9_i217] = (*(real_T (*)[16])c9_inData)[c9_i219 +
        c9_i217];
    }

    c9_i217 += 4;
  }

  c9_i220 = 0;
  for (c9_i221 = 0; c9_i221 < 4; c9_i221++) {
    for (c9_i222 = 0; c9_i222 < 4; c9_i222++) {
      c9_u[c9_i222 + c9_i220] = c9_b_inData[c9_i222 + c9_i220];
    }

    c9_i220 += 4;
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static void c9_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_w_H_root, const char_T *c9_identifier,
  real_T c9_y[16])
{
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_w_H_root), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_w_H_root);
}

static void c9_b_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[16])
{
  real_T c9_dv22[16];
  int32_T c9_i223;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv22, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c9_i223 = 0; c9_i223 < 16; c9_i223++) {
    c9_y[c9_i223] = c9_dv22[c9_i223];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_w_H_root;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[16];
  int32_T c9_i224;
  int32_T c9_i225;
  int32_T c9_i226;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_w_H_root = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_w_H_root), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_w_H_root);
  c9_i224 = 0;
  for (c9_i225 = 0; c9_i225 < 4; c9_i225++) {
    for (c9_i226 = 0; c9_i226 < 4; c9_i226++) {
      (*(real_T (*)[16])c9_outData)[c9_i226 + c9_i224] = c9_y[c9_i226 + c9_i224];
    }

    c9_i224 += 4;
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData;
  c9_struct_HZPQIAn1UwxVqJtlH5llbD c9_u;
  const mxArray *c9_y = NULL;
  real_T c9_b_u;
  const mxArray *c9_b_y = NULL;
  c9_struct_amzdx4J7qaaMzgPI7fQ3WD c9_c_u;
  const mxArray *c9_c_y = NULL;
  boolean_T c9_d_u;
  const mxArray *c9_d_y = NULL;
  boolean_T c9_e_u;
  const mxArray *c9_e_y = NULL;
  boolean_T c9_f_u;
  const mxArray *c9_f_y = NULL;
  boolean_T c9_g_u;
  const mxArray *c9_g_y = NULL;
  boolean_T c9_h_u;
  const mxArray *c9_h_y = NULL;
  boolean_T c9_i_u;
  const mxArray *c9_i_y = NULL;
  boolean_T c9_j_u;
  const mxArray *c9_j_y = NULL;
  boolean_T c9_k_u;
  const mxArray *c9_k_y = NULL;
  boolean_T c9_l_u;
  const mxArray *c9_l_y = NULL;
  boolean_T c9_m_u;
  const mxArray *c9_m_y = NULL;
  boolean_T c9_n_u;
  const mxArray *c9_n_y = NULL;
  boolean_T c9_o_u;
  const mxArray *c9_o_y = NULL;
  boolean_T c9_p_u;
  const mxArray *c9_p_y = NULL;
  real_T c9_q_u;
  const mxArray *c9_q_y = NULL;
  boolean_T c9_r_u;
  const mxArray *c9_r_y = NULL;
  int32_T c9_i227;
  real_T c9_s_u[2];
  const mxArray *c9_s_y = NULL;
  real_T c9_t_u;
  const mxArray *c9_t_y = NULL;
  real_T c9_u_u;
  const mxArray *c9_u_y = NULL;
  real_T c9_v_u;
  const mxArray *c9_v_y = NULL;
  boolean_T c9_w_u;
  const mxArray *c9_w_y = NULL;
  real_T c9_x_u;
  const mxArray *c9_x_y = NULL;
  int32_T c9_i228;
  real_T c9_y_u[23];
  const mxArray *c9_y_y = NULL;
  int32_T c9_i229;
  real_T c9_ab_u[23];
  const mxArray *c9_ab_y = NULL;
  boolean_T c9_bb_u;
  const mxArray *c9_bb_y = NULL;
  boolean_T c9_cb_u;
  const mxArray *c9_cb_y = NULL;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_mxArrayOutData = NULL;
  c9_u = *(c9_struct_HZPQIAn1UwxVqJtlH5llbD *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c9_b_u = c9_u.SIMULATION_TIME;
  c9_b_y = NULL;
  sf_mex_assign(&c9_b_y, sf_mex_create("y", &c9_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_b_y, "SIMULATION_TIME", "SIMULATION_TIME", 0);
  c9_c_u = c9_u.SCOPES;
  c9_c_y = NULL;
  sf_mex_assign(&c9_c_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c9_d_u = c9_c_u.ALL;
  c9_d_y = NULL;
  sf_mex_assign(&c9_d_y, sf_mex_create("y", &c9_d_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_c_y, c9_d_y, "ALL", "ALL", 0);
  c9_e_u = c9_c_u.BASE_EST_IMU;
  c9_e_y = NULL;
  sf_mex_assign(&c9_e_y, sf_mex_create("y", &c9_e_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_c_y, c9_e_y, "BASE_EST_IMU", "BASE_EST_IMU", 0);
  c9_f_u = c9_c_u.EXTWRENCHES;
  c9_f_y = NULL;
  sf_mex_assign(&c9_f_y, sf_mex_create("y", &c9_f_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_c_y, c9_f_y, "EXTWRENCHES", "EXTWRENCHES", 0);
  c9_g_u = c9_c_u.GAIN_SCHE_INFO;
  c9_g_y = NULL;
  sf_mex_assign(&c9_g_y, sf_mex_create("y", &c9_g_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_c_y, c9_g_y, "GAIN_SCHE_INFO", "GAIN_SCHE_INFO", 0);
  c9_h_u = c9_c_u.MAIN;
  c9_h_y = NULL;
  sf_mex_assign(&c9_h_y, sf_mex_create("y", &c9_h_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_c_y, c9_h_y, "MAIN", "MAIN", 0);
  c9_i_u = c9_c_u.QP;
  c9_i_y = NULL;
  sf_mex_assign(&c9_i_y, sf_mex_create("y", &c9_i_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_c_y, c9_i_y, "QP", "QP", 0);
  sf_mex_addfield(c9_y, c9_c_y, "SCOPES", "SCOPES", 0);
  c9_j_u = c9_u.CHECK_LIMITS;
  c9_j_y = NULL;
  sf_mex_assign(&c9_j_y, sf_mex_create("y", &c9_j_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_j_y, "CHECK_LIMITS", "CHECK_LIMITS", 0);
  c9_k_u = c9_u.USE_IMU4EST_BASE;
  c9_k_y = NULL;
  sf_mex_assign(&c9_k_y, sf_mex_create("y", &c9_k_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_k_y, "USE_IMU4EST_BASE", "USE_IMU4EST_BASE", 0);
  c9_l_u = c9_u.YAW_IMU_FILTER;
  c9_l_y = NULL;
  sf_mex_assign(&c9_l_y, sf_mex_create("y", &c9_l_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_l_y, "YAW_IMU_FILTER", "YAW_IMU_FILTER", 0);
  c9_m_u = c9_u.PITCH_IMU_FILTER;
  c9_m_y = NULL;
  sf_mex_assign(&c9_m_y, sf_mex_create("y", &c9_m_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_m_y, "PITCH_IMU_FILTER", "PITCH_IMU_FILTER", 0);
  c9_n_u = c9_u.CORRECT_NECK_IMU;
  c9_n_y = NULL;
  sf_mex_assign(&c9_n_y, sf_mex_create("y", &c9_n_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_n_y, "CORRECT_NECK_IMU", "CORRECT_NECK_IMU", 0);
  c9_o_u = c9_u.ONSOFTCARPET;
  c9_o_y = NULL;
  sf_mex_assign(&c9_o_y, sf_mex_create("y", &c9_o_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_o_y, "ONSOFTCARPET", "ONSOFTCARPET", 0);
  c9_p_u = c9_u.USE_QP_SOLVER;
  c9_p_y = NULL;
  sf_mex_assign(&c9_p_y, sf_mex_create("y", &c9_p_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_p_y, "USE_QP_SOLVER", "USE_QP_SOLVER", 0);
  c9_q_u = c9_u.Ts;
  c9_q_y = NULL;
  sf_mex_assign(&c9_q_y, sf_mex_create("y", &c9_q_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_q_y, "Ts", "Ts", 0);
  c9_r_u = c9_u.ON_GAZEBO;
  c9_r_y = NULL;
  sf_mex_assign(&c9_r_y, sf_mex_create("y", &c9_r_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_r_y, "ON_GAZEBO", "ON_GAZEBO", 0);
  for (c9_i227 = 0; c9_i227 < 2; c9_i227++) {
    c9_s_u[c9_i227] = c9_u.LEFT_RIGHT_FOOT_IN_CONTACT[c9_i227];
  }

  c9_s_y = NULL;
  sf_mex_assign(&c9_s_y, sf_mex_create("y", c9_s_u, 0, 0U, 1U, 0U, 2, 1, 2),
                FALSE);
  sf_mex_addfield(c9_y, c9_s_y, "LEFT_RIGHT_FOOT_IN_CONTACT",
                  "LEFT_RIGHT_FOOT_IN_CONTACT", 0);
  c9_t_u = c9_u.SMOOTH_DES_COM;
  c9_t_y = NULL;
  sf_mex_assign(&c9_t_y, sf_mex_create("y", &c9_t_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_t_y, "SMOOTH_DES_COM", "SMOOTH_DES_COM", 0);
  c9_u_u = c9_u.SMOOTH_DES_Q;
  c9_u_y = NULL;
  sf_mex_assign(&c9_u_y, sf_mex_create("y", &c9_u_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_u_y, "SMOOTH_DES_Q", "SMOOTH_DES_Q", 0);
  c9_v_u = c9_u.smoothingTimeTranDynamics;
  c9_v_y = NULL;
  sf_mex_assign(&c9_v_y, sf_mex_create("y", &c9_v_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_v_y, "smoothingTimeTranDynamics",
                  "smoothingTimeTranDynamics", 0);
  c9_w_u = c9_u.DEMO_MOVEMENTS;
  c9_w_y = NULL;
  sf_mex_assign(&c9_w_y, sf_mex_create("y", &c9_w_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_w_y, "DEMO_MOVEMENTS", "DEMO_MOVEMENTS", 0);
  c9_x_u = c9_u.TIME_CONTROLLER_SWITCH;
  c9_x_y = NULL;
  sf_mex_assign(&c9_x_y, sf_mex_create("y", &c9_x_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_x_y, "TIME_CONTROLLER_SWITCH",
                  "TIME_CONTROLLER_SWITCH", 0);
  for (c9_i228 = 0; c9_i228 < 23; c9_i228++) {
    c9_y_u[c9_i228] = c9_u.JOINTS[c9_i228];
  }

  c9_y_y = NULL;
  sf_mex_assign(&c9_y_y, sf_mex_create("y", c9_y_u, 0, 0U, 1U, 0U, 2, 23, 1),
                FALSE);
  sf_mex_addfield(c9_y, c9_y_y, "JOINTS", "JOINTS", 0);
  for (c9_i229 = 0; c9_i229 < 23; c9_i229++) {
    c9_ab_u[c9_i229] = c9_u.JOINTSITING[c9_i229];
  }

  c9_ab_y = NULL;
  sf_mex_assign(&c9_ab_y, sf_mex_create("y", c9_ab_u, 0, 0U, 1U, 0U, 2, 23, 1),
                FALSE);
  sf_mex_addfield(c9_y, c9_ab_y, "JOINTSITING", "JOINTSITING", 0);
  c9_bb_u = c9_u.iCubStandUp;
  c9_bb_y = NULL;
  sf_mex_assign(&c9_bb_y, sf_mex_create("y", &c9_bb_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_bb_y, "iCubStandUp", "iCubStandUp", 0);
  c9_cb_u = c9_u.useExtArmForces;
  c9_cb_y = NULL;
  sf_mex_assign(&c9_cb_y, sf_mex_create("y", &c9_cb_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c9_y, c9_cb_y, "useExtArmForces", "useExtArmForces", 0);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static void c9_c_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  c9_struct_HZPQIAn1UwxVqJtlH5llbD *c9_y)
{
  emlrtMsgIdentifier c9_thisId;
  static const char * c9_fieldNames[21] = { "SIMULATION_TIME", "SCOPES",
    "CHECK_LIMITS", "USE_IMU4EST_BASE", "YAW_IMU_FILTER", "PITCH_IMU_FILTER",
    "CORRECT_NECK_IMU", "ONSOFTCARPET", "USE_QP_SOLVER", "Ts", "ON_GAZEBO",
    "LEFT_RIGHT_FOOT_IN_CONTACT", "SMOOTH_DES_COM", "SMOOTH_DES_Q",
    "smoothingTimeTranDynamics", "DEMO_MOVEMENTS", "TIME_CONTROLLER_SWITCH",
    "JOINTS", "JOINTSITING", "iCubStandUp", "useExtArmForces" };

  c9_thisId.fParent = c9_parentId;
  sf_mex_check_struct(c9_parentId, c9_u, 21, c9_fieldNames, 0U, 0);
  c9_thisId.fIdentifier = "SIMULATION_TIME";
  c9_y->SIMULATION_TIME = c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "SIMULATION_TIME", "SIMULATION_TIME", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "SCOPES";
  c9_y->SCOPES = c9_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
                                        (c9_u, "SCOPES", "SCOPES", 0)),
    &c9_thisId);
  c9_thisId.fIdentifier = "CHECK_LIMITS";
  c9_y->CHECK_LIMITS = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "CHECK_LIMITS", "CHECK_LIMITS", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "USE_IMU4EST_BASE";
  c9_y->USE_IMU4EST_BASE = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "USE_IMU4EST_BASE", "USE_IMU4EST_BASE", 0)),
    &c9_thisId);
  c9_thisId.fIdentifier = "YAW_IMU_FILTER";
  c9_y->YAW_IMU_FILTER = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "YAW_IMU_FILTER", "YAW_IMU_FILTER", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "PITCH_IMU_FILTER";
  c9_y->PITCH_IMU_FILTER = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "PITCH_IMU_FILTER", "PITCH_IMU_FILTER", 0)),
    &c9_thisId);
  c9_thisId.fIdentifier = "CORRECT_NECK_IMU";
  c9_y->CORRECT_NECK_IMU = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "CORRECT_NECK_IMU", "CORRECT_NECK_IMU", 0)),
    &c9_thisId);
  c9_thisId.fIdentifier = "ONSOFTCARPET";
  c9_y->ONSOFTCARPET = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "ONSOFTCARPET", "ONSOFTCARPET", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "USE_QP_SOLVER";
  c9_y->USE_QP_SOLVER = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "USE_QP_SOLVER", "USE_QP_SOLVER", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "Ts";
  c9_y->Ts = c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c9_u, "Ts", "Ts", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "ON_GAZEBO";
  c9_y->ON_GAZEBO = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "ON_GAZEBO", "ON_GAZEBO", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "LEFT_RIGHT_FOOT_IN_CONTACT";
  c9_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c9_u,
    "LEFT_RIGHT_FOOT_IN_CONTACT", "LEFT_RIGHT_FOOT_IN_CONTACT", 0)), &c9_thisId,
                        c9_y->LEFT_RIGHT_FOOT_IN_CONTACT);
  c9_thisId.fIdentifier = "SMOOTH_DES_COM";
  c9_y->SMOOTH_DES_COM = c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "SMOOTH_DES_COM", "SMOOTH_DES_COM", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "SMOOTH_DES_Q";
  c9_y->SMOOTH_DES_Q = c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "SMOOTH_DES_Q", "SMOOTH_DES_Q", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "smoothingTimeTranDynamics";
  c9_y->smoothingTimeTranDynamics = c9_d_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c9_u, "smoothingTimeTranDynamics",
    "smoothingTimeTranDynamics", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "DEMO_MOVEMENTS";
  c9_y->DEMO_MOVEMENTS = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "DEMO_MOVEMENTS", "DEMO_MOVEMENTS", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "TIME_CONTROLLER_SWITCH";
  c9_y->TIME_CONTROLLER_SWITCH = c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "TIME_CONTROLLER_SWITCH", "TIME_CONTROLLER_SWITCH", 0)),
    &c9_thisId);
  c9_thisId.fIdentifier = "JOINTS";
  c9_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c9_u, "JOINTS",
    "JOINTS", 0)), &c9_thisId, c9_y->JOINTS);
  c9_thisId.fIdentifier = "JOINTSITING";
  c9_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c9_u,
    "JOINTSITING", "JOINTSITING", 0)), &c9_thisId, c9_y->JOINTSITING);
  c9_thisId.fIdentifier = "iCubStandUp";
  c9_y->iCubStandUp = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "iCubStandUp", "iCubStandUp", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "useExtArmForces";
  c9_y->useExtArmForces = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "useExtArmForces", "useExtArmForces", 0)), &c9_thisId);
  sf_mex_destroy(&c9_u);
}

static real_T c9_d_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  real_T c9_y;
  real_T c9_d0;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_d0, 1, 0, 0U, 0, 0U, 0);
  c9_y = c9_d0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static c9_struct_amzdx4J7qaaMzgPI7fQ3WD c9_e_emlrt_marshallIn
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c9_u,
   const emlrtMsgIdentifier *c9_parentId)
{
  c9_struct_amzdx4J7qaaMzgPI7fQ3WD c9_y;
  emlrtMsgIdentifier c9_thisId;
  static const char * c9_fieldNames[6] = { "ALL", "BASE_EST_IMU", "EXTWRENCHES",
    "GAIN_SCHE_INFO", "MAIN", "QP" };

  c9_thisId.fParent = c9_parentId;
  sf_mex_check_struct(c9_parentId, c9_u, 6, c9_fieldNames, 0U, 0);
  c9_thisId.fIdentifier = "ALL";
  c9_y.ALL = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c9_u, "ALL", "ALL", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "BASE_EST_IMU";
  c9_y.BASE_EST_IMU = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "BASE_EST_IMU", "BASE_EST_IMU", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "EXTWRENCHES";
  c9_y.EXTWRENCHES = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "EXTWRENCHES", "EXTWRENCHES", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "GAIN_SCHE_INFO";
  c9_y.GAIN_SCHE_INFO = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "GAIN_SCHE_INFO", "GAIN_SCHE_INFO", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "MAIN";
  c9_y.MAIN = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c9_u, "MAIN", "MAIN", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "QP";
  c9_y.QP = c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c9_u,
    "QP", "QP", 0)), &c9_thisId);
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static boolean_T c9_f_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  boolean_T c9_y;
  boolean_T c9_b0;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_b0, 1, 11, 0U, 0, 0U, 0);
  c9_y = c9_b0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_g_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[2])
{
  real_T c9_dv23[2];
  int32_T c9_i230;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv23, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c9_i230 = 0; c9_i230 < 2; c9_i230++) {
    c9_y[c9_i230] = c9_dv23[c9_i230];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_h_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[23])
{
  real_T c9_dv24[23];
  int32_T c9_i231;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv24, 1, 0, 0U, 1, 0U, 2, 23,
                1);
  for (c9_i231 = 0; c9_i231 < 23; c9_i231++) {
    c9_y[c9_i231] = c9_dv24[c9_i231];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_CONFIG;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  c9_struct_HZPQIAn1UwxVqJtlH5llbD c9_y;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_b_CONFIG = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_CONFIG), &c9_thisId,
                        &c9_y);
  sf_mex_destroy(&c9_b_CONFIG);
  *(c9_struct_HZPQIAn1UwxVqJtlH5llbD *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i232;
  real_T c9_b_inData[3];
  int32_T c9_i233;
  real_T c9_u[3];
  const mxArray *c9_y = NULL;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  for (c9_i232 = 0; c9_i232 < 3; c9_i232++) {
    c9_b_inData[c9_i232] = (*(real_T (*)[3])c9_inData)[c9_i232];
  }

  for (c9_i233 = 0; c9_i233 < 3; c9_i233++) {
    c9_u[c9_i233] = c9_b_inData[c9_i233];
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static const mxArray *c9_d_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i234;
  real_T c9_b_inData[12];
  int32_T c9_i235;
  real_T c9_u[12];
  const mxArray *c9_y = NULL;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  for (c9_i234 = 0; c9_i234 < 12; c9_i234++) {
    c9_b_inData[c9_i234] = (*(real_T (*)[12])c9_inData)[c9_i234];
  }

  for (c9_i235 = 0; c9_i235 < 12; c9_i235++) {
    c9_u[c9_i235] = c9_b_inData[c9_i235];
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static const mxArray *c9_e_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  real_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_u = *(real_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_nargout;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_nargout = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_nargout), &c9_thisId);
  sf_mex_destroy(&c9_nargout);
  *(real_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_f_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i236;
  int32_T c9_i237;
  int32_T c9_i238;
  real_T c9_b_inData[9];
  int32_T c9_i239;
  int32_T c9_i240;
  int32_T c9_i241;
  real_T c9_u[9];
  const mxArray *c9_y = NULL;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_i236 = 0;
  for (c9_i237 = 0; c9_i237 < 3; c9_i237++) {
    for (c9_i238 = 0; c9_i238 < 3; c9_i238++) {
      c9_b_inData[c9_i238 + c9_i236] = (*(real_T (*)[9])c9_inData)[c9_i238 +
        c9_i236];
    }

    c9_i236 += 3;
  }

  c9_i239 = 0;
  for (c9_i240 = 0; c9_i240 < 3; c9_i240++) {
    for (c9_i241 = 0; c9_i241 < 3; c9_i241++) {
      c9_u[c9_i241 + c9_i239] = c9_b_inData[c9_i241 + c9_i239];
    }

    c9_i239 += 3;
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static void c9_i_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[9])
{
  real_T c9_dv25[9];
  int32_T c9_i242;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv25, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c9_i242 = 0; c9_i242 < 9; c9_i242++) {
    c9_y[c9_i242] = c9_dv25[c9_i242];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_R;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[9];
  int32_T c9_i243;
  int32_T c9_i244;
  int32_T c9_i245;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_R = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_R), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_R);
  c9_i243 = 0;
  for (c9_i244 = 0; c9_i244 < 3; c9_i244++) {
    for (c9_i245 = 0; c9_i245 < 3; c9_i245++) {
      (*(real_T (*)[9])c9_outData)[c9_i245 + c9_i243] = c9_y[c9_i245 + c9_i243];
    }

    c9_i243 += 3;
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

static void c9_j_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[3])
{
  real_T c9_dv26[3];
  int32_T c9_i246;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv26, 1, 0, 0U, 1, 0U, 1, 3);
  for (c9_i246 = 0; c9_i246 < 3; c9_i246++) {
    c9_y[c9_i246] = c9_dv26[c9_i246];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_rollPitchYaw;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[3];
  int32_T c9_i247;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_rollPitchYaw = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_rollPitchYaw), &c9_thisId,
                        c9_y);
  sf_mex_destroy(&c9_rollPitchYaw);
  for (c9_i247 = 0; c9_i247 < 3; c9_i247++) {
    (*(real_T (*)[3])c9_outData)[c9_i247] = c9_y[c9_i247];
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

static void c9_k_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[12])
{
  real_T c9_dv27[12];
  int32_T c9_i248;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv27, 1, 0, 0U, 1, 0U, 1, 12);
  for (c9_i248 = 0; c9_i248 < 12; c9_i248++) {
    c9_y[c9_i248] = c9_dv27[c9_i248];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_inertial;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[12];
  int32_T c9_i249;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_inertial = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_inertial), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_inertial);
  for (c9_i249 = 0; c9_i249 < 12; c9_i249++) {
    (*(real_T (*)[12])c9_outData)[c9_i249] = c9_y[c9_i249];
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

const mxArray *sf_c9_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c9_nameCaptureInfo;
  c9_ResolvedFunctionInfo c9_info[147];
  const mxArray *c9_m2 = NULL;
  int32_T c9_i250;
  c9_ResolvedFunctionInfo *c9_r1;
  c9_nameCaptureInfo = NULL;
  c9_nameCaptureInfo = NULL;
  c9_info_helper(c9_info);
  c9_b_info_helper(c9_info);
  c9_c_info_helper(c9_info);
  sf_mex_assign(&c9_m2, sf_mex_createstruct("nameCaptureInfo", 1, 147), FALSE);
  for (c9_i250 = 0; c9_i250 < 147; c9_i250++) {
    c9_r1 = &c9_info[c9_i250];
    sf_mex_addfield(c9_m2, sf_mex_create("nameCaptureInfo", c9_r1->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c9_r1->context)), "context", "nameCaptureInfo",
                    c9_i250);
    sf_mex_addfield(c9_m2, sf_mex_create("nameCaptureInfo", c9_r1->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c9_r1->name)), "name", "nameCaptureInfo", c9_i250);
    sf_mex_addfield(c9_m2, sf_mex_create("nameCaptureInfo", c9_r1->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c9_r1->dominantType)), "dominantType",
                    "nameCaptureInfo", c9_i250);
    sf_mex_addfield(c9_m2, sf_mex_create("nameCaptureInfo", c9_r1->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c9_r1->resolved)), "resolved", "nameCaptureInfo",
                    c9_i250);
    sf_mex_addfield(c9_m2, sf_mex_create("nameCaptureInfo", &c9_r1->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c9_i250);
    sf_mex_addfield(c9_m2, sf_mex_create("nameCaptureInfo", &c9_r1->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c9_i250);
    sf_mex_addfield(c9_m2, sf_mex_create("nameCaptureInfo", &c9_r1->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c9_i250);
    sf_mex_addfield(c9_m2, sf_mex_create("nameCaptureInfo", &c9_r1->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c9_i250);
  }

  sf_mex_assign(&c9_nameCaptureInfo, c9_m2, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c9_nameCaptureInfo);
  return c9_nameCaptureInfo;
}

static void c9_info_helper(c9_ResolvedFunctionInfo c9_info[147])
{
  c9_info[0].context = "";
  c9_info[0].name = "fromBaseToWorldWithImu";
  c9_info[0].dominantType = "struct";
  c9_info[0].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[0].fileTimeLo = 1495631764U;
  c9_info[0].fileTimeHi = 0U;
  c9_info[0].mFileTimeLo = 0U;
  c9_info[0].mFileTimeHi = 0U;
  c9_info[1].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[1].name = "mtimes";
  c9_info[1].dominantType = "double";
  c9_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[1].fileTimeLo = 1289519692U;
  c9_info[1].fileTimeHi = 0U;
  c9_info[1].mFileTimeLo = 0U;
  c9_info[1].mFileTimeHi = 0U;
  c9_info[2].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[2].name = "mrdivide";
  c9_info[2].dominantType = "double";
  c9_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c9_info[2].fileTimeLo = 1357951548U;
  c9_info[2].fileTimeHi = 0U;
  c9_info[2].mFileTimeLo = 1319729966U;
  c9_info[2].mFileTimeHi = 0U;
  c9_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c9_info[3].name = "rdivide";
  c9_info[3].dominantType = "double";
  c9_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c9_info[3].fileTimeLo = 1346510388U;
  c9_info[3].fileTimeHi = 0U;
  c9_info[3].mFileTimeLo = 0U;
  c9_info[3].mFileTimeHi = 0U;
  c9_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c9_info[4].name = "eml_scalexp_compatible";
  c9_info[4].dominantType = "double";
  c9_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c9_info[4].fileTimeLo = 1286818796U;
  c9_info[4].fileTimeHi = 0U;
  c9_info[4].mFileTimeLo = 0U;
  c9_info[4].mFileTimeHi = 0U;
  c9_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c9_info[5].name = "eml_div";
  c9_info[5].dominantType = "double";
  c9_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c9_info[5].fileTimeLo = 1313347810U;
  c9_info[5].fileTimeHi = 0U;
  c9_info[5].mFileTimeLo = 0U;
  c9_info[5].mFileTimeHi = 0U;
  c9_info[6].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[6].name = "rotz";
  c9_info[6].dominantType = "double";
  c9_info[6].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c9_info[6].fileTimeLo = 1495631764U;
  c9_info[6].fileTimeHi = 0U;
  c9_info[6].mFileTimeLo = 0U;
  c9_info[6].mFileTimeHi = 0U;
  c9_info[7].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c9_info[7].name = "cos";
  c9_info[7].dominantType = "double";
  c9_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c9_info[7].fileTimeLo = 1343830372U;
  c9_info[7].fileTimeHi = 0U;
  c9_info[7].mFileTimeLo = 0U;
  c9_info[7].mFileTimeHi = 0U;
  c9_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c9_info[8].name = "eml_scalar_cos";
  c9_info[8].dominantType = "double";
  c9_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c9_info[8].fileTimeLo = 1286818722U;
  c9_info[8].fileTimeHi = 0U;
  c9_info[8].mFileTimeLo = 0U;
  c9_info[8].mFileTimeHi = 0U;
  c9_info[9].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c9_info[9].name = "sin";
  c9_info[9].dominantType = "double";
  c9_info[9].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c9_info[9].fileTimeLo = 1343830386U;
  c9_info[9].fileTimeHi = 0U;
  c9_info[9].mFileTimeLo = 0U;
  c9_info[9].mFileTimeHi = 0U;
  c9_info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c9_info[10].name = "eml_scalar_sin";
  c9_info[10].dominantType = "double";
  c9_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c9_info[10].fileTimeLo = 1286818736U;
  c9_info[10].fileTimeHi = 0U;
  c9_info[10].mFileTimeLo = 0U;
  c9_info[10].mFileTimeHi = 0U;
  c9_info[11].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[11].name = "roty";
  c9_info[11].dominantType = "double";
  c9_info[11].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c9_info[11].fileTimeLo = 1495631764U;
  c9_info[11].fileTimeHi = 0U;
  c9_info[11].mFileTimeLo = 0U;
  c9_info[11].mFileTimeHi = 0U;
  c9_info[12].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c9_info[12].name = "cos";
  c9_info[12].dominantType = "double";
  c9_info[12].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c9_info[12].fileTimeLo = 1343830372U;
  c9_info[12].fileTimeHi = 0U;
  c9_info[12].mFileTimeLo = 0U;
  c9_info[12].mFileTimeHi = 0U;
  c9_info[13].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c9_info[13].name = "sin";
  c9_info[13].dominantType = "double";
  c9_info[13].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c9_info[13].fileTimeLo = 1343830386U;
  c9_info[13].fileTimeHi = 0U;
  c9_info[13].mFileTimeLo = 0U;
  c9_info[13].mFileTimeHi = 0U;
  c9_info[14].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[14].name = "eml_index_class";
  c9_info[14].dominantType = "";
  c9_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[14].fileTimeLo = 1323170578U;
  c9_info[14].fileTimeHi = 0U;
  c9_info[14].mFileTimeLo = 0U;
  c9_info[14].mFileTimeHi = 0U;
  c9_info[15].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[15].name = "eml_scalar_eg";
  c9_info[15].dominantType = "double";
  c9_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[15].fileTimeLo = 1286818796U;
  c9_info[15].fileTimeHi = 0U;
  c9_info[15].mFileTimeLo = 0U;
  c9_info[15].mFileTimeHi = 0U;
  c9_info[16].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[16].name = "eml_xgemm";
  c9_info[16].dominantType = "char";
  c9_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c9_info[16].fileTimeLo = 1299076772U;
  c9_info[16].fileTimeHi = 0U;
  c9_info[16].mFileTimeLo = 0U;
  c9_info[16].mFileTimeHi = 0U;
  c9_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c9_info[17].name = "eml_blas_inline";
  c9_info[17].dominantType = "";
  c9_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c9_info[17].fileTimeLo = 1299076768U;
  c9_info[17].fileTimeHi = 0U;
  c9_info[17].mFileTimeLo = 0U;
  c9_info[17].mFileTimeHi = 0U;
  c9_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c9_info[18].name = "mtimes";
  c9_info[18].dominantType = "double";
  c9_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[18].fileTimeLo = 1289519692U;
  c9_info[18].fileTimeHi = 0U;
  c9_info[18].mFileTimeLo = 0U;
  c9_info[18].mFileTimeHi = 0U;
  c9_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c9_info[19].name = "eml_index_class";
  c9_info[19].dominantType = "";
  c9_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[19].fileTimeLo = 1323170578U;
  c9_info[19].fileTimeHi = 0U;
  c9_info[19].mFileTimeLo = 0U;
  c9_info[19].mFileTimeHi = 0U;
  c9_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c9_info[20].name = "eml_scalar_eg";
  c9_info[20].dominantType = "double";
  c9_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[20].fileTimeLo = 1286818796U;
  c9_info[20].fileTimeHi = 0U;
  c9_info[20].mFileTimeLo = 0U;
  c9_info[20].mFileTimeHi = 0U;
  c9_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c9_info[21].name = "eml_refblas_xgemm";
  c9_info[21].dominantType = "char";
  c9_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c9_info[21].fileTimeLo = 1299076774U;
  c9_info[21].fileTimeHi = 0U;
  c9_info[21].mFileTimeLo = 0U;
  c9_info[21].mFileTimeHi = 0U;
  c9_info[22].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[22].name = "rotx";
  c9_info[22].dominantType = "double";
  c9_info[22].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c9_info[22].fileTimeLo = 1495631764U;
  c9_info[22].fileTimeHi = 0U;
  c9_info[22].mFileTimeLo = 0U;
  c9_info[22].mFileTimeHi = 0U;
  c9_info[23].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c9_info[23].name = "cos";
  c9_info[23].dominantType = "double";
  c9_info[23].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c9_info[23].fileTimeLo = 1343830372U;
  c9_info[23].fileTimeHi = 0U;
  c9_info[23].mFileTimeLo = 0U;
  c9_info[23].mFileTimeHi = 0U;
  c9_info[24].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c9_info[24].name = "sin";
  c9_info[24].dominantType = "double";
  c9_info[24].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c9_info[24].fileTimeLo = 1343830386U;
  c9_info[24].fileTimeHi = 0U;
  c9_info[24].mFileTimeLo = 0U;
  c9_info[24].mFileTimeHi = 0U;
  c9_info[25].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[25].name = "rollPitchYawFromRotation";
  c9_info[25].dominantType = "double";
  c9_info[25].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c9_info[25].fileTimeLo = 1495631764U;
  c9_info[25].fileTimeHi = 0U;
  c9_info[25].mFileTimeLo = 0U;
  c9_info[25].mFileTimeHi = 0U;
  c9_info[26].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c9_info[26].name = "asin";
  c9_info[26].dominantType = "double";
  c9_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c9_info[26].fileTimeLo = 1343830370U;
  c9_info[26].fileTimeHi = 0U;
  c9_info[26].mFileTimeLo = 0U;
  c9_info[26].mFileTimeHi = 0U;
  c9_info[27].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c9_info[27].name = "eml_error";
  c9_info[27].dominantType = "char";
  c9_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c9_info[27].fileTimeLo = 1343830358U;
  c9_info[27].fileTimeHi = 0U;
  c9_info[27].mFileTimeLo = 0U;
  c9_info[27].mFileTimeHi = 0U;
  c9_info[28].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c9_info[28].name = "eml_scalar_asin";
  c9_info[28].dominantType = "double";
  c9_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_asin.m";
  c9_info[28].fileTimeLo = 1343830376U;
  c9_info[28].fileTimeHi = 0U;
  c9_info[28].mFileTimeLo = 0U;
  c9_info[28].mFileTimeHi = 0U;
  c9_info[29].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c9_info[29].name = "atan2";
  c9_info[29].dominantType = "double";
  c9_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c9_info[29].fileTimeLo = 1343830372U;
  c9_info[29].fileTimeHi = 0U;
  c9_info[29].mFileTimeLo = 0U;
  c9_info[29].mFileTimeHi = 0U;
  c9_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c9_info[30].name = "eml_scalar_eg";
  c9_info[30].dominantType = "double";
  c9_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[30].fileTimeLo = 1286818796U;
  c9_info[30].fileTimeHi = 0U;
  c9_info[30].mFileTimeLo = 0U;
  c9_info[30].mFileTimeHi = 0U;
  c9_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c9_info[31].name = "eml_scalexp_alloc";
  c9_info[31].dominantType = "double";
  c9_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c9_info[31].fileTimeLo = 1352424860U;
  c9_info[31].fileTimeHi = 0U;
  c9_info[31].mFileTimeLo = 0U;
  c9_info[31].mFileTimeHi = 0U;
  c9_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c9_info[32].name = "eml_scalar_atan2";
  c9_info[32].dominantType = "double";
  c9_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m";
  c9_info[32].fileTimeLo = 1286818720U;
  c9_info[32].fileTimeHi = 0U;
  c9_info[32].mFileTimeLo = 0U;
  c9_info[32].mFileTimeHi = 0U;
  c9_info[33].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[33].name = "correctIMU";
  c9_info[33].dominantType = "double";
  c9_info[33].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c9_info[33].fileTimeLo = 1495631764U;
  c9_info[33].fileTimeHi = 0U;
  c9_info[33].mFileTimeLo = 0U;
  c9_info[33].mFileTimeHi = 0U;
  c9_info[34].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c9_info[34].name = "mtimes";
  c9_info[34].dominantType = "double";
  c9_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[34].fileTimeLo = 1289519692U;
  c9_info[34].fileTimeHi = 0U;
  c9_info[34].mFileTimeLo = 0U;
  c9_info[34].mFileTimeHi = 0U;
  c9_info[35].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c9_info[35].name = "mrdivide";
  c9_info[35].dominantType = "double";
  c9_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c9_info[35].fileTimeLo = 1357951548U;
  c9_info[35].fileTimeHi = 0U;
  c9_info[35].mFileTimeLo = 1319729966U;
  c9_info[35].mFileTimeHi = 0U;
  c9_info[36].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c9_info[36].name = "evalDHMatrix";
  c9_info[36].dominantType = "double";
  c9_info[36].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c9_info[36].fileTimeLo = 1495631765U;
  c9_info[36].fileTimeHi = 0U;
  c9_info[36].mFileTimeLo = 0U;
  c9_info[36].mFileTimeHi = 0U;
  c9_info[37].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c9_info[37].name = "cos";
  c9_info[37].dominantType = "double";
  c9_info[37].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c9_info[37].fileTimeLo = 1343830372U;
  c9_info[37].fileTimeHi = 0U;
  c9_info[37].mFileTimeLo = 0U;
  c9_info[37].mFileTimeHi = 0U;
  c9_info[38].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c9_info[38].name = "sin";
  c9_info[38].dominantType = "double";
  c9_info[38].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c9_info[38].fileTimeLo = 1343830386U;
  c9_info[38].fileTimeHi = 0U;
  c9_info[38].mFileTimeLo = 0U;
  c9_info[38].mFileTimeHi = 0U;
  c9_info[39].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c9_info[39].name = "mtimes";
  c9_info[39].dominantType = "double";
  c9_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[39].fileTimeLo = 1289519692U;
  c9_info[39].fileTimeHi = 0U;
  c9_info[39].mFileTimeLo = 0U;
  c9_info[39].mFileTimeHi = 0U;
  c9_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c9_info[40].name = "mldivide";
  c9_info[40].dominantType = "double";
  c9_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c9_info[40].fileTimeLo = 1357951548U;
  c9_info[40].fileTimeHi = 0U;
  c9_info[40].mFileTimeLo = 1319729966U;
  c9_info[40].mFileTimeHi = 0U;
  c9_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c9_info[41].name = "eml_lusolve";
  c9_info[41].dominantType = "double";
  c9_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c9_info[41].fileTimeLo = 1309451196U;
  c9_info[41].fileTimeHi = 0U;
  c9_info[41].mFileTimeLo = 0U;
  c9_info[41].mFileTimeHi = 0U;
  c9_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c9_info[42].name = "eml_index_class";
  c9_info[42].dominantType = "";
  c9_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[42].fileTimeLo = 1323170578U;
  c9_info[42].fileTimeHi = 0U;
  c9_info[42].mFileTimeLo = 0U;
  c9_info[42].mFileTimeHi = 0U;
  c9_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c9_info[43].name = "eml_index_class";
  c9_info[43].dominantType = "";
  c9_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[43].fileTimeLo = 1323170578U;
  c9_info[43].fileTimeHi = 0U;
  c9_info[43].mFileTimeLo = 0U;
  c9_info[43].mFileTimeHi = 0U;
  c9_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c9_info[44].name = "eml_xgetrf";
  c9_info[44].dominantType = "double";
  c9_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c9_info[44].fileTimeLo = 1286818806U;
  c9_info[44].fileTimeHi = 0U;
  c9_info[44].mFileTimeLo = 0U;
  c9_info[44].mFileTimeHi = 0U;
  c9_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c9_info[45].name = "eml_lapack_xgetrf";
  c9_info[45].dominantType = "double";
  c9_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c9_info[45].fileTimeLo = 1286818810U;
  c9_info[45].fileTimeHi = 0U;
  c9_info[45].mFileTimeLo = 0U;
  c9_info[45].mFileTimeHi = 0U;
  c9_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c9_info[46].name = "eml_matlab_zgetrf";
  c9_info[46].dominantType = "double";
  c9_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[46].fileTimeLo = 1302688994U;
  c9_info[46].fileTimeHi = 0U;
  c9_info[46].mFileTimeLo = 0U;
  c9_info[46].mFileTimeHi = 0U;
  c9_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[47].name = "realmin";
  c9_info[47].dominantType = "char";
  c9_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c9_info[47].fileTimeLo = 1307651242U;
  c9_info[47].fileTimeHi = 0U;
  c9_info[47].mFileTimeLo = 0U;
  c9_info[47].mFileTimeHi = 0U;
  c9_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c9_info[48].name = "eml_realmin";
  c9_info[48].dominantType = "char";
  c9_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c9_info[48].fileTimeLo = 1307651244U;
  c9_info[48].fileTimeHi = 0U;
  c9_info[48].mFileTimeLo = 0U;
  c9_info[48].mFileTimeHi = 0U;
  c9_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c9_info[49].name = "eml_float_model";
  c9_info[49].dominantType = "char";
  c9_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c9_info[49].fileTimeLo = 1326727996U;
  c9_info[49].fileTimeHi = 0U;
  c9_info[49].mFileTimeLo = 0U;
  c9_info[49].mFileTimeHi = 0U;
  c9_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[50].name = "eps";
  c9_info[50].dominantType = "char";
  c9_info[50].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c9_info[50].fileTimeLo = 1326727996U;
  c9_info[50].fileTimeHi = 0U;
  c9_info[50].mFileTimeLo = 0U;
  c9_info[50].mFileTimeHi = 0U;
  c9_info[51].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c9_info[51].name = "eml_is_float_class";
  c9_info[51].dominantType = "char";
  c9_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c9_info[51].fileTimeLo = 1286818782U;
  c9_info[51].fileTimeHi = 0U;
  c9_info[51].mFileTimeLo = 0U;
  c9_info[51].mFileTimeHi = 0U;
  c9_info[52].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c9_info[52].name = "eml_eps";
  c9_info[52].dominantType = "char";
  c9_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c9_info[52].fileTimeLo = 1326727996U;
  c9_info[52].fileTimeHi = 0U;
  c9_info[52].mFileTimeLo = 0U;
  c9_info[52].mFileTimeHi = 0U;
  c9_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c9_info[53].name = "eml_float_model";
  c9_info[53].dominantType = "char";
  c9_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c9_info[53].fileTimeLo = 1326727996U;
  c9_info[53].fileTimeHi = 0U;
  c9_info[53].mFileTimeLo = 0U;
  c9_info[53].mFileTimeHi = 0U;
  c9_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[54].name = "min";
  c9_info[54].dominantType = "coder.internal.indexInt";
  c9_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c9_info[54].fileTimeLo = 1311255318U;
  c9_info[54].fileTimeHi = 0U;
  c9_info[54].mFileTimeLo = 0U;
  c9_info[54].mFileTimeHi = 0U;
  c9_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c9_info[55].name = "eml_min_or_max";
  c9_info[55].dominantType = "char";
  c9_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c9_info[55].fileTimeLo = 1334071490U;
  c9_info[55].fileTimeHi = 0U;
  c9_info[55].mFileTimeLo = 0U;
  c9_info[55].mFileTimeHi = 0U;
  c9_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c9_info[56].name = "eml_scalar_eg";
  c9_info[56].dominantType = "coder.internal.indexInt";
  c9_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[56].fileTimeLo = 1286818796U;
  c9_info[56].fileTimeHi = 0U;
  c9_info[56].mFileTimeLo = 0U;
  c9_info[56].mFileTimeHi = 0U;
  c9_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c9_info[57].name = "eml_scalexp_alloc";
  c9_info[57].dominantType = "coder.internal.indexInt";
  c9_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c9_info[57].fileTimeLo = 1352424860U;
  c9_info[57].fileTimeHi = 0U;
  c9_info[57].mFileTimeLo = 0U;
  c9_info[57].mFileTimeHi = 0U;
  c9_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c9_info[58].name = "eml_index_class";
  c9_info[58].dominantType = "";
  c9_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[58].fileTimeLo = 1323170578U;
  c9_info[58].fileTimeHi = 0U;
  c9_info[58].mFileTimeLo = 0U;
  c9_info[58].mFileTimeHi = 0U;
  c9_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c9_info[59].name = "eml_scalar_eg";
  c9_info[59].dominantType = "coder.internal.indexInt";
  c9_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[59].fileTimeLo = 1286818796U;
  c9_info[59].fileTimeHi = 0U;
  c9_info[59].mFileTimeLo = 0U;
  c9_info[59].mFileTimeHi = 0U;
  c9_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[60].name = "colon";
  c9_info[60].dominantType = "double";
  c9_info[60].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c9_info[60].fileTimeLo = 1348191928U;
  c9_info[60].fileTimeHi = 0U;
  c9_info[60].mFileTimeLo = 0U;
  c9_info[60].mFileTimeHi = 0U;
  c9_info[61].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c9_info[61].name = "colon";
  c9_info[61].dominantType = "double";
  c9_info[61].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c9_info[61].fileTimeLo = 1348191928U;
  c9_info[61].fileTimeHi = 0U;
  c9_info[61].mFileTimeLo = 0U;
  c9_info[61].mFileTimeHi = 0U;
  c9_info[62].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c9_info[62].name = "floor";
  c9_info[62].dominantType = "double";
  c9_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c9_info[62].fileTimeLo = 1343830380U;
  c9_info[62].fileTimeHi = 0U;
  c9_info[62].mFileTimeLo = 0U;
  c9_info[62].mFileTimeHi = 0U;
  c9_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c9_info[63].name = "eml_scalar_floor";
  c9_info[63].dominantType = "double";
  c9_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c9_info[63].fileTimeLo = 1286818726U;
  c9_info[63].fileTimeHi = 0U;
  c9_info[63].mFileTimeLo = 0U;
  c9_info[63].mFileTimeHi = 0U;
}

static void c9_b_info_helper(c9_ResolvedFunctionInfo c9_info[147])
{
  c9_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c9_info[64].name = "intmin";
  c9_info[64].dominantType = "char";
  c9_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c9_info[64].fileTimeLo = 1311255318U;
  c9_info[64].fileTimeHi = 0U;
  c9_info[64].mFileTimeLo = 0U;
  c9_info[64].mFileTimeHi = 0U;
  c9_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c9_info[65].name = "intmax";
  c9_info[65].dominantType = "char";
  c9_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c9_info[65].fileTimeLo = 1311255316U;
  c9_info[65].fileTimeHi = 0U;
  c9_info[65].mFileTimeLo = 0U;
  c9_info[65].mFileTimeHi = 0U;
  c9_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c9_info[66].name = "intmin";
  c9_info[66].dominantType = "char";
  c9_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c9_info[66].fileTimeLo = 1311255318U;
  c9_info[66].fileTimeHi = 0U;
  c9_info[66].mFileTimeLo = 0U;
  c9_info[66].mFileTimeHi = 0U;
  c9_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c9_info[67].name = "intmax";
  c9_info[67].dominantType = "char";
  c9_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c9_info[67].fileTimeLo = 1311255316U;
  c9_info[67].fileTimeHi = 0U;
  c9_info[67].mFileTimeLo = 0U;
  c9_info[67].mFileTimeHi = 0U;
  c9_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c9_info[68].name = "eml_isa_uint";
  c9_info[68].dominantType = "coder.internal.indexInt";
  c9_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c9_info[68].fileTimeLo = 1286818784U;
  c9_info[68].fileTimeHi = 0U;
  c9_info[68].mFileTimeLo = 0U;
  c9_info[68].mFileTimeHi = 0U;
  c9_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c9_info[69].name = "eml_unsigned_class";
  c9_info[69].dominantType = "char";
  c9_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c9_info[69].fileTimeLo = 1323170580U;
  c9_info[69].fileTimeHi = 0U;
  c9_info[69].mFileTimeLo = 0U;
  c9_info[69].mFileTimeHi = 0U;
  c9_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c9_info[70].name = "eml_index_class";
  c9_info[70].dominantType = "";
  c9_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[70].fileTimeLo = 1323170578U;
  c9_info[70].fileTimeHi = 0U;
  c9_info[70].mFileTimeLo = 0U;
  c9_info[70].mFileTimeHi = 0U;
  c9_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c9_info[71].name = "eml_index_class";
  c9_info[71].dominantType = "";
  c9_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[71].fileTimeLo = 1323170578U;
  c9_info[71].fileTimeHi = 0U;
  c9_info[71].mFileTimeLo = 0U;
  c9_info[71].mFileTimeHi = 0U;
  c9_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c9_info[72].name = "intmax";
  c9_info[72].dominantType = "char";
  c9_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c9_info[72].fileTimeLo = 1311255316U;
  c9_info[72].fileTimeHi = 0U;
  c9_info[72].mFileTimeLo = 0U;
  c9_info[72].mFileTimeHi = 0U;
  c9_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c9_info[73].name = "eml_isa_uint";
  c9_info[73].dominantType = "coder.internal.indexInt";
  c9_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c9_info[73].fileTimeLo = 1286818784U;
  c9_info[73].fileTimeHi = 0U;
  c9_info[73].mFileTimeLo = 0U;
  c9_info[73].mFileTimeHi = 0U;
  c9_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c9_info[74].name = "eml_index_plus";
  c9_info[74].dominantType = "double";
  c9_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[74].fileTimeLo = 1286818778U;
  c9_info[74].fileTimeHi = 0U;
  c9_info[74].mFileTimeLo = 0U;
  c9_info[74].mFileTimeHi = 0U;
  c9_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[75].name = "eml_index_class";
  c9_info[75].dominantType = "";
  c9_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[75].fileTimeLo = 1323170578U;
  c9_info[75].fileTimeHi = 0U;
  c9_info[75].mFileTimeLo = 0U;
  c9_info[75].mFileTimeHi = 0U;
  c9_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c9_info[76].name = "eml_int_forloop_overflow_check";
  c9_info[76].dominantType = "";
  c9_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c9_info[76].fileTimeLo = 1346510340U;
  c9_info[76].fileTimeHi = 0U;
  c9_info[76].mFileTimeLo = 0U;
  c9_info[76].mFileTimeHi = 0U;
  c9_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c9_info[77].name = "intmax";
  c9_info[77].dominantType = "char";
  c9_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c9_info[77].fileTimeLo = 1311255316U;
  c9_info[77].fileTimeHi = 0U;
  c9_info[77].mFileTimeLo = 0U;
  c9_info[77].mFileTimeHi = 0U;
  c9_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[78].name = "eml_index_class";
  c9_info[78].dominantType = "";
  c9_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[78].fileTimeLo = 1323170578U;
  c9_info[78].fileTimeHi = 0U;
  c9_info[78].mFileTimeLo = 0U;
  c9_info[78].mFileTimeHi = 0U;
  c9_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[79].name = "eml_index_plus";
  c9_info[79].dominantType = "double";
  c9_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[79].fileTimeLo = 1286818778U;
  c9_info[79].fileTimeHi = 0U;
  c9_info[79].mFileTimeLo = 0U;
  c9_info[79].mFileTimeHi = 0U;
  c9_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[80].name = "eml_int_forloop_overflow_check";
  c9_info[80].dominantType = "";
  c9_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c9_info[80].fileTimeLo = 1346510340U;
  c9_info[80].fileTimeHi = 0U;
  c9_info[80].mFileTimeLo = 0U;
  c9_info[80].mFileTimeHi = 0U;
  c9_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[81].name = "eml_index_minus";
  c9_info[81].dominantType = "double";
  c9_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c9_info[81].fileTimeLo = 1286818778U;
  c9_info[81].fileTimeHi = 0U;
  c9_info[81].mFileTimeLo = 0U;
  c9_info[81].mFileTimeHi = 0U;
  c9_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c9_info[82].name = "eml_index_class";
  c9_info[82].dominantType = "";
  c9_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[82].fileTimeLo = 1323170578U;
  c9_info[82].fileTimeHi = 0U;
  c9_info[82].mFileTimeLo = 0U;
  c9_info[82].mFileTimeHi = 0U;
  c9_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[83].name = "eml_index_minus";
  c9_info[83].dominantType = "coder.internal.indexInt";
  c9_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c9_info[83].fileTimeLo = 1286818778U;
  c9_info[83].fileTimeHi = 0U;
  c9_info[83].mFileTimeLo = 0U;
  c9_info[83].mFileTimeHi = 0U;
  c9_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[84].name = "eml_index_times";
  c9_info[84].dominantType = "coder.internal.indexInt";
  c9_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c9_info[84].fileTimeLo = 1286818780U;
  c9_info[84].fileTimeHi = 0U;
  c9_info[84].mFileTimeLo = 0U;
  c9_info[84].mFileTimeHi = 0U;
  c9_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c9_info[85].name = "eml_index_class";
  c9_info[85].dominantType = "";
  c9_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[85].fileTimeLo = 1323170578U;
  c9_info[85].fileTimeHi = 0U;
  c9_info[85].mFileTimeLo = 0U;
  c9_info[85].mFileTimeHi = 0U;
  c9_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[86].name = "eml_index_plus";
  c9_info[86].dominantType = "coder.internal.indexInt";
  c9_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[86].fileTimeLo = 1286818778U;
  c9_info[86].fileTimeHi = 0U;
  c9_info[86].mFileTimeLo = 0U;
  c9_info[86].mFileTimeHi = 0U;
  c9_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[87].name = "eml_ixamax";
  c9_info[87].dominantType = "double";
  c9_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c9_info[87].fileTimeLo = 1299076770U;
  c9_info[87].fileTimeHi = 0U;
  c9_info[87].mFileTimeLo = 0U;
  c9_info[87].mFileTimeHi = 0U;
  c9_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c9_info[88].name = "eml_blas_inline";
  c9_info[88].dominantType = "";
  c9_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c9_info[88].fileTimeLo = 1299076768U;
  c9_info[88].fileTimeHi = 0U;
  c9_info[88].mFileTimeLo = 0U;
  c9_info[88].mFileTimeHi = 0U;
  c9_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c9_info[89].name = "length";
  c9_info[89].dominantType = "double";
  c9_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c9_info[89].fileTimeLo = 1303146206U;
  c9_info[89].fileTimeHi = 0U;
  c9_info[89].mFileTimeLo = 0U;
  c9_info[89].mFileTimeHi = 0U;
  c9_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c9_info[90].name = "eml_index_class";
  c9_info[90].dominantType = "";
  c9_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[90].fileTimeLo = 1323170578U;
  c9_info[90].fileTimeHi = 0U;
  c9_info[90].mFileTimeLo = 0U;
  c9_info[90].mFileTimeHi = 0U;
  c9_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c9_info[91].name = "eml_index_class";
  c9_info[91].dominantType = "";
  c9_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[91].fileTimeLo = 1323170578U;
  c9_info[91].fileTimeHi = 0U;
  c9_info[91].mFileTimeLo = 0U;
  c9_info[91].mFileTimeHi = 0U;
  c9_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c9_info[92].name = "eml_refblas_ixamax";
  c9_info[92].dominantType = "double";
  c9_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c9_info[92].fileTimeLo = 1299076770U;
  c9_info[92].fileTimeHi = 0U;
  c9_info[92].mFileTimeLo = 0U;
  c9_info[92].mFileTimeHi = 0U;
  c9_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c9_info[93].name = "eml_index_class";
  c9_info[93].dominantType = "";
  c9_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[93].fileTimeLo = 1323170578U;
  c9_info[93].fileTimeHi = 0U;
  c9_info[93].mFileTimeLo = 0U;
  c9_info[93].mFileTimeHi = 0U;
  c9_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c9_info[94].name = "eml_xcabs1";
  c9_info[94].dominantType = "double";
  c9_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c9_info[94].fileTimeLo = 1286818706U;
  c9_info[94].fileTimeHi = 0U;
  c9_info[94].mFileTimeLo = 0U;
  c9_info[94].mFileTimeHi = 0U;
  c9_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c9_info[95].name = "abs";
  c9_info[95].dominantType = "double";
  c9_info[95].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c9_info[95].fileTimeLo = 1343830366U;
  c9_info[95].fileTimeHi = 0U;
  c9_info[95].mFileTimeLo = 0U;
  c9_info[95].mFileTimeHi = 0U;
  c9_info[96].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c9_info[96].name = "eml_scalar_abs";
  c9_info[96].dominantType = "double";
  c9_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c9_info[96].fileTimeLo = 1286818712U;
  c9_info[96].fileTimeHi = 0U;
  c9_info[96].mFileTimeLo = 0U;
  c9_info[96].mFileTimeHi = 0U;
  c9_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c9_info[97].name = "eml_int_forloop_overflow_check";
  c9_info[97].dominantType = "";
  c9_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c9_info[97].fileTimeLo = 1346510340U;
  c9_info[97].fileTimeHi = 0U;
  c9_info[97].mFileTimeLo = 0U;
  c9_info[97].mFileTimeHi = 0U;
  c9_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c9_info[98].name = "eml_index_plus";
  c9_info[98].dominantType = "coder.internal.indexInt";
  c9_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[98].fileTimeLo = 1286818778U;
  c9_info[98].fileTimeHi = 0U;
  c9_info[98].mFileTimeLo = 0U;
  c9_info[98].mFileTimeHi = 0U;
  c9_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[99].name = "eml_xswap";
  c9_info[99].dominantType = "double";
  c9_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c9_info[99].fileTimeLo = 1299076778U;
  c9_info[99].fileTimeHi = 0U;
  c9_info[99].mFileTimeLo = 0U;
  c9_info[99].mFileTimeHi = 0U;
  c9_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c9_info[100].name = "eml_blas_inline";
  c9_info[100].dominantType = "";
  c9_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c9_info[100].fileTimeLo = 1299076768U;
  c9_info[100].fileTimeHi = 0U;
  c9_info[100].mFileTimeLo = 0U;
  c9_info[100].mFileTimeHi = 0U;
  c9_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c9_info[101].name = "eml_index_class";
  c9_info[101].dominantType = "";
  c9_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[101].fileTimeLo = 1323170578U;
  c9_info[101].fileTimeHi = 0U;
  c9_info[101].mFileTimeLo = 0U;
  c9_info[101].mFileTimeHi = 0U;
  c9_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c9_info[102].name = "eml_refblas_xswap";
  c9_info[102].dominantType = "double";
  c9_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c9_info[102].fileTimeLo = 1299076786U;
  c9_info[102].fileTimeHi = 0U;
  c9_info[102].mFileTimeLo = 0U;
  c9_info[102].mFileTimeHi = 0U;
  c9_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c9_info[103].name = "eml_index_class";
  c9_info[103].dominantType = "";
  c9_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[103].fileTimeLo = 1323170578U;
  c9_info[103].fileTimeHi = 0U;
  c9_info[103].mFileTimeLo = 0U;
  c9_info[103].mFileTimeHi = 0U;
  c9_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c9_info[104].name = "abs";
  c9_info[104].dominantType = "coder.internal.indexInt";
  c9_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c9_info[104].fileTimeLo = 1343830366U;
  c9_info[104].fileTimeHi = 0U;
  c9_info[104].mFileTimeLo = 0U;
  c9_info[104].mFileTimeHi = 0U;
  c9_info[105].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c9_info[105].name = "eml_scalar_abs";
  c9_info[105].dominantType = "coder.internal.indexInt";
  c9_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c9_info[105].fileTimeLo = 1286818712U;
  c9_info[105].fileTimeHi = 0U;
  c9_info[105].mFileTimeLo = 0U;
  c9_info[105].mFileTimeHi = 0U;
  c9_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c9_info[106].name = "eml_int_forloop_overflow_check";
  c9_info[106].dominantType = "";
  c9_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c9_info[106].fileTimeLo = 1346510340U;
  c9_info[106].fileTimeHi = 0U;
  c9_info[106].mFileTimeLo = 0U;
  c9_info[106].mFileTimeHi = 0U;
  c9_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c9_info[107].name = "eml_index_plus";
  c9_info[107].dominantType = "coder.internal.indexInt";
  c9_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[107].fileTimeLo = 1286818778U;
  c9_info[107].fileTimeHi = 0U;
  c9_info[107].mFileTimeLo = 0U;
  c9_info[107].mFileTimeHi = 0U;
  c9_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[108].name = "eml_div";
  c9_info[108].dominantType = "double";
  c9_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c9_info[108].fileTimeLo = 1313347810U;
  c9_info[108].fileTimeHi = 0U;
  c9_info[108].mFileTimeLo = 0U;
  c9_info[108].mFileTimeHi = 0U;
  c9_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c9_info[109].name = "eml_xgeru";
  c9_info[109].dominantType = "double";
  c9_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c9_info[109].fileTimeLo = 1299076774U;
  c9_info[109].fileTimeHi = 0U;
  c9_info[109].mFileTimeLo = 0U;
  c9_info[109].mFileTimeHi = 0U;
  c9_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c9_info[110].name = "eml_blas_inline";
  c9_info[110].dominantType = "";
  c9_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c9_info[110].fileTimeLo = 1299076768U;
  c9_info[110].fileTimeHi = 0U;
  c9_info[110].mFileTimeLo = 0U;
  c9_info[110].mFileTimeHi = 0U;
  c9_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c9_info[111].name = "eml_xger";
  c9_info[111].dominantType = "double";
  c9_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c9_info[111].fileTimeLo = 1299076774U;
  c9_info[111].fileTimeHi = 0U;
  c9_info[111].mFileTimeLo = 0U;
  c9_info[111].mFileTimeHi = 0U;
  c9_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c9_info[112].name = "eml_blas_inline";
  c9_info[112].dominantType = "";
  c9_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c9_info[112].fileTimeLo = 1299076768U;
  c9_info[112].fileTimeHi = 0U;
  c9_info[112].mFileTimeLo = 0U;
  c9_info[112].mFileTimeHi = 0U;
  c9_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c9_info[113].name = "intmax";
  c9_info[113].dominantType = "char";
  c9_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c9_info[113].fileTimeLo = 1311255316U;
  c9_info[113].fileTimeHi = 0U;
  c9_info[113].mFileTimeLo = 0U;
  c9_info[113].mFileTimeHi = 0U;
  c9_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c9_info[114].name = "min";
  c9_info[114].dominantType = "double";
  c9_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c9_info[114].fileTimeLo = 1311255318U;
  c9_info[114].fileTimeHi = 0U;
  c9_info[114].mFileTimeLo = 0U;
  c9_info[114].mFileTimeHi = 0U;
  c9_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c9_info[115].name = "eml_scalar_eg";
  c9_info[115].dominantType = "double";
  c9_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[115].fileTimeLo = 1286818796U;
  c9_info[115].fileTimeHi = 0U;
  c9_info[115].mFileTimeLo = 0U;
  c9_info[115].mFileTimeHi = 0U;
  c9_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c9_info[116].name = "eml_scalexp_alloc";
  c9_info[116].dominantType = "double";
  c9_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c9_info[116].fileTimeLo = 1352424860U;
  c9_info[116].fileTimeHi = 0U;
  c9_info[116].mFileTimeLo = 0U;
  c9_info[116].mFileTimeHi = 0U;
  c9_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c9_info[117].name = "eml_scalar_eg";
  c9_info[117].dominantType = "double";
  c9_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[117].fileTimeLo = 1286818796U;
  c9_info[117].fileTimeHi = 0U;
  c9_info[117].mFileTimeLo = 0U;
  c9_info[117].mFileTimeHi = 0U;
  c9_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c9_info[118].name = "mtimes";
  c9_info[118].dominantType = "double";
  c9_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[118].fileTimeLo = 1289519692U;
  c9_info[118].fileTimeHi = 0U;
  c9_info[118].mFileTimeLo = 0U;
  c9_info[118].mFileTimeHi = 0U;
  c9_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c9_info[119].name = "eml_index_class";
  c9_info[119].dominantType = "";
  c9_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[119].fileTimeLo = 1323170578U;
  c9_info[119].fileTimeHi = 0U;
  c9_info[119].mFileTimeLo = 0U;
  c9_info[119].mFileTimeHi = 0U;
  c9_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c9_info[120].name = "eml_refblas_xger";
  c9_info[120].dominantType = "double";
  c9_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c9_info[120].fileTimeLo = 1299076776U;
  c9_info[120].fileTimeHi = 0U;
  c9_info[120].mFileTimeLo = 0U;
  c9_info[120].mFileTimeHi = 0U;
  c9_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c9_info[121].name = "eml_refblas_xgerx";
  c9_info[121].dominantType = "char";
  c9_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c9_info[121].fileTimeLo = 1299076778U;
  c9_info[121].fileTimeHi = 0U;
  c9_info[121].mFileTimeLo = 0U;
  c9_info[121].mFileTimeHi = 0U;
  c9_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c9_info[122].name = "eml_index_class";
  c9_info[122].dominantType = "";
  c9_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[122].fileTimeLo = 1323170578U;
  c9_info[122].fileTimeHi = 0U;
  c9_info[122].mFileTimeLo = 0U;
  c9_info[122].mFileTimeHi = 0U;
  c9_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c9_info[123].name = "abs";
  c9_info[123].dominantType = "coder.internal.indexInt";
  c9_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c9_info[123].fileTimeLo = 1343830366U;
  c9_info[123].fileTimeHi = 0U;
  c9_info[123].mFileTimeLo = 0U;
  c9_info[123].mFileTimeHi = 0U;
  c9_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c9_info[124].name = "eml_index_minus";
  c9_info[124].dominantType = "double";
  c9_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c9_info[124].fileTimeLo = 1286818778U;
  c9_info[124].fileTimeHi = 0U;
  c9_info[124].mFileTimeLo = 0U;
  c9_info[124].mFileTimeHi = 0U;
  c9_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c9_info[125].name = "eml_int_forloop_overflow_check";
  c9_info[125].dominantType = "";
  c9_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c9_info[125].fileTimeLo = 1346510340U;
  c9_info[125].fileTimeHi = 0U;
  c9_info[125].mFileTimeLo = 0U;
  c9_info[125].mFileTimeHi = 0U;
  c9_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c9_info[126].name = "eml_index_plus";
  c9_info[126].dominantType = "double";
  c9_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[126].fileTimeLo = 1286818778U;
  c9_info[126].fileTimeHi = 0U;
  c9_info[126].mFileTimeLo = 0U;
  c9_info[126].mFileTimeHi = 0U;
  c9_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c9_info[127].name = "eml_index_plus";
  c9_info[127].dominantType = "coder.internal.indexInt";
  c9_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[127].fileTimeLo = 1286818778U;
  c9_info[127].fileTimeHi = 0U;
  c9_info[127].mFileTimeLo = 0U;
  c9_info[127].mFileTimeHi = 0U;
}

static void c9_c_info_helper(c9_ResolvedFunctionInfo c9_info[147])
{
  c9_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c9_info[128].name = "eml_scalar_eg";
  c9_info[128].dominantType = "double";
  c9_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[128].fileTimeLo = 1286818796U;
  c9_info[128].fileTimeHi = 0U;
  c9_info[128].mFileTimeLo = 0U;
  c9_info[128].mFileTimeHi = 0U;
  c9_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c9_info[129].name = "eml_int_forloop_overflow_check";
  c9_info[129].dominantType = "";
  c9_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c9_info[129].fileTimeLo = 1346510340U;
  c9_info[129].fileTimeHi = 0U;
  c9_info[129].mFileTimeLo = 0U;
  c9_info[129].mFileTimeHi = 0U;
  c9_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c9_info[130].name = "eml_xtrsm";
  c9_info[130].dominantType = "char";
  c9_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c9_info[130].fileTimeLo = 1299076778U;
  c9_info[130].fileTimeHi = 0U;
  c9_info[130].mFileTimeLo = 0U;
  c9_info[130].mFileTimeHi = 0U;
  c9_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c9_info[131].name = "eml_blas_inline";
  c9_info[131].dominantType = "";
  c9_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c9_info[131].fileTimeLo = 1299076768U;
  c9_info[131].fileTimeHi = 0U;
  c9_info[131].mFileTimeLo = 0U;
  c9_info[131].mFileTimeHi = 0U;
  c9_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c9_info[132].name = "mtimes";
  c9_info[132].dominantType = "double";
  c9_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c9_info[132].fileTimeLo = 1289519692U;
  c9_info[132].fileTimeHi = 0U;
  c9_info[132].mFileTimeLo = 0U;
  c9_info[132].mFileTimeHi = 0U;
  c9_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c9_info[133].name = "eml_index_class";
  c9_info[133].dominantType = "";
  c9_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[133].fileTimeLo = 1323170578U;
  c9_info[133].fileTimeHi = 0U;
  c9_info[133].mFileTimeLo = 0U;
  c9_info[133].mFileTimeHi = 0U;
  c9_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c9_info[134].name = "eml_scalar_eg";
  c9_info[134].dominantType = "double";
  c9_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[134].fileTimeLo = 1286818796U;
  c9_info[134].fileTimeHi = 0U;
  c9_info[134].mFileTimeLo = 0U;
  c9_info[134].mFileTimeHi = 0U;
  c9_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c9_info[135].name = "eml_refblas_xtrsm";
  c9_info[135].dominantType = "char";
  c9_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[135].fileTimeLo = 1299076786U;
  c9_info[135].fileTimeHi = 0U;
  c9_info[135].mFileTimeLo = 0U;
  c9_info[135].mFileTimeHi = 0U;
  c9_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[136].name = "eml_scalar_eg";
  c9_info[136].dominantType = "double";
  c9_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c9_info[136].fileTimeLo = 1286818796U;
  c9_info[136].fileTimeHi = 0U;
  c9_info[136].mFileTimeLo = 0U;
  c9_info[136].mFileTimeHi = 0U;
  c9_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[137].name = "eml_index_minus";
  c9_info[137].dominantType = "double";
  c9_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c9_info[137].fileTimeLo = 1286818778U;
  c9_info[137].fileTimeHi = 0U;
  c9_info[137].mFileTimeLo = 0U;
  c9_info[137].mFileTimeHi = 0U;
  c9_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[138].name = "eml_index_class";
  c9_info[138].dominantType = "";
  c9_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c9_info[138].fileTimeLo = 1323170578U;
  c9_info[138].fileTimeHi = 0U;
  c9_info[138].mFileTimeLo = 0U;
  c9_info[138].mFileTimeHi = 0U;
  c9_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[139].name = "eml_int_forloop_overflow_check";
  c9_info[139].dominantType = "";
  c9_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c9_info[139].fileTimeLo = 1346510340U;
  c9_info[139].fileTimeHi = 0U;
  c9_info[139].mFileTimeLo = 0U;
  c9_info[139].mFileTimeHi = 0U;
  c9_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[140].name = "eml_index_times";
  c9_info[140].dominantType = "coder.internal.indexInt";
  c9_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c9_info[140].fileTimeLo = 1286818780U;
  c9_info[140].fileTimeHi = 0U;
  c9_info[140].mFileTimeLo = 0U;
  c9_info[140].mFileTimeHi = 0U;
  c9_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[141].name = "eml_index_plus";
  c9_info[141].dominantType = "coder.internal.indexInt";
  c9_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[141].fileTimeLo = 1286818778U;
  c9_info[141].fileTimeHi = 0U;
  c9_info[141].mFileTimeLo = 0U;
  c9_info[141].mFileTimeHi = 0U;
  c9_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[142].name = "eml_index_plus";
  c9_info[142].dominantType = "double";
  c9_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c9_info[142].fileTimeLo = 1286818778U;
  c9_info[142].fileTimeHi = 0U;
  c9_info[142].mFileTimeLo = 0U;
  c9_info[142].mFileTimeHi = 0U;
  c9_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c9_info[143].name = "intmin";
  c9_info[143].dominantType = "char";
  c9_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c9_info[143].fileTimeLo = 1311255318U;
  c9_info[143].fileTimeHi = 0U;
  c9_info[143].mFileTimeLo = 0U;
  c9_info[143].mFileTimeHi = 0U;
  c9_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c9_info[144].name = "eml_div";
  c9_info[144].dominantType = "double";
  c9_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c9_info[144].fileTimeLo = 1313347810U;
  c9_info[144].fileTimeHi = 0U;
  c9_info[144].mFileTimeLo = 0U;
  c9_info[144].mFileTimeHi = 0U;
  c9_info[145].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c9_info[145].name = "mldivide";
  c9_info[145].dominantType = "double";
  c9_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c9_info[145].fileTimeLo = 1357951548U;
  c9_info[145].fileTimeHi = 0U;
  c9_info[145].mFileTimeLo = 1319729966U;
  c9_info[145].mFileTimeHi = 0U;
  c9_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c9_info[146].name = "eml_warning";
  c9_info[146].dominantType = "char";
  c9_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c9_info[146].fileTimeLo = 1286818802U;
  c9_info[146].fileTimeHi = 0U;
  c9_info[146].mFileTimeLo = 0U;
  c9_info[146].mFileTimeHi = 0U;
}

static void c9_eml_scalar_eg(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c9_eml_error(SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
  int32_T c9_i251;
  static char_T c9_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c9_u[30];
  const mxArray *c9_y = NULL;
  int32_T c9_i252;
  static char_T c9_cv1[4] = { 'a', 's', 'i', 'n' };

  char_T c9_b_u[4];
  const mxArray *c9_b_y = NULL;
  for (c9_i251 = 0; c9_i251 < 30; c9_i251++) {
    c9_u[c9_i251] = c9_cv0[c9_i251];
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c9_i252 = 0; c9_i252 < 4; c9_i252++) {
    c9_b_u[c9_i252] = c9_cv1[c9_i252];
  }

  c9_b_y = NULL;
  sf_mex_assign(&c9_b_y, sf_mex_create("y", c9_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c9_y, 14, c9_b_y));
}

static real_T c9_atan2(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c9_y, real_T c9_x)
{
  real_T c9_b_y;
  real_T c9_b_x;
  c9_b_eml_scalar_eg(chartInstance);
  c9_b_y = c9_y;
  c9_b_x = c9_x;
  return muDoubleScalarAtan2(c9_b_y, c9_b_x);
}

static void c9_b_eml_scalar_eg(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c9_c_eml_scalar_eg(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c9_realmin(SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c9_eps(SFc9_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c9_eml_matlab_zgetrf(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], real_T c9_b_A[16], int32_T c9_ipiv[4],
  int32_T *c9_info)
{
  int32_T c9_i253;
  for (c9_i253 = 0; c9_i253 < 16; c9_i253++) {
    c9_b_A[c9_i253] = c9_A[c9_i253];
  }

  c9_b_eml_matlab_zgetrf(chartInstance, c9_b_A, c9_ipiv, c9_info);
}

static void c9_check_forloop_overflow_error
  (SFc9_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T c9_overflow)
{
  int32_T c9_i254;
  static char_T c9_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c9_u[34];
  const mxArray *c9_y = NULL;
  int32_T c9_i255;
  static char_T c9_cv3[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c9_b_u[23];
  const mxArray *c9_b_y = NULL;
  if (!c9_overflow) {
  } else {
    for (c9_i254 = 0; c9_i254 < 34; c9_i254++) {
      c9_u[c9_i254] = c9_cv2[c9_i254];
    }

    c9_y = NULL;
    sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c9_i255 = 0; c9_i255 < 23; c9_i255++) {
      c9_b_u[c9_i255] = c9_cv3[c9_i255];
    }

    c9_b_y = NULL;
    sf_mex_assign(&c9_b_y, sf_mex_create("y", c9_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c9_y, 14, c9_b_y));
  }
}

static void c9_eml_xger(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c9_m, int32_T c9_n, real_T c9_alpha1, int32_T c9_ix0, int32_T c9_iy0,
  real_T c9_A[16], int32_T c9_ia0, real_T c9_b_A[16])
{
  int32_T c9_i256;
  for (c9_i256 = 0; c9_i256 < 16; c9_i256++) {
    c9_b_A[c9_i256] = c9_A[c9_i256];
  }

  c9_b_eml_xger(chartInstance, c9_m, c9_n, c9_alpha1, c9_ix0, c9_iy0, c9_b_A,
                c9_ia0);
}

static void c9_eml_xtrsm(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c9_A[16], real_T c9_B[16], real_T c9_b_B[16])
{
  int32_T c9_i257;
  int32_T c9_i258;
  real_T c9_b_A[16];
  for (c9_i257 = 0; c9_i257 < 16; c9_i257++) {
    c9_b_B[c9_i257] = c9_B[c9_i257];
  }

  for (c9_i258 = 0; c9_i258 < 16; c9_i258++) {
    c9_b_A[c9_i258] = c9_A[c9_i258];
  }

  c9_c_eml_xtrsm(chartInstance, c9_b_A, c9_b_B);
}

static void c9_below_threshold(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c9_d_eml_scalar_eg(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c9_b_eml_xtrsm(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], real_T c9_B[16], real_T c9_b_B[16])
{
  int32_T c9_i259;
  int32_T c9_i260;
  real_T c9_b_A[16];
  for (c9_i259 = 0; c9_i259 < 16; c9_i259++) {
    c9_b_B[c9_i259] = c9_B[c9_i259];
  }

  for (c9_i260 = 0; c9_i260 < 16; c9_i260++) {
    c9_b_A[c9_i260] = c9_A[c9_i260];
  }

  c9_d_eml_xtrsm(chartInstance, c9_b_A, c9_b_B);
}

static void c9_eml_warning(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c9_i261;
  static char_T c9_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c9_u[27];
  const mxArray *c9_y = NULL;
  for (c9_i261 = 0; c9_i261 < 27; c9_i261++) {
    c9_u[c9_i261] = c9_varargin_1[c9_i261];
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c9_y));
}

static const mxArray *c9_g_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_u = *(int32_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static int32_T c9_l_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  int32_T c9_y;
  int32_T c9_i262;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_i262, 1, 6, 0U, 0, 0U, 0);
  c9_y = c9_i262;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_sfEvent;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  int32_T c9_y;
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c9_b_sfEvent = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_sfEvent),
    &c9_thisId);
  sf_mex_destroy(&c9_b_sfEvent);
  *(int32_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static uint8_T c9_m_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_torqueBalancing2012b, const
  char_T *c9_identifier)
{
  uint8_T c9_y;
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c9_b_is_active_c9_torqueBalancing2012b), &c9_thisId);
  sf_mex_destroy(&c9_b_is_active_c9_torqueBalancing2012b);
  return c9_y;
}

static uint8_T c9_n_emlrt_marshallIn(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  uint8_T c9_y;
  uint8_T c9_u0;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_u0, 1, 3, 0U, 0, 0U, 0);
  c9_y = c9_u0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_b_eml_matlab_zgetrf(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], int32_T c9_ipiv[4], int32_T *c9_info)
{
  int32_T c9_i263;
  int32_T c9_j;
  int32_T c9_b_j;
  int32_T c9_a;
  int32_T c9_jm1;
  int32_T c9_b;
  int32_T c9_mmj;
  int32_T c9_b_a;
  int32_T c9_c;
  int32_T c9_b_b;
  int32_T c9_jj;
  int32_T c9_c_a;
  int32_T c9_jp1j;
  int32_T c9_d_a;
  int32_T c9_b_c;
  int32_T c9_n;
  int32_T c9_ix0;
  int32_T c9_b_n;
  int32_T c9_b_ix0;
  int32_T c9_c_n;
  int32_T c9_c_ix0;
  int32_T c9_idxmax;
  int32_T c9_ix;
  real_T c9_x;
  real_T c9_b_x;
  real_T c9_c_x;
  real_T c9_y;
  real_T c9_d_x;
  real_T c9_e_x;
  real_T c9_b_y;
  real_T c9_smax;
  int32_T c9_d_n;
  int32_T c9_c_b;
  int32_T c9_d_b;
  boolean_T c9_overflow;
  int32_T c9_k;
  int32_T c9_b_k;
  int32_T c9_e_a;
  real_T c9_f_x;
  real_T c9_g_x;
  real_T c9_h_x;
  real_T c9_c_y;
  real_T c9_i_x;
  real_T c9_j_x;
  real_T c9_d_y;
  real_T c9_s;
  int32_T c9_f_a;
  int32_T c9_jpiv_offset;
  int32_T c9_g_a;
  int32_T c9_e_b;
  int32_T c9_jpiv;
  int32_T c9_h_a;
  int32_T c9_f_b;
  int32_T c9_c_c;
  int32_T c9_g_b;
  int32_T c9_jrow;
  int32_T c9_i_a;
  int32_T c9_h_b;
  int32_T c9_jprow;
  int32_T c9_d_ix0;
  int32_T c9_iy0;
  int32_T c9_e_ix0;
  int32_T c9_b_iy0;
  int32_T c9_f_ix0;
  int32_T c9_c_iy0;
  int32_T c9_b_ix;
  int32_T c9_iy;
  int32_T c9_c_k;
  real_T c9_temp;
  int32_T c9_j_a;
  int32_T c9_k_a;
  int32_T c9_b_jp1j;
  int32_T c9_l_a;
  int32_T c9_d_c;
  int32_T c9_m_a;
  int32_T c9_i_b;
  int32_T c9_i264;
  int32_T c9_n_a;
  int32_T c9_j_b;
  int32_T c9_o_a;
  int32_T c9_k_b;
  boolean_T c9_b_overflow;
  int32_T c9_i;
  int32_T c9_b_i;
  real_T c9_k_x;
  real_T c9_e_y;
  real_T c9_z;
  int32_T c9_l_b;
  int32_T c9_e_c;
  int32_T c9_p_a;
  int32_T c9_f_c;
  int32_T c9_q_a;
  int32_T c9_g_c;
  int32_T c9_m;
  int32_T c9_e_n;
  int32_T c9_g_ix0;
  int32_T c9_d_iy0;
  int32_T c9_ia0;
  real_T c9_d1;
  c9_realmin(chartInstance);
  c9_eps(chartInstance);
  for (c9_i263 = 0; c9_i263 < 4; c9_i263++) {
    c9_ipiv[c9_i263] = 1 + c9_i263;
  }

  *c9_info = 0;
  for (c9_j = 1; c9_j < 4; c9_j++) {
    c9_b_j = c9_j;
    c9_a = c9_b_j - 1;
    c9_jm1 = c9_a;
    c9_b = c9_b_j;
    c9_mmj = 4 - c9_b;
    c9_b_a = c9_jm1;
    c9_c = c9_b_a * 5;
    c9_b_b = c9_c + 1;
    c9_jj = c9_b_b;
    c9_c_a = c9_jj + 1;
    c9_jp1j = c9_c_a;
    c9_d_a = c9_mmj;
    c9_b_c = c9_d_a;
    c9_n = c9_b_c + 1;
    c9_ix0 = c9_jj;
    c9_b_n = c9_n;
    c9_b_ix0 = c9_ix0;
    c9_c_n = c9_b_n;
    c9_c_ix0 = c9_b_ix0;
    if (c9_c_n < 1) {
      c9_idxmax = 0;
    } else {
      c9_idxmax = 1;
      if (c9_c_n > 1) {
        c9_ix = c9_c_ix0;
        c9_x = c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c9_ix), 1, 16, 1, 0) - 1];
        c9_b_x = c9_x;
        c9_c_x = c9_b_x;
        c9_y = muDoubleScalarAbs(c9_c_x);
        c9_d_x = 0.0;
        c9_e_x = c9_d_x;
        c9_b_y = muDoubleScalarAbs(c9_e_x);
        c9_smax = c9_y + c9_b_y;
        c9_d_n = c9_c_n;
        c9_c_b = c9_d_n;
        c9_d_b = c9_c_b;
        if (2 > c9_d_b) {
          c9_overflow = FALSE;
        } else {
          c9_overflow = (c9_d_b > 2147483646);
        }

        if (c9_overflow) {
          c9_check_forloop_overflow_error(chartInstance, c9_overflow);
        }

        for (c9_k = 2; c9_k <= c9_d_n; c9_k++) {
          c9_b_k = c9_k;
          c9_e_a = c9_ix + 1;
          c9_ix = c9_e_a;
          c9_f_x = c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c9_ix), 1, 16, 1, 0) - 1];
          c9_g_x = c9_f_x;
          c9_h_x = c9_g_x;
          c9_c_y = muDoubleScalarAbs(c9_h_x);
          c9_i_x = 0.0;
          c9_j_x = c9_i_x;
          c9_d_y = muDoubleScalarAbs(c9_j_x);
          c9_s = c9_c_y + c9_d_y;
          if (c9_s > c9_smax) {
            c9_idxmax = c9_b_k;
            c9_smax = c9_s;
          }
        }
      }
    }

    c9_f_a = c9_idxmax - 1;
    c9_jpiv_offset = c9_f_a;
    c9_g_a = c9_jj;
    c9_e_b = c9_jpiv_offset;
    c9_jpiv = c9_g_a + c9_e_b;
    if (c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c9_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if (c9_jpiv_offset != 0) {
        c9_h_a = c9_b_j;
        c9_f_b = c9_jpiv_offset;
        c9_c_c = c9_h_a + c9_f_b;
        c9_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c9_b_j), 1, 4, 1, 0) - 1] = c9_c_c;
        c9_g_b = c9_jm1 + 1;
        c9_jrow = c9_g_b;
        c9_i_a = c9_jrow;
        c9_h_b = c9_jpiv_offset;
        c9_jprow = c9_i_a + c9_h_b;
        c9_d_ix0 = c9_jrow;
        c9_iy0 = c9_jprow;
        c9_e_ix0 = c9_d_ix0;
        c9_b_iy0 = c9_iy0;
        c9_f_ix0 = c9_e_ix0;
        c9_c_iy0 = c9_b_iy0;
        c9_b_ix = c9_f_ix0;
        c9_iy = c9_c_iy0;
        for (c9_c_k = 1; c9_c_k < 5; c9_c_k++) {
          c9_temp = c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c9_b_ix), 1, 16, 1, 0) - 1];
          c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_b_ix), 1, 16, 1, 0) - 1] =
            c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_iy), 1, 16, 1, 0) - 1];
          c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_iy), 1, 16, 1, 0) - 1] = c9_temp;
          c9_j_a = c9_b_ix + 4;
          c9_b_ix = c9_j_a;
          c9_k_a = c9_iy + 4;
          c9_iy = c9_k_a;
        }
      }

      c9_b_jp1j = c9_jp1j;
      c9_l_a = c9_mmj;
      c9_d_c = c9_l_a;
      c9_m_a = c9_jp1j;
      c9_i_b = c9_d_c - 1;
      c9_i264 = c9_m_a + c9_i_b;
      c9_n_a = c9_b_jp1j;
      c9_j_b = c9_i264;
      c9_o_a = c9_n_a;
      c9_k_b = c9_j_b;
      if (c9_o_a > c9_k_b) {
        c9_b_overflow = FALSE;
      } else {
        c9_b_overflow = (c9_k_b > 2147483646);
      }

      if (c9_b_overflow) {
        c9_check_forloop_overflow_error(chartInstance, c9_b_overflow);
      }

      for (c9_i = c9_b_jp1j; c9_i <= c9_i264; c9_i++) {
        c9_b_i = c9_i;
        c9_k_x = c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_b_i), 1, 16, 1, 0) - 1];
        c9_e_y = c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c9_jj), 1, 16, 1, 0) - 1];
        c9_z = c9_k_x / c9_e_y;
        c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c9_b_i), 1, 16, 1, 0) - 1] = c9_z;
      }
    } else {
      *c9_info = c9_b_j;
    }

    c9_l_b = c9_b_j;
    c9_e_c = 4 - c9_l_b;
    c9_p_a = c9_jj;
    c9_f_c = c9_p_a;
    c9_q_a = c9_jj;
    c9_g_c = c9_q_a;
    c9_m = c9_mmj;
    c9_e_n = c9_e_c;
    c9_g_ix0 = c9_jp1j;
    c9_d_iy0 = c9_f_c + 4;
    c9_ia0 = c9_g_c + 5;
    c9_d1 = -1.0;
    c9_b_eml_xger(chartInstance, c9_m, c9_e_n, c9_d1, c9_g_ix0, c9_d_iy0, c9_A,
                  c9_ia0);
  }

  if (*c9_info == 0) {
    if (!(c9_A[15] != 0.0)) {
      *c9_info = 4;
    }
  }
}

static void c9_b_eml_xger(SFc9_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c9_m, int32_T c9_n, real_T c9_alpha1, int32_T c9_ix0, int32_T c9_iy0,
  real_T c9_A[16], int32_T c9_ia0)
{
  int32_T c9_b_m;
  int32_T c9_b_n;
  real_T c9_b_alpha1;
  int32_T c9_b_ix0;
  int32_T c9_b_iy0;
  int32_T c9_b_ia0;
  int32_T c9_c_m;
  int32_T c9_c_n;
  real_T c9_c_alpha1;
  int32_T c9_c_ix0;
  int32_T c9_c_iy0;
  int32_T c9_c_ia0;
  int32_T c9_d_m;
  int32_T c9_d_n;
  real_T c9_d_alpha1;
  int32_T c9_d_ix0;
  int32_T c9_d_iy0;
  int32_T c9_d_ia0;
  int32_T c9_ixstart;
  int32_T c9_a;
  int32_T c9_jA;
  int32_T c9_jy;
  int32_T c9_e_n;
  int32_T c9_b;
  int32_T c9_b_b;
  boolean_T c9_overflow;
  int32_T c9_j;
  real_T c9_yjy;
  real_T c9_temp;
  int32_T c9_ix;
  int32_T c9_c_b;
  int32_T c9_i265;
  int32_T c9_b_a;
  int32_T c9_d_b;
  int32_T c9_i266;
  int32_T c9_c_a;
  int32_T c9_e_b;
  int32_T c9_d_a;
  int32_T c9_f_b;
  boolean_T c9_b_overflow;
  int32_T c9_ijA;
  int32_T c9_b_ijA;
  int32_T c9_e_a;
  int32_T c9_f_a;
  int32_T c9_g_a;
  c9_b_m = c9_m;
  c9_b_n = c9_n;
  c9_b_alpha1 = c9_alpha1;
  c9_b_ix0 = c9_ix0;
  c9_b_iy0 = c9_iy0;
  c9_b_ia0 = c9_ia0;
  c9_c_m = c9_b_m;
  c9_c_n = c9_b_n;
  c9_c_alpha1 = c9_b_alpha1;
  c9_c_ix0 = c9_b_ix0;
  c9_c_iy0 = c9_b_iy0;
  c9_c_ia0 = c9_b_ia0;
  c9_d_m = c9_c_m;
  c9_d_n = c9_c_n;
  c9_d_alpha1 = c9_c_alpha1;
  c9_d_ix0 = c9_c_ix0;
  c9_d_iy0 = c9_c_iy0;
  c9_d_ia0 = c9_c_ia0;
  if (c9_d_alpha1 == 0.0) {
  } else {
    c9_ixstart = c9_d_ix0;
    c9_a = c9_d_ia0 - 1;
    c9_jA = c9_a;
    c9_jy = c9_d_iy0;
    c9_e_n = c9_d_n;
    c9_b = c9_e_n;
    c9_b_b = c9_b;
    if (1 > c9_b_b) {
      c9_overflow = FALSE;
    } else {
      c9_overflow = (c9_b_b > 2147483646);
    }

    if (c9_overflow) {
      c9_check_forloop_overflow_error(chartInstance, c9_overflow);
    }

    for (c9_j = 1; c9_j <= c9_e_n; c9_j++) {
      c9_yjy = c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c9_jy), 1, 16, 1, 0) - 1];
      if (c9_yjy != 0.0) {
        c9_temp = c9_yjy * c9_d_alpha1;
        c9_ix = c9_ixstart;
        c9_c_b = c9_jA + 1;
        c9_i265 = c9_c_b;
        c9_b_a = c9_d_m;
        c9_d_b = c9_jA;
        c9_i266 = c9_b_a + c9_d_b;
        c9_c_a = c9_i265;
        c9_e_b = c9_i266;
        c9_d_a = c9_c_a;
        c9_f_b = c9_e_b;
        if (c9_d_a > c9_f_b) {
          c9_b_overflow = FALSE;
        } else {
          c9_b_overflow = (c9_f_b > 2147483646);
        }

        if (c9_b_overflow) {
          c9_check_forloop_overflow_error(chartInstance, c9_b_overflow);
        }

        for (c9_ijA = c9_i265; c9_ijA <= c9_i266; c9_ijA++) {
          c9_b_ijA = c9_ijA;
          c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_b_ijA), 1, 16, 1, 0) - 1] =
            c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_b_ijA), 1, 16, 1, 0) - 1] +
            c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_ix), 1, 16, 1, 0) - 1] * c9_temp;
          c9_e_a = c9_ix + 1;
          c9_ix = c9_e_a;
        }
      }

      c9_f_a = c9_jy + 4;
      c9_jy = c9_f_a;
      c9_g_a = c9_jA + 4;
      c9_jA = c9_g_a;
    }
  }
}

static void c9_c_eml_xtrsm(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], real_T c9_B[16])
{
  int32_T c9_j;
  int32_T c9_b_j;
  int32_T c9_a;
  int32_T c9_c;
  int32_T c9_b;
  int32_T c9_b_c;
  int32_T c9_b_b;
  int32_T c9_jBcol;
  int32_T c9_k;
  int32_T c9_b_k;
  int32_T c9_b_a;
  int32_T c9_c_c;
  int32_T c9_c_b;
  int32_T c9_d_c;
  int32_T c9_d_b;
  int32_T c9_kAcol;
  int32_T c9_c_a;
  int32_T c9_e_b;
  int32_T c9_e_c;
  int32_T c9_d_a;
  int32_T c9_i267;
  boolean_T c9_overflow;
  int32_T c9_i;
  int32_T c9_b_i;
  int32_T c9_e_a;
  int32_T c9_f_b;
  int32_T c9_f_c;
  int32_T c9_f_a;
  int32_T c9_g_b;
  int32_T c9_g_c;
  int32_T c9_g_a;
  int32_T c9_h_b;
  int32_T c9_h_c;
  int32_T c9_h_a;
  int32_T c9_i_b;
  int32_T c9_i_c;
  c9_below_threshold(chartInstance);
  c9_d_eml_scalar_eg(chartInstance);
  for (c9_j = 1; c9_j < 5; c9_j++) {
    c9_b_j = c9_j;
    c9_a = c9_b_j;
    c9_c = c9_a;
    c9_b = c9_c - 1;
    c9_b_c = c9_b << 2;
    c9_b_b = c9_b_c;
    c9_jBcol = c9_b_b;
    for (c9_k = 1; c9_k < 5; c9_k++) {
      c9_b_k = c9_k;
      c9_b_a = c9_b_k;
      c9_c_c = c9_b_a;
      c9_c_b = c9_c_c - 1;
      c9_d_c = c9_c_b << 2;
      c9_d_b = c9_d_c;
      c9_kAcol = c9_d_b;
      c9_c_a = c9_b_k;
      c9_e_b = c9_jBcol;
      c9_e_c = c9_c_a + c9_e_b;
      if (c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c9_d_a = c9_b_k;
        c9_i267 = c9_d_a;
        c9_overflow = FALSE;
        if (c9_overflow) {
          c9_check_forloop_overflow_error(chartInstance, c9_overflow);
        }

        for (c9_i = c9_i267 + 1; c9_i < 5; c9_i++) {
          c9_b_i = c9_i;
          c9_e_a = c9_b_i;
          c9_f_b = c9_jBcol;
          c9_f_c = c9_e_a + c9_f_b;
          c9_f_a = c9_b_i;
          c9_g_b = c9_jBcol;
          c9_g_c = c9_f_a + c9_g_b;
          c9_g_a = c9_b_k;
          c9_h_b = c9_jBcol;
          c9_h_c = c9_g_a + c9_h_b;
          c9_h_a = c9_b_i;
          c9_i_b = c9_kAcol;
          c9_i_c = c9_h_a + c9_i_b;
          c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_f_c), 1, 16, 1, 0) - 1] =
            c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_g_c), 1, 16, 1, 0) - 1] -
            c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_h_c), 1, 16, 1, 0) - 1] *
            c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_i_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void c9_d_eml_xtrsm(SFc9_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c9_A[16], real_T c9_B[16])
{
  int32_T c9_j;
  int32_T c9_b_j;
  int32_T c9_a;
  int32_T c9_c;
  int32_T c9_b;
  int32_T c9_b_c;
  int32_T c9_b_b;
  int32_T c9_jBcol;
  int32_T c9_k;
  int32_T c9_b_k;
  int32_T c9_b_a;
  int32_T c9_c_c;
  int32_T c9_c_b;
  int32_T c9_d_c;
  int32_T c9_d_b;
  int32_T c9_kAcol;
  int32_T c9_c_a;
  int32_T c9_e_b;
  int32_T c9_e_c;
  int32_T c9_d_a;
  int32_T c9_f_b;
  int32_T c9_f_c;
  int32_T c9_e_a;
  int32_T c9_g_b;
  int32_T c9_g_c;
  int32_T c9_f_a;
  int32_T c9_h_b;
  int32_T c9_h_c;
  real_T c9_x;
  real_T c9_y;
  real_T c9_z;
  int32_T c9_g_a;
  int32_T c9_i268;
  int32_T c9_i_b;
  int32_T c9_j_b;
  boolean_T c9_overflow;
  int32_T c9_i;
  int32_T c9_b_i;
  int32_T c9_h_a;
  int32_T c9_k_b;
  int32_T c9_i_c;
  int32_T c9_i_a;
  int32_T c9_l_b;
  int32_T c9_j_c;
  int32_T c9_j_a;
  int32_T c9_m_b;
  int32_T c9_k_c;
  int32_T c9_k_a;
  int32_T c9_n_b;
  int32_T c9_l_c;
  c9_below_threshold(chartInstance);
  c9_d_eml_scalar_eg(chartInstance);
  for (c9_j = 1; c9_j < 5; c9_j++) {
    c9_b_j = c9_j;
    c9_a = c9_b_j;
    c9_c = c9_a;
    c9_b = c9_c - 1;
    c9_b_c = c9_b << 2;
    c9_b_b = c9_b_c;
    c9_jBcol = c9_b_b;
    for (c9_k = 4; c9_k > 0; c9_k--) {
      c9_b_k = c9_k;
      c9_b_a = c9_b_k;
      c9_c_c = c9_b_a;
      c9_c_b = c9_c_c - 1;
      c9_d_c = c9_c_b << 2;
      c9_d_b = c9_d_c;
      c9_kAcol = c9_d_b;
      c9_c_a = c9_b_k;
      c9_e_b = c9_jBcol;
      c9_e_c = c9_c_a + c9_e_b;
      if (c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c9_d_a = c9_b_k;
        c9_f_b = c9_jBcol;
        c9_f_c = c9_d_a + c9_f_b;
        c9_e_a = c9_b_k;
        c9_g_b = c9_jBcol;
        c9_g_c = c9_e_a + c9_g_b;
        c9_f_a = c9_b_k;
        c9_h_b = c9_kAcol;
        c9_h_c = c9_f_a + c9_h_b;
        c9_x = c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c9_g_c), 1, 16, 1, 0) - 1];
        c9_y = c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c9_h_c), 1, 16, 1, 0) - 1];
        c9_z = c9_x / c9_y;
        c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c9_f_c), 1, 16, 1, 0) - 1] = c9_z;
        c9_g_a = c9_b_k - 1;
        c9_i268 = c9_g_a;
        c9_i_b = c9_i268;
        c9_j_b = c9_i_b;
        if (1 > c9_j_b) {
          c9_overflow = FALSE;
        } else {
          c9_overflow = (c9_j_b > 2147483646);
        }

        if (c9_overflow) {
          c9_check_forloop_overflow_error(chartInstance, c9_overflow);
        }

        for (c9_i = 1; c9_i <= c9_i268; c9_i++) {
          c9_b_i = c9_i;
          c9_h_a = c9_b_i;
          c9_k_b = c9_jBcol;
          c9_i_c = c9_h_a + c9_k_b;
          c9_i_a = c9_b_i;
          c9_l_b = c9_jBcol;
          c9_j_c = c9_i_a + c9_l_b;
          c9_j_a = c9_b_k;
          c9_m_b = c9_jBcol;
          c9_k_c = c9_j_a + c9_m_b;
          c9_k_a = c9_b_i;
          c9_n_b = c9_kAcol;
          c9_l_c = c9_k_a + c9_n_b;
          c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_i_c), 1, 16, 1, 0) - 1] =
            c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_j_c), 1, 16, 1, 0) - 1] -
            c9_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_k_c), 1, 16, 1, 0) - 1] *
            c9_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c9_l_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void init_dsm_address_info(SFc9_torqueBalancing2012bInstanceStruct
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

void sf_c9_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(44106875U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(841437657U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2368952469U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3882389986U);
}

mxArray *sf_c9_torqueBalancing2012b_get_autoinheritance_info(void)
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

mxArray *sf_c9_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c9_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[71],T\"w_H_root\",},{M[8],M[0],T\"is_active_c9_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c9_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           9,
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
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"fromBaseToWorldWithImu",0,-1,1610);
        _SFD_CV_INIT_SCRIPT_IF(0,0,860,884,-1,947);
        _SFD_CV_INIT_SCRIPT_IF(0,1,948,974,-1,1037);
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
            1.0,0,0,(MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)
            c9_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)c9_b_sf_marshallIn);

        {
          real_T (*c9_imu_H_link)[16];
          real_T (*c9_imu_H_link_0)[16];
          real_T (*c9_link_H_root)[16];
          real_T (*c9_inertial_0)[12];
          real_T (*c9_inertial)[12];
          real_T (*c9_neck)[3];
          real_T (*c9_w_H_root)[16];
          c9_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S,
            1);
          c9_neck = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
          c9_inertial = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 4);
          c9_inertial_0 = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S,
            3);
          c9_link_H_root = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            2);
          c9_imu_H_link_0 = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 1);
          c9_imu_H_link = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c9_imu_H_link);
          _SFD_SET_DATA_VALUE_PTR(1U, *c9_imu_H_link_0);
          _SFD_SET_DATA_VALUE_PTR(2U, *c9_link_H_root);
          _SFD_SET_DATA_VALUE_PTR(3U, *c9_inertial_0);
          _SFD_SET_DATA_VALUE_PTR(4U, *c9_inertial);
          _SFD_SET_DATA_VALUE_PTR(5U, *c9_neck);
          _SFD_SET_DATA_VALUE_PTR(6U, *c9_w_H_root);
          _SFD_SET_DATA_VALUE_PTR(7U, &chartInstance->c9_CONFIG);
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

static void sf_opaque_initialize_c9_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc9_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c9_torqueBalancing2012b
    ((SFc9_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c9_torqueBalancing2012b((SFc9_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c9_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c9_torqueBalancing2012b((SFc9_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c9_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c9_torqueBalancing2012b((SFc9_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c9_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c9_torqueBalancing2012b((SFc9_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c9_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c9_torqueBalancing2012b
    ((SFc9_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c9_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c9_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c9_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c9_torqueBalancing2012b((SFc9_torqueBalancing2012bInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c9_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c9_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c9_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c9_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c9_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc9_torqueBalancing2012bInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c9_torqueBalancing2012b((SFc9_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc9_torqueBalancing2012b((SFc9_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c9_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c9_torqueBalancing2012b
      ((SFc9_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c9_torqueBalancing2012b(SimStruct *S)
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
      9);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,9,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,9,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,9);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,9,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,9,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,9);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2274714917U));
  ssSetChecksum1(S,(1662336581U));
  ssSetChecksum2(S,(937218894U));
  ssSetChecksum3(S,(3161659269U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c9_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c9_torqueBalancing2012b(SimStruct *S)
{
  SFc9_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc9_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc9_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc9_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c9_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c9_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c9_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c9_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c9_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c9_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c9_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c9_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c9_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c9_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c9_torqueBalancing2012b;
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

void c9_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void *
  data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c9_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c9_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c9_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c9_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
