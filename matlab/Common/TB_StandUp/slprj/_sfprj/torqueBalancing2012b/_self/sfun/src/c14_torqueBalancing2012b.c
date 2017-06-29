/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c14_torqueBalancing2012b.h"
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
static const char * c14_debug_family_names[10] = { "nargin", "nargout",
  "imu_H_link", "imu_H_link_0", "link_H_root", "inertial_0", "inertial", "neck",
  "CONFIG", "w_H_root" };

static const char * c14_b_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c14_c_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c14_d_debug_family_names[4] = { "nargin", "nargout", "alpha",
  "R" };

static const char * c14_e_debug_family_names[4] = { "nargin", "nargout", "R",
  "rollPitchYaw" };

static const char * c14_f_debug_family_names[7] = { "nargin", "nargout", "a",
  "d", "alph", "thet", "G" };

static const char * c14_g_debug_family_names[10] = { "G_34", "G_45", "G_56",
  "G_6I", "imuAssumingNeckToZero_H_neckBase", "imu_H_neckBase", "nargin",
  "nargout", "neckJoints", "imu_H_imuAssumingNeckToZero" };

static const char * c14_h_debug_family_names[23] = { "wImu_R_imu",
  "wImu_R_imu_0", "imu_R_link", "imu_R_link_0", "wImu_R_link_0",
  "rollPitchYaw_link_0", "rollPitchYaw_link", "rollPitchYawFiltered_link",
  "wImu_H_link", "wImu_H_link_0", "wImu_H_root", "wImu_H_wImuAssumingNeckToZero",
  "wImu_R_link", "nargin", "nargout", "imu_H_link", "imu_H_link_0",
  "link_H_root", "inertial_0", "inertial", "neck", "CONFIG", "w_H_root" };

/* Function Declarations */
static void initialize_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void c14_update_debugger_state_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c14_st);
static void finalize_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c14_torqueBalancing2012b(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void c14_fromBaseToWorldWithImu(SFc14_torqueBalancing2012bInstanceStruct *
  chartInstance, real_T c14_imu_H_link[16], real_T c14_imu_H_link_0[16], real_T
  c14_link_H_root[16], real_T c14_inertial_0[12], real_T c14_inertial[12],
  real_T c14_neck[3], c14_struct_6drkWFoHJIWwAUE1rJmWJH *c14_b_CONFIG, real_T
  c14_w_H_root[16]);
static void c14_rotz(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c14_alpha, real_T c14_R[9]);
static void c14_roty(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c14_alpha, real_T c14_R[9]);
static void c14_rotx(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c14_alpha, real_T c14_R[9]);
static void c14_rollPitchYawFromRotation
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance, real_T c14_R[9],
   real_T c14_rollPitchYaw[3]);
static void c14_correctIMU(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_neckJoints[3], real_T
  c14_imu_H_imuAssumingNeckToZero[16]);
static void c14_evalDHMatrix(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_a, real_T c14_d, real_T c14_alph, real_T c14_thet,
  real_T c14_G[16]);
static void init_script_number_translation(uint32_T c14_machineNumber, uint32_T
  c14_chartNumber);
static const mxArray *c14_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_w_H_root, const char_T *c14_identifier,
  real_T c14_y[16]);
static void c14_b_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[16]);
static void c14_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static const mxArray *c14_b_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_c_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  c14_struct_6drkWFoHJIWwAUE1rJmWJH *c14_y);
static real_T c14_d_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static c14_struct_amzdx4J7qaaMzgPI7fQ3WD c14_e_emlrt_marshallIn
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c14_u,
   const emlrtMsgIdentifier *c14_parentId);
static boolean_T c14_f_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_g_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[2]);
static void c14_h_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[23]);
static void c14_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static const mxArray *c14_c_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static const mxArray *c14_d_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static const mxArray *c14_e_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static const mxArray *c14_f_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_i_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[9]);
static void c14_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static void c14_j_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[3]);
static void c14_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static void c14_k_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[12]);
static void c14_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static void c14_info_helper(c14_ResolvedFunctionInfo c14_info[147]);
static void c14_b_info_helper(c14_ResolvedFunctionInfo c14_info[147]);
static void c14_c_info_helper(c14_ResolvedFunctionInfo c14_info[147]);
static void c14_eml_scalar_eg(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c14_eml_error(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);
static real_T c14_atan2(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c14_y, real_T c14_x);
static void c14_b_eml_scalar_eg(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c14_c_eml_scalar_eg(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c14_realmin(SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void c14_eps(SFc14_torqueBalancing2012bInstanceStruct *chartInstance);
static void c14_eml_matlab_zgetrf(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_b_A[16], int32_T c14_ipiv[4],
  int32_T *c14_info);
static void c14_check_forloop_overflow_error
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c14_overflow);
static void c14_eml_xger(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c14_m, int32_T c14_n, real_T c14_alpha1, int32_T c14_ix0, int32_T
  c14_iy0, real_T c14_A[16], int32_T c14_ia0, real_T c14_b_A[16]);
static void c14_eml_xtrsm(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_B[16], real_T c14_b_B[16]);
static void c14_below_threshold(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c14_d_eml_scalar_eg(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void c14_b_eml_xtrsm(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_B[16], real_T c14_b_B[16]);
static void c14_eml_warning(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);
static const mxArray *c14_g_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static int32_T c14_l_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static uint8_T c14_m_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_b_is_active_c14_torqueBalancing2012b, const
  char_T *c14_identifier);
static uint8_T c14_n_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_b_eml_matlab_zgetrf(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], int32_T c14_ipiv[4], int32_T *c14_info);
static void c14_b_eml_xger(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c14_m, int32_T c14_n, real_T c14_alpha1, int32_T
  c14_ix0, int32_T c14_iy0, real_T c14_A[16], int32_T c14_ia0);
static void c14_c_eml_xtrsm(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_B[16]);
static void c14_d_eml_xtrsm(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_B[16]);
static void init_dsm_address_info(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c14_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c14_is_active_c14_torqueBalancing2012b = 0U;
}

static void initialize_params_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c14_m0 = NULL;
  const mxArray *c14_mxField;
  c14_struct_6drkWFoHJIWwAUE1rJmWJH c14_r0;
  const mxArray *c14_m1 = NULL;
  const mxArray *c14_b_mxField;
  sf_set_error_prefix_string(
    "Error evaluating data 'CONFIG' in the parent workspace.\n");
  c14_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c14_mxField = sf_mex_getfield(c14_m0, "SIMULATION_TIME", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.SIMULATION_TIME,
                      1, 0, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "SCOPES", "CONFIG", 0);
  c14_m1 = sf_mex_dup(c14_mxField);
  c14_b_mxField = sf_mex_getfield(c14_m1, "ALL", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_b_mxField), &c14_r0.SCOPES.ALL, 1,
                      11, 0U, 0, 0U, 0);
  c14_b_mxField = sf_mex_getfield(c14_m1, "BASE_EST_IMU", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_b_mxField),
                      &c14_r0.SCOPES.BASE_EST_IMU, 1, 11, 0U, 0, 0U, 0);
  c14_b_mxField = sf_mex_getfield(c14_m1, "EXTWRENCHES", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_b_mxField),
                      &c14_r0.SCOPES.EXTWRENCHES, 1, 11, 0U, 0, 0U, 0);
  c14_b_mxField = sf_mex_getfield(c14_m1, "GAIN_SCHE_INFO", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_b_mxField),
                      &c14_r0.SCOPES.GAIN_SCHE_INFO, 1, 11, 0U, 0, 0U, 0);
  c14_b_mxField = sf_mex_getfield(c14_m1, "MAIN", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_b_mxField), &c14_r0.SCOPES.MAIN,
                      1, 11, 0U, 0, 0U, 0);
  c14_b_mxField = sf_mex_getfield(c14_m1, "QP", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_b_mxField), &c14_r0.SCOPES.QP, 1,
                      11, 0U, 0, 0U, 0);
  sf_mex_destroy(&c14_m1);
  c14_mxField = sf_mex_getfield(c14_m0, "CHECK_LIMITS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.CHECK_LIMITS, 1,
                      11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "USE_IMU4EST_BASE", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField),
                      &c14_r0.USE_IMU4EST_BASE, 1, 11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "YAW_IMU_FILTER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.YAW_IMU_FILTER,
                      1, 11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "PITCH_IMU_FILTER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField),
                      &c14_r0.PITCH_IMU_FILTER, 1, 11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "CORRECT_NECK_IMU", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField),
                      &c14_r0.CORRECT_NECK_IMU, 1, 11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "ONSOFTCARPET", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.ONSOFTCARPET, 1,
                      11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "USE_QP_SOLVER", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.USE_QP_SOLVER,
                      1, 11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "Ts", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.Ts, 1, 0, 0U, 0,
                      0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "ON_GAZEBO", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.ON_GAZEBO, 1,
                      11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "LEFT_RIGHT_FOOT_IN_CONTACT", "CONFIG",
    0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField),
                      c14_r0.LEFT_RIGHT_FOOT_IN_CONTACT, 1, 0, 0U, 1, 0U, 2, 1,
                      2);
  c14_mxField = sf_mex_getfield(c14_m0, "SMOOTH_DES_COM", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.SMOOTH_DES_COM,
                      1, 0, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "SMOOTH_DES_Q", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.SMOOTH_DES_Q, 1,
                      0, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "smoothingTimeTranDynamics", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField),
                      &c14_r0.smoothingTimeTranDynamics, 1, 0, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "DEMO_MOVEMENTS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.DEMO_MOVEMENTS,
                      1, 11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "TIME_CONTROLLER_SWITCH", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField),
                      &c14_r0.TIME_CONTROLLER_SWITCH, 1, 0, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "JOINTS", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), c14_r0.JOINTS, 1, 0, 0U,
                      1, 0U, 2, 23, 1);
  c14_mxField = sf_mex_getfield(c14_m0, "JOINTSITING", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), c14_r0.JOINTSITING, 1,
                      0, 0U, 1, 0U, 2, 23, 1);
  c14_mxField = sf_mex_getfield(c14_m0, "FIXEDCOMBENCH", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.FIXEDCOMBENCH,
                      1, 11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "iCubStandUp", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.iCubStandUp, 1,
                      11, 0U, 0, 0U, 0);
  c14_mxField = sf_mex_getfield(c14_m0, "useExtArmForces", "CONFIG", 0);
  sf_mex_import_named("CONFIG", sf_mex_dup(c14_mxField), &c14_r0.useExtArmForces,
                      1, 11, 0U, 0, 0U, 0);
  sf_mex_destroy(&c14_m0);
  chartInstance->c14_CONFIG = c14_r0;
  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c14_update_debugger_state_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c14_st;
  const mxArray *c14_y = NULL;
  int32_T c14_i0;
  real_T c14_u[16];
  const mxArray *c14_b_y = NULL;
  uint8_T c14_hoistedGlobal;
  uint8_T c14_b_u;
  const mxArray *c14_c_y = NULL;
  real_T (*c14_w_H_root)[16];
  c14_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c14_st = NULL;
  c14_st = NULL;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_createcellarray(2), FALSE);
  for (c14_i0 = 0; c14_i0 < 16; c14_i0++) {
    c14_u[c14_i0] = (*c14_w_H_root)[c14_i0];
  }

  c14_b_y = NULL;
  sf_mex_assign(&c14_b_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 2, 4, 4),
                FALSE);
  sf_mex_setcell(c14_y, 0, c14_b_y);
  c14_hoistedGlobal = chartInstance->c14_is_active_c14_torqueBalancing2012b;
  c14_b_u = c14_hoistedGlobal;
  c14_c_y = NULL;
  sf_mex_assign(&c14_c_y, sf_mex_create("y", &c14_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c14_y, 1, c14_c_y);
  sf_mex_assign(&c14_st, c14_y, FALSE);
  return c14_st;
}

static void set_sim_state_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c14_st)
{
  const mxArray *c14_u;
  real_T c14_dv0[16];
  int32_T c14_i1;
  real_T (*c14_w_H_root)[16];
  c14_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c14_doneDoubleBufferReInit = TRUE;
  c14_u = sf_mex_dup(c14_st);
  c14_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c14_u, 0)),
                       "w_H_root", c14_dv0);
  for (c14_i1 = 0; c14_i1 < 16; c14_i1++) {
    (*c14_w_H_root)[c14_i1] = c14_dv0[c14_i1];
  }

  chartInstance->c14_is_active_c14_torqueBalancing2012b = c14_m_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c14_u, 1)),
     "is_active_c14_torqueBalancing2012b");
  sf_mex_destroy(&c14_u);
  c14_update_debugger_state_c14_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c14_st);
}

static void finalize_c14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c14_torqueBalancing2012b(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c14_i2;
  int32_T c14_i3;
  int32_T c14_i4;
  int32_T c14_i5;
  int32_T c14_i6;
  int32_T c14_i7;
  int32_T c14_i8;
  int32_T c14_i9;
  real_T c14_imu_H_link[16];
  int32_T c14_i10;
  real_T c14_imu_H_link_0[16];
  int32_T c14_i11;
  real_T c14_link_H_root[16];
  int32_T c14_i12;
  real_T c14_inertial_0[12];
  int32_T c14_i13;
  real_T c14_inertial[12];
  int32_T c14_i14;
  real_T c14_neck[3];
  c14_struct_6drkWFoHJIWwAUE1rJmWJH c14_b_CONFIG;
  uint32_T c14_debug_family_var_map[10];
  real_T c14_nargin = 7.0;
  real_T c14_nargout = 1.0;
  real_T c14_w_H_root[16];
  int32_T c14_i15;
  real_T c14_b_imu_H_link[16];
  int32_T c14_i16;
  real_T c14_b_imu_H_link_0[16];
  int32_T c14_i17;
  real_T c14_b_link_H_root[16];
  int32_T c14_i18;
  real_T c14_b_inertial_0[12];
  int32_T c14_i19;
  real_T c14_b_inertial[12];
  int32_T c14_i20;
  real_T c14_b_neck[3];
  c14_struct_6drkWFoHJIWwAUE1rJmWJH c14_c_CONFIG;
  real_T c14_dv1[16];
  int32_T c14_i21;
  int32_T c14_i22;
  real_T (*c14_b_w_H_root)[16];
  real_T (*c14_c_neck)[3];
  real_T (*c14_c_inertial)[12];
  real_T (*c14_c_inertial_0)[12];
  real_T (*c14_c_link_H_root)[16];
  real_T (*c14_c_imu_H_link_0)[16];
  real_T (*c14_c_imu_H_link)[16];
  c14_b_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c14_c_neck = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c14_c_inertial = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 4);
  c14_c_inertial_0 = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 3);
  c14_c_link_H_root = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 2);
  c14_c_imu_H_link_0 = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 1);
  c14_c_imu_H_link = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  for (c14_i2 = 0; c14_i2 < 16; c14_i2++) {
    _SFD_DATA_RANGE_CHECK((*c14_c_imu_H_link)[c14_i2], 0U);
  }

  for (c14_i3 = 0; c14_i3 < 16; c14_i3++) {
    _SFD_DATA_RANGE_CHECK((*c14_c_imu_H_link_0)[c14_i3], 1U);
  }

  for (c14_i4 = 0; c14_i4 < 16; c14_i4++) {
    _SFD_DATA_RANGE_CHECK((*c14_c_link_H_root)[c14_i4], 2U);
  }

  for (c14_i5 = 0; c14_i5 < 12; c14_i5++) {
    _SFD_DATA_RANGE_CHECK((*c14_c_inertial_0)[c14_i5], 3U);
  }

  for (c14_i6 = 0; c14_i6 < 12; c14_i6++) {
    _SFD_DATA_RANGE_CHECK((*c14_c_inertial)[c14_i6], 4U);
  }

  for (c14_i7 = 0; c14_i7 < 3; c14_i7++) {
    _SFD_DATA_RANGE_CHECK((*c14_c_neck)[c14_i7], 5U);
  }

  for (c14_i8 = 0; c14_i8 < 16; c14_i8++) {
    _SFD_DATA_RANGE_CHECK((*c14_b_w_H_root)[c14_i8], 6U);
  }

  chartInstance->c14_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  for (c14_i9 = 0; c14_i9 < 16; c14_i9++) {
    c14_imu_H_link[c14_i9] = (*c14_c_imu_H_link)[c14_i9];
  }

  for (c14_i10 = 0; c14_i10 < 16; c14_i10++) {
    c14_imu_H_link_0[c14_i10] = (*c14_c_imu_H_link_0)[c14_i10];
  }

  for (c14_i11 = 0; c14_i11 < 16; c14_i11++) {
    c14_link_H_root[c14_i11] = (*c14_c_link_H_root)[c14_i11];
  }

  for (c14_i12 = 0; c14_i12 < 12; c14_i12++) {
    c14_inertial_0[c14_i12] = (*c14_c_inertial_0)[c14_i12];
  }

  for (c14_i13 = 0; c14_i13 < 12; c14_i13++) {
    c14_inertial[c14_i13] = (*c14_c_inertial)[c14_i13];
  }

  for (c14_i14 = 0; c14_i14 < 3; c14_i14++) {
    c14_neck[c14_i14] = (*c14_c_neck)[c14_i14];
  }

  c14_b_CONFIG = chartInstance->c14_CONFIG;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c14_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 0U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 1U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_imu_H_link, 2U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_imu_H_link_0, 3U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_link_H_root, 4U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_inertial_0, 5U, c14_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_inertial, 6U, c14_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_neck, 7U, c14_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_CONFIG, 8U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_w_H_root, 9U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 3);
  for (c14_i15 = 0; c14_i15 < 16; c14_i15++) {
    c14_b_imu_H_link[c14_i15] = c14_imu_H_link[c14_i15];
  }

  for (c14_i16 = 0; c14_i16 < 16; c14_i16++) {
    c14_b_imu_H_link_0[c14_i16] = c14_imu_H_link_0[c14_i16];
  }

  for (c14_i17 = 0; c14_i17 < 16; c14_i17++) {
    c14_b_link_H_root[c14_i17] = c14_link_H_root[c14_i17];
  }

  for (c14_i18 = 0; c14_i18 < 12; c14_i18++) {
    c14_b_inertial_0[c14_i18] = c14_inertial_0[c14_i18];
  }

  for (c14_i19 = 0; c14_i19 < 12; c14_i19++) {
    c14_b_inertial[c14_i19] = c14_inertial[c14_i19];
  }

  for (c14_i20 = 0; c14_i20 < 3; c14_i20++) {
    c14_b_neck[c14_i20] = c14_neck[c14_i20];
  }

  c14_c_CONFIG = c14_b_CONFIG;
  c14_fromBaseToWorldWithImu(chartInstance, c14_b_imu_H_link, c14_b_imu_H_link_0,
    c14_b_link_H_root, c14_b_inertial_0, c14_b_inertial, c14_b_neck,
    &c14_c_CONFIG, c14_dv1);
  for (c14_i21 = 0; c14_i21 < 16; c14_i21++) {
    c14_w_H_root[c14_i21] = c14_dv1[c14_i21];
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, -3);
  _SFD_SYMBOL_SCOPE_POP();
  for (c14_i22 = 0; c14_i22 < 16; c14_i22++) {
    (*c14_b_w_H_root)[c14_i22] = c14_w_H_root[c14_i22];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc14_torqueBalancing2012b
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c14_fromBaseToWorldWithImu(SFc14_torqueBalancing2012bInstanceStruct *
  chartInstance, real_T c14_imu_H_link[16], real_T c14_imu_H_link_0[16], real_T
  c14_link_H_root[16], real_T c14_inertial_0[12], real_T c14_inertial[12],
  real_T c14_neck[3], c14_struct_6drkWFoHJIWwAUE1rJmWJH *c14_b_CONFIG, real_T
  c14_w_H_root[16])
{
  uint32_T c14_debug_family_var_map[23];
  real_T c14_wImu_R_imu[9];
  real_T c14_wImu_R_imu_0[9];
  real_T c14_imu_R_link[9];
  real_T c14_imu_R_link_0[9];
  real_T c14_wImu_R_link_0[9];
  real_T c14_rollPitchYaw_link_0[3];
  real_T c14_rollPitchYaw_link[3];
  real_T c14_rollPitchYawFiltered_link[3];
  real_T c14_wImu_H_link[16];
  real_T c14_wImu_H_link_0[16];
  real_T c14_wImu_H_root[16];
  real_T c14_wImu_H_wImuAssumingNeckToZero[16];
  real_T c14_wImu_R_link[9];
  real_T c14_nargin = 7.0;
  real_T c14_nargout = 1.0;
  int32_T c14_i23;
  real_T c14_a[12];
  int32_T c14_i24;
  int32_T c14_i25;
  int32_T c14_i26;
  int32_T c14_i27;
  int32_T c14_i28;
  real_T c14_b_a[9];
  real_T c14_b[9];
  int32_T c14_i29;
  int32_T c14_i30;
  int32_T c14_i31;
  real_T c14_y[9];
  int32_T c14_i32;
  int32_T c14_i33;
  int32_T c14_i34;
  int32_T c14_i35;
  int32_T c14_i36;
  real_T c14_C[9];
  int32_T c14_i37;
  int32_T c14_i38;
  int32_T c14_i39;
  int32_T c14_i40;
  int32_T c14_i41;
  int32_T c14_i42;
  int32_T c14_i43;
  int32_T c14_i44;
  int32_T c14_i45;
  int32_T c14_i46;
  int32_T c14_i47;
  int32_T c14_i48;
  int32_T c14_i49;
  int32_T c14_i50;
  int32_T c14_i51;
  int32_T c14_i52;
  int32_T c14_i53;
  int32_T c14_i54;
  int32_T c14_i55;
  int32_T c14_i56;
  int32_T c14_i57;
  int32_T c14_i58;
  int32_T c14_i59;
  int32_T c14_i60;
  int32_T c14_i61;
  int32_T c14_i62;
  int32_T c14_i63;
  int32_T c14_i64;
  int32_T c14_i65;
  int32_T c14_i66;
  int32_T c14_i67;
  int32_T c14_i68;
  int32_T c14_i69;
  int32_T c14_i70;
  int32_T c14_i71;
  int32_T c14_i72;
  int32_T c14_i73;
  int32_T c14_i74;
  int32_T c14_i75;
  int32_T c14_i76;
  int32_T c14_i77;
  int32_T c14_i78;
  int32_T c14_i79;
  int32_T c14_i80;
  int32_T c14_i81;
  int32_T c14_i82;
  int32_T c14_i83;
  int32_T c14_i84;
  int32_T c14_i85;
  int32_T c14_i86;
  int32_T c14_i87;
  int32_T c14_i88;
  int32_T c14_i89;
  int32_T c14_i90;
  int32_T c14_i91;
  int32_T c14_i92;
  int32_T c14_i93;
  int32_T c14_i94;
  int32_T c14_i95;
  real_T c14_b_wImu_R_link_0[9];
  real_T c14_dv2[3];
  int32_T c14_i96;
  int32_T c14_i97;
  real_T c14_b_wImu_R_link[9];
  real_T c14_dv3[3];
  int32_T c14_i98;
  int32_T c14_i99;
  int32_T c14_i100;
  int32_T c14_i101;
  int32_T c14_i102;
  int32_T c14_i103;
  int32_T c14_i104;
  int32_T c14_i105;
  int32_T c14_i106;
  int32_T c14_i107;
  int32_T c14_i108;
  int32_T c14_i109;
  int32_T c14_i110;
  int32_T c14_i111;
  int32_T c14_i112;
  int32_T c14_i113;
  int32_T c14_i114;
  int32_T c14_i115;
  int32_T c14_i116;
  real_T c14_dv4[3];
  int32_T c14_i117;
  int32_T c14_i118;
  int32_T c14_i119;
  int32_T c14_i120;
  int32_T c14_i121;
  int32_T c14_i122;
  int32_T c14_i123;
  static real_T c14_dv5[4] = { 0.0, 0.0, 0.0, 1.0 };

  int32_T c14_i124;
  int32_T c14_i125;
  int32_T c14_i126;
  int32_T c14_i127;
  int32_T c14_i128;
  int32_T c14_i129;
  int32_T c14_i130;
  int32_T c14_i131;
  int32_T c14_i132;
  real_T c14_c_a[16];
  int32_T c14_i133;
  real_T c14_b_b[16];
  int32_T c14_i134;
  int32_T c14_i135;
  int32_T c14_i136;
  real_T c14_b_C[16];
  int32_T c14_i137;
  int32_T c14_i138;
  int32_T c14_i139;
  int32_T c14_i140;
  int32_T c14_i141;
  int32_T c14_i142;
  int32_T c14_i143;
  int32_T c14_i144;
  int32_T c14_i145;
  real_T c14_b_neck[3];
  real_T c14_dv6[16];
  int32_T c14_i146;
  int32_T c14_i147;
  int32_T c14_i148;
  int32_T c14_i149;
  int32_T c14_i150;
  int32_T c14_i151;
  int32_T c14_i152;
  int32_T c14_i153;
  int32_T c14_i154;
  int32_T c14_i155;
  int32_T c14_i156;
  int32_T c14_i157;
  int32_T c14_i158;
  int32_T c14_i159;
  int32_T c14_i160;
  int32_T c14_i161;
  int32_T c14_info;
  int32_T c14_ipiv[4];
  int32_T c14_b_info;
  int32_T c14_c_info;
  int32_T c14_d_info;
  int32_T c14_i162;
  int32_T c14_i;
  int32_T c14_b_i;
  int32_T c14_ip;
  int32_T c14_j;
  int32_T c14_b_j;
  real_T c14_temp;
  int32_T c14_i163;
  real_T c14_dv7[16];
  int32_T c14_i164;
  real_T c14_dv8[16];
  int32_T c14_i165;
  real_T c14_dv9[16];
  int32_T c14_i166;
  real_T c14_dv10[16];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 23U, 23U, c14_h_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_wImu_R_imu, 0U, c14_f_sf_marshallOut,
    c14_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_wImu_R_imu_0, 1U,
    c14_f_sf_marshallOut, c14_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_imu_R_link, 2U, c14_f_sf_marshallOut,
    c14_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_imu_R_link_0, 3U,
    c14_f_sf_marshallOut, c14_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_wImu_R_link_0, 4U,
    c14_f_sf_marshallOut, c14_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_rollPitchYaw_link_0, 5U,
    c14_c_sf_marshallOut, c14_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_rollPitchYaw_link, 6U,
    c14_c_sf_marshallOut, c14_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_rollPitchYawFiltered_link, 7U,
    c14_c_sf_marshallOut, c14_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_wImu_H_link, 8U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_wImu_H_link_0, 9U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_wImu_H_root, 10U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_wImu_H_wImuAssumingNeckToZero, 11U,
    c14_sf_marshallOut, c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_wImu_R_link, 12U,
    c14_f_sf_marshallOut, c14_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 13U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 14U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_imu_H_link, 15U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_imu_H_link_0, 16U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_link_H_root, 17U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_inertial_0, 18U, c14_d_sf_marshallOut,
    c14_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_inertial, 19U, c14_d_sf_marshallOut,
    c14_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_neck, 20U, c14_c_sf_marshallOut,
    c14_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_b_CONFIG, 21U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_w_H_root, 22U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 5);
  for (c14_i23 = 0; c14_i23 < 12; c14_i23++) {
    c14_a[c14_i23] = c14_inertial[c14_i23];
  }

  for (c14_i24 = 0; c14_i24 < 12; c14_i24++) {
    c14_a[c14_i24] *= 3.1415926535897931;
  }

  for (c14_i25 = 0; c14_i25 < 12; c14_i25++) {
    c14_inertial[c14_i25] = c14_a[c14_i25] / 180.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 6);
  for (c14_i26 = 0; c14_i26 < 12; c14_i26++) {
    c14_a[c14_i26] = c14_inertial_0[c14_i26];
  }

  for (c14_i27 = 0; c14_i27 < 12; c14_i27++) {
    c14_a[c14_i27] *= 3.1415926535897931;
  }

  for (c14_i28 = 0; c14_i28 < 12; c14_i28++) {
    c14_inertial_0[c14_i28] = c14_a[c14_i28] / 180.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 10);
  c14_rotz(chartInstance, c14_inertial[2], c14_b_a);
  c14_roty(chartInstance, c14_inertial[1], c14_b);
  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i29 = 0; c14_i29 < 3; c14_i29++) {
    c14_i30 = 0;
    for (c14_i31 = 0; c14_i31 < 3; c14_i31++) {
      c14_y[c14_i30 + c14_i29] = 0.0;
      c14_i32 = 0;
      for (c14_i33 = 0; c14_i33 < 3; c14_i33++) {
        c14_y[c14_i30 + c14_i29] += c14_b_a[c14_i32 + c14_i29] * c14_b[c14_i33 +
          c14_i30];
        c14_i32 += 3;
      }

      c14_i30 += 3;
    }
  }

  c14_rotx(chartInstance, c14_inertial[0], c14_b);
  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i34 = 0; c14_i34 < 9; c14_i34++) {
    c14_wImu_R_imu[c14_i34] = 0.0;
  }

  for (c14_i35 = 0; c14_i35 < 9; c14_i35++) {
    c14_wImu_R_imu[c14_i35] = 0.0;
  }

  for (c14_i36 = 0; c14_i36 < 9; c14_i36++) {
    c14_C[c14_i36] = c14_wImu_R_imu[c14_i36];
  }

  for (c14_i37 = 0; c14_i37 < 9; c14_i37++) {
    c14_wImu_R_imu[c14_i37] = c14_C[c14_i37];
  }

  for (c14_i38 = 0; c14_i38 < 9; c14_i38++) {
    c14_C[c14_i38] = c14_wImu_R_imu[c14_i38];
  }

  for (c14_i39 = 0; c14_i39 < 9; c14_i39++) {
    c14_wImu_R_imu[c14_i39] = c14_C[c14_i39];
  }

  for (c14_i40 = 0; c14_i40 < 3; c14_i40++) {
    c14_i41 = 0;
    for (c14_i42 = 0; c14_i42 < 3; c14_i42++) {
      c14_wImu_R_imu[c14_i41 + c14_i40] = 0.0;
      c14_i43 = 0;
      for (c14_i44 = 0; c14_i44 < 3; c14_i44++) {
        c14_wImu_R_imu[c14_i41 + c14_i40] += c14_y[c14_i43 + c14_i40] *
          c14_b[c14_i44 + c14_i41];
        c14_i43 += 3;
      }

      c14_i41 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 11);
  c14_rotz(chartInstance, c14_inertial_0[2], c14_b_a);
  c14_roty(chartInstance, c14_inertial_0[1], c14_b);
  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i45 = 0; c14_i45 < 3; c14_i45++) {
    c14_i46 = 0;
    for (c14_i47 = 0; c14_i47 < 3; c14_i47++) {
      c14_y[c14_i46 + c14_i45] = 0.0;
      c14_i48 = 0;
      for (c14_i49 = 0; c14_i49 < 3; c14_i49++) {
        c14_y[c14_i46 + c14_i45] += c14_b_a[c14_i48 + c14_i45] * c14_b[c14_i49 +
          c14_i46];
        c14_i48 += 3;
      }

      c14_i46 += 3;
    }
  }

  c14_rotx(chartInstance, c14_inertial_0[0], c14_b);
  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i50 = 0; c14_i50 < 9; c14_i50++) {
    c14_wImu_R_imu_0[c14_i50] = 0.0;
  }

  for (c14_i51 = 0; c14_i51 < 9; c14_i51++) {
    c14_wImu_R_imu_0[c14_i51] = 0.0;
  }

  for (c14_i52 = 0; c14_i52 < 9; c14_i52++) {
    c14_C[c14_i52] = c14_wImu_R_imu_0[c14_i52];
  }

  for (c14_i53 = 0; c14_i53 < 9; c14_i53++) {
    c14_wImu_R_imu_0[c14_i53] = c14_C[c14_i53];
  }

  for (c14_i54 = 0; c14_i54 < 9; c14_i54++) {
    c14_C[c14_i54] = c14_wImu_R_imu_0[c14_i54];
  }

  for (c14_i55 = 0; c14_i55 < 9; c14_i55++) {
    c14_wImu_R_imu_0[c14_i55] = c14_C[c14_i55];
  }

  for (c14_i56 = 0; c14_i56 < 3; c14_i56++) {
    c14_i57 = 0;
    for (c14_i58 = 0; c14_i58 < 3; c14_i58++) {
      c14_wImu_R_imu_0[c14_i57 + c14_i56] = 0.0;
      c14_i59 = 0;
      for (c14_i60 = 0; c14_i60 < 3; c14_i60++) {
        c14_wImu_R_imu_0[c14_i57 + c14_i56] += c14_y[c14_i59 + c14_i56] *
          c14_b[c14_i60 + c14_i57];
        c14_i59 += 3;
      }

      c14_i57 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 14);
  c14_i61 = 0;
  c14_i62 = 0;
  for (c14_i63 = 0; c14_i63 < 3; c14_i63++) {
    for (c14_i64 = 0; c14_i64 < 3; c14_i64++) {
      c14_imu_R_link[c14_i64 + c14_i61] = c14_imu_H_link[c14_i64 + c14_i62];
    }

    c14_i61 += 3;
    c14_i62 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 15);
  c14_i65 = 0;
  c14_i66 = 0;
  for (c14_i67 = 0; c14_i67 < 3; c14_i67++) {
    for (c14_i68 = 0; c14_i68 < 3; c14_i68++) {
      c14_imu_R_link_0[c14_i68 + c14_i65] = c14_imu_H_link_0[c14_i68 + c14_i66];
    }

    c14_i65 += 3;
    c14_i66 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 19);
  for (c14_i69 = 0; c14_i69 < 9; c14_i69++) {
    c14_b_a[c14_i69] = c14_wImu_R_imu[c14_i69];
  }

  for (c14_i70 = 0; c14_i70 < 9; c14_i70++) {
    c14_b[c14_i70] = c14_imu_R_link[c14_i70];
  }

  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i71 = 0; c14_i71 < 9; c14_i71++) {
    c14_wImu_R_link[c14_i71] = 0.0;
  }

  for (c14_i72 = 0; c14_i72 < 9; c14_i72++) {
    c14_wImu_R_link[c14_i72] = 0.0;
  }

  for (c14_i73 = 0; c14_i73 < 9; c14_i73++) {
    c14_C[c14_i73] = c14_wImu_R_link[c14_i73];
  }

  for (c14_i74 = 0; c14_i74 < 9; c14_i74++) {
    c14_wImu_R_link[c14_i74] = c14_C[c14_i74];
  }

  for (c14_i75 = 0; c14_i75 < 9; c14_i75++) {
    c14_C[c14_i75] = c14_wImu_R_link[c14_i75];
  }

  for (c14_i76 = 0; c14_i76 < 9; c14_i76++) {
    c14_wImu_R_link[c14_i76] = c14_C[c14_i76];
  }

  for (c14_i77 = 0; c14_i77 < 3; c14_i77++) {
    c14_i78 = 0;
    for (c14_i79 = 0; c14_i79 < 3; c14_i79++) {
      c14_wImu_R_link[c14_i78 + c14_i77] = 0.0;
      c14_i80 = 0;
      for (c14_i81 = 0; c14_i81 < 3; c14_i81++) {
        c14_wImu_R_link[c14_i78 + c14_i77] += c14_b_a[c14_i80 + c14_i77] *
          c14_b[c14_i81 + c14_i78];
        c14_i80 += 3;
      }

      c14_i78 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 20);
  for (c14_i82 = 0; c14_i82 < 9; c14_i82++) {
    c14_b_a[c14_i82] = c14_wImu_R_imu_0[c14_i82];
  }

  for (c14_i83 = 0; c14_i83 < 9; c14_i83++) {
    c14_b[c14_i83] = c14_imu_R_link_0[c14_i83];
  }

  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i84 = 0; c14_i84 < 9; c14_i84++) {
    c14_wImu_R_link_0[c14_i84] = 0.0;
  }

  for (c14_i85 = 0; c14_i85 < 9; c14_i85++) {
    c14_wImu_R_link_0[c14_i85] = 0.0;
  }

  for (c14_i86 = 0; c14_i86 < 9; c14_i86++) {
    c14_C[c14_i86] = c14_wImu_R_link_0[c14_i86];
  }

  for (c14_i87 = 0; c14_i87 < 9; c14_i87++) {
    c14_wImu_R_link_0[c14_i87] = c14_C[c14_i87];
  }

  for (c14_i88 = 0; c14_i88 < 9; c14_i88++) {
    c14_C[c14_i88] = c14_wImu_R_link_0[c14_i88];
  }

  for (c14_i89 = 0; c14_i89 < 9; c14_i89++) {
    c14_wImu_R_link_0[c14_i89] = c14_C[c14_i89];
  }

  for (c14_i90 = 0; c14_i90 < 3; c14_i90++) {
    c14_i91 = 0;
    for (c14_i92 = 0; c14_i92 < 3; c14_i92++) {
      c14_wImu_R_link_0[c14_i91 + c14_i90] = 0.0;
      c14_i93 = 0;
      for (c14_i94 = 0; c14_i94 < 3; c14_i94++) {
        c14_wImu_R_link_0[c14_i91 + c14_i90] += c14_b_a[c14_i93 + c14_i90] *
          c14_b[c14_i94 + c14_i91];
        c14_i93 += 3;
      }

      c14_i91 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 23);
  for (c14_i95 = 0; c14_i95 < 9; c14_i95++) {
    c14_b_wImu_R_link_0[c14_i95] = c14_wImu_R_link_0[c14_i95];
  }

  c14_rollPitchYawFromRotation(chartInstance, c14_b_wImu_R_link_0, c14_dv2);
  for (c14_i96 = 0; c14_i96 < 3; c14_i96++) {
    c14_rollPitchYaw_link_0[c14_i96] = c14_dv2[c14_i96];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 24);
  for (c14_i97 = 0; c14_i97 < 9; c14_i97++) {
    c14_b_wImu_R_link[c14_i97] = c14_wImu_R_link[c14_i97];
  }

  c14_rollPitchYawFromRotation(chartInstance, c14_b_wImu_R_link, c14_dv3);
  for (c14_i98 = 0; c14_i98 < 3; c14_i98++) {
    c14_rollPitchYaw_link[c14_i98] = c14_dv3[c14_i98];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 26);
  for (c14_i99 = 0; c14_i99 < 3; c14_i99++) {
    c14_rollPitchYawFiltered_link[c14_i99] = c14_rollPitchYaw_link[c14_i99];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 28);
  if (CV_SCRIPT_IF(0, 0, c14_b_CONFIG->YAW_IMU_FILTER)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 29);
    c14_rollPitchYawFiltered_link[2] = c14_rollPitchYaw_link_0[2];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 31);
  if (CV_SCRIPT_IF(0, 1, c14_b_CONFIG->PITCH_IMU_FILTER)) {
    _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 32);
    c14_rollPitchYawFiltered_link[1] = c14_rollPitchYaw_link_0[1];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 35);
  c14_rotz(chartInstance, c14_rollPitchYawFiltered_link[2], c14_b_a);
  c14_roty(chartInstance, c14_rollPitchYawFiltered_link[1], c14_b);
  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i100 = 0; c14_i100 < 3; c14_i100++) {
    c14_i101 = 0;
    for (c14_i102 = 0; c14_i102 < 3; c14_i102++) {
      c14_y[c14_i101 + c14_i100] = 0.0;
      c14_i103 = 0;
      for (c14_i104 = 0; c14_i104 < 3; c14_i104++) {
        c14_y[c14_i101 + c14_i100] += c14_b_a[c14_i103 + c14_i100] *
          c14_b[c14_i104 + c14_i101];
        c14_i103 += 3;
      }

      c14_i101 += 3;
    }
  }

  c14_rotx(chartInstance, c14_rollPitchYawFiltered_link[0], c14_b);
  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i105 = 0; c14_i105 < 9; c14_i105++) {
    c14_wImu_R_link[c14_i105] = 0.0;
  }

  for (c14_i106 = 0; c14_i106 < 9; c14_i106++) {
    c14_wImu_R_link[c14_i106] = 0.0;
  }

  for (c14_i107 = 0; c14_i107 < 9; c14_i107++) {
    c14_C[c14_i107] = c14_wImu_R_link[c14_i107];
  }

  for (c14_i108 = 0; c14_i108 < 9; c14_i108++) {
    c14_wImu_R_link[c14_i108] = c14_C[c14_i108];
  }

  for (c14_i109 = 0; c14_i109 < 9; c14_i109++) {
    c14_C[c14_i109] = c14_wImu_R_link[c14_i109];
  }

  for (c14_i110 = 0; c14_i110 < 9; c14_i110++) {
    c14_wImu_R_link[c14_i110] = c14_C[c14_i110];
  }

  for (c14_i111 = 0; c14_i111 < 3; c14_i111++) {
    c14_i112 = 0;
    for (c14_i113 = 0; c14_i113 < 3; c14_i113++) {
      c14_wImu_R_link[c14_i112 + c14_i111] = 0.0;
      c14_i114 = 0;
      for (c14_i115 = 0; c14_i115 < 3; c14_i115++) {
        c14_wImu_R_link[c14_i112 + c14_i111] += c14_y[c14_i114 + c14_i111] *
          c14_b[c14_i115 + c14_i112];
        c14_i114 += 3;
      }

      c14_i112 += 3;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 37);
  for (c14_i116 = 0; c14_i116 < 3; c14_i116++) {
    c14_dv4[c14_i116] = 0.0;
  }

  c14_i117 = 0;
  c14_i118 = 0;
  for (c14_i119 = 0; c14_i119 < 3; c14_i119++) {
    for (c14_i120 = 0; c14_i120 < 3; c14_i120++) {
      c14_wImu_H_link[c14_i120 + c14_i117] = c14_wImu_R_link[c14_i120 + c14_i118];
    }

    c14_i117 += 4;
    c14_i118 += 3;
  }

  for (c14_i121 = 0; c14_i121 < 3; c14_i121++) {
    c14_wImu_H_link[c14_i121 + 12] = c14_dv4[c14_i121];
  }

  c14_i122 = 0;
  for (c14_i123 = 0; c14_i123 < 4; c14_i123++) {
    c14_wImu_H_link[c14_i122 + 3] = c14_dv5[c14_i123];
    c14_i122 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 40);
  for (c14_i124 = 0; c14_i124 < 3; c14_i124++) {
    c14_dv4[c14_i124] = 0.0;
  }

  c14_i125 = 0;
  c14_i126 = 0;
  for (c14_i127 = 0; c14_i127 < 3; c14_i127++) {
    for (c14_i128 = 0; c14_i128 < 3; c14_i128++) {
      c14_wImu_H_link_0[c14_i128 + c14_i125] = c14_wImu_R_link_0[c14_i128 +
        c14_i126];
    }

    c14_i125 += 4;
    c14_i126 += 3;
  }

  for (c14_i129 = 0; c14_i129 < 3; c14_i129++) {
    c14_wImu_H_link_0[c14_i129 + 12] = c14_dv4[c14_i129];
  }

  c14_i130 = 0;
  for (c14_i131 = 0; c14_i131 < 4; c14_i131++) {
    c14_wImu_H_link_0[c14_i130 + 3] = c14_dv5[c14_i131];
    c14_i130 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 43);
  for (c14_i132 = 0; c14_i132 < 16; c14_i132++) {
    c14_c_a[c14_i132] = c14_wImu_H_link[c14_i132];
  }

  for (c14_i133 = 0; c14_i133 < 16; c14_i133++) {
    c14_b_b[c14_i133] = c14_link_H_root[c14_i133];
  }

  c14_c_eml_scalar_eg(chartInstance);
  c14_c_eml_scalar_eg(chartInstance);
  for (c14_i134 = 0; c14_i134 < 16; c14_i134++) {
    c14_wImu_H_root[c14_i134] = 0.0;
  }

  for (c14_i135 = 0; c14_i135 < 16; c14_i135++) {
    c14_wImu_H_root[c14_i135] = 0.0;
  }

  for (c14_i136 = 0; c14_i136 < 16; c14_i136++) {
    c14_b_C[c14_i136] = c14_wImu_H_root[c14_i136];
  }

  for (c14_i137 = 0; c14_i137 < 16; c14_i137++) {
    c14_wImu_H_root[c14_i137] = c14_b_C[c14_i137];
  }

  for (c14_i138 = 0; c14_i138 < 16; c14_i138++) {
    c14_b_C[c14_i138] = c14_wImu_H_root[c14_i138];
  }

  for (c14_i139 = 0; c14_i139 < 16; c14_i139++) {
    c14_wImu_H_root[c14_i139] = c14_b_C[c14_i139];
  }

  for (c14_i140 = 0; c14_i140 < 4; c14_i140++) {
    c14_i141 = 0;
    for (c14_i142 = 0; c14_i142 < 4; c14_i142++) {
      c14_wImu_H_root[c14_i141 + c14_i140] = 0.0;
      c14_i143 = 0;
      for (c14_i144 = 0; c14_i144 < 4; c14_i144++) {
        c14_wImu_H_root[c14_i141 + c14_i140] += c14_c_a[c14_i143 + c14_i140] *
          c14_b_b[c14_i144 + c14_i141];
        c14_i143 += 4;
      }

      c14_i141 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 46);
  for (c14_i145 = 0; c14_i145 < 3; c14_i145++) {
    c14_b_neck[c14_i145] = c14_neck[c14_i145];
  }

  c14_correctIMU(chartInstance, c14_b_neck, c14_dv6);
  for (c14_i146 = 0; c14_i146 < 16; c14_i146++) {
    c14_wImu_H_wImuAssumingNeckToZero[c14_i146] = c14_dv6[c14_i146];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 47);
  for (c14_i147 = 0; c14_i147 < 16; c14_i147++) {
    c14_c_a[c14_i147] = c14_wImu_H_wImuAssumingNeckToZero[c14_i147];
  }

  for (c14_i148 = 0; c14_i148 < 16; c14_i148++) {
    c14_b_b[c14_i148] = c14_wImu_H_root[c14_i148];
  }

  c14_c_eml_scalar_eg(chartInstance);
  c14_c_eml_scalar_eg(chartInstance);
  for (c14_i149 = 0; c14_i149 < 16; c14_i149++) {
    c14_wImu_H_root[c14_i149] = 0.0;
  }

  for (c14_i150 = 0; c14_i150 < 16; c14_i150++) {
    c14_wImu_H_root[c14_i150] = 0.0;
  }

  for (c14_i151 = 0; c14_i151 < 16; c14_i151++) {
    c14_b_C[c14_i151] = c14_wImu_H_root[c14_i151];
  }

  for (c14_i152 = 0; c14_i152 < 16; c14_i152++) {
    c14_wImu_H_root[c14_i152] = c14_b_C[c14_i152];
  }

  for (c14_i153 = 0; c14_i153 < 16; c14_i153++) {
    c14_b_C[c14_i153] = c14_wImu_H_root[c14_i153];
  }

  for (c14_i154 = 0; c14_i154 < 16; c14_i154++) {
    c14_wImu_H_root[c14_i154] = c14_b_C[c14_i154];
  }

  for (c14_i155 = 0; c14_i155 < 4; c14_i155++) {
    c14_i156 = 0;
    for (c14_i157 = 0; c14_i157 < 4; c14_i157++) {
      c14_wImu_H_root[c14_i156 + c14_i155] = 0.0;
      c14_i158 = 0;
      for (c14_i159 = 0; c14_i159 < 4; c14_i159++) {
        c14_wImu_H_root[c14_i156 + c14_i155] += c14_c_a[c14_i158 + c14_i155] *
          c14_b_b[c14_i159 + c14_i156];
        c14_i158 += 4;
      }

      c14_i156 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 49);
  for (c14_i160 = 0; c14_i160 < 16; c14_i160++) {
    c14_c_a[c14_i160] = c14_wImu_H_link_0[c14_i160];
  }

  for (c14_i161 = 0; c14_i161 < 16; c14_i161++) {
    c14_b_b[c14_i161] = c14_wImu_H_root[c14_i161];
  }

  c14_b_eml_matlab_zgetrf(chartInstance, c14_c_a, c14_ipiv, &c14_info);
  c14_b_info = c14_info;
  c14_c_info = c14_b_info;
  c14_d_info = c14_c_info;
  if (c14_d_info > 0) {
    c14_eml_warning(chartInstance);
  }

  c14_c_eml_scalar_eg(chartInstance);
  for (c14_i162 = 0; c14_i162 < 16; c14_i162++) {
    c14_w_H_root[c14_i162] = c14_b_b[c14_i162];
  }

  for (c14_i = 1; c14_i < 5; c14_i++) {
    c14_b_i = c14_i;
    if (c14_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c14_b_i), 1, 4, 1, 0) - 1] != c14_b_i) {
      c14_ip = c14_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c14_b_i), 1, 4, 1, 0) - 1];
      for (c14_j = 1; c14_j < 5; c14_j++) {
        c14_b_j = c14_j;
        c14_temp = c14_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_b_i), 1, 4, 1, 0) +
          ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c14_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c14_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_b_i), 1, 4, 1, 0) +
                      ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_b_j), 1, 4, 2, 0) - 1) << 2)) - 1] =
          c14_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_ip), 1, 4, 1, 0) +
                        ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c14_w_H_root[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_ip), 1, 4, 1, 0) +
                      ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_b_j), 1, 4, 2, 0) - 1) << 2)) - 1] =
          c14_temp;
      }
    }
  }

  for (c14_i163 = 0; c14_i163 < 16; c14_i163++) {
    c14_dv7[c14_i163] = c14_c_a[c14_i163];
  }

  for (c14_i164 = 0; c14_i164 < 16; c14_i164++) {
    c14_dv8[c14_i164] = c14_dv7[c14_i164];
  }

  c14_c_eml_xtrsm(chartInstance, c14_dv8, c14_w_H_root);
  for (c14_i165 = 0; c14_i165 < 16; c14_i165++) {
    c14_dv9[c14_i165] = c14_c_a[c14_i165];
  }

  for (c14_i166 = 0; c14_i166 < 16; c14_i166++) {
    c14_dv10[c14_i166] = c14_dv9[c14_i166];
  }

  c14_d_eml_xtrsm(chartInstance, c14_dv10, c14_w_H_root);
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, -49);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c14_rotz(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c14_alpha, real_T c14_R[9])
{
  uint32_T c14_debug_family_var_map[4];
  real_T c14_nargin = 1.0;
  real_T c14_nargout = 1.0;
  int32_T c14_i167;
  real_T c14_x;
  real_T c14_b_x;
  real_T c14_c_x;
  real_T c14_d_x;
  real_T c14_e_x;
  real_T c14_f_x;
  real_T c14_g_x;
  real_T c14_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c14_b_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 0U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 1U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_alpha, 2U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_R, 3U, c14_f_sf_marshallOut,
    c14_d_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 2);
  for (c14_i167 = 0; c14_i167 < 9; c14_i167++) {
    c14_R[c14_i167] = 0.0;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 3);
  c14_R[8] = 1.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 4);
  c14_x = c14_alpha;
  c14_b_x = c14_x;
  c14_b_x = muDoubleScalarCos(c14_b_x);
  c14_R[0] = c14_b_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 5);
  c14_c_x = c14_alpha;
  c14_d_x = c14_c_x;
  c14_d_x = muDoubleScalarSin(c14_d_x);
  c14_R[3] = -c14_d_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 6);
  c14_e_x = c14_alpha;
  c14_f_x = c14_e_x;
  c14_f_x = muDoubleScalarSin(c14_f_x);
  c14_R[1] = c14_f_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 7);
  c14_g_x = c14_alpha;
  c14_h_x = c14_g_x;
  c14_h_x = muDoubleScalarCos(c14_h_x);
  c14_R[4] = c14_h_x;
  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c14_roty(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c14_alpha, real_T c14_R[9])
{
  uint32_T c14_debug_family_var_map[4];
  real_T c14_nargin = 1.0;
  real_T c14_nargout = 1.0;
  int32_T c14_i168;
  real_T c14_x;
  real_T c14_b_x;
  real_T c14_c_x;
  real_T c14_d_x;
  real_T c14_e_x;
  real_T c14_f_x;
  real_T c14_g_x;
  real_T c14_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c14_c_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 0U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 1U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_alpha, 2U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_R, 3U, c14_f_sf_marshallOut,
    c14_d_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c14_sfEvent, 2);
  for (c14_i168 = 0; c14_i168 < 9; c14_i168++) {
    c14_R[c14_i168] = 0.0;
  }

  _SFD_SCRIPT_CALL(2U, chartInstance->c14_sfEvent, 3);
  c14_R[4] = 1.0;
  _SFD_SCRIPT_CALL(2U, chartInstance->c14_sfEvent, 4);
  c14_x = c14_alpha;
  c14_b_x = c14_x;
  c14_b_x = muDoubleScalarCos(c14_b_x);
  c14_R[0] = c14_b_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c14_sfEvent, 5);
  c14_c_x = c14_alpha;
  c14_d_x = c14_c_x;
  c14_d_x = muDoubleScalarSin(c14_d_x);
  c14_R[6] = c14_d_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c14_sfEvent, 6);
  c14_e_x = c14_alpha;
  c14_f_x = c14_e_x;
  c14_f_x = muDoubleScalarSin(c14_f_x);
  c14_R[2] = -c14_f_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c14_sfEvent, 7);
  c14_g_x = c14_alpha;
  c14_h_x = c14_g_x;
  c14_h_x = muDoubleScalarCos(c14_h_x);
  c14_R[8] = c14_h_x;
  _SFD_SCRIPT_CALL(2U, chartInstance->c14_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c14_rotx(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
                     real_T c14_alpha, real_T c14_R[9])
{
  uint32_T c14_debug_family_var_map[4];
  real_T c14_nargin = 1.0;
  real_T c14_nargout = 1.0;
  int32_T c14_i169;
  real_T c14_x;
  real_T c14_b_x;
  real_T c14_c_x;
  real_T c14_d_x;
  real_T c14_e_x;
  real_T c14_f_x;
  real_T c14_g_x;
  real_T c14_h_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c14_d_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 0U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 1U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_alpha, 2U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_R, 3U, c14_f_sf_marshallOut,
    c14_d_sf_marshallIn);
  CV_SCRIPT_FCN(3, 0);
  _SFD_SCRIPT_CALL(3U, chartInstance->c14_sfEvent, 2);
  for (c14_i169 = 0; c14_i169 < 9; c14_i169++) {
    c14_R[c14_i169] = 0.0;
  }

  _SFD_SCRIPT_CALL(3U, chartInstance->c14_sfEvent, 3);
  c14_R[0] = 1.0;
  _SFD_SCRIPT_CALL(3U, chartInstance->c14_sfEvent, 4);
  c14_x = c14_alpha;
  c14_b_x = c14_x;
  c14_b_x = muDoubleScalarCos(c14_b_x);
  c14_R[4] = c14_b_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c14_sfEvent, 5);
  c14_c_x = c14_alpha;
  c14_d_x = c14_c_x;
  c14_d_x = muDoubleScalarSin(c14_d_x);
  c14_R[7] = -c14_d_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c14_sfEvent, 6);
  c14_e_x = c14_alpha;
  c14_f_x = c14_e_x;
  c14_f_x = muDoubleScalarSin(c14_f_x);
  c14_R[5] = c14_f_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c14_sfEvent, 7);
  c14_g_x = c14_alpha;
  c14_h_x = c14_g_x;
  c14_h_x = muDoubleScalarCos(c14_h_x);
  c14_R[8] = c14_h_x;
  _SFD_SCRIPT_CALL(3U, chartInstance->c14_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c14_rollPitchYawFromRotation
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance, real_T c14_R[9],
   real_T c14_rollPitchYaw[3])
{
  uint32_T c14_debug_family_var_map[4];
  real_T c14_nargin = 1.0;
  real_T c14_nargout = 1.0;
  int32_T c14_i170;
  real_T c14_x;
  real_T c14_b_x;
  boolean_T guard1 = FALSE;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c14_e_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 0U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 1U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_R, 2U, c14_f_sf_marshallOut,
    c14_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_rollPitchYaw, 3U,
    c14_c_sf_marshallOut, c14_e_sf_marshallIn);
  CV_SCRIPT_FCN(4, 0);
  _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 4);
  for (c14_i170 = 0; c14_i170 < 3; c14_i170++) {
    c14_rollPitchYaw[c14_i170] = 0.0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 5);
  if (CV_SCRIPT_IF(4, 0, c14_R[2] < 1.0)) {
    _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 6);
    if (CV_SCRIPT_IF(4, 1, c14_R[2] > -1.0)) {
      _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 7);
      c14_x = -c14_R[2];
      c14_b_x = c14_x;
      guard1 = FALSE;
      if (c14_b_x < -1.0) {
        guard1 = TRUE;
      } else {
        if (1.0 < c14_b_x) {
          guard1 = TRUE;
        }
      }

      if (guard1 == TRUE) {
        c14_eml_error(chartInstance);
      }

      c14_b_x = muDoubleScalarAsin(c14_b_x);
      c14_rollPitchYaw[1] = c14_b_x;
      _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 8);
      c14_rollPitchYaw[2] = c14_atan2(chartInstance, c14_R[1], c14_R[0]);
      _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 9);
      c14_rollPitchYaw[0] = c14_atan2(chartInstance, c14_R[5], c14_R[8]);
    } else {
      _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 11);
      c14_rollPitchYaw[2] = -c14_atan2(chartInstance, -c14_R[7], c14_R[4]);
      _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 12);
      c14_rollPitchYaw[0] = 0.0;
    }
  } else {
    _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 15);
    c14_rollPitchYaw[2] = c14_atan2(chartInstance, -c14_R[7], c14_R[4]);
    _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, 16);
    c14_rollPitchYaw[0] = 0.0;
  }

  _SFD_SCRIPT_CALL(4U, chartInstance->c14_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c14_correctIMU(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_neckJoints[3], real_T
  c14_imu_H_imuAssumingNeckToZero[16])
{
  uint32_T c14_debug_family_var_map[10];
  real_T c14_G_34[16];
  real_T c14_G_45[16];
  real_T c14_G_56[16];
  real_T c14_G_6I[16];
  real_T c14_imuAssumingNeckToZero_H_neckBase[16];
  real_T c14_imu_H_neckBase[16];
  real_T c14_nargin = 1.0;
  real_T c14_nargout = 1.0;
  int32_T c14_i171;
  static real_T c14_dv11[16] = { 6.123233995736766E-17, 1.0, 0.0, 0.0,
    -6.123233995736766E-17, 3.749399456654644E-33, 1.0, 0.0, 1.0,
    -6.123233995736766E-17, 6.123233995736766E-17, 0.0, 5.8170722959499274E-19,
    0.0095, 0.0, 1.0 };

  int32_T c14_i172;
  static real_T c14_dv12[16] = { 6.123233995736766E-17, -1.0, 0.0, 0.0,
    6.123233995736766E-17, 3.749399456654644E-33, -1.0, 0.0, 1.0,
    6.123233995736766E-17, 6.123233995736766E-17, 0.0, 0.0, -0.0, 0.0, 1.0 };

  int32_T c14_i173;
  static real_T c14_dv13[16] = { 6.123233995736766E-17, 1.0, 0.0, 0.0,
    -6.123233995736766E-17, 3.749399456654644E-33, -1.0, 0.0, -1.0,
    6.123233995736766E-17, 6.123233995736766E-17, 0.0, 1.1327982892113017E-18,
    0.0185, 0.1108, 1.0 };

  int32_T c14_i174;
  static real_T c14_b[16] = { 1.0, 0.0, 0.0, 0.0, -0.0, 6.123233995736766E-17,
    1.0, 0.0, 0.0, -1.0, 6.123233995736766E-17, 0.0, 0.0, 0.0, 0.0066, 1.0 };

  int32_T c14_i175;
  static real_T c14_dv14[16] = { -1.0, 1.2246467991473532E-16,
    -1.2246467991473532E-16, 0.0, -1.2246467991473532E-16,
    -6.1232339957367648E-17, 1.0, 0.0, 1.2246467991473532E-16, 1.0,
    6.123233995736766E-17, 0.0, -0.018499999999999985, 0.12029999999999999,
    0.0066000000000000043, 1.0 };

  real_T c14_dv15[16];
  int32_T c14_i176;
  real_T c14_dv16[16];
  int32_T c14_i177;
  real_T c14_dv17[16];
  int32_T c14_i178;
  int32_T c14_i179;
  int32_T c14_i180;
  real_T c14_a[16];
  int32_T c14_i181;
  real_T c14_b_b[16];
  int32_T c14_i182;
  int32_T c14_i183;
  int32_T c14_i184;
  real_T c14_y[16];
  int32_T c14_i185;
  int32_T c14_i186;
  int32_T c14_i187;
  int32_T c14_i188;
  int32_T c14_i189;
  int32_T c14_i190;
  real_T c14_b_y[16];
  int32_T c14_i191;
  int32_T c14_i192;
  int32_T c14_i193;
  int32_T c14_i194;
  int32_T c14_i195;
  int32_T c14_i196;
  int32_T c14_i197;
  int32_T c14_i198;
  int32_T c14_i199;
  int32_T c14_i200;
  int32_T c14_i201;
  int32_T c14_i202;
  int32_T c14_i203;
  int32_T c14_i204;
  int32_T c14_i205;
  int32_T c14_i206;
  int32_T c14_i207;
  int32_T c14_i208;
  int32_T c14_i;
  int32_T c14_b_i;
  static int32_T c14_iv0[4] = { 1, 3, 3, 4 };

  int32_T c14_ip;
  int32_T c14_j;
  int32_T c14_b_j;
  real_T c14_temp;
  int32_T c14_i209;
  static real_T c14_dv18[16] = { -1.0, -1.2246467991473532E-16,
    1.2246467991473532E-16, 0.018499999999999985, 1.2246467991473532E-16, 1.0,
    -6.123233995736766E-17, 0.12029999999999999, -1.2246467991473532E-16,
    6.1232339957367648E-17, 1.0, 0.0066, 0.0, 0.0, 0.0, 1.0 };

  real_T c14_dv19[16];
  int32_T c14_i210;
  real_T c14_dv20[16];
  int32_T c14_i211;
  int32_T c14_i212;
  int32_T c14_i213;
  int32_T c14_i214;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c14_g_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_G_34, 0U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_G_45, 1U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_G_56, 2U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_G_6I, 3U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_imuAssumingNeckToZero_H_neckBase, 4U,
    c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_imu_H_neckBase, 5U,
    c14_sf_marshallOut, c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 6U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 7U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_neckJoints, 8U, c14_c_sf_marshallOut,
    c14_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_imu_H_imuAssumingNeckToZero, 9U,
    c14_sf_marshallOut, c14_sf_marshallIn);
  CV_SCRIPT_FCN(5, 0);
  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 9);
  for (c14_i171 = 0; c14_i171 < 16; c14_i171++) {
    c14_G_34[c14_i171] = c14_dv11[c14_i171];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 10);
  for (c14_i172 = 0; c14_i172 < 16; c14_i172++) {
    c14_G_45[c14_i172] = c14_dv12[c14_i172];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 11);
  for (c14_i173 = 0; c14_i173 < 16; c14_i173++) {
    c14_G_56[c14_i173] = c14_dv13[c14_i173];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 12);
  for (c14_i174 = 0; c14_i174 < 16; c14_i174++) {
    c14_G_6I[c14_i174] = c14_b[c14_i174];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 14);
  for (c14_i175 = 0; c14_i175 < 16; c14_i175++) {
    c14_imuAssumingNeckToZero_H_neckBase[c14_i175] = c14_dv14[c14_i175];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 17);
  c14_evalDHMatrix(chartInstance, 0.0095, 0.0, 1.5707963267948966,
                   c14_neckJoints[0] + 1.5707963267948966, c14_dv15);
  for (c14_i176 = 0; c14_i176 < 16; c14_i176++) {
    c14_G_34[c14_i176] = c14_dv15[c14_i176];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 18);
  c14_evalDHMatrix(chartInstance, 0.0, 0.0, -1.5707963267948966, c14_neckJoints
                   [1] - 1.5707963267948966, c14_dv16);
  for (c14_i177 = 0; c14_i177 < 16; c14_i177++) {
    c14_G_45[c14_i177] = c14_dv16[c14_i177];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 19);
  c14_evalDHMatrix(chartInstance, 0.0185, 0.1108, -1.5707963267948966,
                   c14_neckJoints[2] + 1.5707963267948966, c14_dv17);
  for (c14_i178 = 0; c14_i178 < 16; c14_i178++) {
    c14_G_56[c14_i178] = c14_dv17[c14_i178];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 20);
  for (c14_i179 = 0; c14_i179 < 16; c14_i179++) {
    c14_G_6I[c14_i179] = c14_b[c14_i179];
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 22);
  for (c14_i180 = 0; c14_i180 < 16; c14_i180++) {
    c14_a[c14_i180] = c14_G_34[c14_i180];
  }

  for (c14_i181 = 0; c14_i181 < 16; c14_i181++) {
    c14_b_b[c14_i181] = c14_G_45[c14_i181];
  }

  c14_c_eml_scalar_eg(chartInstance);
  c14_c_eml_scalar_eg(chartInstance);
  for (c14_i182 = 0; c14_i182 < 4; c14_i182++) {
    c14_i183 = 0;
    for (c14_i184 = 0; c14_i184 < 4; c14_i184++) {
      c14_y[c14_i183 + c14_i182] = 0.0;
      c14_i185 = 0;
      for (c14_i186 = 0; c14_i186 < 4; c14_i186++) {
        c14_y[c14_i183 + c14_i182] += c14_a[c14_i185 + c14_i182] *
          c14_b_b[c14_i186 + c14_i183];
        c14_i185 += 4;
      }

      c14_i183 += 4;
    }
  }

  for (c14_i187 = 0; c14_i187 < 16; c14_i187++) {
    c14_b_b[c14_i187] = c14_G_56[c14_i187];
  }

  c14_c_eml_scalar_eg(chartInstance);
  c14_c_eml_scalar_eg(chartInstance);
  for (c14_i188 = 0; c14_i188 < 4; c14_i188++) {
    c14_i189 = 0;
    for (c14_i190 = 0; c14_i190 < 4; c14_i190++) {
      c14_b_y[c14_i189 + c14_i188] = 0.0;
      c14_i191 = 0;
      for (c14_i192 = 0; c14_i192 < 4; c14_i192++) {
        c14_b_y[c14_i189 + c14_i188] += c14_y[c14_i191 + c14_i188] *
          c14_b_b[c14_i192 + c14_i189];
        c14_i191 += 4;
      }

      c14_i189 += 4;
    }
  }

  c14_c_eml_scalar_eg(chartInstance);
  c14_c_eml_scalar_eg(chartInstance);
  for (c14_i193 = 0; c14_i193 < 16; c14_i193++) {
    c14_imu_H_neckBase[c14_i193] = 0.0;
  }

  for (c14_i194 = 0; c14_i194 < 16; c14_i194++) {
    c14_imu_H_neckBase[c14_i194] = 0.0;
  }

  for (c14_i195 = 0; c14_i195 < 16; c14_i195++) {
    c14_a[c14_i195] = c14_imu_H_neckBase[c14_i195];
  }

  for (c14_i196 = 0; c14_i196 < 16; c14_i196++) {
    c14_imu_H_neckBase[c14_i196] = c14_a[c14_i196];
  }

  for (c14_i197 = 0; c14_i197 < 16; c14_i197++) {
    c14_a[c14_i197] = c14_imu_H_neckBase[c14_i197];
  }

  for (c14_i198 = 0; c14_i198 < 16; c14_i198++) {
    c14_imu_H_neckBase[c14_i198] = c14_a[c14_i198];
  }

  for (c14_i199 = 0; c14_i199 < 4; c14_i199++) {
    c14_i200 = 0;
    for (c14_i201 = 0; c14_i201 < 4; c14_i201++) {
      c14_imu_H_neckBase[c14_i200 + c14_i199] = 0.0;
      c14_i202 = 0;
      for (c14_i203 = 0; c14_i203 < 4; c14_i203++) {
        c14_imu_H_neckBase[c14_i200 + c14_i199] += c14_b_y[c14_i202 + c14_i199] *
          c14_b[c14_i203 + c14_i200];
        c14_i202 += 4;
      }

      c14_i200 += 4;
    }
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, 24);
  for (c14_i204 = 0; c14_i204 < 16; c14_i204++) {
    c14_a[c14_i204] = c14_imu_H_neckBase[c14_i204];
  }

  c14_i205 = 0;
  for (c14_i206 = 0; c14_i206 < 4; c14_i206++) {
    c14_i207 = 0;
    for (c14_i208 = 0; c14_i208 < 4; c14_i208++) {
      c14_b_b[c14_i208 + c14_i205] = c14_a[c14_i207 + c14_i206];
      c14_i207 += 4;
    }

    c14_i205 += 4;
  }

  c14_c_eml_scalar_eg(chartInstance);
  for (c14_i = 1; c14_i < 5; c14_i++) {
    c14_b_i = c14_i;
    if (c14_iv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c14_b_i), 1, 4, 1, 0) - 1] != c14_b_i) {
      c14_ip = c14_iv0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c14_b_i), 1, 4, 1, 0) - 1];
      for (c14_j = 1; c14_j < 5; c14_j++) {
        c14_b_j = c14_j;
        c14_temp = c14_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_b_i), 1, 4, 1, 0) +
                            ((_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_b_j), 1, 4, 2, 0) - 1) << 2)) - 1];
        c14_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c14_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK
                   ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_b_j), 1, 4,
                    2, 0) - 1) << 2)) - 1] = c14_b_b
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c14_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_b_j), 1, 4, 2, 0) - 1)
             << 2)) - 1];
        c14_b_b[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c14_ip), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK(
                    "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_b_j), 1, 4,
                    2, 0) - 1) << 2)) - 1] = c14_temp;
      }
    }
  }

  for (c14_i209 = 0; c14_i209 < 16; c14_i209++) {
    c14_dv19[c14_i209] = c14_dv18[c14_i209];
  }

  c14_c_eml_xtrsm(chartInstance, c14_dv19, c14_b_b);
  for (c14_i210 = 0; c14_i210 < 16; c14_i210++) {
    c14_dv20[c14_i210] = c14_dv18[c14_i210];
  }

  c14_d_eml_xtrsm(chartInstance, c14_dv20, c14_b_b);
  c14_i211 = 0;
  for (c14_i212 = 0; c14_i212 < 4; c14_i212++) {
    c14_i213 = 0;
    for (c14_i214 = 0; c14_i214 < 4; c14_i214++) {
      c14_imu_H_imuAssumingNeckToZero[c14_i214 + c14_i211] = c14_b_b[c14_i213 +
        c14_i212];
      c14_i213 += 4;
    }

    c14_i211 += 4;
  }

  _SFD_SCRIPT_CALL(5U, chartInstance->c14_sfEvent, -24);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c14_evalDHMatrix(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_a, real_T c14_d, real_T c14_alph, real_T c14_thet,
  real_T c14_G[16])
{
  uint32_T c14_debug_family_var_map[7];
  real_T c14_nargin = 4.0;
  real_T c14_nargout = 1.0;
  real_T c14_x;
  real_T c14_b_x;
  real_T c14_c_x;
  real_T c14_d_x;
  real_T c14_e_x;
  real_T c14_f_x;
  real_T c14_b_a;
  real_T c14_b;
  real_T c14_y;
  real_T c14_g_x;
  real_T c14_h_x;
  real_T c14_i_x;
  real_T c14_j_x;
  real_T c14_c_a;
  real_T c14_b_b;
  real_T c14_b_y;
  real_T c14_k_x;
  real_T c14_l_x;
  real_T c14_d_a;
  real_T c14_c_b;
  real_T c14_c_y;
  real_T c14_m_x;
  real_T c14_n_x;
  real_T c14_o_x;
  real_T c14_p_x;
  real_T c14_q_x;
  real_T c14_r_x;
  real_T c14_e_a;
  real_T c14_d_b;
  real_T c14_d_y;
  real_T c14_s_x;
  real_T c14_t_x;
  real_T c14_u_x;
  real_T c14_v_x;
  real_T c14_f_a;
  real_T c14_e_b;
  real_T c14_e_y;
  real_T c14_w_x;
  real_T c14_x_x;
  real_T c14_g_a;
  real_T c14_f_b;
  real_T c14_f_y;
  real_T c14_y_x;
  real_T c14_ab_x;
  real_T c14_bb_x;
  real_T c14_cb_x;
  int32_T c14_i215;
  int32_T c14_i216;
  static real_T c14_dv21[4] = { 0.0, 0.0, 0.0, 1.0 };

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c14_f_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 0U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 1U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_a, 2U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_d, 3U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_alph, 4U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_thet, 5U, c14_e_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_G, 6U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  CV_SCRIPT_FCN(6, 0);
  _SFD_SCRIPT_CALL(6U, chartInstance->c14_sfEvent, 3);
  c14_x = c14_thet;
  c14_b_x = c14_x;
  c14_b_x = muDoubleScalarCos(c14_b_x);
  c14_c_x = c14_thet;
  c14_d_x = c14_c_x;
  c14_d_x = muDoubleScalarSin(c14_d_x);
  c14_e_x = c14_alph;
  c14_f_x = c14_e_x;
  c14_f_x = muDoubleScalarCos(c14_f_x);
  c14_b_a = -c14_d_x;
  c14_b = c14_f_x;
  c14_y = c14_b_a * c14_b;
  c14_g_x = c14_thet;
  c14_h_x = c14_g_x;
  c14_h_x = muDoubleScalarSin(c14_h_x);
  c14_i_x = c14_alph;
  c14_j_x = c14_i_x;
  c14_j_x = muDoubleScalarSin(c14_j_x);
  c14_c_a = c14_h_x;
  c14_b_b = c14_j_x;
  c14_b_y = c14_c_a * c14_b_b;
  c14_k_x = c14_thet;
  c14_l_x = c14_k_x;
  c14_l_x = muDoubleScalarCos(c14_l_x);
  c14_d_a = c14_l_x;
  c14_c_b = c14_a;
  c14_c_y = c14_d_a * c14_c_b;
  c14_m_x = c14_thet;
  c14_n_x = c14_m_x;
  c14_n_x = muDoubleScalarSin(c14_n_x);
  c14_o_x = c14_thet;
  c14_p_x = c14_o_x;
  c14_p_x = muDoubleScalarCos(c14_p_x);
  c14_q_x = c14_alph;
  c14_r_x = c14_q_x;
  c14_r_x = muDoubleScalarCos(c14_r_x);
  c14_e_a = c14_p_x;
  c14_d_b = c14_r_x;
  c14_d_y = c14_e_a * c14_d_b;
  c14_s_x = c14_thet;
  c14_t_x = c14_s_x;
  c14_t_x = muDoubleScalarCos(c14_t_x);
  c14_u_x = c14_alph;
  c14_v_x = c14_u_x;
  c14_v_x = muDoubleScalarSin(c14_v_x);
  c14_f_a = -c14_t_x;
  c14_e_b = c14_v_x;
  c14_e_y = c14_f_a * c14_e_b;
  c14_w_x = c14_thet;
  c14_x_x = c14_w_x;
  c14_x_x = muDoubleScalarSin(c14_x_x);
  c14_g_a = c14_x_x;
  c14_f_b = c14_a;
  c14_f_y = c14_g_a * c14_f_b;
  c14_y_x = c14_alph;
  c14_ab_x = c14_y_x;
  c14_ab_x = muDoubleScalarSin(c14_ab_x);
  c14_bb_x = c14_alph;
  c14_cb_x = c14_bb_x;
  c14_cb_x = muDoubleScalarCos(c14_cb_x);
  c14_G[0] = c14_b_x;
  c14_G[4] = c14_y;
  c14_G[8] = c14_b_y;
  c14_G[12] = c14_c_y;
  c14_G[1] = c14_n_x;
  c14_G[5] = c14_d_y;
  c14_G[9] = c14_e_y;
  c14_G[13] = c14_f_y;
  c14_G[2] = 0.0;
  c14_G[6] = c14_ab_x;
  c14_G[10] = c14_cb_x;
  c14_G[14] = c14_d;
  c14_i215 = 0;
  for (c14_i216 = 0; c14_i216 < 4; c14_i216++) {
    c14_G[c14_i215 + 3] = c14_dv21[c14_i216];
    c14_i215 += 4;
  }

  _SFD_SCRIPT_CALL(6U, chartInstance->c14_sfEvent, -3);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c14_machineNumber, uint32_T
  c14_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, 0U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m"));
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, 1U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m"));
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, 2U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m"));
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, 3U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m"));
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, 4U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m"));
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, 5U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m"));
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, 6U, sf_debug_get_script_id(
    "/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m"));
}

static const mxArray *c14_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i217;
  int32_T c14_i218;
  int32_T c14_i219;
  real_T c14_b_inData[16];
  int32_T c14_i220;
  int32_T c14_i221;
  int32_T c14_i222;
  real_T c14_u[16];
  const mxArray *c14_y = NULL;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_i217 = 0;
  for (c14_i218 = 0; c14_i218 < 4; c14_i218++) {
    for (c14_i219 = 0; c14_i219 < 4; c14_i219++) {
      c14_b_inData[c14_i219 + c14_i217] = (*(real_T (*)[16])c14_inData)[c14_i219
        + c14_i217];
    }

    c14_i217 += 4;
  }

  c14_i220 = 0;
  for (c14_i221 = 0; c14_i221 < 4; c14_i221++) {
    for (c14_i222 = 0; c14_i222 < 4; c14_i222++) {
      c14_u[c14_i222 + c14_i220] = c14_b_inData[c14_i222 + c14_i220];
    }

    c14_i220 += 4;
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static void c14_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_w_H_root, const char_T *c14_identifier,
  real_T c14_y[16])
{
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_w_H_root), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_w_H_root);
}

static void c14_b_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[16])
{
  real_T c14_dv22[16];
  int32_T c14_i223;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv22, 1, 0, 0U, 1, 0U, 2, 4,
                4);
  for (c14_i223 = 0; c14_i223 < 16; c14_i223++) {
    c14_y[c14_i223] = c14_dv22[c14_i223];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_w_H_root;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[16];
  int32_T c14_i224;
  int32_T c14_i225;
  int32_T c14_i226;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_w_H_root = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_w_H_root), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_w_H_root);
  c14_i224 = 0;
  for (c14_i225 = 0; c14_i225 < 4; c14_i225++) {
    for (c14_i226 = 0; c14_i226 < 4; c14_i226++) {
      (*(real_T (*)[16])c14_outData)[c14_i226 + c14_i224] = c14_y[c14_i226 +
        c14_i224];
    }

    c14_i224 += 4;
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

static const mxArray *c14_b_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData;
  c14_struct_6drkWFoHJIWwAUE1rJmWJH c14_u;
  const mxArray *c14_y = NULL;
  real_T c14_b_u;
  const mxArray *c14_b_y = NULL;
  c14_struct_amzdx4J7qaaMzgPI7fQ3WD c14_c_u;
  const mxArray *c14_c_y = NULL;
  boolean_T c14_d_u;
  const mxArray *c14_d_y = NULL;
  boolean_T c14_e_u;
  const mxArray *c14_e_y = NULL;
  boolean_T c14_f_u;
  const mxArray *c14_f_y = NULL;
  boolean_T c14_g_u;
  const mxArray *c14_g_y = NULL;
  boolean_T c14_h_u;
  const mxArray *c14_h_y = NULL;
  boolean_T c14_i_u;
  const mxArray *c14_i_y = NULL;
  boolean_T c14_j_u;
  const mxArray *c14_j_y = NULL;
  boolean_T c14_k_u;
  const mxArray *c14_k_y = NULL;
  boolean_T c14_l_u;
  const mxArray *c14_l_y = NULL;
  boolean_T c14_m_u;
  const mxArray *c14_m_y = NULL;
  boolean_T c14_n_u;
  const mxArray *c14_n_y = NULL;
  boolean_T c14_o_u;
  const mxArray *c14_o_y = NULL;
  boolean_T c14_p_u;
  const mxArray *c14_p_y = NULL;
  real_T c14_q_u;
  const mxArray *c14_q_y = NULL;
  boolean_T c14_r_u;
  const mxArray *c14_r_y = NULL;
  int32_T c14_i227;
  real_T c14_s_u[2];
  const mxArray *c14_s_y = NULL;
  real_T c14_t_u;
  const mxArray *c14_t_y = NULL;
  real_T c14_u_u;
  const mxArray *c14_u_y = NULL;
  real_T c14_v_u;
  const mxArray *c14_v_y = NULL;
  boolean_T c14_w_u;
  const mxArray *c14_w_y = NULL;
  real_T c14_x_u;
  const mxArray *c14_x_y = NULL;
  int32_T c14_i228;
  real_T c14_y_u[23];
  const mxArray *c14_y_y = NULL;
  int32_T c14_i229;
  real_T c14_ab_u[23];
  const mxArray *c14_ab_y = NULL;
  boolean_T c14_bb_u;
  const mxArray *c14_bb_y = NULL;
  boolean_T c14_cb_u;
  const mxArray *c14_cb_y = NULL;
  boolean_T c14_db_u;
  const mxArray *c14_db_y = NULL;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_mxArrayOutData = NULL;
  c14_u = *(c14_struct_6drkWFoHJIWwAUE1rJmWJH *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c14_b_u = c14_u.SIMULATION_TIME;
  c14_b_y = NULL;
  sf_mex_assign(&c14_b_y, sf_mex_create("y", &c14_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_b_y, "SIMULATION_TIME", "SIMULATION_TIME", 0);
  c14_c_u = c14_u.SCOPES;
  c14_c_y = NULL;
  sf_mex_assign(&c14_c_y, sf_mex_createstruct("structure", 2, 1, 1), FALSE);
  c14_d_u = c14_c_u.ALL;
  c14_d_y = NULL;
  sf_mex_assign(&c14_d_y, sf_mex_create("y", &c14_d_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_c_y, c14_d_y, "ALL", "ALL", 0);
  c14_e_u = c14_c_u.BASE_EST_IMU;
  c14_e_y = NULL;
  sf_mex_assign(&c14_e_y, sf_mex_create("y", &c14_e_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_c_y, c14_e_y, "BASE_EST_IMU", "BASE_EST_IMU", 0);
  c14_f_u = c14_c_u.EXTWRENCHES;
  c14_f_y = NULL;
  sf_mex_assign(&c14_f_y, sf_mex_create("y", &c14_f_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_c_y, c14_f_y, "EXTWRENCHES", "EXTWRENCHES", 0);
  c14_g_u = c14_c_u.GAIN_SCHE_INFO;
  c14_g_y = NULL;
  sf_mex_assign(&c14_g_y, sf_mex_create("y", &c14_g_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_c_y, c14_g_y, "GAIN_SCHE_INFO", "GAIN_SCHE_INFO", 0);
  c14_h_u = c14_c_u.MAIN;
  c14_h_y = NULL;
  sf_mex_assign(&c14_h_y, sf_mex_create("y", &c14_h_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_c_y, c14_h_y, "MAIN", "MAIN", 0);
  c14_i_u = c14_c_u.QP;
  c14_i_y = NULL;
  sf_mex_assign(&c14_i_y, sf_mex_create("y", &c14_i_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_c_y, c14_i_y, "QP", "QP", 0);
  sf_mex_addfield(c14_y, c14_c_y, "SCOPES", "SCOPES", 0);
  c14_j_u = c14_u.CHECK_LIMITS;
  c14_j_y = NULL;
  sf_mex_assign(&c14_j_y, sf_mex_create("y", &c14_j_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_j_y, "CHECK_LIMITS", "CHECK_LIMITS", 0);
  c14_k_u = c14_u.USE_IMU4EST_BASE;
  c14_k_y = NULL;
  sf_mex_assign(&c14_k_y, sf_mex_create("y", &c14_k_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_k_y, "USE_IMU4EST_BASE", "USE_IMU4EST_BASE", 0);
  c14_l_u = c14_u.YAW_IMU_FILTER;
  c14_l_y = NULL;
  sf_mex_assign(&c14_l_y, sf_mex_create("y", &c14_l_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_l_y, "YAW_IMU_FILTER", "YAW_IMU_FILTER", 0);
  c14_m_u = c14_u.PITCH_IMU_FILTER;
  c14_m_y = NULL;
  sf_mex_assign(&c14_m_y, sf_mex_create("y", &c14_m_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_m_y, "PITCH_IMU_FILTER", "PITCH_IMU_FILTER", 0);
  c14_n_u = c14_u.CORRECT_NECK_IMU;
  c14_n_y = NULL;
  sf_mex_assign(&c14_n_y, sf_mex_create("y", &c14_n_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_n_y, "CORRECT_NECK_IMU", "CORRECT_NECK_IMU", 0);
  c14_o_u = c14_u.ONSOFTCARPET;
  c14_o_y = NULL;
  sf_mex_assign(&c14_o_y, sf_mex_create("y", &c14_o_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_o_y, "ONSOFTCARPET", "ONSOFTCARPET", 0);
  c14_p_u = c14_u.USE_QP_SOLVER;
  c14_p_y = NULL;
  sf_mex_assign(&c14_p_y, sf_mex_create("y", &c14_p_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_p_y, "USE_QP_SOLVER", "USE_QP_SOLVER", 0);
  c14_q_u = c14_u.Ts;
  c14_q_y = NULL;
  sf_mex_assign(&c14_q_y, sf_mex_create("y", &c14_q_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_q_y, "Ts", "Ts", 0);
  c14_r_u = c14_u.ON_GAZEBO;
  c14_r_y = NULL;
  sf_mex_assign(&c14_r_y, sf_mex_create("y", &c14_r_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_r_y, "ON_GAZEBO", "ON_GAZEBO", 0);
  for (c14_i227 = 0; c14_i227 < 2; c14_i227++) {
    c14_s_u[c14_i227] = c14_u.LEFT_RIGHT_FOOT_IN_CONTACT[c14_i227];
  }

  c14_s_y = NULL;
  sf_mex_assign(&c14_s_y, sf_mex_create("y", c14_s_u, 0, 0U, 1U, 0U, 2, 1, 2),
                FALSE);
  sf_mex_addfield(c14_y, c14_s_y, "LEFT_RIGHT_FOOT_IN_CONTACT",
                  "LEFT_RIGHT_FOOT_IN_CONTACT", 0);
  c14_t_u = c14_u.SMOOTH_DES_COM;
  c14_t_y = NULL;
  sf_mex_assign(&c14_t_y, sf_mex_create("y", &c14_t_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_t_y, "SMOOTH_DES_COM", "SMOOTH_DES_COM", 0);
  c14_u_u = c14_u.SMOOTH_DES_Q;
  c14_u_y = NULL;
  sf_mex_assign(&c14_u_y, sf_mex_create("y", &c14_u_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_u_y, "SMOOTH_DES_Q", "SMOOTH_DES_Q", 0);
  c14_v_u = c14_u.smoothingTimeTranDynamics;
  c14_v_y = NULL;
  sf_mex_assign(&c14_v_y, sf_mex_create("y", &c14_v_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_v_y, "smoothingTimeTranDynamics",
                  "smoothingTimeTranDynamics", 0);
  c14_w_u = c14_u.DEMO_MOVEMENTS;
  c14_w_y = NULL;
  sf_mex_assign(&c14_w_y, sf_mex_create("y", &c14_w_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_w_y, "DEMO_MOVEMENTS", "DEMO_MOVEMENTS", 0);
  c14_x_u = c14_u.TIME_CONTROLLER_SWITCH;
  c14_x_y = NULL;
  sf_mex_assign(&c14_x_y, sf_mex_create("y", &c14_x_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_addfield(c14_y, c14_x_y, "TIME_CONTROLLER_SWITCH",
                  "TIME_CONTROLLER_SWITCH", 0);
  for (c14_i228 = 0; c14_i228 < 23; c14_i228++) {
    c14_y_u[c14_i228] = c14_u.JOINTS[c14_i228];
  }

  c14_y_y = NULL;
  sf_mex_assign(&c14_y_y, sf_mex_create("y", c14_y_u, 0, 0U, 1U, 0U, 2, 23, 1),
                FALSE);
  sf_mex_addfield(c14_y, c14_y_y, "JOINTS", "JOINTS", 0);
  for (c14_i229 = 0; c14_i229 < 23; c14_i229++) {
    c14_ab_u[c14_i229] = c14_u.JOINTSITING[c14_i229];
  }

  c14_ab_y = NULL;
  sf_mex_assign(&c14_ab_y, sf_mex_create("y", c14_ab_u, 0, 0U, 1U, 0U, 2, 23, 1),
                FALSE);
  sf_mex_addfield(c14_y, c14_ab_y, "JOINTSITING", "JOINTSITING", 0);
  c14_bb_u = c14_u.FIXEDCOMBENCH;
  c14_bb_y = NULL;
  sf_mex_assign(&c14_bb_y, sf_mex_create("y", &c14_bb_u, 11, 0U, 0U, 0U, 0),
                FALSE);
  sf_mex_addfield(c14_y, c14_bb_y, "FIXEDCOMBENCH", "FIXEDCOMBENCH", 0);
  c14_cb_u = c14_u.iCubStandUp;
  c14_cb_y = NULL;
  sf_mex_assign(&c14_cb_y, sf_mex_create("y", &c14_cb_u, 11, 0U, 0U, 0U, 0),
                FALSE);
  sf_mex_addfield(c14_y, c14_cb_y, "iCubStandUp", "iCubStandUp", 0);
  c14_db_u = c14_u.useExtArmForces;
  c14_db_y = NULL;
  sf_mex_assign(&c14_db_y, sf_mex_create("y", &c14_db_u, 11, 0U, 0U, 0U, 0),
                FALSE);
  sf_mex_addfield(c14_y, c14_db_y, "useExtArmForces", "useExtArmForces", 0);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static void c14_c_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  c14_struct_6drkWFoHJIWwAUE1rJmWJH *c14_y)
{
  emlrtMsgIdentifier c14_thisId;
  static const char * c14_fieldNames[22] = { "SIMULATION_TIME", "SCOPES",
    "CHECK_LIMITS", "USE_IMU4EST_BASE", "YAW_IMU_FILTER", "PITCH_IMU_FILTER",
    "CORRECT_NECK_IMU", "ONSOFTCARPET", "USE_QP_SOLVER", "Ts", "ON_GAZEBO",
    "LEFT_RIGHT_FOOT_IN_CONTACT", "SMOOTH_DES_COM", "SMOOTH_DES_Q",
    "smoothingTimeTranDynamics", "DEMO_MOVEMENTS", "TIME_CONTROLLER_SWITCH",
    "JOINTS", "JOINTSITING", "FIXEDCOMBENCH", "iCubStandUp", "useExtArmForces" };

  c14_thisId.fParent = c14_parentId;
  sf_mex_check_struct(c14_parentId, c14_u, 22, c14_fieldNames, 0U, 0);
  c14_thisId.fIdentifier = "SIMULATION_TIME";
  c14_y->SIMULATION_TIME = c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "SIMULATION_TIME", "SIMULATION_TIME", 0)),
    &c14_thisId);
  c14_thisId.fIdentifier = "SCOPES";
  c14_y->SCOPES = c14_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "SCOPES", "SCOPES", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "CHECK_LIMITS";
  c14_y->CHECK_LIMITS = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "CHECK_LIMITS", "CHECK_LIMITS", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "USE_IMU4EST_BASE";
  c14_y->USE_IMU4EST_BASE = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "USE_IMU4EST_BASE", "USE_IMU4EST_BASE", 0)),
    &c14_thisId);
  c14_thisId.fIdentifier = "YAW_IMU_FILTER";
  c14_y->YAW_IMU_FILTER = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "YAW_IMU_FILTER", "YAW_IMU_FILTER", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "PITCH_IMU_FILTER";
  c14_y->PITCH_IMU_FILTER = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "PITCH_IMU_FILTER", "PITCH_IMU_FILTER", 0)),
    &c14_thisId);
  c14_thisId.fIdentifier = "CORRECT_NECK_IMU";
  c14_y->CORRECT_NECK_IMU = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "CORRECT_NECK_IMU", "CORRECT_NECK_IMU", 0)),
    &c14_thisId);
  c14_thisId.fIdentifier = "ONSOFTCARPET";
  c14_y->ONSOFTCARPET = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "ONSOFTCARPET", "ONSOFTCARPET", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "USE_QP_SOLVER";
  c14_y->USE_QP_SOLVER = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "USE_QP_SOLVER", "USE_QP_SOLVER", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "Ts";
  c14_y->Ts = c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c14_u, "Ts", "Ts", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "ON_GAZEBO";
  c14_y->ON_GAZEBO = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "ON_GAZEBO", "ON_GAZEBO", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "LEFT_RIGHT_FOOT_IN_CONTACT";
  c14_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c14_u,
    "LEFT_RIGHT_FOOT_IN_CONTACT", "LEFT_RIGHT_FOOT_IN_CONTACT", 0)), &c14_thisId,
    c14_y->LEFT_RIGHT_FOOT_IN_CONTACT);
  c14_thisId.fIdentifier = "SMOOTH_DES_COM";
  c14_y->SMOOTH_DES_COM = c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "SMOOTH_DES_COM", "SMOOTH_DES_COM", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "SMOOTH_DES_Q";
  c14_y->SMOOTH_DES_Q = c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "SMOOTH_DES_Q", "SMOOTH_DES_Q", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "smoothingTimeTranDynamics";
  c14_y->smoothingTimeTranDynamics = c14_d_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c14_u, "smoothingTimeTranDynamics",
    "smoothingTimeTranDynamics", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "DEMO_MOVEMENTS";
  c14_y->DEMO_MOVEMENTS = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "DEMO_MOVEMENTS", "DEMO_MOVEMENTS", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "TIME_CONTROLLER_SWITCH";
  c14_y->TIME_CONTROLLER_SWITCH = c14_d_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c14_u, "TIME_CONTROLLER_SWITCH",
    "TIME_CONTROLLER_SWITCH", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "JOINTS";
  c14_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c14_u,
    "JOINTS", "JOINTS", 0)), &c14_thisId, c14_y->JOINTS);
  c14_thisId.fIdentifier = "JOINTSITING";
  c14_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c14_u,
    "JOINTSITING", "JOINTSITING", 0)), &c14_thisId, c14_y->JOINTSITING);
  c14_thisId.fIdentifier = "FIXEDCOMBENCH";
  c14_y->FIXEDCOMBENCH = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "FIXEDCOMBENCH", "FIXEDCOMBENCH", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "iCubStandUp";
  c14_y->iCubStandUp = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "iCubStandUp", "iCubStandUp", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "useExtArmForces";
  c14_y->useExtArmForces = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "useExtArmForces", "useExtArmForces", 0)),
    &c14_thisId);
  sf_mex_destroy(&c14_u);
}

static real_T c14_d_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  real_T c14_y;
  real_T c14_d0;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_d0, 1, 0, 0U, 0, 0U, 0);
  c14_y = c14_d0;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static c14_struct_amzdx4J7qaaMzgPI7fQ3WD c14_e_emlrt_marshallIn
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray *c14_u,
   const emlrtMsgIdentifier *c14_parentId)
{
  c14_struct_amzdx4J7qaaMzgPI7fQ3WD c14_y;
  emlrtMsgIdentifier c14_thisId;
  static const char * c14_fieldNames[6] = { "ALL", "BASE_EST_IMU", "EXTWRENCHES",
    "GAIN_SCHE_INFO", "MAIN", "QP" };

  c14_thisId.fParent = c14_parentId;
  sf_mex_check_struct(c14_parentId, c14_u, 6, c14_fieldNames, 0U, 0);
  c14_thisId.fIdentifier = "ALL";
  c14_y.ALL = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c14_u, "ALL", "ALL", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "BASE_EST_IMU";
  c14_y.BASE_EST_IMU = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "BASE_EST_IMU", "BASE_EST_IMU", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "EXTWRENCHES";
  c14_y.EXTWRENCHES = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "EXTWRENCHES", "EXTWRENCHES", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "GAIN_SCHE_INFO";
  c14_y.GAIN_SCHE_INFO = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c14_u, "GAIN_SCHE_INFO", "GAIN_SCHE_INFO", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "MAIN";
  c14_y.MAIN = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c14_u, "MAIN", "MAIN", 0)), &c14_thisId);
  c14_thisId.fIdentifier = "QP";
  c14_y.QP = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c14_u, "QP", "QP", 0)), &c14_thisId);
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static boolean_T c14_f_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  boolean_T c14_y;
  boolean_T c14_b0;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_b0, 1, 11, 0U, 0, 0U, 0);
  c14_y = c14_b0;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void c14_g_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[2])
{
  real_T c14_dv23[2];
  int32_T c14_i230;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv23, 1, 0, 0U, 1, 0U, 2, 1,
                2);
  for (c14_i230 = 0; c14_i230 < 2; c14_i230++) {
    c14_y[c14_i230] = c14_dv23[c14_i230];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_h_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[23])
{
  real_T c14_dv24[23];
  int32_T c14_i231;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv24, 1, 0, 0U, 1, 0U, 2,
                23, 1);
  for (c14_i231 = 0; c14_i231 < 23; c14_i231++) {
    c14_y[c14_i231] = c14_dv24[c14_i231];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_b_CONFIG;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  c14_struct_6drkWFoHJIWwAUE1rJmWJH c14_y;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_b_CONFIG = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_CONFIG), &c14_thisId,
    &c14_y);
  sf_mex_destroy(&c14_b_CONFIG);
  *(c14_struct_6drkWFoHJIWwAUE1rJmWJH *)c14_outData = c14_y;
  sf_mex_destroy(&c14_mxArrayInData);
}

static const mxArray *c14_c_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i232;
  real_T c14_b_inData[3];
  int32_T c14_i233;
  real_T c14_u[3];
  const mxArray *c14_y = NULL;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  for (c14_i232 = 0; c14_i232 < 3; c14_i232++) {
    c14_b_inData[c14_i232] = (*(real_T (*)[3])c14_inData)[c14_i232];
  }

  for (c14_i233 = 0; c14_i233 < 3; c14_i233++) {
    c14_u[c14_i233] = c14_b_inData[c14_i233];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static const mxArray *c14_d_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i234;
  real_T c14_b_inData[12];
  int32_T c14_i235;
  real_T c14_u[12];
  const mxArray *c14_y = NULL;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  for (c14_i234 = 0; c14_i234 < 12; c14_i234++) {
    c14_b_inData[c14_i234] = (*(real_T (*)[12])c14_inData)[c14_i234];
  }

  for (c14_i235 = 0; c14_i235 < 12; c14_i235++) {
    c14_u[c14_i235] = c14_b_inData[c14_i235];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static const mxArray *c14_e_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  real_T c14_u;
  const mxArray *c14_y = NULL;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_u = *(real_T *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static void c14_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_nargout;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_nargout = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_nargout),
    &c14_thisId);
  sf_mex_destroy(&c14_nargout);
  *(real_T *)c14_outData = c14_y;
  sf_mex_destroy(&c14_mxArrayInData);
}

static const mxArray *c14_f_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i236;
  int32_T c14_i237;
  int32_T c14_i238;
  real_T c14_b_inData[9];
  int32_T c14_i239;
  int32_T c14_i240;
  int32_T c14_i241;
  real_T c14_u[9];
  const mxArray *c14_y = NULL;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_i236 = 0;
  for (c14_i237 = 0; c14_i237 < 3; c14_i237++) {
    for (c14_i238 = 0; c14_i238 < 3; c14_i238++) {
      c14_b_inData[c14_i238 + c14_i236] = (*(real_T (*)[9])c14_inData)[c14_i238
        + c14_i236];
    }

    c14_i236 += 3;
  }

  c14_i239 = 0;
  for (c14_i240 = 0; c14_i240 < 3; c14_i240++) {
    for (c14_i241 = 0; c14_i241 < 3; c14_i241++) {
      c14_u[c14_i241 + c14_i239] = c14_b_inData[c14_i241 + c14_i239];
    }

    c14_i239 += 3;
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static void c14_i_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[9])
{
  real_T c14_dv25[9];
  int32_T c14_i242;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv25, 1, 0, 0U, 1, 0U, 2, 3,
                3);
  for (c14_i242 = 0; c14_i242 < 9; c14_i242++) {
    c14_y[c14_i242] = c14_dv25[c14_i242];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_R;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[9];
  int32_T c14_i243;
  int32_T c14_i244;
  int32_T c14_i245;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_R = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_R), &c14_thisId, c14_y);
  sf_mex_destroy(&c14_R);
  c14_i243 = 0;
  for (c14_i244 = 0; c14_i244 < 3; c14_i244++) {
    for (c14_i245 = 0; c14_i245 < 3; c14_i245++) {
      (*(real_T (*)[9])c14_outData)[c14_i245 + c14_i243] = c14_y[c14_i245 +
        c14_i243];
    }

    c14_i243 += 3;
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

static void c14_j_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[3])
{
  real_T c14_dv26[3];
  int32_T c14_i246;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv26, 1, 0, 0U, 1, 0U, 1, 3);
  for (c14_i246 = 0; c14_i246 < 3; c14_i246++) {
    c14_y[c14_i246] = c14_dv26[c14_i246];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_rollPitchYaw;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[3];
  int32_T c14_i247;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_rollPitchYaw = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_rollPitchYaw),
    &c14_thisId, c14_y);
  sf_mex_destroy(&c14_rollPitchYaw);
  for (c14_i247 = 0; c14_i247 < 3; c14_i247++) {
    (*(real_T (*)[3])c14_outData)[c14_i247] = c14_y[c14_i247];
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

static void c14_k_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[12])
{
  real_T c14_dv27[12];
  int32_T c14_i248;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv27, 1, 0, 0U, 1, 0U, 1,
                12);
  for (c14_i248 = 0; c14_i248 < 12; c14_i248++) {
    c14_y[c14_i248] = c14_dv27[c14_i248];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_inertial;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[12];
  int32_T c14_i249;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_inertial = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_inertial), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_inertial);
  for (c14_i249 = 0; c14_i249 < 12; c14_i249++) {
    (*(real_T (*)[12])c14_outData)[c14_i249] = c14_y[c14_i249];
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

const mxArray *sf_c14_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c14_nameCaptureInfo;
  c14_ResolvedFunctionInfo c14_info[147];
  const mxArray *c14_m2 = NULL;
  int32_T c14_i250;
  c14_ResolvedFunctionInfo *c14_r1;
  c14_nameCaptureInfo = NULL;
  c14_nameCaptureInfo = NULL;
  c14_info_helper(c14_info);
  c14_b_info_helper(c14_info);
  c14_c_info_helper(c14_info);
  sf_mex_assign(&c14_m2, sf_mex_createstruct("nameCaptureInfo", 1, 147), FALSE);
  for (c14_i250 = 0; c14_i250 < 147; c14_i250++) {
    c14_r1 = &c14_info[c14_i250];
    sf_mex_addfield(c14_m2, sf_mex_create("nameCaptureInfo", c14_r1->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c14_r1->context)), "context", "nameCaptureInfo",
                    c14_i250);
    sf_mex_addfield(c14_m2, sf_mex_create("nameCaptureInfo", c14_r1->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c14_r1->name)), "name", "nameCaptureInfo",
                    c14_i250);
    sf_mex_addfield(c14_m2, sf_mex_create("nameCaptureInfo",
      c14_r1->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c14_r1->dominantType)),
                    "dominantType", "nameCaptureInfo", c14_i250);
    sf_mex_addfield(c14_m2, sf_mex_create("nameCaptureInfo", c14_r1->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c14_r1->resolved)), "resolved",
                    "nameCaptureInfo", c14_i250);
    sf_mex_addfield(c14_m2, sf_mex_create("nameCaptureInfo", &c14_r1->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c14_i250);
    sf_mex_addfield(c14_m2, sf_mex_create("nameCaptureInfo", &c14_r1->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c14_i250);
    sf_mex_addfield(c14_m2, sf_mex_create("nameCaptureInfo",
      &c14_r1->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c14_i250);
    sf_mex_addfield(c14_m2, sf_mex_create("nameCaptureInfo",
      &c14_r1->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c14_i250);
  }

  sf_mex_assign(&c14_nameCaptureInfo, c14_m2, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c14_nameCaptureInfo);
  return c14_nameCaptureInfo;
}

static void c14_info_helper(c14_ResolvedFunctionInfo c14_info[147])
{
  c14_info[0].context = "";
  c14_info[0].name = "fromBaseToWorldWithImu";
  c14_info[0].dominantType = "struct";
  c14_info[0].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[0].fileTimeLo = 1498554660U;
  c14_info[0].fileTimeHi = 0U;
  c14_info[0].mFileTimeLo = 0U;
  c14_info[0].mFileTimeHi = 0U;
  c14_info[1].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[1].name = "mtimes";
  c14_info[1].dominantType = "double";
  c14_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[1].fileTimeLo = 1289519692U;
  c14_info[1].fileTimeHi = 0U;
  c14_info[1].mFileTimeLo = 0U;
  c14_info[1].mFileTimeHi = 0U;
  c14_info[2].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[2].name = "mrdivide";
  c14_info[2].dominantType = "double";
  c14_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c14_info[2].fileTimeLo = 1357951548U;
  c14_info[2].fileTimeHi = 0U;
  c14_info[2].mFileTimeLo = 1319729966U;
  c14_info[2].mFileTimeHi = 0U;
  c14_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c14_info[3].name = "rdivide";
  c14_info[3].dominantType = "double";
  c14_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c14_info[3].fileTimeLo = 1346510388U;
  c14_info[3].fileTimeHi = 0U;
  c14_info[3].mFileTimeLo = 0U;
  c14_info[3].mFileTimeHi = 0U;
  c14_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c14_info[4].name = "eml_scalexp_compatible";
  c14_info[4].dominantType = "double";
  c14_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c14_info[4].fileTimeLo = 1286818796U;
  c14_info[4].fileTimeHi = 0U;
  c14_info[4].mFileTimeLo = 0U;
  c14_info[4].mFileTimeHi = 0U;
  c14_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c14_info[5].name = "eml_div";
  c14_info[5].dominantType = "double";
  c14_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c14_info[5].fileTimeLo = 1313347810U;
  c14_info[5].fileTimeHi = 0U;
  c14_info[5].mFileTimeLo = 0U;
  c14_info[5].mFileTimeHi = 0U;
  c14_info[6].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[6].name = "rotz";
  c14_info[6].dominantType = "double";
  c14_info[6].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c14_info[6].fileTimeLo = 1495631764U;
  c14_info[6].fileTimeHi = 0U;
  c14_info[6].mFileTimeLo = 0U;
  c14_info[6].mFileTimeHi = 0U;
  c14_info[7].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c14_info[7].name = "cos";
  c14_info[7].dominantType = "double";
  c14_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c14_info[7].fileTimeLo = 1343830372U;
  c14_info[7].fileTimeHi = 0U;
  c14_info[7].mFileTimeLo = 0U;
  c14_info[7].mFileTimeHi = 0U;
  c14_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c14_info[8].name = "eml_scalar_cos";
  c14_info[8].dominantType = "double";
  c14_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c14_info[8].fileTimeLo = 1286818722U;
  c14_info[8].fileTimeHi = 0U;
  c14_info[8].mFileTimeLo = 0U;
  c14_info[8].mFileTimeHi = 0U;
  c14_info[9].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotz.m";
  c14_info[9].name = "sin";
  c14_info[9].dominantType = "double";
  c14_info[9].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c14_info[9].fileTimeLo = 1343830386U;
  c14_info[9].fileTimeHi = 0U;
  c14_info[9].mFileTimeLo = 0U;
  c14_info[9].mFileTimeHi = 0U;
  c14_info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c14_info[10].name = "eml_scalar_sin";
  c14_info[10].dominantType = "double";
  c14_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c14_info[10].fileTimeLo = 1286818736U;
  c14_info[10].fileTimeHi = 0U;
  c14_info[10].mFileTimeLo = 0U;
  c14_info[10].mFileTimeHi = 0U;
  c14_info[11].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[11].name = "roty";
  c14_info[11].dominantType = "double";
  c14_info[11].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c14_info[11].fileTimeLo = 1495631764U;
  c14_info[11].fileTimeHi = 0U;
  c14_info[11].mFileTimeLo = 0U;
  c14_info[11].mFileTimeHi = 0U;
  c14_info[12].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c14_info[12].name = "cos";
  c14_info[12].dominantType = "double";
  c14_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c14_info[12].fileTimeLo = 1343830372U;
  c14_info[12].fileTimeHi = 0U;
  c14_info[12].mFileTimeLo = 0U;
  c14_info[12].mFileTimeHi = 0U;
  c14_info[13].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/roty.m";
  c14_info[13].name = "sin";
  c14_info[13].dominantType = "double";
  c14_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c14_info[13].fileTimeLo = 1343830386U;
  c14_info[13].fileTimeHi = 0U;
  c14_info[13].mFileTimeLo = 0U;
  c14_info[13].mFileTimeHi = 0U;
  c14_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[14].name = "eml_index_class";
  c14_info[14].dominantType = "";
  c14_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[14].fileTimeLo = 1323170578U;
  c14_info[14].fileTimeHi = 0U;
  c14_info[14].mFileTimeLo = 0U;
  c14_info[14].mFileTimeHi = 0U;
  c14_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[15].name = "eml_scalar_eg";
  c14_info[15].dominantType = "double";
  c14_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[15].fileTimeLo = 1286818796U;
  c14_info[15].fileTimeHi = 0U;
  c14_info[15].mFileTimeLo = 0U;
  c14_info[15].mFileTimeHi = 0U;
  c14_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[16].name = "eml_xgemm";
  c14_info[16].dominantType = "char";
  c14_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c14_info[16].fileTimeLo = 1299076772U;
  c14_info[16].fileTimeHi = 0U;
  c14_info[16].mFileTimeLo = 0U;
  c14_info[16].mFileTimeHi = 0U;
  c14_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c14_info[17].name = "eml_blas_inline";
  c14_info[17].dominantType = "";
  c14_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c14_info[17].fileTimeLo = 1299076768U;
  c14_info[17].fileTimeHi = 0U;
  c14_info[17].mFileTimeLo = 0U;
  c14_info[17].mFileTimeHi = 0U;
  c14_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c14_info[18].name = "mtimes";
  c14_info[18].dominantType = "double";
  c14_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[18].fileTimeLo = 1289519692U;
  c14_info[18].fileTimeHi = 0U;
  c14_info[18].mFileTimeLo = 0U;
  c14_info[18].mFileTimeHi = 0U;
  c14_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c14_info[19].name = "eml_index_class";
  c14_info[19].dominantType = "";
  c14_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[19].fileTimeLo = 1323170578U;
  c14_info[19].fileTimeHi = 0U;
  c14_info[19].mFileTimeLo = 0U;
  c14_info[19].mFileTimeHi = 0U;
  c14_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c14_info[20].name = "eml_scalar_eg";
  c14_info[20].dominantType = "double";
  c14_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[20].fileTimeLo = 1286818796U;
  c14_info[20].fileTimeHi = 0U;
  c14_info[20].mFileTimeLo = 0U;
  c14_info[20].mFileTimeHi = 0U;
  c14_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c14_info[21].name = "eml_refblas_xgemm";
  c14_info[21].dominantType = "char";
  c14_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c14_info[21].fileTimeLo = 1299076774U;
  c14_info[21].fileTimeHi = 0U;
  c14_info[21].mFileTimeLo = 0U;
  c14_info[21].mFileTimeHi = 0U;
  c14_info[22].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[22].name = "rotx";
  c14_info[22].dominantType = "double";
  c14_info[22].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c14_info[22].fileTimeLo = 1495631764U;
  c14_info[22].fileTimeHi = 0U;
  c14_info[22].mFileTimeLo = 0U;
  c14_info[22].mFileTimeHi = 0U;
  c14_info[23].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c14_info[23].name = "cos";
  c14_info[23].dominantType = "double";
  c14_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c14_info[23].fileTimeLo = 1343830372U;
  c14_info[23].fileTimeHi = 0U;
  c14_info[23].mFileTimeLo = 0U;
  c14_info[23].mFileTimeHi = 0U;
  c14_info[24].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rotx.m";
  c14_info[24].name = "sin";
  c14_info[24].dominantType = "double";
  c14_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c14_info[24].fileTimeLo = 1343830386U;
  c14_info[24].fileTimeHi = 0U;
  c14_info[24].mFileTimeLo = 0U;
  c14_info[24].mFileTimeHi = 0U;
  c14_info[25].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[25].name = "rollPitchYawFromRotation";
  c14_info[25].dominantType = "double";
  c14_info[25].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c14_info[25].fileTimeLo = 1495631764U;
  c14_info[25].fileTimeHi = 0U;
  c14_info[25].mFileTimeLo = 0U;
  c14_info[25].mFileTimeHi = 0U;
  c14_info[26].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c14_info[26].name = "asin";
  c14_info[26].dominantType = "double";
  c14_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c14_info[26].fileTimeLo = 1343830370U;
  c14_info[26].fileTimeHi = 0U;
  c14_info[26].mFileTimeLo = 0U;
  c14_info[26].mFileTimeHi = 0U;
  c14_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c14_info[27].name = "eml_error";
  c14_info[27].dominantType = "char";
  c14_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c14_info[27].fileTimeLo = 1343830358U;
  c14_info[27].fileTimeHi = 0U;
  c14_info[27].mFileTimeLo = 0U;
  c14_info[27].mFileTimeHi = 0U;
  c14_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c14_info[28].name = "eml_scalar_asin";
  c14_info[28].dominantType = "double";
  c14_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_asin.m";
  c14_info[28].fileTimeLo = 1343830376U;
  c14_info[28].fileTimeHi = 0U;
  c14_info[28].mFileTimeLo = 0U;
  c14_info[28].mFileTimeHi = 0U;
  c14_info[29].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/rollPitchYawFromRotation.m";
  c14_info[29].name = "atan2";
  c14_info[29].dominantType = "double";
  c14_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c14_info[29].fileTimeLo = 1343830372U;
  c14_info[29].fileTimeHi = 0U;
  c14_info[29].mFileTimeLo = 0U;
  c14_info[29].mFileTimeHi = 0U;
  c14_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c14_info[30].name = "eml_scalar_eg";
  c14_info[30].dominantType = "double";
  c14_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[30].fileTimeLo = 1286818796U;
  c14_info[30].fileTimeHi = 0U;
  c14_info[30].mFileTimeLo = 0U;
  c14_info[30].mFileTimeHi = 0U;
  c14_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c14_info[31].name = "eml_scalexp_alloc";
  c14_info[31].dominantType = "double";
  c14_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c14_info[31].fileTimeLo = 1352424860U;
  c14_info[31].fileTimeHi = 0U;
  c14_info[31].mFileTimeLo = 0U;
  c14_info[31].mFileTimeHi = 0U;
  c14_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  c14_info[32].name = "eml_scalar_atan2";
  c14_info[32].dominantType = "double";
  c14_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m";
  c14_info[32].fileTimeLo = 1286818720U;
  c14_info[32].fileTimeHi = 0U;
  c14_info[32].mFileTimeLo = 0U;
  c14_info[32].mFileTimeHi = 0U;
  c14_info[33].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[33].name = "correctIMU";
  c14_info[33].dominantType = "double";
  c14_info[33].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c14_info[33].fileTimeLo = 1495631764U;
  c14_info[33].fileTimeHi = 0U;
  c14_info[33].mFileTimeLo = 0U;
  c14_info[33].mFileTimeHi = 0U;
  c14_info[34].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c14_info[34].name = "mtimes";
  c14_info[34].dominantType = "double";
  c14_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[34].fileTimeLo = 1289519692U;
  c14_info[34].fileTimeHi = 0U;
  c14_info[34].mFileTimeLo = 0U;
  c14_info[34].mFileTimeHi = 0U;
  c14_info[35].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c14_info[35].name = "mrdivide";
  c14_info[35].dominantType = "double";
  c14_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c14_info[35].fileTimeLo = 1357951548U;
  c14_info[35].fileTimeHi = 0U;
  c14_info[35].mFileTimeLo = 1319729966U;
  c14_info[35].mFileTimeHi = 0U;
  c14_info[36].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/correctIMU.m";
  c14_info[36].name = "evalDHMatrix";
  c14_info[36].dominantType = "double";
  c14_info[36].resolved =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c14_info[36].fileTimeLo = 1495631765U;
  c14_info[36].fileTimeHi = 0U;
  c14_info[36].mFileTimeLo = 0U;
  c14_info[36].mFileTimeHi = 0U;
  c14_info[37].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c14_info[37].name = "cos";
  c14_info[37].dominantType = "double";
  c14_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c14_info[37].fileTimeLo = 1343830372U;
  c14_info[37].fileTimeHi = 0U;
  c14_info[37].mFileTimeLo = 0U;
  c14_info[37].mFileTimeHi = 0U;
  c14_info[38].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c14_info[38].name = "sin";
  c14_info[38].dominantType = "double";
  c14_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c14_info[38].fileTimeLo = 1343830386U;
  c14_info[38].fileTimeHi = 0U;
  c14_info[38].mFileTimeLo = 0U;
  c14_info[38].mFileTimeHi = 0U;
  c14_info[39].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/evalDHMatrix.m";
  c14_info[39].name = "mtimes";
  c14_info[39].dominantType = "double";
  c14_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[39].fileTimeLo = 1289519692U;
  c14_info[39].fileTimeHi = 0U;
  c14_info[39].mFileTimeLo = 0U;
  c14_info[39].mFileTimeHi = 0U;
  c14_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c14_info[40].name = "mldivide";
  c14_info[40].dominantType = "double";
  c14_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c14_info[40].fileTimeLo = 1357951548U;
  c14_info[40].fileTimeHi = 0U;
  c14_info[40].mFileTimeLo = 1319729966U;
  c14_info[40].mFileTimeHi = 0U;
  c14_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c14_info[41].name = "eml_lusolve";
  c14_info[41].dominantType = "double";
  c14_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c14_info[41].fileTimeLo = 1309451196U;
  c14_info[41].fileTimeHi = 0U;
  c14_info[41].mFileTimeLo = 0U;
  c14_info[41].mFileTimeHi = 0U;
  c14_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c14_info[42].name = "eml_index_class";
  c14_info[42].dominantType = "";
  c14_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[42].fileTimeLo = 1323170578U;
  c14_info[42].fileTimeHi = 0U;
  c14_info[42].mFileTimeLo = 0U;
  c14_info[42].mFileTimeHi = 0U;
  c14_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c14_info[43].name = "eml_index_class";
  c14_info[43].dominantType = "";
  c14_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[43].fileTimeLo = 1323170578U;
  c14_info[43].fileTimeHi = 0U;
  c14_info[43].mFileTimeLo = 0U;
  c14_info[43].mFileTimeHi = 0U;
  c14_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c14_info[44].name = "eml_xgetrf";
  c14_info[44].dominantType = "double";
  c14_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c14_info[44].fileTimeLo = 1286818806U;
  c14_info[44].fileTimeHi = 0U;
  c14_info[44].mFileTimeLo = 0U;
  c14_info[44].mFileTimeHi = 0U;
  c14_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c14_info[45].name = "eml_lapack_xgetrf";
  c14_info[45].dominantType = "double";
  c14_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c14_info[45].fileTimeLo = 1286818810U;
  c14_info[45].fileTimeHi = 0U;
  c14_info[45].mFileTimeLo = 0U;
  c14_info[45].mFileTimeHi = 0U;
  c14_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c14_info[46].name = "eml_matlab_zgetrf";
  c14_info[46].dominantType = "double";
  c14_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[46].fileTimeLo = 1302688994U;
  c14_info[46].fileTimeHi = 0U;
  c14_info[46].mFileTimeLo = 0U;
  c14_info[46].mFileTimeHi = 0U;
  c14_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[47].name = "realmin";
  c14_info[47].dominantType = "char";
  c14_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c14_info[47].fileTimeLo = 1307651242U;
  c14_info[47].fileTimeHi = 0U;
  c14_info[47].mFileTimeLo = 0U;
  c14_info[47].mFileTimeHi = 0U;
  c14_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c14_info[48].name = "eml_realmin";
  c14_info[48].dominantType = "char";
  c14_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c14_info[48].fileTimeLo = 1307651244U;
  c14_info[48].fileTimeHi = 0U;
  c14_info[48].mFileTimeLo = 0U;
  c14_info[48].mFileTimeHi = 0U;
  c14_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c14_info[49].name = "eml_float_model";
  c14_info[49].dominantType = "char";
  c14_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c14_info[49].fileTimeLo = 1326727996U;
  c14_info[49].fileTimeHi = 0U;
  c14_info[49].mFileTimeLo = 0U;
  c14_info[49].mFileTimeHi = 0U;
  c14_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[50].name = "eps";
  c14_info[50].dominantType = "char";
  c14_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c14_info[50].fileTimeLo = 1326727996U;
  c14_info[50].fileTimeHi = 0U;
  c14_info[50].mFileTimeLo = 0U;
  c14_info[50].mFileTimeHi = 0U;
  c14_info[51].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c14_info[51].name = "eml_is_float_class";
  c14_info[51].dominantType = "char";
  c14_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c14_info[51].fileTimeLo = 1286818782U;
  c14_info[51].fileTimeHi = 0U;
  c14_info[51].mFileTimeLo = 0U;
  c14_info[51].mFileTimeHi = 0U;
  c14_info[52].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c14_info[52].name = "eml_eps";
  c14_info[52].dominantType = "char";
  c14_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c14_info[52].fileTimeLo = 1326727996U;
  c14_info[52].fileTimeHi = 0U;
  c14_info[52].mFileTimeLo = 0U;
  c14_info[52].mFileTimeHi = 0U;
  c14_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c14_info[53].name = "eml_float_model";
  c14_info[53].dominantType = "char";
  c14_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c14_info[53].fileTimeLo = 1326727996U;
  c14_info[53].fileTimeHi = 0U;
  c14_info[53].mFileTimeLo = 0U;
  c14_info[53].mFileTimeHi = 0U;
  c14_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[54].name = "min";
  c14_info[54].dominantType = "coder.internal.indexInt";
  c14_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c14_info[54].fileTimeLo = 1311255318U;
  c14_info[54].fileTimeHi = 0U;
  c14_info[54].mFileTimeLo = 0U;
  c14_info[54].mFileTimeHi = 0U;
  c14_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c14_info[55].name = "eml_min_or_max";
  c14_info[55].dominantType = "char";
  c14_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c14_info[55].fileTimeLo = 1334071490U;
  c14_info[55].fileTimeHi = 0U;
  c14_info[55].mFileTimeLo = 0U;
  c14_info[55].mFileTimeHi = 0U;
  c14_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c14_info[56].name = "eml_scalar_eg";
  c14_info[56].dominantType = "coder.internal.indexInt";
  c14_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[56].fileTimeLo = 1286818796U;
  c14_info[56].fileTimeHi = 0U;
  c14_info[56].mFileTimeLo = 0U;
  c14_info[56].mFileTimeHi = 0U;
  c14_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c14_info[57].name = "eml_scalexp_alloc";
  c14_info[57].dominantType = "coder.internal.indexInt";
  c14_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c14_info[57].fileTimeLo = 1352424860U;
  c14_info[57].fileTimeHi = 0U;
  c14_info[57].mFileTimeLo = 0U;
  c14_info[57].mFileTimeHi = 0U;
  c14_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c14_info[58].name = "eml_index_class";
  c14_info[58].dominantType = "";
  c14_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[58].fileTimeLo = 1323170578U;
  c14_info[58].fileTimeHi = 0U;
  c14_info[58].mFileTimeLo = 0U;
  c14_info[58].mFileTimeHi = 0U;
  c14_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c14_info[59].name = "eml_scalar_eg";
  c14_info[59].dominantType = "coder.internal.indexInt";
  c14_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[59].fileTimeLo = 1286818796U;
  c14_info[59].fileTimeHi = 0U;
  c14_info[59].mFileTimeLo = 0U;
  c14_info[59].mFileTimeHi = 0U;
  c14_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[60].name = "colon";
  c14_info[60].dominantType = "double";
  c14_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c14_info[60].fileTimeLo = 1348191928U;
  c14_info[60].fileTimeHi = 0U;
  c14_info[60].mFileTimeLo = 0U;
  c14_info[60].mFileTimeHi = 0U;
  c14_info[61].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c14_info[61].name = "colon";
  c14_info[61].dominantType = "double";
  c14_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c14_info[61].fileTimeLo = 1348191928U;
  c14_info[61].fileTimeHi = 0U;
  c14_info[61].mFileTimeLo = 0U;
  c14_info[61].mFileTimeHi = 0U;
  c14_info[62].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c14_info[62].name = "floor";
  c14_info[62].dominantType = "double";
  c14_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c14_info[62].fileTimeLo = 1343830380U;
  c14_info[62].fileTimeHi = 0U;
  c14_info[62].mFileTimeLo = 0U;
  c14_info[62].mFileTimeHi = 0U;
  c14_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c14_info[63].name = "eml_scalar_floor";
  c14_info[63].dominantType = "double";
  c14_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c14_info[63].fileTimeLo = 1286818726U;
  c14_info[63].fileTimeHi = 0U;
  c14_info[63].mFileTimeLo = 0U;
  c14_info[63].mFileTimeHi = 0U;
}

static void c14_b_info_helper(c14_ResolvedFunctionInfo c14_info[147])
{
  c14_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c14_info[64].name = "intmin";
  c14_info[64].dominantType = "char";
  c14_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c14_info[64].fileTimeLo = 1311255318U;
  c14_info[64].fileTimeHi = 0U;
  c14_info[64].mFileTimeLo = 0U;
  c14_info[64].mFileTimeHi = 0U;
  c14_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c14_info[65].name = "intmax";
  c14_info[65].dominantType = "char";
  c14_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c14_info[65].fileTimeLo = 1311255316U;
  c14_info[65].fileTimeHi = 0U;
  c14_info[65].mFileTimeLo = 0U;
  c14_info[65].mFileTimeHi = 0U;
  c14_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c14_info[66].name = "intmin";
  c14_info[66].dominantType = "char";
  c14_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c14_info[66].fileTimeLo = 1311255318U;
  c14_info[66].fileTimeHi = 0U;
  c14_info[66].mFileTimeLo = 0U;
  c14_info[66].mFileTimeHi = 0U;
  c14_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c14_info[67].name = "intmax";
  c14_info[67].dominantType = "char";
  c14_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c14_info[67].fileTimeLo = 1311255316U;
  c14_info[67].fileTimeHi = 0U;
  c14_info[67].mFileTimeLo = 0U;
  c14_info[67].mFileTimeHi = 0U;
  c14_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c14_info[68].name = "eml_isa_uint";
  c14_info[68].dominantType = "coder.internal.indexInt";
  c14_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c14_info[68].fileTimeLo = 1286818784U;
  c14_info[68].fileTimeHi = 0U;
  c14_info[68].mFileTimeLo = 0U;
  c14_info[68].mFileTimeHi = 0U;
  c14_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c14_info[69].name = "eml_unsigned_class";
  c14_info[69].dominantType = "char";
  c14_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c14_info[69].fileTimeLo = 1323170580U;
  c14_info[69].fileTimeHi = 0U;
  c14_info[69].mFileTimeLo = 0U;
  c14_info[69].mFileTimeHi = 0U;
  c14_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c14_info[70].name = "eml_index_class";
  c14_info[70].dominantType = "";
  c14_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[70].fileTimeLo = 1323170578U;
  c14_info[70].fileTimeHi = 0U;
  c14_info[70].mFileTimeLo = 0U;
  c14_info[70].mFileTimeHi = 0U;
  c14_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c14_info[71].name = "eml_index_class";
  c14_info[71].dominantType = "";
  c14_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[71].fileTimeLo = 1323170578U;
  c14_info[71].fileTimeHi = 0U;
  c14_info[71].mFileTimeLo = 0U;
  c14_info[71].mFileTimeHi = 0U;
  c14_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c14_info[72].name = "intmax";
  c14_info[72].dominantType = "char";
  c14_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c14_info[72].fileTimeLo = 1311255316U;
  c14_info[72].fileTimeHi = 0U;
  c14_info[72].mFileTimeLo = 0U;
  c14_info[72].mFileTimeHi = 0U;
  c14_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c14_info[73].name = "eml_isa_uint";
  c14_info[73].dominantType = "coder.internal.indexInt";
  c14_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c14_info[73].fileTimeLo = 1286818784U;
  c14_info[73].fileTimeHi = 0U;
  c14_info[73].mFileTimeLo = 0U;
  c14_info[73].mFileTimeHi = 0U;
  c14_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c14_info[74].name = "eml_index_plus";
  c14_info[74].dominantType = "double";
  c14_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[74].fileTimeLo = 1286818778U;
  c14_info[74].fileTimeHi = 0U;
  c14_info[74].mFileTimeLo = 0U;
  c14_info[74].mFileTimeHi = 0U;
  c14_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[75].name = "eml_index_class";
  c14_info[75].dominantType = "";
  c14_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[75].fileTimeLo = 1323170578U;
  c14_info[75].fileTimeHi = 0U;
  c14_info[75].mFileTimeLo = 0U;
  c14_info[75].mFileTimeHi = 0U;
  c14_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c14_info[76].name = "eml_int_forloop_overflow_check";
  c14_info[76].dominantType = "";
  c14_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[76].fileTimeLo = 1346510340U;
  c14_info[76].fileTimeHi = 0U;
  c14_info[76].mFileTimeLo = 0U;
  c14_info[76].mFileTimeHi = 0U;
  c14_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c14_info[77].name = "intmax";
  c14_info[77].dominantType = "char";
  c14_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c14_info[77].fileTimeLo = 1311255316U;
  c14_info[77].fileTimeHi = 0U;
  c14_info[77].mFileTimeLo = 0U;
  c14_info[77].mFileTimeHi = 0U;
  c14_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[78].name = "eml_index_class";
  c14_info[78].dominantType = "";
  c14_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[78].fileTimeLo = 1323170578U;
  c14_info[78].fileTimeHi = 0U;
  c14_info[78].mFileTimeLo = 0U;
  c14_info[78].mFileTimeHi = 0U;
  c14_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[79].name = "eml_index_plus";
  c14_info[79].dominantType = "double";
  c14_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[79].fileTimeLo = 1286818778U;
  c14_info[79].fileTimeHi = 0U;
  c14_info[79].mFileTimeLo = 0U;
  c14_info[79].mFileTimeHi = 0U;
  c14_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[80].name = "eml_int_forloop_overflow_check";
  c14_info[80].dominantType = "";
  c14_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[80].fileTimeLo = 1346510340U;
  c14_info[80].fileTimeHi = 0U;
  c14_info[80].mFileTimeLo = 0U;
  c14_info[80].mFileTimeHi = 0U;
  c14_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[81].name = "eml_index_minus";
  c14_info[81].dominantType = "double";
  c14_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c14_info[81].fileTimeLo = 1286818778U;
  c14_info[81].fileTimeHi = 0U;
  c14_info[81].mFileTimeLo = 0U;
  c14_info[81].mFileTimeHi = 0U;
  c14_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c14_info[82].name = "eml_index_class";
  c14_info[82].dominantType = "";
  c14_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[82].fileTimeLo = 1323170578U;
  c14_info[82].fileTimeHi = 0U;
  c14_info[82].mFileTimeLo = 0U;
  c14_info[82].mFileTimeHi = 0U;
  c14_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[83].name = "eml_index_minus";
  c14_info[83].dominantType = "coder.internal.indexInt";
  c14_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c14_info[83].fileTimeLo = 1286818778U;
  c14_info[83].fileTimeHi = 0U;
  c14_info[83].mFileTimeLo = 0U;
  c14_info[83].mFileTimeHi = 0U;
  c14_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[84].name = "eml_index_times";
  c14_info[84].dominantType = "coder.internal.indexInt";
  c14_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c14_info[84].fileTimeLo = 1286818780U;
  c14_info[84].fileTimeHi = 0U;
  c14_info[84].mFileTimeLo = 0U;
  c14_info[84].mFileTimeHi = 0U;
  c14_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c14_info[85].name = "eml_index_class";
  c14_info[85].dominantType = "";
  c14_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[85].fileTimeLo = 1323170578U;
  c14_info[85].fileTimeHi = 0U;
  c14_info[85].mFileTimeLo = 0U;
  c14_info[85].mFileTimeHi = 0U;
  c14_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[86].name = "eml_index_plus";
  c14_info[86].dominantType = "coder.internal.indexInt";
  c14_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[86].fileTimeLo = 1286818778U;
  c14_info[86].fileTimeHi = 0U;
  c14_info[86].mFileTimeLo = 0U;
  c14_info[86].mFileTimeHi = 0U;
  c14_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[87].name = "eml_ixamax";
  c14_info[87].dominantType = "double";
  c14_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c14_info[87].fileTimeLo = 1299076770U;
  c14_info[87].fileTimeHi = 0U;
  c14_info[87].mFileTimeLo = 0U;
  c14_info[87].mFileTimeHi = 0U;
  c14_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c14_info[88].name = "eml_blas_inline";
  c14_info[88].dominantType = "";
  c14_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c14_info[88].fileTimeLo = 1299076768U;
  c14_info[88].fileTimeHi = 0U;
  c14_info[88].mFileTimeLo = 0U;
  c14_info[88].mFileTimeHi = 0U;
  c14_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c14_info[89].name = "length";
  c14_info[89].dominantType = "double";
  c14_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c14_info[89].fileTimeLo = 1303146206U;
  c14_info[89].fileTimeHi = 0U;
  c14_info[89].mFileTimeLo = 0U;
  c14_info[89].mFileTimeHi = 0U;
  c14_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c14_info[90].name = "eml_index_class";
  c14_info[90].dominantType = "";
  c14_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[90].fileTimeLo = 1323170578U;
  c14_info[90].fileTimeHi = 0U;
  c14_info[90].mFileTimeLo = 0U;
  c14_info[90].mFileTimeHi = 0U;
  c14_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c14_info[91].name = "eml_index_class";
  c14_info[91].dominantType = "";
  c14_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[91].fileTimeLo = 1323170578U;
  c14_info[91].fileTimeHi = 0U;
  c14_info[91].mFileTimeLo = 0U;
  c14_info[91].mFileTimeHi = 0U;
  c14_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c14_info[92].name = "eml_refblas_ixamax";
  c14_info[92].dominantType = "double";
  c14_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c14_info[92].fileTimeLo = 1299076770U;
  c14_info[92].fileTimeHi = 0U;
  c14_info[92].mFileTimeLo = 0U;
  c14_info[92].mFileTimeHi = 0U;
  c14_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c14_info[93].name = "eml_index_class";
  c14_info[93].dominantType = "";
  c14_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[93].fileTimeLo = 1323170578U;
  c14_info[93].fileTimeHi = 0U;
  c14_info[93].mFileTimeLo = 0U;
  c14_info[93].mFileTimeHi = 0U;
  c14_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c14_info[94].name = "eml_xcabs1";
  c14_info[94].dominantType = "double";
  c14_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c14_info[94].fileTimeLo = 1286818706U;
  c14_info[94].fileTimeHi = 0U;
  c14_info[94].mFileTimeLo = 0U;
  c14_info[94].mFileTimeHi = 0U;
  c14_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c14_info[95].name = "abs";
  c14_info[95].dominantType = "double";
  c14_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c14_info[95].fileTimeLo = 1343830366U;
  c14_info[95].fileTimeHi = 0U;
  c14_info[95].mFileTimeLo = 0U;
  c14_info[95].mFileTimeHi = 0U;
  c14_info[96].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c14_info[96].name = "eml_scalar_abs";
  c14_info[96].dominantType = "double";
  c14_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c14_info[96].fileTimeLo = 1286818712U;
  c14_info[96].fileTimeHi = 0U;
  c14_info[96].mFileTimeLo = 0U;
  c14_info[96].mFileTimeHi = 0U;
  c14_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c14_info[97].name = "eml_int_forloop_overflow_check";
  c14_info[97].dominantType = "";
  c14_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[97].fileTimeLo = 1346510340U;
  c14_info[97].fileTimeHi = 0U;
  c14_info[97].mFileTimeLo = 0U;
  c14_info[97].mFileTimeHi = 0U;
  c14_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c14_info[98].name = "eml_index_plus";
  c14_info[98].dominantType = "coder.internal.indexInt";
  c14_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[98].fileTimeLo = 1286818778U;
  c14_info[98].fileTimeHi = 0U;
  c14_info[98].mFileTimeLo = 0U;
  c14_info[98].mFileTimeHi = 0U;
  c14_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[99].name = "eml_xswap";
  c14_info[99].dominantType = "double";
  c14_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c14_info[99].fileTimeLo = 1299076778U;
  c14_info[99].fileTimeHi = 0U;
  c14_info[99].mFileTimeLo = 0U;
  c14_info[99].mFileTimeHi = 0U;
  c14_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c14_info[100].name = "eml_blas_inline";
  c14_info[100].dominantType = "";
  c14_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c14_info[100].fileTimeLo = 1299076768U;
  c14_info[100].fileTimeHi = 0U;
  c14_info[100].mFileTimeLo = 0U;
  c14_info[100].mFileTimeHi = 0U;
  c14_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c14_info[101].name = "eml_index_class";
  c14_info[101].dominantType = "";
  c14_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[101].fileTimeLo = 1323170578U;
  c14_info[101].fileTimeHi = 0U;
  c14_info[101].mFileTimeLo = 0U;
  c14_info[101].mFileTimeHi = 0U;
  c14_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c14_info[102].name = "eml_refblas_xswap";
  c14_info[102].dominantType = "double";
  c14_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c14_info[102].fileTimeLo = 1299076786U;
  c14_info[102].fileTimeHi = 0U;
  c14_info[102].mFileTimeLo = 0U;
  c14_info[102].mFileTimeHi = 0U;
  c14_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c14_info[103].name = "eml_index_class";
  c14_info[103].dominantType = "";
  c14_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[103].fileTimeLo = 1323170578U;
  c14_info[103].fileTimeHi = 0U;
  c14_info[103].mFileTimeLo = 0U;
  c14_info[103].mFileTimeHi = 0U;
  c14_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c14_info[104].name = "abs";
  c14_info[104].dominantType = "coder.internal.indexInt";
  c14_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c14_info[104].fileTimeLo = 1343830366U;
  c14_info[104].fileTimeHi = 0U;
  c14_info[104].mFileTimeLo = 0U;
  c14_info[104].mFileTimeHi = 0U;
  c14_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c14_info[105].name = "eml_scalar_abs";
  c14_info[105].dominantType = "coder.internal.indexInt";
  c14_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c14_info[105].fileTimeLo = 1286818712U;
  c14_info[105].fileTimeHi = 0U;
  c14_info[105].mFileTimeLo = 0U;
  c14_info[105].mFileTimeHi = 0U;
  c14_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c14_info[106].name = "eml_int_forloop_overflow_check";
  c14_info[106].dominantType = "";
  c14_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[106].fileTimeLo = 1346510340U;
  c14_info[106].fileTimeHi = 0U;
  c14_info[106].mFileTimeLo = 0U;
  c14_info[106].mFileTimeHi = 0U;
  c14_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c14_info[107].name = "eml_index_plus";
  c14_info[107].dominantType = "coder.internal.indexInt";
  c14_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[107].fileTimeLo = 1286818778U;
  c14_info[107].fileTimeHi = 0U;
  c14_info[107].mFileTimeLo = 0U;
  c14_info[107].mFileTimeHi = 0U;
  c14_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[108].name = "eml_div";
  c14_info[108].dominantType = "double";
  c14_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c14_info[108].fileTimeLo = 1313347810U;
  c14_info[108].fileTimeHi = 0U;
  c14_info[108].mFileTimeLo = 0U;
  c14_info[108].mFileTimeHi = 0U;
  c14_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c14_info[109].name = "eml_xgeru";
  c14_info[109].dominantType = "double";
  c14_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c14_info[109].fileTimeLo = 1299076774U;
  c14_info[109].fileTimeHi = 0U;
  c14_info[109].mFileTimeLo = 0U;
  c14_info[109].mFileTimeHi = 0U;
  c14_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c14_info[110].name = "eml_blas_inline";
  c14_info[110].dominantType = "";
  c14_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c14_info[110].fileTimeLo = 1299076768U;
  c14_info[110].fileTimeHi = 0U;
  c14_info[110].mFileTimeLo = 0U;
  c14_info[110].mFileTimeHi = 0U;
  c14_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c14_info[111].name = "eml_xger";
  c14_info[111].dominantType = "double";
  c14_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c14_info[111].fileTimeLo = 1299076774U;
  c14_info[111].fileTimeHi = 0U;
  c14_info[111].mFileTimeLo = 0U;
  c14_info[111].mFileTimeHi = 0U;
  c14_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c14_info[112].name = "eml_blas_inline";
  c14_info[112].dominantType = "";
  c14_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c14_info[112].fileTimeLo = 1299076768U;
  c14_info[112].fileTimeHi = 0U;
  c14_info[112].mFileTimeLo = 0U;
  c14_info[112].mFileTimeHi = 0U;
  c14_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c14_info[113].name = "intmax";
  c14_info[113].dominantType = "char";
  c14_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c14_info[113].fileTimeLo = 1311255316U;
  c14_info[113].fileTimeHi = 0U;
  c14_info[113].mFileTimeLo = 0U;
  c14_info[113].mFileTimeHi = 0U;
  c14_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c14_info[114].name = "min";
  c14_info[114].dominantType = "double";
  c14_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c14_info[114].fileTimeLo = 1311255318U;
  c14_info[114].fileTimeHi = 0U;
  c14_info[114].mFileTimeLo = 0U;
  c14_info[114].mFileTimeHi = 0U;
  c14_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c14_info[115].name = "eml_scalar_eg";
  c14_info[115].dominantType = "double";
  c14_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[115].fileTimeLo = 1286818796U;
  c14_info[115].fileTimeHi = 0U;
  c14_info[115].mFileTimeLo = 0U;
  c14_info[115].mFileTimeHi = 0U;
  c14_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c14_info[116].name = "eml_scalexp_alloc";
  c14_info[116].dominantType = "double";
  c14_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c14_info[116].fileTimeLo = 1352424860U;
  c14_info[116].fileTimeHi = 0U;
  c14_info[116].mFileTimeLo = 0U;
  c14_info[116].mFileTimeHi = 0U;
  c14_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c14_info[117].name = "eml_scalar_eg";
  c14_info[117].dominantType = "double";
  c14_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[117].fileTimeLo = 1286818796U;
  c14_info[117].fileTimeHi = 0U;
  c14_info[117].mFileTimeLo = 0U;
  c14_info[117].mFileTimeHi = 0U;
  c14_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c14_info[118].name = "mtimes";
  c14_info[118].dominantType = "double";
  c14_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[118].fileTimeLo = 1289519692U;
  c14_info[118].fileTimeHi = 0U;
  c14_info[118].mFileTimeLo = 0U;
  c14_info[118].mFileTimeHi = 0U;
  c14_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c14_info[119].name = "eml_index_class";
  c14_info[119].dominantType = "";
  c14_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[119].fileTimeLo = 1323170578U;
  c14_info[119].fileTimeHi = 0U;
  c14_info[119].mFileTimeLo = 0U;
  c14_info[119].mFileTimeHi = 0U;
  c14_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c14_info[120].name = "eml_refblas_xger";
  c14_info[120].dominantType = "double";
  c14_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c14_info[120].fileTimeLo = 1299076776U;
  c14_info[120].fileTimeHi = 0U;
  c14_info[120].mFileTimeLo = 0U;
  c14_info[120].mFileTimeHi = 0U;
  c14_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c14_info[121].name = "eml_refblas_xgerx";
  c14_info[121].dominantType = "char";
  c14_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c14_info[121].fileTimeLo = 1299076778U;
  c14_info[121].fileTimeHi = 0U;
  c14_info[121].mFileTimeLo = 0U;
  c14_info[121].mFileTimeHi = 0U;
  c14_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c14_info[122].name = "eml_index_class";
  c14_info[122].dominantType = "";
  c14_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[122].fileTimeLo = 1323170578U;
  c14_info[122].fileTimeHi = 0U;
  c14_info[122].mFileTimeLo = 0U;
  c14_info[122].mFileTimeHi = 0U;
  c14_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c14_info[123].name = "abs";
  c14_info[123].dominantType = "coder.internal.indexInt";
  c14_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c14_info[123].fileTimeLo = 1343830366U;
  c14_info[123].fileTimeHi = 0U;
  c14_info[123].mFileTimeLo = 0U;
  c14_info[123].mFileTimeHi = 0U;
  c14_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c14_info[124].name = "eml_index_minus";
  c14_info[124].dominantType = "double";
  c14_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c14_info[124].fileTimeLo = 1286818778U;
  c14_info[124].fileTimeHi = 0U;
  c14_info[124].mFileTimeLo = 0U;
  c14_info[124].mFileTimeHi = 0U;
  c14_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c14_info[125].name = "eml_int_forloop_overflow_check";
  c14_info[125].dominantType = "";
  c14_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[125].fileTimeLo = 1346510340U;
  c14_info[125].fileTimeHi = 0U;
  c14_info[125].mFileTimeLo = 0U;
  c14_info[125].mFileTimeHi = 0U;
  c14_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c14_info[126].name = "eml_index_plus";
  c14_info[126].dominantType = "double";
  c14_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[126].fileTimeLo = 1286818778U;
  c14_info[126].fileTimeHi = 0U;
  c14_info[126].mFileTimeLo = 0U;
  c14_info[126].mFileTimeHi = 0U;
  c14_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c14_info[127].name = "eml_index_plus";
  c14_info[127].dominantType = "coder.internal.indexInt";
  c14_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[127].fileTimeLo = 1286818778U;
  c14_info[127].fileTimeHi = 0U;
  c14_info[127].mFileTimeLo = 0U;
  c14_info[127].mFileTimeHi = 0U;
}

static void c14_c_info_helper(c14_ResolvedFunctionInfo c14_info[147])
{
  c14_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c14_info[128].name = "eml_scalar_eg";
  c14_info[128].dominantType = "double";
  c14_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[128].fileTimeLo = 1286818796U;
  c14_info[128].fileTimeHi = 0U;
  c14_info[128].mFileTimeLo = 0U;
  c14_info[128].mFileTimeHi = 0U;
  c14_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c14_info[129].name = "eml_int_forloop_overflow_check";
  c14_info[129].dominantType = "";
  c14_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[129].fileTimeLo = 1346510340U;
  c14_info[129].fileTimeHi = 0U;
  c14_info[129].mFileTimeLo = 0U;
  c14_info[129].mFileTimeHi = 0U;
  c14_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c14_info[130].name = "eml_xtrsm";
  c14_info[130].dominantType = "char";
  c14_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c14_info[130].fileTimeLo = 1299076778U;
  c14_info[130].fileTimeHi = 0U;
  c14_info[130].mFileTimeLo = 0U;
  c14_info[130].mFileTimeHi = 0U;
  c14_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c14_info[131].name = "eml_blas_inline";
  c14_info[131].dominantType = "";
  c14_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c14_info[131].fileTimeLo = 1299076768U;
  c14_info[131].fileTimeHi = 0U;
  c14_info[131].mFileTimeLo = 0U;
  c14_info[131].mFileTimeHi = 0U;
  c14_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c14_info[132].name = "mtimes";
  c14_info[132].dominantType = "double";
  c14_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[132].fileTimeLo = 1289519692U;
  c14_info[132].fileTimeHi = 0U;
  c14_info[132].mFileTimeLo = 0U;
  c14_info[132].mFileTimeHi = 0U;
  c14_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c14_info[133].name = "eml_index_class";
  c14_info[133].dominantType = "";
  c14_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[133].fileTimeLo = 1323170578U;
  c14_info[133].fileTimeHi = 0U;
  c14_info[133].mFileTimeLo = 0U;
  c14_info[133].mFileTimeHi = 0U;
  c14_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c14_info[134].name = "eml_scalar_eg";
  c14_info[134].dominantType = "double";
  c14_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[134].fileTimeLo = 1286818796U;
  c14_info[134].fileTimeHi = 0U;
  c14_info[134].mFileTimeLo = 0U;
  c14_info[134].mFileTimeHi = 0U;
  c14_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c14_info[135].name = "eml_refblas_xtrsm";
  c14_info[135].dominantType = "char";
  c14_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[135].fileTimeLo = 1299076786U;
  c14_info[135].fileTimeHi = 0U;
  c14_info[135].mFileTimeLo = 0U;
  c14_info[135].mFileTimeHi = 0U;
  c14_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[136].name = "eml_scalar_eg";
  c14_info[136].dominantType = "double";
  c14_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[136].fileTimeLo = 1286818796U;
  c14_info[136].fileTimeHi = 0U;
  c14_info[136].mFileTimeLo = 0U;
  c14_info[136].mFileTimeHi = 0U;
  c14_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[137].name = "eml_index_minus";
  c14_info[137].dominantType = "double";
  c14_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c14_info[137].fileTimeLo = 1286818778U;
  c14_info[137].fileTimeHi = 0U;
  c14_info[137].mFileTimeLo = 0U;
  c14_info[137].mFileTimeHi = 0U;
  c14_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[138].name = "eml_index_class";
  c14_info[138].dominantType = "";
  c14_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[138].fileTimeLo = 1323170578U;
  c14_info[138].fileTimeHi = 0U;
  c14_info[138].mFileTimeLo = 0U;
  c14_info[138].mFileTimeHi = 0U;
  c14_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[139].name = "eml_int_forloop_overflow_check";
  c14_info[139].dominantType = "";
  c14_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[139].fileTimeLo = 1346510340U;
  c14_info[139].fileTimeHi = 0U;
  c14_info[139].mFileTimeLo = 0U;
  c14_info[139].mFileTimeHi = 0U;
  c14_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[140].name = "eml_index_times";
  c14_info[140].dominantType = "coder.internal.indexInt";
  c14_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c14_info[140].fileTimeLo = 1286818780U;
  c14_info[140].fileTimeHi = 0U;
  c14_info[140].mFileTimeLo = 0U;
  c14_info[140].mFileTimeHi = 0U;
  c14_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[141].name = "eml_index_plus";
  c14_info[141].dominantType = "coder.internal.indexInt";
  c14_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[141].fileTimeLo = 1286818778U;
  c14_info[141].fileTimeHi = 0U;
  c14_info[141].mFileTimeLo = 0U;
  c14_info[141].mFileTimeHi = 0U;
  c14_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[142].name = "eml_index_plus";
  c14_info[142].dominantType = "double";
  c14_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[142].fileTimeLo = 1286818778U;
  c14_info[142].fileTimeHi = 0U;
  c14_info[142].mFileTimeLo = 0U;
  c14_info[142].mFileTimeHi = 0U;
  c14_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c14_info[143].name = "intmin";
  c14_info[143].dominantType = "char";
  c14_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c14_info[143].fileTimeLo = 1311255318U;
  c14_info[143].fileTimeHi = 0U;
  c14_info[143].mFileTimeLo = 0U;
  c14_info[143].mFileTimeHi = 0U;
  c14_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c14_info[144].name = "eml_div";
  c14_info[144].dominantType = "double";
  c14_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c14_info[144].fileTimeLo = 1313347810U;
  c14_info[144].fileTimeHi = 0U;
  c14_info[144].mFileTimeLo = 0U;
  c14_info[144].mFileTimeHi = 0U;
  c14_info[145].context =
    "[E]/home/valerio/git/learnOptimWBC/matlab/Common/TB_StandUp/utilityMatlabFunctions/fromBaseToWorldWithImu.m";
  c14_info[145].name = "mldivide";
  c14_info[145].dominantType = "double";
  c14_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c14_info[145].fileTimeLo = 1357951548U;
  c14_info[145].fileTimeHi = 0U;
  c14_info[145].mFileTimeLo = 1319729966U;
  c14_info[145].mFileTimeHi = 0U;
  c14_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c14_info[146].name = "eml_warning";
  c14_info[146].dominantType = "char";
  c14_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c14_info[146].fileTimeLo = 1286818802U;
  c14_info[146].fileTimeHi = 0U;
  c14_info[146].mFileTimeLo = 0U;
  c14_info[146].mFileTimeHi = 0U;
}

static void c14_eml_scalar_eg(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c14_eml_error(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c14_i251;
  static char_T c14_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c14_u[30];
  const mxArray *c14_y = NULL;
  int32_T c14_i252;
  static char_T c14_cv1[4] = { 'a', 's', 'i', 'n' };

  char_T c14_b_u[4];
  const mxArray *c14_b_y = NULL;
  for (c14_i251 = 0; c14_i251 < 30; c14_i251++) {
    c14_u[c14_i251] = c14_cv0[c14_i251];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 10, 0U, 1U, 0U, 2, 1, 30),
                FALSE);
  for (c14_i252 = 0; c14_i252 < 4; c14_i252++) {
    c14_b_u[c14_i252] = c14_cv1[c14_i252];
  }

  c14_b_y = NULL;
  sf_mex_assign(&c14_b_y, sf_mex_create("y", c14_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c14_y, 14, c14_b_y));
}

static real_T c14_atan2(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
  real_T c14_y, real_T c14_x)
{
  real_T c14_b_y;
  real_T c14_b_x;
  c14_b_eml_scalar_eg(chartInstance);
  c14_b_y = c14_y;
  c14_b_x = c14_x;
  return muDoubleScalarAtan2(c14_b_y, c14_b_x);
}

static void c14_b_eml_scalar_eg(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c14_c_eml_scalar_eg(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c14_realmin(SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c14_eps(SFc14_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void c14_eml_matlab_zgetrf(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_b_A[16], int32_T c14_ipiv[4],
  int32_T *c14_info)
{
  int32_T c14_i253;
  for (c14_i253 = 0; c14_i253 < 16; c14_i253++) {
    c14_b_A[c14_i253] = c14_A[c14_i253];
  }

  c14_b_eml_matlab_zgetrf(chartInstance, c14_b_A, c14_ipiv, c14_info);
}

static void c14_check_forloop_overflow_error
  (SFc14_torqueBalancing2012bInstanceStruct *chartInstance, boolean_T
   c14_overflow)
{
  int32_T c14_i254;
  static char_T c14_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c14_u[34];
  const mxArray *c14_y = NULL;
  int32_T c14_i255;
  static char_T c14_cv3[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c14_b_u[23];
  const mxArray *c14_b_y = NULL;
  if (!c14_overflow) {
  } else {
    for (c14_i254 = 0; c14_i254 < 34; c14_i254++) {
      c14_u[c14_i254] = c14_cv2[c14_i254];
    }

    c14_y = NULL;
    sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c14_i255 = 0; c14_i255 < 23; c14_i255++) {
      c14_b_u[c14_i255] = c14_cv3[c14_i255];
    }

    c14_b_y = NULL;
    sf_mex_assign(&c14_b_y, sf_mex_create("y", c14_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c14_y, 14, c14_b_y));
  }
}

static void c14_eml_xger(SFc14_torqueBalancing2012bInstanceStruct *chartInstance,
  int32_T c14_m, int32_T c14_n, real_T c14_alpha1, int32_T c14_ix0, int32_T
  c14_iy0, real_T c14_A[16], int32_T c14_ia0, real_T c14_b_A[16])
{
  int32_T c14_i256;
  for (c14_i256 = 0; c14_i256 < 16; c14_i256++) {
    c14_b_A[c14_i256] = c14_A[c14_i256];
  }

  c14_b_eml_xger(chartInstance, c14_m, c14_n, c14_alpha1, c14_ix0, c14_iy0,
                 c14_b_A, c14_ia0);
}

static void c14_eml_xtrsm(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_B[16], real_T c14_b_B[16])
{
  int32_T c14_i257;
  int32_T c14_i258;
  real_T c14_b_A[16];
  for (c14_i257 = 0; c14_i257 < 16; c14_i257++) {
    c14_b_B[c14_i257] = c14_B[c14_i257];
  }

  for (c14_i258 = 0; c14_i258 < 16; c14_i258++) {
    c14_b_A[c14_i258] = c14_A[c14_i258];
  }

  c14_c_eml_xtrsm(chartInstance, c14_b_A, c14_b_B);
}

static void c14_below_threshold(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c14_d_eml_scalar_eg(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
}

static void c14_b_eml_xtrsm(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_B[16], real_T c14_b_B[16])
{
  int32_T c14_i259;
  int32_T c14_i260;
  real_T c14_b_A[16];
  for (c14_i259 = 0; c14_i259 < 16; c14_i259++) {
    c14_b_B[c14_i259] = c14_B[c14_i259];
  }

  for (c14_i260 = 0; c14_i260 < 16; c14_i260++) {
    c14_b_A[c14_i260] = c14_A[c14_i260];
  }

  c14_d_eml_xtrsm(chartInstance, c14_b_A, c14_b_B);
}

static void c14_eml_warning(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c14_i261;
  static char_T c14_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c14_u[27];
  const mxArray *c14_y = NULL;
  for (c14_i261 = 0; c14_i261 < 27; c14_i261++) {
    c14_u[c14_i261] = c14_varargin_1[c14_i261];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 10, 0U, 1U, 0U, 2, 1, 27),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c14_y));
}

static const mxArray *c14_g_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_u;
  const mxArray *c14_y = NULL;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_u = *(int32_T *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static int32_T c14_l_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  int32_T c14_y;
  int32_T c14_i262;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_i262, 1, 6, 0U, 0, 0U, 0);
  c14_y = c14_i262;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void c14_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_b_sfEvent;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  int32_T c14_y;
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c14_b_sfEvent = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_sfEvent),
    &c14_thisId);
  sf_mex_destroy(&c14_b_sfEvent);
  *(int32_T *)c14_outData = c14_y;
  sf_mex_destroy(&c14_mxArrayInData);
}

static uint8_T c14_m_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_b_is_active_c14_torqueBalancing2012b, const
  char_T *c14_identifier)
{
  uint8_T c14_y;
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c14_b_is_active_c14_torqueBalancing2012b), &c14_thisId);
  sf_mex_destroy(&c14_b_is_active_c14_torqueBalancing2012b);
  return c14_y;
}

static uint8_T c14_n_emlrt_marshallIn(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  uint8_T c14_y;
  uint8_T c14_u0;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_u0, 1, 3, 0U, 0, 0U, 0);
  c14_y = c14_u0;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void c14_b_eml_matlab_zgetrf(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], int32_T c14_ipiv[4], int32_T *c14_info)
{
  int32_T c14_i263;
  int32_T c14_j;
  int32_T c14_b_j;
  int32_T c14_a;
  int32_T c14_jm1;
  int32_T c14_b;
  int32_T c14_mmj;
  int32_T c14_b_a;
  int32_T c14_c;
  int32_T c14_b_b;
  int32_T c14_jj;
  int32_T c14_c_a;
  int32_T c14_jp1j;
  int32_T c14_d_a;
  int32_T c14_b_c;
  int32_T c14_n;
  int32_T c14_ix0;
  int32_T c14_b_n;
  int32_T c14_b_ix0;
  int32_T c14_c_n;
  int32_T c14_c_ix0;
  int32_T c14_idxmax;
  int32_T c14_ix;
  real_T c14_x;
  real_T c14_b_x;
  real_T c14_c_x;
  real_T c14_y;
  real_T c14_d_x;
  real_T c14_e_x;
  real_T c14_b_y;
  real_T c14_smax;
  int32_T c14_d_n;
  int32_T c14_c_b;
  int32_T c14_d_b;
  boolean_T c14_overflow;
  int32_T c14_k;
  int32_T c14_b_k;
  int32_T c14_e_a;
  real_T c14_f_x;
  real_T c14_g_x;
  real_T c14_h_x;
  real_T c14_c_y;
  real_T c14_i_x;
  real_T c14_j_x;
  real_T c14_d_y;
  real_T c14_s;
  int32_T c14_f_a;
  int32_T c14_jpiv_offset;
  int32_T c14_g_a;
  int32_T c14_e_b;
  int32_T c14_jpiv;
  int32_T c14_h_a;
  int32_T c14_f_b;
  int32_T c14_c_c;
  int32_T c14_g_b;
  int32_T c14_jrow;
  int32_T c14_i_a;
  int32_T c14_h_b;
  int32_T c14_jprow;
  int32_T c14_d_ix0;
  int32_T c14_iy0;
  int32_T c14_e_ix0;
  int32_T c14_b_iy0;
  int32_T c14_f_ix0;
  int32_T c14_c_iy0;
  int32_T c14_b_ix;
  int32_T c14_iy;
  int32_T c14_c_k;
  real_T c14_temp;
  int32_T c14_j_a;
  int32_T c14_k_a;
  int32_T c14_b_jp1j;
  int32_T c14_l_a;
  int32_T c14_d_c;
  int32_T c14_m_a;
  int32_T c14_i_b;
  int32_T c14_i264;
  int32_T c14_n_a;
  int32_T c14_j_b;
  int32_T c14_o_a;
  int32_T c14_k_b;
  boolean_T c14_b_overflow;
  int32_T c14_i;
  int32_T c14_b_i;
  real_T c14_k_x;
  real_T c14_e_y;
  real_T c14_z;
  int32_T c14_l_b;
  int32_T c14_e_c;
  int32_T c14_p_a;
  int32_T c14_f_c;
  int32_T c14_q_a;
  int32_T c14_g_c;
  int32_T c14_m;
  int32_T c14_e_n;
  int32_T c14_g_ix0;
  int32_T c14_d_iy0;
  int32_T c14_ia0;
  real_T c14_d1;
  c14_realmin(chartInstance);
  c14_eps(chartInstance);
  for (c14_i263 = 0; c14_i263 < 4; c14_i263++) {
    c14_ipiv[c14_i263] = 1 + c14_i263;
  }

  *c14_info = 0;
  for (c14_j = 1; c14_j < 4; c14_j++) {
    c14_b_j = c14_j;
    c14_a = c14_b_j - 1;
    c14_jm1 = c14_a;
    c14_b = c14_b_j;
    c14_mmj = 4 - c14_b;
    c14_b_a = c14_jm1;
    c14_c = c14_b_a * 5;
    c14_b_b = c14_c + 1;
    c14_jj = c14_b_b;
    c14_c_a = c14_jj + 1;
    c14_jp1j = c14_c_a;
    c14_d_a = c14_mmj;
    c14_b_c = c14_d_a;
    c14_n = c14_b_c + 1;
    c14_ix0 = c14_jj;
    c14_b_n = c14_n;
    c14_b_ix0 = c14_ix0;
    c14_c_n = c14_b_n;
    c14_c_ix0 = c14_b_ix0;
    if (c14_c_n < 1) {
      c14_idxmax = 0;
    } else {
      c14_idxmax = 1;
      if (c14_c_n > 1) {
        c14_ix = c14_c_ix0;
        c14_x = c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_ix), 1, 16, 1, 0) - 1];
        c14_b_x = c14_x;
        c14_c_x = c14_b_x;
        c14_y = muDoubleScalarAbs(c14_c_x);
        c14_d_x = 0.0;
        c14_e_x = c14_d_x;
        c14_b_y = muDoubleScalarAbs(c14_e_x);
        c14_smax = c14_y + c14_b_y;
        c14_d_n = c14_c_n;
        c14_c_b = c14_d_n;
        c14_d_b = c14_c_b;
        if (2 > c14_d_b) {
          c14_overflow = FALSE;
        } else {
          c14_overflow = (c14_d_b > 2147483646);
        }

        if (c14_overflow) {
          c14_check_forloop_overflow_error(chartInstance, c14_overflow);
        }

        for (c14_k = 2; c14_k <= c14_d_n; c14_k++) {
          c14_b_k = c14_k;
          c14_e_a = c14_ix + 1;
          c14_ix = c14_e_a;
          c14_f_x = c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c14_ix), 1, 16, 1, 0) - 1];
          c14_g_x = c14_f_x;
          c14_h_x = c14_g_x;
          c14_c_y = muDoubleScalarAbs(c14_h_x);
          c14_i_x = 0.0;
          c14_j_x = c14_i_x;
          c14_d_y = muDoubleScalarAbs(c14_j_x);
          c14_s = c14_c_y + c14_d_y;
          if (c14_s > c14_smax) {
            c14_idxmax = c14_b_k;
            c14_smax = c14_s;
          }
        }
      }
    }

    c14_f_a = c14_idxmax - 1;
    c14_jpiv_offset = c14_f_a;
    c14_g_a = c14_jj;
    c14_e_b = c14_jpiv_offset;
    c14_jpiv = c14_g_a + c14_e_b;
    if (c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c14_jpiv), 1, 16, 1, 0) - 1] != 0.0) {
      if (c14_jpiv_offset != 0) {
        c14_h_a = c14_b_j;
        c14_f_b = c14_jpiv_offset;
        c14_c_c = c14_h_a + c14_f_b;
        c14_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c14_b_j), 1, 4, 1, 0) - 1] = c14_c_c;
        c14_g_b = c14_jm1 + 1;
        c14_jrow = c14_g_b;
        c14_i_a = c14_jrow;
        c14_h_b = c14_jpiv_offset;
        c14_jprow = c14_i_a + c14_h_b;
        c14_d_ix0 = c14_jrow;
        c14_iy0 = c14_jprow;
        c14_e_ix0 = c14_d_ix0;
        c14_b_iy0 = c14_iy0;
        c14_f_ix0 = c14_e_ix0;
        c14_c_iy0 = c14_b_iy0;
        c14_b_ix = c14_f_ix0;
        c14_iy = c14_c_iy0;
        for (c14_c_k = 1; c14_c_k < 5; c14_c_k++) {
          c14_temp = c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c14_b_ix), 1, 16, 1, 0) - 1];
          c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_b_ix), 1, 16, 1, 0) - 1] =
            c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_iy), 1, 16, 1, 0) - 1];
          c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_iy), 1, 16, 1, 0) - 1] = c14_temp;
          c14_j_a = c14_b_ix + 4;
          c14_b_ix = c14_j_a;
          c14_k_a = c14_iy + 4;
          c14_iy = c14_k_a;
        }
      }

      c14_b_jp1j = c14_jp1j;
      c14_l_a = c14_mmj;
      c14_d_c = c14_l_a;
      c14_m_a = c14_jp1j;
      c14_i_b = c14_d_c - 1;
      c14_i264 = c14_m_a + c14_i_b;
      c14_n_a = c14_b_jp1j;
      c14_j_b = c14_i264;
      c14_o_a = c14_n_a;
      c14_k_b = c14_j_b;
      if (c14_o_a > c14_k_b) {
        c14_b_overflow = FALSE;
      } else {
        c14_b_overflow = (c14_k_b > 2147483646);
      }

      if (c14_b_overflow) {
        c14_check_forloop_overflow_error(chartInstance, c14_b_overflow);
      }

      for (c14_i = c14_b_jp1j; c14_i <= c14_i264; c14_i++) {
        c14_b_i = c14_i;
        c14_k_x = c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_b_i), 1, 16, 1, 0) - 1];
        c14_e_y = c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_jj), 1, 16, 1, 0) - 1];
        c14_z = c14_k_x / c14_e_y;
        c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c14_b_i), 1, 16, 1, 0) - 1] = c14_z;
      }
    } else {
      *c14_info = c14_b_j;
    }

    c14_l_b = c14_b_j;
    c14_e_c = 4 - c14_l_b;
    c14_p_a = c14_jj;
    c14_f_c = c14_p_a;
    c14_q_a = c14_jj;
    c14_g_c = c14_q_a;
    c14_m = c14_mmj;
    c14_e_n = c14_e_c;
    c14_g_ix0 = c14_jp1j;
    c14_d_iy0 = c14_f_c + 4;
    c14_ia0 = c14_g_c + 5;
    c14_d1 = -1.0;
    c14_b_eml_xger(chartInstance, c14_m, c14_e_n, c14_d1, c14_g_ix0, c14_d_iy0,
                   c14_A, c14_ia0);
  }

  if (*c14_info == 0) {
    if (!(c14_A[15] != 0.0)) {
      *c14_info = 4;
    }
  }
}

static void c14_b_eml_xger(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, int32_T c14_m, int32_T c14_n, real_T c14_alpha1, int32_T
  c14_ix0, int32_T c14_iy0, real_T c14_A[16], int32_T c14_ia0)
{
  int32_T c14_b_m;
  int32_T c14_b_n;
  real_T c14_b_alpha1;
  int32_T c14_b_ix0;
  int32_T c14_b_iy0;
  int32_T c14_b_ia0;
  int32_T c14_c_m;
  int32_T c14_c_n;
  real_T c14_c_alpha1;
  int32_T c14_c_ix0;
  int32_T c14_c_iy0;
  int32_T c14_c_ia0;
  int32_T c14_d_m;
  int32_T c14_d_n;
  real_T c14_d_alpha1;
  int32_T c14_d_ix0;
  int32_T c14_d_iy0;
  int32_T c14_d_ia0;
  int32_T c14_ixstart;
  int32_T c14_a;
  int32_T c14_jA;
  int32_T c14_jy;
  int32_T c14_e_n;
  int32_T c14_b;
  int32_T c14_b_b;
  boolean_T c14_overflow;
  int32_T c14_j;
  real_T c14_yjy;
  real_T c14_temp;
  int32_T c14_ix;
  int32_T c14_c_b;
  int32_T c14_i265;
  int32_T c14_b_a;
  int32_T c14_d_b;
  int32_T c14_i266;
  int32_T c14_c_a;
  int32_T c14_e_b;
  int32_T c14_d_a;
  int32_T c14_f_b;
  boolean_T c14_b_overflow;
  int32_T c14_ijA;
  int32_T c14_b_ijA;
  int32_T c14_e_a;
  int32_T c14_f_a;
  int32_T c14_g_a;
  c14_b_m = c14_m;
  c14_b_n = c14_n;
  c14_b_alpha1 = c14_alpha1;
  c14_b_ix0 = c14_ix0;
  c14_b_iy0 = c14_iy0;
  c14_b_ia0 = c14_ia0;
  c14_c_m = c14_b_m;
  c14_c_n = c14_b_n;
  c14_c_alpha1 = c14_b_alpha1;
  c14_c_ix0 = c14_b_ix0;
  c14_c_iy0 = c14_b_iy0;
  c14_c_ia0 = c14_b_ia0;
  c14_d_m = c14_c_m;
  c14_d_n = c14_c_n;
  c14_d_alpha1 = c14_c_alpha1;
  c14_d_ix0 = c14_c_ix0;
  c14_d_iy0 = c14_c_iy0;
  c14_d_ia0 = c14_c_ia0;
  if (c14_d_alpha1 == 0.0) {
  } else {
    c14_ixstart = c14_d_ix0;
    c14_a = c14_d_ia0 - 1;
    c14_jA = c14_a;
    c14_jy = c14_d_iy0;
    c14_e_n = c14_d_n;
    c14_b = c14_e_n;
    c14_b_b = c14_b;
    if (1 > c14_b_b) {
      c14_overflow = FALSE;
    } else {
      c14_overflow = (c14_b_b > 2147483646);
    }

    if (c14_overflow) {
      c14_check_forloop_overflow_error(chartInstance, c14_overflow);
    }

    for (c14_j = 1; c14_j <= c14_e_n; c14_j++) {
      c14_yjy = c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c14_jy), 1, 16, 1, 0) - 1];
      if (c14_yjy != 0.0) {
        c14_temp = c14_yjy * c14_d_alpha1;
        c14_ix = c14_ixstart;
        c14_c_b = c14_jA + 1;
        c14_i265 = c14_c_b;
        c14_b_a = c14_d_m;
        c14_d_b = c14_jA;
        c14_i266 = c14_b_a + c14_d_b;
        c14_c_a = c14_i265;
        c14_e_b = c14_i266;
        c14_d_a = c14_c_a;
        c14_f_b = c14_e_b;
        if (c14_d_a > c14_f_b) {
          c14_b_overflow = FALSE;
        } else {
          c14_b_overflow = (c14_f_b > 2147483646);
        }

        if (c14_b_overflow) {
          c14_check_forloop_overflow_error(chartInstance, c14_b_overflow);
        }

        for (c14_ijA = c14_i265; c14_ijA <= c14_i266; c14_ijA++) {
          c14_b_ijA = c14_ijA;
          c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_b_ijA), 1, 16, 1, 0) - 1] =
            c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_b_ijA), 1, 16, 1, 0) - 1] +
            c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_ix), 1, 16, 1, 0) - 1] * c14_temp;
          c14_e_a = c14_ix + 1;
          c14_ix = c14_e_a;
        }
      }

      c14_f_a = c14_jy + 4;
      c14_jy = c14_f_a;
      c14_g_a = c14_jA + 4;
      c14_jA = c14_g_a;
    }
  }
}

static void c14_c_eml_xtrsm(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_B[16])
{
  int32_T c14_j;
  int32_T c14_b_j;
  int32_T c14_a;
  int32_T c14_c;
  int32_T c14_b;
  int32_T c14_b_c;
  int32_T c14_b_b;
  int32_T c14_jBcol;
  int32_T c14_k;
  int32_T c14_b_k;
  int32_T c14_b_a;
  int32_T c14_c_c;
  int32_T c14_c_b;
  int32_T c14_d_c;
  int32_T c14_d_b;
  int32_T c14_kAcol;
  int32_T c14_c_a;
  int32_T c14_e_b;
  int32_T c14_e_c;
  int32_T c14_d_a;
  int32_T c14_i267;
  boolean_T c14_overflow;
  int32_T c14_i;
  int32_T c14_b_i;
  int32_T c14_e_a;
  int32_T c14_f_b;
  int32_T c14_f_c;
  int32_T c14_f_a;
  int32_T c14_g_b;
  int32_T c14_g_c;
  int32_T c14_g_a;
  int32_T c14_h_b;
  int32_T c14_h_c;
  int32_T c14_h_a;
  int32_T c14_i_b;
  int32_T c14_i_c;
  c14_below_threshold(chartInstance);
  c14_d_eml_scalar_eg(chartInstance);
  for (c14_j = 1; c14_j < 5; c14_j++) {
    c14_b_j = c14_j;
    c14_a = c14_b_j;
    c14_c = c14_a;
    c14_b = c14_c - 1;
    c14_b_c = c14_b << 2;
    c14_b_b = c14_b_c;
    c14_jBcol = c14_b_b;
    for (c14_k = 1; c14_k < 5; c14_k++) {
      c14_b_k = c14_k;
      c14_b_a = c14_b_k;
      c14_c_c = c14_b_a;
      c14_c_b = c14_c_c - 1;
      c14_d_c = c14_c_b << 2;
      c14_d_b = c14_d_c;
      c14_kAcol = c14_d_b;
      c14_c_a = c14_b_k;
      c14_e_b = c14_jBcol;
      c14_e_c = c14_c_a + c14_e_b;
      if (c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c14_d_a = c14_b_k;
        c14_i267 = c14_d_a;
        c14_overflow = FALSE;
        if (c14_overflow) {
          c14_check_forloop_overflow_error(chartInstance, c14_overflow);
        }

        for (c14_i = c14_i267 + 1; c14_i < 5; c14_i++) {
          c14_b_i = c14_i;
          c14_e_a = c14_b_i;
          c14_f_b = c14_jBcol;
          c14_f_c = c14_e_a + c14_f_b;
          c14_f_a = c14_b_i;
          c14_g_b = c14_jBcol;
          c14_g_c = c14_f_a + c14_g_b;
          c14_g_a = c14_b_k;
          c14_h_b = c14_jBcol;
          c14_h_c = c14_g_a + c14_h_b;
          c14_h_a = c14_b_i;
          c14_i_b = c14_kAcol;
          c14_i_c = c14_h_a + c14_i_b;
          c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_f_c), 1, 16, 1, 0) - 1] =
            c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_g_c), 1, 16, 1, 0) - 1] -
            c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_h_c), 1, 16, 1, 0) - 1] *
            c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_i_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void c14_d_eml_xtrsm(SFc14_torqueBalancing2012bInstanceStruct
  *chartInstance, real_T c14_A[16], real_T c14_B[16])
{
  int32_T c14_j;
  int32_T c14_b_j;
  int32_T c14_a;
  int32_T c14_c;
  int32_T c14_b;
  int32_T c14_b_c;
  int32_T c14_b_b;
  int32_T c14_jBcol;
  int32_T c14_k;
  int32_T c14_b_k;
  int32_T c14_b_a;
  int32_T c14_c_c;
  int32_T c14_c_b;
  int32_T c14_d_c;
  int32_T c14_d_b;
  int32_T c14_kAcol;
  int32_T c14_c_a;
  int32_T c14_e_b;
  int32_T c14_e_c;
  int32_T c14_d_a;
  int32_T c14_f_b;
  int32_T c14_f_c;
  int32_T c14_e_a;
  int32_T c14_g_b;
  int32_T c14_g_c;
  int32_T c14_f_a;
  int32_T c14_h_b;
  int32_T c14_h_c;
  real_T c14_x;
  real_T c14_y;
  real_T c14_z;
  int32_T c14_g_a;
  int32_T c14_i268;
  int32_T c14_i_b;
  int32_T c14_j_b;
  boolean_T c14_overflow;
  int32_T c14_i;
  int32_T c14_b_i;
  int32_T c14_h_a;
  int32_T c14_k_b;
  int32_T c14_i_c;
  int32_T c14_i_a;
  int32_T c14_l_b;
  int32_T c14_j_c;
  int32_T c14_j_a;
  int32_T c14_m_b;
  int32_T c14_k_c;
  int32_T c14_k_a;
  int32_T c14_n_b;
  int32_T c14_l_c;
  c14_below_threshold(chartInstance);
  c14_d_eml_scalar_eg(chartInstance);
  for (c14_j = 1; c14_j < 5; c14_j++) {
    c14_b_j = c14_j;
    c14_a = c14_b_j;
    c14_c = c14_a;
    c14_b = c14_c - 1;
    c14_b_c = c14_b << 2;
    c14_b_b = c14_b_c;
    c14_jBcol = c14_b_b;
    for (c14_k = 4; c14_k > 0; c14_k--) {
      c14_b_k = c14_k;
      c14_b_a = c14_b_k;
      c14_c_c = c14_b_a;
      c14_c_b = c14_c_c - 1;
      c14_d_c = c14_c_b << 2;
      c14_d_b = c14_d_c;
      c14_kAcol = c14_d_b;
      c14_c_a = c14_b_k;
      c14_e_b = c14_jBcol;
      c14_e_c = c14_c_a + c14_e_b;
      if (c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_e_c), 1, 16, 1, 0) - 1] != 0.0) {
        c14_d_a = c14_b_k;
        c14_f_b = c14_jBcol;
        c14_f_c = c14_d_a + c14_f_b;
        c14_e_a = c14_b_k;
        c14_g_b = c14_jBcol;
        c14_g_c = c14_e_a + c14_g_b;
        c14_f_a = c14_b_k;
        c14_h_b = c14_kAcol;
        c14_h_c = c14_f_a + c14_h_b;
        c14_x = c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_g_c), 1, 16, 1, 0) - 1];
        c14_y = c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_h_c), 1, 16, 1, 0) - 1];
        c14_z = c14_x / c14_y;
        c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c14_f_c), 1, 16, 1, 0) - 1] = c14_z;
        c14_g_a = c14_b_k - 1;
        c14_i268 = c14_g_a;
        c14_i_b = c14_i268;
        c14_j_b = c14_i_b;
        if (1 > c14_j_b) {
          c14_overflow = FALSE;
        } else {
          c14_overflow = (c14_j_b > 2147483646);
        }

        if (c14_overflow) {
          c14_check_forloop_overflow_error(chartInstance, c14_overflow);
        }

        for (c14_i = 1; c14_i <= c14_i268; c14_i++) {
          c14_b_i = c14_i;
          c14_h_a = c14_b_i;
          c14_k_b = c14_jBcol;
          c14_i_c = c14_h_a + c14_k_b;
          c14_i_a = c14_b_i;
          c14_l_b = c14_jBcol;
          c14_j_c = c14_i_a + c14_l_b;
          c14_j_a = c14_b_k;
          c14_m_b = c14_jBcol;
          c14_k_c = c14_j_a + c14_m_b;
          c14_k_a = c14_b_i;
          c14_n_b = c14_kAcol;
          c14_l_c = c14_k_a + c14_n_b;
          c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_i_c), 1, 16, 1, 0) - 1] =
            c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_j_c), 1, 16, 1, 0) - 1] -
            c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_k_c), 1, 16, 1, 0) - 1] *
            c14_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c14_l_c), 1, 16, 1, 0) - 1];
        }
      }
    }
  }
}

static void init_dsm_address_info(SFc14_torqueBalancing2012bInstanceStruct
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

void sf_c14_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3853123732U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4225446195U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2845734475U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1223003812U);
}

mxArray *sf_c14_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("5NGEe6J8FMNHwSbai8nj5E");
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

mxArray *sf_c14_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c14_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[71],T\"w_H_root\",},{M[8],M[0],T\"is_active_c14_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c14_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           14,
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
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)
            c14_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_b_sf_marshallOut,(MexInFcnForType)
          c14_b_sf_marshallIn);

        {
          real_T (*c14_imu_H_link)[16];
          real_T (*c14_imu_H_link_0)[16];
          real_T (*c14_link_H_root)[16];
          real_T (*c14_inertial_0)[12];
          real_T (*c14_inertial)[12];
          real_T (*c14_neck)[3];
          real_T (*c14_w_H_root)[16];
          c14_w_H_root = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S,
            1);
          c14_neck = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
          c14_inertial = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S,
            4);
          c14_inertial_0 = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S,
            3);
          c14_link_H_root = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 2);
          c14_imu_H_link_0 = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 1);
          c14_imu_H_link = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c14_imu_H_link);
          _SFD_SET_DATA_VALUE_PTR(1U, *c14_imu_H_link_0);
          _SFD_SET_DATA_VALUE_PTR(2U, *c14_link_H_root);
          _SFD_SET_DATA_VALUE_PTR(3U, *c14_inertial_0);
          _SFD_SET_DATA_VALUE_PTR(4U, *c14_inertial);
          _SFD_SET_DATA_VALUE_PTR(5U, *c14_neck);
          _SFD_SET_DATA_VALUE_PTR(6U, *c14_w_H_root);
          _SFD_SET_DATA_VALUE_PTR(7U, &chartInstance->c14_CONFIG);
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
  return "GVWiF6laSmdXaPaeF1eCR";
}

static void sf_opaque_initialize_c14_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc14_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c14_torqueBalancing2012b
    ((SFc14_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c14_torqueBalancing2012b((SFc14_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c14_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c14_torqueBalancing2012b((SFc14_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c14_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c14_torqueBalancing2012b((SFc14_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c14_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c14_torqueBalancing2012b((SFc14_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c14_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c14_torqueBalancing2012b
    ((SFc14_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c14_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c14_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c14_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c14_torqueBalancing2012b
    ((SFc14_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c14_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c14_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c14_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c14_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c14_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc14_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c14_torqueBalancing2012b((SFc14_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc14_torqueBalancing2012b
    ((SFc14_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c14_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c14_torqueBalancing2012b
      ((SFc14_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c14_torqueBalancing2012b(SimStruct *S)
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
      14);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,14,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,14,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,14);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,14,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,14,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,14);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2508711673U));
  ssSetChecksum1(S,(1986591927U));
  ssSetChecksum2(S,(3393621003U));
  ssSetChecksum3(S,(2332823037U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c14_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c14_torqueBalancing2012b(SimStruct *S)
{
  SFc14_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc14_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc14_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc14_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c14_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c14_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c14_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c14_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c14_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c14_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c14_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c14_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c14_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c14_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c14_torqueBalancing2012b;
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

void c14_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c14_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c14_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c14_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c14_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
