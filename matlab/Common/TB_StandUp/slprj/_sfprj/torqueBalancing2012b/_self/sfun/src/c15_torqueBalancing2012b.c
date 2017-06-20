/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing2012b_sfun.h"
#include "c15_torqueBalancing2012b.h"
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
static const char * c15_debug_family_names[4] = { "nargin", "nargout", "u",
  "normU" };

/* Function Declarations */
static void initialize_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static void initialize_params_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static void enable_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static void disable_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static void c15_update_debugger_state_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static void set_sim_state_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c15_st);
static void finalize_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static void sf_c15_torqueBalancing2012b(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance);
static void initSimStructsc15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static void registerMessagesc15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c15_machineNumber, uint32_T
  c15_chartNumber);
static const mxArray *c15_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData);
static real_T c15_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_normU, const char_T *c15_identifier);
static real_T c15_b_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId);
static void c15_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData);
static const mxArray *c15_b_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData);
static void c15_info_helper(c15_ResolvedFunctionInfo c15_info[20]);
static const mxArray *c15_c_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData);
static int32_T c15_c_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId);
static void c15_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData);
static uint8_T c15_d_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_b_is_active_c15_torqueBalancing2012b, const
  char_T *c15_identifier);
static uint8_T c15_e_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId);
static void init_dsm_address_info(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
  chartInstance->c15_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c15_is_active_c15_torqueBalancing2012b = 0U;
}

static void initialize_params_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void enable_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c15_update_debugger_state_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
  const mxArray *c15_st;
  const mxArray *c15_y = NULL;
  real_T c15_hoistedGlobal;
  real_T c15_u;
  const mxArray *c15_b_y = NULL;
  uint8_T c15_b_hoistedGlobal;
  uint8_T c15_b_u;
  const mxArray *c15_c_y = NULL;
  real_T *c15_normU;
  c15_normU = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c15_st = NULL;
  c15_st = NULL;
  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_createcellarray(2), FALSE);
  c15_hoistedGlobal = *c15_normU;
  c15_u = c15_hoistedGlobal;
  c15_b_y = NULL;
  sf_mex_assign(&c15_b_y, sf_mex_create("y", &c15_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c15_y, 0, c15_b_y);
  c15_b_hoistedGlobal = chartInstance->c15_is_active_c15_torqueBalancing2012b;
  c15_b_u = c15_b_hoistedGlobal;
  c15_c_y = NULL;
  sf_mex_assign(&c15_c_y, sf_mex_create("y", &c15_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c15_y, 1, c15_c_y);
  sf_mex_assign(&c15_st, c15_y, FALSE);
  return c15_st;
}

static void set_sim_state_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance, const mxArray
   *c15_st)
{
  const mxArray *c15_u;
  real_T *c15_normU;
  c15_normU = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c15_doneDoubleBufferReInit = TRUE;
  c15_u = sf_mex_dup(c15_st);
  *c15_normU = c15_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c15_u, 0)), "normU");
  chartInstance->c15_is_active_c15_torqueBalancing2012b = c15_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c15_u, 1)),
     "is_active_c15_torqueBalancing2012b");
  sf_mex_destroy(&c15_u);
  c15_update_debugger_state_c15_torqueBalancing2012b(chartInstance);
  sf_mex_destroy(&c15_st);
}

static void finalize_c15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void sf_c15_torqueBalancing2012b(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance)
{
  int32_T c15_i0;
  int32_T c15_i1;
  real_T c15_u[6];
  uint32_T c15_debug_family_var_map[4];
  real_T c15_nargin = 1.0;
  real_T c15_nargout = 1.0;
  real_T c15_normU;
  int32_T c15_i2;
  real_T c15_x[6];
  real_T c15_scale;
  int32_T c15_k;
  int32_T c15_b_k;
  real_T c15_b_x;
  real_T c15_c_x;
  real_T c15_absxk;
  real_T c15_t;
  real_T *c15_b_normU;
  real_T (*c15_b_u)[6];
  c15_b_normU = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c15_b_u = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 13U, chartInstance->c15_sfEvent);
  for (c15_i0 = 0; c15_i0 < 6; c15_i0++) {
    _SFD_DATA_RANGE_CHECK((*c15_b_u)[c15_i0], 0U);
  }

  _SFD_DATA_RANGE_CHECK(*c15_b_normU, 1U);
  chartInstance->c15_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 13U, chartInstance->c15_sfEvent);
  for (c15_i1 = 0; c15_i1 < 6; c15_i1++) {
    c15_u[c15_i1] = (*c15_b_u)[c15_i1];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c15_debug_family_names,
    c15_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_nargin, 0U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_nargout, 1U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c15_u, 2U, c15_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c15_normU, 3U, c15_sf_marshallOut,
    c15_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, 4);
  for (c15_i2 = 0; c15_i2 < 6; c15_i2++) {
    c15_x[c15_i2] = c15_u[c15_i2];
  }

  c15_normU = 0.0;
  c15_scale = 2.2250738585072014E-308;
  for (c15_k = 1; c15_k < 7; c15_k++) {
    c15_b_k = c15_k;
    c15_b_x = c15_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c15_b_k), 1, 6, 1, 0) - 1];
    c15_c_x = c15_b_x;
    c15_absxk = muDoubleScalarAbs(c15_c_x);
    if (c15_absxk > c15_scale) {
      c15_t = c15_scale / c15_absxk;
      c15_normU = 1.0 + c15_normU * c15_t * c15_t;
      c15_scale = c15_absxk;
    } else {
      c15_t = c15_absxk / c15_scale;
      c15_normU += c15_t * c15_t;
    }
  }

  c15_normU = c15_scale * muDoubleScalarSqrt(c15_normU);
  _SFD_EML_CALL(0U, chartInstance->c15_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
  *c15_b_normU = c15_normU;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 13U, chartInstance->c15_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancing2012bMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void registerMessagesc15_torqueBalancing2012b
  (SFc15_torqueBalancing2012bInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c15_machineNumber, uint32_T
  c15_chartNumber)
{
}

static const mxArray *c15_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData)
{
  const mxArray *c15_mxArrayOutData = NULL;
  real_T c15_u;
  const mxArray *c15_y = NULL;
  SFc15_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc15_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c15_mxArrayOutData = NULL;
  c15_u = *(real_T *)c15_inData;
  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", &c15_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c15_mxArrayOutData, c15_y, FALSE);
  return c15_mxArrayOutData;
}

static real_T c15_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_normU, const char_T *c15_identifier)
{
  real_T c15_y;
  emlrtMsgIdentifier c15_thisId;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_y = c15_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c15_normU),
    &c15_thisId);
  sf_mex_destroy(&c15_normU);
  return c15_y;
}

static real_T c15_b_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId)
{
  real_T c15_y;
  real_T c15_d0;
  sf_mex_import(c15_parentId, sf_mex_dup(c15_u), &c15_d0, 1, 0, 0U, 0, 0U, 0);
  c15_y = c15_d0;
  sf_mex_destroy(&c15_u);
  return c15_y;
}

static void c15_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData)
{
  const mxArray *c15_normU;
  const char_T *c15_identifier;
  emlrtMsgIdentifier c15_thisId;
  real_T c15_y;
  SFc15_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc15_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c15_normU = sf_mex_dup(c15_mxArrayInData);
  c15_identifier = c15_varName;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_y = c15_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c15_normU),
    &c15_thisId);
  sf_mex_destroy(&c15_normU);
  *(real_T *)c15_outData = c15_y;
  sf_mex_destroy(&c15_mxArrayInData);
}

static const mxArray *c15_b_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData)
{
  const mxArray *c15_mxArrayOutData = NULL;
  int32_T c15_i3;
  real_T c15_b_inData[6];
  int32_T c15_i4;
  real_T c15_u[6];
  const mxArray *c15_y = NULL;
  SFc15_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc15_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c15_mxArrayOutData = NULL;
  for (c15_i3 = 0; c15_i3 < 6; c15_i3++) {
    c15_b_inData[c15_i3] = (*(real_T (*)[6])c15_inData)[c15_i3];
  }

  for (c15_i4 = 0; c15_i4 < 6; c15_i4++) {
    c15_u[c15_i4] = c15_b_inData[c15_i4];
  }

  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", c15_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c15_mxArrayOutData, c15_y, FALSE);
  return c15_mxArrayOutData;
}

const mxArray *sf_c15_torqueBalancing2012b_get_eml_resolved_functions_info(void)
{
  const mxArray *c15_nameCaptureInfo;
  c15_ResolvedFunctionInfo c15_info[20];
  const mxArray *c15_m0 = NULL;
  int32_T c15_i5;
  c15_ResolvedFunctionInfo *c15_r0;
  c15_nameCaptureInfo = NULL;
  c15_nameCaptureInfo = NULL;
  c15_info_helper(c15_info);
  sf_mex_assign(&c15_m0, sf_mex_createstruct("nameCaptureInfo", 1, 20), FALSE);
  for (c15_i5 = 0; c15_i5 < 20; c15_i5++) {
    c15_r0 = &c15_info[c15_i5];
    sf_mex_addfield(c15_m0, sf_mex_create("nameCaptureInfo", c15_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c15_r0->context)), "context", "nameCaptureInfo",
                    c15_i5);
    sf_mex_addfield(c15_m0, sf_mex_create("nameCaptureInfo", c15_r0->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c15_r0->name)), "name", "nameCaptureInfo", c15_i5);
    sf_mex_addfield(c15_m0, sf_mex_create("nameCaptureInfo",
      c15_r0->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c15_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c15_i5);
    sf_mex_addfield(c15_m0, sf_mex_create("nameCaptureInfo", c15_r0->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c15_r0->resolved)), "resolved",
                    "nameCaptureInfo", c15_i5);
    sf_mex_addfield(c15_m0, sf_mex_create("nameCaptureInfo", &c15_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c15_i5);
    sf_mex_addfield(c15_m0, sf_mex_create("nameCaptureInfo", &c15_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c15_i5);
    sf_mex_addfield(c15_m0, sf_mex_create("nameCaptureInfo",
      &c15_r0->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c15_i5);
    sf_mex_addfield(c15_m0, sf_mex_create("nameCaptureInfo",
      &c15_r0->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c15_i5);
  }

  sf_mex_assign(&c15_nameCaptureInfo, c15_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c15_nameCaptureInfo);
  return c15_nameCaptureInfo;
}

static void c15_info_helper(c15_ResolvedFunctionInfo c15_info[20])
{
  c15_info[0].context = "";
  c15_info[0].name = "norm";
  c15_info[0].dominantType = "double";
  c15_info[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c15_info[0].fileTimeLo = 1336522094U;
  c15_info[0].fileTimeHi = 0U;
  c15_info[0].mFileTimeLo = 0U;
  c15_info[0].mFileTimeHi = 0U;
  c15_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c15_info[1].name = "eml_index_class";
  c15_info[1].dominantType = "";
  c15_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c15_info[1].fileTimeLo = 1323170578U;
  c15_info[1].fileTimeHi = 0U;
  c15_info[1].mFileTimeLo = 0U;
  c15_info[1].mFileTimeHi = 0U;
  c15_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c15_info[2].name = "eml_xnrm2";
  c15_info[2].dominantType = "double";
  c15_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c15_info[2].fileTimeLo = 1299076776U;
  c15_info[2].fileTimeHi = 0U;
  c15_info[2].mFileTimeLo = 0U;
  c15_info[2].mFileTimeHi = 0U;
  c15_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c15_info[3].name = "eml_blas_inline";
  c15_info[3].dominantType = "";
  c15_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c15_info[3].fileTimeLo = 1299076768U;
  c15_info[3].fileTimeHi = 0U;
  c15_info[3].mFileTimeLo = 0U;
  c15_info[3].mFileTimeHi = 0U;
  c15_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c15_info[4].name = "eml_index_class";
  c15_info[4].dominantType = "";
  c15_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c15_info[4].fileTimeLo = 1323170578U;
  c15_info[4].fileTimeHi = 0U;
  c15_info[4].mFileTimeLo = 0U;
  c15_info[4].mFileTimeHi = 0U;
  c15_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c15_info[5].name = "eml_refblas_xnrm2";
  c15_info[5].dominantType = "double";
  c15_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c15_info[5].fileTimeLo = 1299076784U;
  c15_info[5].fileTimeHi = 0U;
  c15_info[5].mFileTimeLo = 0U;
  c15_info[5].mFileTimeHi = 0U;
  c15_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c15_info[6].name = "realmin";
  c15_info[6].dominantType = "char";
  c15_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c15_info[6].fileTimeLo = 1307651242U;
  c15_info[6].fileTimeHi = 0U;
  c15_info[6].mFileTimeLo = 0U;
  c15_info[6].mFileTimeHi = 0U;
  c15_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c15_info[7].name = "eml_realmin";
  c15_info[7].dominantType = "char";
  c15_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c15_info[7].fileTimeLo = 1307651244U;
  c15_info[7].fileTimeHi = 0U;
  c15_info[7].mFileTimeLo = 0U;
  c15_info[7].mFileTimeHi = 0U;
  c15_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c15_info[8].name = "eml_float_model";
  c15_info[8].dominantType = "char";
  c15_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c15_info[8].fileTimeLo = 1326727996U;
  c15_info[8].fileTimeHi = 0U;
  c15_info[8].mFileTimeLo = 0U;
  c15_info[8].mFileTimeHi = 0U;
  c15_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c15_info[9].name = "eml_index_class";
  c15_info[9].dominantType = "";
  c15_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c15_info[9].fileTimeLo = 1323170578U;
  c15_info[9].fileTimeHi = 0U;
  c15_info[9].mFileTimeLo = 0U;
  c15_info[9].mFileTimeHi = 0U;
  c15_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c15_info[10].name = "eml_index_minus";
  c15_info[10].dominantType = "double";
  c15_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c15_info[10].fileTimeLo = 1286818778U;
  c15_info[10].fileTimeHi = 0U;
  c15_info[10].mFileTimeLo = 0U;
  c15_info[10].mFileTimeHi = 0U;
  c15_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c15_info[11].name = "eml_index_class";
  c15_info[11].dominantType = "";
  c15_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c15_info[11].fileTimeLo = 1323170578U;
  c15_info[11].fileTimeHi = 0U;
  c15_info[11].mFileTimeLo = 0U;
  c15_info[11].mFileTimeHi = 0U;
  c15_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c15_info[12].name = "eml_index_times";
  c15_info[12].dominantType = "coder.internal.indexInt";
  c15_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c15_info[12].fileTimeLo = 1286818780U;
  c15_info[12].fileTimeHi = 0U;
  c15_info[12].mFileTimeLo = 0U;
  c15_info[12].mFileTimeHi = 0U;
  c15_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c15_info[13].name = "eml_index_class";
  c15_info[13].dominantType = "";
  c15_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c15_info[13].fileTimeLo = 1323170578U;
  c15_info[13].fileTimeHi = 0U;
  c15_info[13].mFileTimeLo = 0U;
  c15_info[13].mFileTimeHi = 0U;
  c15_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c15_info[14].name = "eml_index_plus";
  c15_info[14].dominantType = "coder.internal.indexInt";
  c15_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c15_info[14].fileTimeLo = 1286818778U;
  c15_info[14].fileTimeHi = 0U;
  c15_info[14].mFileTimeLo = 0U;
  c15_info[14].mFileTimeHi = 0U;
  c15_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c15_info[15].name = "eml_index_class";
  c15_info[15].dominantType = "";
  c15_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c15_info[15].fileTimeLo = 1323170578U;
  c15_info[15].fileTimeHi = 0U;
  c15_info[15].mFileTimeLo = 0U;
  c15_info[15].mFileTimeHi = 0U;
  c15_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c15_info[16].name = "eml_int_forloop_overflow_check";
  c15_info[16].dominantType = "";
  c15_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c15_info[16].fileTimeLo = 1346510340U;
  c15_info[16].fileTimeHi = 0U;
  c15_info[16].mFileTimeLo = 0U;
  c15_info[16].mFileTimeHi = 0U;
  c15_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c15_info[17].name = "intmax";
  c15_info[17].dominantType = "char";
  c15_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c15_info[17].fileTimeLo = 1311255316U;
  c15_info[17].fileTimeHi = 0U;
  c15_info[17].mFileTimeLo = 0U;
  c15_info[17].mFileTimeHi = 0U;
  c15_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c15_info[18].name = "abs";
  c15_info[18].dominantType = "double";
  c15_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c15_info[18].fileTimeLo = 1343830366U;
  c15_info[18].fileTimeHi = 0U;
  c15_info[18].mFileTimeLo = 0U;
  c15_info[18].mFileTimeHi = 0U;
  c15_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c15_info[19].name = "eml_scalar_abs";
  c15_info[19].dominantType = "double";
  c15_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c15_info[19].fileTimeLo = 1286818712U;
  c15_info[19].fileTimeHi = 0U;
  c15_info[19].mFileTimeLo = 0U;
  c15_info[19].mFileTimeHi = 0U;
}

static const mxArray *c15_c_sf_marshallOut(void *chartInstanceVoid, void
  *c15_inData)
{
  const mxArray *c15_mxArrayOutData = NULL;
  int32_T c15_u;
  const mxArray *c15_y = NULL;
  SFc15_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc15_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c15_mxArrayOutData = NULL;
  c15_u = *(int32_T *)c15_inData;
  c15_y = NULL;
  sf_mex_assign(&c15_y, sf_mex_create("y", &c15_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c15_mxArrayOutData, c15_y, FALSE);
  return c15_mxArrayOutData;
}

static int32_T c15_c_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId)
{
  int32_T c15_y;
  int32_T c15_i6;
  sf_mex_import(c15_parentId, sf_mex_dup(c15_u), &c15_i6, 1, 6, 0U, 0, 0U, 0);
  c15_y = c15_i6;
  sf_mex_destroy(&c15_u);
  return c15_y;
}

static void c15_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c15_mxArrayInData, const char_T *c15_varName, void *c15_outData)
{
  const mxArray *c15_b_sfEvent;
  const char_T *c15_identifier;
  emlrtMsgIdentifier c15_thisId;
  int32_T c15_y;
  SFc15_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc15_torqueBalancing2012bInstanceStruct *)chartInstanceVoid;
  c15_b_sfEvent = sf_mex_dup(c15_mxArrayInData);
  c15_identifier = c15_varName;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_y = c15_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c15_b_sfEvent),
    &c15_thisId);
  sf_mex_destroy(&c15_b_sfEvent);
  *(int32_T *)c15_outData = c15_y;
  sf_mex_destroy(&c15_mxArrayInData);
}

static uint8_T c15_d_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_b_is_active_c15_torqueBalancing2012b, const
  char_T *c15_identifier)
{
  uint8_T c15_y;
  emlrtMsgIdentifier c15_thisId;
  c15_thisId.fIdentifier = c15_identifier;
  c15_thisId.fParent = NULL;
  c15_y = c15_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c15_b_is_active_c15_torqueBalancing2012b), &c15_thisId);
  sf_mex_destroy(&c15_b_is_active_c15_torqueBalancing2012b);
  return c15_y;
}

static uint8_T c15_e_emlrt_marshallIn(SFc15_torqueBalancing2012bInstanceStruct
  *chartInstance, const mxArray *c15_u, const emlrtMsgIdentifier *c15_parentId)
{
  uint8_T c15_y;
  uint8_T c15_u0;
  sf_mex_import(c15_parentId, sf_mex_dup(c15_u), &c15_u0, 1, 3, 0U, 0, 0U, 0);
  c15_y = c15_u0;
  sf_mex_destroy(&c15_u);
  return c15_y;
}

static void init_dsm_address_info(SFc15_torqueBalancing2012bInstanceStruct
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

void sf_c15_torqueBalancing2012b_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1211976118U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(81256891U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1495379496U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(826273398U);
}

mxArray *sf_c15_torqueBalancing2012b_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("XbXLi8aTXx8Iw3BntxBYdB");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c15_torqueBalancing2012b_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c15_torqueBalancing2012b(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"normU\",},{M[8],M[0],T\"is_active_c15_torqueBalancing2012b\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c15_torqueBalancing2012b_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc15_torqueBalancing2012bInstanceStruct *chartInstance;
    chartInstance = (SFc15_torqueBalancing2012bInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancing2012bMachineNumber_,
           15,
           1,
           1,
           2,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"u");
          _SFD_SET_DATA_PROPS(1,2,0,1,"normU");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,51);
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
            1.0,0,0,(MexFcnForType)c15_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c15_sf_marshallOut,(MexInFcnForType)c15_sf_marshallIn);

        {
          real_T *c15_normU;
          real_T (*c15_u)[6];
          c15_normU = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c15_u = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c15_u);
          _SFD_SET_DATA_VALUE_PTR(1U, c15_normU);
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
  return "QuYTs5VTfnJFG5EPvT3iHD";
}

static void sf_opaque_initialize_c15_torqueBalancing2012b(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc15_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c15_torqueBalancing2012b
    ((SFc15_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
  initialize_c15_torqueBalancing2012b((SFc15_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c15_torqueBalancing2012b(void *chartInstanceVar)
{
  enable_c15_torqueBalancing2012b((SFc15_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c15_torqueBalancing2012b(void *chartInstanceVar)
{
  disable_c15_torqueBalancing2012b((SFc15_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c15_torqueBalancing2012b(void *chartInstanceVar)
{
  sf_c15_torqueBalancing2012b((SFc15_torqueBalancing2012bInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c15_torqueBalancing2012b
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c15_torqueBalancing2012b
    ((SFc15_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c15_torqueBalancing2012b();/* state var info */
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

extern void sf_internal_set_sim_state_c15_torqueBalancing2012b(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c15_torqueBalancing2012b();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c15_torqueBalancing2012b
    ((SFc15_torqueBalancing2012bInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c15_torqueBalancing2012b(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c15_torqueBalancing2012b(S);
}

static void sf_opaque_set_sim_state_c15_torqueBalancing2012b(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c15_torqueBalancing2012b(S, st);
}

static void sf_opaque_terminate_c15_torqueBalancing2012b(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc15_torqueBalancing2012bInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing2012b_optimization_info();
    }

    finalize_c15_torqueBalancing2012b((SFc15_torqueBalancing2012bInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc15_torqueBalancing2012b
    ((SFc15_torqueBalancing2012bInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c15_torqueBalancing2012b(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c15_torqueBalancing2012b
      ((SFc15_torqueBalancing2012bInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c15_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing2012b_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      15);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,15,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,15,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,15);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,15,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,15,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 1; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,15);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1182012417U));
  ssSetChecksum1(S,(3718791996U));
  ssSetChecksum2(S,(969625412U));
  ssSetChecksum3(S,(1432740052U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c15_torqueBalancing2012b(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c15_torqueBalancing2012b(SimStruct *S)
{
  SFc15_torqueBalancing2012bInstanceStruct *chartInstance;
  chartInstance = (SFc15_torqueBalancing2012bInstanceStruct *)utMalloc(sizeof
    (SFc15_torqueBalancing2012bInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc15_torqueBalancing2012bInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c15_torqueBalancing2012b;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c15_torqueBalancing2012b;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c15_torqueBalancing2012b;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c15_torqueBalancing2012b;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c15_torqueBalancing2012b;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c15_torqueBalancing2012b;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c15_torqueBalancing2012b;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c15_torqueBalancing2012b;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c15_torqueBalancing2012b;
  chartInstance->chartInfo.mdlStart = mdlStart_c15_torqueBalancing2012b;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c15_torqueBalancing2012b;
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

void c15_torqueBalancing2012b_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c15_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c15_torqueBalancing2012b(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c15_torqueBalancing2012b(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c15_torqueBalancing2012b_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
