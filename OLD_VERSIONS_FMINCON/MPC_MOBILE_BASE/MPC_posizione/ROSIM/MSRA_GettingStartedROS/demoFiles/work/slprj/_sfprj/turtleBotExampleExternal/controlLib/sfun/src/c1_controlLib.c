/* Include files */

#include "controlLib_sfun.h"
#include "c1_controlLib.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "controlLib_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(S);
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

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)
#define c1_IN_NO_ACTIVE_CHILD          ((uint8_T)0U)
#define c1_IN_Abnormal                 ((uint8_T)1U)
#define c1_IN_Normal                   ((uint8_T)2U)
#define c1_IN_Scan                     ((uint8_T)1U)
#define c1_IN_Stop                     ((uint8_T)2U)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_b_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_c_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_d_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_e_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_f_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_g_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_h_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_i_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_j_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_k_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_l_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_m_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_n_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_o_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_p_debug_family_names[2] = { "nargin", "nargout" };

/* Function Declarations */
static void initialize_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance);
static void initialize_params_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance);
static void enable_c1_controlLib(SFc1_controlLibInstanceStruct *chartInstance);
static void disable_c1_controlLib(SFc1_controlLibInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance);
static void ext_mode_exec_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_controlLib(SFc1_controlLibInstanceStruct *
  chartInstance);
static void set_sim_state_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_st);
static void c1_set_sim_state_side_effects_c1_controlLib
  (SFc1_controlLibInstanceStruct *chartInstance);
static void finalize_c1_controlLib(SFc1_controlLibInstanceStruct *chartInstance);
static void sf_gateway_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance);
static void mdl_start_c1_controlLib(SFc1_controlLibInstanceStruct *chartInstance);
static void c1_chartstep_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance);
static void initSimStructsc1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance);
static void c1_TrackMarker(SFc1_controlLibInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_nargin, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static boolean_T c1_c_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_velControl(SFc1_controlLibInstanceStruct *chartInstance, real_T
  c1_d_x, real_T c1_b_bSize, real_T c1_b_iWidth, real_T c1_b_linGain, real_T
  c1_b_angGain, real_T *c1_c_v, real_T *c1_c_w);
static void c1_avgFilter(SFc1_controlLibInstanceStruct *chartInstance, real_T
  c1_d_x, real_T c1_b_s, real_T *c1_b_xf, real_T *c1_b_sf);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_d_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static uint8_T c1_e_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_b_is_TrackMarker, const char_T
  *c1_identifier);
static uint8_T c1_f_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_g_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  c1_struct_NQPZsPS5PAq52I8Ane3iuE *c1_y);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_h_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_b_dataWrittenToVector, const char_T *c1_identifier,
  boolean_T c1_y[6]);
static void c1_i_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, boolean_T c1_y[6]);
static const mxArray *c1_j_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_b_setSimStateSideEffectsInfo, const char_T
  *c1_identifier);
static const mxArray *c1_k_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static const mxArray *sf_get_hover_data_for_msg(SFc1_controlLibInstanceStruct
  *chartInstance, int32_T c1_ssid);
static void c1_init_sf_message_store_memory(SFc1_controlLibInstanceStruct
  *chartInstance);
static void init_dsm_address_info(SFc1_controlLibInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc1_controlLibInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_controlLib(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  sim_mode_is_external(chartInstance->S);
  *chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_doSetSimStateSideEffects = 0U;
  chartInstance->c1_setSimStateSideEffectsInfo = NULL;
  *chartInstance->c1_is_active_DetectOutliers = 0U;
  *chartInstance->c1_is_active_TrackMarker = 0U;
  *chartInstance->c1_is_TrackMarker = c1_IN_NO_ACTIVE_CHILD;
  *chartInstance->c1_is_Abnormal = c1_IN_NO_ACTIVE_CHILD;
  *chartInstance->c1_is_active_c1_controlLib = 0U;
  setLegacyDebuggerFlagForRuntime(chartInstance->S, true);
}

static void initialize_params_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  const mxArray *c1_m0 = NULL;
  static const char * c1_fieldNames[14] = { "Ts", "bufSize", "maxDisp",
    "minSize", "maxSize", "maxCounts", "linVelGain", "angVelGain", "maxLinVel",
    "maxAngVel", "posDeadZone", "targetSize", "sizeDeadZone", "speedRedSize" };

  const mxArray *c1_mxField;
  c1_struct_NQPZsPS5PAq52I8Ane3iuE c1_r0;
  c1_m0 = sf_mex_get_sfun_param(chartInstance->S, 0U, 1U);
  sf_mex_check_bus_parameter(c1_m0, 0, NULL, 14, c1_fieldNames, "params",
    "struct_NQPZsPS5PAq52I8Ane3iuE");
  c1_mxField = sf_mex_getfield(c1_m0, "Ts", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.Ts, 1, 0, 0U, 0,
                      0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "bufSize", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.bufSize, 1, 0, 0U,
                      0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "maxDisp", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.maxDisp, 1, 0, 0U,
                      0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "minSize", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.minSize, 1, 0, 0U,
                      0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "maxSize", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.maxSize, 1, 0, 0U,
                      0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "maxCounts", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.maxCounts, 1, 0,
                      0U, 0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "linVelGain", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.linVelGain, 1, 0,
                      0U, 0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "angVelGain", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.angVelGain, 1, 0,
                      0U, 0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "maxLinVel", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.maxLinVel, 1, 0,
                      0U, 0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "maxAngVel", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.maxAngVel, 1, 0,
                      0U, 0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "posDeadZone", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.posDeadZone, 1, 0,
                      0U, 0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "targetSize", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.targetSize, 1, 0,
                      0U, 0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "sizeDeadZone", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.sizeDeadZone, 1,
                      0, 0U, 0, 0U, 0);
  c1_mxField = sf_mex_getfield(c1_m0, "speedRedSize", "params", 0);
  sf_mex_import_named("params", sf_mex_dup(c1_mxField), &c1_r0.speedRedSize, 1,
                      0, 0U, 0, 0U, 0);
  sf_mex_destroy(&c1_m0);
  chartInstance->c1_params = c1_r0;
}

static void enable_c1_controlLib(SFc1_controlLibInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
  sf_call_output_fcn_enable(chartInstance->S, 0, "velControl", 0);
  sf_call_output_fcn_enable(chartInstance->S, 1, "avgFilter", 0);
}

static void disable_c1_controlLib(SFc1_controlLibInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
  sf_call_output_fcn_disable(chartInstance->S, 0, "velControl", 0);
  sf_call_output_fcn_disable(chartInstance->S, 1, "avgFilter", 0);
}

static void c1_update_debugger_state_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  uint32_T c1_prevAniVal;
  c1_prevAniVal = _SFD_GET_ANIMATION();
  _SFD_SET_ANIMATION(0U);
  _SFD_SET_HONOR_BREAKPOINTS(0U);
  if (*chartInstance->c1_is_active_c1_controlLib == 1U) {
    _SFD_CC_CALL(CHART_ACTIVE_TAG, 0U, *chartInstance->c1_sfEvent);
  }

  if (*chartInstance->c1_is_active_TrackMarker == 1U) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, *chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, *chartInstance->c1_sfEvent);
  }

  if (*chartInstance->c1_is_TrackMarker == c1_IN_Normal) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, *chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, *chartInstance->c1_sfEvent);
  }

  if (*chartInstance->c1_is_TrackMarker == c1_IN_Abnormal) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, *chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, *chartInstance->c1_sfEvent);
  }

  if (*chartInstance->c1_is_Abnormal == c1_IN_Stop) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, *chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, *chartInstance->c1_sfEvent);
  }

  if (*chartInstance->c1_is_Abnormal == c1_IN_Scan) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
  }

  if (*chartInstance->c1_is_active_DetectOutliers == 1U) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, *chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, *chartInstance->c1_sfEvent);
  }

  _SFD_SET_ANIMATION(c1_prevAniVal);
  _SFD_SET_HONOR_BREAKPOINTS(1U);
  _SFD_ANIMATE();
}

static void ext_mode_exec_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  c1_update_debugger_state_c1_controlLib(chartInstance);
}

static const mxArray *get_sim_state_c1_controlLib(SFc1_controlLibInstanceStruct *
  chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_hoistedGlobal;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_hoistedGlobal;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_hoistedGlobal;
  const mxArray *c1_g_y = NULL;
  uint8_T c1_g_hoistedGlobal;
  const mxArray *c1_h_y = NULL;
  uint8_T c1_h_hoistedGlobal;
  const mxArray *c1_i_y = NULL;
  uint8_T c1_i_hoistedGlobal;
  const mxArray *c1_j_y = NULL;
  uint8_T c1_j_hoistedGlobal;
  const mxArray *c1_k_y = NULL;
  uint8_T c1_k_hoistedGlobal;
  const mxArray *c1_l_y = NULL;
  const mxArray *c1_m_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(12, 1), false);
  c1_hoistedGlobal = *chartInstance->c1_v;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_hoistedGlobal, 0, 0U, 0U, 0U, 0),
                false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *chartInstance->c1_w;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_hoistedGlobal, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = *chartInstance->c1_count;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_hoistedGlobal, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = *chartInstance->c1_sizeFilt;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_hoistedGlobal, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_e_hoistedGlobal = *chartInstance->c1_xFilt;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_hoistedGlobal, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_f_hoistedGlobal = *chartInstance->c1_xPrev;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_hoistedGlobal, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_g_hoistedGlobal = *chartInstance->c1_is_active_c1_controlLib;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_hoistedGlobal, 3, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 6, c1_h_y);
  c1_h_hoistedGlobal = *chartInstance->c1_is_active_DetectOutliers;
  c1_i_y = NULL;
  sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_h_hoistedGlobal, 3, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 7, c1_i_y);
  c1_i_hoistedGlobal = *chartInstance->c1_is_active_TrackMarker;
  c1_j_y = NULL;
  sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_i_hoistedGlobal, 3, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 8, c1_j_y);
  c1_j_hoistedGlobal = *chartInstance->c1_is_TrackMarker;
  c1_k_y = NULL;
  sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_j_hoistedGlobal, 3, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 9, c1_k_y);
  c1_k_hoistedGlobal = *chartInstance->c1_is_Abnormal;
  c1_l_y = NULL;
  sf_mex_assign(&c1_l_y, sf_mex_create("y", &c1_k_hoistedGlobal, 3, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 10, c1_l_y);
  c1_m_y = NULL;
  sf_mex_assign(&c1_m_y, sf_mex_create("y",
    chartInstance->c1_dataWrittenToVector, 11, 0U, 1U, 0U, 1, 6), false);
  sf_mex_setcell(c1_y, 11, c1_m_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  boolean_T c1_bv0[6];
  int32_T c1_i0;
  c1_u = sf_mex_dup(c1_st);
  *chartInstance->c1_v = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 0)), "v");
  *chartInstance->c1_w = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 1)), "w");
  *chartInstance->c1_count = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 2)), "count");
  *chartInstance->c1_sizeFilt = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 3)), "sizeFilt");
  *chartInstance->c1_xFilt = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 4)), "xFilt");
  *chartInstance->c1_xPrev = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 5)), "xPrev");
  *chartInstance->c1_is_active_c1_controlLib = c1_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 6)),
     "is_active_c1_controlLib");
  *chartInstance->c1_is_active_DetectOutliers = c1_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 7)),
     "is_active_DetectOutliers");
  *chartInstance->c1_is_active_TrackMarker = c1_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 8)), "is_active_TrackMarker");
  *chartInstance->c1_is_TrackMarker = c1_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 9)), "is_TrackMarker");
  *chartInstance->c1_is_Abnormal = c1_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 10)), "is_Abnormal");
  c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 11)),
                        "dataWrittenToVector", c1_bv0);
  for (c1_i0 = 0; c1_i0 < 6; c1_i0++) {
    chartInstance->c1_dataWrittenToVector[c1_i0] = c1_bv0[c1_i0];
  }

  sf_mex_assign(&chartInstance->c1_setSimStateSideEffectsInfo,
                c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 12)), "setSimStateSideEffectsInfo"), true);
  sf_mex_destroy(&c1_u);
  chartInstance->c1_doSetSimStateSideEffects = 1U;
  c1_update_debugger_state_c1_controlLib(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void c1_set_sim_state_side_effects_c1_controlLib
  (SFc1_controlLibInstanceStruct *chartInstance)
{
  if (chartInstance->c1_doSetSimStateSideEffects != 0) {
    chartInstance->c1_doSetSimStateSideEffects = 0U;
  }
}

static void finalize_c1_controlLib(SFc1_controlLibInstanceStruct *chartInstance)
{
  sf_mex_destroy(&chartInstance->c1_setSimStateSideEffectsInfo);
}

static void sf_gateway_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  c1_set_sim_state_side_effects_c1_controlLib(chartInstance);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0, *chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_angVelGain, 8U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_linVelGain, 7U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_imgWidth, 6U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_blobSize, 5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_x, 4U);
  *chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_controlLib(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
}

static void mdl_start_c1_controlLib(SFc1_controlLibInstanceStruct *chartInstance)
{
  c1_init_sf_message_store_memory(chartInstance);
  sim_mode_is_external(chartInstance->S);
}

static void c1_chartstep_c1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  uint32_T c1_debug_family_var_map[3];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 1.0;
  boolean_T c1_out;
  uint32_T c1_b_debug_family_var_map[2];
  real_T c1_b_nargin = 0.0;
  real_T c1_b_nargout = 0.0;
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_d_x;
  real_T c1_e_x;
  real_T c1_f_x;
  real_T c1_y;
  c1_struct_NQPZsPS5PAq52I8Ane3iuE c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  real_T c1_e_hoistedGlobal;
  uint32_T c1_c_debug_family_var_map[3];
  real_T c1_g_x;
  real_T c1_c_nargin = 0.0;
  real_T c1_h_x;
  real_T c1_c_nargout = 1.0;
  real_T c1_i_x;
  boolean_T c1_b_out;
  real_T c1_b_y;
  real_T c1_f_hoistedGlobal;
  real_T c1_g_hoistedGlobal;
  real_T c1_j_x;
  real_T c1_k_x;
  real_T c1_l_x;
  real_T c1_c_y;
  real_T c1_h_hoistedGlobal;
  real_T c1_i_hoistedGlobal;
  uint32_T c1_d_debug_family_var_map[2];
  real_T c1_m_x;
  real_T c1_d_nargin = 0.0;
  real_T c1_n_x;
  real_T c1_d_nargout = 0.0;
  uint32_T c1_e_debug_family_var_map[3];
  real_T c1_o_x;
  real_T c1_e_nargin = 0.0;
  real_T c1_d_y;
  real_T c1_e_nargout = 1.0;
  boolean_T c1_c_out;
  uint32_T c1_f_debug_family_var_map[2];
  real_T c1_f_nargin = 0.0;
  real_T c1_f_nargout = 0.0;
  uint32_T c1_g_debug_family_var_map[2];
  uint32_T c1_h_debug_family_var_map[2];
  real_T c1_g_nargin = 0.0;
  real_T c1_h_nargin = 0.0;
  real_T c1_g_nargout = 0.0;
  uint32_T c1_i_debug_family_var_map[3];
  real_T c1_h_nargout = 0.0;
  real_T c1_i_nargin = 0.0;
  real_T c1_i_nargout = 1.0;
  boolean_T c1_d_out;
  uint32_T c1_j_debug_family_var_map[2];
  real_T c1_j_nargin = 0.0;
  real_T c1_j_nargout = 0.0;
  uint32_T c1_k_debug_family_var_map[2];
  real_T c1_k_nargin = 0.0;
  real_T c1_k_nargout = 0.0;
  uint32_T c1_l_debug_family_var_map[2];
  real_T c1_l_nargin = 0.0;
  real_T c1_l_nargout = 0.0;
  real_T c1_b_xFilt;
  real_T c1_b_sizeFilt;
  uint32_T c1_m_debug_family_var_map[2];
  real_T c1_m_nargin = 0.0;
  real_T c1_m_nargout = 0.0;
  real_T c1_c_v;
  real_T c1_c_w;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, *chartInstance->c1_sfEvent);
  if (*chartInstance->c1_is_active_c1_controlLib == 0U) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG, 0U, *chartInstance->c1_sfEvent);
    *chartInstance->c1_is_active_c1_controlLib = 1U;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, *chartInstance->c1_sfEvent);
    *chartInstance->c1_is_active_DetectOutliers = 1U;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, *chartInstance->c1_sfEvent);
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 0U, *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_k_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *chartInstance->c1_count = 0.0;
    chartInstance->c1_dataWrittenToVector[4U] = true;
    _SFD_DATA_RANGE_CHECK(*chartInstance->c1_count, 0U);
    *chartInstance->c1_xPrev = 0.0;
    chartInstance->c1_dataWrittenToVector[5U] = true;
    _SFD_DATA_RANGE_CHECK(*chartInstance->c1_xPrev, 3U);
    _SFD_SYMBOL_SCOPE_POP();
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 1U,
                 *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_l_debug_family_names,
      c1_c_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    if (!chartInstance->c1_dataWrittenToVector[5U]) {
      sf_read_before_write_error(chartInstance->S, 3U, 17U, 26, 5);
    }

    c1_f_hoistedGlobal = *chartInstance->c1_x;
    c1_g_hoistedGlobal = *chartInstance->c1_xPrev;
    c1_j_x = c1_f_hoistedGlobal - c1_g_hoistedGlobal;
    c1_k_x = c1_j_x;
    c1_l_x = c1_k_x;
    c1_c_y = muDoubleScalarAbs(c1_l_x);
    c1_c_hoistedGlobal = chartInstance->c1_params;
    c1_h_hoistedGlobal = *chartInstance->c1_x;
    c1_i_hoistedGlobal = *chartInstance->c1_xPrev;
    c1_m_x = c1_h_hoistedGlobal - c1_i_hoistedGlobal;
    c1_n_x = c1_m_x;
    c1_o_x = c1_n_x;
    c1_d_y = muDoubleScalarAbs(c1_o_x);
    if (CV_EML_COND(1, 0, 0, CV_RELATIONAL_EVAL(5U, 1U, 0, c1_c_y,
          c1_c_hoistedGlobal.maxDisp, -1, 5U, c1_d_y >=
          chartInstance->c1_params.maxDisp)) || CV_EML_COND(1, 0, 1,
         CV_RELATIONAL_EVAL(5U, 1U, 1, *chartInstance->c1_blobSize,
                            chartInstance->c1_params.minSize, -1, 3U,
                            *chartInstance->c1_blobSize <=
                            chartInstance->c1_params.minSize)) || CV_EML_COND(1,
         0, 2, CV_RELATIONAL_EVAL(5U, 1U, 2, *chartInstance->c1_blobSize,
          chartInstance->c1_params.maxSize, -1, 5U, *chartInstance->c1_blobSize >=
          chartInstance->c1_params.maxSize))) {
      CV_EML_MCDC(1, 0, 0, true);
      CV_EML_IF(1, 0, 0, true);
      c1_b_out = true;
    } else {
      CV_EML_MCDC(1, 0, 0, false);
      CV_EML_IF(1, 0, 0, false);
      c1_b_out = false;
    }

    _SFD_SYMBOL_SCOPE_POP();
    if (c1_b_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 1U, *chartInstance->c1_sfEvent);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 4U, *chartInstance->c1_sfEvent);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 7U,
                   *chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_n_debug_family_names,
        c1_i_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_out, 2U, c1_b_sf_marshallOut,
        c1_b_sf_marshallIn);
      if (!chartInstance->c1_dataWrittenToVector[4U]) {
        sf_read_before_write_error(chartInstance->S, 0U, 28U, 1, 5);
      }

      if (CV_EML_IF(7, 0, 0, CV_RELATIONAL_EVAL(5U, 7U, 0,
            *chartInstance->c1_count, 1000.0, -1, 2U, *chartInstance->c1_count <
            1000.0))) {
        c1_d_out = true;
      } else {
        c1_d_out = false;
      }

      _SFD_SYMBOL_SCOPE_POP();
      if (c1_d_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 7U, *chartInstance->c1_sfEvent);
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 8U, *chartInstance->c1_sfEvent);
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_o_debug_family_names,
          c1_k_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_k_nargin, 0U, c1_sf_marshallOut,
          c1_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_k_nargout, 1U,
          c1_sf_marshallOut, c1_sf_marshallIn);
        if (!chartInstance->c1_dataWrittenToVector[4U]) {
          sf_read_before_write_error(chartInstance->S, 0U, 30U, 7, 5);
        }

        (*chartInstance->c1_count)++;
        chartInstance->c1_dataWrittenToVector[4U] = true;
        _SFD_DATA_RANGE_CHECK(*chartInstance->c1_count, 0U);
        _SFD_SYMBOL_SCOPE_POP();
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 9U, *chartInstance->c1_sfEvent);
      } else {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 6U, *chartInstance->c1_sfEvent);
      }

      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 5U, *chartInstance->c1_sfEvent);
    } else {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_m_debug_family_names,
        c1_g_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *chartInstance->c1_count = 0.0;
      chartInstance->c1_dataWrittenToVector[4U] = true;
      _SFD_DATA_RANGE_CHECK(*chartInstance->c1_count, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 11U, *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_p_debug_family_names,
      c1_j_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_j_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_j_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *chartInstance->c1_xPrev = *chartInstance->c1_x;
    chartInstance->c1_dataWrittenToVector[5U] = true;
    _SFD_DATA_RANGE_CHECK(*chartInstance->c1_xPrev, 3U);
    _SFD_SYMBOL_SCOPE_POP();
    *chartInstance->c1_is_active_TrackMarker = 1U;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_debug_family_names,
      c1_l_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_l_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_l_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    c1_avgFilter(chartInstance, *chartInstance->c1_x,
                 *chartInstance->c1_blobSize, &c1_b_xFilt, &c1_b_sizeFilt);
    *chartInstance->c1_xFilt = c1_b_xFilt;
    chartInstance->c1_dataWrittenToVector[3U] = true;
    _SFD_DATA_RANGE_CHECK(*chartInstance->c1_xFilt, 2U);
    *chartInstance->c1_sizeFilt = c1_b_sizeFilt;
    chartInstance->c1_dataWrittenToVector[2U] = true;
    _SFD_DATA_RANGE_CHECK(*chartInstance->c1_sizeFilt, 1U);
    _SFD_SYMBOL_SCOPE_POP();
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 10U, *chartInstance->c1_sfEvent);
    *chartInstance->c1_is_TrackMarker = c1_IN_Normal;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_c_debug_family_names,
      c1_m_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_m_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_m_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    if (!chartInstance->c1_dataWrittenToVector[3U]) {
      sf_read_before_write_error(chartInstance->S, 2U, 50U, 42, 5);
    }

    if (!chartInstance->c1_dataWrittenToVector[2U]) {
      sf_read_before_write_error(chartInstance->S, 1U, 50U, 48, 8);
    }

    c1_velControl(chartInstance, *chartInstance->c1_xFilt,
                  *chartInstance->c1_sizeFilt, *chartInstance->c1_imgWidth,
                  *chartInstance->c1_linVelGain, *chartInstance->c1_angVelGain,
                  &c1_c_v, &c1_c_w);
    *chartInstance->c1_v = c1_c_v;
    chartInstance->c1_dataWrittenToVector[0U] = true;
    _SFD_DATA_RANGE_CHECK(*chartInstance->c1_v, 9U);
    *chartInstance->c1_w = c1_c_w;
    chartInstance->c1_dataWrittenToVector[1U] = true;
    _SFD_DATA_RANGE_CHECK(*chartInstance->c1_w, 10U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 0U, *chartInstance->c1_sfEvent);
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 2U, *chartInstance->c1_sfEvent);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 1U,
                 *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_l_debug_family_names,
      c1_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    if (!chartInstance->c1_dataWrittenToVector[5U]) {
      sf_read_before_write_error(chartInstance->S, 3U, 17U, 26, 5);
    }

    c1_hoistedGlobal = *chartInstance->c1_x;
    c1_b_hoistedGlobal = *chartInstance->c1_xPrev;
    c1_d_x = c1_hoistedGlobal - c1_b_hoistedGlobal;
    c1_e_x = c1_d_x;
    c1_f_x = c1_e_x;
    c1_y = muDoubleScalarAbs(c1_f_x);
    c1_c_hoistedGlobal = chartInstance->c1_params;
    c1_d_hoistedGlobal = *chartInstance->c1_x;
    c1_e_hoistedGlobal = *chartInstance->c1_xPrev;
    c1_g_x = c1_d_hoistedGlobal - c1_e_hoistedGlobal;
    c1_h_x = c1_g_x;
    c1_i_x = c1_h_x;
    c1_b_y = muDoubleScalarAbs(c1_i_x);
    if (CV_EML_COND(1, 0, 0, CV_RELATIONAL_EVAL(5U, 1U, 0, c1_y,
          c1_c_hoistedGlobal.maxDisp, -1, 5U, c1_b_y >=
          chartInstance->c1_params.maxDisp)) || CV_EML_COND(1, 0, 1,
         CV_RELATIONAL_EVAL(5U, 1U, 1, *chartInstance->c1_blobSize,
                            chartInstance->c1_params.minSize, -1, 3U,
                            *chartInstance->c1_blobSize <=
                            chartInstance->c1_params.minSize)) || CV_EML_COND(1,
         0, 2, CV_RELATIONAL_EVAL(5U, 1U, 2, *chartInstance->c1_blobSize,
          chartInstance->c1_params.maxSize, -1, 5U, *chartInstance->c1_blobSize >=
          chartInstance->c1_params.maxSize))) {
      CV_EML_MCDC(1, 0, 0, true);
      CV_EML_IF(1, 0, 0, true);
      c1_out = true;
    } else {
      CV_EML_MCDC(1, 0, 0, false);
      CV_EML_IF(1, 0, 0, false);
      c1_out = false;
    }

    _SFD_SYMBOL_SCOPE_POP();
    if (c1_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 1U, *chartInstance->c1_sfEvent);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 4U, *chartInstance->c1_sfEvent);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 7U,
                   *chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_n_debug_family_names,
        c1_e_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_e_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_e_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_out, 2U, c1_b_sf_marshallOut,
        c1_b_sf_marshallIn);
      if (!chartInstance->c1_dataWrittenToVector[4U]) {
        sf_read_before_write_error(chartInstance->S, 0U, 28U, 1, 5);
      }

      if (CV_EML_IF(7, 0, 0, CV_RELATIONAL_EVAL(5U, 7U, 0,
            *chartInstance->c1_count, 1000.0, -1, 2U, *chartInstance->c1_count <
            1000.0))) {
        c1_c_out = true;
      } else {
        c1_c_out = false;
      }

      _SFD_SYMBOL_SCOPE_POP();
      if (c1_c_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 7U, *chartInstance->c1_sfEvent);
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 8U, *chartInstance->c1_sfEvent);
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_o_debug_family_names,
          c1_h_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_h_nargin, 0U, c1_sf_marshallOut,
          c1_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_h_nargout, 1U,
          c1_sf_marshallOut, c1_sf_marshallIn);
        if (!chartInstance->c1_dataWrittenToVector[4U]) {
          sf_read_before_write_error(chartInstance->S, 0U, 30U, 7, 5);
        }

        (*chartInstance->c1_count)++;
        chartInstance->c1_dataWrittenToVector[4U] = true;
        _SFD_DATA_RANGE_CHECK(*chartInstance->c1_count, 0U);
        _SFD_SYMBOL_SCOPE_POP();
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 9U, *chartInstance->c1_sfEvent);
      } else {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 6U, *chartInstance->c1_sfEvent);
      }

      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 5U, *chartInstance->c1_sfEvent);
    } else {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_m_debug_family_names,
        c1_d_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *chartInstance->c1_count = 0.0;
      chartInstance->c1_dataWrittenToVector[4U] = true;
      _SFD_DATA_RANGE_CHECK(*chartInstance->c1_count, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 11U, *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_p_debug_family_names,
      c1_f_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_f_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_f_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *chartInstance->c1_xPrev = *chartInstance->c1_x;
    chartInstance->c1_dataWrittenToVector[5U] = true;
    _SFD_DATA_RANGE_CHECK(*chartInstance->c1_xPrev, 3U);
    _SFD_SYMBOL_SCOPE_POP();
    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, *chartInstance->c1_sfEvent);
    c1_TrackMarker(chartInstance);
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, *chartInstance->c1_sfEvent);
}

static void initSimStructsc1_controlLib(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c1_TrackMarker(SFc1_controlLibInstanceStruct *chartInstance)
{
  uint32_T c1_debug_family_var_map[2];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 0.0;
  real_T c1_b_xFilt;
  real_T c1_b_sizeFilt;
  uint32_T c1_b_debug_family_var_map[3];
  uint32_T c1_c_debug_family_var_map[3];
  real_T c1_b_nargin = 0.0;
  real_T c1_c_nargin = 0.0;
  real_T c1_b_nargout = 1.0;
  real_T c1_c_nargout = 1.0;
  boolean_T c1_out;
  boolean_T c1_b_out;
  uint32_T c1_d_debug_family_var_map[2];
  real_T c1_d_nargin = 0.0;
  real_T c1_d_nargout = 0.0;
  uint32_T c1_e_debug_family_var_map[3];
  uint32_T c1_f_debug_family_var_map[3];
  real_T c1_e_nargin = 0.0;
  real_T c1_f_nargin = 0.0;
  real_T c1_e_nargout = 1.0;
  real_T c1_f_nargout = 1.0;
  boolean_T c1_c_out;
  boolean_T c1_d_out;
  real_T c1_c_v;
  real_T c1_c_w;
  uint32_T c1_g_debug_family_var_map[2];
  real_T c1_g_nargin = 0.0;
  uint32_T c1_h_debug_family_var_map[2];
  real_T c1_g_nargout = 0.0;
  real_T c1_h_nargin = 0.0;
  real_T c1_h_nargout = 0.0;
  real_T c1_d_v;
  real_T c1_d_w;
  uint32_T c1_i_debug_family_var_map[2];
  real_T c1_i_nargin = 0.0;
  real_T c1_i_nargout = 0.0;
  uint32_T c1_j_debug_family_var_map[2];
  real_T c1_j_nargin = 0.0;
  real_T c1_j_nargout = 0.0;
  boolean_T guard1 = false;
  _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 1U, *chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_b_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  c1_avgFilter(chartInstance, *chartInstance->c1_x, *chartInstance->c1_blobSize,
               &c1_b_xFilt, &c1_b_sizeFilt);
  *chartInstance->c1_xFilt = c1_b_xFilt;
  chartInstance->c1_dataWrittenToVector[3U] = true;
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_xFilt, 2U);
  *chartInstance->c1_sizeFilt = c1_b_sizeFilt;
  chartInstance->c1_dataWrittenToVector[2U] = true;
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_sizeFilt, 1U);
  _SFD_SYMBOL_SCOPE_POP();
  switch (*chartInstance->c1_is_TrackMarker) {
   case c1_IN_Abnormal:
    CV_STATE_EVAL(1, 0, c1_IN_Abnormal);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 16U,
                 *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_j_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    if (!chartInstance->c1_dataWrittenToVector[4U]) {
      sf_read_before_write_error(chartInstance->S, 0U, 104U, 1, 5);
    }

    if (CV_EML_IF(16, 0, 0, CV_RELATIONAL_EVAL(5U, 16U, 0,
          *chartInstance->c1_count, chartInstance->c1_params.maxCounts, -1, 2U, *
          chartInstance->c1_count < chartInstance->c1_params.maxCounts))) {
      c1_out = true;
    } else {
      c1_out = false;
    }

    _SFD_SYMBOL_SCOPE_POP();
    if (c1_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 16U, *chartInstance->c1_sfEvent);
      switch (*chartInstance->c1_is_Abnormal) {
       case c1_IN_Scan:
        CV_STATE_EVAL(2, 1, c1_IN_Scan);
        *chartInstance->c1_is_Abnormal = c1_IN_NO_ACTIVE_CHILD;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
        break;

       case c1_IN_Stop:
        CV_STATE_EVAL(2, 1, c1_IN_Stop);
        *chartInstance->c1_is_Abnormal = c1_IN_NO_ACTIVE_CHILD;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, *chartInstance->c1_sfEvent);
        break;

       default:
        CV_STATE_EVAL(2, 1, 0);
        *chartInstance->c1_is_Abnormal = c1_IN_NO_ACTIVE_CHILD;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
        break;
      }

      *chartInstance->c1_is_TrackMarker = c1_IN_NO_ACTIVE_CHILD;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, *chartInstance->c1_sfEvent);
      *chartInstance->c1_is_TrackMarker = c1_IN_Normal;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, *chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_c_debug_family_names,
        c1_h_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_h_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_h_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      if (!chartInstance->c1_dataWrittenToVector[3U]) {
        sf_read_before_write_error(chartInstance->S, 2U, 50U, 42, 5);
      }

      if (!chartInstance->c1_dataWrittenToVector[2U]) {
        sf_read_before_write_error(chartInstance->S, 1U, 50U, 48, 8);
      }

      c1_velControl(chartInstance, *chartInstance->c1_xFilt,
                    *chartInstance->c1_sizeFilt, *chartInstance->c1_imgWidth,
                    *chartInstance->c1_linVelGain, *chartInstance->c1_angVelGain,
                    &c1_d_v, &c1_d_w);
      *chartInstance->c1_v = c1_d_v;
      chartInstance->c1_dataWrittenToVector[0U] = true;
      _SFD_DATA_RANGE_CHECK(*chartInstance->c1_v, 9U);
      *chartInstance->c1_w = c1_d_w;
      chartInstance->c1_dataWrittenToVector[1U] = true;
      _SFD_DATA_RANGE_CHECK(*chartInstance->c1_w, 10U);
      _SFD_SYMBOL_SCOPE_POP();
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 2U,
                   *chartInstance->c1_sfEvent);
      switch (*chartInstance->c1_is_Abnormal) {
       case c1_IN_Scan:
        CV_STATE_EVAL(2, 0, c1_IN_Scan);
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 14U,
                     *chartInstance->c1_sfEvent);
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_h_debug_family_names,
          c1_e_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_e_nargin, 0U, c1_sf_marshallOut,
          c1_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_e_nargout, 1U,
          c1_sf_marshallOut, c1_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_out, 2U, c1_b_sf_marshallOut,
          c1_b_sf_marshallIn);
        if (!chartInstance->c1_dataWrittenToVector[4U]) {
          sf_read_before_write_error(chartInstance->S, 0U, 58U, 1, 5);
        }

        if (CV_EML_IF(14, 0, 0, CV_RELATIONAL_EVAL(5U, 14U, 0,
              *chartInstance->c1_count, 500.0, -1, 5U, *chartInstance->c1_count >=
              500.0))) {
          c1_c_out = true;
        } else {
          c1_c_out = false;
        }

        _SFD_SYMBOL_SCOPE_POP();
        if (c1_c_out) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 14U, *chartInstance->c1_sfEvent);
          *chartInstance->c1_is_Abnormal = c1_IN_NO_ACTIVE_CHILD;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
          *chartInstance->c1_is_Abnormal = c1_IN_Stop;
          _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, *chartInstance->c1_sfEvent);
          _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_e_debug_family_names,
            c1_i_debug_family_var_map);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i_nargin, 0U,
            c1_sf_marshallOut, c1_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i_nargout, 1U,
            c1_sf_marshallOut, c1_sf_marshallIn);
          *chartInstance->c1_v = 0.0;
          chartInstance->c1_dataWrittenToVector[0U] = true;
          _SFD_DATA_RANGE_CHECK(*chartInstance->c1_v, 9U);
          *chartInstance->c1_w = 0.0;
          chartInstance->c1_dataWrittenToVector[1U] = true;
          _SFD_DATA_RANGE_CHECK(*chartInstance->c1_w, 10U);
          _SFD_SYMBOL_SCOPE_POP();
        } else {
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 3U,
                       *chartInstance->c1_sfEvent);
        }

        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, *chartInstance->c1_sfEvent);
        break;

       case c1_IN_Stop:
        CV_STATE_EVAL(2, 0, c1_IN_Stop);
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 13U,
                     *chartInstance->c1_sfEvent);
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_g_debug_family_names,
          c1_f_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_f_nargin, 0U, c1_sf_marshallOut,
          c1_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_f_nargout, 1U,
          c1_sf_marshallOut, c1_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_out, 2U, c1_b_sf_marshallOut,
          c1_b_sf_marshallIn);
        if (!chartInstance->c1_dataWrittenToVector[4U]) {
          sf_read_before_write_error(chartInstance->S, 0U, 57U, 1, 5);
        }

        guard1 = false;
        if (CV_EML_COND(13, 0, 0, CV_RELATIONAL_EVAL(5U, 13U, 0,
              *chartInstance->c1_count, 50.0, -1, 5U, *chartInstance->c1_count >=
              50.0))) {
          if (CV_EML_COND(13, 0, 1, CV_RELATIONAL_EVAL(5U, 13U, 1,
                *chartInstance->c1_count, 500.0, -1, 3U,
                *chartInstance->c1_count <= 500.0))) {
            CV_EML_MCDC(13, 0, 0, true);
            CV_EML_IF(13, 0, 0, true);
            c1_d_out = true;
          } else {
            guard1 = true;
          }
        } else {
          if (!chartInstance->c1_dataWrittenToVector[4U]) {
            sf_read_before_write_error(chartInstance->S, 0U, 57U, 20, 5);
          }

          guard1 = true;
        }

        if (guard1) {
          CV_EML_MCDC(13, 0, 0, false);
          CV_EML_IF(13, 0, 0, false);
          c1_d_out = false;
        }

        _SFD_SYMBOL_SCOPE_POP();
        if (c1_d_out) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 13U, *chartInstance->c1_sfEvent);
          *chartInstance->c1_is_Abnormal = c1_IN_NO_ACTIVE_CHILD;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, *chartInstance->c1_sfEvent);
          *chartInstance->c1_is_Abnormal = c1_IN_Scan;
          _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
          _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_f_debug_family_names,
            c1_j_debug_family_var_map);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_j_nargin, 0U,
            c1_sf_marshallOut, c1_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_j_nargout, 1U,
            c1_sf_marshallOut, c1_sf_marshallIn);
          *chartInstance->c1_v = 0.0;
          chartInstance->c1_dataWrittenToVector[0U] = true;
          _SFD_DATA_RANGE_CHECK(*chartInstance->c1_v, 9U);
          *chartInstance->c1_w = chartInstance->c1_params.maxAngVel / 2.0;
          chartInstance->c1_dataWrittenToVector[1U] = true;
          _SFD_DATA_RANGE_CHECK(*chartInstance->c1_w, 10U);
          _SFD_SYMBOL_SCOPE_POP();
        } else {
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 4U,
                       *chartInstance->c1_sfEvent);
        }

        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, *chartInstance->c1_sfEvent);
        break;

       default:
        CV_STATE_EVAL(2, 0, 0);

        /* Unreachable state, for coverage only */
        *chartInstance->c1_is_Abnormal = c1_IN_NO_ACTIVE_CHILD;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, *chartInstance->c1_sfEvent);
        break;
      }
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, *chartInstance->c1_sfEvent);
    break;

   case c1_IN_Normal:
    CV_STATE_EVAL(1, 0, c1_IN_Normal);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 12U,
                 *chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_i_debug_family_names,
      c1_c_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    if (!chartInstance->c1_dataWrittenToVector[4U]) {
      sf_read_before_write_error(chartInstance->S, 0U, 56U, 1, 5);
    }

    if (CV_EML_IF(12, 0, 0, CV_RELATIONAL_EVAL(5U, 12U, 0,
          *chartInstance->c1_count, chartInstance->c1_params.maxCounts, -1, 5U, *
          chartInstance->c1_count >= chartInstance->c1_params.maxCounts))) {
      c1_b_out = true;
    } else {
      c1_b_out = false;
    }

    _SFD_SYMBOL_SCOPE_POP();
    if (c1_b_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 12U, *chartInstance->c1_sfEvent);
      *chartInstance->c1_is_TrackMarker = c1_IN_NO_ACTIVE_CHILD;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, *chartInstance->c1_sfEvent);
      *chartInstance->c1_is_TrackMarker = c1_IN_Abnormal;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, *chartInstance->c1_sfEvent);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 15U, *chartInstance->c1_sfEvent);
      *chartInstance->c1_is_Abnormal = c1_IN_Stop;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, *chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_e_debug_family_names,
        c1_g_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *chartInstance->c1_v = 0.0;
      chartInstance->c1_dataWrittenToVector[0U] = true;
      _SFD_DATA_RANGE_CHECK(*chartInstance->c1_v, 9U);
      *chartInstance->c1_w = 0.0;
      chartInstance->c1_dataWrittenToVector[1U] = true;
      _SFD_DATA_RANGE_CHECK(*chartInstance->c1_w, 10U);
      _SFD_SYMBOL_SCOPE_POP();
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 5U,
                   *chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_d_debug_family_names,
        c1_d_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      if (!chartInstance->c1_dataWrittenToVector[3U]) {
        sf_read_before_write_error(chartInstance->S, 2U, 50U, 42, 5);
      }

      if (!chartInstance->c1_dataWrittenToVector[2U]) {
        sf_read_before_write_error(chartInstance->S, 1U, 50U, 48, 8);
      }

      c1_velControl(chartInstance, *chartInstance->c1_xFilt,
                    *chartInstance->c1_sizeFilt, *chartInstance->c1_imgWidth,
                    *chartInstance->c1_linVelGain, *chartInstance->c1_angVelGain,
                    &c1_c_v, &c1_c_w);
      *chartInstance->c1_v = c1_c_v;
      chartInstance->c1_dataWrittenToVector[0U] = true;
      _SFD_DATA_RANGE_CHECK(*chartInstance->c1_v, 9U);
      *chartInstance->c1_w = c1_c_w;
      chartInstance->c1_dataWrittenToVector[1U] = true;
      _SFD_DATA_RANGE_CHECK(*chartInstance->c1_w, 10U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 5U, *chartInstance->c1_sfEvent);
    break;

   default:
    CV_STATE_EVAL(1, 0, 0);

    /* Unreachable state, for coverage only */
    *chartInstance->c1_is_TrackMarker = c1_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, *chartInstance->c1_sfEvent);
    break;
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, *chartInstance->c1_sfEvent);
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)(c1_machineNumber);
  (void)(c1_chartNumber);
  (void)(c1_instanceNumber);
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_nargin, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargin), &c1_thisId);
  sf_mex_destroy(&c1_nargin);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargin;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_nargin = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargin), &c1_thisId);
  sf_mex_destroy(&c1_nargin);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  boolean_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(boolean_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static boolean_T c1_c_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  boolean_T c1_y;
  boolean_T c1_b0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_b0, 1, 11, 0U, 0, 0U, 0);
  c1_y = c1_b0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_sf_internal_predicateOutput;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  boolean_T c1_y;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_sf_internal_predicateOutput = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_sf_internal_predicateOutput), &c1_thisId);
  sf_mex_destroy(&c1_sf_internal_predicateOutput);
  *(boolean_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_controlLib_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c1_nameCaptureInfo;
}

static void c1_velControl(SFc1_controlLibInstanceStruct *chartInstance, real_T
  c1_d_x, real_T c1_b_bSize, real_T c1_b_iWidth, real_T c1_b_linGain, real_T
  c1_b_angGain, real_T *c1_c_v, real_T *c1_c_w)
{
  _SFD_SET_DATA_VALUE_PTR(21U, c1_c_w);
  _SFD_SET_DATA_VALUE_PTR(19U, c1_c_v);
  _SFD_SET_DATA_VALUE_PTR(17U, &c1_b_angGain);
  _SFD_SET_DATA_VALUE_PTR(16U, &c1_b_linGain);
  _SFD_SET_DATA_VALUE_PTR(15U, &c1_b_iWidth);
  _SFD_SET_DATA_VALUE_PTR(14U, &c1_b_bSize);
  _SFD_SET_DATA_VALUE_PTR(12U, &c1_d_x);
  _SFD_CS_CALL(FUNCTION_ACTIVE_TAG, 7U, *chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH(7U, 0U);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("x", &c1_d_x, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("bSize", &c1_b_bSize, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("iWidth", &c1_b_iWidth, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("linGain", &c1_b_linGain, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("angGain", &c1_b_angGain, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("v", c1_c_v, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("w", c1_c_w, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 7U, *chartInstance->c1_sfEvent);
  *chartInstance->c1_b_x = c1_d_x;
  *chartInstance->c1_bSize = c1_b_bSize;
  *chartInstance->c1_iWidth = c1_b_iWidth;
  *chartInstance->c1_linGain = c1_b_linGain;
  *chartInstance->c1_angGain = c1_b_angGain;
  sf_call_output_fcn_call(chartInstance->S, 0, "velControl", 0);
  *c1_c_v = *chartInstance->c1_b_v;
  *c1_c_w = *chartInstance->c1_b_w;
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CS_CALL(FUNCTION_INACTIVE_TAG, 7U, *chartInstance->c1_sfEvent);
  _SFD_UNSET_DATA_VALUE_PTR(21U);
  _SFD_UNSET_DATA_VALUE_PTR(19U);
  _SFD_UNSET_DATA_VALUE_PTR(17U);
  _SFD_UNSET_DATA_VALUE_PTR(16U);
  _SFD_UNSET_DATA_VALUE_PTR(15U);
  _SFD_UNSET_DATA_VALUE_PTR(14U);
  _SFD_UNSET_DATA_VALUE_PTR(12U);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 7U, *chartInstance->c1_sfEvent);
}

static void c1_avgFilter(SFc1_controlLibInstanceStruct *chartInstance, real_T
  c1_d_x, real_T c1_b_s, real_T *c1_b_xf, real_T *c1_b_sf)
{
  _SFD_SET_DATA_VALUE_PTR(20U, c1_b_sf);
  _SFD_SET_DATA_VALUE_PTR(18U, c1_b_xf);
  _SFD_SET_DATA_VALUE_PTR(13U, &c1_b_s);
  _SFD_SET_DATA_VALUE_PTR(11U, &c1_d_x);
  _SFD_CS_CALL(FUNCTION_ACTIVE_TAG, 6U, *chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH(4U, 0U);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("x", &c1_d_x, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("s", &c1_b_s, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("xf", c1_b_xf, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_IMPORTABLE("sf", c1_b_sf, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 6U, *chartInstance->c1_sfEvent);
  *chartInstance->c1_c_x = c1_d_x;
  *chartInstance->c1_s = c1_b_s;
  sf_call_output_fcn_call(chartInstance->S, 1, "avgFilter", 0);
  *c1_b_xf = *chartInstance->c1_xf;
  *c1_b_sf = *chartInstance->c1_sf;
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CS_CALL(FUNCTION_INACTIVE_TAG, 6U, *chartInstance->c1_sfEvent);
  _SFD_UNSET_DATA_VALUE_PTR(20U);
  _SFD_UNSET_DATA_VALUE_PTR(18U);
  _SFD_UNSET_DATA_VALUE_PTR(13U);
  _SFD_UNSET_DATA_VALUE_PTR(11U);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 6U, *chartInstance->c1_sfEvent);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_d_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i1;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i1, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i1;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  uint8_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(uint8_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_b_is_TrackMarker, const char_T
  *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_is_TrackMarker),
    &c1_thisId);
  sf_mex_destroy(&c1_b_is_TrackMarker);
  return c1_y;
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_is_TrackMarker;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  uint8_T c1_y;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_b_is_TrackMarker = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_is_TrackMarker),
    &c1_thisId);
  sf_mex_destroy(&c1_b_is_TrackMarker);
  *(uint8_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  c1_struct_NQPZsPS5PAq52I8Ane3iuE c1_u;
  const mxArray *c1_y = NULL;
  static const char * c1_sv0[14] = { "Ts", "bufSize", "maxDisp", "minSize",
    "maxSize", "maxCounts", "linVelGain", "angVelGain", "maxLinVel", "maxAngVel",
    "posDeadZone", "targetSize", "sizeDeadZone", "speedRedSize" };

  real_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_c_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_d_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_e_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_f_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_g_u;
  const mxArray *c1_g_y = NULL;
  real_T c1_h_u;
  const mxArray *c1_h_y = NULL;
  real_T c1_i_u;
  const mxArray *c1_i_y = NULL;
  real_T c1_j_u;
  const mxArray *c1_j_y = NULL;
  real_T c1_k_u;
  const mxArray *c1_k_y = NULL;
  real_T c1_l_u;
  const mxArray *c1_l_y = NULL;
  real_T c1_m_u;
  const mxArray *c1_m_y = NULL;
  real_T c1_n_u;
  const mxArray *c1_n_y = NULL;
  real_T c1_o_u;
  const mxArray *c1_o_y = NULL;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(c1_struct_NQPZsPS5PAq52I8Ane3iuE *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createstruct("structure", 14, c1_sv0, 2, 1, 1),
                false);
  c1_b_u = c1_u.Ts;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "Ts", c1_b_y, 0);
  c1_c_u = c1_u.bufSize;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "bufSize", c1_c_y, 1);
  c1_d_u = c1_u.maxDisp;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "maxDisp", c1_d_y, 2);
  c1_e_u = c1_u.minSize;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "minSize", c1_e_y, 3);
  c1_f_u = c1_u.maxSize;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "maxSize", c1_f_y, 4);
  c1_g_u = c1_u.maxCounts;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "maxCounts", c1_g_y, 5);
  c1_h_u = c1_u.linVelGain;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "linVelGain", c1_h_y, 6);
  c1_i_u = c1_u.angVelGain;
  c1_i_y = NULL;
  sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "angVelGain", c1_i_y, 7);
  c1_j_u = c1_u.maxLinVel;
  c1_j_y = NULL;
  sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_j_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "maxLinVel", c1_j_y, 8);
  c1_k_u = c1_u.maxAngVel;
  c1_k_y = NULL;
  sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "maxAngVel", c1_k_y, 9);
  c1_l_u = c1_u.posDeadZone;
  c1_l_y = NULL;
  sf_mex_assign(&c1_l_y, sf_mex_create("y", &c1_l_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "posDeadZone", c1_l_y, 10);
  c1_m_u = c1_u.targetSize;
  c1_m_y = NULL;
  sf_mex_assign(&c1_m_y, sf_mex_create("y", &c1_m_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "targetSize", c1_m_y, 11);
  c1_n_u = c1_u.sizeDeadZone;
  c1_n_y = NULL;
  sf_mex_assign(&c1_n_y, sf_mex_create("y", &c1_n_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "sizeDeadZone", c1_n_y, 12);
  c1_o_u = c1_u.speedRedSize;
  c1_o_y = NULL;
  sf_mex_assign(&c1_o_y, sf_mex_create("y", &c1_o_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c1_y, 0, "speedRedSize", c1_o_y, 13);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_g_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  c1_struct_NQPZsPS5PAq52I8Ane3iuE *c1_y)
{
  emlrtMsgIdentifier c1_thisId;
  static const char * c1_fieldNames[14] = { "Ts", "bufSize", "maxDisp",
    "minSize", "maxSize", "maxCounts", "linVelGain", "angVelGain", "maxLinVel",
    "maxAngVel", "posDeadZone", "targetSize", "sizeDeadZone", "speedRedSize" };

  c1_thisId.fParent = c1_parentId;
  c1_thisId.bParentIsCell = false;
  sf_mex_check_struct(c1_parentId, c1_u, 14, c1_fieldNames, 0U, NULL);
  c1_thisId.fIdentifier = "Ts";
  c1_y->Ts = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c1_u, "Ts", "Ts", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "bufSize";
  c1_y->bufSize = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "bufSize", "bufSize", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "maxDisp";
  c1_y->maxDisp = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "maxDisp", "maxDisp", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "minSize";
  c1_y->minSize = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "minSize", "minSize", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "maxSize";
  c1_y->maxSize = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "maxSize", "maxSize", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "maxCounts";
  c1_y->maxCounts = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "maxCounts", "maxCounts", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "linVelGain";
  c1_y->linVelGain = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "linVelGain", "linVelGain", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "angVelGain";
  c1_y->angVelGain = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "angVelGain", "angVelGain", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "maxLinVel";
  c1_y->maxLinVel = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "maxLinVel", "maxLinVel", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "maxAngVel";
  c1_y->maxAngVel = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "maxAngVel", "maxAngVel", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "posDeadZone";
  c1_y->posDeadZone = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "posDeadZone", "posDeadZone", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "targetSize";
  c1_y->targetSize = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "targetSize", "targetSize", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "sizeDeadZone";
  c1_y->sizeDeadZone = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "sizeDeadZone", "sizeDeadZone", 0)), &c1_thisId);
  c1_thisId.fIdentifier = "speedRedSize";
  c1_y->speedRedSize = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c1_u, "speedRedSize", "speedRedSize", 0)), &c1_thisId);
  sf_mex_destroy(&c1_u);
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_params;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  c1_struct_NQPZsPS5PAq52I8Ane3iuE c1_y;
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)chartInstanceVoid;
  c1_b_params = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_params), &c1_thisId,
                        &c1_y);
  sf_mex_destroy(&c1_b_params);
  *(c1_struct_NQPZsPS5PAq52I8Ane3iuE *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static void c1_h_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_b_dataWrittenToVector, const char_T *c1_identifier,
  boolean_T c1_y[6])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_dataWrittenToVector),
                        &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_dataWrittenToVector);
}

static void c1_i_emlrt_marshallIn(SFc1_controlLibInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, boolean_T c1_y[6])
{
  boolean_T c1_bv1[6];
  int32_T c1_i2;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_bv1, 1, 11, 0U, 1, 0U, 1, 6);
  for (c1_i2 = 0; c1_i2 < 6; c1_i2++) {
    c1_y[c1_i2] = c1_bv1[c1_i2];
  }

  sf_mex_destroy(&c1_u);
}

static const mxArray *c1_j_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_b_setSimStateSideEffectsInfo, const char_T
  *c1_identifier)
{
  const mxArray *c1_y = NULL;
  emlrtMsgIdentifier c1_thisId;
  c1_y = NULL;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  sf_mex_assign(&c1_y, c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_setSimStateSideEffectsInfo), &c1_thisId), false);
  sf_mex_destroy(&c1_b_setSimStateSideEffectsInfo);
  return c1_y;
}

static const mxArray *c1_k_emlrt_marshallIn(SFc1_controlLibInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  const mxArray *c1_y = NULL;
  (void)chartInstance;
  (void)c1_parentId;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_duplicatearraysafe(&c1_u), false);
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static const mxArray *sf_get_hover_data_for_msg(SFc1_controlLibInstanceStruct
  *chartInstance, int32_T c1_ssid)
{
  (void)chartInstance;
  (void)c1_ssid;
  return NULL;
}

static void c1_init_sf_message_store_memory(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_dsm_address_info(SFc1_controlLibInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_controlLibInstanceStruct
  *chartInstance)
{
  chartInstance->c1_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c1_sfEvent = (int32_T *)ssGetDWork_wrapper(chartInstance->S, 0);
  chartInstance->c1_is_active_c1_controlLib = (uint8_T *)ssGetDWork_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_is_TrackMarker = (uint8_T *)ssGetDWork_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_is_active_TrackMarker = (uint8_T *)ssGetDWork_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_is_Abnormal = (uint8_T *)ssGetDWork_wrapper(chartInstance->S,
    4);
  chartInstance->c1_is_active_DetectOutliers = (uint8_T *)ssGetDWork_wrapper
    (chartInstance->S, 5);
  chartInstance->c1_x = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c1_blobSize = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_imgWidth = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_linVelGain = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_angVelGain = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c1_v = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c1_w = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c1_sizeFilt = (real_T *)ssGetDWork_wrapper(chartInstance->S, 6);
  chartInstance->c1_xFilt = (real_T *)ssGetDWork_wrapper(chartInstance->S, 7);
  chartInstance->c1_count = (real_T *)ssGetDWork_wrapper(chartInstance->S, 8);
  chartInstance->c1_xPrev = (real_T *)ssGetDWork_wrapper(chartInstance->S, 9);
  chartInstance->c1_b_x = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_bSize = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c1_iWidth = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c1_b_v = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c1_b_w = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c1_linGain = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c1_angGain = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c1_c_x = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 8);
  chartInstance->c1_s = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    9);
  chartInstance->c1_xf = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    7);
  chartInstance->c1_sf = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    8);
}

/* SFunction Glue Code */
static uint32_T* sf_get_sfun_dwork_checksum(void);
void sf_c1_controlLib_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(337522471U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3916388197U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2926396848U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1913656115U);
}

mxArray* sf_c1_controlLib_get_post_codegen_info(void);
mxArray *sf_c1_controlLib_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("CPdpUnsl4D0c74RvFOm51G");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxData);
  }

  {
    mxArray* mxPostCodegenInfo = sf_c1_controlLib_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_controlLib_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_controlLib_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("early");
  mxArray *fallbackReason = mxCreateString("ext_mode");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_controlLib_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_controlLib_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c1_controlLib(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[4],T\"v\",},{M[1],M[5],T\"w\",},{M[3],M[13],T\"count\",},{M[3],M[96],T\"sizeFilt\",},{M[3],M[97],T\"xFilt\",},{M[3],M[14],T\"xPrev\",},{M[8],M[0],T\"is_active_c1_controlLib\",},{M[8],M[6],T\"is_active_DetectOutliers\",},{M[8],M[49],T\"is_active_TrackMarker\",},{M[9],M[49],T\"is_TrackMarker\",}}",
    "100 S1x2'type','srcId','name','auxInfo'{{M[9],M[59],T\"is_Abnormal\",},{M[15],M[0],T\"dataWrittenToVector\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 12, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_controlLib_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const mxArray* sf_opaque_get_hover_data_for_msg(void* chartInstance,
  int32_T msgSSID)
{
  return sf_get_hover_data_for_msg( (SFc1_controlLibInstanceStruct *)
    chartInstance, msgSSID);
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_controlLibInstanceStruct *chartInstance =
      (SFc1_controlLibInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _controlLibMachineNumber_,
           1,
           8,
           17,
           0,
           23,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_controlLibMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_controlLibMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _controlLibMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,0,0,0,"count");
          _SFD_SET_DATA_PROPS(1,0,0,0,"sizeFilt");
          _SFD_SET_DATA_PROPS(2,0,0,0,"xFilt");
          _SFD_SET_DATA_PROPS(3,0,0,0,"xPrev");
          _SFD_SET_DATA_PROPS(4,1,1,0,"x");
          _SFD_SET_DATA_PROPS(5,1,1,0,"blobSize");
          _SFD_SET_DATA_PROPS(6,1,1,0,"imgWidth");
          _SFD_SET_DATA_PROPS(7,1,1,0,"linVelGain");
          _SFD_SET_DATA_PROPS(8,1,1,0,"angVelGain");
          _SFD_SET_DATA_PROPS(9,2,0,1,"v");
          _SFD_SET_DATA_PROPS(10,2,0,1,"w");
          _SFD_SET_DATA_PROPS(11,8,0,0,"");
          _SFD_SET_DATA_PROPS(12,8,0,0,"");
          _SFD_SET_DATA_PROPS(13,8,0,0,"");
          _SFD_SET_DATA_PROPS(14,8,0,0,"");
          _SFD_SET_DATA_PROPS(15,8,0,0,"");
          _SFD_SET_DATA_PROPS(16,8,0,0,"");
          _SFD_SET_DATA_PROPS(17,8,0,0,"");
          _SFD_SET_DATA_PROPS(18,9,0,0,"");
          _SFD_SET_DATA_PROPS(19,9,0,0,"");
          _SFD_SET_DATA_PROPS(20,9,0,0,"");
          _SFD_SET_DATA_PROPS(21,9,0,0,"");
          _SFD_SET_DATA_PROPS(22,10,0,0,"params");
          _SFD_STATE_INFO(0,0,1);
          _SFD_STATE_INFO(1,0,1);
          _SFD_STATE_INFO(2,0,0);
          _SFD_STATE_INFO(3,0,0);
          _SFD_STATE_INFO(4,0,0);
          _SFD_STATE_INFO(5,0,0);
          _SFD_STATE_INFO(6,0,2);
          _SFD_STATE_INFO(7,0,2);
          _SFD_CH_SUBSTATE_COUNT(2);
          _SFD_CH_SUBSTATE_DECOMP(1);
          _SFD_CH_SUBSTATE_INDEX(0,0);
          _SFD_CH_SUBSTATE_INDEX(1,1);
          _SFD_ST_SUBSTATE_COUNT(0,0);
          _SFD_ST_SUBSTATE_COUNT(1,2);
          _SFD_ST_SUBSTATE_INDEX(1,0,2);
          _SFD_ST_SUBSTATE_INDEX(1,1,5);
          _SFD_ST_SUBSTATE_COUNT(2,2);
          _SFD_ST_SUBSTATE_INDEX(2,0,3);
          _SFD_ST_SUBSTATE_INDEX(2,1,4);
          _SFD_ST_SUBSTATE_COUNT(3,0);
          _SFD_ST_SUBSTATE_COUNT(4,0);
          _SFD_ST_SUBSTATE_COUNT(5,0);
        }

        _SFD_CV_INIT_CHART(2,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(1,2,1,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(2,2,1,1,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(3,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(4,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(5,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(6,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(7,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(10,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(12,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(16,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(15,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(13,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(14,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(2,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(1,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(3,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(4,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(7,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(6,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(8,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(5,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(9,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(11,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(1,1,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(5,1,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(4,1,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(3,1,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(0,1,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(13,0,0,0,1,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_IF(13,0,0,1,32,1,32);

        {
          static int condStart[] = { 1, 20 };

          static int condEnd[] = { 12, 32 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(13,0,0,1,32,2,0,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(13,0,0,1,12,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(13,0,1,20,32,-1,3);
        _SFD_CV_INIT_EML(14,0,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(14,0,0,1,13,1,13);
        _SFD_CV_INIT_EML_RELATIONAL(14,0,0,1,13,-1,5);
        _SFD_CV_INIT_EML(12,0,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(12,0,0,1,30,1,30);
        _SFD_CV_INIT_EML_RELATIONAL(12,0,0,1,30,-1,5);
        _SFD_CV_INIT_EML(16,0,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(16,0,0,1,29,1,29);
        _SFD_CV_INIT_EML_RELATIONAL(16,0,0,1,29,-1,2);
        _SFD_CV_INIT_EML(0,0,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(1,0,0,0,1,0,0,0,0,0,3,1);
        _SFD_CV_INIT_EML_IF(1,0,0,20,113,20,67);

        {
          static int condStart[] = { 20, 57, 89 };

          static int condEnd[] = { 48, 81, 113 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2 };

          _SFD_CV_INIT_EML_MCDC(1,0,0,20,113,3,0,&(condStart[0]),&(condEnd[0]),5,
                                &(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(1,0,0,20,48,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(1,0,1,57,81,-1,3);
        _SFD_CV_INIT_EML_RELATIONAL(1,0,2,89,113,-1,5);
        _SFD_CV_INIT_EML(3,0,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(4,0,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(7,0,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(7,0,0,1,11,1,11);
        _SFD_CV_INIT_EML_RELATIONAL(7,0,0,1,11,-1,2);
        _SFD_CV_INIT_EML(8,0,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(11,0,0,0,0,0,0,0,0,0,0,0);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(21,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(22,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_e_sf_marshallOut,(MexInFcnForType)c1_e_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(11,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(12,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(13,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(14,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(15,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(16,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(17,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(18,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(19,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(20,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(21,(void *)(NULL));
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _controlLibMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_controlLibInstanceStruct *chartInstance =
      (SFc1_controlLibInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c1_x);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c1_blobSize);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c1_imgWidth);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c1_linVelGain);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c1_angVelGain);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c1_v);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c1_w);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c1_sizeFilt);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c1_xFilt);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c1_count);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c1_xPrev);
        _SFD_SET_DATA_VALUE_PTR(22U, &chartInstance->c1_params);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sFJLJNZvNSjGBx21DKKI8JH";
}

static void sf_check_dwork_consistency(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    const uint32_T *sfunDWorkChecksum = sf_get_sfun_dwork_checksum();
    mxArray *infoStruct = load_controlLib_optimization_info(sim_mode_is_rtw_gen
      (S), sim_mode_is_modelref_sim(S), sim_mode_is_external(S));
    mxArray* mxRTWDWorkChecksum = sf_get_dwork_info_from_mat_file(S,
      sf_get_instance_specialization(), infoStruct, 1, "dworkChecksum");
    if (mxRTWDWorkChecksum != NULL) {
      double *pr = mxGetPr(mxRTWDWorkChecksum);
      if ((uint32_T)pr[0] != sfunDWorkChecksum[0] ||
          (uint32_T)pr[1] != sfunDWorkChecksum[1] ||
          (uint32_T)pr[2] != sfunDWorkChecksum[2] ||
          (uint32_T)pr[3] != sfunDWorkChecksum[3]) {
        sf_mex_error_message("Code generation and simulation targets registered different sets of persistent variables for the block. "
                             "External or Rapid Accelerator mode simulation requires code generation and simulation targets to "
                             "register the same set of persistent variables for this block. "
                             "This discrepancy is typically caused by MATLAB functions that have different code paths for "
                             "simulation and code generation targets where these code paths define different sets of persistent variables. "
                             "Please identify these code paths in the offending block and rewrite the MATLAB code so that "
                             "the set of persistent variables is the same between simulation and code generation.");
      }
    }
  }
}

static void sf_opaque_initialize_c1_controlLib(void *chartInstanceVar)
{
  sf_check_dwork_consistency(((SFc1_controlLibInstanceStruct*) chartInstanceVar
    )->S);
  chart_debug_initialization(((SFc1_controlLibInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c1_controlLib((SFc1_controlLibInstanceStruct*)
    chartInstanceVar);
  initialize_c1_controlLib((SFc1_controlLibInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_controlLib(void *chartInstanceVar)
{
  enable_c1_controlLib((SFc1_controlLibInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_controlLib(void *chartInstanceVar)
{
  disable_c1_controlLib((SFc1_controlLibInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_controlLib(void *chartInstanceVar)
{
  sf_gateway_c1_controlLib((SFc1_controlLibInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_ext_mode_exec_c1_controlLib(void *chartInstanceVar)
{
  ext_mode_exec_c1_controlLib((SFc1_controlLibInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_controlLib(SimStruct* S)
{
  return get_sim_state_c1_controlLib((SFc1_controlLibInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_controlLib(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c1_controlLib((SFc1_controlLibInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c1_controlLib(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_controlLibInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_controlLib_optimization_info();
    }

    finalize_c1_controlLib((SFc1_controlLibInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_controlLib((SFc1_controlLibInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_controlLib(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_controlLib((SFc1_controlLibInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

mxArray *sf_c1_controlLib_get_testpoint_info(void)
{
  const char *infoEncStr[] = {
    "100 S1x5'varName','path'{{T\"is_active_c1_controlLib\",T\"is_active_c1_controlLib\"},{T\"is_TrackMarker\",T\"TrackMarker.is_TrackMarker\"},{T\"is_active_TrackMarker\",T\"TrackMarker.is_active_TrackMarker\"},{T\"is_Abnormal\",T\"TrackMarker.Abnormal.is_Abnormal\"},{T\"is_active_DetectOutliers\",T\"DetectOutliers.is_active_DetectOutliers\"}}"
  };

  mxArray *mxTpInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  return mxTpInfo;
}

static void sf_set_sfun_dwork_info(SimStruct *S)
{
  const char *dworkEncStr[] = {
    "100 S1x10'type','isSigned','wordLength','bias','slope','exponent','isScaledDouble','isComplex','size','local_queue_dwork_idx','local_queue_id','maxStringLength'{{T\"int32\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"uint8\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"uint8\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"uint8\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"uint8\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"uint8\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"double\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"double\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"double\",,,,,,,M[0],M[],M[-1],M[-1],},{T\"double\",,,,,,,M[0],M[],M[-1],M[-1],}}"
  };

  sf_set_encoded_dwork_info(S, dworkEncStr, 10, 10);
}

static uint32_T* sf_get_sfun_dwork_checksum()
{
  static uint32_T checksum[4] = { 2832530040U, 1137665392U, 4219911252U,
    4167581439U };

  return checksum;
}

static void mdlSetWorkWidths_c1_controlLib(SimStruct *S)
{
  /* Actual parameters from chart:
     params
   */
  const char_T *rtParamNames[] = { "params" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0],
    sf_get_param_data_type_id(S,0));
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortDirectFeedThrough(S, 1, 1);
  ssSetInputPortDirectFeedThrough(S, 2, 1);
  ssSetInputPortDirectFeedThrough(S, 3, 1);
  ssSetInputPortDirectFeedThrough(S, 4, 1);
  ssSetInputPortDirectFeedThrough(S, 5, 1);
  ssSetInputPortDirectFeedThrough(S, 6, 1);
  ssSetInputPortDirectFeedThrough(S, 7, 1);
  ssSetInputPortDirectFeedThrough(S, 8, 1);
  ssSetStatesModifiedOnlyInUpdate(S, 0);
  ssSetBlockIsPurelyCombinatorial_wrapper(S, 0);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_controlLib_optimization_info(sim_mode_is_rtw_gen
      (S), sim_mode_is_modelref_sim(S), sim_mode_is_external(S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetSupportedForRowMajorCodeGen(S, 1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 1);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,1);
    sf_mark_output_events_with_multiple_callers(S,sf_get_instance_specialization
      (),infoStruct,1,2);
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
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,9);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=9; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 9; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
    sf_set_sfun_dwork_info(S);
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1347243784U));
  ssSetChecksum1(S,(2181269776U));
  ssSetChecksum2(S,(3842989469U));
  ssSetChecksum3(S,(653130400U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_controlLib(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Stateflow");
  }
}

static void mdlStart_c1_controlLib(SimStruct *S)
{
  SFc1_controlLibInstanceStruct *chartInstance;
  chartInstance = (SFc1_controlLibInstanceStruct *)utMalloc(sizeof
    (SFc1_controlLibInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc1_controlLibInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 0;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_controlLib;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_controlLib;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_controlLib;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_controlLib;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_controlLib;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_controlLib;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_controlLib;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_controlLib;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_controlLib;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_controlLib;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_controlLib;
  chartInstance->chartInfo.callGetHoverDataForMsg =
    sf_opaque_get_hover_data_for_msg;
  chartInstance->chartInfo.extModeExec = sf_opaque_ext_mode_exec_c1_controlLib;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartEventFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  chartInstance->chartInfo.dispatchToExportedFcn = NULL;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0, NULL, NULL);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c1_controlLib(chartInstance);
}

void c1_controlLib_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_controlLib(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_controlLib(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_controlLib(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_controlLib_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
