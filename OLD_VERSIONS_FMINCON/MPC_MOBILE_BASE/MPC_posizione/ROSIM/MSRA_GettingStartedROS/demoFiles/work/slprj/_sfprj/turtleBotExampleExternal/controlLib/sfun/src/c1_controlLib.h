#ifndef __c1_controlLib_h__
#define __c1_controlLib_h__

/* Type Definitions */
#ifndef struct_struct_NQPZsPS5PAq52I8Ane3iuE_tag
#define struct_struct_NQPZsPS5PAq52I8Ane3iuE_tag

struct struct_NQPZsPS5PAq52I8Ane3iuE_tag
{
  real_T Ts;
  real_T bufSize;
  real_T maxDisp;
  real_T minSize;
  real_T maxSize;
  real_T maxCounts;
  real_T linVelGain;
  real_T angVelGain;
  real_T maxLinVel;
  real_T maxAngVel;
  real_T posDeadZone;
  real_T targetSize;
  real_T sizeDeadZone;
  real_T speedRedSize;
};

#endif                                 /*struct_struct_NQPZsPS5PAq52I8Ane3iuE_tag*/

#ifndef typedef_c1_struct_NQPZsPS5PAq52I8Ane3iuE
#define typedef_c1_struct_NQPZsPS5PAq52I8Ane3iuE

typedef struct struct_NQPZsPS5PAq52I8Ane3iuE_tag
  c1_struct_NQPZsPS5PAq52I8Ane3iuE;

#endif                                 /*typedef_c1_struct_NQPZsPS5PAq52I8Ane3iuE*/

#ifndef typedef_SFc1_controlLibInstanceStruct
#define typedef_SFc1_controlLibInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  c1_struct_NQPZsPS5PAq52I8Ane3iuE c1_params;
  boolean_T c1_dataWrittenToVector[6];
  uint8_T c1_doSetSimStateSideEffects;
  const mxArray *c1_setSimStateSideEffectsInfo;
  void *c1_fEmlrtCtx;
  int32_T *c1_sfEvent;
  uint8_T *c1_is_active_c1_controlLib;
  uint8_T *c1_is_TrackMarker;
  uint8_T *c1_is_active_TrackMarker;
  uint8_T *c1_is_Abnormal;
  uint8_T *c1_is_active_DetectOutliers;
  real_T *c1_x;
  real_T *c1_blobSize;
  real_T *c1_imgWidth;
  real_T *c1_linVelGain;
  real_T *c1_angVelGain;
  real_T *c1_v;
  real_T *c1_w;
  real_T *c1_sizeFilt;
  real_T *c1_xFilt;
  real_T *c1_count;
  real_T *c1_xPrev;
  real_T *c1_b_x;
  real_T *c1_bSize;
  real_T *c1_iWidth;
  real_T *c1_b_v;
  real_T *c1_b_w;
  real_T *c1_linGain;
  real_T *c1_angGain;
  real_T *c1_c_x;
  real_T *c1_s;
  real_T *c1_xf;
  real_T *c1_sf;
} SFc1_controlLibInstanceStruct;

#endif                                 /*typedef_SFc1_controlLibInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_controlLib_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_controlLib_get_check_sum(mxArray *plhs[]);
extern void c1_controlLib_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
