#include "__cf_subs_MM.h"
#ifndef RTW_HEADER_subs_MM_h_
#define RTW_HEADER_subs_MM_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef subs_MM_COMMON_INCLUDES_
#define subs_MM_COMMON_INCLUDES_
#include <stdlib.h>
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "subs_MM_types.h"
#include "multiword_types.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME subs_MM
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (4) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (0)   
#elif NCSTATES != 0
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T esl5eo5qeb ; real_T agp2npl4fr ; real_T h2lwobz0fs ;
real_T pooxw41krp [ 6 ] ; } B ; typedef struct { struct { void * AQHandles ;
void * SlioLTF ; } agbuwqhw3q ; struct { void * LoggedData ; } loxbs1pqbs ; }
DW ; typedef struct { rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct
P_ { real_T Constant_Value ; real_T Step2_Time ; real_T Step2_Y0 ; real_T
Step2_YFinal ; real_T Step_Time ; real_T Step_Y0 ; real_T Step_YFinal ;
real_T Step1_Time ; real_T Step1_Y0 ; real_T Step1_YFinal ; uint8_T
ManualSwitch1_CurrentSetting ; uint8_T ManualSwitch2_CurrentSetting ; uint8_T
ManualSwitch_CurrentSetting ; } ; extern const char *
RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern DW rtDW ; extern P rtP ;
extern const rtwCAPI_ModelMappingStaticInfo * subs_MM_GetCAPIStaticMap ( void
) ; extern SimStruct * const rtS ; extern const int_T gblNumToFiles ; extern
const int_T gblNumFrFiles ; extern const int_T gblNumFrWksBlocks ; extern
rtInportTUtable * gblInportTUtables ; extern const char * gblInportFileName ;
extern const int_T gblNumRootInportBlks ; extern const int_T
gblNumModelInputs ; extern const int_T gblInportDataTypeIdx [ ] ; extern
const int_T gblInportDims [ ] ; extern const int_T gblInportComplex [ ] ;
extern const int_T gblInportInterpoFlag [ ] ; extern const int_T
gblInportContinuous [ ] ; extern const int_T gblParameterTuningTid ; extern
DataMapInfo * rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo *
rt_modelMapInfoPtr ; void MdlOutputs ( int_T tid ) ; void
MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ;
void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void
MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( void
) ;
#endif
