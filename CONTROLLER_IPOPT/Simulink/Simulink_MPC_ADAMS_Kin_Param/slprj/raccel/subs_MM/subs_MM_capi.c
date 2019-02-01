#include "__cf_subs_MM.h"
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "subs_MM_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "subs_MM.h"
#include "subs_MM_capi.h"
#include "subs_MM_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"subs_MM/Constant" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 0 ,
TARGET_STRING (
 "subs_MM/TmpSignal ConversionAtHiddenToAsyncQueue_InsertedFor_Mux_at_outport_0Inport1"
) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 1 } , { 2 , 0 , TARGET_STRING (
"subs_MM/Manual Switch" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 2 } , { 3
, 0 , TARGET_STRING ( "subs_MM/Manual Switch2" ) , TARGET_STRING ( "" ) , 0 ,
0 , 0 , 0 , 2 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ;
static const rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 4 ,
TARGET_STRING ( "subs_MM/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0
} , { 5 , TARGET_STRING ( "subs_MM/Step" ) , TARGET_STRING ( "Time" ) , 0 , 0
, 0 } , { 6 , TARGET_STRING ( "subs_MM/Step" ) , TARGET_STRING ( "Before" ) ,
0 , 0 , 0 } , { 7 , TARGET_STRING ( "subs_MM/Step" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 8 , TARGET_STRING ( "subs_MM/Step1" ) ,
TARGET_STRING ( "Time" ) , 0 , 0 , 0 } , { 9 , TARGET_STRING (
"subs_MM/Step1" ) , TARGET_STRING ( "Before" ) , 0 , 0 , 0 } , { 10 ,
TARGET_STRING ( "subs_MM/Step1" ) , TARGET_STRING ( "After" ) , 0 , 0 , 0 } ,
{ 11 , TARGET_STRING ( "subs_MM/Step2" ) , TARGET_STRING ( "Time" ) , 0 , 0 ,
0 } , { 12 , TARGET_STRING ( "subs_MM/Step2" ) , TARGET_STRING ( "Before" ) ,
0 , 0 , 0 } , { 13 , TARGET_STRING ( "subs_MM/Step2" ) , TARGET_STRING (
"After" ) , 0 , 0 , 0 } , { 14 , TARGET_STRING ( "subs_MM/Manual Switch" ) ,
TARGET_STRING ( "CurrentSetting" ) , 1 , 0 , 0 } , { 15 , TARGET_STRING (
"subs_MM/Manual Switch1" ) , TARGET_STRING ( "CurrentSetting" ) , 1 , 0 , 0 }
, { 16 , TARGET_STRING ( "subs_MM/Manual Switch2" ) , TARGET_STRING (
"CurrentSetting" ) , 1 , 0 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } }
; static const rtwCAPI_ModelParameters rtModelParameters [ ] = { { 0 , ( NULL
) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . esl5eo5qeb , & rtB . pooxw41krp [
0 ] , & rtB . h2lwobz0fs , & rtB . agp2npl4fr , & rtP . Constant_Value , &
rtP . Step_Time , & rtP . Step_Y0 , & rtP . Step_YFinal , & rtP . Step1_Time
, & rtP . Step1_Y0 , & rtP . Step1_YFinal , & rtP . Step2_Time , & rtP .
Step2_Y0 , & rtP . Step2_YFinal , & rtP . ManualSwitch_CurrentSetting , & rtP
. ManualSwitch1_CurrentSetting , & rtP . ManualSwitch2_CurrentSetting , } ;
static int32_T * rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , SS_DOUBLE , 0 , 0 } , {
"unsigned char" , "uint8_T" , 0 , 0 , sizeof ( uint8_T ) , SS_UINT8 , 0 , 0 }
} ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } } ; static
const uint_T rtDimensionArray [ ] = { 1 , 1 , 6 , 1 } ; static const real_T
rtcapiStoredFloats [ ] = { 0.5 , 0.0 } ; static const rtwCAPI_FixPtMap
rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , 0 }
, } ; static const rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( NULL ) ,
( NULL ) , 2 , 0 } , { ( const void * ) & rtcapiStoredFloats [ 0 ] , ( const
void * ) & rtcapiStoredFloats [ 1 ] , 1 , 0 } , { ( const void * ) &
rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , 0 ,
0 } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals
, 4 , ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockParameters , 13 ,
rtModelParameters , 0 } , { ( NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap
, rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float"
, { 2685760919U , 4231558893U , 2448741060U , 3383562446U } , ( NULL ) , 0 ,
0 } ; const rtwCAPI_ModelMappingStaticInfo * subs_MM_GetCAPIStaticMap ( void
) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void subs_MM_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void subs_MM_host_InitializeDataMapInfo ( subs_MM_host_DataMapInfo_T *
dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , NULL ) ; rtwCAPI_SetPath (
dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
