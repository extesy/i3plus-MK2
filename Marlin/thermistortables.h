#ifndef THERMISTORTABLES_H_
#define THERMISTORTABLES_H_
#include "Marlin.h"
#include "macros.h"
#define OVERSAMPLENR 16
#define ANY_THERMISTOR_IS(n) (THERMISTORHEATER_0 == n || THERMISTORHEATER_1 == n || THERMISTORHEATER_2 == n || THERMISTORHEATER_3 == n || THERMISTORHEATER_4 == n || THERMISTORBED == n)
#define PtA 3.9083E-3
#define PtB -5.775E-7
#define PtRt(T,R0) ((R0)*(1.0+(PtA)*(T)+(PtB)*(T)*(T)))
#define PtAdVal(T,R0,Rup) (short)(1024/(Rup/PtRt(T,R0)+1))
#define PtLine(T,R0,Rup) { PtAdVal(T,R0,Rup)*OVERSAMPLENR, T },
#if ANY_THERMISTOR_IS(1) 
  #include "thermistortable_1.h"
#endif
#if ANY_THERMISTOR_IS(2) 
  #include "thermistortable_2.h"
#endif
#if ANY_THERMISTOR_IS(3) 
  #include "thermistortable_3.h"
#endif
#if ANY_THERMISTOR_IS(4) 
  #include "thermistortable_4.h"
#endif
#if ANY_THERMISTOR_IS(5) 
  #include "thermistortable_5.h"
#endif
#if ANY_THERMISTOR_IS(6) 
  #include "thermistortable_6.h"
#endif
#if ANY_THERMISTOR_IS(7) 
  #include "thermistortable_7.h"
#endif
#if ANY_THERMISTOR_IS(71) 
  #include "thermistortable_71.h"
#endif
#if ANY_THERMISTOR_IS(8) 
  #include "thermistortable_8.h"
#endif
#if ANY_THERMISTOR_IS(9) 
  #include "thermistortable_9.h"
#endif
#if ANY_THERMISTOR_IS(10) 
  #include "thermistortable_10.h"
#endif
#if ANY_THERMISTOR_IS(11) 
  #include "thermistortable_11.h"
#endif
#if ANY_THERMISTOR_IS(13) 
  #include "thermistortable_13.h"
#endif
#if ANY_THERMISTOR_IS(20) 
  #include "thermistortable_20.h"
#endif
#if ANY_THERMISTOR_IS(51) 
  #include "thermistortable_51.h"
#endif
#if ANY_THERMISTOR_IS(52) 
  #include "thermistortable_52.h"
#endif
#if ANY_THERMISTOR_IS(55) 
  #include "thermistortable_55.h"
#endif
#if ANY_THERMISTOR_IS(60) 
  #include "thermistortable_60.h"
#endif
#if ANY_THERMISTOR_IS(66) 
  #include "thermistortable_66.h"
#endif
#if ANY_THERMISTOR_IS(12) 
  #include "thermistortable_12.h"
#endif
#if ANY_THERMISTOR_IS(70) 
  #include "thermistortable_70.h"
#endif
#if ANY_THERMISTOR_IS(75) 
  #include "thermistortable_75.h"
#endif
#if ANY_THERMISTOR_IS(110) 
  #include "thermistortable_110.h"
#endif
#if ANY_THERMISTOR_IS(147) 
  #include "thermistortable_147.h"
#endif
#if ANY_THERMISTOR_IS(1010) 
  #include "thermistortable_1010.h"
#endif
#if ANY_THERMISTOR_IS(1047) 
  #include "thermistortable_1047.h"
#endif
#if ANY_THERMISTOR_IS(998) 
  #include "thermistortable_998.h"
#endif
#if ANY_THERMISTOR_IS(999) 
  #include "thermistortable_999.h"
#endif
#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)
#ifdef THERMISTORHEATER_0
  #define HEATER_0_TEMPTABLE TT_NAME(THERMISTORHEATER_0)
  #define HEATER_0_TEMPTABLE_LEN COUNT(HEATER_0_TEMPTABLE)
#elif defined(HEATER_0_USES_THERMISTOR)
  #error "No heater 0 thermistor table specified"
#else
  #define HEATER_0_TEMPTABLE NULL
  #define HEATER_0_TEMPTABLE_LEN 0
#endif
#ifdef THERMISTORHEATER_1
  #define HEATER_1_TEMPTABLE TT_NAME(THERMISTORHEATER_1)
  #define HEATER_1_TEMPTABLE_LEN COUNT(HEATER_1_TEMPTABLE)
#elif defined(HEATER_1_USES_THERMISTOR)
  #error "No heater 1 thermistor table specified"
#else
  #define HEATER_1_TEMPTABLE NULL
  #define HEATER_1_TEMPTABLE_LEN 0
#endif
#ifdef THERMISTORHEATER_2
  #define HEATER_2_TEMPTABLE TT_NAME(THERMISTORHEATER_2)
  #define HEATER_2_TEMPTABLE_LEN COUNT(HEATER_2_TEMPTABLE)
#elif defined(HEATER_2_USES_THERMISTOR)
  #error "No heater 2 thermistor table specified"
#else
  #define HEATER_2_TEMPTABLE NULL
  #define HEATER_2_TEMPTABLE_LEN 0
#endif
#ifdef THERMISTORHEATER_3
  #define HEATER_3_TEMPTABLE TT_NAME(THERMISTORHEATER_3)
  #define HEATER_3_TEMPTABLE_LEN COUNT(HEATER_3_TEMPTABLE)
#elif defined(HEATER_3_USES_THERMISTOR)
  #error "No heater 3 thermistor table specified"
#else
  #define HEATER_3_TEMPTABLE NULL
  #define HEATER_3_TEMPTABLE_LEN 0
#endif
#ifdef THERMISTORHEATER_4
  #define HEATER_4_TEMPTABLE TT_NAME(THERMISTORHEATER_4)
  #define HEATER_4_TEMPTABLE_LEN COUNT(HEATER_4_TEMPTABLE)
#elif defined(HEATER_4_USES_THERMISTOR)
  #error "No heater 4 thermistor table specified"
#else
  #define HEATER_4_TEMPTABLE NULL
  #define HEATER_4_TEMPTABLE_LEN 0
#endif
#ifdef THERMISTORBED
  #define BEDTEMPTABLE TT_NAME(THERMISTORBED)
  #define BEDTEMPTABLE_LEN COUNT(BEDTEMPTABLE)
#else
  #ifdef BED_USES_THERMISTOR
    #error "No bed thermistor table specified"
  #endif
#endif
#ifndef HEATER_0_RAW_HI_TEMP
  #ifdef HEATER_0_USES_THERMISTOR
    #define HEATER_0_RAW_HI_TEMP 0
    #define HEATER_0_RAW_LO_TEMP 16383
  #else
    #define HEATER_0_RAW_HI_TEMP 16383
    #define HEATER_0_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_1_RAW_HI_TEMP
  #ifdef HEATER_1_USES_THERMISTOR
    #define HEATER_1_RAW_HI_TEMP 0
    #define HEATER_1_RAW_LO_TEMP 16383
  #else
    #define HEATER_1_RAW_HI_TEMP 16383
    #define HEATER_1_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_2_RAW_HI_TEMP
  #ifdef HEATER_2_USES_THERMISTOR
    #define HEATER_2_RAW_HI_TEMP 0
    #define HEATER_2_RAW_LO_TEMP 16383
  #else
    #define HEATER_2_RAW_HI_TEMP 16383
    #define HEATER_2_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_3_RAW_HI_TEMP
  #ifdef HEATER_3_USES_THERMISTOR
    #define HEATER_3_RAW_HI_TEMP 0
    #define HEATER_3_RAW_LO_TEMP 16383
  #else
    #define HEATER_3_RAW_HI_TEMP 16383
    #define HEATER_3_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_4_RAW_HI_TEMP
  #ifdef HEATER_4_USES_THERMISTOR
    #define HEATER_4_RAW_HI_TEMP 0
    #define HEATER_4_RAW_LO_TEMP 16383
  #else
    #define HEATER_4_RAW_HI_TEMP 16383
    #define HEATER_4_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_BED_RAW_HI_TEMP
  #ifdef BED_USES_THERMISTOR
    #define HEATER_BED_RAW_HI_TEMP 0
    #define HEATER_BED_RAW_LO_TEMP 16383
  #else
    #define HEATER_BED_RAW_HI_TEMP 16383
    #define HEATER_BED_RAW_LO_TEMP 0
  #endif
#endif
#endif 
