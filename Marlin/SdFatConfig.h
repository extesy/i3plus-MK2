#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#ifndef SdFatConfig_h
  #define SdFatConfig_h
  #include <stdint.h>
  #define USE_MULTIPLE_CARDS 0
  #define ENDL_CALLS_FLUSH 0
  #define ALLOW_DEPRECATED_FUNCTIONS 1
  #define FAT12_SUPPORT 0
  #define SPI_SD_INIT_RATE 5
  #define SET_SPI_SS_HIGH 1
  #define MEGA_SOFT_SPI 0
  #define USE_SOFTWARE_SPI 0
  #define SOFT_SPI_CS_PIN 10
  #define SOFT_SPI_MOSI_PIN 11
  #define SOFT_SPI_MISO_PIN 12
  #define SOFT_SPI_SCK_PIN 13
  #define USE_CXA_PURE_VIRTUAL 1
  #define FILENAME_LENGTH 13
  #define MAX_VFAT_ENTRIES (2)
  #define LONG_FILENAME_LENGTH (FILENAME_LENGTH*MAX_VFAT_ENTRIES+1)
#endif  
#endif
