#if ENABLED(USE_AUTOMATIC_VERSIONING)
  #include "_Version.h"
#else
  #define SHORT_BUILD_VERSION "1.1.4"
  #define DETAILED_BUILD_VERSION SHORT_BUILD_VERSION " (Github)"
  #define STRING_DISTRIBUTION_DATE "2017-07-04 12:00"
  #define REQUIRED_CONFIGURATION_H_VERSION 010100
  #define REQUIRED_CONFIGURATION_ADV_H_VERSION 010100
  #define PROTOCOL_VERSION "1.0"
    #define MACHINE_NAME "3D Printer"
  #define SOURCE_CODE_URL "https://github.com/MarlinFirmware/Marlin"
  /**
   * Default generic printer UUID.
   */
  #define DEFAULT_MACHINE_UUID "cede2a2f-41a2-4748-9b12-c55c62f367ff"
  /**
   * The WEBSITE_URL is the location where users can get more information such as
   * documentation about a specific Marlin release.
   */
  #define WEBSITE_URL "http://marlinfw.org"
#endif // USE_AUTOMATIC_VERSIONING
