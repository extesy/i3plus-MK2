#ifndef CARDREADER_H
#define CARDREADER_H
#include "MarlinConfig.h"
#if ENABLED(SDSUPPORT)
#define MAX_DIR_DEPTH 10          
#include "SdFile.h"
#include "types.h"
#include "enum.h"
class CardReader {
public:
  CardReader();
  void initsd();
  void write_command(char *buf);
  #ifdef FYS_ULTILCD2_COMPATIBLE
    bool write_string(char* buffer);
    FORCE_INLINE int16_t fgets(char* str, int16_t num) { return file.fgets(str, num, NULL); }
    FORCE_INLINE int errorCode() { return card.errorCode(); }
    FORCE_INLINE bool atRoot() { return workDirDepth == 0; }
    FORCE_INLINE void clearError() { card.error(0); }
    FORCE_INLINE uint32_t getFilePos() { return sdpos; }
    FORCE_INLINE uint32_t getFileSize() { return filesize; }
  #endif
  void checkautostart(bool x);
  void openFile(char* name, bool read, bool push_current=false);
  void openLogFile(char* name);
  void removeFile(char* name);
  void closefile(bool store_location=false);
  void release();
  void openAndPrintFile(const char *name);
  void startFileprint();
  void stopSDPrint();
  void getStatus();
  void printingHasFinished();
  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
    void printLongPath(char *path);
  #endif
  void getfilename(uint16_t nr, const char* const match=NULL);
  uint16_t getnrfilenames();
  void getAbsFilename(char *t);
  void ls();
  void chdir(const char *relpath);
  void updir();
  void setroot();
  #if ENABLED(SDCARD_SORT_ALPHA)
    void presort();
    void getfilename_sorted(const uint16_t nr);
    #if ENABLED(SDSORT_GCODE)
      FORCE_INLINE void setSortOn(bool b) { sort_alpha = b; presort(); }
      FORCE_INLINE void setSortFolders(int i) { sort_folders = i; presort(); }
    #endif
  #endif
  FORCE_INLINE void pauseSDPrint() { sdprinting = false; }
  FORCE_INLINE bool isFileOpen() { return file.isOpen(); }
  FORCE_INLINE bool eof() { return sdpos >= filesize; }
  FORCE_INLINE int16_t get() { sdpos = file.curPosition(); return (int16_t)file.read(); }
  FORCE_INLINE void setIndex(long index) { sdpos = index; file.seekSet(index); }
  FORCE_INLINE uint8_t percentDone() { return (isFileOpen() && filesize) ? sdpos / ((filesize + 99) / 100) : 0; }
  FORCE_INLINE char* getWorkDirName() { workDir.getFilename(filename); return filename; }
public:
  bool saving, logging, sdprinting, cardOK, filenameIsDir;
  char filename[FILENAME_LENGTH], longFilename[LONG_FILENAME_LENGTH];
  int autostart_index;
  SdFile file;
private:
  SdFile root, *curDir, workDir, workDirParents[MAX_DIR_DEPTH];
  uint8_t workDirDepth;
  #if ENABLED(SDCARD_SORT_ALPHA)
    uint16_t sort_count;        
    #if ENABLED(SDSORT_GCODE)
      bool sort_alpha;          
      int sort_folders;         
    #endif
    #if ENABLED(SDSORT_DYNAMIC_RAM)
      uint8_t *sort_order;
    #else
      uint8_t sort_order[SDSORT_LIMIT];
    #endif
    #if ENABLED(SDSORT_USES_RAM)
      #if ENABLED(SDSORT_CACHE_NAMES)
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          char **sortshort, **sortnames;
        #else
          char sortshort[SDSORT_LIMIT][FILENAME_LENGTH];
          char sortnames[SDSORT_LIMIT][FILENAME_LENGTH];
        #endif
      #elif DISABLED(SDSORT_USES_STACK)
        char sortnames[SDSORT_LIMIT][FILENAME_LENGTH];
      #endif
      #if HAS_FOLDER_SORTING
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          uint8_t *isDir;
        #elif ENABLED(SDSORT_CACHE_NAMES) || DISABLED(SDSORT_USES_STACK)
          uint8_t isDir[(SDSORT_LIMIT+7)>>3];
        #endif
      #endif
    #endif 
  #endif 
  Sd2Card card;
  SdVolume volume;
  #define SD_PROCEDURE_DEPTH 1
  #define MAXPATHNAMELENGTH (FILENAME_LENGTH*MAX_DIR_DEPTH + MAX_DIR_DEPTH + 1)
  uint8_t file_subcall_ctr;
  uint32_t filespos[SD_PROCEDURE_DEPTH];
  char proc_filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
  uint32_t filesize;
  uint32_t sdpos;
  millis_t next_autostart_ms;
  bool autostart_stilltocheck; 
  LsAction lsAction; 
  uint16_t nrFiles; 
  char* diveDirName;
  void lsDive(const char *prepend, SdFile parent, const char * const match=NULL);
  #if ENABLED(SDCARD_SORT_ALPHA)
    void flush_presort();
  #endif
};
extern CardReader card;
#define IS_SD_PRINTING (card.sdprinting)
#if PIN_EXISTS(SD_DETECT)
  #if ENABLED(SD_DETECT_INVERTED)
    #define IS_SD_INSERTED (READ(SD_DETECT_PIN) != 0)
  #else
    #define IS_SD_INSERTED (READ(SD_DETECT_PIN) == 0)
  #endif
#else
  #define IS_SD_INSERTED true
#endif
#else
#define IS_SD_PRINTING (false)
#endif 
#endif 
