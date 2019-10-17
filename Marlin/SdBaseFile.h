#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#ifndef SdBaseFile_h
#define SdBaseFile_h
#include "Marlin.h"
#include "SdFatConfig.h"
#include "SdVolume.h"
struct filepos_t {
  uint32_t position;
  uint32_t cluster;
  filepos_t() : position(0), cluster(0) {}
};
uint8_t const O_READ = 0X01;
uint8_t const O_RDONLY = O_READ;
uint8_t const O_WRITE = 0X02;
uint8_t const O_WRONLY = O_WRITE;
uint8_t const O_RDWR = (O_READ | O_WRITE);
uint8_t const O_ACCMODE = (O_READ | O_WRITE);
uint8_t const O_APPEND = 0X04;
uint8_t const O_SYNC = 0X08;
uint8_t const O_TRUNC = 0X10;
uint8_t const O_AT_END = 0X20;
uint8_t const O_CREAT = 0X40;
uint8_t const O_EXCL = 0X80;
uint8_t const LS_DATE = 1;
uint8_t const LS_SIZE = 2;
uint8_t const LS_R = 4;
uint8_t const T_ACCESS = 1;
uint8_t const T_CREATE = 2;
uint8_t const T_WRITE = 4;
uint8_t const FAT_FILE_TYPE_CLOSED = 0;
uint8_t const FAT_FILE_TYPE_NORMAL = 1;
uint8_t const FAT_FILE_TYPE_ROOT_FIXED = 2;
uint8_t const FAT_FILE_TYPE_ROOT32 = 3;
uint8_t const FAT_FILE_TYPE_SUBDIR = 4;
uint8_t const FAT_FILE_TYPE_MIN_DIR = FAT_FILE_TYPE_ROOT_FIXED;
static inline uint16_t FAT_DATE(uint16_t year, uint8_t month, uint8_t day) {
  return (year - 1980) << 9 | month << 5 | day;
}
static inline uint16_t FAT_YEAR(uint16_t fatDate) {
  return 1980 + (fatDate >> 9);
}
static inline uint8_t FAT_MONTH(uint16_t fatDate) {
  return (fatDate >> 5) & 0XF;
}
static inline uint8_t FAT_DAY(uint16_t fatDate) {
  return fatDate & 0X1F;
}
static inline uint16_t FAT_TIME(uint8_t hour, uint8_t minute, uint8_t second) {
  return hour << 11 | minute << 5 | second >> 1;
}
static inline uint8_t FAT_HOUR(uint16_t fatTime) {
  return fatTime >> 11;
}
static inline uint8_t FAT_MINUTE(uint16_t fatTime) {
  return (fatTime >> 5) & 0X3F;
}
static inline uint8_t FAT_SECOND(uint16_t fatTime) {
  return 2 * (fatTime & 0X1F);
}
uint16_t const FAT_DEFAULT_DATE = ((2000 - 1980) << 9) | (1 << 5) | 1;
uint16_t const FAT_DEFAULT_TIME = (1 << 11);
class SdBaseFile {
 public:
  SdBaseFile() : writeError(false), type_(FAT_FILE_TYPE_CLOSED) {}
  SdBaseFile(const char* path, uint8_t oflag);
  ~SdBaseFile() {if (isOpen()) close();}
  bool writeError;
  void getpos(filepos_t* pos);
  void setpos(filepos_t* pos);
  bool close();
  bool contiguousRange(uint32_t* bgnBlock, uint32_t* endBlock);
  bool createContiguous(SdBaseFile* dirFile,
                        const char* path, uint32_t size);
  uint32_t curCluster() const {return curCluster_;}
  uint32_t curPosition() const {return curPosition_;}
  static SdBaseFile* cwd() {return cwd_;}
  static void dateTimeCallback(
    void (*dateTime)(uint16_t* date, uint16_t* time)) {
    dateTime_ = dateTime;
  }
  static void dateTimeCallbackCancel() {dateTime_ = 0;}
  bool dirEntry(dir_t* dir);
  static void dirName(const dir_t& dir, char* name);
  bool exists(const char* name);
  int16_t fgets(char* str, int16_t num, char* delim = 0);
  uint32_t fileSize() const {return fileSize_;}
  uint32_t firstCluster() const {return firstCluster_;}
  bool getFilename(char* name);
  bool isDir() const {return type_ >= FAT_FILE_TYPE_MIN_DIR;}
  bool isFile() const {return type_ == FAT_FILE_TYPE_NORMAL;}
  bool isOpen() const {return type_ != FAT_FILE_TYPE_CLOSED;}
  bool isSubDir() const {return type_ == FAT_FILE_TYPE_SUBDIR;}
  bool isRoot() const {
    return type_ == FAT_FILE_TYPE_ROOT_FIXED || type_ == FAT_FILE_TYPE_ROOT32;
  }
  void ls(uint8_t flags = 0, uint8_t indent = 0);
  bool mkdir(SdBaseFile* dir, const char* path, bool pFlag = true);
  bool makeDir(SdBaseFile* dir, const char* path) {
    return mkdir(dir, path, false);
  }
  bool open(SdBaseFile* dirFile, uint16_t index, uint8_t oflag);
  bool open(SdBaseFile* dirFile, const char* path, uint8_t oflag);
  bool open(const char* path, uint8_t oflag = O_READ);
  bool openNext(SdBaseFile* dirFile, uint8_t oflag);
  bool openRoot(SdVolume* vol);
  int peek();
  static void printFatDate(uint16_t fatDate);
  static void printFatTime(uint16_t fatTime);
  bool printName();
  int16_t read();
  int16_t read(void* buf, uint16_t nbyte);
  int8_t readDir(dir_t* dir, char* longFilename);
  static bool remove(SdBaseFile* dirFile, const char* path);
  bool remove();
  void rewind() {seekSet(0);}
  bool rename(SdBaseFile* dirFile, const char* newPath);
  bool rmdir();
  bool rmDir() {return rmdir();}
  bool rmRfStar();
  bool seekCur(int32_t offset) {
    return seekSet(curPosition_ + offset);
  }
  bool seekEnd(int32_t offset = 0) {return seekSet(fileSize_ + offset);}
  bool seekSet(uint32_t pos);
  bool sync();
  bool timestamp(SdBaseFile* file);
  bool timestamp(uint8_t flag, uint16_t year, uint8_t month, uint8_t day,
                 uint8_t hour, uint8_t minute, uint8_t second);
  uint8_t type() const {return type_;}
  bool truncate(uint32_t size);
  SdVolume* volume() const {return vol_;}
  int16_t write(const void* buf, uint16_t nbyte);
 private:
  friend class SdFat;
  static SdBaseFile* cwd_;
  static void (*dateTime_)(uint16_t* date, uint16_t* time);
  static uint8_t const F_OFLAG = (O_ACCMODE | O_APPEND | O_SYNC);
  static uint8_t const F_FILE_DIR_DIRTY = 0X80;
  uint8_t   flags_;         
  uint8_t   fstate_;        
  uint8_t   type_;          
  uint32_t  curCluster_;    
  uint32_t  curPosition_;   
  uint32_t  dirBlock_;      
  uint8_t   dirIndex_;      
  uint32_t  fileSize_;      
  uint32_t  firstCluster_;  
  SdVolume* vol_;           
  bool openParent(SdBaseFile* dir);
  bool addCluster();
  bool addDirCluster();
  dir_t* cacheDirEntry(uint8_t action);
  int8_t lsPrintNext(uint8_t flags, uint8_t indent);
  static bool make83Name(const char* str, uint8_t* name, const char** ptr);
  bool mkdir(SdBaseFile* parent, const uint8_t dname[11]);
  bool open(SdBaseFile* dirFile, const uint8_t dname[11], uint8_t oflag);
  bool openCachedEntry(uint8_t cacheIndex, uint8_t oflags);
  dir_t* readDirCache();
  static void printDirName(const dir_t& dir,
                           uint8_t width, bool printSlash);
#if ALLOW_DEPRECATED_FUNCTIONS && !defined(DOXYGEN)
 public:
  bool contiguousRange(uint32_t& bgnBlock, uint32_t& endBlock) {  
    return contiguousRange(&bgnBlock, &endBlock);
  }
  bool createContiguous(SdBaseFile& dirFile,  
                        const char* path, uint32_t size) {
    return createContiguous(&dirFile, path, size);
  }
  static void dateTimeCallback(
    void (*dateTime)(uint16_t &date, uint16_t &time)) {  
    oldDateTime_ = dateTime;
    dateTime_ = dateTime ? oldToNew : 0;
  }
  bool dirEntry(dir_t& dir) {return dirEntry(&dir);}  
  bool mkdir(SdBaseFile& dir, const char* path) {  
    return mkdir(&dir, path);
  }
  bool open(SdBaseFile& dirFile, 
            const char* path, uint8_t oflag) {
    return open(&dirFile, path, oflag);
  }
  bool open(SdBaseFile& dirFile, const char* path) {  
    return open(dirFile, path, O_RDWR);
  }
  bool open(SdBaseFile& dirFile, uint16_t index, uint8_t oflag) {  
    return open(&dirFile, index, oflag);
  }
  bool openRoot(SdVolume& vol) {return openRoot(&vol);}  
  int8_t readDir(dir_t& dir, char* longFilename) {return readDir(&dir, longFilename);}  
  static bool remove(SdBaseFile& dirFile, const char* path) {  
    return remove(&dirFile, path);
  }
 private:
  static void (*oldDateTime_)(uint16_t &date, uint16_t &time);  
  static void oldToNew(uint16_t* date, uint16_t* time) {
    uint16_t d;
    uint16_t t;
    oldDateTime_(d, t);
    *date = d;
    *time = t;
  }
#endif  
};
#endif  
#endif
