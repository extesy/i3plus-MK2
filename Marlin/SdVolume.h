#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#ifndef SdVolume_h
#define SdVolume_h
#include "SdFatConfig.h"
#include "Sd2Card.h"
#include "SdFatStructs.h"
union cache_t {
  uint8_t  data[512];
  uint16_t fat16[256];
  uint32_t fat32[128];
  dir_t    dir[16];
  mbr_t    mbr;
  fat_boot_t fbs;
  fat32_boot_t fbs32;
  fat32_fsinfo_t fsinfo;
};
class SdVolume {
 public:
  SdVolume() : fatType_(0) {}
  cache_t* cacheClear() {
    if (!cacheFlush()) return 0;
    cacheBlockNumber_ = 0XFFFFFFFF;
    return &cacheBuffer_;
  }
  bool init(Sd2Card* dev) { return init(dev, 1) ? true : init(dev, 0);}
  bool init(Sd2Card* dev, uint8_t part);
  uint8_t blocksPerCluster() const {return blocksPerCluster_;}
  uint32_t blocksPerFat()  const {return blocksPerFat_;}
  uint32_t clusterCount() const {return clusterCount_;}
  uint8_t clusterSizeShift() const {return clusterSizeShift_;}
  uint32_t dataStartBlock() const {return dataStartBlock_;}
  uint8_t fatCount() const {return fatCount_;}
  uint32_t fatStartBlock() const {return fatStartBlock_;}
  uint8_t fatType() const {return fatType_;}
  int32_t freeClusterCount();
  uint32_t rootDirEntryCount() const {return rootDirEntryCount_;}
  uint32_t rootDirStart() const {return rootDirStart_;}
  Sd2Card* sdCard() {return sdCard_;}
  bool dbgFat(uint32_t n, uint32_t* v) {return fatGet(n, v);}
 private:
  friend class SdBaseFile;
  static bool const CACHE_FOR_READ = false;
  static bool const CACHE_FOR_WRITE = true;
#if USE_MULTIPLE_CARDS
  cache_t cacheBuffer_;        
  uint32_t cacheBlockNumber_;  
  Sd2Card* sdCard_;            
  bool cacheDirty_;            
  uint32_t cacheMirrorBlock_;  
#else  
  static cache_t cacheBuffer_;        
  static uint32_t cacheBlockNumber_;  
  static Sd2Card* sdCard_;            
  static bool cacheDirty_;            
  static uint32_t cacheMirrorBlock_;  
#endif  
  uint32_t allocSearchStart_;   
  uint8_t blocksPerCluster_;    
  uint32_t blocksPerFat_;       
  uint32_t clusterCount_;       
  uint8_t clusterSizeShift_;    
  uint32_t dataStartBlock_;     
  uint8_t fatCount_;            
  uint32_t fatStartBlock_;      
  uint8_t fatType_;             
  uint16_t rootDirEntryCount_;  
  uint32_t rootDirStart_;       
  bool allocContiguous(uint32_t count, uint32_t* curCluster);
  uint8_t blockOfCluster(uint32_t position) const {
    return (position >> 9) & (blocksPerCluster_ - 1);
  }
  uint32_t clusterStartBlock(uint32_t cluster) const {
    return dataStartBlock_ + ((cluster - 2) << clusterSizeShift_);
  }
  uint32_t blockNumber(uint32_t cluster, uint32_t position) const {
    return clusterStartBlock(cluster) + blockOfCluster(position);
  }
  cache_t* cache() {return &cacheBuffer_;}
  uint32_t cacheBlockNumber() {return cacheBlockNumber_;}
#if USE_MULTIPLE_CARDS
  bool cacheFlush();
  bool cacheRawBlock(uint32_t blockNumber, bool dirty);
#else  
  static bool cacheFlush();
  static bool cacheRawBlock(uint32_t blockNumber, bool dirty);
#endif  
  void cacheSetBlockNumber(uint32_t blockNumber, bool dirty) {
    cacheDirty_ = dirty;
    cacheBlockNumber_  = blockNumber;
  }
  void cacheSetDirty() {cacheDirty_ |= CACHE_FOR_WRITE;}
  bool chainSize(uint32_t beginCluster, uint32_t* size);
  bool fatGet(uint32_t cluster, uint32_t* value);
  bool fatPut(uint32_t cluster, uint32_t value);
  bool fatPutEOC(uint32_t cluster) {
    return fatPut(cluster, 0x0FFFFFFF);
  }
  bool freeChain(uint32_t cluster);
  bool isEOC(uint32_t cluster) const {
    if (FAT12_SUPPORT && fatType_ == 12) return  cluster >= FAT12EOC_MIN;
    if (fatType_ == 16) return cluster >= FAT16EOC_MIN;
    return  cluster >= FAT32EOC_MIN;
  }
  bool readBlock(uint32_t block, uint8_t* dst) {
    return sdCard_->readBlock(block, dst);
  }
  bool writeBlock(uint32_t block, const uint8_t* dst) {
    return sdCard_->writeBlock(block, dst);
  }
#if ALLOW_DEPRECATED_FUNCTIONS && !defined(DOXYGEN)
 public:
  bool init(Sd2Card& dev) {return init(&dev);}  
  bool init(Sd2Card& dev, uint8_t part) {  
    return init(&dev, part);
  }
#endif  
};
#endif  
#endif
