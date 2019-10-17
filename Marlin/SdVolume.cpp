#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#include "SdVolume.h"
#if !USE_MULTIPLE_CARDS
  uint32_t SdVolume::cacheBlockNumber_;  
  cache_t  SdVolume::cacheBuffer_;       
  Sd2Card* SdVolume::sdCard_;            
  bool     SdVolume::cacheDirty_;        
  uint32_t SdVolume::cacheMirrorBlock_;  
#endif  
bool SdVolume::allocContiguous(uint32_t count, uint32_t* curCluster) {
  uint32_t bgnCluster;
  uint32_t endCluster;
  uint32_t fatEnd = clusterCount_ + 1;
  bool setStart;
  if (*curCluster) {
    bgnCluster = *curCluster + 1;
    setStart = false;
  }
  else {
    bgnCluster = allocSearchStart_;
    setStart = count == 1;
  }
  endCluster = bgnCluster;
  for (uint32_t n = 0;; n++, endCluster++) {
    if (n >= clusterCount_) goto fail;
    if (endCluster > fatEnd) {
      bgnCluster = endCluster = 2;
    }
    uint32_t f;
    if (!fatGet(endCluster, &f)) goto fail;
    if (f != 0) {
      bgnCluster = endCluster + 1;
    }
    else if ((endCluster - bgnCluster + 1) == count) {
      break;
    }
  }
  if (!fatPutEOC(endCluster)) goto fail;
  while (endCluster > bgnCluster) {
    if (!fatPut(endCluster - 1, endCluster)) goto fail;
    endCluster--;
  }
  if (*curCluster != 0) {
    if (!fatPut(*curCluster, bgnCluster)) goto fail;
  }
  *curCluster = bgnCluster;
  if (setStart) allocSearchStart_ = bgnCluster + 1;
  return true;
fail:
  return false;
}
bool SdVolume::cacheFlush() {
  if (cacheDirty_) {
    if (!sdCard_->writeBlock(cacheBlockNumber_, cacheBuffer_.data)) {
      goto fail;
    }
    if (cacheMirrorBlock_) {
      if (!sdCard_->writeBlock(cacheMirrorBlock_, cacheBuffer_.data)) {
        goto fail;
      }
      cacheMirrorBlock_ = 0;
    }
    cacheDirty_ = 0;
  }
  return true;
fail:
  return false;
}
bool SdVolume::cacheRawBlock(uint32_t blockNumber, bool dirty) {
  if (cacheBlockNumber_ != blockNumber) {
    if (!cacheFlush()) goto fail;
    if (!sdCard_->readBlock(blockNumber, cacheBuffer_.data)) goto fail;
    cacheBlockNumber_ = blockNumber;
  }
  if (dirty) cacheDirty_ = true;
  return true;
fail:
  return false;
}
bool SdVolume::chainSize(uint32_t cluster, uint32_t* size) {
  uint32_t s = 0;
  do {
    if (!fatGet(cluster, &cluster)) goto fail;
    s += 512UL << clusterSizeShift_;
  } while (!isEOC(cluster));
  *size = s;
  return true;
fail:
  return false;
}
bool SdVolume::fatGet(uint32_t cluster, uint32_t* value) {
  uint32_t lba;
  if (cluster > (clusterCount_ + 1)) goto fail;
  if (FAT12_SUPPORT && fatType_ == 12) {
    uint16_t index = cluster;
    index += index >> 1;
    lba = fatStartBlock_ + (index >> 9);
    if (!cacheRawBlock(lba, CACHE_FOR_READ)) goto fail;
    index &= 0X1FF;
    uint16_t tmp = cacheBuffer_.data[index];
    index++;
    if (index == 512) {
      if (!cacheRawBlock(lba + 1, CACHE_FOR_READ)) goto fail;
      index = 0;
    }
    tmp |= cacheBuffer_.data[index] << 8;
    *value = cluster & 1 ? tmp >> 4 : tmp & 0XFFF;
    return true;
  }
  if (fatType_ == 16) {
    lba = fatStartBlock_ + (cluster >> 8);
  }
  else if (fatType_ == 32) {
    lba = fatStartBlock_ + (cluster >> 7);
  }
  else {
    goto fail;
  }
  if (lba != cacheBlockNumber_) {
    if (!cacheRawBlock(lba, CACHE_FOR_READ)) goto fail;
  }
  if (fatType_ == 16) {
    *value = cacheBuffer_.fat16[cluster & 0XFF];
  }
  else {
    *value = cacheBuffer_.fat32[cluster & 0X7F] & FAT32MASK;
  }
  return true;
fail:
  return false;
}
bool SdVolume::fatPut(uint32_t cluster, uint32_t value) {
  uint32_t lba;
  if (cluster < 2) goto fail;
  if (cluster > (clusterCount_ + 1)) goto fail;
  if (FAT12_SUPPORT && fatType_ == 12) {
    uint16_t index = cluster;
    index += index >> 1;
    lba = fatStartBlock_ + (index >> 9);
    if (!cacheRawBlock(lba, CACHE_FOR_WRITE)) goto fail;
    if (fatCount_ > 1) cacheMirrorBlock_ = lba + blocksPerFat_;
    index &= 0X1FF;
    uint8_t tmp = value;
    if (cluster & 1) {
      tmp = (cacheBuffer_.data[index] & 0XF) | tmp << 4;
    }
    cacheBuffer_.data[index] = tmp;
    index++;
    if (index == 512) {
      lba++;
      index = 0;
      if (!cacheRawBlock(lba, CACHE_FOR_WRITE)) goto fail;
      if (fatCount_ > 1) cacheMirrorBlock_ = lba + blocksPerFat_;
    }
    tmp = value >> 4;
    if (!(cluster & 1)) {
      tmp = ((cacheBuffer_.data[index] & 0XF0)) | tmp >> 4;
    }
    cacheBuffer_.data[index] = tmp;
    return true;
  }
  if (fatType_ == 16) {
    lba = fatStartBlock_ + (cluster >> 8);
  }
  else if (fatType_ == 32) {
    lba = fatStartBlock_ + (cluster >> 7);
  }
  else {
    goto fail;
  }
  if (!cacheRawBlock(lba, CACHE_FOR_WRITE)) goto fail;
  if (fatType_ == 16) {
    cacheBuffer_.fat16[cluster & 0XFF] = value;
  }
  else {
    cacheBuffer_.fat32[cluster & 0X7F] = value;
  }
  if (fatCount_ > 1) cacheMirrorBlock_ = lba + blocksPerFat_;
  return true;
fail:
  return false;
}
bool SdVolume::freeChain(uint32_t cluster) {
  uint32_t next;
  allocSearchStart_ = 2;
  do {
    if (!fatGet(cluster, &next)) goto fail;
    if (!fatPut(cluster, 0)) goto fail;
    cluster = next;
  } while (!isEOC(cluster));
  return true;
fail:
  return false;
}
int32_t SdVolume::freeClusterCount() {
  uint32_t free = 0;
  uint16_t n;
  uint32_t todo = clusterCount_ + 2;
  if (fatType_ == 16) {
    n = 256;
  }
  else if (fatType_ == 32) {
    n = 128;
  }
  else {
    return -1;
  }
  for (uint32_t lba = fatStartBlock_; todo; todo -= n, lba++) {
    if (!cacheRawBlock(lba, CACHE_FOR_READ)) return -1;
    NOMORE(n, todo);
    if (fatType_ == 16) {
      for (uint16_t i = 0; i < n; i++) {
        if (cacheBuffer_.fat16[i] == 0) free++;
      }
    }
    else {
      for (uint16_t i = 0; i < n; i++) {
        if (cacheBuffer_.fat32[i] == 0) free++;
      }
    }
  }
  return free;
}
bool SdVolume::init(Sd2Card* dev, uint8_t part) {
  uint32_t totalBlocks;
  uint32_t volumeStartBlock = 0;
  fat32_boot_t* fbs;
  sdCard_ = dev;
  fatType_ = 0;
  allocSearchStart_ = 2;
  cacheDirty_ = 0;  
  cacheMirrorBlock_ = 0;
  cacheBlockNumber_ = 0XFFFFFFFF;
  if (part) {
    if (part > 4)goto fail;
    if (!cacheRawBlock(volumeStartBlock, CACHE_FOR_READ)) goto fail;
    part_t* p = &cacheBuffer_.mbr.part[part - 1];
    if ((p->boot & 0X7F) != 0  ||
        p->totalSectors < 100 ||
        p->firstSector == 0) {
      goto fail;
    }
    volumeStartBlock = p->firstSector;
  }
  if (!cacheRawBlock(volumeStartBlock, CACHE_FOR_READ)) goto fail;
  fbs = &cacheBuffer_.fbs32;
  if (fbs->bytesPerSector != 512 ||
      fbs->fatCount == 0 ||
      fbs->reservedSectorCount == 0 ||
      fbs->sectorsPerCluster == 0) {
    goto fail;
  }
  fatCount_ = fbs->fatCount;
  blocksPerCluster_ = fbs->sectorsPerCluster;
  clusterSizeShift_ = 0;
  while (blocksPerCluster_ != _BV(clusterSizeShift_)) {
    if (clusterSizeShift_++ > 7) goto fail;
  }
  blocksPerFat_ = fbs->sectorsPerFat16 ?
                  fbs->sectorsPerFat16 : fbs->sectorsPerFat32;
  fatStartBlock_ = volumeStartBlock + fbs->reservedSectorCount;
  rootDirEntryCount_ = fbs->rootDirEntryCount;
  rootDirStart_ = fatStartBlock_ + fbs->fatCount * blocksPerFat_;
  dataStartBlock_ = rootDirStart_ + ((32 * fbs->rootDirEntryCount + 511) / 512);
  totalBlocks = fbs->totalSectors16 ?
                fbs->totalSectors16 : fbs->totalSectors32;
  clusterCount_ = totalBlocks - (dataStartBlock_ - volumeStartBlock);
  clusterCount_ >>= clusterSizeShift_;
  if (clusterCount_ < 4085) {
    fatType_ = 12;
    if (!FAT12_SUPPORT) goto fail;
  }
  else if (clusterCount_ < 65525) {
    fatType_ = 16;
  }
  else {
    rootDirStart_ = fbs->fat32RootCluster;
    fatType_ = 32;
  }
  return true;
fail:
  return false;
}
#endif
