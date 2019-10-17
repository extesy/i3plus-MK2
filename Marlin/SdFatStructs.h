#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#ifndef SdFatStructs_h
#define SdFatStructs_h
#define PACKED __attribute__((__packed__))
uint8_t const BOOTSIG0 = 0X55;
uint8_t const BOOTSIG1 = 0XAA;
uint8_t const EXTENDED_BOOT_SIG = 0X29;
struct partitionTable {
  uint8_t  boot;
  uint8_t  beginHead;
  unsigned beginSector : 6;
  unsigned beginCylinderHigh : 2;
  uint8_t  beginCylinderLow;
  uint8_t  type;
  uint8_t  endHead;
  unsigned endSector : 6;
  unsigned endCylinderHigh : 2;
  uint8_t  endCylinderLow;
  uint32_t firstSector;
  uint32_t totalSectors;
} PACKED;
typedef struct partitionTable part_t;
struct masterBootRecord {
  uint8_t  codeArea[440];
  uint32_t diskSignature;
  uint16_t usuallyZero;
  part_t   part[4];
  uint8_t  mbrSig0;
  uint8_t  mbrSig1;
} PACKED;
typedef struct masterBootRecord mbr_t;
struct fat_boot {
  uint8_t jump[3];
  char    oemId[8];
  uint16_t bytesPerSector;
  uint8_t  sectorsPerCluster;
  uint16_t reservedSectorCount;
  uint8_t  fatCount;
  uint16_t rootDirEntryCount;
  uint16_t totalSectors16;
  uint8_t  mediaType;
  uint16_t sectorsPerFat16;
  uint16_t sectorsPerTrack;
  uint16_t headCount;
  uint32_t hidddenSectors;
  uint32_t totalSectors32;
  uint8_t  driveNumber;
  uint8_t  reserved1;
  uint8_t  bootSignature;
  uint32_t volumeSerialNumber;
  char     volumeLabel[11];
  char     fileSystemType[8];
  uint8_t  bootCode[448];
  uint8_t  bootSectorSig0;
  uint8_t  bootSectorSig1;
} PACKED;
typedef struct fat_boot fat_boot_t;
struct fat32_boot {
  uint8_t jump[3];
  char    oemId[8];
  uint16_t bytesPerSector;
  uint8_t  sectorsPerCluster;
  uint16_t reservedSectorCount;
  uint8_t  fatCount;
  uint16_t rootDirEntryCount;
  uint16_t totalSectors16;
  uint8_t  mediaType;
  uint16_t sectorsPerFat16;
  uint16_t sectorsPerTrack;
  uint16_t headCount;
  uint32_t hidddenSectors;
  uint32_t totalSectors32;
  uint32_t sectorsPerFat32;
  uint16_t fat32Flags;
  uint16_t fat32Version;
  uint32_t fat32RootCluster;
  uint16_t fat32FSInfo;
  uint16_t fat32BackBootBlock;
  uint8_t  fat32Reserved[12];
  uint8_t  driveNumber;
  uint8_t  reserved1;
  uint8_t  bootSignature;
  uint32_t volumeSerialNumber;
  char     volumeLabel[11];
  char     fileSystemType[8];
  uint8_t  bootCode[420];
  uint8_t  bootSectorSig0;
  uint8_t  bootSectorSig1;
} PACKED;
typedef struct fat32_boot fat32_boot_t;
uint32_t const FSINFO_LEAD_SIG = 0x41615252;
uint32_t const FSINFO_STRUCT_SIG = 0x61417272;
struct fat32_fsinfo {
  uint32_t  leadSignature;
  uint8_t  reserved1[480];
  uint32_t  structSignature;
  uint32_t freeCount;
  uint32_t nextFree;
  uint8_t  reserved2[12];
  uint8_t  tailSignature[4];
} PACKED;
typedef struct fat32_fsinfo fat32_fsinfo_t;
uint16_t const FAT12EOC = 0XFFF;
uint16_t const FAT12EOC_MIN = 0XFF8;
uint16_t const FAT16EOC = 0XFFFF;
uint16_t const FAT16EOC_MIN = 0XFFF8;
uint32_t const FAT32EOC = 0X0FFFFFFF;
uint32_t const FAT32EOC_MIN = 0X0FFFFFF8;
uint32_t const FAT32MASK = 0X0FFFFFFF;
struct directoryEntry {
  uint8_t  name[11];
  uint8_t  attributes;
  uint8_t  reservedNT;
  uint8_t  creationTimeTenths;
  uint16_t creationTime;
  uint16_t creationDate;
  uint16_t lastAccessDate;
  uint16_t firstClusterHigh;
  uint16_t lastWriteTime;
  uint16_t lastWriteDate;
  uint16_t firstClusterLow;
  uint32_t fileSize;
} PACKED;
struct directoryVFATEntry {
  uint8_t  sequenceNumber;
  uint16_t name1[5];
  uint8_t  attributes;
  uint8_t  reservedNT;
  uint8_t  checksum;
  uint16_t name2[6];
  uint16_t firstClusterLow;
  uint16_t name3[2];
} PACKED;
typedef struct directoryEntry dir_t;
typedef struct directoryVFATEntry vfat_t;
uint8_t const DIR_NAME_0XE5 = 0X05;
uint8_t const DIR_NAME_DELETED = 0XE5;
uint8_t const DIR_NAME_FREE = 0X00;
uint8_t const DIR_ATT_READ_ONLY = 0X01;
uint8_t const DIR_ATT_HIDDEN = 0X02;
uint8_t const DIR_ATT_SYSTEM = 0X04;
uint8_t const DIR_ATT_VOLUME_ID = 0X08;
uint8_t const DIR_ATT_DIRECTORY = 0X10;
uint8_t const DIR_ATT_ARCHIVE = 0X20;
uint8_t const DIR_ATT_LONG_NAME = 0X0F;
uint8_t const DIR_ATT_LONG_NAME_MASK = 0X3F;
uint8_t const DIR_ATT_DEFINED_BITS = 0X3F;
static inline uint8_t DIR_IS_LONG_NAME(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_LONG_NAME_MASK) == DIR_ATT_LONG_NAME;
}
uint8_t const DIR_ATT_FILE_TYPE_MASK = (DIR_ATT_VOLUME_ID | DIR_ATT_DIRECTORY);
static inline uint8_t DIR_IS_FILE(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_FILE_TYPE_MASK) == 0;
}
static inline uint8_t DIR_IS_SUBDIR(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_FILE_TYPE_MASK) == DIR_ATT_DIRECTORY;
}
static inline uint8_t DIR_IS_FILE_OR_SUBDIR(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_VOLUME_ID) == 0;
}
#endif  
#endif
