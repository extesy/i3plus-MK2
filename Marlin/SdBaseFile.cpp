#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#include "SdBaseFile.h"
SdBaseFile* SdBaseFile::cwd_ = 0;
void (*SdBaseFile::dateTime_)(uint16_t* date, uint16_t* time) = 0;
bool SdBaseFile::addCluster() {
  if (!vol_->allocContiguous(1, &curCluster_)) goto fail;
  if (firstCluster_ == 0) {
    firstCluster_ = curCluster_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  return true;
 fail:
  return false;
}
bool SdBaseFile::addDirCluster() {
  uint32_t block;
  if (fileSize_ / sizeof(dir_t) >= 0XFFFF) goto fail;
  if (!addCluster()) goto fail;
  if (!vol_->cacheFlush()) goto fail;
  block = vol_->clusterStartBlock(curCluster_);
  vol_->cacheSetBlockNumber(block, true);
  memset(vol_->cacheBuffer_.data, 0, 512);
  for (uint8_t i = 1; i < vol_->blocksPerCluster_; i++) {
    if (!vol_->writeBlock(block + i, vol_->cacheBuffer_.data)) goto fail;
  }
  fileSize_ += 512UL << vol_->clusterSizeShift_;
  return true;
fail:
  return false;
}
dir_t* SdBaseFile::cacheDirEntry(uint8_t action) {
  if (!vol_->cacheRawBlock(dirBlock_, action)) goto fail;
  return vol_->cache()->dir + dirIndex_;
fail:
  return 0;
}
bool SdBaseFile::close() {
  bool rtn = sync();
  type_ = FAT_FILE_TYPE_CLOSED;
  return rtn;
}
bool SdBaseFile::contiguousRange(uint32_t* bgnBlock, uint32_t* endBlock) {
  if (firstCluster_ == 0) goto fail;
  for (uint32_t c = firstCluster_; ; c++) {
    uint32_t next;
    if (!vol_->fatGet(c, &next)) goto fail;
    if (next != (c + 1)) {
      if (!vol_->isEOC(next)) goto fail;
      *bgnBlock = vol_->clusterStartBlock(firstCluster_);
      *endBlock = vol_->clusterStartBlock(c)
                  + vol_->blocksPerCluster_ - 1;
      return true;
    }
  }
fail:
  return false;
}
bool SdBaseFile::createContiguous(SdBaseFile* dirFile,
                                  const char* path, uint32_t size) {
  uint32_t count;
  if (size == 0) goto fail;
  if (!open(dirFile, path, O_CREAT | O_EXCL | O_RDWR)) goto fail;
  count = ((size - 1) >> (vol_->clusterSizeShift_ + 9)) + 1;
  if (!vol_->allocContiguous(count, &firstCluster_)) {
    remove();
    goto fail;
  }
  fileSize_ = size;
  flags_ |= F_FILE_DIR_DIRTY;
  return sync();
fail:
  return false;
}
bool SdBaseFile::dirEntry(dir_t* dir) {
  dir_t* p;
  if (!sync()) goto fail;
  p = cacheDirEntry(SdVolume::CACHE_FOR_READ);
  if (!p) goto fail;
  memcpy(dir, p, sizeof(dir_t));
  return true;
fail:
  return false;
}
void SdBaseFile::dirName(const dir_t& dir, char* name) {
  uint8_t j = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ')continue;
    if (i == 8) name[j++] = '.';
    name[j++] = dir.name[i];
  }
  name[j] = 0;
}
bool SdBaseFile::exists(const char* name) {
  SdBaseFile file;
  return file.open(this, name, O_READ);
}
int16_t SdBaseFile::fgets(char* str, int16_t num, char* delim) {
  char ch;
  int16_t n = 0;
  int16_t r = -1;
  while ((n + 1) < num && (r = read(&ch, 1)) == 1) {
    if (ch == '\r') continue;
    str[n++] = ch;
    if (!delim) {
      if (ch == '\n') break;
    }
    else {
      if (strchr(delim, ch)) break;
    }
  }
  if (r < 0) {
    return -1;
  }
  str[n] = '\0';
  return n;
}
bool SdBaseFile::getFilename(char* name) {
  if (!isOpen()) return false;
  if (isRoot()) {
    name[0] = '/';
    name[1] = '\0';
    return true;
  }
  dir_t* p = cacheDirEntry(SdVolume::CACHE_FOR_READ);
  if (!p) return false;
  dirName(*p, name);
  return true;
}
void SdBaseFile::getpos(filepos_t* pos) {
  pos->position = curPosition_;
  pos->cluster = curCluster_;
}
void SdBaseFile::ls(uint8_t flags, uint8_t indent) {
  rewind();
  int8_t status;
  while ((status = lsPrintNext(flags, indent))) {
    if (status > 1 && (flags & LS_R)) {
      uint16_t index = curPosition() / 32 - 1;
      SdBaseFile s;
      if (s.open(this, index, O_READ)) s.ls(flags, indent + 2);
      seekSet(32 * (index + 1));
    }
  }
}
int8_t SdBaseFile::lsPrintNext(uint8_t flags, uint8_t indent) {
  dir_t dir;
  uint8_t w = 0;
  while (1) {
    if (read(&dir, sizeof(dir)) != sizeof(dir)) return 0;
    if (dir.name[0] == DIR_NAME_FREE) return 0;
    if (dir.name[0] != DIR_NAME_DELETED && dir.name[0] != '.'
        && DIR_IS_FILE_OR_SUBDIR(&dir)) break;
  }
  for (uint8_t i = 0; i < indent; i++) MYSERIAL.write(' ');
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ')continue;
    if (i == 8) {
      MYSERIAL.write('.');
      w++;
    }
    MYSERIAL.write(dir.name[i]);
    w++;
  }
  if (DIR_IS_SUBDIR(&dir)) {
    MYSERIAL.write('/');
    w++;
  }
  if (flags & (LS_DATE | LS_SIZE)) {
    while (w++ < 14) MYSERIAL.write(' ');
  }
  if (flags & LS_DATE) {
    MYSERIAL.write(' ');
    printFatDate(dir.lastWriteDate);
    MYSERIAL.write(' ');
    printFatTime(dir.lastWriteTime);
  }
  if (!DIR_IS_SUBDIR(&dir) && (flags & LS_SIZE)) {
    MYSERIAL.write(' ');
    MYSERIAL.print(dir.fileSize);
  }
  MYSERIAL.println();
  return DIR_IS_FILE(&dir) ? 1 : 2;
}
bool SdBaseFile::make83Name(const char* str, uint8_t* name, const char** ptr) {
  uint8_t c;
  uint8_t n = 7;  
  uint8_t i = 0;
  while (i < 11) name[i++] = ' ';
  i = 0;
  while (*str != '\0' && *str != '/') {
    c = *str++;
    if (c == '.') {
      if (n == 10) goto fail;  
      n = 10;  
      i = 8;   
    }
    else {
      PGM_P p = PSTR("|<>^+=?/[];,*\"\\");
      uint8_t b;
      while ((b = pgm_read_byte(p++))) if (b == c) goto fail;
      if (i > n || c < 0x21 || c == 0x7F) goto fail;
      name[i++] = (c < 'a' || c > 'z') ? (c) : (c + ('A' - 'a'));
    }
  }
  *ptr = str;
  return name[0] != ' ';
fail:
  return false;
}
bool SdBaseFile::mkdir(SdBaseFile* parent, const char* path, bool pFlag) {
  uint8_t dname[11];
  SdBaseFile dir1, dir2;
  SdBaseFile* sub = &dir1;
  SdBaseFile* start = parent;
  if (!parent || isOpen()) goto fail;
  if (*path == '/') {
    while (*path == '/') path++;
    if (!parent->isRoot()) {
      if (!dir2.openRoot(parent->vol_)) goto fail;
      parent = &dir2;
    }
  }
  while (1) {
    if (!make83Name(path, dname, &path)) goto fail;
    while (*path == '/') path++;
    if (!*path) break;
    if (!sub->open(parent, dname, O_READ)) {
      if (!pFlag || !sub->mkdir(parent, dname)) {
        goto fail;
      }
    }
    if (parent != start) parent->close();
    parent = sub;
    sub = parent != &dir1 ? &dir1 : &dir2;
  }
  return mkdir(parent, dname);
fail:
  return false;
}
bool SdBaseFile::mkdir(SdBaseFile* parent, const uint8_t dname[11]) {
  uint32_t block;
  dir_t d;
  dir_t* p;
  if (!parent->isDir()) goto fail;
  if (!open(parent, dname, O_CREAT | O_EXCL | O_RDWR)) goto fail;
  flags_ = O_READ;
  type_ = FAT_FILE_TYPE_SUBDIR;
  if (!addDirCluster())goto fail;
  if (!sync()) goto fail;
  p = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!p) goto fail;
  p->attributes = DIR_ATT_DIRECTORY;
  memcpy(&d, p, sizeof(d));
  d.name[0] = '.';
  for (uint8_t i = 1; i < 11; i++) d.name[i] = ' ';
  block = vol_->clusterStartBlock(firstCluster_);
  if (!vol_->cacheRawBlock(block, SdVolume::CACHE_FOR_WRITE)) goto fail;
  memcpy(&vol_->cache()->dir[0], &d, sizeof(d));
  d.name[1] = '.';
  if (parent->isRoot()) {
    d.firstClusterLow = 0;
    d.firstClusterHigh = 0;
  }
  else {
    d.firstClusterLow = parent->firstCluster_ & 0XFFFF;
    d.firstClusterHigh = parent->firstCluster_ >> 16;
  }
  memcpy(&vol_->cache()->dir[1], &d, sizeof(d));
  return vol_->cacheFlush();
fail:
  return false;
}
bool SdBaseFile::open(const char* path, uint8_t oflag) {
  return open(cwd_, path, oflag);
}
bool SdBaseFile::open(SdBaseFile* dirFile, const char* path, uint8_t oflag) {
  uint8_t dname[11];
  SdBaseFile dir1, dir2;
  SdBaseFile* parent = dirFile;
  SdBaseFile* sub = &dir1;
  if (!dirFile) goto fail;
  if (isOpen()) goto fail;
  if (*path == '/') {
    while (*path == '/') path++;
    if (!dirFile->isRoot()) {
      if (!dir2.openRoot(dirFile->vol_)) goto fail;
      parent = &dir2;
    }
  }
  while (1) {
    if (!make83Name(path, dname, &path)) goto fail;
    while (*path == '/') path++;
    if (!*path) break;
    if (!sub->open(parent, dname, O_READ)) goto fail;
    if (parent != dirFile) parent->close();
    parent = sub;
    sub = parent != &dir1 ? &dir1 : &dir2;
  }
  return open(parent, dname, oflag);
fail:
  return false;
}
bool SdBaseFile::open(SdBaseFile* dirFile,
                      const uint8_t dname[11], uint8_t oflag) {
  bool emptyFound = false;
  bool fileFound = false;
  uint8_t index;
  dir_t* p;
  vol_ = dirFile->vol_;
  dirFile->rewind();
  while (dirFile->curPosition_ < dirFile->fileSize_) {
    index = 0XF & (dirFile->curPosition_ >> 5);
    p = dirFile->readDirCache();
    if (!p) goto fail;
    if (p->name[0] == DIR_NAME_FREE || p->name[0] == DIR_NAME_DELETED) {
      if (!emptyFound) {
        dirBlock_ = dirFile->vol_->cacheBlockNumber();
        dirIndex_ = index;
        emptyFound = true;
      }
      if (p->name[0] == DIR_NAME_FREE) break;
    }
    else if (!memcmp(dname, p->name, 11)) {
      fileFound = true;
      break;
    }
  }
  if (fileFound) {
    if (oflag & O_EXCL) goto fail;
  }
  else {
    if (!(oflag & O_CREAT) || !(oflag & O_WRITE)) goto fail;
    if (emptyFound) {
      index = dirIndex_;
      p = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
      if (!p) goto fail;
    }
    else {
      if (dirFile->type_ == FAT_FILE_TYPE_ROOT_FIXED) goto fail;
      if (!dirFile->addDirCluster()) goto fail;
      p = dirFile->vol_->cache()->dir;
      index = 0;
    }
    memset(p, 0, sizeof(*p));
    memcpy(p->name, dname, 11);
    if (dateTime_) {
      dateTime_(&p->creationDate, &p->creationTime);
    }
    else {
      p->creationDate = FAT_DEFAULT_DATE;
      p->creationTime = FAT_DEFAULT_TIME;
    }
    p->lastAccessDate = p->creationDate;
    p->lastWriteDate = p->creationDate;
    p->lastWriteTime = p->creationTime;
    if (!dirFile->vol_->cacheFlush()) goto fail;
  }
  return openCachedEntry(index, oflag);
fail:
  return false;
}
bool SdBaseFile::open(SdBaseFile* dirFile, uint16_t index, uint8_t oflag) {
  dir_t* p;
  vol_ = dirFile->vol_;
  if (isOpen() || !dirFile) goto fail;
  if (oflag & O_EXCL) goto fail;
  if (!dirFile->seekSet(32 * index)) goto fail;
  p = dirFile->readDirCache();
  if (!p) goto fail;
  if (p->name[0] == DIR_NAME_FREE ||
      p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') {
    goto fail;
  }
  return openCachedEntry(index & 0XF, oflag);
fail:
  return false;
}
bool SdBaseFile::openCachedEntry(uint8_t dirIndex, uint8_t oflag) {
  dir_t* p = &vol_->cache()->dir[dirIndex];
  if (p->attributes & (DIR_ATT_READ_ONLY | DIR_ATT_DIRECTORY)) {
    if (oflag & (O_WRITE | O_TRUNC)) goto fail;
  }
  dirBlock_ = vol_->cacheBlockNumber();
  dirIndex_ = dirIndex;
  firstCluster_ = (uint32_t)p->firstClusterHigh << 16;
  firstCluster_ |= p->firstClusterLow;
  if (DIR_IS_FILE(p)) {
    fileSize_ = p->fileSize;
    type_ = FAT_FILE_TYPE_NORMAL;
  }
  else if (DIR_IS_SUBDIR(p)) {
    if (!vol_->chainSize(firstCluster_, &fileSize_)) goto fail;
    type_ = FAT_FILE_TYPE_SUBDIR;
  }
  else {
    goto fail;
  }
  flags_ = oflag & F_OFLAG;
  curCluster_ = 0;
  curPosition_ = 0;
  if ((oflag & O_TRUNC) && !truncate(0)) return false;
  return oflag & O_AT_END ? seekEnd(0) : true;
fail:
  type_ = FAT_FILE_TYPE_CLOSED;
  return false;
}
bool SdBaseFile::openNext(SdBaseFile* dirFile, uint8_t oflag) {
  dir_t* p;
  uint8_t index;
  if (!dirFile) goto fail;
  if (isOpen()) goto fail;
  vol_ = dirFile->vol_;
  while (1) {
    index = 0XF & (dirFile->curPosition_ >> 5);
    p = dirFile->readDirCache();
    if (!p) goto fail;
    if (p->name[0] == DIR_NAME_FREE) goto fail;
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') {
      continue;
    }
    if (DIR_IS_FILE_OR_SUBDIR(p)) {
      return openCachedEntry(index, oflag);
    }
  }
fail:
  return false;
}
bool SdBaseFile::openParent(SdBaseFile* dir) {
  dir_t entry;
  dir_t* p;
  SdBaseFile file;
  uint32_t c;
  uint32_t cluster;
  uint32_t lbn;
  if (isOpen() || !dir || dir->isRoot() || !dir->isDir()) goto fail;
  vol_ = dir->vol_;
  if (!dir->seekSet(32)) goto fail;
  if (dir->read(&entry, sizeof(entry)) != 32) goto fail;
  if (entry.name[0] != '.' || entry.name[1] != '.') goto fail;
  cluster = entry.firstClusterLow;
  cluster |= (uint32_t)entry.firstClusterHigh << 16;
  if (cluster == 0) return openRoot(vol_);
  lbn = vol_->clusterStartBlock(cluster);
  if (!vol_->cacheRawBlock(lbn, SdVolume::CACHE_FOR_READ)) {
    goto fail;
  }
  p = &vol_->cacheBuffer_.dir[1];
  if (p->name[0] != '.' || p->name[1] != '.') goto fail;
  if (p->firstClusterHigh == 0 && p->firstClusterLow == 0) {
    if (!file.openRoot(dir->volume())) goto fail;
  }
  else if (!file.openCachedEntry(1, O_READ)) {
    goto fail;
  }
  do {
    if (file.readDir(&entry, NULL) != 32) goto fail;
    c = entry.firstClusterLow;
    c |= (uint32_t)entry.firstClusterHigh << 16;
  } while (c != cluster);
  return open(&file, file.curPosition() / 32 - 1, O_READ);
fail:
  return false;
}
bool SdBaseFile::openRoot(SdVolume* vol) {
  if (isOpen()) goto fail;
  if (vol->fatType() == 16 || (FAT12_SUPPORT && vol->fatType() == 12)) {
    type_ = FAT_FILE_TYPE_ROOT_FIXED;
    firstCluster_ = 0;
    fileSize_ = 32 * vol->rootDirEntryCount();
  }
  else if (vol->fatType() == 32) {
    type_ = FAT_FILE_TYPE_ROOT32;
    firstCluster_ = vol->rootDirStart();
    if (!vol->chainSize(firstCluster_, &fileSize_)) goto fail;
  }
  else {
    return false;
  }
  vol_ = vol;
  flags_ = O_READ;
  curCluster_ = 0;
  curPosition_ = 0;
  dirBlock_ = 0;
  dirIndex_ = 0;
  return true;
fail:
  return false;
}
int SdBaseFile::peek() {
  filepos_t pos;
  getpos(&pos);
  int c = read();
  if (c >= 0) setpos(&pos);
  return c;
}
void SdBaseFile::printDirName(const dir_t& dir,
                              uint8_t width, bool printSlash) {
  uint8_t w = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ')continue;
    if (i == 8) {
      MYSERIAL.write('.');
      w++;
    }
    MYSERIAL.write(dir.name[i]);
    w++;
  }
  if (DIR_IS_SUBDIR(&dir) && printSlash) {
    MYSERIAL.write('/');
    w++;
  }
  while (w < width) {
    MYSERIAL.write(' ');
    w++;
  }
}
static void print2u(uint8_t v) {
  if (v < 10) MYSERIAL.write('0');
  MYSERIAL.print(v, DEC);
}
void SdBaseFile::printFatDate(uint16_t fatDate) {
  MYSERIAL.print(FAT_YEAR(fatDate));
  MYSERIAL.write('-');
  print2u(FAT_MONTH(fatDate));
  MYSERIAL.write('-');
  print2u(FAT_DAY(fatDate));
}
void SdBaseFile::printFatTime(uint16_t fatTime) {
  print2u(FAT_HOUR(fatTime));
  MYSERIAL.write(':');
  print2u(FAT_MINUTE(fatTime));
  MYSERIAL.write(':');
  print2u(FAT_SECOND(fatTime));
}
bool SdBaseFile::printName() {
  char name[FILENAME_LENGTH];
  if (!getFilename(name)) return false;
  MYSERIAL.print(name);
  return true;
}
int16_t SdBaseFile::read() {
  uint8_t b;
  return read(&b, 1) == 1 ? b : -1;
}
int16_t SdBaseFile::read(void* buf, uint16_t nbyte) {
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
  uint16_t offset;
  uint16_t toRead;
  uint32_t block;  
  if (!isOpen() || !(flags_ & O_READ)) goto fail;
  NOMORE(nbyte, fileSize_ - curPosition_);
  toRead = nbyte;
  while (toRead > 0) {
    offset = curPosition_ & 0X1FF;  
    if (type_ == FAT_FILE_TYPE_ROOT_FIXED) {
      block = vol_->rootDirStart() + (curPosition_ >> 9);
    }
    else {
      uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
      if (offset == 0 && blockOfCluster == 0) {
        if (curPosition_ == 0) {
          curCluster_ = firstCluster_;
        }
        else {
          if (!vol_->fatGet(curCluster_, &curCluster_)) goto fail;
        }
      }
      block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    }
    uint16_t n = toRead;
    NOMORE(n, 512 - offset);
    if (n == 512 && block != vol_->cacheBlockNumber()) {
      if (!vol_->readBlock(block, dst)) goto fail;
    }
    else {
      if (!vol_->cacheRawBlock(block, SdVolume::CACHE_FOR_READ)) goto fail;
      uint8_t* src = vol_->cache()->data + offset;
      memcpy(dst, src, n);
    }
    dst += n;
    curPosition_ += n;
    toRead -= n;
  }
  return nbyte;
fail:
  return -1;
}
int8_t SdBaseFile::readDir(dir_t* dir, char* longFilename) {
  int16_t n;
  if (!isDir() || (0X1F & curPosition_)) return -1;
  if (longFilename != NULL) longFilename[0] = '\0';
  while (1) {
    n = read(dir, sizeof(dir_t));
    if (n != sizeof(dir_t)) return n == 0 ? 0 : -1;
    if (dir->name[0] == DIR_NAME_FREE) return 0;
    if (dir->name[0] == DIR_NAME_DELETED || dir->name[0] == '.') continue;
    if (longFilename != NULL && DIR_IS_LONG_NAME(dir)) {
      vfat_t* VFAT = (vfat_t*)dir;
      if (VFAT->firstClusterLow == 0 && (VFAT->sequenceNumber & 0x1F) > 0 && (VFAT->sequenceNumber & 0x1F) <= MAX_VFAT_ENTRIES) {
        n = ((VFAT->sequenceNumber & 0x1F) - 1) * (FILENAME_LENGTH);
        for (uint8_t i = 0; i < FILENAME_LENGTH; i++)
          longFilename[n + i] = (i < 5) ? VFAT->name1[i] : (i < 11) ? VFAT->name2[i - 5] : VFAT->name3[i - 11];
        if (VFAT->sequenceNumber & 0x40) longFilename[n + FILENAME_LENGTH] = '\0';
      }
    }
    if (DIR_IS_FILE_OR_SUBDIR(dir)) return n;
  }
}
dir_t* SdBaseFile::readDirCache() {
  uint8_t i;
  if (!isDir()) goto fail;
  i = (curPosition_ >> 5) & 0XF;
  if (read() < 0) goto fail;
  curPosition_ += 31;
  return vol_->cache()->dir + i;
fail:
  return 0;
}
bool SdBaseFile::remove() {
  dir_t* d;
  if (!truncate(0)) goto fail;
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) goto fail;
  d->name[0] = DIR_NAME_DELETED;
  type_ = FAT_FILE_TYPE_CLOSED;
  return vol_->cacheFlush();
  return true;
fail:
  return false;
}
bool SdBaseFile::remove(SdBaseFile* dirFile, const char* path) {
  SdBaseFile file;
  if (!file.open(dirFile, path, O_WRITE)) goto fail;
  return file.remove();
fail:
  return false;
}
bool SdBaseFile::rename(SdBaseFile* dirFile, const char* newPath) {
  dir_t entry;
  uint32_t dirCluster = 0;
  SdBaseFile file;
  dir_t* d;
  if (!(isFile() || isSubDir())) goto fail;
  if (vol_ != dirFile->vol_) goto fail;
  sync();
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) goto fail;
  memcpy(&entry, d, sizeof(entry));
  d->name[0] = DIR_NAME_DELETED;
  if (isFile()) {
    if (!file.open(dirFile, newPath, O_CREAT | O_EXCL | O_WRITE)) {
      goto restore;
    }
  }
  else {
    if (!file.mkdir(dirFile, newPath, false)) {
      goto restore;
    }
    dirCluster = file.firstCluster_;
  }
  dirBlock_ = file.dirBlock_;
  dirIndex_ = file.dirIndex_;
  file.type_ = FAT_FILE_TYPE_CLOSED;
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) goto fail;
  memcpy(&d->attributes, &entry.attributes, sizeof(entry) - sizeof(d->name));
  if (dirCluster) {
    uint32_t block = vol_->clusterStartBlock(dirCluster);
    if (!vol_->cacheRawBlock(block, SdVolume::CACHE_FOR_READ)) goto fail;
    memcpy(&entry, &vol_->cache()->dir[1], sizeof(entry));
    if (!vol_->freeChain(dirCluster)) goto fail;
    block = vol_->clusterStartBlock(firstCluster_);
    if (!vol_->cacheRawBlock(block, SdVolume::CACHE_FOR_WRITE)) goto fail;
    memcpy(&vol_->cache()->dir[1], &entry, sizeof(entry));
  }
  return vol_->cacheFlush();
restore:
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) goto fail;
  d->name[0] = entry.name[0];
  vol_->cacheFlush();
fail:
  return false;
}
bool SdBaseFile::rmdir() {
  if (!isSubDir()) goto fail;
  rewind();
  while (curPosition_ < fileSize_) {
    dir_t* p = readDirCache();
    if (!p) goto fail;
    if (p->name[0] == DIR_NAME_FREE) break;
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;
    if (DIR_IS_FILE_OR_SUBDIR(p)) goto fail;
  }
  type_ = FAT_FILE_TYPE_NORMAL;
  flags_ |= O_WRITE;
  return remove();
fail:
  return false;
}
bool SdBaseFile::rmRfStar() {
  uint16_t index;
  SdBaseFile f;
  rewind();
  while (curPosition_ < fileSize_) {
    index = curPosition_ / 32;
    dir_t* p = readDirCache();
    if (!p) goto fail;
    if (p->name[0] == DIR_NAME_FREE) break;
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;
    if (!DIR_IS_FILE_OR_SUBDIR(p)) continue;
    if (!f.open(this, index, O_READ)) goto fail;
    if (f.isSubDir()) {
      if (!f.rmRfStar()) goto fail;
    }
    else {
      f.flags_ |= O_WRITE;
      if (!f.remove()) goto fail;
    }
    if (curPosition_ != (32 * (index + 1))) {
      if (!seekSet(32 * (index + 1))) goto fail;
    }
  }
  if (!isRoot()) {
    if (!rmdir()) goto fail;
  }
  return true;
fail:
  return false;
}
SdBaseFile::SdBaseFile(const char* path, uint8_t oflag) {
  type_ = FAT_FILE_TYPE_CLOSED;
  writeError = false;
  open(path, oflag);
}
bool SdBaseFile::seekSet(uint32_t pos) {
  uint32_t nCur;
  uint32_t nNew;
  if (!isOpen() || pos > fileSize_) goto fail;
  if (type_ == FAT_FILE_TYPE_ROOT_FIXED) {
    curPosition_ = pos;
    goto done;
  }
  if (pos == 0) {
    curCluster_ = 0;
    curPosition_ = 0;
    goto done;
  }
  nCur = (curPosition_ - 1) >> (vol_->clusterSizeShift_ + 9);
  nNew = (pos - 1) >> (vol_->clusterSizeShift_ + 9);
  if (nNew < nCur || curPosition_ == 0) {
    curCluster_ = firstCluster_;
  }
  else {
    nNew -= nCur;
  }
  while (nNew--) {
    if (!vol_->fatGet(curCluster_, &curCluster_)) goto fail;
  }
  curPosition_ = pos;
done:
  return true;
fail:
  return false;
}
void SdBaseFile::setpos(filepos_t* pos) {
  curPosition_ = pos->position;
  curCluster_ = pos->cluster;
}
bool SdBaseFile::sync() {
  if (!isOpen()) goto fail;
  if (flags_ & F_FILE_DIR_DIRTY) {
    dir_t* d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
    if (!d || d->name[0] == DIR_NAME_DELETED) goto fail;
    if (!isDir()) d->fileSize = fileSize_;
    d->firstClusterLow = firstCluster_ & 0XFFFF;
    d->firstClusterHigh = firstCluster_ >> 16;
    if (dateTime_) {
      dateTime_(&d->lastWriteDate, &d->lastWriteTime);
      d->lastAccessDate = d->lastWriteDate;
    }
    flags_ &= ~F_FILE_DIR_DIRTY;
  }
  return vol_->cacheFlush();
fail:
  writeError = true;
  return false;
}
bool SdBaseFile::timestamp(SdBaseFile* file) {
  dir_t* d;
  dir_t dir;
  if (!file->dirEntry(&dir)) goto fail;
  if (!sync()) goto fail;
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) goto fail;
  d->lastAccessDate = dir.lastAccessDate;
  d->creationDate = dir.creationDate;
  d->creationTime = dir.creationTime;
  d->creationTimeTenths = dir.creationTimeTenths;
  d->lastWriteDate = dir.lastWriteDate;
  d->lastWriteTime = dir.lastWriteTime;
  return vol_->cacheFlush();
fail:
  return false;
}
bool SdBaseFile::timestamp(uint8_t flags, uint16_t year, uint8_t month,
                           uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  uint16_t dirDate;
  uint16_t dirTime;
  dir_t* d;
  if (!isOpen()
      || year < 1980
      || year > 2107
      || month < 1
      || month > 12
      || day < 1
      || day > 31
      || hour > 23
      || minute > 59
      || second > 59) {
    goto fail;
  }
  if (!sync()) goto fail;
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) goto fail;
  dirDate = FAT_DATE(year, month, day);
  dirTime = FAT_TIME(hour, minute, second);
  if (flags & T_ACCESS) {
    d->lastAccessDate = dirDate;
  }
  if (flags & T_CREATE) {
    d->creationDate = dirDate;
    d->creationTime = dirTime;
    d->creationTimeTenths = second & 1 ? 100 : 0;
  }
  if (flags & T_WRITE) {
    d->lastWriteDate = dirDate;
    d->lastWriteTime = dirTime;
  }
  return vol_->cacheFlush();
fail:
  return false;
}
bool SdBaseFile::truncate(uint32_t length) {
  uint32_t newPos;
  if (!isFile() || !(flags_ & O_WRITE)) goto fail;
  if (length > fileSize_) goto fail;
  if (fileSize_ == 0) return true;
  newPos = curPosition_ > length ? length : curPosition_;
  if (!seekSet(length)) goto fail;
  if (length == 0) {
    if (!vol_->freeChain(firstCluster_)) goto fail;
    firstCluster_ = 0;
  }
  else {
    uint32_t toFree;
    if (!vol_->fatGet(curCluster_, &toFree)) goto fail;
    if (!vol_->isEOC(toFree)) {
      if (!vol_->freeChain(toFree)) goto fail;
      if (!vol_->fatPutEOC(curCluster_)) goto fail;
    }
  }
  fileSize_ = length;
  flags_ |= F_FILE_DIR_DIRTY;
  if (!sync()) goto fail;
  return seekSet(newPos);
fail:
  return false;
}
int16_t SdBaseFile::write(const void* buf, uint16_t nbyte) {
  const uint8_t* src = reinterpret_cast<const uint8_t*>(buf);
  uint16_t nToWrite = nbyte;
  if (!isFile() || !(flags_ & O_WRITE)) goto fail;
  if ((flags_ & O_APPEND) && curPosition_ != fileSize_) {
    if (!seekEnd()) goto fail;
  }
  while (nToWrite > 0) {
    uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
    uint16_t blockOffset = curPosition_ & 0X1FF;
    if (blockOfCluster == 0 && blockOffset == 0) {
      if (curCluster_ == 0) {
        if (firstCluster_ == 0) {
          if (!addCluster()) goto fail;
        }
        else {
          curCluster_ = firstCluster_;
        }
      }
      else {
        uint32_t next;
        if (!vol_->fatGet(curCluster_, &next)) goto fail;
        if (vol_->isEOC(next)) {
          if (!addCluster()) goto fail;
        }
        else {
          curCluster_ = next;
        }
      }
    }
    uint16_t n = 512 - blockOffset;
    NOMORE(n, nToWrite);
    uint32_t block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    if (n == 512) {
      if (vol_->cacheBlockNumber() == block) {
        vol_->cacheSetBlockNumber(0XFFFFFFFF, false);
      }
      if (!vol_->writeBlock(block, src)) goto fail;
    }
    else {
      if (blockOffset == 0 && curPosition_ >= fileSize_) {
        if (!vol_->cacheFlush()) goto fail;
        vol_->cacheSetBlockNumber(block, true);
      }
      else {
        if (!vol_->cacheRawBlock(block, SdVolume::CACHE_FOR_WRITE)) goto fail;
      }
      uint8_t* dst = vol_->cache()->data + blockOffset;
      memcpy(dst, src, n);
    }
    curPosition_ += n;
    src += n;
    nToWrite -= n;
  }
  if (curPosition_ > fileSize_) {
    fileSize_ = curPosition_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  else if (dateTime_ && nbyte) {
    flags_ |= F_FILE_DIR_DIRTY;
  }
  if (flags_ & O_SYNC) {
    if (!sync()) goto fail;
  }
  return nbyte;
fail:
  writeError = true;
  return -1;
}
#if ALLOW_DEPRECATED_FUNCTIONS && !defined(DOXYGEN)
  void (*SdBaseFile::oldDateTime_)(uint16_t &date, uint16_t &time) = 0;  
#endif  
#endif
