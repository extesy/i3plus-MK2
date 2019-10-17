#include "cardreader.h"
#include "ultralcd.h"
#include "stepper.h"
#include "language.h"
#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#define LONGEST_FILENAME (longFilename[0] ? longFilename : filename)
CardReader::CardReader() {
  #if ENABLED(SDCARD_SORT_ALPHA)
    sort_count = 0;
    #if ENABLED(SDSORT_GCODE)
      sort_alpha = true;
      sort_folders = FOLDER_SORTING;
    #endif
  #endif
  sdprinting = cardOK = saving = logging = false;
  filesize = 0;
  sdpos = 0;
  workDirDepth = 0;
  file_subcall_ctr = 0;
  ZERO(workDirParents);
  autostart_stilltocheck = true; 
  autostart_index = 0;
  #if SDPOWER > -1
    OUT_WRITE(SDPOWER, HIGH);
  #endif 
  next_autostart_ms = millis() + 5000;
}
char *createFilename(char *buffer, const dir_t &p) { 
  char *pos = buffer;
  for (uint8_t i = 0; i < 11; i++) {
    if (p.name[i] == ' ') continue;
    if (i == 8) *pos++ = '.';
    *pos++ = p.name[i];
  }
  *pos++ = 0;
  return buffer;
}
void CardReader::lsDive(const char *prepend, SdFile parent, const char * const match) {
  dir_t p;
  uint8_t cnt = 0;
  while (parent.readDir(p, longFilename) > 0) {
    if (DIR_IS_SUBDIR(&p) && lsAction != LS_Count && lsAction != LS_GetFilename) {
      char lfilename[FILENAME_LENGTH];
      createFilename(lfilename, p);
      bool prepend_is_empty = (prepend[0] == '\0');
      int len = (prepend_is_empty ? 1 : strlen(prepend)) + strlen(lfilename) + 1 + 1;
      char path[len];
      strcpy(path, prepend_is_empty ? "/" : prepend); 
      strcat(path, lfilename); 
      strcat(path, "/");       
      SdFile dir;
      if (!dir.open(parent, lfilename, O_READ)) {
        if (lsAction == LS_SerialPrint) {
          SERIAL_ECHO_START();
          SERIAL_ECHOPGM(MSG_SD_CANT_OPEN_SUBDIR);
          SERIAL_ECHOLN(lfilename);
        }
      }
      lsDive(path, dir);
    }
    else {
      uint8_t pn0 = p.name[0];
      if (pn0 == DIR_NAME_FREE) break;
      if (pn0 == DIR_NAME_DELETED || pn0 == '.') continue;
      if (longFilename[0] == '.') continue;
      if (!DIR_IS_FILE_OR_SUBDIR(&p) || (p.attributes & DIR_ATT_HIDDEN)) continue;
      filenameIsDir = DIR_IS_SUBDIR(&p);
      if (!filenameIsDir && (p.name[8] != 'G' || p.name[9] == '~')) continue;
      switch (lsAction) {
        case LS_Count:
          nrFiles++;
          break;
        case LS_SerialPrint:
          createFilename(filename, p);
          SERIAL_PROTOCOL(prepend);
          SERIAL_PROTOCOL(filename);
          SERIAL_PROTOCOLCHAR(' ');
          SERIAL_PROTOCOLLN(p.fileSize);
          break;
        case LS_GetFilename:
          createFilename(filename, p);
          if (match != NULL) {
            if (strcasecmp(match, filename) == 0) return;
          }
          else if (cnt == nrFiles) return;
          cnt++;
          break;
      }
    }
  } 
}
void CardReader::ls() {
  lsAction = LS_SerialPrint;
  root.rewind();
  lsDive("", root);
}
#if ENABLED(LONG_FILENAME_HOST_SUPPORT)
  void CardReader::printLongPath(char *path) {
    lsAction = LS_GetFilename;
    int i, pathLen = strlen(path);
    for (i = 0; i < pathLen; i++) if (path[i] == '/') path[i] = '\0';
    SdFile diveDir = root; 
    for (i = 0; i < pathLen;) {
      if (path[i] == '\0') i++; 
      char *segment = &path[i]; 
      if (!*segment) break;
      while (path[++i]) { }
      diveDir.rewind();
      lsDive("", diveDir, segment);
      SERIAL_PROTOCOLCHAR('/');
      SERIAL_PROTOCOL(longFilename[0] ? longFilename : "???");
      if (!filenameIsDir) break;
      SdFile dir;
      if (!dir.open(diveDir, segment, O_READ)) {
        SERIAL_EOL();
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM(MSG_SD_CANT_OPEN_SUBDIR);
        SERIAL_ECHO(segment);
        break;
      }
      diveDir.close();
      diveDir = dir;
    } 
    SERIAL_EOL();
  }
#endif 
void CardReader::initsd() {
  cardOK = false;
  if (root.isOpen()) root.close();
  #ifndef SPI_SPEED
    #define SPI_SPEED SPI_FULL_SPEED
  #endif
  if (!card.init(SPI_SPEED, SDSS)
    #if defined(LCD_SDSS) && (LCD_SDSS != SDSS)
      && !card.init(SPI_SPEED, LCD_SDSS)
    #endif
  ) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
  }
  else if (!volume.init(&card)) {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_SD_VOL_INIT_FAIL);
  }
  else if (!root.openRoot(&volume)) {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
  }
  else {
    cardOK = true;
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
  }
  workDir = root;
  curDir = &root;
  #if ENABLED(SDCARD_SORT_ALPHA)
    presort();
  #endif
}
void CardReader::setroot() {
  workDir = root;
  curDir = &workDir;
  #if ENABLED(SDCARD_SORT_ALPHA)
    presort();
  #endif
}
void CardReader::release() {
  sdprinting = false;
  cardOK = false;
}
void CardReader::openAndPrintFile(const char *name) {
  char cmd[4 + strlen(name) + 1]; 
  sprintf_P(cmd, PSTR("M23 %s"), name);
  for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
  enqueue_and_echo_command(cmd);
  enqueue_and_echo_commands_P(PSTR("M24"));
}
void CardReader::startFileprint() {
  if (cardOK) {
    sdprinting = true;
    #if ENABLED(SDCARD_SORT_ALPHA)
      flush_presort();
    #endif
  }
}
void CardReader::stopSDPrint() {
  sdprinting = false;
  if (isFileOpen()) file.close();
}
void CardReader::openLogFile(char* name) {
  logging = true;
  openFile(name, false);
}
void CardReader::getAbsFilename(char *t) {
  uint8_t cnt = 0;
  *t = '/'; t++; cnt++;
  for (uint8_t i = 0; i < workDirDepth; i++) {
    workDirParents[i].getFilename(t); 
    while (*t && cnt < MAXPATHNAMELENGTH) { t++; cnt++; } 
  }
  if (cnt < MAXPATHNAMELENGTH - (FILENAME_LENGTH))
    file.getFilename(t);
  else
    t[0] = 0;
}
void CardReader::openFile(char* name, bool read, bool push_current) {
  if (!cardOK) return;
  uint8_t doing = 0;
  if (isFileOpen()) { 
    if (push_current) {
      if (file_subcall_ctr > SD_PROCEDURE_DEPTH - 1) {
        SERIAL_ERROR_START();
        SERIAL_ERRORPGM("trying to call sub-gcode files with too many levels. MAX level is:");
        FunV006("Trying to call sub-gcode files with too many levels.");
        SERIAL_ERRORLN(SD_PROCEDURE_DEPTH);
        kill(PSTR(MSG_KILLED));
        return;
      }
      getAbsFilename(proc_filenames[file_subcall_ctr]);
      SERIAL_ECHO_START();
      SERIAL_ECHOPAIR("SUBROUTINE CALL target:\"", name);
      SERIAL_ECHOPAIR("\" parent:\"", proc_filenames[file_subcall_ctr]);
      SERIAL_ECHOLNPAIR("\" pos", sdpos);
      filespos[file_subcall_ctr] = sdpos;
      file_subcall_ctr++;
    }
    else {
      doing = 1;
    }
  }
  else { 
    doing = 2;
    file_subcall_ctr = 0; 
  }
  if (doing) {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM("Now ");
    SERIAL_ECHO(doing == 1 ? "doing" : "fresh");
    SERIAL_ECHOLNPAIR(" file: ", name);
  }
  stopSDPrint();
  SdFile myDir;
  curDir = &root;
  char *fname = name;
  char *dirname_start, *dirname_end;
  if (name[0] == '/') {
    dirname_start = &name[1];
    while (dirname_start != NULL) {
      dirname_end = strchr(dirname_start, '/');
      if (dirname_end != NULL && dirname_end > dirname_start) {
        char subdirname[FILENAME_LENGTH];
        strncpy(subdirname, dirname_start, dirname_end - dirname_start);
        subdirname[dirname_end - dirname_start] = 0;
        SERIAL_ECHOLN(subdirname);
        if (!myDir.open(curDir, subdirname, O_READ)) {
          SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
          SERIAL_PROTOCOL(subdirname);
          SERIAL_PROTOCOLCHAR('.');
          return;
        }
        else {
        }
        curDir = &myDir;
        dirname_start = dirname_end + 1;
      }
      else { 
        fname = dirname_start;
        break;
      }
    }
  }
  else { 
    curDir = &workDir;
  }
  if (read) {
    if (file.open(curDir, fname, O_READ)) {
      filesize = file.fileSize();
      SERIAL_PROTOCOLPAIR(MSG_SD_FILE_OPENED, fname);
      SERIAL_PROTOCOLLNPAIR(MSG_SD_SIZE, filesize);
      sdpos = 0;
      SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
      getfilename(0, fname);
      lcd_setstatus(longFilename[0] ? longFilename : fname);
    }
    else {
      SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, fname);
      SERIAL_PROTOCOLCHAR('.');
      SERIAL_EOL();
    }
    #ifdef FYS_START_UP_100_FEEDRATE
    feedrate_percentage = 100;
    #endif
  }
  else { 
    if (!file.open(curDir, fname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC)) {
      SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, fname);
      SERIAL_PROTOCOLCHAR('.');
      SERIAL_EOL();
    }
    else {
      saving = true;
      SERIAL_PROTOCOLLNPAIR(MSG_SD_WRITE_TO_FILE, name);
      lcd_setstatus(fname);
    }
  }
}
void CardReader::removeFile(char* name) {
  if (!cardOK) return;
  stopSDPrint();
  SdFile myDir;
  curDir = &root;
  char *fname = name;
  char *dirname_start, *dirname_end;
  if (name[0] == '/') {
    dirname_start = strchr(name, '/') + 1;
    while (dirname_start != NULL) {
      dirname_end = strchr(dirname_start, '/');
      if (dirname_end != NULL && dirname_end > dirname_start) {
        char subdirname[FILENAME_LENGTH];
        strncpy(subdirname, dirname_start, dirname_end - dirname_start);
        subdirname[dirname_end - dirname_start] = 0;
        SERIAL_ECHOLN(subdirname);
        if (!myDir.open(curDir, subdirname, O_READ)) {
          SERIAL_PROTOCOLPAIR("open failed, File: ", subdirname);
          SERIAL_PROTOCOLCHAR('.');
          SERIAL_EOL();
          return;
        }
        else {
        }
        curDir = &myDir;
        dirname_start = dirname_end + 1;
      }
      else { 
        fname = dirname_start;
        break;
      }
    }
  }
  else { 
    curDir = &workDir;
  }
  if (file.remove(curDir, fname)) {
    SERIAL_PROTOCOLPGM("File deleted:");
    SERIAL_PROTOCOLLN(fname);
    sdpos = 0;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
  else {
    SERIAL_PROTOCOLPGM("Deletion failed, File: ");
    SERIAL_PROTOCOL(fname);
    SERIAL_PROTOCOLCHAR('.');
  }
}
void CardReader::getStatus() {
  if (cardOK) {
    SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
    SERIAL_PROTOCOL(sdpos);
    SERIAL_PROTOCOLCHAR('/');
    SERIAL_PROTOCOLLN(filesize);
  }
  else {
    SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
  }
}
void CardReader::write_command(char *buf) {
  char* begin = buf;
  char* npos = 0;
  char* end = buf + strlen(buf) - 1;
  file.writeError = false;
  if ((npos = strchr(buf, 'N')) != NULL) {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
  file.write(begin);
  if (file.writeError) {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
  }
}
#ifdef FYS_ULTILCD2_COMPATIBLE
bool CardReader::write_string(char* buffer)
{
    file.write(buffer);
    return file.writeError;
}
#endif
void CardReader::checkautostart(bool force) {
  if (!force && (!autostart_stilltocheck || ELAPSED(millis(), next_autostart_ms)))
    return;
  autostart_stilltocheck = false;
  if (!cardOK) {
    initsd();
    if (!cardOK) return; 
  }
  char autoname[10];
  sprintf_P(autoname, PSTR("auto%i.g"), autostart_index);
  for (int8_t i = 0; i < (int8_t)strlen(autoname); i++) autoname[i] = tolower(autoname[i]);
  dir_t p;
  root.rewind();
  bool found = false;
  while (root.readDir(p, NULL) > 0) {
    for (int8_t i = (int8_t)strlen((char*)p.name); i--;) p.name[i] = tolower(p.name[i]);
    if (p.name[9] != '~' && strncmp((char*)p.name, autoname, 5) == 0) {
      openAndPrintFile(autoname);
      found = true;
    }
  }
  if (!found)
    autostart_index = -1;
  else
    autostart_index++;
}
void CardReader::closefile(bool store_location) {
  file.sync();
  file.close();
  saving = logging = false;
  if (store_location) {
  }
}
void CardReader::getfilename(uint16_t nr, const char * const match) {
  #if ENABLED(SDSORT_CACHE_NAMES)
    if (match != NULL) {
      while (nr < sort_count) {
        if (strcasecmp(match, sortshort[nr]) == 0) break;
        nr++;
      }
    }
    if (nr < sort_count) {
      strcpy(filename, sortshort[nr]);
      strcpy(longFilename, sortnames[nr]);
      filenameIsDir = TEST(isDir[nr>>3], nr & 0x07);
      return;
    }
  #endif 
  curDir = &workDir;
  lsAction = LS_GetFilename;
  nrFiles = nr;
  curDir->rewind();
  lsDive("", *curDir, match);
}
uint16_t CardReader::getnrfilenames() {
  curDir = &workDir;
  lsAction = LS_Count;
  nrFiles = 0;
  curDir->rewind();
  lsDive("", *curDir);
  return nrFiles;
}
void CardReader::chdir(const char * relpath) {
  SdFile newfile;
  SdFile *parent = &root;
  if (workDir.isOpen()) parent = &workDir;
  if (!newfile.open(*parent, relpath, O_READ)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
    SERIAL_ECHOLN(relpath);
  }
  else {
    if (workDirDepth < MAX_DIR_DEPTH)
      workDirParents[workDirDepth++] = *parent;
    workDir = newfile;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
}
void CardReader::updir() {
  if (workDirDepth > 0) {
    workDir = workDirParents[--workDirDepth];
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
}
#if ENABLED(SDCARD_SORT_ALPHA)
  void CardReader::getfilename_sorted(const uint16_t nr) {
    getfilename(
      #if ENABLED(SDSORT_GCODE)
        sort_alpha &&
      #endif
      (nr < sort_count) ? sort_order[nr] : nr
    );
  }
  void CardReader::presort() {
    #if ENABLED(SDSORT_GCODE)
      if (!sort_alpha) return;
    #endif
    flush_presort();
    uint16_t fileCnt = getnrfilenames();
    if (fileCnt > 0) {
      if (fileCnt > SDSORT_LIMIT) fileCnt = SDSORT_LIMIT;
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        sort_order = new uint8_t[fileCnt];
      #endif
      #if ENABLED(SDSORT_USES_RAM)
        #if ENABLED(SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            sortshort = new char*[fileCnt];
            sortnames = new char*[fileCnt];
          #endif
        #elif ENABLED(SDSORT_USES_STACK)
          char sortnames[fileCnt][LONG_FILENAME_LENGTH];
        #endif
        #if HAS_FOLDER_SORTING
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            isDir = new uint8_t[(fileCnt + 7) >> 3];
          #elif ENABLED(SDSORT_USES_STACK)
            uint8_t isDir[(fileCnt + 7) >> 3];
          #endif
        #endif
      #else 
        char name1[LONG_FILENAME_LENGTH + 1];
      #endif
      if (fileCnt > 1) {
        for (uint16_t i = 0; i < fileCnt; i++) {
          sort_order[i] = i;
          #if ENABLED(SDSORT_USES_RAM)
            getfilename(i);
            #if ENABLED(SDSORT_DYNAMIC_RAM)
              sortnames[i] = strdup(LONGEST_FILENAME);
              #if ENABLED(SDSORT_CACHE_NAMES)
                sortshort[i] = strdup(filename);
              #endif
            #else
              strcpy(sortnames[i], LONGEST_FILENAME);
              #if ENABLED(SDSORT_CACHE_NAMES)
                strcpy(sortshort[i], filename);
              #endif
            #endif
            #if HAS_FOLDER_SORTING
              const uint16_t bit = i & 0x07, ind = i >> 3;
              if (bit == 0) isDir[ind] = 0x00;
              if (filenameIsDir) isDir[ind] |= _BV(bit);
            #endif
          #endif
        }
        for (uint16_t i = fileCnt; --i;) {
          bool didSwap = false;
          for (uint16_t j = 0; j < i; ++j) {
            const uint16_t o1 = sort_order[j], o2 = sort_order[j + 1];
            #if ENABLED(SDSORT_USES_RAM)
              #define _SORT_CMP_NODIR() (strcasecmp(sortnames[o1], sortnames[o2]) > 0)
            #else
              #define _SORT_CMP_NODIR() (strcasecmp(name1, name2) > 0)
            #endif
            #if HAS_FOLDER_SORTING
              #if ENABLED(SDSORT_USES_RAM)
                const uint8_t ind1 = o1 >> 3, bit1 = o1 & 0x07,
                              ind2 = o2 >> 3, bit2 = o2 & 0x07;
                #define _SORT_CMP_DIR(fs) \
                  (((isDir[ind1] & _BV(bit1)) != 0) == ((isDir[ind2] & _BV(bit2)) != 0) \
                    ? _SORT_CMP_NODIR() \
                    : (isDir[fs > 0 ? ind1 : ind2] & (fs > 0 ? _BV(bit1) : _BV(bit2))) != 0)
              #else
                #define _SORT_CMP_DIR(fs) ((dir1 == filenameIsDir) ? _SORT_CMP_NODIR() : (fs > 0 ? dir1 : !dir1))
              #endif
            #endif
            #if DISABLED(SDSORT_USES_RAM)
              getfilename(o1);
              strcpy(name1, LONGEST_FILENAME); 
              #if HAS_FOLDER_SORTING
                bool dir1 = filenameIsDir;
              #endif
              getfilename(o2);
              char *name2 = LONGEST_FILENAME; 
            #endif 
            if (
              #if HAS_FOLDER_SORTING
                #if ENABLED(SDSORT_GCODE)
                  sort_folders ? _SORT_CMP_DIR(sort_folders) : _SORT_CMP_NODIR()
                #else
                  _SORT_CMP_DIR(FOLDER_SORTING)
                #endif
              #else
                _SORT_CMP_NODIR()
              #endif
            ) {
              sort_order[j] = o2;
              sort_order[j + 1] = o1;
              didSwap = true;
            }
          }
          if (!didSwap) break;
        }
        #if ENABLED(SDSORT_USES_RAM) && DISABLED(SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            for (uint16_t i = 0; i < fileCnt; ++i) free(sortnames[i]);
            #if HAS_FOLDER_SORTING
              free(isDir);
            #endif
          #endif
        #endif
      }
      else {
        sort_order[0] = 0;
        #if ENABLED(SDSORT_USES_RAM) && ENABLED(SDSORT_CACHE_NAMES)
          getfilename(0);
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            sortnames = new char*[1];
            sortnames[0] = strdup(LONGEST_FILENAME); 
            sortshort = new char*[1];
            sortshort[0] = strdup(filename);         
            isDir = new uint8_t[1];
          #else
            strcpy(sortnames[0], LONGEST_FILENAME);
            strcpy(sortshort[0], filename);
          #endif
          isDir[0] = filenameIsDir ? 0x01 : 0x00;
        #endif
      }
      sort_count = fileCnt;
    }
  }
  void CardReader::flush_presort() {
    if (sort_count > 0) {
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        delete sort_order;
        #if ENABLED(SDSORT_CACHE_NAMES)
          for (uint8_t i = 0; i < sort_count; ++i) {
            free(sortshort[i]); 
            free(sortnames[i]); 
          }
          delete sortshort;
          delete sortnames;
        #endif
      #endif
      sort_count = 0;
    }
  }
#endif 
void CardReader::printingHasFinished() {
  stepper.synchronize();
  file.close();
  if (file_subcall_ctr > 0) { 
    file_subcall_ctr--;
    openFile(proc_filenames[file_subcall_ctr], true, true);
    setIndex(filespos[file_subcall_ctr]);
    startFileprint();
  }
  else {
    sdprinting = false;
    if (SD_FINISHED_STEPPERRELEASE)
      enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    print_job_timer.stop();
    if (print_job_timer.duration() > 60)
      enqueue_and_echo_commands_P(PSTR("M31"));
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
}
#endif 
