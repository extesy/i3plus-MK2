#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#ifndef Sd2Card_h
#define Sd2Card_h
#include "SdFatConfig.h"
#include "SdInfo.h"
uint8_t const SPI_FULL_SPEED = 0;
uint8_t const SPI_HALF_SPEED = 1;
uint8_t const SPI_QUARTER_SPEED = 2;
uint8_t const SPI_EIGHTH_SPEED = 3;
uint8_t const SPI_SIXTEENTH_SPEED = 4;
uint16_t const SD_INIT_TIMEOUT = 2000;
uint16_t const SD_ERASE_TIMEOUT = 10000;
uint16_t const SD_READ_TIMEOUT = 300;
uint16_t const SD_WRITE_TIMEOUT = 600;
uint8_t const SD_CARD_ERROR_CMD0 = 0X1;
uint8_t const SD_CARD_ERROR_CMD8 = 0X2;
uint8_t const SD_CARD_ERROR_CMD12 = 0X3;
uint8_t const SD_CARD_ERROR_CMD17 = 0X4;
uint8_t const SD_CARD_ERROR_CMD18 = 0X5;
uint8_t const SD_CARD_ERROR_CMD24 = 0X6;
uint8_t const SD_CARD_ERROR_CMD25 = 0X7;
uint8_t const SD_CARD_ERROR_CMD58 = 0X8;
uint8_t const SD_CARD_ERROR_ACMD23 = 0X9;
uint8_t const SD_CARD_ERROR_ACMD41 = 0XA;
uint8_t const SD_CARD_ERROR_BAD_CSD = 0XB;
uint8_t const SD_CARD_ERROR_ERASE = 0XC;
uint8_t const SD_CARD_ERROR_ERASE_SINGLE_BLOCK = 0XD;
uint8_t const SD_CARD_ERROR_ERASE_TIMEOUT = 0XE;
uint8_t const SD_CARD_ERROR_READ = 0XF;
uint8_t const SD_CARD_ERROR_READ_REG = 0X10;
uint8_t const SD_CARD_ERROR_READ_TIMEOUT = 0X11;
uint8_t const SD_CARD_ERROR_STOP_TRAN = 0X12;
uint8_t const SD_CARD_ERROR_WRITE = 0X13;
uint8_t const SD_CARD_ERROR_WRITE_BLOCK_ZERO = 0X14;  
uint8_t const SD_CARD_ERROR_WRITE_MULTIPLE = 0X15;
uint8_t const SD_CARD_ERROR_WRITE_PROGRAMMING = 0X16;
uint8_t const SD_CARD_ERROR_WRITE_TIMEOUT = 0X17;
uint8_t const SD_CARD_ERROR_SCK_RATE = 0X18;
uint8_t const SD_CARD_ERROR_INIT_NOT_CALLED = 0X19;
uint8_t const SD_CARD_ERROR_CRC = 0X20;
uint8_t const SD_CARD_TYPE_SD1  = 1;
uint8_t const SD_CARD_TYPE_SD2  = 2;
uint8_t const SD_CARD_TYPE_SDHC = 3;
#if MEGA_SOFT_SPI
  #define SOFTWARE_SPI
#elif USE_SOFTWARE_SPI
  #define SOFTWARE_SPI
#endif  
#if DISABLED(SOFTWARE_SPI)
  #define SD_CHIP_SELECT_PIN SS_PIN
  #define SPI_MOSI_PIN MOSI_PIN
  #define SPI_MISO_PIN MISO_PIN
  #define SPI_SCK_PIN SCK_PIN
#else  
  #define SD_CHIP_SELECT_PIN SOFT_SPI_CS_PIN
  #define SPI_MOSI_PIN SOFT_SPI_MOSI_PIN
  #define SPI_MISO_PIN SOFT_SPI_MISO_PIN
  #define SPI_SCK_PIN SOFT_SPI_SCK_PIN
#endif  
class Sd2Card {
 public:
  Sd2Card() : errorCode_(SD_CARD_ERROR_INIT_NOT_CALLED), type_(0) {}
  uint32_t cardSize();
  bool erase(uint32_t firstBlock, uint32_t lastBlock);
  bool eraseSingleBlockEnable();
  void error(uint8_t code) {errorCode_ = code;}
  int errorCode() const {return errorCode_;}
  int errorData() const {return status_;}
  bool init(uint8_t sckRateID = SPI_FULL_SPEED,
            uint8_t chipSelectPin = SD_CHIP_SELECT_PIN);
  bool readBlock(uint32_t block, uint8_t* dst);
  bool readCID(cid_t* cid) {
    return readRegister(CMD10, cid);
  }
  bool readCSD(csd_t* csd) {
    return readRegister(CMD9, csd);
  }
  bool readData(uint8_t* dst);
  bool readStart(uint32_t blockNumber);
  bool readStop();
  bool setSckRate(uint8_t sckRateID);
  int type() const {return type_;}
  bool writeBlock(uint32_t blockNumber, const uint8_t* src);
  bool writeData(const uint8_t* src);
  bool writeStart(uint32_t blockNumber, uint32_t eraseCount);
  bool writeStop();
 private:
  uint8_t chipSelectPin_;
  uint8_t errorCode_;
  uint8_t spiRate_;
  uint8_t status_;
  uint8_t type_;
  uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
    cardCommand(CMD55, 0);
    return cardCommand(cmd, arg);
  }
  uint8_t cardCommand(uint8_t cmd, uint32_t arg);
  bool readData(uint8_t* dst, uint16_t count);
  bool readRegister(uint8_t cmd, void* buf);
  void chipSelectHigh();
  void chipSelectLow();
  void type(uint8_t value) {type_ = value;}
  bool waitNotBusy(uint16_t timeoutMillis);
  bool writeData(uint8_t token, const uint8_t* src);
};
#endif  
#endif
