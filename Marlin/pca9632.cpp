#include "MarlinConfig.h"
#if ENABLED(PCA9632)
#include "pca9632.h"
#define PCA9632_MODE1_VALUE   0b00000001 
#define PCA9632_MODE2_VALUE   0b00010101 
#define PCA9632_LEDOUT_VALUE  0b00101010
#define PCA9632_MODE1       0x00
#define PCA9632_MODE2       0x01
#define PCA9632_PWM0        0x02
#define PCA9632_PWM1        0x03
#define PCA9632_PWM2        0x04
#define PCA9632_PWM3        0x05
#define PCA9632_GRPPWM      0x06
#define PCA9632_GRPFREQ     0x07
#define PCA9632_LEDOUT      0X08
#define PCA9632_SUBADR1     0x09
#define PCA9632_SUBADR2     0x0A
#define PCA9632_SUBADR3     0x0B
#define PCA9632_ALLCALLADDR 0x0C
#define PCA9632_NO_AUTOINC  0x00
#define PCA9632_AUTO_ALL    0x80
#define PCA9632_AUTO_IND    0xA0
#define PCA9632_AUTOGLO     0xC0
#define PCA9632_AUTOGI      0xE0
#define PCA9632_RED     0x00
#define PCA9632_GRN     0x02
#define PCA9632_BLU     0x04
#define LED_OFF   0x00
#define LED_ON    0x01
#define LED_PWM   0x02
#define PCA9632_ADDRESS 0b01100000
byte PCA_init = 0;
static void PCA9632_WriteRegister(const byte addr, const byte regadd, const byte value) {
  Wire.beginTransmission(addr);
  Wire.write(regadd);
  Wire.write(value);
  Wire.endTransmission();
}
static void PCA9632_WriteAllRegisters(const byte addr, const byte regadd, const byte value1, const byte value2, const byte value3) {
  Wire.beginTransmission(addr);
  Wire.write(PCA9632_AUTO_IND | regadd);
  Wire.write(value1);
  Wire.write(value2);
  Wire.write(value3);
  Wire.endTransmission();
}
static byte PCA9632_ReadRegister(const byte addr, const byte regadd) {
  Wire.beginTransmission(addr);
  Wire.write(regadd);
  const byte value = Wire.read();
  Wire.endTransmission();
  return value;
}
void PCA9632_SetColor(const byte r, const byte g, const byte b) {
  if (!PCA_init) {
    PCA_init = 1;
    Wire.begin();
    PCA9632_WriteRegister(PCA9632_ADDRESS,PCA9632_MODE1, PCA9632_MODE1_VALUE);
    PCA9632_WriteRegister(PCA9632_ADDRESS,PCA9632_MODE2, PCA9632_MODE2_VALUE);
  }
  const byte LEDOUT = (r ? LED_PWM << PCA9632_RED : 0)
                    | (g ? LED_PWM << PCA9632_GRN : 0)
                    | (b ? LED_PWM << PCA9632_BLU : 0);
  PCA9632_WriteAllRegisters(PCA9632_ADDRESS,PCA9632_PWM0, r, g, b);
  PCA9632_WriteRegister(PCA9632_ADDRESS,PCA9632_LEDOUT, LEDOUT);
}
#endif 
