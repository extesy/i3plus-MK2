#include "dac_mcp4728.h"
#include "enum.h"
#if ENABLED(DAC_STEPPER_CURRENT)
uint16_t mcp4728_values[XYZE];
void mcp4728_init() {
  Wire.begin();
  Wire.requestFrom(int(DAC_DEV_ADDRESS), 24);
  while (Wire.available()) {
    char deviceID = Wire.read(),
         hiByte = Wire.read(),
         loByte = Wire.read();
    if (!(deviceID & 0x08))
      mcp4728_values[(deviceID & 0x30) >> 4] = word((hiByte & 0x0F), loByte);
  }
}
uint8_t mcp4728_analogWrite(uint8_t channel, uint16_t value) {
  mcp4728_values[channel] = value;
  return mcp4728_fastWrite();
}
uint8_t mcp4728_eepromWrite() {
  Wire.beginTransmission(DAC_DEV_ADDRESS);
  Wire.write(SEQWRITE);
  LOOP_XYZE(i) {
    Wire.write(DAC_STEPPER_VREF << 7 | DAC_STEPPER_GAIN << 4 | highByte(mcp4728_values[i]));
    Wire.write(lowByte(mcp4728_values[i]));
  }
  return Wire.endTransmission();
}
uint8_t mcp4728_setVref_all(uint8_t value) {
  Wire.beginTransmission(DAC_DEV_ADDRESS);
  Wire.write(VREFWRITE | (value ? 0x0F : 0x00));
  return Wire.endTransmission();
}
uint8_t mcp4728_setGain_all(uint8_t value) {
  Wire.beginTransmission(DAC_DEV_ADDRESS);
  Wire.write(GAINWRITE | (value ? 0x0F : 0x00));
  return Wire.endTransmission();
}
uint16_t mcp4728_getValue(uint8_t channel) { return mcp4728_values[channel]; }
uint8_t mcp4728_getDrvPct(uint8_t channel) { return uint8_t(100.0 * mcp4728_values[channel] / (DAC_STEPPER_MAX) + 0.5); }
void mcp4728_setDrvPct(uint8_t pct[XYZE]) {
  LOOP_XYZE(i) mcp4728_values[i] = 0.01 * pct[i] * (DAC_STEPPER_MAX);
  mcp4728_fastWrite();
}
uint8_t mcp4728_fastWrite() {
  Wire.beginTransmission(DAC_DEV_ADDRESS);
  LOOP_XYZE(i) {
    Wire.write(highByte(mcp4728_values[i]));
    Wire.write(lowByte(mcp4728_values[i]));
  }
  return Wire.endTransmission();
}
uint8_t mcp4728_simpleCommand(byte simpleCommand) {
  Wire.beginTransmission(GENERALCALL);
  Wire.write(simpleCommand);
  return Wire.endTransmission();
}
#endif 
