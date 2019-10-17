#include "MarlinConfig.h"
#if ENABLED(DIGIPOT_I2C) && DISABLED(DIGIPOT_MCP4018)
#include "Stream.h"
#include "utility/twi.h"
#include "Wire.h"
#if MB(5DPRINT)
  #define DIGIPOT_I2C_FACTOR 117.96
  #define DIGIPOT_I2C_MAX_CURRENT 1.736
#else
  #define DIGIPOT_I2C_FACTOR 106.7
  #define DIGIPOT_I2C_MAX_CURRENT 2.5
#endif
static byte current_to_wiper(const float current) {
  return byte(CEIL(float((DIGIPOT_I2C_FACTOR * current))));
}
static void i2c_send(const byte addr, const byte a, const byte b) {
  Wire.beginTransmission(addr);
  Wire.write(a);
  Wire.write(b);
  Wire.endTransmission();
}
void digipot_i2c_set_current(uint8_t channel, float current) {
  current = min((float) max(current, 0.0f), DIGIPOT_I2C_MAX_CURRENT);
  byte addr = 0x2C; 
  if (channel >= 4) {
    addr = 0x2E; 
    channel -= 4;
  }
  i2c_send(addr, 0x40, 0xFF);
  i2c_send(addr, 0xA0, 0xFF);
  byte addresses[4] = { 0x00, 0x10, 0x60, 0x70 };
  i2c_send(addr, addresses[channel], current_to_wiper(current));
}
void digipot_i2c_init() {
  static const float digipot_motor_current[] PROGMEM = DIGIPOT_I2C_MOTOR_CURRENTS;
  Wire.begin();
  for (uint8_t i = 0; i < COUNT(digipot_motor_current); i++)
    digipot_i2c_set_current(i, pgm_read_float(&digipot_motor_current[i]));
}
#endif 
