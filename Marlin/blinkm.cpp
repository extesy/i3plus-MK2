#include "Marlin.h"
#if ENABLED(BLINKM)
#include "blinkm.h"
void SendColors(byte red, byte grn, byte blu) {
  Wire.begin();
  Wire.beginTransmission(0x09);
  Wire.write('o');                    
  Wire.write('n');
  Wire.write(red);
  Wire.write(grn);
  Wire.write(blu);
  Wire.endTransmission();
}
#endif 
