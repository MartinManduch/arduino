#include "Arduino.h"

byte mapInterrupt(byte PIN) { // mapping PIN to number of interrupt, valid for Arduino Leonardo
  switch (PIN) {
    case 0:
      return 2;
      break;
    case 1:
      return 3;
      break;
    case 2:
      return 1;
      break;
    case 3:
      return 0;
      break;
    case 7:
      return 4;
      break;
    default :
      return 255;
  }
}

unsigned long readULongFromBytes() {
  union u_tag {
    byte b[4];
    unsigned long ulval;
  } u;
  u.b[0] = Serial.read();
  u.b[1] = Serial.read();
  u.b[2] = Serial.read();
  u.b[3] = Serial.read();
  return u.ulval;
}
