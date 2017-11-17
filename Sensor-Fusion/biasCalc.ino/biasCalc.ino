#include <Arduino.h>
#include <Wire.h>
#include <sensor_fusion.h>

#define ahx 59
#define alx 60
#define ahy 61
#define aly 62
#define ahz 63
#define alz 64

#define ghx 67
#define glx 68
#define ghy 69
#define gly 70
#define ghz 71
#define glz 72


struct vector accelBias;
struct vector gyroBias;

void computeBiasComp(int lowBit, int highBit, float* dest) {
  *dest = 0;
  short ready;
  short temp;

    for (int i = 0; i < 75; i++) {
    do
      {
        readRegister(58,&ready,1)
      }while(!(ready & 0x01))


    readRegister(highBit,&temp,1);
    *dest += temp << 4;
    readRegister(lowBit,&temp,1);
    *dest += *dest + temp;
  }
  *dest = *dest / 75;
}

void setup(){

  Wire.begin();
  config();

  uint8_t* pwrMgmt;
  readRegister(107, pwrMgmt, 1);
  Serial.print(*pwrMgmt, BIN);
  Serial.print("\n");

  uint8_t* set_full_scale;
  readRegister(0x1B, set_full_scale, 1);
  Serial.print(*set_full_scale, BIN);
  Serial.print("\n");

  uint8_t* con;
  readRegister(26, con, 1);
  Serial.print(*con, BIN);
  Serial.print("\n");
  
  computeBiasComp(alx, ahx, &accelBias.x);
  Serial.print(accelBias.x);
  Serial.print("\n");
  computeBiasComp(aly, ahy, &accelBias.y);
  Serial.print(accelBias.y);
  Serial.print("\n");
  computeBiasComp(alz, ahz, &accelBias.z);
  Serial.print(accelBias.z);
  Serial.print("\n");

  computeBiasComp(glx, ghx, &gyroBias.x);
  Serial.print(accelBias.x);
  Serial.print("\n");
  computeBiasComp(gly, ghy, &gyroBias.y);
  Serial.print(accelBias.y);
  Serial.print("\n");
  computeBiasComp(glz, ghz, &gyroBias.z);
  Serial.print(accelBias.z);
  Serial.print("\n");

}