#include <Arduino.h>
#include <Wire.h>
#include "sensor_fusion.h"

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
  long sum = 0;
  uint8_t ready;
  uint8_t temp;
  unsigned short addMe = 0;
  
  int i = 0;
  for (;i < 75; i++) {
    addMe = 0;
    readReg(highBit, &temp, 1);
    addMe += ((unsigned short)(temp)) << 8;
    readReg(lowBit, &temp, 1);
    addMe += (unsigned short)temp;
    sum += (short)(addMe);
  }
  *dest = (float)(sum / i);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Opened serial");

  Wire.begin();
  Wire.setClock(100000);
  config();

  computeBiasComp(alx, ahx, &accelBias.x);
  Serial.print("AccelBias X: ");
  Serial.println(accelBias.x);

  computeBiasComp(aly, ahy, &accelBias.y);
  Serial.print("AccelBias Y: ");
  Serial.println(accelBias.y);

  computeBiasComp(alz, ahz, &accelBias.z);
  Serial.print("AccelBias Z: ");
  Serial.println(accelBias.z);

  computeBiasComp(glx, ghx, &gyroBias.x);
  Serial.print("GyroBias X: ");
  Serial.println(gyroBias.x);

  computeBiasComp(gly, ghy, &gyroBias.y);
  Serial.print("GyroBias Y: ");
  Serial.println(gyroBias.y);

  computeBiasComp(glz, ghz, &gyroBias.z);
  Serial.print("GyroBias Z: ");
  Serial.println(gyroBias.z);

}

void loop() {

}
