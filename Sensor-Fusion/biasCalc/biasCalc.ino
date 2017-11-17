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

struct vector accelVector;
struct vector gyroVector;

struct vector trueUpG;

uint8_t timePrev = 0;
uint8_t timeCurr = 0;

float gAngle;

void computeBiasComp(int lowBit, int highBit, float* dest) {
  long sum = 0;
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
  *dest = (float)(-sum / i);
}

void getVectors(struct vector *accelVector, struct vector *gyroVector) {
  uint8_t temp;
  unsigned short val = 0;

  struct vector tempVector;

  short vals[3];
  
  for (int i = 0; i < 3; i++) {
    val = 0;
    readReg(ahx + 2*i, &temp, 1);
    val += ((unsigned short)(temp)) << 8;
    readReg(alx + 2*i, &temp, 1);
    val += (short)temp;
    vals[i] = val;
  }

  accelVector->x = (float)vals[0];
  accelVector->y = (float)vals[1];
  accelVector->z = (float)vals[2];

  vector_add(accelVector, &accelBias, accelVector);
  vector_normalize(accelVector, accelVector);
  
  for (int i = 0; i < 3; i++) {
    val = 0;
    readReg(ghx + 2*i, &temp, 1);
    val += ((unsigned short)(temp)) << 8;
    readReg(glx + 2*i, &temp, 1);
    val += (unsigned short)temp;
    vals[i] = (short)val;
  }

  gyroVector->x = (float)vals[0];
  gyroVector->y = (float)vals[1];
  gyroVector->z = (float)vals[2];

  vector_add(gyroVector, &gyroBias, gyroVector);

  timePrev = timeCurr;
  timeCurr = millis();
  uint8_t stepTime = timeCurr - timePrev;

  gAngle = sqrt(pow(gyroVector->x, 2) + pow(gyroVector->y, 2) + pow(gyroVector->z, 2)) * stepTime;

  vector_normalize(gyroVector, gyroVector);
}

void setup() {
  Serial.begin(115200);
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

  accelBias.z += 2048;

  trueUpG.x = 0;
  trueUpG.y = 0;
  trueUpG.z = 1;
}

void loop() {
  getVectors(&accelVector, &gyroVector);

  quaternion rotateQuat;
  vector tempUp;

  quaternion_create(&gyroVector, -gAngle, &rotateQuat);
  quaternion_rotate(&trueUpG, &rotateQuat, &tempUp);
  trueUpG.x = tempUp.x;
  trueUpG.y = tempUp.y;
  trueUpG.z = tempUp.z;

  Serial.print(accelVector.x);
  Serial.print(" ");
  Serial.print(accelVector.y);
  Serial.print(" ");
  Serial.print(accelVector.z);
  Serial.print(" ");
//    Serial.print("Gyroscope:");
  Serial.print(gyroVector.x);
  Serial.print(" ");
  Serial.print(gyroVector.y);
  Serial.print(" ");
  Serial.println(gyroVector.z);
}
