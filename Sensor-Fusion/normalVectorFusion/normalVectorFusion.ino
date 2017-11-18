#include <Arduino.h>
#include <Wire.h>
#include "sensor_fusion.h"

//registers for high and low bytes of
//accelerometer and gyroscope reading
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

//normalized vectors pointing towards relative true-up
//use normalized readings from accelerometer, no need for
//separate vector
struct vector trueUpGyro;
struct vector trueUpFusion;

unsigned long timePrev = 0;
unsigned long timeCurr = 0;

//constant to control weight of accelerometer
//and gyroscope contributing to fusionNormal
float alpha = 0.25;


void computeBiasComp(int lowBit, int highBit, float* dest) {
  long sum = 0;
  uint8_t readBuf;
  unsigned short addMe;
  
  int i = 0;
  for (;i < 75; i++) {
    readReg(highBit, &readBuf, 1);
    addMe = ((unsigned short)readBuf) << 8;
    readReg(lowBit, &readBuf, 1);
    addMe += (unsigned short)readBuf;
    sum += (short)(addMe);
  }
  *dest = (float)(-sum / i);
}

//returns normalized vector of acclerometer readings and a vector
//of gyroscope readings with unchanged magnitude
void getVectors(struct vector *accelVector, struct vector *gyroVector) {
  uint8_t readBuf;
  
  //2byte 2-s compliment integers stored in registers
  short vals[3];
  unsigned short val;
  
  for (int i = 0; i < 3; i++) {
    readReg(ahx + 2*i, &readBuf, 1);
    val = ((unsigned short)readBuf) << 8;
    readReg(alx + 2*i, &readBuf, 1);
    val += (short)readBuf;
    vals[i] = val;
  }

  accelVector->x = (float)vals[0];
  accelVector->y = (float)vals[1];
  accelVector->z = (float)vals[2];

  vector_add(accelVector, &accelBias, accelVector);
  vector_normalize(accelVector, accelVector);
  
  for (int i = 0; i < 3; i++) {
    readReg(ghx + 2*i, &readBuf, 1);
    val = ((unsigned short)readBuf) << 8;
    readReg(glx + 2*i, &readBuf, 1);
    val += (short)readBuf;
    vals[i] = val;
  }

  gyroVector->x = (float)vals[0];
  gyroVector->y = (float)vals[1];
  gyroVector->z = (float)vals[2];
  
  vector_add(gyroVector, &gyroBias, gyroVector);
}


void setup() {
  Serial.begin(115200);
  Serial.println("OPENED SERIAL");

  Wire.begin();
  Wire.setClock(100000);
  config();

  computeBiasComp(alx, ahx, &accelBias.x);
  computeBiasComp(aly, ahy, &accelBias.y);
  computeBiasComp(alz, ahz, &accelBias.z);

  computeBiasComp(glx, ghx, &gyroBias.x);
  computeBiasComp(gly, ghy, &gyroBias.y);
  computeBiasComp(glz, ghz, &gyroBias.z);

  accelBias.z += 2048; //~gravity

  trueUpGyro.x = 0;
  trueUpGyro.y = 0;
  trueUpGyro.z = 1;

  trueUpFusion.x = 0;
  trueUpFusion.y = 0;
  trueUpFusion.z = 1;
}

void loop() {
  timePrev = timeCurr;
  timeCurr = millis();
  unsigned long stepTime = (timeCurr - timePrev);
  
  //vectors of sensor readings
  struct vector accelVector;
  struct vector gyroVector;
  getVectors(&accelVector, &gyroVector);
  
  float angularVelocity = vector_normalize(gyroVector, gyroVector);
  float gyroAngle = angularVelocity * stepTime * PI / (180*16.4)/1000;  
  
  struct quaternion rotateQuat;
  //use a quaternion to apply a rotational transformation 
  quaternion_create(&gyroVector, -gAngle, &rotateQuat);
  quaternion_rotate(&trueUpGyro, &rotateQuat, &trueUpGyro);
  vector_normalize(&trueUpGyro, &trueUpGyro);

  quaternion_rotate(&trueUpFusion, &rotateQuat, &trueUpFusion);
  vector_multiply(&trueUpFusion, 1.0-alpha, &trueUpFusion);
  vector_multiply(&accelVector, alpha, &accelVector);
  vector_add(&accelVector, &trueUpFusion, &trueUpFusion);
  vector_normalize(&trueUpFusion,&trueUpFusion);


  //print components of each vector to be used in visualizer
  Serial.print(accelVector.x);
  Serial.print(" ");
  Serial.print(accelVector.y);
  Serial.print(" ");
  Serial.print(accelVector.z);
  Serial.print(" ");
  
  Serial.print(trueUpGyro.x);
  Serial.print(" ");
  Serial.print(trueUpGyro.y);
  Serial.print(" ");
  Serial.print(trueUpGyro.z);
  Serial.print(" ");

  Serial.print(trueUpFusion.x);
  Serial.print(" ");
  Serial.print(trueUpFusion.y);
  Serial.print(" ");
  Serial.println(trueUpFusion.z);
}
