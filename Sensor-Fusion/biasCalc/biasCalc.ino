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
  uint8_t ready;
  uint8_t temp;

    for (int i = 0; i < 75; i++) {
    do
      {
        readReg(58,&ready,1);
      }while(!(ready & 0x01));


    readReg(highBit,&temp,1);
    *dest += temp << 4;
    readReg(lowBit,&temp,1);
    *dest += *dest + temp;
  }
  *dest = *dest / 75;
}

void setup(){
  Serial.begin(9600);
  Wire.begin();
  config();

  uint8_t* pwrMgmt;
  readReg(107, pwrMgmt, 1);
  Serial.print("Power Management Register: ");
  Serial.println(*pwrMgmt, BIN);

  uint8_t* set_full_scale;
  readReg(0x1B, set_full_scale, 1);
  Serial.print("Set Full Scale Register: ");
  Serial.println(*set_full_scale, BIN);

  uint8_t* con;
  readReg(26, con, 1);
  Serial.print("Con(fig?) Register: ");
  Serial.println(*con, BIN);
  
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

void loop()
{
}

