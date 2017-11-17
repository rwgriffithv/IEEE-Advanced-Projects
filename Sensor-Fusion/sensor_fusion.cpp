#include <Arduino.h>
#include <Wire.h>
#include "sensor_fusion.h"

void config()
{
    uint8_t* pwrMgmt;
    readRegister(107, pwrMgmt,1);
    *pwrMgmt |= 0x40;
    writeReg(107, pwrMgmt, 1);
    
    const uint8_t GYRO_CONFIG = 0x1B;
    const uint8_t set_full_scale = 0x18;
    writeReg(GRYO_CONFIG, &set_full_scale, 1);
    
    const uint8_t ACCEL_CONFIG = 0x1C;
    writeReg(ACCEL_CONFIG, &set_full_scale, 1);

    uint8_t* con;
    readRegister(26, con, 1);
    *con &= 0xF7;
    writeReg(26, con, 1);

}

const uint8_t ADRslave = 0x68;

void readReg(uint8_t reg, uint8_t *buf, size_t len)
{
    Wire.beginTransmission(ADRslave);
    
    uint8_t ADplusR = (reg << 1) | 1;
    writeReg(reg, &ADplusR, 1);
    
    Wire.endTransmission();
    
    if (Wire.requestFrom(ADRslave, len) == 0) 
    {
        Serial.print("Read called but nothing read from slave: ");
        Serial.println(ADRslave,BIN);
    }
    
    int i = 0;
    while(Wire.available())
    {
        buf[i] = Wire.read();
        i++:
    } 
}

void writeReg(uint8_t reg, uint8_t *buf, size_t len)
{
  Wire.beginTransmission(ADRslave);
    
  Wire.write(reg);
    
  while (len--)
    Wire.write(*(buf++));

  char ret = Wire.endTransmission();

  if (!ret){
    if (ret == 1)
      serial.print("Error 1: Data too long to fit in transmit buffer!");
    else if (ret == 2)
      Serial.print("Error 2: Received NACK on transmit of address!");
    else if (ret == 3)
      Serial.print("Error 3: Received NACK on transmit of data!");
    else
      Serial.print("Error 4: Other error!");
}

float vector_normalize(struct vector *raw, struct vector *unit)
{
  float mag = sqrt(pow(raw->x, 2) + pow(raw->y, 2) + pow(raw->z, 2));
  unit->x = raw->x / mag;
  unit->y = raw->y / mag;
  unit->z = raw->z / mag;
  return mag;
}

void vector_add(struct vector *v1, struct vector *v2, struct vector *result)
{
  result->x = v1->x + v2->x;
  result->y = v1->y + v2->y;
  result->z = v1->z + v2->z;
}

void vector_multiply(struct vector *v, float c, struct vector *result)
{
  result->x = v->x * c;
  result->y = v->y * c;
  result->z = v->z * c;
}

void quaternion_create(struct vector *v, float angle, struct quaternion *result)
{
  result->r = cos(angle/2);
  result->i = v->x * sin(angle/2);
  result->j = v->y * sin(angle/2);
  result->k = v->z * sin(angle/2);
}

void quaternion_rotate(struct vector *v, struct quaternion *q, struct vector *result)
{
  struct vector n;
  memcpy(&n, v, sizeof(n));
  result->x = n.x * (1 - 2*(pow(q->j, 2) + pow(q->k, 2))) + n.y * 2*(q->i * q->j - q->k * q->r) + n.z * 2*(q->i * q->k + q->j * q->r);
  result->y = n.x * 2*(q->i * q->j + q->k * q->r) + n.y * (1 - 2*(pow(q->i, 2) + pow(q->k, 2))) + n.z * 2*(q->j * q->k - q->i * q->r);
  result->z = n.x * 2*(q->i * q->k - q->j * q->r) + n.y * 2*(q->j * q->k + q->i * q->r) + n.z * (1 - 2*(pow(q->i, 2) + pow(q->j, 2)));
}

void quaternion_multiply(struct quaternion *q1, struct quaternion *q2, struct quaternion *result)
{
  result->r = q1->r * q2->r - q1->i * q2->i - q1->j * q2->j - q1->k * q2->k;
  result->i = q1->r * q2->i + q1->i * q2->r + q1->j * q2->k - q1->k * q2->j;
  result->j = q1->r * q2->j - q1->i * q2->k + q1->j * q2->r + q1->k * q2->i;
  result->k = q1->r * q2->k + q1->i * q2->j - q1->j * q2->i + q1->k * q2->r;
}
