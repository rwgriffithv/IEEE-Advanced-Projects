#include <sensor_fusion.cpp>

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

double bax;
double bay;
double baz;
double bgx;
double bgy;
double bgz;


void setup(){

  
  unsigned long sumax = 0;
  unsigned long sumay = 0;
  unsigned long sumaz = 0;

  unsigned long sumgx = 0;
  unsigned long sumgy = 0;
  unsigned long sumgz = 0;

  short ready;
  short xh;
  short xl;
  short yh;
  short yl;
  short zh;
  short zl;
  
  for(size_t i = 0; i < 75 ; i++)
  {
    do
    {
      readRegister(58,&ready,1)
    }while(!(ready & 0x01))
    
    readRegister(ahx,&xh,1);
    readRegister(alx,&xl,1);
    readRegister(ahy,&yh,1);
    readRegister(aly,&yl,1);
    readRegister(ahz,&zh,1);
    readRegister(alz,&zl,1);

    sumax += (xh << 4 + xl);
    sumay += (yh << 4 + yl);
    sumaz += (zh << 4 + zl);

    readRegister(ghx,&xh,1);
    readRegister(glx,&xl,1);
    readRegister(ghy,&yh,1);
    readRegister(gly,&yl,1);
    readRegister(ghz,&zh,1);
    readRegister(glz,&zl,1);

    sumgx += (xh << 4 + xl);
    sumgy += (yh << 4 + yl);
    sumgz += (zh << 4 + zl);
  }
  
  bax = (sumax / 75.0);
  bay = (sumay / 75.0);
  baz = (sumaz / 75.0) - 2048.0;

  bgx = sumgx /75.0;
  bgy = sumgy / 75.0;
  bgz = sumgz /75.0;
}

