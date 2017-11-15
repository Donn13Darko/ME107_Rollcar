#include <Wire.h>

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ;
int16_t Tmp;
int16_t GcX, GcY, GcZ;

int minVal = 265;
int maxVal = 402;

double x, y, z;

void setup()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  mpu.setXAccelOffset(100);
  mpu.setYAccelOffset(100);
  mpu.setZAccelOffset(1788);
  
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  
}

void loop()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX = Wire.read()<<8 | Wire.read();
  AcY = Wire.read()<<8 | Wire.read();
  AcZ = Wire.read()<<8 | Wire.read();
  Tmp = Wire.read()<<8 | Wire.read();
  GcX = Wire.read()<<8 | Wire.read();
  GcY = Wire.read()<<8 | Wire.read();
  GcZ = Wire.read()<<8 | Wire.read();

   Serial.print("Accel = ");
   Serial.print(AcX);
   Serial.print(" ");
   Serial.print(AcY);
   Serial.print(" ");
   Serial.println(AcZ);
   Serial.print("Gyro = ");
   Serial.print(GcX);
   Serial.print(" ");
   Serial.print(GcY);
   Serial.print(" ");
   Serial.println(GcZ);
   Serial.println("-----------------------------------------");
   delay(400);
}
