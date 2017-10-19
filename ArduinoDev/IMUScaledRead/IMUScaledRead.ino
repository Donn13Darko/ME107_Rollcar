#include "CurieIMU.h"

float ax, ay, az;   // scaled accelerometer values
float gx, gy, gz;   // scaled gyroscope values

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);

  // Set the accelerometer range to 250 degrees/second
  CurieIMU.setGyroRange(250);
}

void loop() {
  // read accelerometer measurements from device, scaled to the configured range
  CurieIMU.readAccelerometerScaled(ax, ay, az);

  // read gyro measurements from device, scaled to the configured range
  CurieIMU.readGyroScaled(gx, gy, gz);

  // display tab-separated accelerometer x/y/z values
  Serial.print("a:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();

  // display tab-separated accelerometer x/y/z values
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();
  
  Serial.println();

  delay(100);
}
