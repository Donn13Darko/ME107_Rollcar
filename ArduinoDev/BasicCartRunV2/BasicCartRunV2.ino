//#include <SdFat.h>
//#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define INTERRUPT_PIN 2

// Gyro readings are in radians and acceleration is in m/s
// (Must be transformed from g's for accel)
// Program must be reset every 70 mintues otherwise micros will overflow

// Variables
int i, j , k, l, rdySw, endCal;
int rdyPinIN = 3;
int servoPinX = 9;
int servoPinY = 6;
bool rdy, detecting;
const float Pi = 3.141593;
int uR = 200;
float prec = 32768.0;
unsigned long microsPrevious;
float microsNow;
float gOffZ;

// MPU6050
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Save Data
//SdFat sd;
//File testData;
//int pinCS = 10;

// Transform matrix sines and cosines
float sx, sy, sz;
float cx, cy, cz;

// Servo controllers
Servo xServo;
Servo yServo;

// Globa Accel (m/s^2)
float ca[3];

// Global Velocity (m/s)
float cvel[3];

// Global Distance (m)
float cdist[3];

// New Raw Values (g, deg/s)
float ax[3], gx[3];
int16_t a[3], g[3];
int16_t tm;

// Filtered Raw Values (m/s^s, rad)
float fa[3], fg[3];

// Normal Plane Distance (m)
float ndist[3];

// Spherical transform for gyro (Deg 0-180)
float sTheta, sPhi;

// Scaling Values
float rawToG = 16384.0;
float gToMS = 9.81;
float radToDeg = 180.0/Pi;
float alpha = 0.05;
float alpha2 = gToMS * alpha / rawToG;
float alphaO = 1 - alpha;

// Function Prototypes
static void get_new_data();

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

static void zeroVars()
{
  for (i = 0; i < 3; i++)
  {
    cvel[i] = cdist[i] = 0.0;
    ax[i] = gx[i] = 0;
    a[i] = g[i] = 0;
    ca[i] = ndist[i] = 0.0;
  }
  sTheta = sPhi = 0.0;
  sx = sy = sz = 0.0;
  cx = cy = cz = 0.0;
}

void setup()
{
  // Start I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize Serial communication
  Serial.begin(115200);
  
  // Initialize Variables
  rdy = false;
  detecting = false;
  pinMode(rdyPinIN, INPUT);
//  pinMode(pinCS, OUTPUT);
  rdySw = digitalRead(rdyPinIN);

  // Initialize Servos
  xServo.attach(servoPinX);
  yServo.attach(servoPinY);

  // Initilize the SD Card
//  sd.begin(pinCS, SD_SCK_MHZ(50));
//  testData = sd.open("test.txt", FILE_WRITE);
//
//  testData.println("Start of Data Collection");
//  testData.flush();
  
  // Initialize the IMU
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  
  mpu.setXAccelOffset(2142);
  mpu.setYAccelOffset(358);
  mpu.setZAccelOffset(1115);
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(-112);
  mpu.setZGyroOffset(7);
  if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  gOffZ = 0.0;
}

static void get_new_data()
{
  // read accel/gyro measurements from device, scaled to the configured range
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Update filtered values
  ca[0] = alpha2*aaWorld.y + alphaO*ca[0];
  ca[1] = alpha2*aaWorld.x + alphaO*ca[1];
  ca[2] = alpha2*aaWorld.z + alphaO*ca[2];

  fg[0] = alpha*ypr[1] + alphaO*fg[0];
  fg[1] = alpha*ypr[2] + alphaO*fg[1];
  fg[2] = alpha*(ypr[0]-gOffZ) + alphaO*fg[2];
}

static void compute_scVals()
{
  // Setup the acceleration transform sines & cosines
  sx = sin(fg[0]);
  sy = sin(fg[1]);
  sz = sin(fg[2]);
  cx = cos(fg[0]);
  cy = cos(fg[1]);
  cz = cos(fg[2]);
}

static void update_globalVals()
{
  // Get accel timing info (take us to s)
  microsNow = 1.0 * (micros() - microsPrevious) / 1000000.0;
  microsPrevious = micros();

  for (i = 0; i < 3; i++)
  {
    cdist[i] = cdist[i];// + cvel[i]*microsNow + 0.5*ca[i]*microsNow;
    cvel[i] = cvel[i] + ca[i]*microsNow;
  }
}

static void dist_to_normal()
{
  // Compute the normal distance transform
  ndist[0] = cdist[0]*cy*cx + cdist[1]*(cz*sx*sy-cx*sz) + cdist[2]*(sx*sz+cx*cz*sy);
  ndist[1] = cdist[0]*cy*sz + cdist[1]*(cx*cz+sx*sy*sz) + cdist[2]*(cx*sy*sz-cz*sx);
  ndist[2] = cdist[0]*-sy + cdist[1]*cy*sx + cdist[2]*cx*cy;

//  ndist[0] = cdist[0]*cy*cz + cdist[1]*-(cy*sz) + cdist[2]*sy;
//  ndist[1] = cdist[0]*(cz*sx*sy+cx*sz) + cdist[1]*(cx*cz-sx*sy*sz) + cdist[2]*-(cy*sx);
//  ndist[2] = cdist[0]*(sx*sz-cx*cz*sy) + cdist[1]*(cz*sx+cx*sy*sz) + cdist[2]*cx*cy;
}

static void ndist_to_spherical()
{
  sTheta = 90 + atan2(ndist[1], ndist[0])*radToDeg;
  sPhi = 90 - atan2(ndist[2], sqrt((pow(ndist[0], 2) + pow(ndist[1], 2))))*radToDeg;
}

void set_servos()
{
  xServo.write(sTheta);
  yServo.write(sPhi);
}

static void setup_angles()
{ 
  zeroVars();
  
  // Set starting distances
  cdist[0] = 1.0;
  cdist[1] = 1.0;
  cdist[2] = 0.0;
}

static void printToSerial()
{
  Serial.print("fa:\t");
  for (i = 0; i < 3; i++)
  {
    Serial.print(fa[i]);
    Serial.print("\t");
  }
  Serial.println("");

  Serial.print("fg:\t");
  for (i = 0; i < 3; i++)
  {
    Serial.print(fg[i]);
    Serial.print("\t");
  }
  Serial.println("");

  Serial.print("ca:\t");
  for (i = 0; i < 3; i++)
  {
    Serial.print(ca[i]);
    Serial.print("\t");
  }
  Serial.println("");

  Serial.print("cvel:\t");
  for (i = 0; i < 3; i++)
  {
    Serial.print(cvel[i]);
    Serial.print("\t");
  }
  Serial.println("");

  Serial.print("cdist:\t");
  for (i = 0; i < 3; i++)
  {
    Serial.print(cdist[i]);
    Serial.print("\t");
  }
  Serial.println("");

  Serial.print("ndist:\t");
  for (i = 0; i < 3; i++)
  {
    Serial.print(ndist[i]);
    Serial.print("\t");
  }
  Serial.println("");
  
  Serial.print("X Ang:\t");
  Serial.println(sTheta);
  
  Serial.print("Y Ang:\t");
  Serial.println(sPhi);

  Serial.println("");
}

//static void printToFile()
//{
//  testData.print("fa:\t");
//  for (i = 0; i < 3; i++)
//  {
//    testData.print(fa[i]);
//    testData.print("\t");
//  }
//  testData.println("");
//
//  testData.print("fg:\t");
//  for (i = 0; i < 3; i++)
//  {
//    testData.print(fg[i]);
//    testData.print("\t");
//  }
//  testData.println("");
//
//  testData.print("ca:\t");
//  for (i = 0; i < 3; i++)
//  {
//    testData.print(ca[i]);
//    testData.print("\t");
//  }
//  testData.println("");
//
//  testData.print("cvel:\t");
//  for (i = 0; i < 3; i++)
//  {
//    testData.print(cvel[i]);
//    testData.print("\t");
//  }
//  testData.println("");
//
//  testData.print("cdist:\t");
//  for (i = 0; i < 3; i++)
//  {
//    testData.print(cdist[i]);
//    testData.print("\t");
//  }
//  testData.println("");
//
//  testData.print("ndist:\t");
//  for (i = 0; i < 3; i++)
//  {
//    testData.print(ndist[i]);
//    testData.print("\t");
//  }
//  testData.println("");
//  
//  testData.print("X Ang:\t");
//  testData.println(sTheta);
//  
//  testData.print("Y Ang:\t");
//  testData.println(sPhi);
//
//  testData.println("");
//  testData.flush();
//}

static void begin_motion_detection()
{
  microsPrevious = micros();
  j = 0;
  k = 0;
  l = 0;
  endCal = 1000;
  
  while (rdySw)
  {
    // Update button and block till interrupt
    rdySw = digitalRead(rdyPinIN);
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    }

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // Wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // Read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // Track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // Stabalize sensor
      if (k < endCal)
      {
        get_new_data();
        k = k + 1;

        if (k == endCal)
        {
          setup_angles();
          microsPrevious = micros();
          gOffZ = fg[2];
        }
      } else
      {
        
        // Read measurements from device, scaled to the configured range
//        Serial.println("Get vals...");
        get_new_data();
    
        // Compute the sine and cosine values
//        Serial.println("Sin/cos...");
        compute_scVals();
    
        // Update the distance & velocity in the global plane
//        Serial.println("Update Globals...");
        update_globalVals();
    
        // Compute the new servo angle in the base system
//        Serial.println("Normal Dist...");
        dist_to_normal();
        
        // Spherical transform for servo angles
//        Serial.println("Spherical Normal Dist...");
        ndist_to_spherical();

        // Set servos
        set_servos();

        j = j + 1;
        if (uR <= j)
        {
          j = 0;
          
          printToSerial();
//          printToFile();
        }

        l = l + 1;
        if (endCal == l)
        {
          cvel[0] = cvel[1] = cvel[2] = 0;
          l = 0;
        }
      }
    }
  }
}

// Used to continuously relay data over bluetooth connection
void loop()
{
  if (!dmpReady) return;
  
  rdySw = digitalRead(rdyPinIN);
  if (rdySw & rdy & !detecting)
  {
    Serial.println("Detecting...");
    detecting = true;
    begin_motion_detection();
    rdy = false;
    detecting = false;
  } else if (rdySw & !rdy & !detecting) // Put code for calibration button tap & attach interrupt to button
  {
    Serial.println("Setting up values...");
    rdy = true;
  }
}
