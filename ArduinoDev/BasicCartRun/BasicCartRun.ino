#include <CurieIMU.h>
#include <Servo.h>
#include <MadgwickAHRS.h>

// Gyro readings are in radians and acceleration is in m/s
// (Must be transformed from g's for accel)
// Program must be reset every 70 mintues otherwise micros will overflow

// Variables
int i, rdySw;
int rdyPinIN = 2;
int servoPinX = 6;
int servoPinY = 9;
bool rdy, detecting;
const float Pi = 3.141593;
int aR = 2; // g's
int gR = 250; // deg/s
int uR = 25;
float prec = 32768.0;
unsigned long microsPrevious, microsNow, d;
unsigned long microsPerReading = 1000000 / uR;
float secPerReading = 1.0 / uR;
float secPerReadingSQ = pow(secPerReading, 2);

// Madgwick filter
Madgwick filter;

// Transform matrix sines and cosines
float sx, sy, sz;
float cx, cy, cz;

// Servo controllers
Servo xServo;
Servo yServo;

// Globa Accel (m/s)
float ca[3];

// Global Velocity (m/s)
float cvel[3];

// Global Distance (m)
float cdist[3];

// New Raw Values (g, deg/s)
float ax[3], gx[3];
float a[3], g[3];

// Filtered Raw Values (m/s^s, rad)
float fa[3], fg[3];

// Normal Plane Distance (m)
float ndist[3];

// Spherical transform for gyro (Deg 0-180)
float sTheta, sPhi;

// Scaling Values
float gToMS = 9.8;
float radToDeg = 180/Pi;
float alpha = 0.1;
float alpha2 = 1 - alpha;

static void zeroVars()
{
  for (i = 0; i < 3; i++)
  {
    cvel[i] = cdist[i] = 0.0;
    ax[i] = gx[i] = 0;
    a[i] = g[i] = 0.0;
    ca[i] = ndist[i] = 0.0;
  }
  sTheta = sPhi = 0.0;
  sx = sy = sz = 0.0;
  cx = cy = cz = 0.0;
}

void setup()
{
  // Initialize Serial communication
  Serial.begin(9600);
  
  // Initialize Variables
  rdy = false;
  detecting = false;
  pinMode(rdyPinIN, INPUT);
  rdySw = digitalRead(rdyPinIN);

  // Initialize Servos
  xServo.attach(servoPinX);
  yServo.attach(servoPinY);
  
  // Initialize the IMU
  CurieIMU.begin();

  // Set Rates for Gyro, Accel, and filter
  CurieIMU.setGyroRate(uR);
  CurieIMU.setAccelerometerRate(uR);
  filter.begin(uR);

  // Increase Accelerometer range to
  // allow detection of stronger taps (< 2g)
  CurieIMU.setAccelerometerRange(aR);

  // Set the accelerometer range to 500 degrees/second
  CurieIMU.setGyroRange(gR);

  // Pull data till it stabalizes
  d = uR * 15;
  while (0 < d)
  {
    microsNow = micros();
    if (microsPerReading < (microsNow - microsPrevious))
    {
      CurieIMU.readAccelerometerScaled(ax[0], ax[1], ax[2]);
      CurieIMU.readGyroScaled(gx[0], gx[1], gx[2]);

      filter.updateIMU(ax[0], ax[1], ax[2], gx[0], gx[1], gx[2]);
      
      microsPrevious = microsPrevious + microsPerReading;
      d = d - 1;
    }
  }

  // Zero all values
  zeroVars();
}

static void get_new_data()
{
  // read accel/gyro measurements from device, scaled to the configured range
  CurieIMU.readAccelerometerScaled(ax[0], ax[1], ax[2]);
  CurieIMU.readGyroScaled(gx[0], gx[1], gx[2]);
  
  // Update IMU
  filter.updateIMU(ax[0], ax[1], ax[2], gx[0], gx[1], gx[2]);

  // Update filtered values
  fg[0] = filter.getRollRadians();
  fg[1] = filter.getPitchRadians();
  fg[2] = filter.getYawRadians();

  for (i = 0; i < 3; i++)
  {
    fa[i] = alpha*ax[i] + alpha2 * fa[i];
  }
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

static void accel_to_global()
{
  // Compute the normal acceleration transform
  ca[0] = (fa[0]*(sx*sz+cx*cz*sy) + fa[1]*(cx*sy*sz-cz*sx) + fa[2]*cx*cy) - 0.08;
  ca[1] = (fa[0]*(cz*sx*sy-cx*sz) + fa[1]*(cx*cz+sx*sy*sz) + fa[2]*(cy*sx)) - 0.5;
  ca[2] = (fa[0]*cy*cz + fa[1]*(cy*sz) + fa[2]*(-sy)) + 1.0;
}

static void update_globalVals()
{
  // Get accel timing info (take us to s)

  for (i = 0; i < 3; i++)
  {
    cdist[i] = cdist[i] + cvel[i]*secPerReading + 0.5*gToMS*ca[i]*secPerReadingSQ;
    cvel[i] = cvel[i] + gToMS*ca[i]*secPerReading;
  }
}

static void dist_to_normal()
{
  // Compute the normal distance transform
  ndist[0] = cdist[0]*cy*cx + cdist[1]*(cz*sx*sy-cx*sz) + cdist[2]*sy;
  ndist[1] = cdist[0]*cy*sz + cdist[1]*(cx*cz+sx*sy*sz) + cdist[2]*(cx*sy*sz-cz*sx);
  ndist[2] = cdist[0]*-sy + cdist[1]*cy*sx + cdist[2]*cx*cy;
}

static void ndist_to_spherical()
{
  sTheta = atan2(ndist[1], ndist[0])*radToDeg;
  sPhi = atan2((pow(ndist[0], 2) + pow(ndist[1], 2)), ndist[2])*radToDeg;
}

void set_servos()
{
  xServo.write(sTheta);
  yServo.write(sPhi);
}

static void setup_angles()
{ 
  // Set starting distances
  cdist[0] = 11.0;
  cdist[1] = 11.0;
  cdist[2] = 1.0;
}

static void begin_motion_detection()
{
  microsPrevious = micros();
  int j = 0;
  
  while (rdySw)
  {
    rdySw = digitalRead(rdyPinIN);
    microsNow = micros();
    if (microsPerReading < (microsNow - microsPrevious))
    {
      // read measurements from device, scaled to the configured range
//      Serial.println("Get vals...");
      get_new_data();
  
      // Compute the sine and cosine values
//      Serial.println("Sin/cos...");
      compute_scVals();
  
      // transform normal plane accel to global plane
//      Serial.println("Global Plane...");
      accel_to_global();
  
      // Update the distance & velocity in the global plane
//      Serial.println("Update Globals...");
      update_globalVals();
  
      // Compute the new servo angle in the base system
//      Serial.println("Normal Dist...");
      dist_to_normal();
      
      // Spherical transform for servo angles
//      Serial.println("Spherical Normal Dist...");
      ndist_to_spherical();
  
      // Set servo angles to desired
//      Serial.println("Set Servos...");
      set_servos();
  
      // Wait for an update
//      Serial.println("Delay...");
//      Serial.println("");

      j = j + 1;
      if (uR <= j)
      {
        j = 0;
        for (i = 0; i < 3; i++)
        {
          Serial.print(ax[i]);
          Serial.print(" ");
        }
        Serial.println("");

        for (i = 0; i < 3; i++)
        {
          Serial.print(gx[i]);
          Serial.print(" ");
        }
        Serial.println("");

        for (i = 0; i < 3; i++)
        {
          Serial.print(fa[i]);
          Serial.print(" ");
        }
        Serial.println("");

        for (i = 0; i < 3; i++)
        {
          Serial.print(fg[i]);
          Serial.print(" ");
        }
        Serial.println("");

        for (i = 0; i < 3; i++)
        {
          Serial.print(ca[i]);
          Serial.print(" ");
        }
        Serial.println("");

        for (i = 0; i < 3; i++)
        {
          Serial.print(cvel[i]);
          Serial.print(" ");
        }
        Serial.println("");

        for (i = 0; i < 3; i++)
        {
          Serial.print(cdist[i]);
          Serial.print(" ");
        }
        Serial.println("");

        for (i = 0; i < 3; i++)
        {
          Serial.print(ndist[i]);
          Serial.print(" ");
        }
        Serial.println("");
        
        Serial.print("X Ang: ");
        Serial.println(sTheta);
        
        Serial.print("Y Ang: ");
        Serial.println(sPhi);
  
        Serial.println("");
      }

      microsPrevious = microsPrevious + microsPerReading;
    }
  }
}

// Used to continuously relay data over bluetooth connection
void loop()
{
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
    setup_angles();
  }
}
