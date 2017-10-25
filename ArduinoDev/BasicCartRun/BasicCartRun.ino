#include <CurieIMU.h>
#include <Servo.h>

// Gyro readings are in radians and acceleration is in m/s
// (Must be transformed from g's for accel)
// Program must be reset every 70 mintues otherwise micros will overflow

// Variables
int i;
bool rdy, detecting;
const float Pi = 3.141593;
unsigned long dst, dsdt;
unsigned long gst, gsdt;

// Transform matrix sines and cosines
float sx, sy, sz;
float cx, cy, cz;

// Servo controllers
Servo xServo;
Servo yServo;

// Global Plane Velocity (m/s)
float cvel[3];

// Global Plane Distance (m)
float cdist[3];

// New Raw Values (g, deg/s)
float a[3], g[3];

// Filtered Raw Values (m/s^s, rad)
float fa[3], fg[3];

// Globa Accel Values (m/s)
float na[3];

// Normal Plane Distance Values (m)
float ndist[3];

// Spherical transform
float sTheta;
float sPhi;

// Gyro values (Deg 0-180)
int xAngle, yAngle;

// Scaling Values
float gToMS = 9.8;
float radToDeg = 180/Pi;
float degToRad = Pi/180;

// Filter Multipliers
float filt_new = 0.1;
float filt_old = 0.9;

// Multiplication Consts
float aRawFilt = gToMS*filt_new;
float gRawFilt = gToMS*degToRad;
float usToS = pow(10, -6);

static void zeroVars()
{
  for (i = 0; i < 3; i++)
  {
    cvel[i] = cdist[i] = 0;
    a[i] = g[i] = 0;
    fa[i] = fg[i] = 0;
    na[i] = ndist[i] = 0;
  }
  sTheta = sPhi = 0;
  xAngle = yAngle = 0;
  sx = sy = sz = 0;
  cx = cy = cz = 0;
  dst = dsdt = 0;
  gst = gsdt = 0;
}

void setup()
{
  // Initialize Serial communication & wait for connection
  Serial.begin(9600);
  while(!Serial) ;

  // Zero all values
  zeroVars();
  
  // Initialize Variables
  rdy = false;
  detecting = false;

  // Initialize Servos
  xServo.attach(9);
  yServo.attach(6);
  
  // Initialize the IMU
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);

  // Set Rates for Gyro, Accel, and filter
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);

  // Increase Accelerometer range to
  // allow detection of stronger taps (< 4g)
  CurieIMU.setAccelerometerRange(4);

  // Set the accelerometer range to 500 degrees/second
  CurieIMU.setGyroRange(500);

  // Reduce threshold to 
  // allow detection of weaker taps (>= 750mg)
  CurieIMU.setDetectionThreshold(CURIE_IMU_TAP, 750);

  // Enable Tap detection
  CurieIMU.interrupts(CURIE_IMU_TAP);
}

static void get_new_data()
{
  // read accelerometer measurements from device, scaled to the configured range
  CurieIMU.readAccelerometerScaled(a[0], a[1], a[2]);

  // read gyro measurements from device, scaled to the configured range
  CurieIMU.readGyroScaled(g[0], g[1], g[2]);

  // Get gyro timing info (take us to s)
  gsdt = (micros() - gst)*usToS;
  gst = micros();
  
  // Filter raw data
  for (i = 0; i < 3; i++)
  {
    fa[i] = aRawFilt*a[i] + filt_old*fa[i];
    fg[i] = gRawFilt*g[i]*gsdt + filt_old*fg[i];
  }
}

static void compute_scVals()
{
  // Setup the acceleration transform sines & cosines
  sx = cos(fg[0]);
  sy = cos(fg[1]);
  sz = cos(fg[2]);
  cx = cos(fg[0]);
  cy = cos(fg[1]);
  cz = cos(fg[2]);
}

static void accel_to_global()
{
  // Compute the acceleration transform
  na[0] = fa[0]*cy*cz + fa[1]*(cz*sx*sy+cx*sz) + fa[2]*(sx*sz-cx*cz*sy);
  na[1] = -fa[0]*cy*sz + fa[1]*(cx*cz-sx*sy*sz) + fa[2]*(cz*sx+cx*sy*sz);
  na[2] = fa[0]*sy - fa[1]*cy*sx + fa[2]*cx*cy;
}

static void dist_to_normal()
{
  // Compute the normal transform
  ndist[0] = cdist[0]*cy*cx - cdist[1]*cy*sz + cdist[2]*sy;
  ndist[1] = cdist[0]*(cz*sx*sy+cx*sz) + cdist[1]*(cx*cz-sx*sy*sz) - cdist[2]*cy*sx;
  ndist[2] = cdist[0]*(sx*sz-cx*cz*sy) + cdist[1]*(cz*sx+cx*sy*sz) + cdist[2]*cx*cy;
}

static void update_globalVals()
{
  // Get accel timing info (take us to s)
  dsdt = (micros() - dst)*usToS;
  dst = micros();

  for (i = 0; i < 3; i++)
  {
    cdist[i] = cdist[i] + cvel[i]*dsdt + 0.5*na[i]*pow(dsdt, 2);
    cvel[i] = cvel[i] + na[i]*dsdt;
  }
}

static void ndist_to_spherical()
{
  sTheta = atan(ndist[1]/ndist[0])*radToDeg;
  sPhi = atan((pow(ndist[0], 2) + pow(ndist[1], 2))/ndist[2])*radToDeg;
}

void set_servos()
{
  xServo.write(xAngle);
  yServo.write(yAngle);
}

static void setup_angles()
{
  // Get starting values
  xAngle = xServo.read();
  yAngle = yServo.read();

  // Get Accel
  get_new_data();

  // Set rdy to true
  rdy = true;
}

static void begin_motion_detection()
{  
  dst = micros();
  gst = micros();
  
  while (rdy)
  {
    // read measurements from device, scaled to the configured range
    get_new_data();

    // Compute the sine and cosine values
    compute_scVals();

    // transform normal plane accel to global plane
    accel_to_global();

    // Update the distance & velocity in the global plane
    update_globalVals();

    // Compute the new servo angle in the base system
    dist_to_normal();
    
    // Spherical transform for servo angles
    ndist_to_spherical();

    // Set servo angles to desired
    xServo.write(xAngle);
    yServo.write(yAngle);

    // Wait for an update
    delay(10);
  }
}

static void eventCallback()
{
  Serial.println("Interrupt!");
  // Wait for interrupt to signal start motion
  if (CurieIMU.getInterruptStatus(CURIE_IMU_TAP))
  {
    if (!detecting & rdy)
    {
      detecting = true;
      begin_motion_detection();
    }
  } else if (true) // Put code for calibration button tap & attach interrupt to button
  {
    setup_angles();
  }
}

// Used to continuously relay data over bluetooth connection
void loop()
{
  
}
