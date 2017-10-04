#include <CurieIMU.h>
#include <Servo.h>

// Gyro readings are in radians and acceleration is in m/s
// (Must be transformed from g's for accel)

Servo xServo;
Servo yServo;
float tar_dist, ddist;
int xAngle, yAngle;
bool rdy;

void setup() {
  // Initialize Serial communication & wait for connection
  Serial.begin(9600);
  while(!Serial) ;

  // Initialize Variables
  rdy = false;

  // Initialize Servos
  xServo.attach(9);
  yServo.attach(6);
  
  // Initialize the IMU
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);

  // Increase Accelerometer range to
  // allow detection of stronger taps (< 4g)
  CurieIMU.setAccelerometerRange(4);

  // Set the accelerometer range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // Reduce threshold to 
  // allow detection of weaker taps (>= 750mg)
  CurieIMU.setDetectionThreshold(CURIE_IMU_TAP, 750);

  // Enable Tap detection
  CurieIMU.interrupts(CURIE_IMU_TAP);
}

static void setup_angles()
{
  tar_dist = 1.00;

  xAngle = xServo.read();
  yAngle = yServo.read();
  
  rdy = true;
}

static void begin_motion_detection()
{
  if (!rdy)
  {
    return;
  }
  
  // New Raw Values
  float g[3];
  float a[3];

  // Filtered Raw Values
  float fg[3];
  float fa[3];

  // Normalized Accel Values
  float na[3];

  // Prev gyro values

  // Variables
  int i;
  float filt_new = 0.1;
  float filt_old = 0.9;
  double sx, sy, sz;
  double cx, cy, cz;
  
  while (true)
  {
    // read accelerometer measurements from device, scaled to the configured range
    CurieIMU.readAccelerometerScaled(a[0], a[1], a[2]);

    // read gyro measurements from device, scaled to the configured range
    CurieIMU.readGyroScaled(g[0], g[1], g[2]);

    // Filter raw data
    for (i = 0; i < 3; i++)
    {
      fg[i] = filt_new*g[i] + filt_old*fg[i];
      fa[i] = filt_new*a[i] + filt_old*fa[i];
    }

    // Setup the acceleration transform sines & cosines
    sx = cos(fg[0]);
    sy = cos(fg[1]);
    sz = cos(fg[2]);
    cx = cos(fg[0]);
    cy = cos(fg[1]);
    cz = cos(fg[2]);

    // Compute the acceleration transform
    na[0] = fa[0]*cy*cz + fa[1]*(cz*sx*sy+cx*sz) + fa[2]*(sx*sz-cx*cz*sy);
    na[1] = -fa[0]*cy*sz + fa[1]*(cx*cz-sx*sy*sz) + fa[2]*(cz*sx+cx*sy*sz);
    na[2] = fa[0]*sy - fa[1]*cy*sx + fa[2]*cx*cy;
    

    // Compute the new servo angle in the base system

    // Compute the servo angle transform

    // Set servo angles to desired

    // Wait for an update
    delay(10);
  }
}

static void eventCallback()
{
  // Wait for interrupt to signal start motion
  if (CurieIMU.getInterruptStatus(CURIE_IMU_TAP))
  {
    begin_motion_detection();
  } else if (true) // Put code for calibration button tap & attach interrupt to button
  {
    setup_angles();
  }
}

// Used to continuously relay data over bluetooth connection
void loop()
{
  
}
