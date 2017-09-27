#include <CurieIMU.h>
#include <BasicLinearAlgebra.h>
#include <Geometry.h>

void setup() {
  // Initialize Serial communication & wait for connection
  Serial.begin(9600);
  while(!Serial) ;
  
  // Initialise the IMU
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);

  // Increase Accelerometer range to
  // allow detection of stronger taps (< 4g)
  CurieIMU.setAccelerometerRange(4);

  // Reduce threshold to 
  // allow detection of weaker taps (>= 750mg)
  CurieIMU.setDetectionThreshold(CURIE_IMU_TAP, 750);

  // Enable Tap detection
  CurieIMU.interrupts(CURIE_IMU_TAP);
}

static void begin_motion_detection()
{
  // New Raw Values
  float g[3];
  float a[3];

  // Filtered Raw Values
  float fg[3];
  float fa[3];

  // Normalized Accel Values
  float na[3];

  // Variables
  int i;
  float filt_new = 0.1;
  float filt_old = 0.9;
  
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

    delay(10);
  }
}

static void eventCallback()
{
  // Wait for interrupt to signal start motion
  if (CurieIMU.getInterruptStatus(CURIE_IMU_TAP))
  {
    begin_motion_detection();
  }
}

// Used to continuously relay data over bluetooth connection
void loop()
{
  
}
