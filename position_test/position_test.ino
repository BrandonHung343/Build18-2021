#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 5
#define DRIFT_CAL_CYCLES 500

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);
SimpleKalmanFilter kf(0, 0, 0.1);
double xPos = 0.0;
double yPos = 0.0;
double zPos = 0.0;
double xVel = 0.0;
double yVel = 0.0;
double zVel = 0.0;
double xDrift = 0.0;
double yDrift = 0.0;
double zDrift = 0.0;
double h = double(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void calibrate_accel() {
  int i = 0;
  while(i < DRIFT_CAL_CYCLES){
     imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
     xDrift = xDrift + accel.x();
     yDrift = yDrift + accel.y();
     zDrift = zDrift + accel.z();
     i++;
     delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  xDrift = xDrift / i;
  yDrift = yDrift / i;
  zDrift = zDrift / i;
  kf.setMeasurementError(yDrift);
  kf.setEstimateError(yDrift);
}

double rk2_vel(double a, double h, double vi, double drift) {
  return vi + (a) * h / 2;
}

double rk2_pos(double a, double h, double vi, double xi, double drift){
  return xi + (h * vi) / 2 + (a) * h * h / 6;
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");

  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
 
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Integration Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
//  while(accel < 3){
//    displayCalStatus();
//    bno.getCalibration(&system, &gyro, &accel, &mag);
//    delay(100);
//  }

  Serial.print("Internal cal done.\nCalibrating sensors in 2 sec...\n");
  delay(2000);
  // Calibrate sensors
  Serial.println("Calibrating...");
  calibrate_accel();
  Serial.print(F("xDrift: "));
  Serial.print(xDrift);
  Serial.print(F(", yDrift: "));
  Serial.print(yDrift);
  Serial.print(F(", zDrift: "));
  Serial.println(zDrift);
  delay(1000);
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL );

  /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */

  /* The processing sketch expects data as roll, pitch, heading */
  
  
  double ax = accel.x();
  double ay = accel.y();
  double az = accel.z();

  ay = kf.updateEstimate(ay);

  xPos = rk2_pos(ax, h, xVel, xPos, 0);
  yPos = rk2_pos(ay, h, yVel, yPos, yDrift);
  zPos = rk2_pos(az, h, zVel, zPos, 0);
  
  xVel = rk2_vel(ax, h, xVel, 0);
  yVel = rk2_vel(ay, h, yVel, yDrift);
  zVel = rk2_vel(az, h, zVel, 0);


  Serial.print(F("yPos: "));
  Serial.print(yPos, 5);
  Serial.print(" yAccel: ");
  Serial.print(ay, 5);

  Serial.println();

  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
