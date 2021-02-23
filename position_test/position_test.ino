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
#define BNO055_SAMPLERATE_DELAY_MS 10

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

double xPos = 0.0;
double yPos = 0.0;
double zPos = 0.0;
double xVel = 0.0;
double yVel = 0.0;
double zVel = 0.0;
double h = double(BNO055_SAMPLERATE_DELAY_MS) / 1000;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
double rk2_vel(double a, double h, double vi) {
  return vi + a * h;
}

double rk2_pos(double a, double h, double vi, double xi){
  return xi + h * vi + a / 2 * h * h;
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

  xPos = rk2_pos(ax, h, xVel, xPos);
  yPos = rk2_pos(ay, h, yVel, yPos);
  zPos = rk2_pos(az, h, zVel, zPos);
  
  xVel = rk2_vel(ax, h, xVel);
  yVel = rk2_vel(ay, h, yVel);
  zVel = rk2_vel(az, h, zVel);

  Serial.print(F("xPos: "));
  Serial.print(xPos);
  Serial.print(" ");
  Serial.print(F("yPos: "));
  Serial.print(yPos);
  Serial.print(" ");
  Serial.print(F("zPos: "));
  Serial.print(zPos);
  Serial.println();
;
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
