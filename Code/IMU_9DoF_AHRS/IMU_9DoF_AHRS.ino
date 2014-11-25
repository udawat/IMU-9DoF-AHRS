// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//          and by Explore Labs <info@explorelabs.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-01-04 - added raw magnetometer output
//     2014-09-23 - Explore Labs IMU 9DoF AHRS compatibility

/* ============================================
 I2Cdev device library code is placed under the MIT license
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */
/*****************************************************************
 * IMU_9DoF_AHRS.ino
 * Explore Labs IMU 9DoF AHRS Data Fusion Example Code
 * Original Creation by Kris Winer for Sparkfun Electronics
 * Original Date: April 8, 2014
 * Modification Date: September 22, 2014
 * https://github.com/explorelabs/IMU-9DoF-AHRS
 * 
 * The MPU9150 is a versatile 9DOF sensor. It has a built-in
 * accelerometer, gyroscope, and magnetometer that
 * functions over I2C. It is very similar to the 6 DoF MPU6050 for which an extensive library has already been built.
 * Most of the function of the MPU9150 can utilize the MPU6050 library.
 * 
 * This Arduino sketch utilizes Jeff Rowberg's MPU6050 library to generate the basic sensor data
 * for use in two sensor fusion algorithms becoming increasingly popular with DIY quadcopter and robotics engineers. 
 * Kris added and slightly modified Jeff's library here.
 * Some more modifications were required to make it compatible with Explore Labs' IMU 9DoF AHRS.
 * 
 * This simple sketch will demo the following:
 * Sensor readings coming out of the serial port of ATmega328 at 57600 bps.
 * Baud rate of receiver xBee, if used, should also be set to 57600 bps.
 * An example of how to use the quaternion data to generate standard aircraft orientation data in the form of
 * Tait-Bryan angles representing the sensor yaw, pitch, and roll angles suitable for any vehicle stablization control application.
 * Here, Sebastian Madgwick's Filter is used as shown on: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * Output Format: $,Yaw,Pitch,Roll,Heading,Altutde,Humidity,Temperature#
 * Future Software Support for Latitude, Longitude and other important GPS data.
 * 
 * Development environment specifics:
 * 	IDE: Arduino 1.0.6
 * 	Hardware Platform: IMU 9DoF AHRS (ATmega328 @ 3.3V running at 8MHz)
 * 
 *****************************************************************/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"
#include "MPL3115A2.h"
#include "HTU21D.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
// Declare device MPU6050 class
MPU6050 mpu;
MPL3115A2 myPressure; // Create instance for Pressure sensor - MPL3115A2
HTU21D myHumidity; // Create instance for Humidity sensor - HTU21D

// Global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// Kris hasn't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float deltat = 0.0f;     // integration interval for both filter schemes
uint16_t lastUpdate = 0; // used to calculate integration interval
uint16_t now = 0;        // used to calculate integration interval

#define LED_PIN 13       // LED connected on-board on pin 13
#define userLED 5        // user programmable LED connected on-board on pin 5.
                         // Here, it is turned ON if Relative Humidity is above 50%
#define gpsLED 8         // LED for GPS 1PPS output connected on-board on pin 8.
                         // Can also be used as a user programmable LED when no GPS module connected.
bool blinkState = false; // LED blink for activity

int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3;     // raw data arrays reading
uint16_t mcount = 0; // used to control display output rate
uint8_t MagRate;     // read rate for magnetometer data

float pitch, yaw, roll;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float Xh, Yh, heading;  // Heading from North in degrees

float altitude, temp1, temp2, temperature, humidity; // Sensor readings
float pressure; // for setModeBarometer() function.

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  
  // initialize serial communication
  // (57600 chosen because xBee Series 1 programmed at the same baud rate and it 
  // allows for wireless programming (to be demonstrated in future with a simple
  // Green-wire hack) of ATmega328.
  Serial.begin(57600);
  
  // initialize devices
  Serial.println("Initializing I2C devices...");
  mpu.initialize();  // Get various sensors online
  myPressure.begin();
  myHumidity.begin();

  // Configure the Pressure sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(1); // Set Oversample to 001
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  // Change Resolution of Humidity sensor as RH resolution = 8 bit and
  // Temperature resolution = 12 bit for faster updates.
  myHumidity.setResolution(0b10000000); // change RH resolution for fast calculations

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Set up the accelerometer, gyro, and magnetometer for data output
  mpu.setRate(7); // set gyro rate to 8 kHz/(1 * rate) shows 1 kHz, accelerometer ODR is fixed at 1 KHz
  
  MagRate = 10; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
// Digital low pass filter configuration. 
// It also determines the internal sampling rate used by the device as shown in the table below.
// The accelerometer output rate is fixed at 1kHz. This means that for a Sample
// Rate greater than 1kHz, the same accelerometer sample may be output to the
// FIFO, DMP, and sensor registers more than once.
/*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 */
  mpu.setDLPFMode(4);  // set bandwidth of both gyro and accelerometer to ~20 Hz

  // Full-scale range of the gyro sensors:
  // 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
  mpu.setFullScaleGyroRange(0); // set gyro range to 250 degrees/sec

  // Full-scale accelerometer range.
  // The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
  mpu.setFullScaleAccelRange(0); // set accelerometer to 2 g range

  mpu.setIntDataReadyEnabled(true); // enable data ready interrupt

  // configure on-board LED for showing activity
  pinMode(LED_PIN, OUTPUT);
  Serial.println("IMU 9 DoF AHRS ready. Wait for a few seconds to get stable Yaw readings.");
}

void loop() {
  if(mpu.getIntDataReadyStatus() == 1) { // wait for data ready status register to update all data registers
    mcount++;
    //read the raw sensor data
    mpu.getAcceleration(&a1, &a2, &a3);
    //    ax = a1*2.0f/32768.0f + 0.038f;
    //    ay = a2*2.0f/32768.0f - 0.005f;
    //    az = a3*2.0f/32768.0f + 0.089f;
    ax = a1*2.0f/32768.0f;    // 2 g full range for accelerometer
    ay = a2*2.0f/32768.0f;
    az = a3*2.0f/32768.0f;
    mpu.getRotation(&g1, &g2, &g3);
    gx = g1*250.0f/32768.0f;  // 250 deg/s full range for gyroscope
    gy = g2*250.0f/32768.0f;
    gz = g3*250.0f/32768.0f;
    //  The gyros and accelerometers can in principle be calibrated in addition to any factory calibration but they are generally
    //  pretty accurate. You can check the accelerometer by making sure the reading is +1 g in the positive direction for each axis.
    //  The gyro should read zero for each axis when the sensor is at rest. Small or zero adjustment should be needed for these sensors.
    //  The magnetometer is a different thing. Most magnetometers will be sensitive to circuit currents, computers, and 
    //  other both man-made and natural sources of magnetic field. The rough way to calibrate the magnetometer is to record
    //  the maximum and minimum readings (generally achieved at the North magnetic direction). The average of the sum divided by two
    //  should provide a pretty good calibration offset. Don't forget that for the MPU9150, the magnetometer x- and y-axes are switched 
    //  compared to the gyro and accelerometer!
    if (mcount > 1000/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
      mpu.getMag(&m1, &m2, &m3);
      //    mx = m1*10.0f*1229.0f/4096.0f + 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
      //    my = m2*10.0f*1229.0f/4096.0f + 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
      //    mz = m3*10.0f*1229.0f/4096.0f + 270.0f;
      mx = m1*10.0f*1229.0f/4096.0f;
      my = m2*10.0f*1229.0f/4096.0f;
      mz = m3*10.0f*1229.0f/4096.0f;
      mcount = 0;
    }
  }
  now = micros();
  deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz); //pass gyro readings in rad/s
  // MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

  // Uncomment the below to view Raw readings.
  /*
   Serial.print("ax = "); Serial.print((int)1000*ax);  
   Serial.print(" ay = "); Serial.print((int)1000*ay); 
   Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
   Serial.print("gx = "); Serial.print( gx, 2); 
   Serial.print(" gy = "); Serial.print( gy, 2); 
   Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
   Serial.print("mx = "); Serial.print( (int)mx ); 
   Serial.print(" my = "); Serial.print( (int)my ); 
   Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
   
   Serial.print("q0 = "); Serial.print(q[0]);
   Serial.print(" qx = "); Serial.print(q[1]); 
   Serial.print(" qy = "); Serial.print(q[2]); 
   Serial.print(" qz = "); Serial.println(q[3]); 
   */

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI;
  roll  *= 180.0f / PI;

  /* Calculated non-compensated heading with compass lying flat on surface.
  heading = atan2(my, mx);
  //  Correct for when signs are reversed. Will output heading in 0 to 360 degrees.
  if(heading < 0)
    heading += 2*PI;
  */
  
  heading = tiltCompensatedHeading(ax, ay, mx, my, mz);
  // pressure = myPressure.readPressure();
  altitude = myPressure.readAltitude();
  temp1 = myPressure.readTemp();
  humidity = myHumidity.readHumidity();
  // temp2 = myHumidity.readTemperature();

  // Average of temperature readings from Pressure Sensor (MPL3115A2)
  // and Humidity Sensor (HTU21D).
  // temperature = (temp1+temp2)/2.0;
  
  /* Future software support for GPS */
  //  if(GPS.availalbe()) {
  //    printSensorReadingsGPS();
  //  }
  //  else {
      printSensorReadings();
  //  }
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;   

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
      eInt[0] += ex;      // accumulate integral error
      eInt[1] += ey;
      eInt[2] += ez;
  }
  else
  {
      eInt[0] = 0.0f;     // prevent integral wind up
      eInt[1] = 0.0f;
      eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

/*
From https://www.loveelectronics.co.uk/Tutorials/13/tilt-compensated-compass-arduino-tutorial
and https://github.com/landis/arduino/blob/master/tiltcompass/tiltcompass.ino
*/
float tiltCompensatedHeading(float aX, float aY, float mX, float mY, float mZ) {   
  // Another technique for tilt compensation ~40 degrees
  // Configure this for your setup.
  float accX = aY; // or -ay;
  float accY = aX;
  
  float rollRadians = asin(accY);
  float pitchRadians = asin(accX);
  
  // We cannot correct for tilt over 40 degrees with this algorthem, if the board is tilted as such, return 0.
  if(rollRadians > 0.78 || rollRadians < -0.78 || pitchRadians > 0.78 || pitchRadians < -0.78)
  {
    return 0;
  }
  
  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(rollRadians);
  float sinRoll = sin(rollRadians);  
  float cosPitch = cos(pitchRadians);
  float sinPitch = sin(pitchRadians);
  
  float Xh = mX * cosPitch + mZ * sinPitch;
  float Yh = mX * sinRoll * sinPitch + mY * cosRoll - mZ * sinRoll * cosPitch;
  
  heading = atan2(Yh, Xh);
  return heading;
  }

float RadiansToDegrees(float rads)
{
  // Correct for when signs are reversed.
  if(rads < 0)
    rads += 2*PI;
      
  // Check for wrap due to addition of declination.
  if(rads > 2*PI)
    rads -= 2*PI;
   
  // Convert radians to degrees for readability.
  float heading = rads * 180/PI;
       
  return heading;
}

void printSensorReadings() {
  Serial.print("$,");
  Serial.print(yaw, 2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print(RadiansToDegrees(heading), 0);
  Serial.print(",");  
  Serial.print(altitude, 0);
  Serial.print(",");
  Serial.print(humidity, 0);
  Serial.print(",");  
  Serial.print(temp1, 2);
  Serial.println("#");   
  
  // Blink LED to show activity.
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  
  if (humidity > 50.0)
    digitalWrite(userLED, HIGH);
  else
    digitalWrite(userLED, LOW);
}
