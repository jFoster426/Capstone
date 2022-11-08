//
// MPU-6050 Mahony IMU  (yaw angle is relative to starting orientation)
// last update 7/9/2020 
//

#include "Wire.h"

// AD0 low = 0x68 (default for Sparkfun module)
// AD0 high = 0x69
int MPU_addr = 0x68;

// vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//These are the previously determined offsets and scale factors for accelerometer and gyro for
// a particular example of an MPU-6050. They are not correct for other examples.
//The IMU code will NOT work well or at all if these are not correct

float A_cal[6] = {0., 0., 0., 1., 1., 1.}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = {0., 0., 0.}; //raw offsets, determined for gyro at rest
#define gscale ((2000./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s

// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0;
float Ki = 0.0;

// globals for AHRS loop timing
unsigned long now_ms, last_ms = 0; //millis() timers

// print interval
unsigned long print_ms = 200; //print angles every "print_ms" milliseconds

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void setup() {

  Wire.begin();
  Serial.begin(115200);

  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(27);    // GYRO_CONFIG register
  Wire.write(0b00011000);     // self test off, 2000DPS
  Wire.endTransmission(true);

}

// AHRS loop

void loop()
{
  static int i = 0;
  static float deltat = 0;  //loop time in seconds
  static unsigned long now = 0, last = 0; //micros() timers

  //raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t Tmp; //temperature

  //scaled data as vector
  float Axyz[3];
  float Gxyz[3];


  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  int t = Wire.read() << 8;
  ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read() << 8;
  az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read() << 8;
  Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read() << 8;
  gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read() << 8;
  gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;

  //apply offsets and scale factors from Magneto
  // for (int i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

  Gxyz[0] = (float) gx * gscale; //250 LSB(d/s) default to radians/s
  Gxyz[1] = (float) gy * gscale;
  Gxyz[2] = (float) gz * gscale;

  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  last = now;

  Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

  uint16_t qi[4] = { uint16_t(q[0] * 16384.0), uint16_t(q[1] * 16384.0), uint16_t(q[2] * 16384.0), uint16_t(q[3] * 16384.0) };
  teapotPacket[2] = (qi[0] >> 8) & 0xff;
  teapotPacket[3] = qi[0] & 0xff;
  teapotPacket[4] = (qi[1] >> 8) & 0xff;
  teapotPacket[5] = qi[1] & 0xff;
  teapotPacket[6] = (qi[2] >> 8) & 0xff;
  teapotPacket[7] = qi[2] & 0xff;
  teapotPacket[8] = (qi[3] >> 8) & 0xff;
  teapotPacket[9] = qi[3] & 0xff;
  
  now_ms = millis(); //time to print?
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
  }
}
//--------------------------------------------------------------------------------------------------
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// last update 07/09/2020 SJR minor edits
//--------------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}
