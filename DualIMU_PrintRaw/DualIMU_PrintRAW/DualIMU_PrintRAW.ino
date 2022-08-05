#include <MPU6500_WE.h>
#include <Wire.h>

MPU6500_WE myMPU6500_1 = MPU6500_WE(0x68);
MPU6500_WE myMPU6500_2 = MPU6500_WE(0x69);

//void thisfunction(int arg1, float arg2):
//void printRawData():

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  myMPU6500_1.init();
  myMPU6500_2.init();

  // First IMU Data
  myMPU6500_1.enableGyrDLPF();
  myMPU6500_1.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500_1.setSampleRateDivider(5);
  myMPU6500_1.setGyrRange(MPU6500_GYRO_RANGE_500);
  myMPU6500_1.setAccRange(MPU6500_ACC_RANGE_8G);
  myMPU6500_1.enableAccDLPF(true);
  myMPU6500_1.setAccDLPF(MPU6500_DLPF_6);

  // Second IMU Data
  myMPU6500_2.enableGyrDLPF();
  myMPU6500_2.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500_2.setSampleRateDivider(5);
  myMPU6500_2.setGyrRange(MPU6500_GYRO_RANGE_500);
  myMPU6500_2.setAccRange(MPU6500_ACC_RANGE_8G);
  myMPU6500_2.enableAccDLPF(true);
  myMPU6500_2.setAccDLPF(MPU6500_DLPF_6);

  delay(200);
}

void printRawData()
{
  xyzFloat acc_1 = myMPU6500_1.getGValues();
  xyzFloat gyr_1 = myMPU6500_1.getGyrValues();

  xyzFloat acc_2 = myMPU6500_2.getGValues();
  xyzFloat gyr_2 = myMPU6500_2.getGyrValues();

  Serial.print(millis()), Serial.print(',');
  Serial.print(acc_1.x);
  Serial.print(",");
  Serial.print(acc_1.y);
  Serial.print(",");
  Serial.print(acc_1.z);
  Serial.print(",");
  Serial.print(gyr_1.x);
  Serial.print(",");
  Serial.print(gyr_1.y);
  Serial.print(",");
  Serial.print(gyr_1.z);
  Serial.print(",");
  Serial.print(acc_2.x);
  Serial.print(",");
  Serial.print(acc_2.y);
  Serial.print(",");
  Serial.print(acc_2.z);
  Serial.print(",");
  Serial.print(gyr_2.x);
  Serial.print(",");
  Serial.print(gyr_2.y);
  Serial.print(",");
  Serial.print(gyr_2.z);
  Serial.print("\n");
}

float angleWrap(float current_angle, float previous_angle)
{
  float wrapped_angle;

  if (previous_angle - current_angle > PI)
  {
    wrapped_angle = current_angle + 2 * PI;
  }
  else if (previous_angle - current_angle < -PI)
  {
    wrapped_angle = current_angle - 2 * PI;
  }
  else
  {
    wrapped_angle = current_angle;
  }

  return wrapped_angle;
}

void combinationFilter()
{
  float prev_f_roll = 0;
  float prev_f_pitch = 0;
  float prev_s_roll = 0;
  float prev_s_pitch = 0;
  float prev_roll = 0;
  float prev_pitch = 0;

  float f_roll = 0;
  float s_roll = 0;
  float f_pitch = 0;
  float s_pitch = 0;

  float prev_s_acc_pitch = 0;
  float prev_s_acc_roll = 0;
  float prev_f_acc_pitch = 0;
  float prev_f_acc_roll = 0;
  float prev_time = 0;

  float roll = 0;
  float pitch = 0;
  while (1)
  {
    xyzFloat acc_1 = myMPU6500_2.getGValues();
    xyzFloat gyr_1 = myMPU6500_2.getGyrValues();

    xyzFloat acc_2 = myMPU6500_1.getGValues();
    xyzFloat gyr_2 = myMPU6500_1.getGyrValues();

    float Time = millis();

    // Footplate IMU Data
    float f_acc_x = acc_1.x;
    float f_acc_y = acc_1.y;
    float f_acc_z = acc_1.z;

    float s_acc_x = acc_2.x;
    float s_acc_y = acc_2.y;
    float s_acc_z = acc_2.z;

    // Shinstrap IMU Data
    float f_gyr_x = gyr_1.x;
    float f_gyr_y = gyr_1.y;
    float f_gyr_z = gyr_1.z;

    float s_gyr_x = gyr_2.x;
    float s_gyr_y = gyr_2.y;
    float s_gyr_z = gyr_2.z;

    // Output variables
    float f_acc_pitch;
    float s_acc_pitch;
    float f_acc_roll;
    float s_acc_roll;
    float f_roll;
    float s_roll;
    float f_pitch;
    float s_pitch;

    // Calculate footplate roll
    /*
      if (f_acc_z > 0) // Above x-y plane
      f_acc_roll = atan2(f_acc_x, -1*sqrt(f_acc_y*f_acc_y + f_acc_z*f_acc_z)); // rad
      else // below x-y plane
      f_acc_roll = atan2(f_acc_x, sqrt(f_acc_y*f_acc_y + f_acc_z*f_acc_z)); // rad

      // calculate footplate pitch
      f_acc_pitch = atan2(f_acc_y, f_acc_z); // rad

      // Calculate shinstrap roll
      if  (s_acc_z > 0) // above x-y plane
      s_acc_roll = atan2(s_acc_x, -1*sqrt(s_acc_y*s_acc_y + s_acc_z*s_acc_z)); // rad
      else // below x-y plane
      s_acc_roll = atan2(s_acc_x, sqrt(s_acc_y*s_acc_y + s_acc_z*s_acc_z)); // rad

      // Calculate shinstrap pitch
      s_acc_pitch = atan2(s_acc_y, s_acc_z); // rad
    */

    if (s_acc_z > 0)
      s_acc_pitch = atan2(-1 * s_acc_x, sqrt(s_acc_y * s_acc_y + s_acc_z * s_acc_z));
    else
      s_acc_pitch = atan2(   s_acc_x, sqrt(s_acc_y * s_acc_y + s_acc_z * s_acc_z));
    s_acc_roll = atan2(s_acc_y, s_acc_z);

    if (f_acc_z > 0)
      f_acc_pitch = atan2(-1 * f_acc_x, sqrt(f_acc_y * f_acc_y + f_acc_z * f_acc_z));
    else
      f_acc_pitch = atan2(   f_acc_x, sqrt(f_acc_y * f_acc_y + f_acc_z * f_acc_z));
    f_acc_roll = atan2(f_acc_y, f_acc_z);

    // This allows for continuous arc tan angles across multiple branches
    s_acc_pitch = angleWrap(s_acc_pitch, prev_s_acc_pitch);
    s_acc_roll = angleWrap(s_acc_roll, prev_s_acc_roll);
    f_acc_pitch = angleWrap(f_acc_pitch, prev_f_acc_pitch);
    f_acc_roll = angleWrap(f_acc_roll, prev_f_acc_roll);

    // Calculate Joint Angle
    f_roll = prev_f_roll + (PI / 180) * f_gyr_x * (Time - prev_time) / 1000; // rad
    s_roll = prev_s_roll + (PI / 180) * s_gyr_x * (Time - prev_time) / 1000; // rad
    f_pitch = prev_f_pitch + (PI / 180) * f_gyr_y * (Time - prev_time) / 1000; // rad
    s_pitch = prev_s_pitch + (PI / 180) * s_gyr_y * (Time - prev_time) / 1000; // rad

    // Combination filter
    f_roll  = 0.98 * f_roll  + 0.02 * f_acc_roll;
    s_roll  = 0.98 * s_roll  + 0.02 * s_acc_roll;
    f_pitch = 0.98 * f_pitch + 0.02 * f_acc_pitch;
    s_pitch = 0.98 * s_pitch + 0.02 * s_acc_pitch;

    // Store previous measured parameters
    prev_s_acc_pitch = s_acc_pitch;
    prev_s_acc_roll = s_acc_roll;
    prev_f_acc_pitch = f_acc_pitch;
    prev_f_acc_roll = f_acc_roll;
    prev_time = Time;
    prev_f_roll = f_roll;
    prev_f_pitch = f_pitch;
    prev_s_roll = s_roll;
    prev_s_pitch = s_pitch;

    // Calculate joint angle wrap correction
    roll = angleWrap(roll, prev_roll);
    pitch = angleWrap(pitch, prev_pitch);

    // Calculate Joint Angle
    roll  = (180 / PI) * (s_roll - f_roll); // deg
    pitch = (180 / PI) * (s_pitch - f_pitch); // deg

    s_roll = (180 / PI) * s_roll;
    f_roll = (180 / PI) * f_roll;
    s_pitch = (180 / PI) * s_pitch;
    f_pitch = (180 / PI) * f_pitch;
    roll = (180 / PI) * roll;
    pitch = (180 / PI) * pitch;

    prev_roll = roll;
    prev_pitch = pitch;

    // Print the variables to the serial monitor and plotter
    //Serial.print(Time), Serial.print(',');
    //Serial.print(s_acc_pitch);
    //Serial.print(",");
    //Serial.print(s_acc_roll);
    //Serial.print(",");
    //Serial.print(roll);
    //Serial.print(",");
    //Serial.print(s_roll);
    //Serial.print(',');
    Serial.print(pitch);
    Serial.print(',');
    Serial.print(roll);
    Serial.print("\n");

    // Convert back to radians, as some variables are saved across multiple loops
    s_roll = s_roll * (PI / 180);
    f_roll = f_roll * (PI / 180);
    s_pitch = s_pitch * (PI / 180);
    s_roll = s_roll * (PI / 180);
    roll = roll * (PI / 180);
    pitch = pitch * (PI / 180);
  }
}

void loop()
{
  //printRawData();
  combinationFilter();
}
