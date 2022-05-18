#include <FaBo9Axis_MPU9250.h>
#include <MadgwickAHRS.h>

Madgwick filter;
FaBo9Axis fabo_9axis;
unsigned long microsPerReading, microsPrevious;

void setup() {
  Serial.begin(115200);

  // start the IMU and filter
  fabo_9axis.begin();
  fabo_9axis.configMPU9250(MPU9250_GFS_2000, MPU9250_AFS_2G);
  
  filter.begin(25);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, yaw;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    fabo_9axis.readAccelXYZ(&ax, &ay, &az);
    fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
    fabo_9axis.readMagnetXYZ(&mx, &my, &mz);
    //fabo_9axis.readTemperature(&temp);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}
