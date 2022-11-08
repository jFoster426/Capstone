#include "Wire.h"
#include <Adafruit_NeoPixel.h>

// AD0 high = 0x69
int MPU_addr_wrist = 0x68;
int MPU_addr_hand = 0x69;

#define HX711_C  D2
#define HX711_D  D3

#define NUMPIXELS 1

const uint8_t BUTTON_PIN = D7;

const uint8_t POWER = 11;
const uint8_t PIN  = 12;

bool runningFlag = false;
bool wasPressedFlag = false;
 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int32_t hx711_read(int clk_pin, int data_pin)
{
  
  while (digitalRead(data_pin) == HIGH); // Wait for amplifier to be ready.

  int32_t hx711_data = 0;
  hx711_data |= shiftIn(data_pin, clk_pin, MSBFIRST);
  hx711_data <<= 8;
  hx711_data |= shiftIn(data_pin, clk_pin, MSBFIRST);
  hx711_data <<= 8;
  hx711_data |= shiftIn(data_pin, clk_pin, MSBFIRST);

  // 25 pulses = channel A, gain of 128.
  delayMicroseconds(50); // Pulse the clk pin to bring output back to high.
  digitalWrite(clk_pin, HIGH);
  delayMicroseconds(50);
  digitalWrite(clk_pin, LOW);

  //if (hx711_data > 8000000) hx711_data -= pow(2, 24);

  return -hx711_data;
  
}

int32_t l_cal = 0;

void setup()
{

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Serial.begin(115200);
  pinMode(HX711_C, OUTPUT);
  pinMode(HX711_D, INPUT);

  delay(1000);

//  for (int i = 0; i < 100; i++)
//  {
//    l_cal += hx711_read(HX711_C, HX711_D) / 100;
//  }

  Wire.begin();
  Serial.begin(115200);

  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr_wrist);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr_wrist);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0b00011000);     // set to 2000 DPS
  Wire.endTransmission(true);

  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr_hand);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr_hand);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0b00011000);     // set to 2000 DPS
  Wire.endTransmission(true);

  pixels.begin();
  pinMode(POWER,OUTPUT);
  digitalWrite(POWER, HIGH);
}

void loop()
{
  // Button is pressed
  if(wasPressedFlag == false && digitalRead(BUTTON_PIN) == LOW)
  {
    wasPressedFlag = true;
    if (runningFlag == false)
    {
      runningFlag = true;
    }
    else
    {
      runningFlag = false;
      // Red LED
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.show(); 
      delay(100); // lazy debounce method
    }
  }
  // Button is released
  else if(wasPressedFlag == true && digitalRead(BUTTON_PIN) == HIGH)
  {
    wasPressedFlag = false;
  }

  // When not running
  if(runningFlag == false)
  {
    // Red LED
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();  
  }
  // When running
  else
  {
    // Green LED
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();  

    int16_t t;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t Tmp; //temperature
    uint32_t tim;
  
    Wire.beginTransmission(MPU_addr_wrist);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr_wrist, 14, true); // request a total of 14 registers
    t = Wire.read() << 8;
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
  
    char s[128];
    tim = millis();
    snprintf(s,sizeof(s),"%d,%d,%d,%d,%d,%d,%d, ", tim, ax, ay, az, gx, gy, gz);
    Serial.print(s);
  
    Wire.beginTransmission(MPU_addr_hand);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr_hand, 14, true); // request a total of 14 registers
    t = Wire.read() << 8;
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
  
  //  char s[128];
  //  t = millis();
    snprintf(s,sizeof(s),"%d,%d,%d,%d,%d,%d, ", ay, ax, -az, gy, gx, -gz); // Note orientation shift of this IMU
    Serial.print(s);
    
    Serial.print(String((hx711_read(HX711_C, HX711_D) - l_cal) * -1.0) + ", " + String((hx711_read(HX711_C, HX711_D) - l_cal) * -1.0) + "\n");
  }
}
