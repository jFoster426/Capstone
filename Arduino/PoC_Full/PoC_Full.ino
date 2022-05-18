#include <MPU9250_WE.h>
#include <Wire.h>

#define HX711_C   6
#define HX711_D   7
#define LED_GRN   9
#define LED_RED   10
#define BUTTON    11
#define SHIN_ADDR 0x69
#define FOOT_ADDR 0x68

MPU9250_WE foot = MPU9250_WE(FOOT_ADDR);
MPU9250_WE shin = MPU9250_WE(SHIN_ADDR);

bool recording = false;
unsigned long int buttonPressTime = 0;

void setup() {
  Serial.begin(230400);

  pinMode(HX711_C, OUTPUT);
  pinMode(HX711_D, INPUT);
  digitalWrite(HX711_C, LOW); // Clock should be low during idle.

  pinMode(BUTTON, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GRN, LOW);

  Wire.begin();
  Wire.setTimeout(100);
  if (!foot.init()) while (1); // Serial.print("Foot IMU does not respond.\n");
  //else Serial.println("Foot IMU is connected.");
  if (!shin.init()) while (1); // Serial.print("Shin IMU does not respond.\n");
  //else Serial.println("Shin IMU is connected.");
  Serial.print("\nCompleted initialization.\n");

  foot.setAccOffsets(0, 0, 0, 0, 0, 0);
  foot.setGyrOffsets(0, 0, 0);
  shin.setAccOffsets(0, 0, 0, 0, 0, 0);
  shin.setGyrOffsets(0, 0, 0);

  foot.setSampleRateDivider(1);
  foot.setAccRange(MPU9250_ACC_RANGE_4G);
  foot.setGyrRange(MPU9250_GYRO_RANGE_1000);
  foot.enableAccDLPF(true);
  foot.setAccDLPF(MPU9250_DLPF_6);

  shin.setSampleRateDivider(1);
  shin.setAccRange(MPU9250_ACC_RANGE_4G);
  shin.setGyrRange(MPU9250_GYRO_RANGE_1000);
  shin.enableAccDLPF(true);
  shin.setAccDLPF(MPU9250_DLPF_6);
}

void loop() {

  if (recording == true)
  {
    xyzFloat facc = foot.getGValues();
    xyzFloat fgyr = foot.getGyrValues();
    xyzFloat fmag = foot.getMagValues();

    xyzFloat sacc = shin.getGValues();
    xyzFloat sgyr = shin.getGyrValues();
    xyzFloat smag = shin.getMagValues();

    while (digitalRead(HX711_D) == HIGH); // Wait for amplifier to be ready.

    uint32_t hx711_data = 0;
    hx711_data |= shiftIn(HX711_D, HX711_C, MSBFIRST);
    hx711_data <<= 8;
    hx711_data |= shiftIn(HX711_D, HX711_C, MSBFIRST);
    hx711_data <<= 8;
    hx711_data |= shiftIn(HX711_D, HX711_C, MSBFIRST);

    // 25 pulses = channel A, gain of 128.
    delayMicroseconds(50); // Pulse the clk pin to bring output back to high.
    digitalWrite(HX711_C, HIGH);
    delayMicroseconds(50);
    digitalWrite(HX711_C, LOW);

    Serial.print(String(millis()) + "," +    // 1
                 String(facc.x) + "," +      // 2
                 String(facc.y) + "," +      // 3
                 String(facc.z) + "," +      // 4
                 String(fgyr.x) + "," +      // 5
                 String(fgyr.y) + "," +      // 6
                 String(fgyr.z) + "," +      // 7
                 String(sacc.x) + "," +      // 8
                 String(sacc.y) + "," +      // 9
                 String(sacc.z) + "," +      // 10
                 String(sgyr.x) + "," +      // 11
                 String(sgyr.y) + "," +      // 12
                 String(sgyr.z) + "," +      // 13
                 String(hx711_data) + "\n"); // 14
  }

  if (digitalRead(BUTTON) == HIGH &&     // button is pressed.
      millis() - buttonPressTime > 100)  // debouncing.
  {
    if (recording == false)
    {
      recording = true;
      Serial.print("\nSTART\n");
      //Serial.println("CSV Format\n");
      //Serial.println("time,faccx,faccy,faccz,fgyrx,fgyry,fgyrz,saccx,saccy,saccz,sgyrx,sgyry,sgyrz,load\n");
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GRN, HIGH);
    }
    else
    {
      recording = false;
      Serial.print("END\n");
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GRN, LOW);
    }
    while (digitalRead(BUTTON) == HIGH); // wait for button release.
    buttonPressTime = millis();
  }
}
