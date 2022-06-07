#define HX711_C1  31
#define HX711_C2  33
#define HX711_C3  35
#define HX711_C4  37
#define HX711_D1  30
#define HX711_D2  32
#define HX711_D3  34
#define HX711_D4  36

int32_t hx711_read(int clk_pin, int data_pin)
{
  while (digitalRead(data_pin) == HIGH); // Wait for amplifier to be ready.

  uint32_t hx711_data = 0;
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

  return hx711_data;
}

int32_t l1_cal = 0, l2_cal = 0, l3_cal = 0, l4_cal = 0;

void setup()
{
  Serial.begin(230400);
  pinMode(HX711_C1, OUTPUT);
  pinMode(HX711_C2, OUTPUT);
  pinMode(HX711_C3, OUTPUT);
  pinMode(HX711_C4, OUTPUT);
  pinMode(HX711_D1, INPUT);
  pinMode(HX711_D2, INPUT);
  pinMode(HX711_D3, INPUT);
  pinMode(HX711_D4, INPUT);

  delay(1000);

  for (int i = 0; i < 100; i++)
  {
    //l1_cal += hx711_read(HX711_C1, HX711_D1) / 100;
    l2_cal += hx711_read(HX711_C2, HX711_D2) / 100;
    //l3_cal += hx711_read(HX711_C3, HX711_D3) / 100;
    l4_cal += hx711_read(HX711_C4, HX711_D4) / 100;
  }
}

void loop()
{
  Serial.print(/*String((hx711_read(HX711_C1, HX711_D1) - l1_cal) * -(5.0 / 105000.0)) + ", " +*/
               String((hx711_read(HX711_C2, HX711_D2) - l2_cal) * -(5.0 / 100000.0)) + ", " +
               /*String((hx711_read(HX711_C3, HX711_D3) - l3_cal) * -(5.0 / 101000.0)) + ", " +  */
               String((hx711_read(HX711_C4, HX711_D4) - l4_cal) * -(5.0 / 111000.0)) + "\n");
}
