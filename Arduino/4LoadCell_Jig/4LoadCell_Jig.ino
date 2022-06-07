#define HX711_C  8
#define HX711_D  9

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

  if (hx711_data > 8000000) hx711_data -= pow(2, 24);

  return hx711_data;
}

int32_t l_cal = 0;

void setup()
{
  Serial.begin(230400);
  pinMode(HX711_C, OUTPUT);
  pinMode(HX711_D, INPUT);

  delay(1000);

  for (int i = 0; i < 100; i++)
  {
    l_cal += hx711_read(HX711_C, HX711_D) / 100;
  }
}

void loop()
{
  Serial.print(String((hx711_read(HX711_C, HX711_D) - l_cal) * 1.0) + "\n");
}
