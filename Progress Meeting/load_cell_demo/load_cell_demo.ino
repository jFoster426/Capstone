// Run on TEENSY 3.2

int aanalog(int samp)
{
  unsigned long int x = 0;
  for (int j = 0; j < samp; j++)
  {
    x += analogRead(A0);
  }
  x /= samp;
  return x;
}

int cal;

void setup() {
  analogWriteResolution(12);
  pinMode(A14, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  int i;
  for (i = 2000; i <= 4095; i++)
  {
    analogWrite(A14, i);
    delay(10);
    int x = aanalog(10);
    Serial.println(x);
    if (x < 300) break;
  }
  if (i < 4095) digitalWrite(2, HIGH);
  else digitalWrite(3, HIGH);
}

String s(20);

void loop() {
  Serial.println(aanalog(1000));
  if (Serial.available() > 0)
  {
    char buf = Serial.read();
    if (buf == '\n')
    {
      cal = s.toInt();
      s = "";
    }
    else s += buf;
  }
  delay(20);
}
