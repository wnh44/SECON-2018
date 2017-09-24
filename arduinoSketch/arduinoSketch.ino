// test code for raspi<->aruino comm

char dataString[50] = {0};
int a = 0;

int ultrasonicPin = A0;
int distance;

int wait = 100; // 1000 ms

void setup()
{
  Serial.begin(9600);
  // Declare outputs here and inputs (if DI?)
}

void loop()
{
  a++;
  distance = analogRead(ultrasonicPin) / 2;
  sprintf(dataString, "%02X", distance);
  Serial.println(dataString);
  //Serial.print(distance);
  delay(wait);
}
