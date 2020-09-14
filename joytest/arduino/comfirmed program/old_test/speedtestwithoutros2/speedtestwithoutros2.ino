int SPDcount = 8;
float speed = 0;
void setup()
{
  pinMode(SPDcount, INPUT);
  Serial.begin(9600);
}
float out = 0.0, out1=0;
void loop() {
  bool flag = digitalRead(SPDcount);
  Serial.println((int)flag);
}
