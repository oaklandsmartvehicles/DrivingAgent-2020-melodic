int SPDcount = 8;
float speed = 0;
void setup()
{
  pinMode(SPDcount, INPUT);
  Serial.begin(9600);
}
float out = 0.0, out1=0;
void loop() {
 // unsigned long duration0 = pulseIn(SPDcount, HIGH,500000);
  unsigned long duration1 = pulseIn(SPDcount, LOW,500000);
  if(duration1 > 2000) duration1 = 0;
//  if(duration0 > 2000) duration0 = 0;
 out = (float)duration1/100 + out*99/100;
  out1 = (float)duration1;
  speed = 220./629.*out-4251/629;
  if(speed < 0) speed = 0.0;
  //if(speed > 300.0) speed = 0.0; // Frequency from 1 to 300 [Hz]
  //else {
  //  if(speed < 1.0) speed = 0.0;
  //}
  if(duration1 > 2000) duration1 = 0;
 //  // Serial.print(duration0 + duration1);

 // // Serial.print(" ");
//  Serial.print(duration0);
//Serial.print(" speed ");
// // Serial.println(speed);
  

      Serial.print(out); Serial.print(" ");  Serial.println(speed);
}
