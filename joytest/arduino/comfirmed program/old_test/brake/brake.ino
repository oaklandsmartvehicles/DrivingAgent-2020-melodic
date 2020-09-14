const int outPin = 9;        // pin9 that the Brake control is attached to

// variables:
int sensorValue = 0;         // the sensor value
void setup() {
  pinMode(outPin, OUTPUT);
  //digitalWrite(outPin, LOW);
  analogWrite(outPin, LOW);
  Serial.begin(9600);
}

void loop() {
  // read the from ROS command:
  sensorValue = 0;//replace this with ROS read command
 // sensorValue = map(sensorValue, 0, 255, 50, 170);
  // please don't remove the line below
  Serial.println(sensorValue);
 // sensorValue = constrain(sensorValue, 50, 170);
  analogWrite(outPin, sensorValue);
  delay(2000);

  sensorValue = 100;//replace this with ROS read command
 // sensorValue = map(sensorValue, 0, 255, 50, 170);
  // please don't remove the line below
  Serial.println(sensorValue);
 // sensorValue = constrain(sensorValue, 50, 170);
  analogWrite(outPin, sensorValue);
delay(2000);

}
