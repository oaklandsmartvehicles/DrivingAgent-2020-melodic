/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  Serial.begin(9600);
  myservo.attach(13);  // attaches the servo on pin 9 to the servo object
}
void loop() {
     // scale it to use it with the servo (value between 0 and 180)
  myservo.writeMicroseconds (1600);
  delay(200);
  myservo.writeMicroseconds (1950);
  delay(200);
  myservo.writeMicroseconds (1600);

  
  Serial.println("Pull");
  delay(5000);// waits for the servo to get there
  
  myservo.writeMicroseconds (1950);
  delay(200);
  myservo.writeMicroseconds (1600);
  delay(200);
  myservo.writeMicroseconds (1950);

  
  Serial.println("release");
  delay(10000);// waits for the servo to get there



}
