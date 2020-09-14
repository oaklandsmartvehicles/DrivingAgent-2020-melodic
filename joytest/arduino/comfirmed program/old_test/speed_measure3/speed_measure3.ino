/*Need convert from Hz to mph
  speed measurement
  D0,1,2,3,4,5,6,7,8,9,10,11,12,13,A0,1,2,3,4,5
   x,x,2,3,4,5,6,7,8,9,10,11, x,13,A0,1,2,3,4,5
  --
  Pin assignment
  D12 speed in
  D3  speed out for acc & brake
  ---
  Wire assignment
  --speed measurement-------------------------------
                      --------
  5V------------------|4     |
  5V----|14        |  |LM324 |
  D12---|2 74LS14 1|--|1,2  3|---C--------SPEED SENSOR
  GND---|7         |  |      |   |0.1uF
  GND-----------------|11    |  GND
                      --------
  --------------------------------------------------
  -----Speed measurement ----
  D3---|LOWPASS|----|A0 acc & brake arduino|
       R=100k,C=0.33uF
  ---------------------------
TOPIC in-------
  nothing
  TOPIC out---------
  c_speed<- Float32
*/
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
//int SPDcount = 12;
int SPDcount = 8;
std_msgs::Float32 c_speed;
ros::Publisher pub("c_speed", &c_speed);
float speed = 0;
float out = 0.0;
void setup()
{
  pinMode(SPDcount, INPUT);
  nh.initNode();
  nh.advertise(pub);
}
void loop() {
  unsigned long duration1 = pulseIn(SPDcount, LOW,500000);
  unsigned long duration2 = pulseIn(SPDcount, HIGH,500000);
  if(duration1 > 2000) duration1 = 0;
  if(duration2 > 2000) duration2 = 0;
  out = (float)(duration1+duration2)/30 + out*29/30;
  speed = 220./629.*out-4251/629;
  if(speed < 0) speed = 0.0;
  c_speed.data = speed;
  pub.publish(&c_speed);
  nh.spinOnce();
}
