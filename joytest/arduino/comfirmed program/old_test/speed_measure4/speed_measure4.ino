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
float spd[16];
void setup()
{
  pinMode(SPDcount, INPUT);
  nh.initNode();
  nh.advertise(pub);
  for(int i = 0;i < 16;i++) spd[i]=2.0*(float)i/3.;
}
// time out 50mS ->50000
unsigned long c_millis;
unsigned long duration;
int count = 0;
void loop() {
  c_millis = millis() + 50;
  count = 0;
  while(1) {
    duration = pulseIn(SPDcount, LOW,50000);
    if(c_millis < millis()) break;
    count++;
  }
//  if(count < 16) {
    out = 0.8*out + 0.2*(float)count;
    c_speed.data = (float)100/duration;//spd[count];
    pub.publish(&c_speed);
//  }
  nh.spinOnce();
}
