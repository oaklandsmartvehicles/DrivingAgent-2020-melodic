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
bool c_flag = false;
bool p_flag = false;
void setup()
{
  pinMode(SPDcount, INPUT_PULLUP);
  nh.initNode();
  nh.advertise(pub);
  for(int i = 0;i < 16;i++) spd[i]=2.0*(float)i/3.;
}
unsigned long c_millis = 0;
unsigned long acount,count;
void loop() {
  acount = 0;
  count = 0;
  c_millis = millis() + 100;
  p_flag = PINB & _BV(0)?true:false;
//  p_flag = digitalRead(SPDcount);
  while(1) {
    c_flag = PINB & _BV(0)?true:false;
 //   c_flag = digitalRead(SPDcount);
    if(c_flag == HIGH && p_flag == LOW) count++;
    if(c_flag == LOW && p_flag == HIGH) count++;
    acount++;
    p_flag = c_flag;
    if(c_millis < millis()) break;
  }
//  count = (int)((float)acount / ((float)count+1.0));
//  out = out*.9+(float)1000.0/count * .1;
  c_speed.data = (float) acount;//spd[count];
  pub.publish(&c_speed);
  nh.spinOnce();
}
