/*
  reverse and safty lihgt
  D0,1,2,3,4,5,6,7,8,9,10,11,12,13,A0,1,2,3,4,5
   x,x,2,x,x,x,x,x,x,x,10,11,12,13, x,1,2,3,4,5
  --
  Pin assignment
  D7  relay1 forward/reverse forward=NC reverse=NO
  D6  no connection 
  D9  relay3 yellow light       on=HIGH, off=LOW
  D8  relay4 red light     
  
  ---
  Wire assignment
  --reverse/forward-------------------------------

  D7--------|relay1  NC|---forward
            |        NO|---reverse
  
  ---safty rlight------------------------------------
  D8--------|
          ---------
          |NO     |
  -RED----|NC  COM|----RED-----RED LAMP max 2A
        ---------
     safty ylight
  D9----------|
          ---------
          |NO     |
  --RED---|NC  COM|----RED-----YELLOW LAMP max 2A
        ---------
  TOPIC in-------
  Rlight  -> Int16
  Ylight  -> Int16
  reverse -> Bool
  c_estop   -> Bool
  TOPIC out---------
  c_reverse <- bool
  c_rlight  <- Int16
  c_ylight  <- Int16
*/

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
ros::NodeHandle nh;
std_msgs::Bool c_reverse; // for debugging
std_msgs::Int16 c_rlight; // for debugging

ros::Publisher pubreverse("c_reverse", &c_reverse);
ros::Publisher pubrlight("c_rlight", &c_rlight);


int forwardreverse_relay = 7;//  D5  relay3 reverse/forward
int rlight_relay = 4;//  D4  relay4
int ylight_relay = 5;//  D5  relay3
unsigned long publishtime;
bool reverse = false;
int ylight = 0;
int rlight = 0;
bool estop_flag = false;
bool estop_flag_pre = false;
bool ch[]={false,false,false,false,false,false,false,false,false,false,false,false,false,false};
void digitalWrite1(int num,bool val) {
  if(val != ch[num]) {
    digitalWrite(num,val);
    ch[num] = val;
  }
}

void reverseCb(const std_msgs::Bool &Reverse)
{
  reverse = Reverse.data;
  c_reverse.data = reverse;
  pubreverse.publish(&c_reverse);
  ylight = (int) reverse;
}
void rlightCb(const std_msgs::Int16 &Rlight)
{
  rlight = Rlight.data;
  c_rlight.data = rlight;
  pubrlight.publish(&c_rlight);
}
void estopCb(const std_msgs::Bool &Estop)
{
  estop_flag = Estop.data;
}

ros::Subscriber<std_msgs::Bool> subestop("c_estop", &estopCb);
ros::Subscriber<std_msgs::Int16> subrlight("rlight", &rlightCb);
ros::Subscriber<std_msgs::Bool> subreverse("reverse", &reverseCb);
void setup()
{
  pinMode(forwardreverse_relay, OUTPUT);
  pinMode(rlight_relay, OUTPUT);
  pinMode(ylight_relay, OUTPUT);
  digitalWrite(forwardreverse_relay, false);
  ch[forwardreverse_relay] = false;
  digitalWrite(rlight_relay, false);
  ch[rlight_relay] = false;
  digitalWrite(ylight_relay, false);
  ch[ylight_relay] = false;
  nh.initNode();
  nh.subscribe(subestop);
  nh.subscribe(subrlight);
  nh.subscribe(subreverse);
  nh.advertise(pubreverse);
  nh.advertise(pubrlight);
  publishtime = millis() + 50;
  estop_flag = false;
  estop_flag_pre = estop_flag;
  reverse = false;
}
bool reversefunc(bool reverse1) {
  digitalWrite1(forwardreverse_relay, reverse1);
}
bool rlightfunc(int rlight1) {
  static unsigned int i = 0;
  i++;
  if(rlight1 == 0) digitalWrite1(rlight_relay, false);
  if(rlight1 == 1) digitalWrite1(rlight_relay, true);
  if(rlight1 == 2) {
  if(i > 30000) digitalWrite1(rlight_relay, true);
  else digitalWrite1(rlight_relay,false);   
    
  }
  if(i > 60000) i = 0;  
}
bool ylightfunc(int ylight1) {
  if(ylight1 == 0) digitalWrite1(ylight_relay, false);
  if(ylight1 == 1) digitalWrite1(ylight_relay, true);
}

void loop() {
  unsigned long c_millis = millis();
  if(estop_flag == true) {
    ylight = 1;
    rlight = 1;
    reverse = false;
  }
  if (estop_flag_pre == true && estop_flag == false) {
    ylight = 0;
    rlight = 0;
    reverse = false;
  }
  rlightfunc(rlight);
  reversefunc(reverse);
  ylightfunc(ylight);
  estop_flag_pre = estop_flag;
  if (c_millis > publishtime) {
    c_reverse.data = reverse;
    pubreverse.publish(&c_reverse);
    c_rlight.data = rlight;
    pubrlight.publish(&c_rlight);
    publishtime = c_millis + 50;
  }
  nh.spinOnce();
}
