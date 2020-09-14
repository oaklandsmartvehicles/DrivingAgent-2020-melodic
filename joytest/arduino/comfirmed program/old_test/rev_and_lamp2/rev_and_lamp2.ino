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
  --E-stop-----
  D3 ----PULLDOWN Normal->LOW Estop->HIGH
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
  estop   -> Bool
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
int estop = 8;
unsigned long publishtime;
bool reverse = false;
int ylight = 0;
int rlight = 0;
bool estop_on = false;
bool estop_flag = false;
bool estop_flag_pre = false;
bool ch[]={false,false,false,false,false,false,false,false,false,false,false,false,false,false};
void digitalWrite1(int num,bool val) {
  if(val != ch[num]) {
    digitalWrite(num,val);
    ch[num] = val;
  }
}
bool reverse_topic = false;
bool rlight_topic = false;
bool ylight_topic = false;
int rlight1 = 0;
int ylight1 = 0;
bool reverse1 = false;

void reverseCb(const std_msgs::Bool &Reverse)
{
  reverse = Reverse.data;
  reverse_topic = true;
  c_reverse.data = reverse;
  pubreverse.publish(&c_reverse);

  ylight_topic = true;
  ylight = (int) reverse;
}
void rlightCb(const std_msgs::Int16 &Rlight)
{
  rlight = Rlight.data;
  rlight_topic = true;
  c_rlight.data = rlight;
  pubrlight.publish(&c_rlight);
}
void ylightCb(const std_msgs::Int16 &Ylight)
{
  ylight = Ylight.data;
}
ros::Subscriber<std_msgs::Int16> subrlight("rlight", &rlightCb);
ros::Subscriber<std_msgs::Int16> subylight("ylight", &ylightCb);
ros::Subscriber<std_msgs::Bool> subreverse("reverse", &reverseCb);
void setup()
{
  pinMode(estop, INPUT);
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
  nh.subscribe(subrlight);
  nh.subscribe(subreverse);
  nh.advertise(pubreverse);
  nh.advertise(pubrlight);
  publishtime = millis() + 50;
  estop_flag = digitalRead(estop);
  estop_flag_pre = estop_flag;
  estop_on == estop_flag;
  if (estop_on == true) {
    reverse1 = false;
    rlight1 = 1;
  } else {
    reverse1 = false;
    rlight1 = 0;
    ylight1 = 0;
  }
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
  static int i = 0;
  i++;
  if(ylight1 == 0) digitalWrite1(ylight_relay, false);
  if(ylight1 == 1) digitalWrite1(ylight_relay, true);
}

void loop() {
  unsigned long c_millis = millis();
  estop_flag = digitalRead(estop);
  if(estop_flag_pre == false && estop_flag == true) {
    estop_on = true;
  }
  if(estop_on == true) {
    ylight1 = 0;
    rlight1 = 0;
    reverse1 = false;
    ylight_topic = false;
    rlight_topic = false;
    reverse_topic = false;
  }
  if (estop_flag_pre == true && estop_flag == false) {
    ylight1 = 0;
    rlight1 = 0;
    reverse1 = false;
    estop_on = false;
  }
  if(reverse_topic == true) {
    reverse1 = reverse;
    ylight1 = ylight;
    reverse_topic = false;
  }
  if(rlight_topic == true) {
    rlight1 = rlight;
    rlight_topic = false;
  }
  reversefunc(reverse1);
  rlightfunc(rlight1);
  ylightfunc(ylight1);
  if (c_millis > publishtime) {
    c_reverse.data = reverse1;
    pubreverse.publish(&c_reverse);
    c_rlight.data = rlight1;
    pubrlight.publish(&c_rlight);
    publishtime = c_millis + 50;
  }
  nh.spinOnce();
}
