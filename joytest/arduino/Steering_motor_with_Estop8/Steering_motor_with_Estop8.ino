/*
  Steering motor & potentiometer & e-stop
  D0,1,2,3,4,5,6,7,8,9,10,11,12,13,A0,1,2,3,4,5
   x,x,2,3,x,5,6,7,x,x,10,11,12,13, x,1,2,3,4,5
  --
  Pin assignment
  D8 cw_plus dir, D9 cp_plus pulse
  D4 estop,A0 potentiometer
  ---
  Wire assignment
  --E-stop-----
  D4 ----PULLUP Normal->LOW Estop->HIGH
  --Steering motor-------------------------------
          --------------
  D8------|dir          |  left rotation plus
  D9------|pulse        |  right rotation minus
          ---------------
  ----Potentio meter----------
              ---------------
  5V---RED----|Potentiometer|  left rotation zero
  GND--BLACK--|             |  right rotation plus
  A0---GREEN--|             |
              ---------------
  ------
  TOPIC in-------
  steering-> Float32
  TOPIC out---------
  c_steering <- Float32
*/

#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
std_msgs::Float32 c_steering;
ros::Publisher pub("c_steering", &c_steering);

#define USEPOTENTIOMETER 0           /// 0 not use potentionmeter/// 1 use potentiometer

int CP_plus = 9;
int CW_plus = 8;
int estop = 4;
int steer_angle = A0; // from potentiometer(steering)
float steering_target = 0.0;
bool steering_topic = false;
int steering_dir = 1;;
int steering_count = 0;
void steeringCb(const std_msgs::Float32 &steering)
{
  steering_target = steering.data;
  if (steering_target > 28.) steering_target =  28.;
  else if (steering_target < -28.) steering_target = -28.;
  float diff_steering;
#if !(USEPOTENTIOMETER)
  diff_steering = steering_target - c_steering.data;
#else
  c_steering.data = -(analogRead(steer_angle) - 512.0) / 512.0 * 40;
  diff_steering = steering_target - c_steering.data;
#endif
  int pulse = (int)diff_steering * (400. / 360.) * (360. / (37.5 / 4)) ; ///wheel angle need calculate
  if (diff_steering > 0.) {
    steering_dir = 1;
    steering_count = pulse * 1.5;
  } else {
    steering_dir = -1;
    steering_count = - pulse * 1.5;
  }
  steering_topic = true;
  pub.publish(&c_steering);
}
ros::Subscriber<std_msgs::Float32> sub("steering", &steeringCb);
unsigned long publishtime = 0;

bool estop_flag = false;
bool estop_flag_pre = false;
void setup()
{
  pinMode(CP_plus, OUTPUT);
  pinMode(CW_plus, OUTPUT);
  pinMode(estop, INPUT);
  pinMode(steer_angle, INPUT);
  digitalWrite(CW_plus, HIGH);
  c_steering.data = 0.0;
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  publishtime = millis() + 50;
  estop_flag = digitalRead(estop);
  estop_flag_pre = estop_flag;
}

int delaytime = 1;
void loop() {
  unsigned long c_millis = millis();
  estop_flag = digitalRead(estop);
  if(estop_flag == true){
    steering_dir = 0;
    steering_count = 0;
    steering_topic = false;
  }
  if (steering_topic == true) {
    if (steering_dir > 0) digitalWrite(CW_plus, true);
    if (steering_dir < 0) digitalWrite(CW_plus, false);
    delaytime = 5;
    steering_topic = false;
  }
  if (steering_count > 0) {
    digitalWrite(CP_plus, LOW);
    delay(delaytime);     //motor speed
    digitalWrite(CP_plus, HIGH);
//    delay(delaytime);
    if(delaytime < 1) delaytime = 1;
    steering_count--;
#if(USEPOTENTIOMETER)
    c_steering.data = -(analogRead(steer_angle) - 512.0) / 512.0 * 40;
#else
    c_steering.data += (steering_dir / ((400. / 360.) * (360. / (37.5 / 4))));
#endif
    if (abs(c_steering.data - steering_target) < 0.03) {
      steering_count = 0;
      steering_dir = 0;
    }
  }
  estop_flag_pre = estop_flag;
  if (c_millis > publishtime) {
    pub.publish(&c_steering);
    publishtime = c_millis + 50;
  }
  nh.spinOnce();
}
