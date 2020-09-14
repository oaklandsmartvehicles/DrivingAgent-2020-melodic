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
float steer = 0.0;
void steeringCb(const std_msgs::Float32 &steering)
{
  steer = steering.data;
  if (steer > 28) steer =  28;
  else if (steer < -28) steer = -28;
  float diff_steering = 0;
#if !(USEPOTENTIOMETER)
  diff_steering = steer - c_steering.data;
#else
  c_steering.data = -(analogRead(steer_angle) - 512.0) / 512.0 * 40;
  diff_steering = steer - c_steering.data;
#endif
  rotate1(diff_steering);
  pub.publish(&c_steering);
}
ros::Subscriber<std_msgs::Float32> sub("steering", &steeringCb);
unsigned long publishtime = 0;

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
}
bool rotstart = false;
int rotdir = false;
int rotcount = 0;
bool estop_flag = false;
int delaytime = 10;
void loop() {
  unsigned long c_millis = millis();
  estop_flag = digitalRead(estop);
  if (estop_flag == true) {
    rotstart = false;
    rotdir = 0;
    rotcount = 0;
  } else {
    if (rotstart == true) {
      if (rotdir > 0) {
        digitalWrite(CW_plus, true);
        delaytime = 10;
      }
      else if (rotdir < 0) {
        digitalWrite(CW_plus, false);
        delaytime = 10;
      }
      rotstart = false;
    }
    if (rotcount) {
      digitalWrite(CP_plus, LOW);
      delay(delaytime--);     //motor speed
      digitalWrite(CP_plus, HIGH);
      delay(delaytime--);
      if(delaytime < 1) delaytime = 1;
      rotcount--;
#if(USEPOTENTIOMETER)
      c_steering.data = -(analogRead(steer_angle) - 512.0) / 512.0 * 40;
#else
      c_steering.data += (rotdir / ((400. / 360.) * (360. / (37.5 / 4))));
#endif
      if (abs(c_steering.data - steer) < 0.03) {
        rotcount = 0;
        rotdir = 0;
      }
      if (rotcount < 0) {
        rotcount = 0;
        rotdir = 0;
      }
    }
  }
  if (c_millis > publishtime) {
    pub.publish(&c_steering);
    publishtime = c_millis + 50;
  }
  nh.spinOnce();
}
void rotate1(float steering) {
  int pulse = (int)steering * (400. / 360.) * (360. / (37.5 / 4)) ; ///wheel angle need calculate
  rotstart = true;
  if (steering > 0.) {
    rotdir = 1;
    rotcount = pulse * 1.5;
  } else {
    rotdir = -1;
    rotcount = -pulse * 1.5;
  }
}
