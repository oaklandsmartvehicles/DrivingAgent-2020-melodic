/*
  This code is connect ROS and Arduino to control Step motor

  ROS input value is Vehicle wheel's angle: -30<angle<30

  tmp=[];
  for steering = -30:10:30; % from -30 to 30
  for c_steering =-28:10:28;% from -30 to 30
    if(steering > 28) steering =  28;end
    if(steering < -28) steering = -28;end
    diff_steering = steering - c_steering;
    tmp=[tmp;[steering,c_steering,diff_steering]];
    c_steering = steering;
  end
  end
  tmp

*/

#include <math.h>
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
std_msgs::Float32 c_steering; // open loop current steering
ros::Publisher pub("c_steering", &c_steering);

int stepsPerSecond = 200;
int CP_plus = 9;
int CW_plus = 8;

void messageCb(const std_msgs::Float32 &steering)
{
  float steer = steering.data;
  if (steer > 28) steer =  28;
  if (steer < -28) steer = -28;
  float diff_steering = steer - c_steering.data;
  c_steering.data = steer;
  rotate1(diff_steering);
  pub.publish(&c_steering);
}
ros::Subscriber<std_msgs::Float32> sub("steering", &messageCb);
void setup()
{
  pinMode(CP_plus, OUTPUT);
  pinMode(CW_plus, OUTPUT);
  digitalWrite(CW_plus, HIGH);
  c_steering.data = 0.0;

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}
void loop() {
  nh.spinOnce();
  delay(1);
}
void rotate1(float steering) {
  int pulse = (int)steering * (400. / 360.) * (360. / (37.5 / 4)) ; ///wheel angle
  if (steering > 0.) {
    digitalWrite(CW_plus, HIGH);
    for (int i = 0; i < pulse; i++) {
        nh.spinOnce();
      digitalWrite(CP_plus, LOW);
   delay(1);         //////////////////////motor speed
      digitalWrite(CP_plus, HIGH);
     delay(1); 
    }
  } else {
    digitalWrite(CW_plus, LOW);
    for (int i = 0; i < -pulse; i++) {
        nh.spinOnce();
      digitalWrite(CP_plus, HIGH);
      delay(1); 
      digitalWrite(CP_plus, LOW);
      delay(1); 
    }
  }
}
