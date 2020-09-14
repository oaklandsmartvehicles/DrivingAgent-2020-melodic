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

  close all
  t= 0:1/5000:5;                   % 10 seconds @ 5kHz sample rate
  fo=10;f1=300;                    % Start at 10Hz, go up to 400Hz
  y=(chirp(t,fo,2,f1,'logarithmic')>0)*1024;
  figure;plot(t,y);
  y=y+100*rand(size(y));
  figure
  spectrogram(y,256,200,256,1000);
  figure
  out = [];
  i = 1;
  while(1)
  c_millis = t(i)*1000 + 100;
  p_val = y(i);
  i = i + 1;
  count = 0;
  acount = 0;
  while(1)
    c_val = y(i);
    if(c_val - p_val > 200) count = count +1;end
    if(p_val - c_val > 200) count = count +1;end
    p_val = c_val;
    acount = acount + 1;
    if(c_millis < t(i)*1000) break;end
    i = i + 1;
    if(i >= length(t)) break;end
  end
  out = [out, (count+1)/acount];
  i = i + 1;
  if(i >= length(t)) break;end
  end
  plot(out)



*/
//#define DEBUG
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
int SPDcount = A0;
std_msgs::Float32 c_speed;
std_msgs::Float32 c_debug1;
std_msgs::Float32 c_debug2;
ros::Publisher speed_pub("c_speed", &c_speed);

#ifdef DEBUG
ros::Publisher debug1_pub("c_debug1", &c_debug1);
ros::Publisher debug2_pub("c_debug2", &c_debug2);
#endif

int i = 0;
int value[3];
float pcount;
float fcount;
void setup()
{
  nh.initNode();
  nh.advertise(speed_pub);

#ifdef DEBUG
  nh.advertise(debug1_pub);
  nh.advertise(debug2_pub);
#endif

  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;
  value[0] = analogRead(SPDcount);
  value[1] = value[0];
  value[2] = value[1];
  i = 0;
  fcount = 0;
  pcount = 0;
}
int c_val;
int p_val;
unsigned long c_millis;
unsigned long n_millis;
unsigned long c_time;
unsigned long c_millis1[3];
unsigned long ud_duration[2];
unsigned int count;
int ud;
float duration;
void loop() {
  p_val = analogRead(SPDcount);
  count = 0;
  ud = 0;
  duration = 0.0;
  n_millis = millis() + 50;
  i = 0;
  while (1) {
    c_millis = millis();
    value[i] = analogRead(SPDcount);
    c_millis1[i] = c_millis;
    i++;
    if (i > 2) i = 0;
    c_time = (c_millis1[0] + c_millis1[1] + c_millis1[2]) / 3.0;
    if ((value[0] >= value[1]) && (value[1] > value[2])) c_val = value[1];
    if ((value[0] >= value[2]) && (value[2] > value[1])) c_val = value[2];
    if ((value[1] >= value[0]) && (value[0] > value[2])) c_val = value[0];
    if ((value[1] >= value[2]) && (value[2] > value[0])) c_val = value[2];
    if ((value[2] >= value[0]) && (value[0] > value[1])) c_val = value[0];
    if ((value[2] >= value[1]) && (value[1] > value[0])) c_val = value[1];
    if (c_val - p_val > 500) {
      ud_duration[ud] = c_time;
      count++;
      ud++;
    }
    if (p_val - c_val > 500) {
      ud_duration[ud] = c_time;
      count++;
      ud++;
    }
    if (ud > 1) {
      duration += (float)(ud_duration[1] - ud_duration[0]);
      ud_duration[0] = ud_duration[1];
      ud = 1;
    }
    p_val = c_val;
    if (n_millis < c_millis) break;
  }
  if (count > 1) {
    float total = (float)(c_millis - n_millis + 50);
    float tmp = (total - duration) * (float)(count - 1) / duration;
    if (tmp > 1) tmp = 1;
    fcount = (float)(count - 1) + tmp;
  } else fcount = (float)count;
  c_speed.data = fcount;
  speed_pub.publish(&c_speed);
#ifdef DEBUG
  debug1_pub.publish(&c_debug1);
  debug2_pub.publish(&c_debug2);
#endif
  nh.spinOnce();
}
