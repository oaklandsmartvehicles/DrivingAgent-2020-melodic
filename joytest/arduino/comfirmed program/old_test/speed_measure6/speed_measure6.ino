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
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
//int SPDcount = 12;
int SPDcount = A0;
std_msgs::Float32 c_speed;
ros::Publisher pub("c_speed", &c_speed);
float speed = 0;
float out = 0.0;
int c_val = 0;
int p_val = 0;
int value[3];
int i = 0;
void setup()
{
  nh.initNode();
  nh.advertise(pub);
  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;
  out = 5037;// 100ms
  for (int i;i < 3;i++) value[i] = 0.0;
}
unsigned long c_millis = 0;
unsigned long count = 0;
unsigned long acount = 0;

void loop() {
  c_millis = millis() + 50;
  p_val   = analogRead(SPDcount);
  count = 0;
  acount = 0;//5037
  i = 0;
  while(1) {
    value[i++] = analogRead(SPDcount);
    if(i > 3) i = 0;
    if(value[0] >= value[1] && value[1] > value[2]) c_val = value[1];
    if(value[0] >= value[2] && value[2] > value[1]) c_val = value[2];
    if(value[1] >= value[0] && value[0] > value[2]) c_val = value[0];
    if(value[1] >= value[2] && value[2] > value[0]) c_val = value[2];
    if(value[2] >= value[0] && value[0] > value[1]) c_val = value[0];
    if(value[2] >= value[1] && value[1] > value[0]) c_val = value[1];
    if(c_val - p_val > 500) count++;
    if(p_val - c_val > 500) count++;
    p_val = c_val;
    acount++;
    if(c_millis < millis()) break;
  }
  out = 0.9 * out + 0.1 * 1000.0 * (float)(count + 1.0) / (float) acount;
  c_speed.data = (float)count;//spd[count];
  pub.publish(&c_speed);
  nh.spinOnce();
}
