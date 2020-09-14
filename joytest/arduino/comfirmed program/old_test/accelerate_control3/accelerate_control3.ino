/* PULLDOWN REQUIRED and SPEED MEASUREMENT REQUIRED
  acc control & brake control & brake lamp & e-stop
  D0,1,2,3,4,5,6,7,8,9,10,11,12,13,A0,1,2,3,4,5
   x,x,2,x,x,x,x,x,x,x, x, x, ?,13, x,1,2,3,4,5
  --
  Pin assignment
  D3  acc signal PWM
  D4  relay4 estop
  D5  relay3 no connection
  D6  relay2 acc signal
  D7  relay1 acc switch
  D8  brake lamp
  D9  front brake PWM
  D10 rear brake PWM
  D11 acc swtich PWM
  A0  speed measurement    Not implemented
  ---
  Wire assignment
  --E-stop-----
  D4 ----PULLDOWN Normal->LOW Estop->HIGH
  --acc control-------------------------------
  D7---------------|
             --------------
  --- BLACK -|NC relay1   |
  D11--BLUE--|NO       COM|--WHITE-- acc switch
             --------------  (255->STOP, 0->MOVE)

  D6-----------------------|
                     --------------
  --- GREEN ---------|NC relay2   |
  D3-|LOW |--YELLOW--|NO       COM|--RED-- acc signal
     |PASS|          --------------
     R=100k,C=0.33uF
  --front brake control-------------------------------
        -------------------
  D9----|In  H-Bridge  Out|--BLUE--brake signal
  12V---|12V              | (D9 PWM from 0 to 170)
  GND---|GND              |
        -------------------
  --rear brake control-------------------------------
  D10-------|Rear brake|---Linear actuator
                           myservo.writeMicroseconds (1950);//released
                           myservo.writeMicroseconds (1600);//pulled
  --brake lamp--------------------------------------
                   ---------------
  D8----YELLOW-----|Ctl          |
  G-----BLACK------|GND relay    |
  Vcc---RED--------|+5V       COM|------|Brake lamp|
  12V--------------|NO           |
  12V--|Pedal |----|NC           |
       |Switch|    ---------------
  -----Speed measurement ----
  A0---|LOWPASS|----|D3 Speed measurement arduino|
       R=100k,C=0.33uF
  TOPIC in-------
  acc-> Int16 from 0 to 255
  brake-> Int16 from 0 to 170
  TOPIC out---------
  c_acc<- Int16
  c_brake<- Int16
  c_blamp<- Bool
  c_estop<- Bool
*/


#include <Servo.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
ros::NodeHandle nh;
std_msgs::Int16 c_acc; // for debugging
std_msgs::Int16 c_brake; // for debugging
std_msgs::Bool c_blamp; // for debugging
std_msgs::Bool c_estop; // for debugging
ros::Publisher pubacc("c_acc", &c_acc);
ros::Publisher pubbrake("c_brake", &c_brake);
ros::Publisher pubblamp("c_blamp", &c_blamp);
ros::Publisher pubestop("c_estop", &c_estop);

int acc_switch_relay = 7;//  pin07 PD7 on/off relay 1
int acc_signal_relay = 6;//  pin06 PD6 on/off relay 2
int acc_switch_PWM = 11;//  pin11 PD11 PWM 255->STOP,0->MOVE
int acc_signal_PWM = 3;//  pin03 PD3 PWM  SpeedControl
int brake_signal_PWM = 9;// pin09
int estop = 4; ////estop
int brake_lamp = 8;
int rear_brake = 10;
Servo myservo;

int acc = 0;
int brake = 0;
long laptime = 0;

bool blamp = false;

long publishtime = -1;

bool estop_flag = false;
bool estop_flag_pre = false;

void accCb(const std_msgs::Int16 &Acc)
{
  acc = Acc.data;
  if (acc > 255) acc = 255;
  if (acc < 0) acc = 0;
  brake = 0;
  estop_flag = digitalRead(estop);
  if (estop_flag == true) {
    laptime = 0; acc = 0;
    digitalWrite(acc_switch_relay, LOW); // trun off relay1
    digitalWrite(acc_signal_relay, LOW); // trun off relay2
    analogWrite(acc_switch_PWM, 255); // STOP
    analogWrite(acc_signal_PWM, (int)acc);
  } else {
    digitalWrite(acc_switch_relay, HIGH); // trun on relay1
    digitalWrite(acc_signal_relay, HIGH); // trun on relay2
    analogWrite(acc_switch_PWM, 0); // move
    if (acc == 0) laptime = millis();
    else laptime = millis() + 1000;
    analogWrite(acc_signal_PWM, (int)acc);
  }
  c_acc.data = acc;
  pubacc.publish(&c_acc);
  ////////////
  myservo.writeMicroseconds (1950);//released
  analogWrite(brake_signal_PWM, (int)brake);

  blamp = false;
  digitalWrite(brake_lamp, blamp);
  c_blamp.data = blamp;
  pubblamp.publish(&c_blamp);

  c_estop.data = estop_flag;
  pubestop.publish(&c_estop);
  c_brake.data = brake;
  pubbrake.publish(&c_brake);
}
void brakeCb(const std_msgs::Int16 &Brake)
{
  brake = Brake.data;
  if (brake > 170) brake = 170; else if (brake < 0) brake = 0;
  acc = 0;
  estop_flag = digitalRead(estop);
  blamp = true;
  if (estop_flag == true) brake = 170;
  laptime = millis() + 1000;
  if (brake == 0) {
    blamp = false;
    laptime = 0;
    myservo.writeMicroseconds (1950);//released
  }
  analogWrite(brake_signal_PWM, (int)brake);
  digitalWrite(brake_lamp, blamp);
  c_blamp.data = blamp;
  pubblamp.publish(&c_blamp);
  c_estop.data = estop_flag;
  pubestop.publish(&c_estop);
  c_brake.data = brake;
  pubbrake.publish(&c_brake);
  //////////////////
  digitalWrite(acc_switch_relay, LOW); // trun off relay1
  digitalWrite(acc_signal_relay, LOW); // trun off relay2
  analogWrite(acc_switch_PWM, 255); // STOP
  analogWrite(acc_signal_PWM, (int)acc);
  c_acc.data = acc;
  pubacc.publish(&c_acc);
}

ros::Subscriber<std_msgs::Int16> subacc("acc", &accCb);
ros::Subscriber<std_msgs::Int16> subbrake("brake", &brakeCb);
void setup()
{
  acc = 0;
  brake = 0;
  laptime = 0;
  pinMode(estop, INPUT);
  pinMode(acc_switch_relay, OUTPUT); //  pin07 PD7 on/off relay 1
  pinMode(acc_signal_relay, OUTPUT); //  pin06 PD6 on/off relay 2
  pinMode(acc_switch_PWM, OUTPUT); //  pin11 PD11 PWM  HIGH->STOP,LOW->move
  pinMode(acc_signal_PWM, OUTPUT); //  pin03 PD3 PWM  SpeedControl
  analogWrite(acc_switch_PWM, 255); // stop
  analogWrite(acc_signal_PWM, 0); //
  pinMode(brake_signal_PWM, OUTPUT);
  pinMode(brake_lamp, OUTPUT);
  analogWrite(brake_signal_PWM, 0);
  digitalWrite(brake_lamp, LOW);
  myservo.attach(10);
  myservo.writeMicroseconds (1950);//released
  nh.initNode();
  nh.subscribe(subacc);
  nh.advertise(pubacc);
  nh.subscribe(subbrake);
  nh.advertise(pubbrake);
  nh.advertise(pubblamp);
  nh.advertise(pubestop);
}

void loop() {
  if (publishtime == -1) publishtime = millis() + 50;
  estop_flag = digitalRead(estop);
  if (estop_flag == true) {
    acc = 0;
    brake = 170;
    if (laptime == 0) laptime = millis() + 1000;
    digitalWrite(acc_switch_relay, LOW); // trun off relay1
    digitalWrite(acc_signal_relay, LOW); // trun off relay2
    analogWrite(acc_switch_PWM, 255); // STOP
    analogWrite(acc_signal_PWM, (int)acc);
  } else {
    if(laptime == 0) {
      acc = 0;brake = 0;blamp = false;
      digitalWrite(acc_switch_relay, LOW); // trun on relay1
      digitalWrite(acc_signal_relay, LOW); // trun on relay2
      analogWrite(acc_switch_PWM, 0); // move
      analogWrite(acc_signal_PWM, (int)acc);
      analogWrite(brake_signal_PWM, (int) brake);
      myservo.writeMicroseconds (1950);//released
      analogWrite(brake_signal_PWM, (int) brake);
      myservo.writeMicroseconds (1950);//released
    }
  }
  if (estop_flag_pre == true && estop_flag == false) {
    acc = 0;brake = 0;laptime = 0;
    blamp = false;
    digitalWrite(acc_switch_relay, LOW); // trun on relay1
    digitalWrite(acc_signal_relay, LOW); // trun on relay2
    analogWrite(acc_switch_PWM, 0); // move
    analogWrite(acc_signal_PWM, (int)acc);
    analogWrite(brake_signal_PWM, (int) brake);
    myservo.writeMicroseconds (1950);//released
  }
  estop_flag_pre = estop_flag;
  if (laptime != 0) {
    if (brake >=
     0) {
      acc = 0;
      digitalWrite(acc_switch_relay, LOW); // trun off relay1
      digitalWrite(acc_signal_relay, LOW); // trun off relay2
      analogWrite(acc_switch_PWM, 255); // STOP
      analogWrite(acc_signal_PWM, 0);
      blamp = true;
      if (millis() < laptime) {
        analogWrite(brake_signal_PWM, (int) brake);
        myservo.writeMicroseconds (1950);//released
      } else {
        brake = 255;  // brake   ->  255 means rear brake
        analogWrite(brake_signal_PWM, 0);
        myservo.writeMicroseconds (1600);//pulled
      }
    } else if (acc) {
      brake = 0;
      myservo.writeMicroseconds (1950);//released
      analogWrite(brake_signal_PWM, 0);
      blamp = false;
      if (millis() <  laptime) {
        digitalWrite(acc_switch_relay, HIGH); // trun on relay1
        digitalWrite(acc_signal_relay, HIGH); // trun on relay2
        analogWrite(acc_switch_PWM, 0); // move
        analogWrite(acc_signal_PWM, (int)acc);
      } else {
        laptime = 0; acc = 0;
        digitalWrite(acc_switch_relay, HIGH); // trun on relay1
        digitalWrite(acc_signal_relay, HIGH); // trun on relay2
        analogWrite(acc_switch_PWM, 0); // move
        analogWrite(acc_signal_PWM, (int)acc);
      }
    }  else {
      acc = 0;
      brake = 0;
      laptime = 0;
      blamp = false;
      digitalWrite(acc_switch_relay, LOW); // trun on relay1
      digitalWrite(acc_signal_relay, LOW); // trun on relay2
      analogWrite(acc_switch_PWM, 255); // move
      analogWrite(acc_signal_PWM, (int)acc);
      analogWrite(brake_signal_PWM, (int) brake);
      myservo.writeMicroseconds (1950);//released

    }
  }
  
  
  
  digitalWrite(brake_lamp, blamp);
  if (millis() > publishtime) {
    c_estop.data = estop_flag;
    pubestop.publish(&c_estop);
    c_brake.data = brake;
    pubbrake.publish(&c_brake);
    c_blamp.data = blamp;
    pubblamp.publish(&c_blamp);
    c_acc.data = acc;
    pubacc.publish(&c_acc);
    publishtime = -1;
  }
  nh.spinOnce();
}
