
#define SPEEDSENSOR 0

#if(SPEEDSENSOR)
#include <SPI.h>
#endif

#include <math.h>
#include <ros.h>
#include <std_msgs/Int16.h>
ros::NodeHandle nh;
std_msgs::Int16 c_acc; // for debugging
ros::Publisher pub("c_acc", &c_acc);

int acc_switch_relay = 7;//  pin07 PD7 on/off relay 1
int acc_signal_relay = 6;//  pin06 PD6 on/off relay 2
int acc_switch_PWM = 11;//  pin11 PD11 PWM  Enable/Disable
int acc_signal_PWM = 3;//  pin03 PD3 PWM  SpeedControl
int Estop = 4; ////Estop
int acc = 0;

#if(SPEEDSENSOR)
int chipSelectPin1=10;// speed_sensor1
int chipSelectPin2=9;
int chipSelectPin3=8;
#endif

void messageCb(const std_msgs::Int16 &Acc)
{
  acc = Acc.data;
  if (acc > 100) acc = 100;;
  if (acc < 0) acc = 0;
  accfunc(acc);
  c_acc.data = acc;
  pub.publish(&c_acc);
}
ros::Subscriber<std_msgs::Int16> sub("acc", &messageCb);
void setup()
{
  pinMode(acc_switch_relay, OUTPUT); //  pin07 PD7 on/off relay 1
  pinMode(acc_signal_relay, OUTPUT); //  pin06 PD6 on/off relay 2
  pinMode(acc_switch_PWM, OUTPUT); //  pin11 PD11 PWM  Enable/Disable
  pinMode(acc_signal_PWM, OUTPUT); //  pin03 PD3 PWM  SpeedControl
  pinMode(Estop, OUTPUT); //////Estop
 // TCCR2B = (TCCR2B & 0b11111000)|0x07;
  #if(SPEEDSENSOR)
  pinMode(chipSelectPin1,OUTPUT);// speed sensor1
  #endif
  digitalWrite(Estop, HIGH);
  analogWrite(acc_switch_PWM, HIGH); //
  analogWrite(acc_signal_PWM, LOW); //
#if(SPEEDSENSOR)
  digitalWrite(chipSelectPin1,HIGH);// speed sensor1
  LS7366_Init();
#endif
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}
void loop() {
  nh.spinOnce();
  delay(1);
}
void accfunc(int acc) {
  if (digitalRead(Estop)) {
    if(acc > 0) {
      digitalWrite(acc_switch_relay, HIGH); // trun on relay1
      digitalWrite(acc_signal_relay, HIGH); // trun on relay2
      analogWrite(acc_switch_PWM, 0); // 
      analogWrite(acc_signal_PWM, (int)(acc * 200.0 / 100.0)); // 
    } else {
      digitalWrite(acc_switch_relay, LOW); // trun on relay1
      digitalWrite(acc_signal_relay, LOW); // trun on relay2
      analogWrite(acc_switch_PWM, 255); // stop
      analogWrite(acc_signal_PWM, 0); // stop
      }
  } else {
    digitalWrite(acc_switch_relay, LOW); // trun on relay1
    digitalWrite(acc_signal_relay, LOW); // trun on relay2
  }
}
#if(SPEEDSENSOR)
//*****************************************************  
long getEncoderValue(int encoder)
//*****************************************************
{
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    selectEncoder(encoder);
    
     SPI.transfer(0x60); // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    
    deselectEncoder(encoder);
   
    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func

//*************************************************
void selectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,LOW);
        break;
     case 2:
       digitalWrite(chipSelectPin2,LOW);
       break;
     case 3:
       digitalWrite(chipSelectPin3,LOW);
       break;    
  }//end switch
  
}//end func

//*************************************************
void deselectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,HIGH);
        break;
     case 2:
       digitalWrite(chipSelectPin2,HIGH);
       break;
     case 3:
       digitalWrite(chipSelectPin3,HIGH);
       break;    
  }//end switch
  
}//end func



// LS7366 Initialization and configuration
//*************************************************
void LS7366_Init(void)
//*************************************************
{
   
    
    // SPI initialization
    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);
   
   digitalWrite(chipSelectPin1,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin1,HIGH); 
   
   
   digitalWrite(chipSelectPin2,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin2,HIGH); 
   
   
   digitalWrite(chipSelectPin3,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin3,HIGH); 
   
}//end func

#endif
