#include <util/atomic.h>
#include <Wire.h>
#include"TimerOne.h"
#define ENCA 2// YELLOW
#define ENCB 3 // WHITE
#define Di2 6
#define Di3 7
#define sample_accelerPeriod 1
#define d2p 4.75
//#define maxsteering 180//must be 50

int target = 0;
int setPoint = 0;
int pos=0;
unsigned long long curr=0;
unsigned long long prev=0;
volatile int posi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;
void setMotor(int dir,int pwr,int D2,int D3);
int flag_u=0;
signed int data=0;

int recieved2=0;
char ch = 'n';
int steer ='A';
void brake ()
{
  digitalWrite(Di2, HIGH);
  digitalWrite(Di3, HIGH);    
}
void stop ()
{
  digitalWrite(Di2, LOW);
  digitalWrite(Di3, LOW);     
}
void ISR_timerone()
{
 if(target < setPoint)
  {
    target++;
  }else if(target > setPoint)
  {
    target--;
  }
}
void setup() {
  Serial.begin(2000000);
 // Wire.begin(8);
 //Wire.onReceive(receiveEvent); // register event
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(Di2,OUTPUT);
  pinMode(Di3,OUTPUT);
  //Serial.println("target pos");
  analogWrite(Di2,255);
  //delay(300);
  Timer1.initialize(sample_accelerPeriod*900); // set timer for 10ms
  Timer1.attachInterrupt(ISR_timerone); // Enable the timer
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB),readEncoder_B,RISING);
  
  
}




void loop() {

  float kp =8.95692;// 2.72;
  float ki = 6.9774;//10.71;

  float kd =1;// 0.455;//0.32;//0.5
  
  
  /*pot_Read=analogRead(SETPOINT_POT_PIN);
  target=map(pot_Read,0,1023,0,390);
  ki=(analogRead(KI_POT_PIN)*5.0)/1023.0;
  kp=(analogRead(KP_POT_PIN)*5.0)/1023.0;
  kd=(analogRead(KD_POT_PIN)*5.0)/1023.0;*/
      // Serial.println("Potentiometer control enabled");
  if(Serial.available()>0)
  {
     String recieved = Serial.readStringUntil('\n');
        //Serial.println("first:",recieved);
     recieved2=recieved.toInt();
    // recieved2=constrain(recieved2,-90,90);
     //recieved2=map(recieved2,-90,90,0,180); 

  }
  

/*
    Serial.print("Position: ");
    Serial.print(pos/4.3333);
    Serial.print("  Set Point: ");
    Serial.println(target/4.3333);*/
 
  
  
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;


   pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
   
  }
 // setPoint=setPoint*(1560.0/360.0);
 //pos=pos*(360.0/390);
  // error
 /* if(flag_u==0)
  {*/
    flag_u=0;

    // recieved=map(recieved,-60,60,-90,90);
   Serial.println("third:");
    Serial.println(recieved2);
       setPoint=recieved2*d2p;
      // delay(2000);
    
    
//  }
while(pos>1760)
 {
  brake();
    stop ();
 }
  int e =  pos - target  ;//setPoint
 
  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
    

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  // signal the motor
  setMotor(dir,pwr,Di2,Di3);

  // store previous error
  eprev = e;

}
void setMotor(int dir, int pwmVal, int D2, int D3){
  static byte direction=1;
  if(dir != direction){
    direction=dir;
   // brake();
    stop ();
  // delay(10);
  }
    
  if(dir == 1){
    analogWrite(Di2,pwmVal);
    digitalWrite(Di3,LOW);
  }
  else if(dir == -1){
    analogWrite(Di3,pwmVal);
    digitalWrite(Di2,LOW);
  }
  else{
    stop ();
    //delay(10);
  }  
}

void readEncoder(){
  static char flag=0;
  int b;
  if(flag==0)
  {
    flag=1;
    attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,FALLING);
    b = digitalRead(ENCB);
    if(b == 0){
      posi++;
    }
    else{
      posi--;
    }
  }else if(flag==1)
  {
    flag=0;
    attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
    b = digitalRead(ENCB);
    if(b ==1){
      posi++;
    }
    else{
      posi--;
    }
  }
}

void readEncoder_B(){
  static char flag=0;
  int b;
  if(flag==0)
  {
    flag=1;
    attachInterrupt(digitalPinToInterrupt(ENCB),readEncoder_B,FALLING);
    b = digitalRead(ENCA);
    if(b == 1){
      posi++;
    }
    else{
      posi--;
    }
  }else if(flag==1)
  {
    flag=0;
    attachInterrupt(digitalPinToInterrupt(ENCB),readEncoder_B,RISING);
    b = digitalRead(ENCA);
    if(b ==0){
      posi++;
    }
    else{
      posi--;
    }
  }
}
