//Scott Scheraga  7/18/2021

//PID control
//https://www.element14.com/community/community/arduino/blog/2020/01/06/simple-arduino-dc-motor-control-with-encoder-part-2
// You have to click refresh than immediately hit the X button to stop the page from being locked.

//Tick counter

//https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/


#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <math.h> 
ros::NodeHandle  nh;



int motorApin1 = 2;
int motorApin2 = 3;
int motorBpin1 = 4;
int motorBpin2 = 5;

int motorAenable= 10;  //was 9
int motorBenable= 9;  //was 10

int encpinA1= 18;
int encpinA2= 19;

int encpinB1= 20;
int encpinB2= 21;

double enc1 = 0 ;
double enc2 = 0 ;
double enc3 = 0 ;
double enc4 = 0 ;

//Tick counter variables
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

// PWM
//const uint16_t ANALOG_WRITE_BITS = 8;
//const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
//const uint16_t MIN_PWM = MAX_PWM / 4;    // Make sure motor turns
int MIN_PWM=0;
int MAX_PWM=255;  //was 240
// Motor timing
unsigned long nowTime = 0;       // updated on every loop
unsigned long startTimeA = 0;    // start timing A interrupts
unsigned long startTimeB = 0;    // start timing B interrupts
unsigned long countIntA = 0;     // count the A interrupts
unsigned long countIntB = 0;     // count the B interrupts
double periodA = 0;              // motor A period
double periodB = 0;              // motor B period
// PID 
const unsigned long SAMPLE_TIME = 50;  // time between PID updates. Was 100
const unsigned long INT_COUNT = 20;     // sufficient interrupts for accurate timing

//DEFINITIONS
//Input: The variable we're trying to control (double)---current pwm
//Output: The variable that will be adjusted by the pid (double)  ---modified pwm
//Setpoint: The value we want to Input to maintain (double) ----speed

double  inputA= 0;             // input is rotational speed in Hz
double  outputA= 0;              // output is PWM to motors
double setpointA = 0;         // setpoint is rotational speed in Hz

double  inputB= 0;             // input is rotational speed in Hz
double  outputB= 0;              // output is PWM to motors
double setpointB = 0;         // setpoint is rotational speed in Hz

double KpA = .5, KiA = 2, KdA = 0.1;  //started with 0.05, 0.03,0   //Kp=2 occilates Set to 1.  Ki set to 0.5 starts high, taks a while to fall 1 is good 2 better
double KpB = .5, KiB = 2, KdB = 0.1; //  1/2/0.1 is good  .5/2/0.1 has slower rise time, but a lot smoother

//step 1, I set P values  to 4 so it occilates around the setpoint
PID motorA(&inputA, &outputA, &setpointA, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, &setpointB, KpB, KiB, KdB, DIRECT);



//ROS 
//https://atadiat.com/en/e-rosserial-arduino-introduction/

double v_r=0, v_l=0;
int motorAdirection=1;
int motorBdirection=1;

double wheel_radius = 0.028;
double wheel_separation = 0.219;   // this may be slightly incorrect. Very rough measurement. 

double speed_angular=0, speed_linear=0;

void messageCb( const geometry_msgs::Twist& msg){
  speed_angular = msg.angular.z;
  speed_linear = msg.linear.x;
  v_r = (((speed_linear*2) + (speed_angular*wheel_separation))/(2.0*wheel_radius));  //radians per second
  v_l = (((speed_linear*2) - (speed_angular*wheel_separation))/(2.0*wheel_radius));  //radians per second
  if (v_r>0){
    motorAdirection=1; }
  else if (v_r<0){
    motorAdirection=-1;}
  else if (v_r==0){
    motorAdirection=0;}
    
  if (v_l>0){
    motorBdirection=1; }
  else if (v_l<0){
    motorBdirection=-1;}
  else if(v_l==0){
    motorBdirection=0;} 

  setpointA=fabs(v_r*75/(2*PI));  // encoder ticks per sec
  setpointB=fabs(v_l*75/(2*PI));  // encoder ticks per sec

  
}
//1:75 gear ratio. 2 encoder pulses per turn. 150 pulses per rotation
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
//ros::Publisher p("funbot_motor_speeds", &test);



void setup() {
  // put your setup code here, to run once:
  pinMode(motorApin1, OUTPUT);
  pinMode(motorApin2, OUTPUT);
  pinMode(motorBpin1, OUTPUT);
  pinMode(motorBpin2, OUTPUT);
  
  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT);
  //Serial.print("input speed 0-255");

  pinMode(encpinA1, INPUT_PULLUP);
  pinMode(encpinA2, INPUT);
  pinMode(encpinB1, INPUT_PULLUP);
  pinMode(encpinB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encpinA1), isr_A, RISING);  //was rising
  attachInterrupt(digitalPinToInterrupt(encpinB1), isr_B, RISING);

  
  Serial.begin(115200);  //was 9600  115200
  nh.getHardware()->setBaud(115200);
  startTimeA = millis();
  startTimeB = millis();
  motorA.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorB.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorA.SetSampleTime(SAMPLE_TIME);
  motorB.SetSampleTime(SAMPLE_TIME);
  motorA.SetMode(AUTOMATIC);
  motorB.SetMode(AUTOMATIC);
  //Serial.println("EncoderA1:,EncoderB1,PWM_A:,PWM_B:,Setpoint_A:,Setpoint_B:");
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  
}

void loop() {
  nh.spinOnce();
  nowTime = millis();
  currentMillis = millis();  //duplicate of above line, but this makes sense because I am bodging 2 scripts together. 

  
  motorA.Compute(); //part of pid library
  motorB.Compute();
  //motormove(1, motorAdirection, outputA);
  //motormove(2,motorBdirection,outputB); 
  
  if (outputA>20){
  motormove(1, motorAdirection, outputA);  //was   (int)outputA
  }
  else{
  digitalWrite(motorApin1, LOW);
  digitalWrite(motorApin2, LOW);            
  analogWrite(motorAenable, 0); 
  }
  if (outputB>20){
  motormove(2,motorBdirection,outputB);  //was (int)outputB
  }
  else{
  digitalWrite(motorBpin1, LOW);
  digitalWrite(motorBpin2, LOW);            
  analogWrite(motorBenable, 0); 
  }

  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    //nh.spinOnce();  //I hope commenting this line doesn't break the code
  }
  
  
  /*
  Serial.print(inputA); 
  Serial.print(","); 
  Serial.print(inputB),
  Serial.print(","); 
  Serial.print(outputA); 
  Serial.print(","); 
  Serial.print(outputB),
  Serial.print(","); 
  Serial.print(setpointA); 
  Serial.print(","); 
  Serial.println(setpointB);
  */
  
}
void motormove(int16_t motor, int16_t motordirection, int16_t pwm){
  //int pwmInt = round(pwm); 
  int motorpin1=0;
  int motorpin2=0;
  int motorenable=0;
  if(motor == 1){ //pin setting
    motorpin1=motorApin1;
    motorpin2=motorApin2;
    motorenable=motorAenable;
    }
  if(motor == 2){
    motorpin1=motorBpin1;  
    motorpin2=motorBpin2;
    motorenable=motorBenable;
    }
  if(motordirection == 1){
      digitalWrite(motorpin1, LOW); 
      digitalWrite(motorpin2, HIGH);
      analogWrite(motorenable, pwm);
    }
  else if(motordirection == -1){
      digitalWrite(motorpin1, HIGH);
      digitalWrite(motorpin2, LOW);   
      analogWrite(motorenable, pwm);   
    }
  else if(motordirection == 0){
     digitalWrite(motorpin1, LOW);
     digitalWrite(motorpin2, LOW);
     analogWrite(motorenable, 0);            
    }
   
  }

void isr_A(){
  //PID ticks per second
  // count sufficient interrupts to get accurate timing
  // inputX is the encoder frequency in Hz
  countIntA++;
  if (countIntA == INT_COUNT){
    inputA = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeA);
    startTimeA = nowTime;
    countIntA = 0;
  }

  //tick counter  
  // Read the value for the encoder for the left wheel
  int val = digitalRead(encpinA2);
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
 
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
void isr_B(){
  //PID loop-related, ticks per second
  countIntB++;
  if (countIntB == INT_COUNT){
    inputB = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeB);
    startTimeB = nowTime;
    countIntB = 0;
  }

  //tick counter
  // Read the value for the encoder for the right wheel
  int val = digitalRead(encpinB2);
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
  
}
 
