


//https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/

#include <math.h> 

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

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;
// One-second interval for measurements
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

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
  attachInterrupt(digitalPinToInterrupt(encpinB1), right_wheel_tick, RISING);  //was rising
  attachInterrupt(digitalPinToInterrupt(encpinA1), left_wheel_tick, RISING);

  
  Serial.begin(115200);  //was 9600  115200

  //Serial.println("EncoderA1:,EncoderB1,PWM_A:,PWM_B:,Setpoint_A:,Setpoint_B:");
 
}

void loop() {
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    Serial.println("Number of Ticks: ");
    Serial.println(right_wheel_tick_count);
    Serial.println(left_wheel_tick_count);
    Serial.println();
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

// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(encpinB2);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count == encoder_maximum) {
      right_wheel_tick_count = encoder_minimum;
    }
    else {
      right_wheel_tick_count++;  
    }    
  }
  else {
    if (right_wheel_tick_count == encoder_minimum) {
      right_wheel_tick_count = encoder_maximum;
    }
    else {
      right_wheel_tick_count--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(encpinA2);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count == encoder_maximum) {
      left_wheel_tick_count = encoder_minimum;
    }
    else {
      left_wheel_tick_count++;  
    }  
  }
  else {
    if (left_wheel_tick_count == encoder_minimum) {
      left_wheel_tick_count = encoder_maximum;
    }
    else {
      left_wheel_tick_count--;  
    }   
  }
}
 
