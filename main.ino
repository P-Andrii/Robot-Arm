/*
This code controls a 2 segment robot arm with a gripper compromised of 4 stepper motors with an Arduino. 
The whole arm can move radially by rotating the base stepper motor. 
The controlling is done via 2 rotary potentiometers and 2 buttons. 
  */
#include <Servo.h>
#define dirPin 2
#define stepPin 3
Servo s1,s2,s3,s4;
float horiz;
float high;
int alpha;
int beta;
boolean closed;
int seg_1 = 75;
int seg_2 = 50;

void stepper(){
  // Simplistic control for a stepper motor: if the "Turn Left" button is on: move the stepper one step left, if button "Turn Right" is on: turn one step right
  if (digitalRead(A2) == 1){
    digitalWrite(dirPin,1);
    digitalWrite(stepPin,1);
    delayMicroseconds(1);
    digitalWrite(stepPin,0);
    }
  if (digitalRead(A3) == 1){
    digitalWrite(dirPin,0);
    digitalWrite(stepPin,1);
    delayMicroseconds(1);
    digitalWrite(stepPin,0);
    }
}

void gripper(){
  // By pressing both the "Turn Left" and "Turn Right" buttons at the same time the state of the gripper can be changed from closed to open and vice versa
  if (digitalRead(A2) == 1 & digitalRead(A3) == 1){
    closed =! closed;
    delay(100);
  }
  if(closed == 0){
    s4.write(130);
  }
  if (closed == 1){
    s4.write(170);
  }
  delay(5);
}

void inverse_kinematics(float horiz, float high, int& alpha, int& beta) {
  float hypotenuse = sqrt(horiz*horiz + high*high);
  // The angle for the first servo motor is divided into 2: the angle between the horizontal and the line between the base and the claw, and the angle between the said line and the first segment of the arm
  // The first angle alpha1 is found through the tangent of the input parameters
  // The second angle alpha2 is found through the rule of cosines
  float alpha1 = 57.3 * atan(high/horiz);
  float alpha2 = 57.3 * acos( (hypotenuse*hypotenuse + seg_1*seg_1 - seg_2*seg_2) / (2*seg_1*hypotenuse) );
  alpha = alpha1 + alpha2;
  // Beta is the angle between the first and second segments of the arm
  beta = 57.3 * acos( (seg_1*seg_1 + seg_2*seg_2 - hypotenuse*hypotenuse)/(2*seg_1*seg_2) );
}

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // Servo setup and reset to default position
  s1.attach(4);
  s2.attach(5);
  s3.attach(6);
  s4.attach(7);
  s1.write(90);
  s2.write(90);
  s3.write(90);
  s4.write(90);
}

void loop() {
  // Update the horizontal and vertical distances from the arm base from the potentiometer positions
  horiz = horiz*3/4 + analogRead(A1)/4;
  high = high*3/4 + analogRead(A0)/4;
  inverse_kinematics(horiz, high, alpha, beta);
  
  s1.write(alpha);
  // Sanity check: if trying to go underground - don't
  if(beta<=0){
    delay(50);
  } 
  else{
    s2.write(180-beta);
    s3.write(270 - beta - alpha);
  }
  stepper();
  gripper();
}
