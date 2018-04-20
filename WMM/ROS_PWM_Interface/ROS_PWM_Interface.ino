#include <ros.h>
#include <std_msgs/Float32.h>

// Set up more potentiometers
const int potPin1 = A1;
const int potPin2 = A2;

// Pins for driver controllers
// TODO: Change pins for convenient setup
const int pwmPin1 = 4;
const int dirPin1 = 5;
const int pwmPin2 = 6;
const int dirPin2 = 7;
const int pwmPin3 = 8;
const int dirPin3 = 9;

#define ENCODER_A 2
#define ENCODER_B 3
volatile int rpmcount = 0

// TODO: find the range of motion of the rotational base joint
const float JOINT1_ANGLE_MIN = 18;
const float JOINT1_ANGLE_MAX = 280;

const float JOINT2_ANGLE_MIN = 44.6;
const float JOINT2_ANGLE_MAX = 304.6;


ros::NodeHandle nh;

std_msgs::Float32 float_msg1;
std_msgs::Float32 float_msg2;
std_msgs::Float32 float_msg3;
float angle1, angle2, angle3;


// ROS Publishers
ros::Publisher joint1_theta("joint1_theta", &float_msg1);
ros::Publisher joint2_theta("joint2_theta", &float_msg2);
ros::Publisher joint2_theta("joint3_theta", &float_msg3);
ros::Publisher testing("testing", &float_msg2);


// Callback function when receivng a rostopic for theta1
// Move joint1 (rotational arm base) to theta1
void pwmCallback1(const std_msgs::Float32& msg) {
  float pwm = msg.data;
  //pwmWrite(pwmPin1, dirPin1, pwm);
  //Serial.print("Received: ");
  //Serial.println(pwm);
  testing.publish(&msg);
  //delay(100);
}

// Callback function when receivng a rostopic for theta1
// Move joint2 (arm elbow motor) to theta2
/*void pwmCallback2(const std_msgs::Float32& msg) {
  float pwm = msg.data;
  pwmWrite(pwmPin2, dirPin2, pwm);
}*/

// Callback function when receivng a rostopic for theta1
// Move joint3 (end effector motor) to theta3
// TODO" Look into encoder of end effector motor since it doesn't use potentiometer
/*void pwmCallback3(const std_msgs::Float32& msg) {
  float pwm = msg.data;
  pwmWrite(pwmPin3, dirPin3, pwm);
}*/

// ROS Subscribers to topics theta 1, 2, and 3
ros::Subscriber<std_msgs::Float32> pwm("pwm", &pwmCallback1 );
//ros::Subscriber<std_msgs::Float32> pwm_signal("pwm_signal2", &pwmCallback2 );
//ros::Subscriber<std_msgs::Int32> state_sub("state", &stateCallback );
// TODO: create subscriber for end effector desired joint angles (joint 3)

// Function to read in potentiometer values from pin and convert to value
float getPotAngle(int potPin) {
  int value;
  float angle;
  value = analogRead(potPin);
  // TODO: need to check that A1, A2 can be used in if statements
  if (potPin == A1) {
    angle = map(value, 0, 1023, JOINT1_ANGLE_MIN, JOINT1_ANGLE_MAX);
  }
  if (potPin == A2) {
    angle = map(value, 0, 1023, JOINT2_ANGLE_MIN, JOINT2_ANGLE_MAX);
  }

  return angle;
}


float getEncoderAngle() {
  float value = rpmcount % COUNTS_PER_REV;
  float angle = map(value, 0, COUNTS_PER_REV, 0, 360);
  return angle;
}


// Function to move motor to desired position/angle
void pwmWrite(int motorPWM, int motorDIR, float pwm) {
  if (pwm >= 0) {
    digitalWrite(motorDIR, HIGH);
  }
  else {
    digitalWrite(motorDIR, LOW);
  }
  analogWrite(motorPWM, pwm);
  delay(500);
}


void setup() {
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(joint1_theta);
  nh.advertise(joint2_theta);
  nh.advertise(testing);
  nh.subscribe(pwm);
  //nh.subscribe(pwm_signal2);
  
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(potPin1, INPUT);
  pinMode(potPin2, INPUT);

  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);

  attachInterrupt(0, rpm_motor, RISING);
}

// Use loop to update the readings of the current motor positions
// and publish them as rostopics for a Python node to subscibe and
// utilize arm kinematics to compute desired angles/positions.
void loop() {

  // TODO: Need to specify joint1 range of motion for potentiometer
  angle1 = getPotAngle(potPin1);
  float_msg1.data = angle1;
  joint1_theta.publish( &float_msg1 );
  
  angle2 = getPotAngle(potPin2);
  float_msg2.data = angle2;
  joint2_theta.publish( &float_msg2 );

  angle3 = getEncoderAngle();
  float_msg3.data = angle3;
  joint3_theta.publish( &float_msg3 );

  
  nh.spinOnce();
  //delay(100);
}


void rpm_motor(){
  INTFLAG1 = 1; 
  if (digitalRead(ENCODER_A) == HIGH) {
    if (digitalRead(ENCODER_B) == LOW) {
      rpmcount++;
    } else {
      rpmcount--;
    }
  } else {
    if (digitalRead(ENCODER_B) == LOW) {
      rpmcount--;
    } else {
      rpmcount++;
    }
  }
}
