#include <ros.h>
#include <std_msgs/Float32.h>

// Pins for potentiometers
const int potPin1 = A1;
const int potPin2 = A2;

// Pins for three motor drivers
const int dirPin1 = 4;
const int pwmPin1 = 5;
const int dirPin2 = 6;
const int pwmPin2 = 7;
const int dirPin3 = 8;
const int pwmPin3 = 9;

#define ENCODER_A 2
#define ENCODER_B 3
volatile int rpmcount = 0;
const int COUNTS_PER_REV = 4225;

const float JOINT1_ANGLE_MIN = 18;
const float JOINT1_ANGLE_MAX = 280;
const float JOINT2_ANGLE_MIN = 44.6;
const float JOINT2_ANGLE_MAX = 304.6;


ros::NodeHandle nh;

std_msgs::Float32 float_msg1;
std_msgs::Float32 float_msg2;
std_msgs::Float32 float_msg3;
std_msgs::Float32 rpm_msg;

std_msgs::Float32 test_msg1;
std_msgs::Float32 test_msg2;
std_msgs::Float32 test_msg3;

float angle1, angle2, angle3;


// ROS Publishers
ros::Publisher joint1_theta("joint1_theta", &float_msg1);
ros::Publisher joint2_theta("joint2_theta", &float_msg2);
ros::Publisher joint3_theta("joint3_theta", &float_msg3);
ros::Publisher rpm_data("rpmcount", &float_msg2);

ros::Publisher testing1("testing1", &test_msg1);
ros::Publisher testing2("testing2", &test_msg2);
ros::Publisher testing3("testing3", &test_msg3);


// Callback function when receivng a rostopic for pwm1
// Write pwm1 to joint1 (rotational arm base)
void pwmCallback1(const std_msgs::Float32& msg) {
  float pwm = msg.data;
  pwmWrite(pwmPin1, dirPin1, pwm);
  testing1.publish(&msg);
}

// Callback function when receivng a rostopic for pwm2
// Write pwm2 to joint2 (arm elbow motor)
void pwmCallback2(const std_msgs::Float32& msg) {
  float pwm = msg.data;
  pwmWrite(pwmPin2, dirPin2, pwm);
  testing2.publish(&msg);
}

// Callback function when receivng a rostopic for pwm3
// Write pwm3 to joint3 (end effector motor)
void pwmCallback3(const std_msgs::Float32& msg) {
  float pwm = msg.data;
  pwmWrite(pwmPin3, dirPin3, pwm);
  testing3.publish(&msg);
}

// ROS Subscribers to pwm signal 1, 2, and 3 topics
ros::Subscriber<std_msgs::Float32> pwm1("pwm1", &pwmCallback1 );
ros::Subscriber<std_msgs::Float32> pwm2("pwm2", &pwmCallback2 );
ros::Subscriber<std_msgs::Float32> pwm3("pwm3", &pwmCallback3 );


// Function to read in potentiometer values from pin and convert to value
float getPotAngle(int potPin) {
  int value;
  float angle;
  value = analogRead(potPin);
  if (potPin == A1) {
    angle = map(value, 0, 1023, JOINT1_ANGLE_MIN, JOINT1_ANGLE_MAX);
  }
  if (potPin == A2) {
    angle = map(value, 0, 1023, JOINT2_ANGLE_MIN, JOINT2_ANGLE_MAX);
  }

  return angle;
}


float getEncoderAngle() {
  float value, angle;
  if (rpmcount >= 0) {
    value = rpmcount % COUNTS_PER_REV;
    angle = map(value, 0, COUNTS_PER_REV, 0, 360);
  }
  else {
    value = rpmcount % COUNTS_PER_REV;
    angle = map(value, 0, -1*COUNTS_PER_REV, 0, -360); 
  }
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
  pwm = abs(pwm);
  analogWrite(motorPWM, pwm);
}


void setup() {
  nh.initNode();
  nh.advertise(joint1_theta);
  nh.advertise(joint2_theta);
  nh.advertise(joint3_theta);
  nh.advertise(rpm_data);
  nh.advertise(testing1);
  nh.advertise(testing2);
  nh.advertise(testing3);
  nh.subscribe(pwm1);
  nh.subscribe(pwm2);
  nh.subscribe(pwm3);
  
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

  attachInterrupt(0, rpm_motor, FALLING);
}

// Use loop to update the readings of the current motor positions
// and publish them as rostopics for a Python node to subscibe and
// utilize arm kinematics to compute desired angles/positions.
void loop() {

  angle1 = getPotAngle(potPin1);
  float_msg1.data = angle1;
  joint1_theta.publish( &float_msg1 );
  
  angle2 = getPotAngle(potPin2);
  float_msg2.data = angle2;
  joint2_theta.publish( &float_msg2 );

  angle3 = getEncoderAngle();
  float_msg3.data = angle3;
  joint3_theta.publish( &float_msg3 );

  rpm_msg.data = rpmcount;
  rpm_data.publish( &rpm_msg );

  nh.spinOnce();
}


void rpm_motor(){
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
