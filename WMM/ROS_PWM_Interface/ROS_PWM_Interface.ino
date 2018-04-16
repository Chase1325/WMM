#include <ros.h>
#include <std_msgs/Float32.h>

// Set up more potentiometers
const int potPin1 = A1;
const int potPin2 = A2;

// Pins for driver controllers
// TODO: Change pins for convenient setup
const int pwmPin1 = 3;
const int dirPin1 = 4;
const int pwmPin2 = 5;
const int dirPin2 = 6;

// TODO: find the range of motion of the rotational base joint
const float JOINT1_ANGLE_MIN = 18;
const float JOINT1_ANGLE_MAX = 280;

const float JOINT2_ANGLE_MIN = 44.6;
const float JOINT2_ANGLE_MAX = 304.6;


ros::NodeHandle nh;

std_msgs::Float32 float_msg1;
std_msgs::Float32 float_msg2;

// ROS Publishers
ros::Publisher joint1_theta("joint1_theta", &float_msg1);
ros::Publisher joint2_theta("joint2_theta", &float_msg2);


// Callback function when receivng a rostopic for theta1
// Move joint1 (rotational arm base) to theta1
void pwmCallback1(const std_msgs::Float32& msg) {
  float pwm = msg.data;
  //pwmWrite(pwmPin1, dirPin1, pwm);
  Serial.print("Received: ");
  Serial.println(pwm);
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

// Function to move motor to desired position/angle
void pwmWrite(int motorPWM, int motorDIR, float pwm) {
  if (pwm >= 0) {
    digitalWrite(motorDIR, HIGH);
  }
  else {
    digitalWrite(motorDIR, LOW);
  }
  analogWrite(motorPin, pwm);
  delay(500);
}


void setup() {
  nh.initNode();
  nh.advertise(joint1_theta);
  nh.advertise(joint2_theta);
  nh.subscribe(pwm);
  //nh.subscribe(pwm_signal2);
}

// Use loop to update the readings of the current motor positions
// and publish them as rostopics for a Python node to subscibe and
// utilize arm kinematics to compute desired angles/positions.
void loop() {

  // TODO: Need to specify joint1 range of motion for potentiometer
  float angle1 = getPotAngle(potPin1);
  float_msg1.data = angle1;
  joint1_theta.publish( &float_msg1 );
  
  /*float angle2 = getPotAngle(potPin2);
  float_msg2.data = angle2;
  joint2_theta.publish( &float_msg2 );*/
  nh.spinOnce();
  delay(500);
}
