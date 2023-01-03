#include <ros.h>
#include <std_msgs/Float32.h>>
#include <AccelStepper.h>

// code for arduino nano
// uses ultrasonic sensor to read the height of the arm
// The target position for each joint is read from the topics /arm_pose_target_re/q1, /arm_pose_target_re/q2, /arm_pose_target_re/q3, /arm_pose_target_re/d0, /arm_pose_target_re/succ
float maxSpeed = 1000; // max speed for the stepper motors
float maxAccel = 500; // max acceleration for the stepper motors

// every motor has its own gear ratio
float ratio1 = 1;
float ratio2 = 1; // corresponds to how may rotations of the motor are needed to move 1m up or down
float ratio3 = 1;
float ratio4 = 1;

// the number of steps per revolution for each motor
float stepsPerRev1 = 200;
float stepsPerRev2 = 200;
float stepsPerRev3 = 200;
float stepsPerRev4 = 200;

// define the pins for the 4 stepper motors
#define step1 4
#define dir1 5
#define step2 6
#define dir2 7
#define step3 8
#define dir3 9
#define step4 10
#define dir4 11

// define pin for emergency stop
#define stop 12

// init stepper motors
AccelStepper stepper1(1, step1, dir1);
AccelStepper stepper2(1, step2, dir2);
AccelStepper stepper3(1, step3, dir3);
AccelStepper stepper4(1, step4, dir4);

// initialy the joint angles are set to 0 and height to 0
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float height = 0;

// control for the gripper, 0 is open and 1 is closed
float gripper = 0;

ros::NodeHandle nh;

// functions that takes a target in degrees and converts it to steps
// using the gear ratio and the number of steps per revolution
long targetToSteps(float target, float ratio, float stepsPerRev) {
  // steps should be (target * ratio * stepsPerRev) / 360 but we need to convert to long
  double stepsDouble = (target * ratio * stepsPerRev) / 360;
  // we need to round the number of steps to the nearest integer
  long steps = round(stepsDouble);
  return steps;
}

// function that returns the number of steps needed to move the ar to the target height
// using the gear ratio and the number of steps per meter
long heightToSteps(float target, float ratio, float stepsPerMeter) {
  // the height is converted to meters
  return (target * ratio * stepsPerMeter);
}



// function to read arm pose from the topic /arm_pose_target and set the target angles and height
void readE0Target(const std_msgs::Float32& msg) {
  theta1 = msg.data;
  char log_msg[35];
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(msg.data, 6, 2, result); // Leave room for too large numbers!
  sprintf(log_msg,"Nuevo target para E0: %sdeg", result);
  nh.loginfo(log_msg);
}

void readE1Target(const std_msgs::Float32& msg) {
  theta2 = msg.data;
  char log_msg[35];
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(msg.data, 6, 2, result); // Leave room for too large numbers!
  sprintf(log_msg,"Nuevo target para E1: %sdeg", result);
  nh.loginfo(log_msg);
}

void readE2Target(const std_msgs::Float32& msg) {
  theta3 = msg.data;
  char log_msg[35];
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(msg.data, 6, 2, result); // Leave room for too large numbers!
  sprintf(log_msg,"Nuevo target para E2: %sdeg", result);
  nh.loginfo(log_msg);
}

void readE3Target(const std_msgs::Float32& msg) {
  height = msg.data;
  char log_msg[35];
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(msg.data, 6, 2, result); // Leave room for too large numbers!
  sprintf(log_msg,"Nuevo target para E3: %sm", result);
  nh.loginfo(log_msg);
}

void readE4Target(const std_msgs::Float32& msg) {
  gripper = msg.data;
  char log_msg[35];
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(msg.data, 6, 2, result); // Leave room for too large numbers!
  sprintf(log_msg,"Grip force: %s", result);
  nh.loginfo(log_msg);
}

// create a subscriber to the topic "arm_pose_target" of type ArmPose
ros::Subscriber<std_msgs::Float32> sub1("arm_pose_target_re/q1", &readE0Target);
ros::Subscriber<std_msgs::Float32> sub2("arm_pose_target_re/q2", &readE1Target);
ros::Subscriber<std_msgs::Float32> sub3("arm_pose_target_re/q3", &readE2Target);
ros::Subscriber<std_msgs::Float32> sub4("arm_pose_target_re/d0", &readE3Target);
ros::Subscriber<std_msgs::Float32> sub5("arm_pose_target_re/succ", &readE4Target);

void setup() {
  //Serial.begin(57600);
  nh.loginfo("Setup Start");
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.loginfo("Inicializando Arduino");

  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAccel);
  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAccel);
  stepper3.setMaxSpeed(maxSpeed);
  stepper3.setAcceleration(maxAccel);
  stepper4.setMaxSpeed(maxSpeed);
  stepper4.setAcceleration(maxAccel);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);

  // set the pins for the status leds as outputs
  //pinMode(led1, OUTPUT);
  //pinMode(led2, OUTPUT);

  // set the pin for the emergency stop as input
  pinMode(stop, INPUT);

  // set the pins for the ultrasonic sensor as outputs and inputs
  /* pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT); */


  // blink the status leds for 1 second
  nh.loginfo("Arduino inicializado");
}

void loop() {
  //nh.loginfo("LoopStart");
  // run the ros node
  nh.spinOnce();
  // delay (of about 50-250)
  //delay(10);

  stepper1.moveTo(targetToSteps(theta1, ratio1, stepsPerRev1));
  stepper2.moveTo(heightToSteps(height, ratio2, stepsPerRev2));
  stepper3.moveTo(targetToSteps(theta2, ratio3, stepsPerRev3));
  stepper4.moveTo(targetToSteps(theta3, ratio4, stepsPerRev4));

  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();

}