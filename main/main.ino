#include <ros.h>
#include <std_msgs/Float32.h>>

// code for arduino nano
// uses 3 potentiometers and one ultrasonic sensor to read de position of the arm
// The target position for each joint is read from the topic /arm_pose_target

// define the pins for the potentiometers
#define pot1 A0
#define pot2 A1
#define pot3 A2

// define the pins for the ultrasonic sensor
#define trigPin 2
#define echoPin 3

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

// gains for the PID controller on each joint
#define Kp1 0.1
#define Ki1 0.01
#define Kd1 0.01

#define Kp2 0.1
#define Ki2 0.01
#define Kd2 0.01

#define Kp3 0.1
#define Ki3 0.01
#define Kd3 0.01

#define Kp4 0.1
#define Ki4 0.01
#define Kd4 0.01

// initialy the joint angles are set to 0 and height to 0
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float height = 0;

// initialize variables for the PID controller and control signals
float error1 = 0;
float error2 = 0;
float error3 = 0;
float error4 = 0;
float error1_old = 0;
float error2_old = 0;
float error3_old = 0;
float error4_old = 0;
float error1_sum = 0;
float error2_sum = 0;
float error3_sum = 0;
float error4_sum = 0;
float u1 = 0;
float u2 = 0;
float u3 = 0;
float u4 = 0;

// control for the gripper, 0 is open and 1 is closed
float gripper = 0;

// status leds
/* #define led1 0
#define led2 1
bool led1_state = true;
bool led2_state = false; */

ros::NodeHandle nh;

// on hearing a rosmessage of type ArmPose print the values to the serial monitor
/* void printMessage(const unity_msgs::ArmPose& msg) {
  Serial.print("q1: ");
  Serial.print(msg.q1);
  Serial.print(" q2: ");
  Serial.print(msg.q2);
  Serial.print(" q3: ");
  Serial.print(msg.q3);
  Serial.print(" d4: ");
  Serial.print(msg.d4);
  Serial.print(" q5: ");
  Serial.print(msg.q5);
  Serial.print(" succ: ");
  Serial.println(msg.succ);
  nh.loginfo("Nueva posicion recibida");
} */

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

// function to read the potentiometers and return the angle in degrees
float readPot(int pin) {
  float angle = map(analogRead(pin), 0, 1023, 0, 180);
  return angle;
}

// function to read the ultrasonic sensor and return the distance in meters
float readUltrasonic() {
  float duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

// function to calculate the control signal for the PID controller
float PID(float error, float error_old, float error_sum, float Kp, float Ki, float Kd) {
  float u = Kp * error + Ki * error_sum + Kd * (error - error_old);
  return u;
}

// function to convert the control signal to a step signal for the stepper motor
int step(float u) {
  int step = 0;
  if (u > 0) {
    step = 1;
  }
  else if (u < 0) {
    step = -1;
  }
  return step;
}

// function to set the direction of the stepper motor
void setDir(int dir, int pin) {
  if (dir == 1) {
    digitalWrite(pin, HIGH);
  }
  else if (dir == -1) {
    digitalWrite(pin, LOW);
  }
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

  // set the pins for the stepper motors as outputs
  pinMode(step1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(step2, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(step3, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(step4, OUTPUT);
  pinMode(dir4, OUTPUT);

  // set the pins for the status leds as outputs
  //pinMode(led1, OUTPUT);
  //pinMode(led2, OUTPUT);

  // set the pin for the emergency stop as input
  pinMode(stop, INPUT);

  // set the pins for the ultrasonic sensor as outputs and inputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // set the pins for the potentiometers as inputs
  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);
  pinMode(pot3, INPUT);

  // blink the status leds for 1 second
  nh.loginfo("Arduino inicializado");
}

void loop() {
  //nh.loginfo("LoopStart");
  // run the ros node
  nh.spinOnce();
  // delay (of about 50-250)
  //delay(10);

  // read the potentiometers and ultrasonic sensor
  float pot1_angle = readPot(pot1);
  float pot2_angle = readPot(pot2);
  float pot3_angle = readPot(pot3);
  float ultrasonic = readUltrasonic();

  // log the potentiometer angles and ultrasonic sensor
  //nh.loginfo("Pot1: " + String(pot1_angle));
  //nh.loginfo("Pot2: " + String(pot2_angle));
  //nh.loginfo("Pot3: " + String(pot3_angle));
  //nh.loginfo("Ultrasonic: " + String(ultrasonic));
  
  // calculate the errors for the PID controller
  error1 = theta1 - pot1_angle;
  error2 = theta2 - pot2_angle;
  error3 = theta3 - pot3_angle;
  error4 = height - ultrasonic;

  // calculate the control signals for the PID controller
  u1 = PID(error1, error1_old, error1_sum, Kp1, Ki1, Kd1);
  u2 = PID(error2, error2_old, error2_sum, Kp2, Ki2, Kd2);
  u3 = PID(error3, error3_old, error3_sum, Kp3, Ki3, Kd3);
  u4 = PID(error4, error4_old, error4_sum, Kp4, Ki4, Kd4);

  // calculate the step signals for the stepper motors
  int step1_signal = step(u1);
  int step2_signal = step(u2);
  int step3_signal = step(u3);
  int step4_signal = step(u4);

  // set the direction of the stepper motors
  setDir(step1_signal, dir1);
  setDir(step2_signal, dir2);
  setDir(step3_signal, dir3);
  setDir(step4_signal, dir4);

  // set the step signal for the stepper motors
  digitalWrite(step1, step1_signal);
  digitalWrite(step2, step2_signal);
  digitalWrite(step3, step3_signal);
  digitalWrite(step4, step4_signal);

  // update the errors for the PID controller
  error1_old = error1;
  error2_old = error2;
  error3_old = error3;
  error4_old = error4;

  // update the error sums for the PID controller
  error1_sum += error1;
  error2_sum += error2;
  error3_sum += error3;
  error4_sum += error4;

  // check if the emergency stop is pressed
  /*if (digitalRead(stop) == HIGH) {
    // stop the stepper motors
    digitalWrite(step1, LOW);
    digitalWrite(step2, LOW);
    digitalWrite(step3, LOW);
    digitalWrite(step4, LOW);

    // turn on the status leds
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);

    // wait for the emergency stop to be released
    while (digitalRead(stop) == HIGH) {
      nh.spinOnce();
    }

    nh.logfatal("Emergency stop pressed!");

    // turn off the status leds
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
  } */
  //nh.loginfo("LoopEnd");
}
