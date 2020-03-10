
/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * BCN3D Moveo robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) joint_steps is computed from the simulation in PC and sent Arduino via rosserial.  It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached 
 * 
 * Publishing to the following ROS topics: joint_steps_feedback
 *    1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
 *       accurately
 *       
 * Author: Jesse Weisberg
 */
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>

#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <sensor_msgs/JointState.h>

// Joint 1
#define E1_STEP_PIN        7
#define E1_DIR_PIN         6
#define E1_ENABLE_PIN      30

// Joint 2
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

// Joint 3
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

// Joint 4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// Joint 5 
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

AccelStepper joint1(1,E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint2(1,Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint3(1,Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint4(1,X_STEP_PIN, X_DIR_PIN);
AccelStepper joint5(1, E0_STEP_PIN, E0_DIR_PIN);

Servo gripper;
MultiStepper steppers;

int joint_step[6];
int joint_status = 0;

ros::NodeHandle nh;
std_msgs::Int16 msg;

//instantiate publisher (for debugging purposes)
//ros::Publisher steps("joint_steps_feedback",&msg);

void servo_cb(const sensor_msgs::JointState& cmd_msg){
  joint_status = 1;
  joint_step[0] = cmd_msg.position[0];
  joint_step[1] = cmd_msg.position[1];
  joint_step[2] = cmd_msg.position[2];
  joint_step[3] = cmd_msg.position[3];
  joint_step[4] = cmd_msg.position[4];
  joint_step[5] = cmd_msg.position[5]; //gripper position <0-180>
}


//instantiate subscribers
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);
//subscribes to joint_states on arm
//to publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>

void setup() {
  //put your setup code here, to run once:
  //Serial.begin(57600);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  joint_status = 1;

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(steps);

  // Configure each stepper
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(750);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(1000);

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);

  
}

void loop() {
  if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[5];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
    positions[1] = -joint_step[1]; 
    positions[2] = joint_step[2]; 
    positions[3] = joint_step[3]; 
    positions[4] = -joint_step[4]; 

    // Publish back to ros to check if everything's correct
    //msg.data=positions[4];
    //steps.publish(&msg);

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
  }

  joint_status = 0;
  
  nh.spinOnce();
  delay(1);
  
}
