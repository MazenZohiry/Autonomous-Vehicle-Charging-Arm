  /*
 * MotorKnob
 * Modefied by Ahmad Shamshiri for Robojax on July 20, 2019
 * in Ajax, Ontario, Canada
 * Watch video instruction for this code: https://youtu.be/cYTICj4DWYc
 * 
 * A stepper motor follows the turns of a potentiometer
 * (or other sensor) on analog input 0.
 *
 * http://www.arduino.cc/en/Reference/Stepper
 * This example code is in the public domain.
 */

#include <Stepper.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
// change this to the number of steps on your motor
const int StepsPerRev = 20000; //

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(StepsPerRev, 6,7);
ros::NodeHandle nh;



void servo_cb(const sensor_msgs::JointState& cmd_msg){
  Serial.println("clockwise");
  stepper.step(StepsPerRev);
  delay(5000);
}
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  // set the speed of the motor to 30 RPMs
  stepper.setSpeed(50);
  // Initialize serial port
  Serial.begin(57600);
}
void loop() {
//  // step on revolution in one direction
//  Serial.println("clockwise");
//  stepper.step(StepsPerRev);
//  delay(5000);
//
//  // step on revolution in the other direction
//  
//  Serial.println("clockwise");
//  stepper.step(-StepsPerRev);
//  delay(500);
  nh.spinOnce();
  delay(1);

}
