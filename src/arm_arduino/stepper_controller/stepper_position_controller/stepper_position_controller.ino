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
 * 
 * rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

 */

#include <Stepper.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
// change this to the number of steps on your motor

float pi=3.14;
int joint_step[6];
const long StepsPerRev = 2748*4; //
//2748*4
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper1(StepsPerRev, 6,7);
ros::NodeHandle nh;



void servo_cb(const sensor_msgs::JointState& cmd_msg){
  //float part = cmd_msg.position[0]/(2*pi);
  joint_step[0] = cmd_msg.position[0]/(2*pi)*StepsPerRev;
  joint_step[1] = cmd_msg.position[1]/(2*pi)*StepsPerRev;
  joint_step[2] = cmd_msg.position[2]/(2*pi)*StepsPerRev;
  joint_step[3] = cmd_msg.position[3]/(2*pi)*StepsPerRev;
  joint_step[4] = cmd_msg.position[4]/(2*pi)*StepsPerRev;
  joint_step[5] = cmd_msg.position[5]/(2*pi)*StepsPerRev; //gripper position <0-180>
  
  Serial.println("clockwise");
  stepper1.step(joint_step[0]);
  
  delay(5000);
}
ros::Subscriber<sensor_msgs::JointState> sub("arduino_cmd", servo_cb);

void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  // set the speed of the motor to 30 RPMs
  stepper1.setSpeed(30);
  // Initialize serial port
  Serial.begin(57600);
}
void loop() {
//  // step on revolution in one direction
//  Serial.println("clockwise");
//  stepper1.step(StepsPerRev);
//  delay(5000);
//
//  // step on revolution in the other direction
//  
//  Serial.println("clockwise");
//  stepper1.step(-StepsPerRev);
//  delay(500);
  nh.spinOnce();
  delay(1);

}
