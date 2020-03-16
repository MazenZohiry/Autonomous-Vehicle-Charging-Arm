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
#include <std_msgs/Float32MultiArray.h>
// change this to the number of steps on your motor

float pi=3.14;
int joint_step[6];
int counter;
const long StepsPerRev = 800; //
//2748*4
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper1(StepsPerRev, 6,7);
Stepper stepper2(StepsPerRev, 8,9);

Stepper stepper3(StepsPerRev, 10,11);
Stepper stepper4(StepsPerRev, 12,13);

Stepper stepper5(StepsPerRev, 14,15);
Stepper stepper6(StepsPerRev, 16,17);

ros::NodeHandle nh;



void servo_cb(const std_msgs::Float32MultiArray& cmd_msg){

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);    
//  stepper1.step(StepsPerRev);
  for (counter=0;counter <= cmd_msg.layout.data_offset;counter=counter+6)
    {
      joint_step[0] = cmd_msg.data[counter]/(2*pi)*StepsPerRev;
//      joint_step[1] = cmd_msg.data[counter+1]/(2*pi)*StepsPerRev;
//      joint_step[2] = cmd_msg.data[counter+2]/(2*pi)*StepsPerRev;
//      joint_step[3] = cmd_msg.data[counter+3]/(2*pi)*StepsPerRev;
//      joint_step[4] = cmd_msg.data[counter+4]/(2*pi)*StepsPerRev;
//      joint_step[5] = cmd_msg.data[counter+5]/(2*pi)*StepsPerRev; //gripper position <0-180>

      stepper1.step(joint_step[0]);
      stepper2.step(cmd_msg.data[counter+1]/(2*pi)*StepsPerRev);
      stepper3.step(cmd_msg.data[counter+2]/(2*pi)*StepsPerRev);
      stepper4.step(cmd_msg.data[counter+3]/(2*pi)*StepsPerRev);
      stepper5.step(cmd_msg.data[counter+4]/(2*pi)*StepsPerRev);
      stepper6.step(cmd_msg.data[counter+5]/(2*pi)*StepsPerRev);
    }
    
  
  
  delay(5000);
}
ros::Subscriber<std_msgs::Float32MultiArray> sub("arduino_cmd", servo_cb);

void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(LED_BUILTIN, OUTPUT);
  // set the speed of the motor to 30 RPMs
  stepper1.setSpeed(30);
  stepper2.setSpeed(30);
  stepper3.setSpeed(30);
  stepper4.setSpeed(30);
  stepper5.setSpeed(30);
  stepper6.setSpeed(30);
  
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
