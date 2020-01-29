
//import servo library 
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

//startup of the internal node
ros::NodeHandle  nh;

//define the servos
Servo J0;
Servo J1;
Servo J2;


double J0_angle=90;
double J1_angle=90;
double J2_angle=90;

void servo_cb(const sensor_msgs::JointState& cmd_msg){
  //convert servo angles from radians to degrees
  
  J0_angle=radiansToDegrees(cmd_msg.position[0]);
  J1_angle=radiansToDegrees(cmd_msg.position[1]);
  J2_angle=radiansToDegrees(cmd_msg.position[2]);

  //move the servos to angles specified
  
  J0.write(J0_angle);
  J1.write(J1_angle);
  J2.write(J2_angle);
  
}

// setting up the subscriber
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  //define what pins the servos are attached to
  
  J0.attach(2); 
  J1.attach(8); 
  J2.attach(11);


  delay(5);
  J0.write(90);
  J1.write(90);
  J2.write(90);

}

void loop(){
  nh.spinOnce();
}

double radiansToDegrees(float position_radians)
{

  position_radians = position_radians + 1.6;

  return position_radians * 57.2958;

}
