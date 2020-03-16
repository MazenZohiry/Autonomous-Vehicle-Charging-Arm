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

// change this to the number of steps on your motor
const int StepsPerRev = 800; //

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(StepsPerRev, 6,7);


void setup() {
  // set the speed of the motor to 30 RPMs
  stepper.setSpeed(200);
  // Initialize serial port
  Serial.begin(57600);
}

void loop() {
  // step on revolution in one direction
  Serial.println("clockwise");
  stepper.step(StepsPerRev);
  delay(200);

  // step on revolution in the other direction
  
  Serial.println("clockwise");
  stepper.step(-StepsPerRev);
  delay(200);

}
