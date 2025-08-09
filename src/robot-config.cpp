#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//The motor constructor takes motors as (port, ratio, reversed), so for example
motor LeftFront = motor(PORT2, ratio6_1, true);
motor LeftRear = motor(PORT4, ratio6_1, true);
motor RightFront = motor(PORT1, ratio6_1, false);
motor RightRear = motor(PORT3, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:
inertial InertialSensor = inertial(PORT10);
distance RightRearDistance = distance(PORT9);
distance LeftRearDistance = distance(PORT8);
myrotation RotationRight = myrotation(PORT19, 10, true);
myrotation RotationRear = myrotation(PORT20, 10, true);

void vexcodeInit( void ) {
  this_thread::sleep_for(100); // Sleep for a short time to allow devices to initialize
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    this_thread::sleep_for(100);
  }
  RotationRight.resetPosition();
  RotationRear.resetPosition();
  // nothing to initialize
}