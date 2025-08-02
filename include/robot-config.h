using namespace vex;

extern brain Brain;

//To set up a motor called LeftFront here, you'd use
extern motor LeftFront;
extern motor LeftRear;
extern motor RightFront;
extern motor RightRear;

//Add your devices below, and don't forget to do the same in robot-config.cpp:
extern inertial InertialSensor;
extern distance RightRearDistance;
extern distance LeftRearDistance;
extern rotation RotationRight;
extern rotation RotationRear;

void  vexcodeInit( void );