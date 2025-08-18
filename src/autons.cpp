#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).

  // chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  // chassis.set_heading_constants(6, .4, 0, 1, 0);
  // chassis.set_turn_constants(6, .4, .03, 3, 15);
  // chassis.set_swing_constants(12, .3, .001, 2, 15);

  chassis.set_drive_constants(6.0, 1.5, 0.0, 10.0, 0.0);
  chassis.set_heading_constants(6.0, .35, 0.0, 1.0, 0.0);
  chassis.set_turn_constants(6.0, .35, .01, 1.0, 15.0);
  chassis.set_swing_constants(12.0, 0.75, 0.001, 1.0, 15.0);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  // chassis.heading_max_voltage = 10;
  // chassis.drive_max_voltage = 8;
  chassis.heading_max_voltage = 6;
  chassis.drive_max_voltage = 6;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  chassis.set_coordinates(0, 0, 0);
  this_thread::sleep_for(100);
  printf("Rotation: %f\n", RotationRight.position(turns));

  for (int i = 0; i < 10; i++) {

    default_constants();
    chassis.drive_distance(-12);
    chassis.drive_stop(brake);
    chassis.turn_to_angle(90);
    chassis.turn_to_angle(0);

    this_thread::sleep_for(500);
    printf("Pose: %lf, %lf, %lf\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

    // chassis.drive_distance(12);
    // chassis.drive_stop(brake);
    // this_thread::sleep_for(1000);
    // printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

    odom_constants();
    chassis.drive_to_pose(0, 0, 0);
    chassis.drive_stop(brake);

    this_thread::sleep_for(500);
    printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  }

}

 void drive_test_temp(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test_temp(bool reverse) {
  chassis.turn_to_angle((reverse) ? -90 : 90);
  this_thread::sleep_for(1000);
  printf("heading: %f\n", chassis.get_absolute_heading());

  chassis.turn_to_angle(0);
  this_thread::sleep_for(1000);
  printf("heading: %f\n", chassis.get_absolute_heading());

}

void turn_test_full(bool reverse) {
  chassis.turn_to_angle((reverse) ? -5 : 5);
  this_thread::sleep_for(1000);
  printf("heading: %f\n", chassis.get_absolute_heading());

  chassis.turn_to_angle((reverse) ? -30 : 30);
  this_thread::sleep_for(1000);
  printf("heading: %f\n", chassis.get_absolute_heading());

  chassis.turn_to_angle((reverse) ? -90 : 90);
  this_thread::sleep_for(1000);
  printf("heading: %f\n", chassis.get_absolute_heading());

  chassis.turn_to_angle((reverse) ? -225 : 225);
  this_thread::sleep_for(1000);
  printf("heading: %f\n", chassis.get_absolute_heading());

  chassis.turn_to_angle(0);
  this_thread::sleep_for(1000);
  printf("heading: %f\n", chassis.get_absolute_heading());
}

void turn_test(bool reverse) {
  chassis.turn_to_angle(180);
  chassis.drive_stop(brake);
  this_thread::sleep_for(100);
  printf("heading: %lf\n", chassis.get_absolute_heading());
  printf("ForwardTracker: %lf\n", RotationRight.position(turns));
  printf("SidewaysTracker: %lf\n", RotationRear.position(turns));

  chassis.turn_to_angle(0);
  chassis.drive_stop(brake);
  this_thread::sleep_for(100);
  printf("heading: %lf\n", chassis.get_absolute_heading());
  printf("ForwardTracker: %lf\n", RotationRight.position(turns));
  printf("SidewaysTracker: %lf\n", RotationRear.position(turns));
}



/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.left_swing_to_angle(0);
  chassis.right_swing_to_angle(-90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test_temp(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.turn_to_point(24, 24);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.drive_to_point(24,24);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.drive_to_point(0,0);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.turn_to_angle(0);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
}

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.drive_to_point(0, 24);
  chassis.drive_stop(brake);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24, 24);
  chassis.drive_stop(brake);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.turn_to_point(24, 0);
  chassis.drive_to_point(24, 0);
  chassis.drive_stop(brake);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.turn_to_point(0, 0);
  chassis.drive_to_point(0,0);
  chassis.drive_stop(brake);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.turn_to_angle(0);
  chassis.drive_stop(brake);
  this_thread::sleep_for(500);
  printf("Pose: %f, %f, %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
}


/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}