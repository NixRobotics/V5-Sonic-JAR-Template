#include "vex.h"

/**
 * Setter method for tracker center distances.
 * The forward tracker center distance is the horizontal distance from the 
 * center of the robot to the center of the wheel the sensor is measuring.
 * The sideways tracker center distance is the vertical distance from the 
 * center of the robot to the center of the sideways wheel being measured.
 * If there's really no sideways wheel we set the center distance to 0 and
 * pretend the wheel never spins, which is equivalent to a no-drift robot.
 * 
 * @param ForwardTracker_center_distance A horizontal distance to the wheel center in inches.
 * @param SidewaysTracker_center_distance A vertical distance to the wheel center in inches.
 */

 #ifdef ODOM_PLAYBACK
int playback_buffer_len = 317;
double playback_buffer[500][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,359.99994},
  {0,0,359.99994},
  {0,0,359.9997},
  {0,0,359.9997},
  {0,0,359.999655},
  {0,0,359.999655},
  {0,0,359.999839},
  {0,0,359.999839},
  {0,0,0.000043},
  {0,0,0.000043},
  {0,0,0.000043},
  {0,0,0.000306},
  {0,0,0.000306},
  {0,0,0.001097},
  {0,0,0.001784},
  {0,0,0.001784},
  {0,0,0.002695},
  {0,0,0.002695},
  {0,0,0.002321},
  {0,0,0.002321},
  {0,0,0.002321},
  {0,0,0.002211},
  {0,0,0.00292},
  {0,0,0.00292},
  {0,0,0.00292},
  {0,0,0.002703},
  {0,0,0.002703},
  {0,0,0.002411},
  {0,0,0.002432},
  {0,0,0.002432},
  {0,0,0.00193},
  {0,0,0.00193},
  {0,0,0.00193},
  {0,0,0.002143},
  {0,0,0.00239},
  {0,0,0.00239},
  {0,0,0.002007},
  {0,0,0.002007},
  {0,0,0.001864},
  {0,0,0.001864},
  {0,0,0.001864},
  {0,0,0.001711},
  {0,0,0.001621},
  {0,0,0.001621},
  {0,0,0.001322},
  {0,0,0.001322},
  {0,0,0.001103},
  {0,0,0.001103},
  {0,0,0.001103},
  {0,0,0.000746},
  {0,0,0.000746},
  {0,0,0.000592},
  {0,0,0.001037},
  {0,0,0.001037},
  {0,0,0.000695},
  {0,0,0.000695},
  {0,0,0.000695},
  {0,0,0.000681},
  {0,0,0.000681},
  {0,0,0.001186},
  {0,0,0.001186},
  {0,0,0.001182},
  {0,0,0.001182},
  {0,0,0.001639},
  {0,0,0.001183},
  {0,0,0.001183},
  {0,0,359.999988},
  {0,0,359.999988},
  {0,0,359.999988},
  {0,0,359.999373},
  {0,0,359.999189},
  {0,0,359.999189},
  {0,0,359.999189},
  {0,0,359.998674},
  {0,0,359.99889},
  {0,0,359.99889},
  {0,0,0.000097},
  {0,0,0.000097},
  {0,0,359.999675},
  {0,0,359.999675},
  {0,0,0.000375},
  {0,0,0.000375},
  {0,0,0.001165},
  {0,0,0.001165},
  {0,0,0.00207},
  {0,0,0.00207},
  {0,0,0.002133},
  {0,0,0.002133},
  {0,0,0.002156},
  {0,0,0.002156},
  {0,0,0.001707},
  {0,0,0.001707},
  {0,0,0.001223},
  {0,0,0.001223},
  {0,0,0.000103},
  {0,0,0.000103},
  {0,0,0.000783},
  {0,0,0.000783},
  {0,0,0.001615},
  {0,0,0.001615},
  {0,0,0.001541},
  {0,0,0.001541},
  {0,0,0.003037},
  {0,0,0.003037},
  {0,0,0.003037},
  {0,0,0.003104},
  {-0.019897,0.007679,0.028824},
  {-0.019897,0.007679,0.028824},
  {-0.062832,0.007679,0.068181},
  {-0.062832,0.007679,0.068181},
  {-0.134914,-0.006283,0.113549},
  {-0.134914,-0.006283,0.113549},
  {-0.24836,-0.006283,0.163105},
  {-0.24836,-0.006283,0.163105},
  {-0.387987,-0.006283,0.205962},
  {-0.387987,-0.006283,0.205962},
  {-0.387987,-0.006283,0.205962},
  {-0.56601,-0.006283,0.239087},
  {-0.723963,-0.006283,0.270468},
  {-0.723963,-0.006283,0.270468},
  {-0.889594,-0.006283,0.302624},
  {-0.889594,-0.006283,0.302624},
  {-1.032362,-0.006283,0.334005},
  {-1.032362,-0.006283,0.334005},
  {-1.228712,-0.006283,0.347624},
  {-1.228712,-0.006283,0.347624},
  {-1.425061,0,0.342511},
  {-1.425061,0,0.342511},
  {-1.425061,0,0.342511},
  {-1.632057,0.006109,0.337376},
  {-1.810081,0.009076,0.339733},
  {-1.810081,0.009076,0.339733},
  {-1.810081,0.009076,0.339733},
  {-1.992642,0.015184,0.343147},
  {-2.188992,0.021468,0.333475},
  {-2.188992,0.021468,0.333475},
  {-2.39913,0.024435,0.327078},
  {-2.39913,0.024435,0.327078},
  {-2.586229,0.027576,0.322236},
  {-2.586229,0.027576,0.322236},
  {-2.784149,0.033685,0.32858},
  {-2.784149,0.033685,0.32858},
  {-2.977357,0.036652,0.330603},
  {-2.977357,0.036652,0.330603},
  {-3.204424,0.039794,0.321339},
  {-3.204424,0.039794,0.321339},
  {-3.389953,0.042935,0.307643},
  {-3.389953,0.042935,0.307643},
  {-3.389953,0.042935,0.307643},
  {-3.60777,0.049044,0.291697},
  {-3.83187,0.052011,0.287057},
  {-3.83187,0.052011,0.287057},
  {-4.026649,0.055152,0.287196},
  {-4.026649,0.055152,0.287196},
  {-4.227536,0.058294,0.280945},
  {-4.227536,0.058294,0.280945},
  {-4.439245,0.064403,0.271692},
  {-4.439245,0.064403,0.271692},
  {-4.439245,0.064403,0.271692},
  {-4.690747,0.06737,0.261274},
  {-4.905597,0.070511,0.247776},
  {-4.905597,0.070511,0.247776},
  {-5.088158,0.070511,0.234273},
  {-5.088158,0.070511,0.234273},
  {-5.310513,0.070511,0.212664},
  {-5.310513,0.070511,0.212664},
  {-5.543689,0.070511,0.188618},
  {-5.543689,0.070511,0.188618},
  {-5.726251,0.070511,0.167843},
  {-5.726251,0.070511,0.167843},
  {-5.959427,0.070511,0.151501},
  {-5.959427,0.070511,0.151501},
  {-6.188065,0.070511,0.133039},
  {-6.188065,0.070511,0.133039},
  {-6.41967,0.070511,0.093909},
  {-6.41967,0.070511,0.093909},
  {-6.602231,0.070511,0.05054},
  {-6.602231,0.070511,0.05054},
  {-6.847625,0.070511,0.015247},
  {-6.847625,0.070511,0.015247},
  {-7.053225,0.070511,359.992915},
  {-7.053225,0.070511,359.992915},
  {-7.261791,0.073478,359.987842},
  {-7.261791,0.073478,359.987842},
  {-7.442782,0.073478,359.979403},
  {-7.442782,0.073478,359.979403},
  {-7.442782,0.073478,359.979403},
  {-7.675958,0.07662,359.949897},
  {-7.899884,0.07662,359.906952},
  {-7.899884,0.07662,359.906952},
  {-8.093266,0.079762,359.858074},
  {-8.093266,0.079762,359.858074},
  {-8.289616,0.08587,359.823369},
  {-8.289616,0.08587,359.823369},
  {-8.484394,0.091979,359.799417},
  {-8.484394,0.091979,359.799417},
  {-8.688423,0.094946,359.777528},
  {-8.688423,0.094946,359.777528},
  {-8.846376,0.094946,359.783047},
  {-8.846376,0.094946,359.783047},
  {-9.030508,0.094946,359.806938},
  {-9.030508,0.094946,359.806938},
  {-9.197711,0.094946,359.844296},
  {-9.197711,0.094946,359.844296},
  {-9.354092,0.094946,359.870743},
  {-9.354092,0.094946,359.870743},
  {-9.481501,0.094946,359.879508},
  {-9.481501,0.094946,359.879508},
  {-9.610306,0.094946,359.882056},
  {-9.610306,0.094946,359.882056},
  {-9.720786,0.094946,359.880319},
  {-9.720786,0.094946,359.880319},
  {-9.720786,0.094946,359.880319},
  {-9.824982,0.09355,359.876743},
  {-9.944711,0.09355,359.88392},
  {-9.944711,0.09355,359.88392},
  {-9.944711,0.09355,359.88392},
  {-10.04437,0.09355,359.887838},
  {-10.04437,0.09355,359.887838},
  {-10.134952,0.09355,359.890844},
  {-10.134952,0.09355,359.890844},
  {-10.222393,0.09355,359.894851},
  {-10.300584,0.09355,359.90207},
  {-10.300584,0.09355,359.90207},
  {-10.380346,0.09355,359.903788},
  {-10.380346,0.09355,359.903788},
  {-10.44789,0.09355,359.904004},
  {-10.44789,0.09355,359.904004},
  {-10.526081,0.09355,359.901885},
  {-10.526081,0.09355,359.901885},
  {-10.598163,0.09355,359.914466},
  {-10.598163,0.09355,359.914466},
  {-10.661169,0.09355,359.919629},
  {-10.661169,0.09355,359.919629},
  {-10.717892,0.09355,359.919214},
  {-10.717892,0.09355,359.919214},
  {-10.717892,0.09355,359.919214},
  {-10.771474,0.09355,359.918828},
  {-10.823659,0.09355,359.920838},
  {-10.823659,0.09355,359.920838},
  {-10.866594,0.09355,359.931376},
  {-10.866594,0.09355,359.931376},
  {-10.912671,0.09355,359.946554},
  {-10.912671,0.09355,359.946554},
  {-10.955606,0.09355,359.95718},
  {-10.955606,0.09355,359.95718},
  {-10.955606,0.09355,359.95718},
  {-10.998541,0.09355,359.967006},
  {-11.032226,0.09355,359.974406},
  {-11.032226,0.09355,359.974406},
  {-11.073765,0.09355,359.982445},
  {-11.073765,0.09355,359.982445},
  {-11.105879,0.09355,359.987642},
  {-11.105879,0.09355,359.987642},
  {-11.138168,0.09355,359.988582},
  {-11.138168,0.09355,359.988582},
  {-11.164173,0.09355,359.993867},
  {-11.164173,0.09355,359.993867},
  {-11.185641,0.09355,359.995803},
  {-11.185641,0.09355,359.995803},
  {-11.21182,0.09355,359.995803},
  {-11.21182,0.09355,359.995803},
  {-11.227179,0.09355,359.999695},
  {-11.227179,0.09355,359.999695},
  {-11.247076,0.09355,0.002094},
  {-11.247076,0.09355,0.002094},
  {-11.266973,0.09355,0.004091},
  {-11.266973,0.09355,0.004091},
  {-11.266973,0.09355,0.004091},
  {-11.279365,0.09355,0.007079},
  {-11.293153,0.09355,0.006695},
  {-11.293153,0.09355,0.006695},
  {-11.30537,0.09355,0.002864},
  {-11.30537,0.09355,0.002864},
  {-11.31462,0.09355,359.998283},
  {-11.31462,0.09355,359.998283},
  {-11.317587,0.09355,359.994846},
  {-11.317587,0.09355,359.994846},
  {-11.323696,0.09355,359.992699},
  {-11.323696,0.09355,359.992699},
  {-11.326838,0.09355,359.994209},
  {-11.326838,0.09355,359.994209},
  {-11.326838,0.09355,359.994209},
  {-11.329979,0.09355,359.995229},
  {-11.332946,0.09355,359.996362},
  {-11.332946,0.09355,359.996362},
  {-11.332946,0.09355,359.996052},
  {-11.332946,0.09355,359.996052},
  {-11.332946,0.09355,359.995814},
  {-11.332946,0.09355,359.995814},
  {-11.332946,0.09355,359.995511},
  {-11.332946,0.09355,359.995511},
  {-11.332946,0.09355,359.99558},
  {-11.332946,0.09355,359.99558},
  {-11.332946,0.09355,359.995121},
  {-11.332946,0.09355,359.995121},
  {-11.331376,0.09355,359.994078},
  {-11.331376,0.09355,359.994078},
  {-11.331376,0.09355,359.994078},
  {-11.331376,0.09355,359.993827},
  {-11.331376,0.09355,359.99397},
  {-11.331376,0.09355,359.99397},
  {-11.331376,0.09355,359.99443},
  {-11.331376,0.09355,359.99443},
  {-11.331376,0.09355,359.995102},
  {-11.331376,0.09355,359.995102},
  {-11.331376,0.09355,359.995547},
  {-11.331376,0.09355,359.995547},
  {-11.329979,0.09355,359.995342},
  {-11.329979,0.09355,359.995342},
  {-11.329979,0.09355,359.994493},
  {-11.329979,0.09355,359.994493},
  {-11.329979,0.09355,359.993752},
  {-11.329979,0.09355,359.993752},
  {-11.329979,0.09355,359.993373},
  {-11.329979,0.09355,359.993373} 
 };
#endif

void Odom::set_physical_distances(double ForwardTracker_center_distance, double SidewaysTracker_center_distance){
  this->ForwardTracker_center_distance = ForwardTracker_center_distance;
  this->SidewaysTracker_center_distance = SidewaysTracker_center_distance;
}

/**
 * Resets the position, including tracking wheels.
 * Position is field-centric, and orientation is such that 0 degrees
 * is in the positive Y direction. Orientation can be provided with 
 * some flexibility, including less than 0 and greater than 360.
 * 
 * @param X_position Field-centric x position of the robot.
 * @param Y_position Field-centric y position of the robot.
 * @param orientation_deg Field-centered, clockwise-positive, orientation.
 * @param ForwardTracker_position Current position of the sensor in inches.
 * @param SidewaysTracker_position Current position of the sensor in inches.
 */

void Odom::set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position){
  this->ForwardTracker_position = ForwardTracker_position;
  this->SideWaysTracker_position = SidewaysTracker_position;
  this->X_position = (double) X_position;
  this->Y_position = (double) Y_position;
  this->orientation_deg = (double) orientation_deg;
}

/**
 * Does the odometry math to update position
 * Uses the Pilons arc method outline here: https://wiki.purduesigbots.com/software/odometry
 * All the deltas are done by getting member variables and comparing them to 
 * the input. Ultimately this all works to update the public member variable
 * X_position. This function needs to be run at 200Hz or so for best results.
 * 
 * @param ForwardTracker_position Current position of the sensor in inches.
 * @param SidewaysTracker_position Current position of the sensor in inches.
 * @param orientation_deg Field-centered, clockwise-positive, orientation.
 */

void Odom::update_position(double ForwardTracker_position, double SidewaysTracker_position, double orientation_deg){
  // this-> always refers to the old version of the variable, so subtracting this->x from x gives delta x.
  double Forward_delta = ForwardTracker_position - this->ForwardTracker_position;
  double Sideways_delta = SidewaysTracker_position - this->SideWaysTracker_position;

  this->ForwardTracker_position = ForwardTracker_position;
  this->SideWaysTracker_position = SidewaysTracker_position;

  double orientation_rad = to_rad(reduce_negative_180_to_180(orientation_deg));
  double prev_orientation_rad = to_rad(reduce_negative_180_to_180(this->orientation_deg));
  double orientation_delta_rad = reduce_negative_PI_to_PI(orientation_rad - prev_orientation_rad);
  this->orientation_deg = orientation_deg;

  if (Forward_delta == 0.0 && Sideways_delta == 0.0 && orientation_delta_rad == 0.0) {
    // No movement, no update needed.
    return;
  }

  double local_X_position;
  double local_Y_position;

  if (orientation_delta_rad == 0.0) {
    local_X_position = Sideways_delta;
    local_Y_position = Forward_delta;
  } else {
    local_X_position = (2.0 * sin(orientation_delta_rad / 2.0))*((Sideways_delta / orientation_delta_rad) + SidewaysTracker_center_distance);
    local_Y_position = (2.0 * sin(orientation_delta_rad / 2.0))*((Forward_delta / orientation_delta_rad) + ForwardTracker_center_distance);
  }

  X_local = local_X_position;
  Y_local = local_Y_position;

#ifdef USE_ROTATION

  double local_polar_angle;
  double local_polar_length;

  if (local_X_position == 0.0 && local_Y_position == 0.0){
    local_polar_angle = 0.0;
    local_polar_length = 0.0;
  } else {
    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 
  }

  double global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad / 2.0);

  double X_position_delta = local_polar_length * cos(global_polar_angle); 
  double Y_position_delta = local_polar_length * sin(global_polar_angle);
#else
  double average_orientation = prev_orientation_rad + (orientation_delta_rad / 2.0);
  // average_orientation = 0.0;

  double X_position_delta =   local_X_position * cos(average_orientation) + local_Y_position * sin(average_orientation);
  double Y_position_delta = - local_X_position * sin(average_orientation) + local_Y_position * cos(average_orientation);
#endif

  X_position += X_position_delta;
  Y_position += Y_position_delta;
}
