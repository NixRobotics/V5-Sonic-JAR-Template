/**
 * General-use odometry class with X_position, Y_position, and
 * orientation_deg being the relevant outputs. This works for one
 * and two-tracker systems, and needs a gyro to get input angle.
 */

class Odom
{
private:
  double ForwardTracker_center_distance;
  double SidewaysTracker_center_distance;
  double ForwardTracker_position;
  double SideWaysTracker_position;
  bool is_running = false;
public:
  double X_position, X_local;
  double Y_position, Y_local;
  double orientation_deg;
  void set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position);
  void update_position(double ForwardTracker_position, double SidewaysTracker_position, double orientation_deg);
  void set_physical_distances(double ForwardTracker_center_distance, double SidewaysTracker_center_distance);
};

#ifdef ODOM_PLAYBACK
extern double playback_buffer[500][3];
extern int playback_buffer_len;
#endif
