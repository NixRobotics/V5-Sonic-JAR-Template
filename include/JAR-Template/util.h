double reduce_0_to_360(double angle);

double reduce_negative_180_to_180(double angle);

double reduce_negative_PI_to_PI(double angle);

float reduce_negative_90_to_90(float angle);

double to_rad(double angle_deg);

double to_deg(double angle_rad);

float clamp(float input, float min, float max);

bool is_reversed(double input);

float to_volt(float percent);

int to_port(int port);

float deadband(float input, float width);

bool is_line_settled(float desired_X, float desired_Y, float desired_angle, float current_X, float current_Y);

float left_voltage_scaling(float drive_output, float heading_output);

float right_voltage_scaling(float drive_output, float heading_output);

float clamp_min_voltage(float drive_output, float drive_min_voltage);