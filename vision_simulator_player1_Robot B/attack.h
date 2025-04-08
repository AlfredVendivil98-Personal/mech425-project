

int attack_path(image& rgb, int& pw_r, int& pw_l, int& laser, double x_robot, double y_robot, double theta, double x_opp_front, double y_opp_front, double x_opp_rear, double y_opp_rear, double opp_theta);
int reach_target_angle(int& pw_r, int& pw_l, double& my_theta, double& off_theta, bool& laser_setting);
double calculate_distance(double xc, double yc, double xc2, double yc2);
double calculate_angle(double dx, double dy);

int draw_corners(image& rgb, int* xp, int* yp);
int convert_rad_to_degrees(double& theta_rad);
void shortest_dist(double* d_line, int N_lines, double* theta_line, double& dmin, double& theta_min);


//my_functions header
double calculate_position(double x, double y, double& theta); //USED
void offense_line(double x, double y, double x_opp, double y_opp, int& stat, image& rgb); //USED
double measure_dist(double x1, double y1, double x2, double y2); //USED
