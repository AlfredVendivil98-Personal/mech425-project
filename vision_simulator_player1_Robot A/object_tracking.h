
int activate_odt();
int deactivate_odt();

int obstacle_detection_tracking(image& rgb1, double* p_ob_X, double* p_ob_Y, int& n_obs, int H, int W);

int robot_detection_tracking(image& rgb1, int H, int W);

double img_threshold_dynamic(image& rgb1, double tvalue);

int rgb_centroid_mrks(image& rgb1, double& im_r, double& jm_r, double& im_g, double& jm_g, double& io_b, 
						double& jo_b, double& io_o, double& jo_o);

int track_centroid_mrks(image& rgb1, i2byte& nlabel, double& it, double& jt);

int track_centroid_robot(image& rgb1, double& ic1_color, double& jc1_color, double& ic2_color, double& jc2_color, 
						double& IC_ROBOT, double& JC_ROBOT);

//Do we really need image objects a and b to pass to spiral_search and spiral_search_area()?
int spiral_search_robot(image& rgb1, i2byte& nlabel, image& label, image& a, image& b, int is, int js);

int spiral_search_area(image& rgb1, i2byte& nlabel, image& label, image& a, image& b, int isa, int jsa);

int follow_object_mrks(image& rgb1, i2byte nlabel, double io, double jo);

//CALL THIS ONE IN MAIN FOR COLLISION (PROCESSED RGB IMAGE PASSED TO MAIN())
int label_rgb_collision(image& rgb_out);

int label_objects_mrks(image& rgb1, int tvalue);


//int label_objects_collision(image& rgb1, image& rgb_out, int tvalue);

void turn_grey(image& rgb1);

int get_position_robot(double& i, double& j, double& angle);

int get_position_opp_robot(double& i_opp, double& j_opp, double& angle_opp);

int get_position_front_robot(double& ifront, double& jfront);

int get_position_rear_robot(double& irear, double& jrear);

int get_position_front_opp_robot(double& ifront_opp, double& jfront_opp);

int get_position_rear_opp_robot(double& irear_opp, double& jrear_opp);