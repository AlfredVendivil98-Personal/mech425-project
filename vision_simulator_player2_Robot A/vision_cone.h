#include "vision.h"

class vision_cone {
	int width, height,w,pw_l,pw_r; //w is the width of the detection rectangle
	double tc0;
	static double tc, tc1;
	int b[6], stat[6]; 
	int state; //Checks for the state of the vision line (FOR OFFENSE) 
	int b_rear, stat_rear;
	static int rev,t,t1;
	double x, y, theta, ra, c_a;
	image rgb;

public:
	static int hit;

	vision_cone(double x, double y, double theta, double ra, double c_a, int state, image &rgb_in);
	void image_processing_collision(image& a, image& b);
	void draw_cone(int &pw_l, int &pw_r,image &rgb_out);
	void vision_line_reverse(int ra, int t1, int t2); 
	void stuck_check(double x_front, double y_front);
	~vision_cone();
};