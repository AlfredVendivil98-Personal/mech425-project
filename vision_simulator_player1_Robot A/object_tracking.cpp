#include <iostream>
#include <fstream>

#include <cmath>
#include <Windows.h>

using namespace std;

// include this header file for basic image transfer functions
#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"
#include "object_tracking.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

image a, b, label, rgb_c; //rgb_c = image copy of original rgb image
i2byte nlabel_ob;
i2byte nlabel_mkr;

static double M_PI = atan(1) * 4;
double i1_r, j1_r, i1_g, j1_g, i2_b, j2_b, i2_o, j2_o;

double IC_ROBOT, JC_ROBOT;
double IC_ROBOT_O, JC_ROBOT_O;

int height;
int width;
double area;

double im_processing_thresh;
double a_thresh = 3.14159*55*55;


int activate_odt()
{

	//cout << "\nActivate_odt() function call";
	static int state = 0;

	if (state == 0) {

		rgb_c.type = RGB_IMAGE;
		rgb_c.width = width;
		rgb_c.height = height;

		a.type = GREY_IMAGE;
		a.width = width;
		a.height = height;

		b.type = GREY_IMAGE;
		b.width = width;
		b.height = height;

		label.type = LABEL_IMAGE;
		label.width = width;
		label.height = height;

		allocate_image(a);
		allocate_image(b);
		allocate_image(rgb_c);
		allocate_image(label);

		state = 1;
	}

	return 0;
}


int deactivate_odt()
{

	free_image(a);
	free_image(b);
	free_image(label);
	free_image(rgb_c);
	return 0;
}


int obstacle_detection_tracking(image& rgb1, double* p_ob_X, double* p_ob_Y, int& n_obs, int H, int W)
{
	//cout << "\nobstacle_detectiong_tracking() function call";

	height = H;
	width = W;
	int ntimes = 10;
	int nlabels;
	int j = 0;

	int calc_cent_state;

	activate_odt();

	//cout << "\ncopy original rgb image from sim to rgb copy";
	copy(rgb1, rgb_c);

	copy(rgb_c, a);

	scale(a, a);

	copy(a, rgb_c);

	lowpass_filter(a, a);

	copy(a, rgb_c);

	im_processing_thresh = 0;
	threshold(a, a, img_threshold_dynamic(rgb1, im_processing_thresh));
	
	copy(a, rgb_c);

	invert(a, a);
	copy(a, rgb_c);

	for (int i = 0; i < ntimes; i++) {

		erode(a, b);
		copy(b, a);

	}

	//Used to analyze the binary image to find a_thresh (or radius of obstacles)
	//r = 55 pixels; x0 = 108, y0 = 163; r = 163-108 = 55 pix
	//copy(a, rgb_c);
	//save_rgb_image("rgb_c_out.bmp", rgb_c);

	label_image(a, label, nlabels);

	//This will check for all the obstacles labelled in the binary image
	//When it reaches the end of the for loop, it will have successfully
	//labelled all the obstacles on screen and we can stop checking for them.

	//TO-DO: Fix small bug where obstacle is detected at window origin (0,0) bottom left

	for (int k = 1; k <= nlabels; k++) {

		calc_cent_state = 0;

		spiral_search_area(rgb1, nlabel_ob, label, a, b, (int)p_ob_X[j], (int)p_ob_Y[j]);

		//spiral_search(rgb1, nlabel_ob, label, a, b, (int)p_ob_X[j], (int)p_ob_Y[j]);

		if (area >= a_thresh && calc_cent_state == 0) {

			centroid(a, label, k, p_ob_X[j], p_ob_Y[j]);
			//cout << "\nChecking area threshold and drawing point in sim";
			draw_point_rgb(rgb1, (int)p_ob_X[j], (int)p_ob_Y[j], 255, 255, 0);

			calc_cent_state = 1;
			n_obs = k;
			j++;
			//cout << "\nObject count =" << k;
		}
		if (area < a_thresh) {
			cout << "\nObstacle was not detected at this index value, error";
			return 0;
		}
	}

	return 0;

}


int robot_detection_tracking(image& rgb1, int H, int W)
{
	//Need to re-initialized every time?
	i1_r = j1_r = i1_g = j1_g = i2_b = j2_b = i2_o = j2_o = 0.0;

	IC_ROBOT = JC_ROBOT = 0.0;
	IC_ROBOT_O = JC_ROBOT_O = 0.0;

	height = H;
	width = W;

	im_processing_thresh = 0;
	im_processing_thresh = img_threshold_dynamic(rgb1, im_processing_thresh);

	activate_odt();
		
	rgb_centroid_mrks(rgb1, i1_r, j1_r, i1_g, j1_g, i2_b, j2_b, i2_o, j2_o);

	track_centroid_mrks(rgb1, nlabel_mkr, i1_r, j1_r);
	track_centroid_mrks(rgb1, nlabel_mkr, i1_g, j1_g);
	track_centroid_mrks(rgb1, nlabel_mkr, i2_b, j2_b);
	track_centroid_mrks(rgb1, nlabel_mkr, i2_o, j2_o);

	track_centroid_robot(rgb1, i1_r, j1_r, i1_g, j1_g, IC_ROBOT, JC_ROBOT);
	track_centroid_robot(rgb1, i2_b, j2_b, i2_o, j2_o, IC_ROBOT_O, JC_ROBOT_O);

	//Maybe have it here instead of in main() "program.cpp"?
	//deactivate_odt();

	return 0; //no errors
}


double img_threshold_dynamic(image& rgb1, double tvalue)
{
	i4byte i, size;
	ibyte* pa, min, max;

	double hmin, hmax;

	// initialize pointer
	pa = rgb1.pdata;

	// number of pixels
	size = (i4byte)rgb1.width * rgb1.height;

	// find min/max
	max = 0; min = 255;
	for (i = 0; i < size; i++) {
		if (*pa > max) max = *pa;
		if (*pa < min) min = *pa;
		pa++;
	}

	//cout << "\nmax = " << (int)max << " , min = " << (int)min;

	// return parameters
	hmin = (double)min;
	hmax = (double)max;

	tvalue = (hmin + hmax) / 2.0;

	//cout << "\nThreshold value = " << tvalue;
	return tvalue;
}

int rgb_centroid_mrks(image& rgb1, double& im_r, double& jm_r, double& im_g, double& jm_g, double& io_b, double& jo_b, double& io_o, double& jo_o)
{
	//static int setup;
	//setup = 0;
	//if (setup = 1) exit; //

	int i, j, k, width, height;
	ibyte* p, * pc;
	double mi_o, mj_o, m, i_o, j_o, eps;
	ibyte R, G, B;
	
	//int nlabels;
	//i2byte* pl;


	double mi_b, mj_b, m_b;
	double opp_avg_i, opp_avg_j;
	double us_avg_i, us_avg_j;

	double i_b, j_b;
	double mi_g, mj_g, m_g, i_g, j_g;
	double mi_r, mj_r, m_r;
	double i_r, j_r;

	//rgb1.type = RGB_IMAGE;
	width = 640;
	height = 480;

	p = rgb1.pdata;
	//pl = (i2byte*)label_col.pdata;

	// Always initialize summation variables
	mi_o = mj_o = m = 0.0;
	mi_b = mj_b = m_b = 0.0;
	mi_r = mj_r = m_r = 0.0;
	mi_g = mj_g = m_g = 0.0;

	copy(rgb1, rgb_c);
	copy(rgb_c, a);

	//label_image(a, label_col, nlabels);


	for (j = 0; j < height; j++) { // j coord

		for (i = 0; i < width; i++) { // i coord

			k = i + width * j;
			pc = p + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			//nlabel_mkr = *(pl + j * label_col.width + i);
			//if (nlabel_mkr == 0) return 0;

			B = *pc;
			G = *(pc + 1);
			R = *(pc + 2);

			// IMPORTANT:The following segment computes the centroid of the different robots based on the RGB method.
			// After the 

			// Do the same with the blue and then compute the avergages of the two colors.
			// the centroid from this function can be used as the starting point for the spiral method. This is faster
			// then randomly using the spiral method.

				// highlight the Red pixels of our robot based on the vision simulator and compute centroid elements
			if ((B < 110) && (R > 200) && (G < 110)) {

				R = 255;
				G = 0;
				B = 0;


				*pc = B;
				*(pc + 1) = G;
				*(pc + 2) = R;

				m_r += R;

				// calculate total moments in the i and j directions
				mi_r += i * R; // (i moment of mk) = mk * i
				mj_r += j * R; // (j moment of mk) = mk * j

			} // end if

			// highlight the Green pixels of our robot based on the vision simulator and compute centroid elements
			if ((B < 150) && (R < 80) && (G > 170)) {

				R = 0;
				G = 100;
				B = 0;


				*pc = B;
				*(pc + 1) = G;
				*(pc + 2) = R;

				m_g += G;

				// calculate total moments in the i and j directions
				mi_g += i * G; // (i moment of mk) = mk * i
				mj_g += j * G; // (j moment of mk) = mk * j



			} // end if

			// highlight the Orange pixels of the opponent robot based on the vision simulator and compute centroid elements
			if ((B > 100) && (B < 150) && (R > 225) && (G > 150) && (G < 225)) {

				R = 225;
				G = 80;
				B = 0;

				// highlight the orange pixels of the opponent based on the vision simulator
				*pc = B;
				*(pc + 1) = G;
				*(pc + 2) = R;

				// to calculate the centroid you need to calculate mk
				// - the mass of each pixel k

	// mk = volume * density
	// mk = (pixel area) * (blue intensity)
	// mk = (blue intensity) = B
	// since (pixel area) = 1 pixel x 1 pixel = 1

	// calculate total mass m = sum (mk)
				m += R;

				// calculate total moments in the i and j directions
				mi_o += i * R; // (i moment of mk) = mk * i
				mj_o += j * R; // (j moment of mk) = mk * j

			}

			// highlight the Blue pixels of the opponent robot based on the vision simulatorand compute centroid elements
			if ((B > 200) && (R < 70) && (G < 170)) {

				R = 0;
				G = 0;
				B = 255;

				// highlight the orange pixels of the opponent based on the vision simulator
				*pc = B;
				*(pc + 1) = G;
				*(pc + 2) = R;

				// to calculate the centroid you need to calculate mk
				// - the mass of each pixel k

				// mk = volume * density
				// mk = (pixel area) * (blue intensity)
				// mk = (blue intensity) = B
				// since (pixel area) = 1 pixel x 1 pixel = 1

				// calculate total mass m = sum (mk)
				m_b += B;

				// calculate total moments in the i and j directions
				mi_b += i * B; // (i moment of mk) = mk * i
				mj_b += j * B; // (j moment of mk) = mk * j


			} // end if

		} // end for i

	} // end for j


	//view_rgb_image(rgb1);

	// Calculate the centroid (ic,jc)
	// -- note that ic, jc can have subpixel accuracy
	// (1/10 of a pixel or better typically)

	// Computation for the orange centroid
	eps = 1.0e-10; // small constant to protect against /0
	i_o = mi_o / (m + eps);
	j_o = mj_o / (m + eps);

	// Computation for the blue centroid.
	i_b = mi_b / (m_b + eps);
	j_b = mj_b / (m_b + eps);

	//compute the average centroid location based on the centroid for the orange and blue colors.
	opp_avg_i = (i_o + i_b) / 2;
	opp_avg_j = (j_o + j_b) / 2;


	//compute the centroid for the green centroid
	i_g = mi_g / (m_g + eps);
	j_g = mj_g / (m_g + eps);

	//compute the centroid for the red centroid
	i_r = mi_r / (m_r + eps);
	j_r = mj_r / (m_r + eps);

	//compute the average centroid location based on the centroid for the orange and blue colors.
	us_avg_i = (i_g + i_r) / 2;
	us_avg_j = (j_g + j_r) / 2;



	// centroid marker for the orange circle of the opponent robot.
	draw_point_rgb(rgb1, (int)i_o, (int)j_o, 0, 0, 0);

	// centroid marker for the blue circle of the opponent robot.
	draw_point_rgb(rgb1, (int)i_b, (int)j_b, 0, 0, 0);

	//centroid marker for the average of the blue and orange centroids to get the middle of the opponent robot.
	draw_point_rgb(rgb1, (int)opp_avg_i, (int)opp_avg_j, 119, 1, 123);


	// centroid marker for the green circle of our robot.
	draw_point_rgb(rgb1, (int)i_g, (int)j_g, 0, 0, 0);

	// centroid marker for the red circle of our robot.
	draw_point_rgb(rgb1, (int)i_r, (int)j_r, 0, 0, 0);

	//centroid marker for the average of the green and red centroids to get the middle of our robot.
	draw_point_rgb(rgb1, (int)us_avg_i, (int)us_avg_j, 119, 1, 123);


	im_r = i_r;
	jm_r = j_r;

	im_g = i_g;
	jm_g = j_g;

	io_b = i_b;
	jo_b = j_b;

	io_o = i_o;
	jo_o = j_o;
	//view_rgb_image(rgb1);

	//setup = 1;

	return 0;
}

int track_centroid_mrks(image& rgb1, i2byte& nlabel, double& it, double& jt)
{

	follow_object_mrks(rgb1, nlabel, it, jt);

	//view_rgb_image(rgb);
	return 0;
}

int track_centroid_robot(image& rgb1, double& ic1_color, double& jc1_color, double& ic2_color, double& jc2_color, double& IC_ROBOT, double& JC_ROBOT)
{
	IC_ROBOT = (ic1_color + ic2_color) / 2.0;

	JC_ROBOT = (jc1_color + jc2_color) / 2.0;

	//BRIGHT PINK COLOR
	draw_point_rgb(rgb1, (int)IC_ROBOT, (int)JC_ROBOT, 255, 0, 127);

	return 0;
}

int spiral_search_robot(image& rgb1, i2byte& nlabel, image& label, image& a, image& b, int is, int js) 
{

	i2byte* pl;
	int i, j;
	double r, rmax, dr, s, smax, ds, theta;

	// pointer to a label image
	pl = (i2byte*)label.pdata;

	rmax = 60.0; // maximum radius of search (pixels)
	dr = 3.0; // radius divisions (pixels)
	ds = 3.0; // arc-length divisions (pixels)

	nlabel = *(pl + js * label.width + is);
	if (nlabel != 0) return 0;

	// acquire image
	//acquire_image(rgb0, cam_number);

	//copy(a, b);

	// search for a labeled object in an outward concentic ring pattern
	for (r = 1.0; r <= rmax; r += dr) {
		smax = 2 * 3.1416 * r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta

			//label_objects(tvalue);

			i = (int)(is + r * cos(theta));
			j = (int)(js + r * sin(theta));

			//YELLOW COLOR
			//draw_point_rgb(rgb1, i, j, 255, 255, 0);
			//draw_point_rgb(rgb, 320, 240, 0, 255, 0);
			//view_rgb_image(rgb);

			// limit (i,j) from going off the image
			if (i < 0) i = 0;
			if (i > label.width - 1) i = label.width - 1;
			if (j < 0) j = 0;
			if (j > label.height - 1) j = label.height - 1;


			area += 3.14159 * r * r;

			nlabel = *(pl + j * label.width + i);
			if (nlabel != 0) return 0;
		}
	}

	//cout << "\n spiral_search() function call";
	return 0;
}

int spiral_search_area(image& rgb1, i2byte& nlabel, image& label, image& a, image& b, int isa, int jsa)
{
	i2byte* pl;
	int i, j;
	double r, rmax, dr, s, smax, ds, theta;

	// pointer to a label image
	pl = (i2byte*)label.pdata;

	rmax = 60.0; // maximum radius of search (pixels)
	dr = 3.0; // radius divisions (pixels)
	ds = 3.0; // arc-length divisions (pixels)

	// search for a labeled object in an outward concentic ring pattern
	for (r = 1.0; r <= rmax; r += dr) {
		smax = 2 * 3.1416 * r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta

			//label_objects(tvalue);

			i = (int)(isa + r * cos(theta));
			j = (int)(jsa + r * sin(theta));

			//YELLOW COLOR
			//draw_point_rgb(rgb1, i, j, 255, 255, 0);
			//draw_point_rgb(rgb, 320, 240, 0, 255, 0);
			//view_rgb_image(rgb);

			// limit (i,j) from going off the image
			if (i < 0) i = 0;
			if (i > label.width - 1) i = label.width - 1;
			if (j < 0) j = 0;
			if (j > label.height - 1) j = label.height - 1;

			area += 3.14159 * r * r;

			nlabel = *(pl + j * label.width + i);
			if (nlabel != 0) return 0;
		}
	}

	//cout << "\n spiral_search() function call";
	return 0;

}


int follow_object_mrks(image& rgb1, i2byte nlabel, double io, double jo)
{

	//cout << "\n follow_object() function call";

	//TO DO: Implement counter variable (i) to determine how many objects
	//on screen are detected
	//This will help for an unknown amount of obstacles in simulator when
	//the program runs

	label_objects_mrks(rgb1, im_processing_thresh);

	spiral_search_robot(rgb1, nlabel, label, a, b, (int)io, (int)jo);

	centroid(a, label, nlabel, io, jo);

	//BRIGHT WHITE COLOR
	draw_point_rgb(rgb1, (int)io, (int)jo, 255, 255, 255);


	//view_rgb_image(rgb);
	//cout << "\nCentroid io: " << io << "\tCentroid jo: " << jo;

	return 0;
}

int label_rgb_collision(image& rgb_out)
{

	copy(rgb_c, rgb_out);

	return 0;
}

int label_objects_mrks(image& rgb1, int tvalue) 
{
	//cout << "\n label_objects() function call";
	int nlabels;

	copy(rgb1, rgb_c);

	// convert RGB image to a greyscale image
	copy(rgb_c, a);
	copy(a, rgb_c);

	// scale the image to enhance contrast
	scale(a, a);
	copy(a, rgb_c);
	
	// use threshold function to make a binary image (0,255)
	threshold(a, a, tvalue);
	copy(a, rgb_c);

	// invert the image
	invert(a, a);
	copy(a, rgb_c);

	// perform an erosion function to remove noise (small objects)
	erode(a, b);
	copy(b, rgb_c);

	// perform a dialation function to fill in 
	// and grow the objects
	dialate(b, a);
	copy(a, rgb_c);

	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(a, label, nlabels);

	//cout << "\nObjects discovered = " << nlabels;

	return 0; // no errors
}


/*int label_objects_collision(image& rgb1, image& rgb_out, int tvalue)
{
	//cout << "\n label_objects() function call";
	int nlabels;

	copy(rgb1, rgb_out);

	// convert RGB image to a greyscale image
	copy(rgb_c, a);
	copy(a, rgb_c);

	// scale the image to enhance contrast
	scale(a, a);
	copy(a, rgb_c);

	// use threshold function to make a binary image (0,255)
	threshold(a, a, tvalue);
	copy(a, rgb_c);

	// invert the image
	invert(a, a);
	copy(a, rgb_c);

	// perform an erosion function to remove noise (small objects)
	erode(a, b);
	copy(b, rgb_c);

	// perform a dialation function to fill in 
	// and grow the objects
	dialate(b, a);
	copy(a, rgb_c);

	return 0;

}
*/

void turn_grey(image& rgb1)
{
	int width1 = 640;
	int height1 = 480;
	int size = width1 * height1;

	int k;
	ibyte* p; // a pointer to a single byte
	p = rgb1.pdata; // get image pointer
	ibyte R, G, B, grey;

	for (k = 0; k < size; k++) { // access each pixel i

		//  B G R
		B = *p;
		G = *(p + 1);
		R = *(p + 2);

		// convert to greyscale
		grey = 0.3333 * B + 0.3333 * G + 0.3333 * R;

		*p = grey; // B
		*(p + 1) = grey; // G
		*(p + 2) = grey; // R

		p += 3;

	}
}

int get_position_robot(double& i, double& j, double& angle)
{
	double delta_i, delta_j, alpha;
	i = IC_ROBOT;
	j = JC_ROBOT;

	delta_i = i1_g - i1_r;
	delta_j = j1_g - j1_r;

	alpha = atan2((delta_j), (delta_i));

	angle = alpha;

	/*if (0 < angle < M_PI/2) {

		//First quadrant
		angle = angle * (180.0 / M_PI);
	}

	if (M_PI/2 < angle < M_PI) {

		//Second quadrant
		angle = angle * (180.0 / M_PI);
	}

	if (-M_PI/2 < angle < 0) {

		//Fourth quadrant
		angle = 360 - (angle * (180 / M_PI));
	}

	if (-M_PI < angle < -M_PI/2) {

		//Third quadrant
		angle = 360 - (angle * (180 / M_PI));
	}
	*/

	return 0;
	
}

int get_position_opp_robot(double& i_opp, double& j_opp, double& angle_opp)
{
	double delta_i, delta_j, alpha;
	i_opp = IC_ROBOT_O;
	j_opp = JC_ROBOT_O;

	delta_i = i2_o - i2_b;
	delta_j = j2_o - j2_b;

	alpha = atan2((delta_j), (delta_i));

	angle_opp = alpha;

	/*if ( 0 < alpha < M_PI/2.0) {

		//First quadrant
		angle_opp = alpha*(180.0 / M_PI);
	}

	if (M_PI / 2.0 < alpha < M_PI) {

		//Second quadrant
		angle_opp = alpha*(180.0 / M_PI);
	}

	if (-M_PI / 2.0 < alpha < 0) {

		//Fourth quadrant
		angle_opp = 360 - (angle_opp * (180 / M_PI));
	}

	if (-M_PI < angle_opp < -M_PI / 2.0) {

		//Third quadrant
		angle_opp = 360 - (angle_opp * (180 / M_PI));
	}
*/
	return 0;
	
}

int get_position_front_robot(double& ifront, double& jfront) 
{
	ifront = i1_g;
	jfront = j1_g;
	return 0;
}

int get_position_rear_robot(double& irear, double& jrear)
{
	irear = i1_r;
	jrear = j1_r;
	return 0;
}

int get_position_front_opp_robot(double& ifront_opp, double& jfront_opp)
{
	ifront_opp = i2_o;
	jfront_opp = j2_o;
	return 0;
}

int get_position_rear_opp_robot(double& irear_opp, double& jrear_opp)
{
	irear_opp = i2_b;
	jrear_opp = j2_b;
	return 0;

}