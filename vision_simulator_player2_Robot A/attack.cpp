#include <iostream>
#include <fstream>

#include <cmath>
#include <Windows.h>

using namespace std;

// include this header file for basic image transfer functions
#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"
#include "attack.h"

const double PI = 3.14159265;



int attack_path(image& rgb, int& pw_r, int& pw_l, int& laser, double x_robot, double y_robot, double theta, double x_opp_front, double y_opp_front, double x_opp_rear, double y_opp_rear, double opp_theta)
{
	int xc[4];
	int yc[4];
	//d_ol = distance from robot centroid to offense lines
	//ol_theta = angle (atan2) of d_ol
	//d_cl = distance from robot centroid to any of the 4 corners of the window
	double dmin, theta_min, d_ol[2], ol_theta[2];
	//double d_cl[4];

	xc[0] = 50;
	yc[0] = 50;
	xc[1] = 580;
	yc[1] = 50;
	xc[2] = 580;
	yc[2] = 440;
	xc[3] = 50;
	yc[3] = 440;

	int attack_state_front = 0;
	int attack_state_rear = 0;

	theta_min = 0.0;

	bool a = FALSE;

	draw_corners(rgb, xc, yc);

	//Draw the lines connecting from rear and front of enemy robot to our robot's centroid
	offense_line(x_robot, y_robot, x_opp_front, y_opp_front, attack_state_front, rgb);
	offense_line(x_robot, y_robot, x_opp_rear, y_opp_rear, attack_state_rear, rgb);


	//Compute Euclidean distance (two points) and respective angles
	//1st - Front offence line
	//2nd - Rear offence line
	d_ol[0] = calculate_distance(x_robot, y_robot, x_opp_front, y_opp_front);
	d_ol[1] = calculate_distance(x_robot, y_robot, x_opp_rear, y_opp_rear);
	ol_theta[0] = calculate_angle(x_opp_front - x_robot, y_opp_front - y_robot);
	ol_theta[1] = calculate_angle(x_opp_rear - x_robot, y_opp_rear - y_robot);

	//DEBUG FOR RADIANS TO DEGREES
	/*
	cout << "\n\nAngle in RAD of off line FRONT = " << ol_theta[0];
	cout << "\n\nAngle in RAD of off line REAR = " << ol_theta[1];

	convert_rad_to_degrees(ol_theta[0]);
	convert_rad_to_degrees(ol_theta[1]);

	cout << "\n\nAngle in DEG of off line FRONT = " << ol_theta[0];
	cout << "\n\nAngle in DEG of off line REAR = " << ol_theta[1];
	*/

	//Find shortest offense line to direct laser
	shortest_dist(d_ol, 2, ol_theta, dmin, theta_min);

	convert_rad_to_degrees(theta_min);
	convert_rad_to_degrees(theta);

	//cout << "\n\nShortest distance is = " << dmin;
	//cout << "\n\nTarget angle is = " << theta_min;

	if (attack_state_rear == 0 || attack_state_front == 0) {

		//CASE 1: CLEAR SHOT (One of the offense lines are GREEN)
		reach_target_angle(pw_r, pw_l, theta, theta_min, a);
		if (a == TRUE) {
			//laser = 0;
			laser = 1;
		}
	}

	if (attack_state_rear == 1 && attack_state_front == 1) {
		//CASE 2: NO CLEAR SHOT (BOTH of the offense lines are RED i.e. an obstacle is in the way)
		pw_l = 1000;
		pw_r = 2000;
		//Terminate the function. return to next time instant tc to get new positions and angles
		return 0;
	}

	return 0;
}

int reach_target_angle(int& pw_r, int& pw_l, double& my_theta, double& off_theta, bool& laser_setting)
{
	//PID control loop
	//Input variable = off_theta (target angle)
	//Output variable = pw_l and pw_r of robot wheels
	int pw_0 = 1500;

	//target_theta is ec(t) 
	double target_theta;
	double kp = 5.0;

	//Tweak for later
	double EPS = 8.0;

	//cout << "\n\nMy theta =  " << my_theta;
	//cout << "\nOffensive line theta =  " << off_theta;

	int pulse_max = 2000;
	int pulse_min = 1000;
	if (pw_l > pulse_max) pw_l = pulse_max;
	if (pw_r > pulse_max) pw_r = pulse_max;
	if (pw_l < pulse_min) pw_l = pulse_min;
	if (pw_r < pulse_min) pw_r = pulse_min;

	//SCENARIO 1: angle of offensive line is GREATER than my_theta
	//The opponent robot is behind me and facing the opposite way
	if (off_theta > my_theta) {
		target_theta = off_theta - my_theta;
		//cout << "\noffense line angle is GREATER than my_theta";
		//cout << "\nTarget theta  " << target_theta;
		if (target_theta <= EPS) {
			//Send back to fire laser in attack_path()
			laser_setting = TRUE;
			return 0;
		}
		if (target_theta <= PI) {
			//TURN RIGHT
			//U(s) = Kp*ec + Ki*ec + sKd*ec
			//Kd = Ki = 0
			pw_l = pw_0 - (int)(kp * target_theta);
			pw_r = pw_0;
			//cout << "\nTURN RIGHT\n\n";
		}
		else {
			//TURN LEFT
			pw_l = pw_0 + (int)(kp * target_theta);
			pw_r = pw_0 + (int)(kp * target_theta);
			//cout << "\nTURN LEFT\n\n";
		}
	}

	//SCENARIO 2: angle of offensive line is SMALLER than my_theta
	//The opponent robot is in front and parallel to me
	if (off_theta < my_theta) {
		target_theta = my_theta - off_theta;
		//cout << "\n\noffense line angle is SMALLER than my_theta";
		//cout << "\nTarget theta  " << target_theta;
		if (target_theta <= EPS) {
			laser_setting = TRUE;
			return 0;
		}
		if (target_theta <= PI) {
			//TURN LEFT
			pw_l = pw_0 + (int)(kp * target_theta);
			pw_r = pw_0 + (int)(kp * target_theta);
			//cout << "\nTURN LEFT\n\n";
		}
		else {
			//TURN RIGHT
			pw_l = pw_0 - (int)(kp * target_theta);
			pw_r = pw_0;
			//cout << "\nTURN RIGHT\n\n";
		}
	}

	//cout << "\nTarget theta  " << target_theta;
	//cout << "\nPW of left wheel  " << pw_l;
	//cout << "\nPW of right wheel  " << pw_r;
	return 0;
}

int draw_corners(image& rgb, int* xp, int* yp)
{

	for (int i = 0; i < 4; i++) {

		draw_point_rgb(rgb, xp[i], yp[i], 255, 0, 0);
	}

	return 0;
}

double calculate_distance(double xc, double yc, double xc2, double yc2)
{
	double dist;

	dist = sqrt(pow((yc2 - yc), 2) + pow((xc2 - xc), 2));


	return dist;
}

double calculate_angle(double dx, double dy)
{
	//converts from RADIANS to DEGREES
	double theta = 0.0;

	theta = atan2(dy, dx);

	if (dy == 0.0) {
		theta = 0.0;
	}

	return theta;
}

int convert_rad_to_degrees(double& theta_rad)
{
	double theta_temp;
	theta_temp = 0.0;

	if (theta_rad < 0) {

		theta_temp = theta_rad + (2 * PI);
		theta_temp = theta_temp * (180 / PI);
		theta_rad = theta_temp;
	}
	else {
		theta_temp = theta_rad * (180 / PI);
		theta_rad = theta_temp;
	}

	return 0;
}

void shortest_dist(double* d_line, int N_lines, double* theta_line, double& dmin, double& theta_min)
{
	int n = N_lines;
	//double dmin;
	dmin = 100000;
	int Nth_line = 0;

	for (int i = 0; i < n; i++) {
		if (d_line[i] < dmin) {

			dmin = d_line[i];
			theta_min = theta_line[i];
			Nth_line = i;
		}
		//cout << "\nDistance d1 = " << d_line[i];

	}
	//cout << "\nShortest off. line number = " << Nth_line+1;
}




double calculate_position(double x, double y, double& theta) {
	//x & y are inputs, theta is the output angle
	//Note: Theta is calculated in the CCW direction
	if ((x >= 0.0) && (y > 0.0)) theta = atan2(y, x);
	else if ((x < 0.0) && (y > 0.0)) theta = PI - atan2(y, -x);
	else if ((x < 0.0) && (y < 0.0)) theta = PI + atan2(-y, -x);
	else if ((x > 0.0) && (y < 0.0)) theta = 2 * PI - atan2(-y, x);
	else if (y == 0.0) {
		//cout << "\nError, y=0";
		//exit(1);
		theta = 0.0; //Special for sobel edge detection, theta
	}
	return theta;
}

void offense_line(double x, double y, double x_opp, double y_opp, int& stat, image& rgb)
{
	double x0, y0, x1, y1, r, dr, theta, dist;
	int i, j, ip, jp, R, G, B, width, height, k, w, CMAX;
	ibyte* pa;

	dist = measure_dist(x, y, x_opp, y_opp);

	//Allocate width and height
	width = rgb.width;
	height = rgb.height;

	//Allocate pointer to image
	pa = rgb.pdata;

	dr = 1; //Increment for r. Note, r is the length of the laser

	// get start point of the laser
	x0 = x;
	y0 = y;

	//Get end points (front and rear of opponent)
	x1 = x_opp;
	y1 = y_opp;

	calculate_position(x1 - x0, y1 - y0, theta);

	//Initialize CMAX which will measure the max pixel value in the detection range
	CMAX = 0;

	//Initialize stat value to random number. Initialize so that it will not be an undefined value, but use a value that will not me used
	stat = 2;


	for (r = 0 + 75; r < dist - 45; r += dr) {

		ip = (int)(x0 + r * cos(theta));
		jp = (int)(y0 + r * sin(theta));

		// stop loop when (i,j) goes out of range / off screen
		if ((ip < 3) || (ip > rgb.width - 3) || (jp < 3) || (jp > rgb.height - 3)) {
			break;
		}

		//Note: ip and jp are the coordinates of the points to be checked

		//Now, a modified version of draw_point_rgb is called

		//Set size of detection rectangle
		w = 1;

		//Limit i and j values
		if (ip < w) ip = w;
		if (ip > width - w - 1) ip = width - w - 1;
		if (jp < w) jp = w;
		if (jp > height - w - 1) jp = height - w - 1;


		//Detection Pass. Will check the colors of the pixels ahead of the robot
		for (i = -w; i <= w; i++) {
			for (j = -w; j <= w; j++) {
				k = (i + ip) + ((j + jp) * width);
				if (pa[3 * k] > CMAX) CMAX = pa[3 * k]; //Note, since this is a greyscale image, R=G=B which means that doing this is okay
			}
		}

		// set laser colour red if an object is detected
		if (CMAX > 200) {
			R = 255;
			G = 0;
			B = 0;
			stat = 1;
		}

		// set laser color green if no object is detected
		else {
			R = 0;
			G = 255;
			B = 0;
			stat = 0;
		}

		//Color pass (will be removed later)
		for (i = -w; i <= w; i++) {
			for (j = -w; j <= w; j++) {
				k = (i + ip) + ((j + jp) * width);

				pa[3 * k] = B;
				pa[3 * k + 1] = G;
				pa[3 * k + 2] = R;
			}
		}
	}
}

double measure_dist(double x1, double y1, double x2, double y2) {
	double distance, dx, dy;
	dx = abs(x1 - x2);
	dy = abs(y1 - y2);

	distance = sqrt((dx * dx) + (dy * dy));
	return distance;
}