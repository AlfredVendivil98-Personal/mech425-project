#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <cmath>

using namespace std;

#include "image_transfer.h"

#include "vision_cone.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

const double PI = 3.14159265;
int vision_cone::rev = 0;
int vision_cone::t = 0;
int vision_cone::hit = 0;

vision_cone::vision_cone(double x, double y, double theta, double ra, double c_a, int state,image &rgb_in) {
	//Initialize all class variables
	this->x = x;
	this->y = y;
	this->theta = theta;
	this->ra = ra;
	this->c_a = c_a;
	this->state = state; 
	width = 640;
	height = 480;
	w = 0;
	pw_l = 0;
	pw_r = 0;

	//Initialize arrays
	int i;
	for (i = 0; i < 6; i++) {
		b[i] = 0;
		stat[i] = 0;
	}

	//Initialize the rgb image.
	// NOTE: activate_vision is assumed to already have been called. This means that all class calls will only work if they are called after activate_vision
	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	allocate_image(rgb);
	copy(rgb_in,rgb);
}

void vision_cone::draw_cone(int &pw_l, int &pw_r,image &rgb_out) {
	int i, j,k, ip, jp, r, CMAX,R,G,B,is;
	double dr,a;
	ibyte* pa;

	//Initialize pw_l and pw_r
	this->pw_l = pw_l;
	this->pw_r = pw_r;

	//Copy input image to class image
	image_processing_collision(rgb, rgb);
	//copy(rgb, this->rgb);

	//Allocate pointer to image
	pa = rgb.pdata;

	//Increment for r. Note, r is the length of the laser
	dr = 1;

	//Initialize CMAX which will measure the max pixel value in the detection range
	CMAX = 0;

	//Initialize stat value to a random number
	//stat = 2;

	// Object detection
	for (a = -c_a,is=0; a <= c_a; a += c_a,is++) {
		for (r = 75; r < ra; r += dr) {

			ip = (int)(x + r * cos(theta + a));
			jp = (int)(y + r * sin(theta + a));

			//Stop loop when (i,j) gets out of range / off screen
			//Out of bounds region is 30 pixels away from the actual borders of the window to prevent the rear of the robot
			//From going out of bounds while it turns 
			if ((ip < 50) || (ip > width - 50) || (jp < 50) || (jp > height - 50)) {
				b[is] = 1;
				break;
			}
			else b[is] = 0;

			//Set size of detection rectangle
			w = 1;

			//Limit ip and jp values
			if (ip < w) ip = w;
			if (ip > width - w - 1) ip = width - w - 1;
			if (jp < w) jp = w;
			if (jp > height - w - 1) jp = height - w - 1;

			//Detection pass. Will check the intensities of the pixels ahead of the robot
			//If pixels are above threshold value, an object is assumed to be located ahead of the robot
			for (i = -w; i <= w; i++) {
				for (j = -2; j <= w; j++) {
					k = (i + ip) + ((j + jp) * width);
					if (pa[3 * k] > CMAX) CMAX = pa[3 * k]; //Note, since this is a greyscale image, R=G=B which means that doing this is okay
				}
			}

			//Color pass. FOR TESTING PURPOSES ONLY
			// set laser colour red if an object is detected
			if (CMAX > 200) {
				R = 255;
				G = 0;
				B = 0;
				stat[is] = 1;
			}
			// set laser color green if no object is detected
			else {
				R = 0;
				G = 255;
				B = 0;
				stat[is] = 0;
			}

			
			for (i = -w; i <= w; i++) {
				for (j = -w; j <= w; j++) {
					k = (i + ip) + ((j + jp) * width);

					pa[3 * k] = B;
					pa[3 * k + 1] = G;
					pa[3 * k + 2] = R;
				}
				
				CMAX = 0;
			}
		}
	}

	//Robot movement
	// Note: 0: RIGHT LASER, 1: CENTER LASER, 2: LEFT LASER

	//Left hit -> Turn right
	if (((stat[2] == 1) && (stat[0] == 0)) || ((b[2] == 1) && (b[0] == 0))) {
		this->pw_l = 1500;
		this->pw_r = 1000;
		
	}

	//Right hit -> Turn left
	if (((stat[2] == 0) && (stat[0] == 1)) || ((b[2] == 0) && (b[0] == 1)) || ((stat[3] == 1) && (stat[5] == 0)) || ((b[3] == 1) && (b[5] == 0))) {
		this->pw_l = 1500;
		this->pw_r = 2000;
		
	}

	//Center hit -> Move back
	if (((stat[2] == 1) && (stat[0] == 1)) || ((b[2] == 1) && (b[0] == 1))|| (stat[1]==1) || (b[1]==1) || ((b[2]==1) && (stat[0]==1)) || ((stat[2]==1) && (b[0]==1))) {
		rev = 1;
	}
	
	//Reverse while rev=1
	if (rev == 1) {
		vision_line_reverse(ra,10,3); // Reverse for 7 ticks, turn for 3 ticks
	}

	//No hit, keep movement the same
	if (((stat[2] == 0) && (stat[0] == 0)) && (b[2] == 0) && (b[0] == 0) && (rev==0)) {
		this->pw_l = pw_l;
		this->pw_r = pw_r;
	}

	pw_l = this->pw_l;
	pw_r = this->pw_r;

	copy(rgb, rgb_out);
}


void vision_cone::vision_line_reverse(int ra,int t1, int t2)
{
	int i, j, k, ip, jp, r, CMAX, R, G, B, is;
	double a,dr;
	ibyte* pa;

	//Allocate pointer to image
	pa = rgb.pdata;
	
	//Reset CMAX to 0
	CMAX = 0;

	//Increment for r. Note, r is the length of the laser
	dr = 1;

	for (a = -c_a,is=3; a <= c_a; a += c_a,is++) {
		for (r = 75; r < ra; r += dr) {

			ip = (int)(x + r * cos(theta - PI+a));
			jp = (int)(y + r * sin(theta - PI+a));

			//Stop loop when (i,j) gets out of range / off screen
			//The reverse out of bounds range is closer to the borders of the window than the forwards vision cone
			//Because we don't have to be as strict with the reverse movement. 
			if ((ip < 3) || (ip > width - 3) || (jp < 3) || (jp > height - 3)) {
				b[is] = 1;
				break;
			}
			else b[is] = 0;
			

			//Set size of detection rectangle
			w = 1;

			//Limit ip and jp values
			if (ip < w) ip = w;
			if (ip > width - w - 1) ip = width - w - 1;
			if (jp < w) jp = w;
			if (jp > height - w - 1) jp = height - w - 1;

			//Detection pass. Will check the intensities of the pixels ahead of the robot
			//If pixels are above threshold value, an object is assumed to be located ahead of the robot
			for (i = -w; i <= w; i++) {
				for (j = -2; j <= w; j++) {
					k = (i + ip) + ((j + jp) * width);
					if (pa[3 * k] > CMAX) CMAX = pa[3 * k]; //Note, since this is a greyscale image, R=G=B which means that doing this is okay
				}
			}

			//Color pass. FOR TESTING PURPOSES ONLY
			// set laser colour red if an object is detected
			if (CMAX > 200) {
				R = 255;
				G = 0;
				B = 0;
				stat[is] = 1;
			}
			// set laser color green if no object is detected
			else {
				R = 0;
				G = 255;
				B = 0;
				stat[is] = 0;
			}

			for (i = -w; i <= w; i++) {
				for (j = -w; j <= w; j++) {
					k = (i + ip) + ((j + jp) * width);

					pa[3 * k] = B;
					pa[3 * k + 1] = G;
					pa[3 * k + 2] = R;
				}
				CMAX = 0;
			}
		}
	}
	t++; //Increment time variable during reverse

	if (t < t1) {
		//Main movement -> Reverse
		this->pw_l = 2000;
		this->pw_r = 1000;

		//Backup movement in case of collision
		//Left hit -> Turn left
		if (((stat[3] == 0) && (stat[5] == 1)) || ((b[3] == 0) && (b[5] == 1))) {
			this->pw_l = 1000;
			this->pw_r = 1000;
		}

		//Right hit -> Turn right
		if (((stat[3] == 1) && (stat[5] == 0)) || ((b[3] == 1) && (b[5] == 0))) {
			this->pw_l = 2000;
			this->pw_r = 2000;
		}

		//Center hit -> Move forward
		if ((stat[4] == 1) || (b[4] == 1)) {
			this->pw_l = 1000;
			this->pw_r = 2000;
			rev = 0;
			t = 0;
		}
	}

	else if ((t > t1) && (t < (t1+t2))) {
		//Main movement state 1 -> Turn CW
		if (state == 1) {
			this->pw_l = 1000;
			this->pw_r = 1000;
		}
		//Main movement state 2 -> Turn CCW
		else if (state == 0) {
			this->pw_l = 2000;
			this->pw_r = 2000;
		}
	
		//Backup movement in case of collision
		//Left hit -> Turn left
		if (((stat[3] == 0) && (stat[5] == 1)) || ((b[3] == 0) && (b[5] == 1))) {
			this->pw_l = 1000;
			this->pw_r = 1000;
		}

		//Right hit -> Turn right
		if (((stat[3] == 1) && (stat[5] == 0)) || ((b[3] == 1) && (b[5] == 0))) {
			this->pw_l = 2000;
			this->pw_r = 2000;
		}

		//Center hit -> Move forward
		if ((stat[4] == 1) || (b[4] == 1)) {
			this->pw_l = 1000;
			this->pw_r = 2000;
			rev = 0;
			t = 0;
		}
	}
	else if (t > (t1+t2)) {
		rev = 0;
		t = 0;
	}
}
/*
void vision_cone::stuck_check(double x_front, double y_front) {
	static double x_fr1, y_fr1, x_fr2, y_fr2;
	double dx, dy,dist;
	t1++; //Increment counter

	if (t1 % 2 == 0) {
		x_fr1 = x_front;
		y_fr1 = y_front;
	}
	if (t1 % 2 != 0) {
		x_fr2 = x_front;
		y_fr2 = y_front;
	}
	dx = x_fr2 - x_fr1;
	dy = y_fr2 - y_fr1;
	dist = sqrt((dx * dx) + (dy * dy));

	cout << "\ndx=" << dx << "\tdy=" << dy << "\tdist=" << dist;
}
*/

void vision_cone::image_processing_collision(image& a, image& b) {
	//Assuming image a and image b are already allocated
	image grey1, grey2;

	grey1.type = GREY_IMAGE;
	grey1.width = a.width;
	grey1.height = a.height;

	grey2.type = GREY_IMAGE;
	grey2.width = a.width;
	grey2.height = a.height;

	allocate_image(grey1);
	allocate_image(grey2);

	//Convert to greyscale
	copy(a, grey1);

	//Scale image
	scale(grey1, grey2);
	copy(grey2, grey1);

	//Invert image
	invert(grey1, grey2);
	copy(grey2, grey1);

	//Threshold image
	threshold(grey1, grey2, 60);

	//Dialate image
	dialate(grey2, grey1);

	//Copy to output
	copy(grey1, b);

	free_image(grey1);
	free_image(grey2);
}

vision_cone:: ~vision_cone() {
	free_image(rgb);
}