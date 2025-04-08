//////////////////////////////////////////////////PLAYER 2//////////////////////////////////////////////////////////
/////////////////////////////////////////////////ROBOT A///////////////////////////////////////////////////////////

#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

//Header files for MODIFIED computer vision functions (PROJECT)
#include "object_tracking.h"
#include "vision_cone.h"

//Header files for robot motion (PLAYER 1/PLAYER 2)
#include "attack.h"
#include "defence.h"


extern robot_system S1;

const double PI = 3.14159265;

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	
	//////////////////////////////////////////////////DR.GORDON'S PROGRAM//////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	N_obs  = 2;

	x_obs[1] = 270; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 135; // pixels
	y_obs[2] = 135; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////
	
	D = 121.0; // distance between front wheels (pixels)
	
	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
	
	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file
	
	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;
	
	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 2;
	level = 1;
	set_simulation_mode(mode,level);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 =300;
	y0 = 400;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
//	x0 = 150;
//	y0 = 375;
//	theta0 = 3.14159/4;
//	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1250; // pulse width for left wheel servo (us)
	pw_r = 2000; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

	// NOTE: for two player mode you shouldn't set the opponent inputs 
	// opponent inputs
//	pw_l_o = 1300; // pulse width for left wheel servo (us)
//	pw_r_o = 1600; // pulse width for right wheel servo (us)
//	pw_laser_o = 1500; // pulse width for laser servo (us)
//	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
//	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
//				opponent_max_speed);

	//////////////////////////////////////////////////OUR PROGRAM//////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	////////////////////////////////////////ROBOT ASSIGNMENT CONVENTION////////////////////////////////////////////////////
	///////////////////////////////ROBOT A (RED/GREEN) -> IC_ROBOT, JC_ROBOT, ETC./////////////////////////////////////////
	///////////////////////////////ROBOT B (BLUE/YELLOW) -> IC_ROBOT_OPP, JC_ROBOT_OPP, ETC.///////////////////////////////

	
	image rgb, rgb_dup, rgb_col;
	int height, width, n_obs, th;
	double p_obs_x[10], p_obs_y[10];
	double ic_robot, jc_robot;
	double ic_robot_opp, jc_robot_opp;
	th = 255;

	//TEMP VARIABLES
	int b, stat;
	int state = 0;

	//FOR OFFENSE ALGORITHM
	double i_front, j_front, i_rear, j_rear, i_front_opp, j_front_opp, i_rear_opp, j_rear_opp;
	i_front = j_front = i_rear = j_rear = i_front_opp = j_front_opp = i_rear_opp = j_rear_opp = 0.0;

	//FOR DEFENSE ALGORITHM
	double destination_x, destination_y;
	ibyte R, G, B;

	//Kept in RADIANS to be used with other <cmath> functions (e.g. sin() and cos())
	double theta, theta_opp;
	theta = theta_opp = 0.0;

	//Initialize array to store x and y position of the obstacles (MAX OBS in SIM = 3)
	//Positions of first object are stored at element 0 of p_obs_x and p_obs_y

	for (int i = 0; i < 10; i++) {
		p_obs_x[i] = 0.0;
		p_obs_y[i] = 0.0;
	}

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	rgb_dup.type = RGB_IMAGE;
	rgb_dup.width = width;
	rgb_dup.height = height;

	rgb_col.type = RGB_IMAGE;
	rgb_col.width = width;
	rgb_col.height = height;

	// allocate memory for the images
	allocate_image(rgb);
	allocate_image(rgb_dup);
	allocate_image(rgb_col);

	join_player();

	// measure initial clock time
	tc0 = high_resolution_time(); 

	while(1) {
		
		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;

		//////////////////////////////////////////COMPUTER VISION////////////////////////////////////////////////////

		//Initialize obstacle detection and store x-y locations in arrays
		obstacle_detection_tracking(rgb, p_obs_x, p_obs_y, n_obs, height, width);

		//Initialize robot detection and return centroid positions (xc,yc,theta)
		//as well as front/rear positions like shown below

		robot_detection_tracking(rgb, height, width);

		get_position_robot(ic_robot, jc_robot, theta);
		get_position_opp_robot(ic_robot_opp, jc_robot_opp, theta_opp);

		get_position_front_robot(i_front, j_front);
		get_position_rear_robot(i_rear, j_rear);
		get_position_front_opp_robot(i_front_opp, j_front_opp);
		get_position_rear_opp_robot(i_rear_opp, j_rear_opp);

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////


		//////////////////////////////////////////ATTACK ALGORITHM///////////////////////////////////////////////////
		
		//Image processing FOR OFFENSE
		//label_rgb_collision(rgb_dup);

		////////////attack robot A as main////////////////////
		//attack_path(rgb_dup, pw_r, pw_l, laser, ic_robot, jc_robot, theta, i_front_opp, j_front_opp, i_rear_opp, j_rear_opp, theta_opp);

		//////////////attack robot B as main//////////////////
		//attack_path(rgb_dup, pw_r, pw_l, laser, ic_robot_opp, jc_robot_opp, theta_opp, i_front, j_front, i_rear, j_rear, theta);

		//Debugging and demonstration purposes, NOT needed for code to run
		//Front offense line cnnnecting OUR robot's centroid to OPP front marker
		//offense_line(ic_robot, jc_robot, i_front_opp, j_front_opp, stat, rgb_dup);

		//Front offense line cnnnecting OUR robot's centroid to OPP rear marker
		//offense_line(ic_robot, jc_robot, i_rear_opp, j_rear_opp, stat, rgb_dup);

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		//////////////////////////////////////////DEFENSE ALGORITHM//////////////////////////////////////////////////

		////////////defense robot A as main////////////////////
		calculate_MyTargetPosition(ic_robot, jc_robot, theta, i_front_opp, j_front_opp, theta_opp, p_obs_x, p_obs_y, n_obs, pw_l, pw_r, destination_x, destination_y);

		//////////////defense robot B as main//////////////////
		//calculate_MyTargetPosition(ic_robot_opp, jc_robot_opp, theta_opp, i_front, j_front, theta, p_obs_x, p_obs_y, n_obs, pw_l, pw_r, destination_x, destination_y);

		
		//Draw point (testing)
		R = 255;
		G = 255;
		B = 0;
		draw_point_rgb(rgb, (int)destination_x, (int)destination_y, R, G, B);
		

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////COLLISION DETECTION////////////////////////////////////////////////

		//vision cone (or FOV) constructor arguments
		//80 - radius of cone
		//PI/6 - arc length of cone (FOV)
		//state 1 or 0 - predefined path of robot
		//1 - CCW
		//0 - CW

		////////////Draw cone A Robot////////////////////
		vision_cone v1(ic_robot, jc_robot, theta, 80, PI / 6, 1, rgb);
		v1.draw_cone(pw_l, pw_r, rgb_col);

		////////////Draw cone B Robot////////////////////
		//vision_cone v1(ic_robot_opp, jc_robot_opp, theta_opp, 80, PI / 6, 1, rgb);
		//v1.draw_cone(pw_l, pw_r, rgb_col);

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		set_inputs(pw_l,pw_r,pw_laser,laser,
			light,light_gradient,light_dir,image_noise,
			max_speed,opponent_max_speed);

		// manually set opponent inputs for the simulation
		// -- good for testing your program
//		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
//					opponent_max_speed);

		// NOTE: only one program can call view_image()
		//view_rgb_image(rgb);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);
	free_image(rgb_col);
	free_image(rgb_dup);

	deactivate_vision();
	
	deactivate_simulation();	

	//Deactivating ODT = Object Detection Tracking 
	deactivate_odt();
	
	cout << "\ndone.\n";

	return 0;
}
