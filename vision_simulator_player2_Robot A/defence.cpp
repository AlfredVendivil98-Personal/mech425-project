#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <cmath>

using namespace std;



void calculate_MyTargetPosition(double my_x, double my_y, double my_theta, double op_x, double op_y, double op_theta, double obstacle_x[], double obstacle_y[], int N_Obstacle, int& pw_l, int& pw_r, double& destination_x, double& destination_y) {
	double min_distance, distance, theta, obstacle_radius, distance_x, distance_y, heading_angle, total_distance, desired_theta, distance_normal, distance_parallel, kp, kn;
	int selected_obj;
	int pulse_min, pulse_max;

	static int x_obs, y_obs, init = 1;

	selected_obj = 1;
	double PI = 3.14159265;

	//find obstacle closest to oponent and my robot
	min_distance = 10000;
	selected_obj = 1;
	if (init == 1) {


		for (int i = 0; i < N_Obstacle; i++) {
			distance = sqrt(pow(my_x - obstacle_x[i], 2) + pow(my_y - obstacle_y[i], 2));//sqrt(pow(op_x - obstacle_x[i], 2) - pow(op_y - obstacle_y[i], 2))+
			if (distance < min_distance) {
				min_distance = distance;
				selected_obj = i;
			}

		}
		init = 0;
		//cout << "check distance";
		x_obs = obstacle_x[selected_obj];
		y_obs = obstacle_y[selected_obj];

	}




	theta = atan2(y_obs - op_y, x_obs - op_x);

	obstacle_radius = 80;

	destination_x = x_obs + obstacle_radius * cos(theta);
	destination_y = y_obs + obstacle_radius * sin(theta);



	//cout << "\nobj x, destination x, my x: " << obstacle_x[selected_obj] << " " << destination_x << " " << my_x;
	//cout << "\nobj y, destination y, my y: " << obstacle_y[selected_obj] << " " << destination_y << " " << my_y;


	heading_angle = atan2(destination_y - my_y, destination_x - my_x) - my_theta;


	//calculate parallel and normal distance between my robot and destination
	distance_x = destination_x - my_x;
	distance_y = destination_y - my_y;

	distance_parallel = distance_x * cos(my_theta) + distance_y * sin(my_theta);
	distance_normal = -distance_x * sin(my_theta) + distance_y * cos(my_theta);

	total_distance = sqrt(pow(distance_x, 2) + pow(distance_y, 2));

	if (total_distance > 10) {
		if (total_distance > 70) {
			kp = 8;
			kn = 12;
		}
		else {
			kp = 2.5;
			kn = 8;
		}


		pw_l = 1500 - kp * distance_parallel + kn * distance_normal;

		pw_r = 1500 + kp * distance_parallel + kn * distance_normal;
	}
	else {
		if (op_theta >= 0) {
			desired_theta = op_theta - PI;
		}
		else {
			desired_theta = PI - op_theta;
		}

		kp = 50;

		pw_l = 1500 - kp * (desired_theta - my_theta);

		pw_r = 1500 - kp * (desired_theta - my_theta);

		//cout << "\nParked";
	}

	pulse_max = 2000;
	pulse_min = 1000;
	if (pw_l > pulse_max) pw_l = pulse_max;
	if (pw_r > pulse_max) pw_r = pulse_max;
	if (pw_l < pulse_min) pw_l = pulse_min;
	if (pw_r < pulse_min) pw_r = pulse_min;
}


