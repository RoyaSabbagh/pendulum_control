/*
 *file_name.py
 *
 *Implementation of ...
 *
 *Author: Roya Sabbagh Novin (sabbaghnovin@gmail.com)
 *
*/

#include "controller.h"
#define PI 3.1415

Controller::Controller(ros::NodeHandle *nodehandle):
		Kp_(10.0),
		Kd_(1.0),
		theta_des_(PI),
		nh_(*nodehandle)
{
	output_publisher_ = nh_.advertise<geometry_msgs::Wrench>("/control_output", 100);
	input_subscriber_ = nh_.subscribe("/control_input", 100, &Controller::controlInputCallback, this);
}

void Controller::controlInputCallback(const pendulum_control::control_input &input)
{
	geometry_msgs::Wrench wrench;
	wrench.torque.z = 0;
	output_publisher_.publish(wrench);
}

Controller::~Controller(){

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pendulumController");
	ros::NodeHandle nh("~");
	Controller controller(&nh);
	ros::spin();
	return 0;
}
