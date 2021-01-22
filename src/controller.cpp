/*
 *controller.cpp
 *
 *Implementation of PD contoller for an inverted pendulum.
 *
 *Author: Roya Sabbagh Novin (sabbaghnovin@gmail.com)
 *
*/

#include "controller.h"
#define PI 3.1415

double wrapAngel(double theta){ //This is to wrap an angle to make sure it is in the range of [-pi, pi]
    if (theta < -PI) theta += 2*PI;
    else if (theta > PI) theta -= 2*PI;
    return theta;
}

Controller::Controller(ros::NodeHandle *nodehandle):
		Kp_(10.0), //larger Kp results in more overshoot, smaller Kp results in steady state error
		Kd_(2.0), //larger Kd results in overdamping, smaller Kd results in oscillation
		theta_des_(PI),
		nh_(*nodehandle)
{
  nh_.getParam("K_p", Kp_);
	nh_.getParam("K_d", Kd_);
	nh_.getParam("desired_angel", theta_des_);
	output_publisher_ = nh_.advertise<geometry_msgs::Wrench>("/control_output", 10);
	input_subscriber_ = nh_.subscribe("/control_input", 10, &Controller::controlInputCallback, this);
}

void Controller::controlInputCallback(const pendulum_control::control_input &input)
{

  double err = wrapAngel(theta_des_ - input.theta);
	double d_err =  -input.theta_dot;
	double u = Kp_*err + Kd_*d_err;

	geometry_msgs::Wrench wrench;
	wrench.torque.z = u;
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
