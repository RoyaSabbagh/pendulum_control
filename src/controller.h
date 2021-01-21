/*
 *file_name.py
 *
 *Implementation of ...
 *
 *Author: Roya Sabbagh Novin (sabbaghnovin@gmail.com)
 *
*/

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"  //for controller output
#include "pendulum_control/control_input.h"

class Controller //Controller class definition
{
	private:
		ros::NodeHandle nh_;
		ros::Publisher output_publisher_;
		ros::Subscriber input_subscriber_;

		double theta_des_;
		double Kp_;
		double Kd_;
		void controlInputCallback(const pendulum_control::control_input &input);

	public:
		Controller(ros::NodeHandle *nodehandle);
		~Controller();
};
