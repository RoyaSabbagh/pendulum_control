/*
 *simulator.h
 *
 *Implementation of ...
 *
 *Author: Roya Sabbagh Novin (sabbaghnovin@gmail.com)
 *
*/


#include "ros/ros.h"
#include <time.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Wrench.h"  //for controller output
#include "pendulum_control/control_input.h" //Custom massage for controller input
#include "visualization_msgs/Marker.h"  	 // visualization message for rviz
#include "visualization_msgs/MarkerArray.h"

class Simulator { //Simulator class definition
	private:
		ros::NodeHandle nh_;  // Node handle
    ros::Time t_;
		ros::Publisher input_publisher_;  // control_input
    ros::Subscriber output_subscriber_;  // control_output
		ros::Publisher viz_publisher_;  // vis_pendulum

    double rod_mass_;
    double rod_length_;
		double pendulum_mass_;
		double theta_;		// pendulum angle
		double theta_dot_;	// pendulum velocity
		double theta_ddot_;  		// pendulum acceleration
		void outputCallback(const geometry_msgs::Wrench &wrench);
	public:
		Simulator(ros::NodeHandle *nodehandle);
		void publishControlInput();
		void publishVizPendulum();
		~Simulator();
};
