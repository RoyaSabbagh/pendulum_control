/*
 *file_name.py
 *
 *Implementation of ...
 *
 *Author: Roya Sabbagh Novin (sabbaghnovin@gmail.com)
 *
*/

#include "simulator.h"
#include <ros/console.h>
#define PI 3.1415


double wrapAngel(double theta){ //This is to wrap an angle to make sure it is in the range of [-pi, pi]
    if (theta < -PI) theta += 2*PI;
    else if (theta > PI) theta -= 2*PI;
    return theta;
}

Simulator::Simulator(ros::NodeHandle *nodehandle):
    rod_mass_(0.2),
    rod_length_(1.0),
		pendulum_mass_(1.0),
		theta_(PI/2),
		theta_dot_(0.0),
		theta_ddot_(0.0),
    mu_f_(0.2),
		nh_(*nodehandle)
{
	t_ = ros::Time::now();
	momentOfInertia_ = rod_length_*rod_length_*(pendulum_mass_+rod_mass_/3);
	viz_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/pendulum_viz", 100);
  input_publisher_ = nh_.advertise<pendulum_control::control_input>("/control_input", 100);
	output_subscriber_ =nh_.subscribe("/control_output", 100, &Simulator::controlOutputCallback, this);
}

void Simulator::publishVizPendulum(){
  visualization_msgs::MarkerArray  markerArray;

  // Mass visualization as a sphere
  visualization_msgs::Marker marker_mass;

	double xMass_;
	double yMass_;
	xMass_ = rod_length_*sin(theta_);
	yMass_ = -rod_length_*cos(theta_);

	marker_mass.header.frame_id = "/world";
	marker_mass.header.stamp = ros::Time::now();
  marker_mass.id = 0;
	marker_mass.ns = "invertated_pendulum";
	marker_mass.type = visualization_msgs::Marker::SPHERE;

  marker_mass.color.r = 0.0f;
  marker_mass.color.g = 0.5f;
  marker_mass.color.b = 0.5f;
  marker_mass.color.a = 1.0;

  marker_mass.scale.x = .1;
  marker_mass.scale.y = .1;
  marker_mass.scale.z = .1;

	marker_mass.pose.position.x = xMass_;
	marker_mass.pose.position.y = yMass_;
	marker_mass.pose.position.z = 0;
	marker_mass.pose.orientation.x = 0.0;
	marker_mass.pose.orientation.y = 0.0;
	marker_mass.pose.orientation.z = 0.0;
	marker_mass.pose.orientation.w = 1.0;

  marker_mass.lifetime = ros::Duration();
  markerArray.markers.push_back(marker_mass);

  // rod visualization as a cylinder
  visualization_msgs::Marker marker_rod;

  double xRod_;
	double yRod_;
  xRod_ = rod_length_*sin(theta_)/2 ;
	yRod_ = -rod_length_*cos(theta_)/2;

	marker_rod.header.frame_id = "/world";
	marker_rod.header.stamp = ros::Time::now();
	marker_rod.ns = "invertated_pendulum";
	marker_rod.id = 1;
	marker_rod.type = visualization_msgs::Marker::CYLINDER;

  marker_rod.color.r = 0.0f;
  marker_rod.color.g = 0.7f;
  marker_rod.color.b = 0.7f;
  marker_rod.color.a = 8.0;

  marker_rod.scale.x = .01;
  marker_rod.scale.y = .01;
  marker_rod.scale.z = rod_length_;

	marker_rod.pose.position.x = xRod_;
	marker_rod.pose.position.y = yRod_;
	marker_rod.pose.position.z = 0;
	tf2::Quaternion q;
	q.setRPY(PI/2, 0, theta_);
	tf2::convert(q, marker_rod.pose.orientation);

  marker_rod.lifetime = ros::Duration();
  markerArray.markers.push_back(marker_rod);

  viz_publisher_.publish(markerArray);
}

void Simulator::publishControlInput(){
	pendulum_control::control_input input;
	input.theta = theta_;
	input.theta_dot = theta_dot_;
	input.theta_ddot = theta_ddot_;
	input_publisher_.publish(input);
}

void Simulator::controlOutputCallback(const geometry_msgs::Wrench &wrench)
{
	ros::Duration dt = ros::Time::now() - t_;
	theta_ddot_ = - rod_length_*(pendulum_mass_+rod_mass_/2) * 9.81 * sin(theta_) / momentOfInertia_ - mu_f_ * theta_dot_ / momentOfInertia_;
	theta_dot_ = theta_dot_ + theta_ddot_ * dt.toSec();
	theta_ = wrapAngel(theta_ + theta_dot_ * dt.toSec());
	t_ = ros::Time::now();
}

Simulator::~Simulator(){

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pendulumSimulator");
	ros::NodeHandle nh("~");;
	Simulator simluator(&nh);
	ros::Rate loop_rate(1000);
	while(ros::ok())
	{
    simluator.publishControlInput();
		simluator.publishVizPendulum();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
