/*
 *file_name.py
 *
 *Implementation of ...
 *
 *Author: Roya Sabbagh Novin (sabbaghnovin@gmail.com)
 *
*/

#include "simulator.h"
#define PI 3.14159

Simulator::Simulator(ros::NodeHandle *nodehandle):
    rod_mass_(0.1),
    rod_length_(1.0),
		pendulum_mass_(0.5),
		theta_(0.0),
		theta_dot_(0.0),
		theta_ddot_(0.0),
		nh_(*nodehandle)
{
	t_ = ros::Time::now();
	viz_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/pendulum_viz", 100);
}


void Simulator::publishVizPendulum(){
  visualization_msgs::MarkerArray  markerArray;

  // Mass visualization as a sphere
  visualization_msgs::Marker marker_mass;

	double xMass_;
	double yMass_;
	xMass_ = rod_length_*cos(theta_);
	yMass_ = rod_length_*sin(theta_);

	marker_mass.header.frame_id = "/world";
	marker_mass.header.stamp = ros::Time::now();
  marker_mass.id = 0;
	marker_mass.ns = "invertated_pendulum";
	marker_mass.type = visualization_msgs::Marker::SPHERE;

  marker_mass.color.r = 0.5f;
  marker_mass.color.g = 0.0f;
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
  xRod_ = rod_length_*cos(theta_)/2 ;
	yRod_ = rod_length_*sin(theta_)/2;

	marker_rod.header.frame_id = "/world";
	marker_rod.header.stamp = ros::Time::now();
	marker_rod.ns = "invertated_pendulum";
	marker_rod.id = 1;
	marker_rod.type = visualization_msgs::Marker::CYLINDER;

  marker_rod.color.r = 1.0f;
  marker_rod.color.g = 0.0f;
  marker_rod.color.b = 0.0f;
  marker_rod.color.a = 1.0;

  marker_rod.scale.x = .01;
  marker_rod.scale.y = .01;
  marker_rod.scale.z = rod_length_;

	marker_rod.pose.position.x = xRod_;
	marker_rod.pose.position.y = yRod_;
	marker_rod.pose.position.z = 0;
	tf2::Quaternion q;
	q.setRPY(PI/2, 0, PI/2+theta_);
	tf2::convert(q, marker_rod.pose.orientation);

  marker_rod.lifetime = ros::Duration();
  markerArray.markers.push_back(marker_rod);

  viz_publisher_.publish(markerArray);
}

Simulator::~Simulator(){

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pendulumSimulator");
	ros::NodeHandle nh("~");;
	Simulator simluator(&nh);
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		simluator.publishVizPendulum();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
