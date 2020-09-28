#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <iostream>
#include <math.h>
#include <chrono>


using namespace std;

vector<geometry_msgs::msg::Point> buffer;
geometry_msgs::msg::Point init, pub_point;
rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub;

float max_dist_threshold = 2.5;
unsigned int counter_filter_threshold = 9;
unsigned int bfr_size = 10;

bool compare_point(geometry_msgs::msg::Point x, geometry_msgs::msg::Point y) { return (x.z<y.z); } 

void sorting (vector<geometry_msgs::msg::Point> &r){
	std::sort(r.begin(), r.end(), compare_point);
}

void camerafilterCB(const geometry_msgs::msg::PoseArray::SharedPtr targets_array){
	unsigned int empty_count = 0;
	vector<geometry_msgs::msg::Point> r;
	geometry_msgs::msg::Point distance;
	bool data_available = false;
	r.assign(1,init);
	
	// Finds all the available targets, loop break immediately if poses is empty
	for(std::vector<geometry_msgs::msg::Pose>::const_iterator iti = targets_array->poses.begin(); iti != targets_array->poses.end(); ++iti)
	{
		float dist = sqrt( pow(iti->position.x,2) + pow(iti->position.y,2) );
		if (dist <= max_dist_threshold) 
		{
			distance.x = iti->position.x;
			distance.y = iti->position.y;
			distance.z = dist;
			r.push_back(distance);
			data_available = true;
		}
	}
	//DEBUG std::cout << r.front().z << endl;
	
	if( data_available ) { r.erase(r.begin()); }	// delete the zero 
	
	if(r.size() > 1)
	{
		sorting(r);
	}

	buffer.push_back(r.front());	// push minimum to the back of the buffer
	
	if( buffer.size() < bfr_size ) 
	{
		// initial state, do nothing
	} else
	{
		pub_point = init;
		buffer.erase( buffer.begin() );
		
		// count how many item in the buffer are empty
		for(std::vector<geometry_msgs::msg::Point>::iterator it = buffer.begin(); it != buffer.end(); ++it) 
		{
			if (it->z == 0) 
			{
				empty_count++;
			}
		}
		
		if (empty_count <= counter_filter_threshold) 	// if there's enough non-empty item
		{
			for(std::vector<geometry_msgs::msg::Point>::iterator it = buffer.end(); it != buffer.begin(); --it)
			{
				if (it->z != 0) {
					pub_point = *it;	// find the last non- zero item
					break;
				}
			}
		}
		
		position_pub->publish(pub_point);
	}
	return;
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	init.x=0;
	init.y=0;
	init.z=0;
	auto n = rclcpp::Node::make_shared("target_filter");
	if( bfr_size < counter_filter_threshold) {
		RCLCPP_ERROR(n->get_logger(),"Buffer size %d is smaller than threshold %d, ending!", bfr_size, counter_filter_threshold);
		return 1;
	}
    
	n->get_parameter("max_target_dist", max_dist_threshold);
	n->get_parameter("target_filter_threshold", counter_filter_threshold);
	n->get_parameter("target_filter_buffer_size", bfr_size);
	if( bfr_size < counter_filter_threshold) {
		RCLCPP_ERROR(n->get_logger(),"Buffer size %d is smaller than threshold %d, ending!", bfr_size, counter_filter_threshold);
		return 1;
	}
	/*
	n->set_parameter(rclcpp::Parameter("max_target_dist",2.5));
	n->set_parameter(rclcpp::Parameter("target_filter_threshold",9));
	n->set_parameter(rclcpp::Parameter("target_filter_buffer_size",10));
	*/
	position_pub = n->create_publisher<geometry_msgs::msg::Point>("tracking_target", 1000);
	auto sub = n->create_subscription<geometry_msgs::msg::PoseArray>("ai_targets",1000, camerafilterCB);
	rclcpp::spin(n);
	return 0;
}
