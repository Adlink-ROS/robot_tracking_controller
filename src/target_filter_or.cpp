#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <algorithm>

using namespace std;

vector<geometry_msgs::Point> buffer;
geometry_msgs::Point init, pub_point;
ros::Publisher position_pub;

float max_dist_threshold = 3.5;
int counter_filter_threshold = 9;
int bfr_size = 10;

bool compare_point(geometry_msgs::Point x, geometry_msgs::Point y) { return (x.z<y.z); } 

void sorting (vector<geometry_msgs::Point> &r){
	std::sort(r.begin(), r.end(), compare_point);
}

void camerafilterCB(const geometry_msgs::PoseArray::ConstPtr& targets_array){
	int empty_count = 0;
	vector<geometry_msgs::Point> r;
	geometry_msgs::Point distance;
	bool data_available = false;
	r.assign(1,init);
	
	// Finds all the available targets, loop break immediately if poses is empty
	for(std::vector<geometry_msgs::Pose>::const_iterator iti = targets_array->poses.begin(); iti != targets_array->poses.end(); ++iti)
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
		for(std::vector<geometry_msgs::Point>::iterator it = buffer.begin(); it != buffer.end(); ++it) 
		{
			if (it->z == 0) 
			{
				empty_count++;
			}
		}
		
		if (empty_count <= counter_filter_threshold) 	// if there's enough non-empty item
		{
			for(std::vector<geometry_msgs::Point>::iterator it = buffer.end(); it != buffer.begin(); --it)
			{
				if (it->z != 0) {
					pub_point = *it;	// find the last non- zero item
					break;
				}
			}
		}
		
		position_pub.publish(pub_point);
	}
	return;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "target_filter");
	init.x=0;
	init.y=0;
	init.z=0;
	ros::NodeHandle n;

	n.param<float>("max_target_dist", max_dist_threshold, 2.5);
	n.param<int>("target_filter_threshold", counter_filter_threshold, 9);
	n.param<int>("target_filter_buffer_size", bfr_size, 10);
	if( bfr_size < counter_filter_threshold) {
		ROS_ERROR("Buffer size %d is smaller than threshold %d, ending!", bfr_size, counter_filter_threshold);
		return 1;
	}

	n.setParam("max_target_dist", max_dist_threshold);
	n.setParam("target_filter_threshold", counter_filter_threshold);
	n.setParam("target_filter_buffer_size", bfr_size);
	
	position_pub = n.advertise<geometry_msgs::Point>("tracking_target", 1000);
	ros::Subscriber sub = n.subscribe("ai_targets",1000, camerafilterCB);
	ros::spin();
	return 0;
}
