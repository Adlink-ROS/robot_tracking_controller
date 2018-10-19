	
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/BoundingBox2D.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"

//EWING from leg_tracker.msg import Person, PersonArray, Leg, LegArray 
//#include <stdlib.h>

#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
using namespace cv;

static geometry_msgs::Pose2D center;
static std::vector<vision_msgs::BoundingBox2D> bbox;
static std::vector<vision_msgs::BoundingBox2D> t;
static std::vector<vision_msgs::Detection2D>::const_iterator object_iter; 
static tf::TransformListener *tf_listener; 
static ros::Publisher point_pub;
static ros::Publisher ai_targets_pub;
static ros::Publisher marker_pub;
static geometry_msgs::Point object_position;
static geometry_msgs::PoseArray ai_targets;
static geometry_msgs::PoseWithCovariance pose;
static visualization_msgs::Marker line_object, person_marker;

/* see labelmap.pbtxt for definition*/
enum cloth_type {
    SHIRTS = 1,
    POLO   = 2,
    TSHIRT = 3,
    SUIT   = 4
};
cloth_type target_cloth = TSHIRT;
double target_threshold = 0.85;

void callbackROI(const vision_msgs::Detection2DArray::ConstPtr& data)
{
  bbox.clear();
  int count = 0;
  vision_msgs::Detection2D detection;
  for (object_iter = data->detections.begin(); object_iter != data->detections.end(); ++object_iter)
  {
    if( (object_iter->results[0].id == target_cloth) && (object_iter->results[0].score >= target_threshold) )
	{  
      bbox.push_back(object_iter->bbox);
    count = ++count;
    }
  }
  //ROS_INFO("Shirts number:%d",count);
}


void callbackPointCloud(const sensor_msgs::PointCloud2 &input)
{
	sensor_msgs::PointCloud2 pc_footprint;
	bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
	if(res == false)
	{
		ROS_ERROR("No transform from \"/base_footprint\" to \"%s\"", input.header.frame_id.c_str());
		return;
	}
	pcl_ros::transformPointCloud("/camera_link", input, pc_footprint, *tf_listener);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
	pcl::fromROSMsg(pc_footprint , cloud_src); 

	int marker_id = 0;
	line_object.points.clear();
	line_object.header.frame_id = "/camera_link";
	line_object.ns = "line_object";
	line_object.action = visualization_msgs::Marker::ADD;
	line_object.id = marker_id;
	marker_id++;
	line_object.type = visualization_msgs::Marker::LINE_LIST;
	line_object.scale.x = 0.01;
	line_object.color.r = 1.0;
	line_object.color.g = 0;
	line_object.color.b = 1.0;
	line_object.color.a = 1.0;
	geometry_msgs::Point p;
	std::vector<vision_msgs::BoundingBox2D> t;

	geometry_msgs::PoseArray ai_targets;
	ai_targets.poses.clear();
	ai_targets.header.frame_id = "odom";

	int iSwitch,count;
	do{
		iSwitch = 0;
		for(int i = 0; i < count; i++)
		{
			if(bbox[i].center.x > bbox[i+1].center.x)
			{
				t[i] = bbox[i] ; bbox[i] = bbox[i+1]; bbox[i+1] = t[i];
				iSwitch = 1;
			}
		}
	}while(iSwitch);

	if(bbox.size() != 0){
		for(size_t i = 0; i != bbox.size(); i++)
		{
			int rgb_object_x = bbox[i].center.x;
			int rgb_object_y = bbox[i].center.y;
			int index_pc = rgb_object_y*input.width + rgb_object_x;
			float object_x = cloud_src.points[index_pc].x;
			float object_y = cloud_src.points[index_pc].y;
			float object_z = cloud_src.points[index_pc].z;

			if(std::isnan(object_x)) continue;
			geometry_msgs::Pose pose_buf;
			object_position.x = object_x; 
			object_position.y = object_y; //object_position.z = object_z; 
			pose_buf.position = object_position;
			pose_buf.position.z = i + 1; //track id
			ai_targets.poses.push_back(pose_buf); 

			p.x = 0; p.y = 0; p.z = 0; 
			line_object.points.push_back(p);
			p.x = object_x; 
			p.y = object_y; 
			p.z = object_z; 
			line_object.points.push_back(p);
			//DEBUG ROS_INFO("objectROI_center (%d,%d) - (%.2f %.2f %.2f)",rgb_object_x,rgb_object_y,object_x,object_y,object_z); 
		}
	}
	marker_pub.publish(line_object);
	ai_targets_pub.publish(ai_targets);
	//EWING
	/*	# publish to people_tracked topic
	new_person = Person() 
	new_person.pose.position.x = ps.point.x 
	new_person.pose.position.y = ps.point.y 
	yaw = math.atan2(person.vel_y, person.vel_x)
	quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
	new_person.pose.orientation.x = quaternion[0]
	new_person.pose.orientation.y = quaternion[1]
	new_person.pose.orientation.z = quaternion[2]
	new_person.pose.orientation.w = quaternion[3] 
	new_person.id = person.id_num 
	people_tracked_msg.people.append(new_person)
	self.people_tracked_pub.publish(people_tracked_msg) */
	
	//EWING
	// publish rviz markers       
	marker_id = 0;
	ros::Time now = ros::Time::now();
	static ros::Time last_seen;
	person_marker.header.frame_id = "/base_footprint";
	person_marker.header.stamp = now;
	person_marker.ns = "People_tracked";
	person_marker.color.r = 0.9;
	person_marker.color.g = 0.8;
	person_marker.color.b = 0.1;
	
	person_marker.color.a = (ros::Duration(3) - (now - last_seen)).toSec()/ros::Duration(3).toSec() + 0.1;
	
	for(std::vector<geometry_msgs::Pose>::iterator it = ai_targets.poses.begin(); it != ai_targets.poses.end(); ++it) 
	{
		// Cylinder for body 
		person_marker.pose.position.x = it->position.x;
		person_marker.pose.position.y = it->position.y;
		person_marker.id = marker_id;
		marker_id++;
		person_marker.type = visualization_msgs::Marker::CYLINDER;
		person_marker.scale.x = 0.2;
		person_marker.scale.y = 0.2;
		person_marker.scale.z = 1.2;
		person_marker.color.a = 0.9;
		person_marker.pose.position.z = 0.8;
		marker_pub.publish(person_marker) ;
		//std::cout << "pub" << std::endl;

		// Sphere for head shape                        
		person_marker.type = visualization_msgs::Marker::SPHERE;
		person_marker.scale.x = 0.2;
		person_marker.scale.y = 0.2;
		person_marker.scale.z = 0.2;
		person_marker.color.a = 0.9;
		person_marker.pose.position.z = 1.5;
		person_marker.id = marker_id;
		marker_id ++;                
		marker_pub.publish(person_marker);
		last_seen = now;
	}
	
	if( ai_targets.poses.empty() )
	{
		// Cylinder for body 
		person_marker.id = marker_id;
		marker_id++;
		person_marker.type = visualization_msgs::Marker::CYLINDER;
		person_marker.scale.x = 0.2;
		person_marker.scale.y = 0.2;
		person_marker.scale.z = 1.2;
		person_marker.pose.position.z = 0.8;
		marker_pub.publish(person_marker) ;
		//std::cout << "pub" << std::endl;

		// Sphere for head shape                        
		person_marker.type = visualization_msgs::Marker::SPHERE;
		person_marker.scale.x = 0.2;
		person_marker.scale.y = 0.2;
		person_marker.scale.z = 0.2;          
		person_marker.pose.position.z = 1.5;
		person_marker.id = marker_id;
		marker_id ++;                
		marker_pub.publish(person_marker);
	}

//  point_pub.publish(object_position);

}

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "target_extractor");
	ros::NodeHandle nh_param;
	ROS_INFO("ai_target_extractor");
	std::string rgb_topic;
	std::string pc_topic;
	std::string target_cloth_type;
	nh_param.param<std::string>("rgb_topic", rgb_topic, "/objects");
	nh_param.param<std::string>("topic", pc_topic, "/camera/depth_registered/points");
	nh_param.param<std::string>("target_cloth_type", target_cloth_type, "shirts");
	nh_param.param<double>("target_threshold", target_threshold, 0.85);
	nh_param.setParam("rgb_topic", rgb_topic);
	nh_param.setParam("topic", pc_topic);
	nh_param.setParam("target_cloth_type", target_cloth_type);
	nh_param.setParam("target_threshold", target_threshold);
        
	// Distinguish target type
	switch ( str2int(target_cloth_type.c_str()) ) 
	{
		case str2int("shirts"):
			target_cloth = SHIRTS;
			ROS_INFO("AI target set to \"shirts\"");
			break;
		case str2int("polo"):
			target_cloth = POLO;
			ROS_INFO("AI target set to \"polo\"");
			break;
		case str2int("tshirt"):
			target_cloth = TSHIRT;
			ROS_INFO("AI target set to \"tshirt\"");
			break;
		case str2int("suit"):
			target_cloth = SUIT;
			ROS_INFO("AI target set to \"suit\"");
		default:
			target_cloth = SHIRTS;
			ROS_INFO("Unknown type, AI target set to \"shirts\"");
	}
		
	
	tf_listener = new tf::TransformListener(); 
	ros::NodeHandle nh;
	ros::Subscriber rgb_sub = nh.subscribe(rgb_topic, 1 , callbackROI);
	ros::Subscriber pc_sub = nh.subscribe(pc_topic, 1 , callbackPointCloud);
	marker_pub = nh.advertise<visualization_msgs::Marker>("object_marker", 20);
	//  point_pub = nh.advertise<geometry_msgs::Point>("object_position", 5);
	ai_targets_pub = nh.advertise<geometry_msgs::PoseArray>("ai_targets", 5);
	ros::Rate loop_rate(30);
	while( ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	delete tf_listener; 
	return 0;
}
