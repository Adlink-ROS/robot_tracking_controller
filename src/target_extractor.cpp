#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <vision_msgs/Detection2DArray.h>
//#include <vision_msgs/BoundingBox2D.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <algorithm>


//from leg_tracker.msg import Person, PersonArray, Leg, LegArray 
//#include <stdlib.h>

#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <object_msgs/ObjectInBox.h>
#include <visualization_msgs/Marker.h>
using namespace cv;

//static geometry_msgs::Pose2D center;
static std::vector<object_msgs::ObjectInBox > bbox;
static std::vector<object_msgs::ObjectInBox > t;
static std::vector<object_msgs::ObjectInBox>::const_iterator object_iter; 
static tf::TransformListener *tf_listener; 
static ros::Publisher point_pub;
static ros::Publisher ai_targets_pub;
static ros::Publisher marker_pub;
static geometry_msgs::PoseArray ai_targets;
static geometry_msgs::PoseWithCovariance pose;
static visualization_msgs::Marker line_object, person_marker;

/* see labelmap.pbtxt for definition*/
/*enum cloth_type {
    SHIRTS = 1,
    POLO   = 2,
    TSHIRT = 3,
    SUIT   = 4
};
cloth_type*/
std::string target_name = "person";
double target_threshold = 0.85;

void callbackROI(const object_msgs::ObjectsInBoxes::ConstPtr& data)
{
  bbox.clear();
  int count = 0;
  //vision_msgs::Detection2D detection;
  
  // filter out desired target by name and probability
  for (object_iter = data->objects_vector.begin(); object_iter != data->objects_vector.end(); ++object_iter)
  {
    if( (object_iter->object.object_name == target_name) && (object_iter->object.probability >= target_threshold) )
	{  
		bbox.push_back((*object_iter));
		count = ++count;
    }
  }
  //ROS_INFO("Shirts number:%d",count);
}


void callbackPointCloud(const sensor_msgs::PointCloud2 &input)
{
	bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
	if(res == false)
	{
		ROS_ERROR("No transform from \"/base_footprint\" to \"%s\"", input.header.frame_id.c_str());
		return;
	}
	tf::StampedTransform tf_cam_to_base, tf_cam_link;
	try{
		//tf_listener->waitForTransform("/base_link", input->header.frame_id, input->header.stamp, ros::Duration(3.0));
		tf_listener->lookupTransform("/base_link", input.header.frame_id, input.header.stamp, tf_cam_to_base);
		tf_listener->lookupTransform("/camera_link", input.header.frame_id, input.header.stamp, tf_cam_link);
	}catch (tf::TransformException ex) {
		ROS_WARN("TF lookup failed %s", ex.what());
	}
	
	//sensor_msgs::PointCloud2 ptcloud_from_cam;
	//pcl_ros::transformPointCloud("/camera_link", input, ptcloud_from_cam, *tf_listener);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
	//pcl::fromROSMsg(ptcloud_from_cam , cloud_src);
	pcl::fromROSMsg(input , cloud_src); 

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
	
	geometry_msgs::PoseArray ai_targets;
	ai_targets.poses.clear();
	ai_targets.header.frame_id = "camera_link";

	if(bbox.size() != 0){
		for(size_t i = 0; i != bbox.size(); i++)
		{
			// find center pixel coordinate, from top left of the frame.
			int pix_center_x = bbox[i].roi.x_offset + ((float)bbox[i].roi.width/2);
			int pix_center_y = bbox[i].roi.y_offset + ((float)bbox[i].roi.height/2);
			int pix_center_ind = pix_center_y*input.width + pix_center_x;
			
			float object_x = cloud_src.points[pix_center_ind].x;	// get pointcloud distance
			float object_y = cloud_src.points[pix_center_ind].y;
			float object_z = cloud_src.points[pix_center_ind].z;
			tf::Vector3 obj_coord(object_x, object_y, object_z);	// point cloud is defined in Z axis forward
			tf::Vector3 obj_coord_in_cam = tf_cam_link * obj_coord;
			tf::Vector3 obj_coord_in_base = tf_cam_to_base * obj_coord;
			
			if(std::isnan(object_x)) continue;	// elimiate point cloud errer?
			
			geometry_msgs::Pose pose_buf;
			pose_buf.position.x = obj_coord_in_cam.x(); //track id
			pose_buf.position.y = obj_coord_in_cam.y(); //track id
			pose_buf.position.z = i + 1; //track id
			ai_targets.poses.push_back(pose_buf); 

			// line object pointing from camera to target
			geometry_msgs::Point p;
			p.x = 0; p.y = 0; p.z = 0; 
			line_object.points.push_back(p);
			p.x = obj_coord_in_cam.x();
			p.y = obj_coord_in_cam.y(); 
			p.z = obj_coord_in_cam.z();	
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
	
	//=== Publish rviz markers ===
	marker_id = 0;
	ros::Time now = ros::Time::now();
	static ros::Time last_seen[20];
	static geometry_msgs::Pose last_pose[20];

	int marker_count;
	static int total_marker_count;
	person_marker.header.frame_id = "/base_footprint";
	person_marker.header.stamp = now;
	person_marker.ns = "People_tracked";
	person_marker.color.r = 0.9;
	person_marker.color.g = 0.8;
	person_marker.color.b = 0.1;
	
	// Publish multiple targets
	// keep publishing all targets so old unpublihsed ones won't hung up in rviz
	total_marker_count = std::max(total_marker_count, (int)ai_targets.poses.size());
	
	for(int j = 0; j<total_marker_count; j++) 
	{
		
		if( j < ai_targets.poses.size() )
		{
			last_pose[j] = ai_targets.poses[j];
			last_seen[j] = now;		// register last_seen if ar_targets is not empty
		}
		float alpha_decay = (ros::Duration(3) - (now - last_seen[j])).toSec()/ros::Duration(3).toSec() + 0.1;
		alpha_decay = std::max(alpha_decay, (float)0.0); // so it won't be negative
		
		// Cylinder for body 
		person_marker.type = visualization_msgs::Marker::CYLINDER;
		person_marker.id = marker_id;
		person_marker.pose.position.x = last_pose[j].position.x;
		person_marker.pose.position.y = last_pose[j].position.y;
		person_marker.pose.position.z = 0.7;	// =scale.z / 2
		person_marker.scale.x = 0.2;
		person_marker.scale.y = 0.2;
		person_marker.scale.z = 1.4;	// 1.4 m in height
		person_marker.color.a = alpha_decay;
		marker_pub.publish(person_marker) ;
		marker_id++;

		// Sphere for head 
		person_marker.type = visualization_msgs::Marker::SPHERE;
		person_marker.id = marker_id;
		person_marker.pose.position.z = 1.5;	//position x, y is the same
		person_marker.scale.x = 0.2;	// 0.2 m in diameter
		person_marker.scale.y = 0.2;
		person_marker.scale.z = 0.2;
		person_marker.color.a = alpha_decay;
		marker_pub.publish(person_marker);
		marker_id ++;  
	}
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
	std::string object_box_topic;
	std::string pointcloud_topic;
	nh_param.param<std::string>("object_box_topic", object_box_topic, "/movidius_ncs_nodelet/detected_objects_multiple");
	nh_param.param<std::string>("pointcloud_topic", pointcloud_topic, "/camera/depth_registered/points");
	nh_param.param<std::string>("target_type", target_name, "person");
	nh_param.param<double>("target_threshold", target_threshold, 0.85);
	nh_param.setParam("object_box_topic", object_box_topic);
	nh_param.setParam("pointcloud_topic", pointcloud_topic);
	nh_param.setParam("target_type", target_name);
	nh_param.setParam("target_threshold", target_threshold);
        
	// Distinguish target type
	/*switch ( str2int(target_type.c_str()) ) 
	{
		case str2int("shirts"):
			target_name = SHIRTS;
			ROS_INFO("AI target set to \"shirts\"");
			break;
		case str2int("polo"):
			target_name = POLO;
			ROS_INFO("AI target set to \"polo\"");
			break;
		case str2int("tshirt"):
			target_name = TSHIRT;
			ROS_INFO("AI target set to \"tshirt\"");
			break;
		case str2int("suit"):
			target_name = SUIT;
			ROS_INFO("AI target set to \"suit\"");
		default:
			target_name = SHIRTS;
			ROS_INFO("Unknown type, AI target set to \"shirts\"");
	}*/
		
	
	tf_listener = new tf::TransformListener(); 
	ros::NodeHandle nh;
	ros::Subscriber rgb_sub = nh.subscribe(object_box_topic, 1 , callbackROI);
	ros::Subscriber pc_sub = nh.subscribe(pointcloud_topic, 1 , callbackPointCloud);
	marker_pub = nh.advertise<visualization_msgs::Marker>("detected_person", 20);
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
