#include "rclcpp/rclcpp.hpp"
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

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <object_msgs/msg/object_in_box.hpp>
#include <visualization_msgs/msg/marker.hpp>
using namespace cv;

//static geometry_msgs::Pose2D center;
static std::vector<object_msgs::msg::ObjectInBox > bbox;
static std::vector<object_msgs::msg::ObjectInBox > t;
static std::vector<object_msgs::msg::ObjectInBox>::const_iterator object_iter; 
static std::shared_ptr<tf2_ros::Buffer> _tfBuffer;
static std::shared_ptr<tf2_ros::TransformListener> tf_listener;
//static rclcpp::Publisher<geometry_msgs::msg::Twist> point_pub;
static rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ai_targets_pub;
static rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
static geometry_msgs::msg::PoseArray ai_targets;
static geometry_msgs::msg::PoseWithCovariance pose;
static visualization_msgs::msg::Marker line_object, person_marker;

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

void callbackROI(object_msgs::msg::ObjectsInBoxes::SharedPtr data)
{
  auto npc = rclcpp::Node::make_shared("roi");
  RCLCPP_INFO(npc->get_logger(), "callbackROI");
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


void callbackPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr input)
{
	auto npc = rclcpp::Node::make_shared("pcl");
	RCLCPP_INFO(npc->get_logger(), "callbackPointCloud");

	std::string base_footprint = npc->declare_parameter("base_footprint", "base_footprint");
	std::string base_link = npc->declare_parameter("base_link", "base_link");
	std::string camera_link = npc->declare_parameter("camera_link", "camera_link");

	npc->get_parameter("base_footprint", base_footprint);
	npc->get_parameter("base_link", base_link);
	npc->get_parameter("camera_link", camera_link);

	bool res = _tfBuffer->canTransform(base_footprint, input->header.frame_id, tf2::timeFromSec(5.0));

	if(res == false)
	{
		RCLCPP_INFO(npc->get_logger(), "No transform from \"%s\" to \"%s\"",base_footprint.c_str() ,input->header.frame_id.c_str());
		return;
	}
	geometry_msgs::msg::TransformStamped  tf_cam_to_base, tf_cam_link;
	try{
		//tf_listener->waitForTransform("/base_link", input->header.frame_id, input->header.stamp, ros::Duration(3.0));
		tf_cam_to_base = _tfBuffer->lookupTransform(base_link, input->header.frame_id, tf2::timeFromSec(3.0));
		tf_cam_link    = _tfBuffer->lookupTransform(camera_link, input->header.frame_id, tf2::timeFromSec(3.0));
	}catch (tf2::TransformException ex) {
		RCLCPP_INFO(npc->get_logger(), "TF lookup failed %s", ex.what());
	}

	//sensor_msgs::PointCloud2 ptcloud_from_cam;
	//pcl_ros::transformPointCloud("/camera_link", input, ptcloud_from_cam, *tf_listener);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
	//pcl::fromROSMsg(ptcloud_from_cam , cloud_src);
	pcl::fromROSMsg<pcl::PointXYZRGB>(*input , cloud_src);

	int marker_id = 0;
	line_object.points.clear();
	line_object.header.frame_id = "/camera_link";
	line_object.ns = "line_object";
	line_object.action = visualization_msgs::msg::Marker::ADD;
	line_object.id = marker_id;
	marker_id++;
	line_object.type = visualization_msgs::msg::Marker::LINE_LIST;
	line_object.scale.x = 0.01;
	line_object.color.r = 1.0;
	line_object.color.g = 0;
	line_object.color.b = 1.0;
	line_object.color.a = 1.0;
	
	geometry_msgs::msg::PoseArray ai_targets;
	ai_targets.poses.clear();
	ai_targets.header.frame_id = "camera_link";

	if(bbox.size() != 0){
		for(size_t i = 0; i != bbox.size(); i++)
		{
			// find center pixel coordinate, from top left of the frame.
			int pix_center_x = bbox[i].roi.x_offset + ((float)bbox[i].roi.width/2);
			int pix_center_y = bbox[i].roi.y_offset + ((float)bbox[i].roi.height/2);
			int pix_center_ind = pix_center_y*input->width + pix_center_x;
			
			float object_x = cloud_src.points[pix_center_ind].x;	// get pointcloud distance
			float object_y = cloud_src.points[pix_center_ind].y;
			float object_z = cloud_src.points[pix_center_ind].z;
			geometry_msgs::msg::TransformStamped cam_obj;
			cam_obj.header.frame_id = input->header.frame_id;
			cam_obj.header.stamp = rclcpp::Clock().now();
			cam_obj.child_frame_id = ai_targets.header.frame_id;
			cam_obj.transform.translation.x = object_x;
			cam_obj.transform.translation.y = object_y;
			cam_obj.transform.translation.z = object_z;

			tf2::Stamped<tf2::Transform> tf2_cam_link, tf2_cam_obj;
			tf2::fromMsg(tf_cam_link, tf2_cam_link);
			tf2::fromMsg(tf_cam_link, cam_obj);
			tf2::Vector3 obj_coord_in_cam = (tf2_cam_link * tf2_cam_obj).getOrigin();
			
			if(std::isnan(object_x)) continue;	// elimiate point cloud errer?
			
			geometry_msgs::msg::Pose pose_buf;
			pose_buf.position.x = obj_coord_in_cam.x(); //track id
			pose_buf.position.y = obj_coord_in_cam.y(); //track id
			pose_buf.position.z = i + 1; //track id
			ai_targets.poses.push_back(pose_buf); 

			// line object pointing from camera to target
			geometry_msgs::msg::Point p;
			p.x = 0; p.y = 0; p.z = 0; 
			line_object.points.push_back(p);
			p.x = obj_coord_in_cam.x();
			p.y = obj_coord_in_cam.y(); 
			p.z = obj_coord_in_cam.z();	
			line_object.points.push_back(p);
			RCLCPP_INFO(npc->get_logger(),"objectROI_center (%d,%d) - (%.2f %.2f %.2f)",pix_center_x,pix_center_y,object_x,object_y,object_z); 
		}
	}
	marker_pub->publish(line_object);
	ai_targets_pub->publish(ai_targets);
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
	auto now = rclcpp::Clock().now();
	static rclcpp::Time last_seen[20];
	static geometry_msgs::msg::Pose last_pose[20];

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
		float alpha_decay = (rclcpp::Duration(3) - (now - last_seen[j])).seconds()/rclcpp::Duration(3).seconds() + 0.1;
		alpha_decay = std::max(alpha_decay, (float)0.0); // so it won't be negative

		// Cylinder for body 
		person_marker.type = visualization_msgs::msg::Marker::CYLINDER;
		person_marker.id = marker_id;
		person_marker.pose.position.x = last_pose[j].position.x;
		person_marker.pose.position.y = last_pose[j].position.y;
		person_marker.pose.position.z = 0.7;	// =scale.z / 2
		person_marker.scale.x = 0.2;
		person_marker.scale.y = 0.2;
		person_marker.scale.z = 1.4;	// 1.4 m in height
		person_marker.color.a = alpha_decay;
		marker_pub->publish(person_marker);
		marker_id++;

		// Sphere for head 
		person_marker.type = visualization_msgs::msg::Marker::SPHERE;
		person_marker.id = marker_id;
		person_marker.pose.position.z = 1.5;	//position x, y is the same
		person_marker.scale.x = 0.2;	// 0.2 m in diameter
		person_marker.scale.y = 0.2;
		person_marker.scale.z = 0.2;
		person_marker.color.a = alpha_decay;
		marker_pub->publish(person_marker);
		marker_id ++;  
	}
}

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto nh = rclcpp::Node::make_shared("target_extractor");
	RCLCPP_INFO(nh->get_logger(), "ai_target_extractor");
	std::string object_box_topic = nh->declare_parameter("object_box_topic","/ros2_openvino_toolkit/detected_objects");
	std::string pointcloud_topic = nh->declare_parameter("pointcloud_topic","/camera/aligned_depth_to_color/color/points");
	std::string target_name = nh->declare_parameter("target_type", "person");
	double target_threshold = nh->declare_parameter("target_threshold", 0.85);
	nh->get_parameter("object_box_topic", object_box_topic);
	nh->get_parameter("pointcloud_topic", pointcloud_topic);
	nh->get_parameter("target_type", target_name);
	nh->get_parameter("target_threshold", target_threshold);

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
	_tfBuffer   = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
	tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);
	auto rgb_sub = nh->create_subscription<object_msgs::msg::ObjectsInBoxes>(object_box_topic, 1 ,callbackROI);
	auto pc_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 1, callbackPointCloud);
	marker_pub = nh->create_publisher<visualization_msgs::msg::Marker>("detected_person", 20);
	//  point_pub = nh.advertise<geometry_msgs::Point>("object_position", 5);
	ai_targets_pub = nh->create_publisher<geometry_msgs::msg::PoseArray>("ai_targets", 5);
	rclcpp::Rate loop_rate(30);
	while( rclcpp::ok() )
	{
		rclcpp::spin_some(nh);
		loop_rate.sleep();
	}

	return 0;
}
