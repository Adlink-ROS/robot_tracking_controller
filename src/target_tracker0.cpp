#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <iostream>
#include <math.h>
#include <chrono>


using namespace std;

static double clamp(double v, double v_min, double v_max)
{
    return std::min(std::max(v_min,v),v_max);
}

# define _USE_MATH_DEFINES

double dist_target, ang_target, dist_Kp, ang_Kp, dist_min;
double vel_max, ang_max;
double decay_factor_lin, decay_factor_ang, inject_factor_lin, inject_factor_ang;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

geometry_msgs::msg::TransformStamped transformc2p;
geometry_msgs::msg::Point Position;
void trackerCB(tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    static std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    
    transformc2p = msg.lookupTransform("camera_color_optical_frame", "person0");
    tf2::Vector3 ttt = transformc2p.getOrigin();
    Position.x = ttt.x();
    Position.y = ttt.y();
    Position.z = ttt.z();

    double d = sqrt( pow(Position.x,2) + pow(Position.y,2) );
    double rad = atan2(Position.y, Position.x);
    
    double dist_err = d - dist_target;
    double ang_err = rad - ang_target;
    std::chrono::duration<double, std::milli> elapsed_ms = now - last_time; // convert duration to ms
    
    double decay_ratio_lin = exp( -decay_factor_lin*elapsed_ms.count()/1000 ); //cmd = cmd * e^(-decayF*delta_t)
    double inject_ratio_lin = exp( -inject_factor_lin*elapsed_ms.count()/1000 );
	
    double decay_ratio_ang = exp( -decay_factor_ang*elapsed_ms.count()/1000 );
    double inject_ratio_ang = exp( -inject_factor_ang*elapsed_ms.count()/1000 ); 
    
    //DEBUG std::cout << "t " << elapsed_ms.count() << " ms" << std::endl;
    
    static geometry_msgs::msg::Twist track, vel_cmd; // static so it remembers last value
    if( d < dist_min)
    {
		vel_cmd.linear.x *= decay_ratio_lin;
		vel_cmd.angular.z *= decay_ratio_ang;
    }else {
		vel_cmd.linear.x = clamp( dist_err*dist_Kp, -vel_max, vel_max)*(1-inject_ratio_lin) + vel_cmd.linear.x*inject_ratio_lin;
		vel_cmd.angular.z = clamp( ang_err*ang_Kp, -ang_max, ang_max)*(1-inject_ratio_ang) + vel_cmd.angular.z*inject_ratio_ang;
    }
    last_time = now;
    vel_pub->publish(vel_cmd);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("target_tracker");
    double dist_target = n->declare_parameter("dist_target", 1.0);
    double ang_target = n->declare_parameter("ang_target", 0.0);
    double dist_Kp = n->declare_parameter("dist_Kp", 1.0);
    double ang_Kp = n->declare_parameter("ang_Kp", 2.0);
    double vel_max = n->declare_parameter("vel_max", 0.3);
    double ang_max = n->declare_parameter("omega_max", 1.5);
    double dist_min = n->declare_parameter("dist_min", 0.5);
    double decay_factor_lin = n->declare_parameter("decay_factor_lin", 3.0);
    double decay_factor_ang = n->declare_parameter("decay_factor_ang", 15.0);
    double inject_factor_lin = n->declare_parameter("inject_factor_lin", 6.0);
    double inject_factor_ang = n->declare_parameter("inject_factor_ang", 20.0);
	
	n->get_parameter("dist_target", dist_target);
    n->get_parameter("ang_target", ang_target);
    n->get_parameter("dist_Kp", dist_Kp);
    n->get_parameter("ang_Kp", ang_Kp);
    n->get_parameter("vel_max", vel_max);
    n->get_parameter("omega_max", ang_max);
    n->get_parameter("dist_min", dist_min);
    n->get_parameter("decay_factor_lin", decay_factor_lin);
	n->get_parameter("inject_factor_lin", inject_factor_lin);
	n->get_parameter("decay_factor_ang", decay_factor_ang);
	n->get_parameter("inject_factor_ang", inject_factor_ang);
    
    vel_pub = n->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto sub = n->create_subscription<tf2_msgs::msg::TFMessage>("tf",10, trackerCB);
    rclcpp::Rate loop_rate(30);
	while( rclcpp::ok() )
	{
		rclcpp::spin_some(n);
		loop_rate.sleep();
	}
    return 0;
}
