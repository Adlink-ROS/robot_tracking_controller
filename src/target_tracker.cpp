#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
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

ros::Publisher vel_pub;

void trackerCB(const geometry_msgs::Point::ConstPtr &Position)
{
    static std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    
    double d = sqrt( pow(Position->x,2) + pow(Position->y,2) );
    double rad = atan2(Position->y, Position->x);
    
    double dist_err = d - dist_target;
    double ang_err = rad - ang_target;
    std::chrono::duration<double, std::milli> elapsed_ms = now - last_time; // convert duration to ms
    
    double decay_ratio_lin = exp( -decay_factor_lin*elapsed_ms.count()/1000 ); //cmd = cmd * e^(-decayF*delta_t)
    double inject_ratio_lin = exp( -inject_factor_lin*elapsed_ms.count()/1000 );
	
    double decay_ratio_ang = exp( -decay_factor_ang*elapsed_ms.count()/1000 );
    double inject_ratio_ang = exp( -inject_factor_ang*elapsed_ms.count()/1000 ); 
    
    //DEBUG std::cout << "t " << elapsed_ms.count() << " ms" << std::endl;
    
    static geometry_msgs::Twist track, vel_cmd; // static so it remembers last value
    if( d < dist_min)
    {
		vel_cmd.linear.x *= decay_ratio_lin;
		vel_cmd.angular.z *= decay_ratio_ang;
    }else {
		vel_cmd.linear.x = clamp( dist_err*dist_Kp, -vel_max, vel_max)*(1-inject_ratio_lin) + vel_cmd.linear.x*inject_ratio_lin;
		vel_cmd.angular.z = clamp( ang_err*ang_Kp, -ang_max, ang_max)*(1-inject_ratio_ang) + vel_cmd.angular.z*inject_ratio_ang;
    }
    last_time = now;
    vel_pub.publish(vel_cmd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_tracker");
    ros::NodeHandle n;
    n.param<double>("dist_target", dist_target, 1.0);
    n.param<double>("ang_target", ang_target, 0.0);
    n.param<double>("dist_Kp", dist_Kp, 1);
    n.param<double>("ang_Kp", ang_Kp, 2.0);
    n.param<double>("vel_max", vel_max, 0.3);
    n.param<double>("omega_max", ang_max, 1.5);
    n.param<double>("dist_min", dist_min, 0.5);
    n.param<double>("decay_factor_lin", decay_factor_lin, 3);
	n.param<double>("decay_factor_ang", decay_factor_ang, 15);
	n.param<double>("inject_factor_lin", inject_factor_lin, 6);
	n.param<double>("inject_factor_ang", inject_factor_ang, 20);
	
	n.setParam("dist_target", dist_target);
    n.setParam("ang_target", ang_target);
    n.setParam("dist_Kp", dist_Kp);
    n.setParam("ang_Kp", ang_Kp);
    n.setParam("vel_max", vel_max);
    n.setParam("omega_max", ang_max);
    n.setParam("dist_min", dist_min);
    n.setParam("decay_factor_lin", decay_factor_lin);
	n.setParam("inject_factor_lin", inject_factor_lin);
	n.setParam("decay_factor_ang", decay_factor_ang);
	n.setParam("inject_factor_ang", inject_factor_ang);
    
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = n.subscribe("tracking_target",10, trackerCB);
    ros::spin();
    return 0;
}
