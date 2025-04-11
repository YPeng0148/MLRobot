#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"

#include <iostream>
#include <robotcontrol.h> // includes ALL Robot Control subsystems

#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <signal.h>
#include <math.h>

double compass = 0.0;
double compass_delta = 0.0;
double w = 0.1;
int got_msg = 0;
double accuracy = 5.0;

double wrap(double angle){
    if (angle > 180.0)
        return angle - 360.0;
    if (angle < -180.0)
        return angle + 360.0;
    return angle;
}

/*
def avg_com_helper(n):
    x = 0
    y = 0
    for i in range(n):
        x = x + math.cos(compass_readings[i] * math.pi / 180.0)
        y = y + math.sin(compass_readings[i] * math.pi / 180.0)
    return math.atan2(y/n,x/n)

def average_compass():
    if compass_received_10:
        return avg_com_helper(10)
    else:
        if compass_count == 0:
            return 0
        else:
            return avg_com_helper(compass_count)
*/

void handle_compass(const std_msgs::Float64::ConstPtr& msg) {
	got_msg = 1;
	double old_compass = compass;
	double x = (1.0 - w) * cos(compass * M_PI / 180.0) + w * cos(msg->data * M_PI / 180.0);
	double y = (1.0 - w) * sin(compass * M_PI / 180.0) + w * sin(msg->data * M_PI / 180.0);
	compass = atan2(y,x) * 180.0 / M_PI;
	compass_delta = wrap(compass - old_compass);
	std::cout << "compass delta: " << compass_delta << std::endl;
}

double get_initial_orientation() {
	while (1) {
		if (got_msg) {
			if (compass_delta < accuracy && compass_delta > -accuracy)
				return compass;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "encoders");
	ros::NodeHandle n;

	ros::Publisher estimated_pose_publisher = n.advertise<geometry_msgs::Point>("estimated_pose", 1000);
	
	//ros::Subscriber compass_subscriber = n.subscribe("compass", 1, handle_compass);

	if (rc_encoder_eqep_init()){
		std::cout << "Failed to initialize encoders" << std::endl;
		return -1;
	}

	int l_prev = 0;
	int r_prev = 0;

	double x = 0.0;
	double y = 0.0;
	double theta = 0.0; //get_initial_orientation();

	double wheel_radius = 0.0419;
	double robot_width = 0.6096;

	double distance_correction = 1.0;
	double angle_correction = 1.0;	

	int freq = 20;
	ros::Rate timer(freq);
	while (ros::ok()) {
		int l= rc_encoder_eqep_read(2);
		int r = -1*rc_encoder_eqep_read(3);

		int dl = l - l_prev;
		int dr = r - r_prev;

		double vl = dl * 2.0 * M_PI / 2200.0;
		double vr = dr * 2.0 * M_PI / 2200.0;

		//std::cout << "dl: " << dl << " dr: " << dr << std::endl;  
		
		double dx = distance_correction * wheel_radius * (vl + vr) * cos(theta) / 2.0; 
		double dy = distance_correction *  wheel_radius * (vl + vr)  * sin(theta) / 2.0;
		double dtheta = wheel_radius * (angle_correction * vr - vl) / robot_width;

		x += dx;
		y += dy;
		theta += dtheta; 	
		if (theta > M_PI)
			theta -= 2 * M_PI;
		else if (theta < -1*M_PI)	
			theta += 2 * M_PI;

		//std::cout << "x: " << x << " y: " << y << " theta: " << theta * 180.0 / M_PI << std::endl;  
 		
		geometry_msgs::Point msg;
		msg.x = x;
		msg.y = y;
		msg.z = theta * 180.0 / M_PI;
		estimated_pose_publisher.publish(msg);
		
		l_prev = l;
		r_prev = r;

		ros::spinOnce();
		timer.sleep();
	} rc_encoder_eqep_cleanup();
	return 0;
}
