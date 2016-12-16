#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sound_play/sound_play.h>

#define LEFT 0
#define RIGHT 1

std::atomic<bool> active;
std::atomic<int> outputRequired;
sound_play::SoundClient* speech;

/*Identifies if the robot will turn based on the direction of the vectors created from the parameters*/
int getTurnId(geometry_msgs::Point p0, geometry_msgs::Point p1, geometry_msgs::Point p3, geometry_msgs::Point p4) {
	int turnId = -1;
	if (p4.x != 0 && p4.y != 0) {
		double dirX = p1.x - p0.x;
		double dirY = p1.y - p0.y;
		double trX = p4.x - p3.x;
		double trY = p4.y - p3.y;
		double crossProduct = dirX * trY - dirY * trX;
		double angle1 = std::atan2(dirY, dirX);
		double angle2 = std::atan2(trY, trX);
		double angle = angle2 - angle1;		
		if (angle > M_PI / 4) {
			ROS_INFO("Angle %lf   %lf", angle, crossProduct);
			if (crossProduct > 0)
				turnId = LEFT;
			else if (crossProduct < 0)
				turnId = RIGHT;
		}
	}
	return turnId;	
}

/*Idenitifies if the robot is turning left or right*/
void planParser(const nav_msgs::PathConstPtr& msg) {
	try {
		if (outputRequired == 1) {
			int firstPos = 0;
			int lastPos = 90;
			int turnId = getTurnId(msg->poses[firstPos].pose.position, msg->poses[firstPos + 10].pose.position, msg->poses[lastPos - 10].pose.position, msg->poses[lastPos].pose.position);
			if (turnId == LEFT || turnId == RIGHT) {
				std::string side;
				if (turnId == LEFT) {
					ROS_INFO("Side identified: LEFT");
					side = "left";
				}
				else {
					ROS_INFO("Side identified: RIGHT");
					side = "right";
				}
				std::string output = "We will turn " + side + " soon";
				if (outputRequired == 1) {
					outputRequired = 0;
					speech->say(output);
					std::this_thread::sleep_for(std::chrono::seconds(5));
				}
			}
		}
	} catch (...) { 
		ROS_INFO("Exception caught.");
	}
}

/*Timer that controls the feedback frecuency*/
void resetFeedbackTimer()
{
    active = true; 
    while (active) {
		outputRequired = 1;
		std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}

int main(int argc, char **argv) {
	outputRequired = 0;
	ros::init(argc, argv, "guide_navigation_feedback");
	ros::NodeHandle n;
	speech = new sound_play::SoundClient();
	ROS_INFO("Navigation feedback initialized.");
	std::thread t(resetFeedbackTimer);
	ros::Subscriber subMarker = n.subscribe("/move_base/NavfnROS/plan", 10, planParser);
	ros::spin();
	active = false;
	t.join(); 
	return 0;
}
