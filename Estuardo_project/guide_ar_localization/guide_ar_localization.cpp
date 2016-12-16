#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <stdio.h>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <sound_play/sound_play.h>

std::atomic<int> matchCounter;
std::atomic<bool> active; 

/* Maps an ar tag id with a location name
	ToDo: move this information to a configuration file
*/
std::string getMarkerName(int id) {
	switch(id) {
		case 1001: return "point alfa";
		case 1002: return "point beta";
		case 1003: return "point gamma";
		case 1004: return "point delta";
		case 1005: return "point epsilon";
		case 1006: return "point zeta";
		case 1007: return "point eta";
		case 1008: return "point theta";
		case 1009: return "point iota";
		case 1010: return "point kappa";
	}
	return "an unknown location";
}

/*Identifies the information sent by the ar_track_alvar node*/
void markerParser(const visualization_msgs::MarkerConstPtr& msg) {
	int id = msg->id;
	if (id < 1011 && id > 1000) {
		ROS_INFO("[%i]", id);
		if (matchCounter == 5) {
			ROS_INFO("AR marker detected: [%i]", id);
				std::string output = "We are approaching " + getMarkerName(id);
				sound_play::SoundClient* speech = new sound_play::SoundClient();
				speech->say(output);
		}
		matchCounter++;
	}
}

/*Timer that controls feedback frecuency*/
void resetFaceRecogTimer()
{
    active = true; 
    while (active) {
       matchCounter = 0;
       std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "guide_ar_localization");
	ros::NodeHandle n;
	ROS_INFO("AR localization initialized.");
	std::thread t(resetFaceRecogTimer);
	ros::Subscriber subMarker = n.subscribe("/visualization_marker", 150, markerParser);
	ros::spin();
	active = false;
	t.join(); 
	return 0;
}
