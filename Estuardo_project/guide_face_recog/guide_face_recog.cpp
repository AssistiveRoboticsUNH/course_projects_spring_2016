#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include "ros/ros.h"
#include <face_recognition/FaceRecognitionAction.h>
#include <sound_play/sound_play.h>

std::atomic<int> matchCounter;
std::atomic<bool> active; 

int currentGoal = 4;

void goalParser(const face_recognition::FaceRecognitionActionGoalConstPtr& msg) {
	currentGoal = msg->goal.order_id;
	ROS_INFO("Current goal: [%i]", msg->goal.order_id);
}

void feedbackParser(const face_recognition::FaceRecognitionActionFeedbackConstPtr& msg) {
	if (currentGoal < 2) {
		if (matchCounter == 3) {
			ROS_INFO("Face recognized: [%s, %f]", msg->feedback.names[0].c_str(), msg->feedback.confidence[0]);
			std::string output = msg->feedback.names[0] + " is approaching";
			sound_play::SoundClient* speech = new sound_play::SoundClient();
			speech->say(output);
		}
		matchCounter++;
	}
}

void resetFaceRecogTimer()
{
    active = true; 
    while (active) {
       matchCounter = 0;
       std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "guide_face_recog");
	ros::NodeHandle n;
	std::thread t(resetFaceRecogTimer);
	ros::Subscriber subGoal = n.subscribe("face_recognition/goal", 20, goalParser);
	ros::Subscriber subFeedback = n.subscribe("face_recognition/feedback", 100, feedbackParser);
	ROS_INFO("Node initialized.");
	ros::spin();
	active = false;
	t.join(); 
	return 0;
}
