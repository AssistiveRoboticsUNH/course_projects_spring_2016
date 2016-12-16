#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <mutex>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sound_play/sound_play.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

/*Robot's possible states*/
#define IDLE 0
#define WAITING_ACTION 1
#define WAITING_GOAL 2
#define NAVIGATING 3

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*Global variables*/
int state;
int goalCode;
std::atomic<bool> goalSet;
std::atomic<bool> active;
std::atomic<bool> cancelled;
std::map<int, std::string> mapCodeToLocation;
std::map<std::string, int> mapLocationToCode;
sound_play::SoundClient* speech;
std::mutex navMutex;
MoveBaseClient* moveClient;

/* Coordinates for each possible goal location
	ToDo: move this coordinates to a configuration file
*/
double xLocations[] = {0, -0.7152312527, -3.3388394749, -3.35367219399, 4.48924891429, -3.58485864347, -1.00352077702, -2.68396542793, 8, 9, 10};
double yLocations[] = {0, 0.178173611668, 9.5832586087, 16.8010122999, -8.99245805451, 4.07063925539, 4.92757308826, -0.355207585345, 8, 9, 10};
double zLocations[] = {0, 0.795136956999, -0.998417568125, 0.983718162599, -0.982393785518, -0.220028067881, -0.767707466864, 0.696961398433, 0, 0, 0};
double wLocations[] = {1, 0.606429896703, 0.0562348616091, 0.17971804743, 0.186821974555, 0.975493541416, 0.640800472316, 0.717108645251, 1, 1, 1};

void commandParser(std::vector<std::string> commands);

/*Prinst feedback on terminal*/
void printFeedback(std::string command) {
	std::string outstring = "Processing command: " + command;
	ROS_INFO("%s",outstring.c_str());
}

/*Set of possible locations to navigate to
	Text is used by the robot to send feedback
	*/
void initLocationSet() {
	mapCodeToLocation[1] = "location one";
	mapCodeToLocation[2] = "location two";
	mapCodeToLocation[3] = "location three";
	mapCodeToLocation[4] = "location four";
	mapCodeToLocation[5] = "location five";
	mapCodeToLocation[6] = "location six";
	mapCodeToLocation[7] = "location seven";
	mapCodeToLocation[8] = "location eight";
	mapCodeToLocation[9] = "location nine";
	mapCodeToLocation[10] = "location ten";
	for (int i = 1; i <= 10; i++) {
		mapLocationToCode[mapCodeToLocation[i]] = i;
	}
}

/*Send a navigation goal to the robot*/
void navigate(int code) {
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = xLocations[code];
	goal.target_pose.pose.position.y = yLocations[code];
	goal.target_pose.pose.orientation.z = zLocations[code];
	goal.target_pose.pose.orientation.w = wLocations[code];
	ROS_INFO("Sending goal");
	moveClient->sendGoal(goal);
	goalSet = true;	
	navMutex.unlock();
}

/*Maps a location code to a location name*/
std::string getLocationName(int code) {
	return mapCodeToLocation[code];
}

/*Function used by navigation threads to monitor and display the state of the navigation*/
void navigationThread() {
	while (active) {
		navMutex.lock();
		if (goalSet) {
		    moveClient->waitForResult();

			if(moveClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				std::string msg = "We have reached " + getLocationName(goalCode);
				speech->say(msg);
				goalSet = false;
				cancelled = false;
				state = IDLE;
			}
			else {
				if (!cancelled) {
					std::string msg = "Navigation to " + getLocationName(goalCode) + " failed. Please contact support.";
					speech->say(msg);
					goalSet = false;
					state = IDLE;
				}
			}			
		}
	}
}

/*Gets a location code from a location name*/
int getGoalCode(std::string goal) {	
	if (mapLocationToCode.find(goal) == mapLocationToCode.end())
		return -1;
	return mapLocationToCode[goal];
}

/*Gets a token from the command sent by the user*/
void split(const std::string &s, char delim, std::vector<std::string> &tokens) {
	std::stringstream ss;
	ss.str(s);
	std::string token;
	std::vector<std::string>::iterator it = tokens.begin();
	while (std::getline(ss, token, delim)) {
		tokens.insert(it, token);
		it = tokens.begin();
	}
}

/*Splits a string given a delimitator*/
std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> tokens;
	split(s, delim, tokens);
	return tokens;
}

/*Parse a command when the robot is idle*/
void parseIdleCommand(std::vector<std::string> commands) {
	std::string command = commands.back();
	commands.pop_back();
	printFeedback(command);
	if (command == "grovi") {		
		state = WAITING_ACTION;
		if (!commands.empty()) {
			commandParser(commands);
		}
		else {
			speech->say("Tell me");
		}
	}
	else if (command == "go") {
		if (goalSet) {
			state = NAVIGATING;
			speech->say("Navigating to " + getLocationName(goalCode));
			navigate(goalCode);
		}
		else {
			speech->say("No goal has been defined");
		}
	}
	else if (command == "feedback") {
		if (!goalSet)
			speech->say("No goal has been defined");
		else {
			std::string msg = "Navigation to " + getLocationName(goalCode) + " was stopped";
			speech->say(msg);
		}
	}
}

/*Parse a command when the robot is expecting an action*/
void parseActionCommand(std::vector<std::string> commands) {
	std::string command;
	do {
		if (commands.empty()) {
			command = "";
		}
		else {
			command = commands.back();
			commands.pop_back();
		}
	} while (command == "grovi");
	printFeedback(command);
	if (command == "stop") {
		state = IDLE;
		speech->say("OK. Stopping");
	}
	else if (command == "navigate") {
		state = WAITING_GOAL;
		if (!commands.empty()) {
			commandParser(commands);
		}
		else {
			speech->say("Where do you want to go?");
		}
	}
	else if (command == "feedback"){
		speech->say("I am waiting for a command");
	}
}

/*Parse a command when the robot is expecting a navigation goal*/
void parseGoalCommand(std::vector<std::string> commands) {
	std::string command;
	std::string goal;
	if (commands.size() == 1) {
		command = commands.back();
		commands.pop_back();
		goal = "";
	}
	else {
		command = commands.back();
		commands.pop_back();
		goal = command + " " + commands.back();
		commands.pop_back();
	}
	printFeedback(command);
	if (command == "stop") {
		state = IDLE;
		speech->say("OK. Stopping");
	}
	else if (command == "feedback"){
		speech->say("I am waiting for a goal");
	}
	else if (goal == "") {
		speech->say(command + " is not a valid goal. Where do you want to go?");
	}
	else {
		goalCode = getGoalCode(goal);
		if (goalCode == -1) {
			speech->say(goal + " is not a valid goal. Where do you want to go?");
		}
		else {
			state = NAVIGATING;
			speech->say("Navigating to " + goal);
			navigate(goalCode);
		}
	}
}

/*Parse a command when the robot is navigating to a goal*/
void parseNavigatingCommand(std::vector<std::string> commands) {
	std::string command;
	command = commands.back();
	commands.pop_back();
	printFeedback(command);
	if (command == "stop") {
		state = IDLE;
		moveClient->cancelGoal();
		cancelled = true;
		std::string msg = "Navigation to " + getLocationName(goalCode) + " has been stopped.";
		speech->say(msg);
	}
	else if (command == "feedback"){
		speech->say("We are navigating to " + getLocationName(goalCode));
	}
}

/*Executes a parsing routine depending on the robot's current state*/
void commandParser(std::vector<std::string> commands) {		
	switch (state) {
		case IDLE:
		parseIdleCommand(commands);
		break;
		case WAITING_ACTION:
		parseActionCommand(commands);
		break;
		case WAITING_GOAL:
		parseGoalCommand(commands);
		break;
		case NAVIGATING:
		parseNavigatingCommand(commands);
		break;
	}
}

/*Subscriber callback procedure*/
void commandParser(const std_msgs::StringConstPtr& msg) {
	std::vector<std::string> commands = split(msg->data, ' ');
	commandParser(commands);	
}

int main(int argc, char **argv) {
	state = IDLE;
	goalSet = false;
	initLocationSet();	
	ros::init(argc, argv, "guide_voice_commands");
	ros::NodeHandle n;
	active = true;
	cancelled = false;
	navMutex.lock();
	std::thread t(navigationThread);
	speech = new sound_play::SoundClient();
	moveClient = new MoveBaseClient("move_base", true);
	ROS_INFO("Node initialized.");
	ros::Subscriber subFeedback = n.subscribe("/recognizer/output", 20, commandParser);
	ros::spin();
	active = false;
	goalSet = false;
	navMutex.unlock();
	t.join(); 
	return 0;
}
