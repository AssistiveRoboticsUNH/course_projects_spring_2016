#GROVI: A Guide Robot for the Visually Impaired

##DESCRIPTION

This repository contains the nodes created to implement a guide robot for the visually impaired.

##DEPENDENCIES

The following packages are needed to run the GROVI's nodes:
	- actionlib
	- ar_track_alvar
	- face_recognition
	- move_base_msgs
	- nav_msgs
	- pocketsphinx
	- roscpp
	- rospy
	- sound_play
	- std_msgs
	- usb_cam
	- visualization_msgs
	
##STARTUP

The following nodes need to be executed in order to have GROVI up and running using a turtlebot:
	- turtlebot_bringup: using minimal launch file
	- turtlebot_navigation: using amcl_demo launch file and indicating the map to be used
	- usb_cam: using usb_cam_face_recog launch file
	- sound_play: using sound_play_node launch file
	- facerecog: indicating Fserver as the executable, optionally set the show_screen_flag to false to decrease the load on the computer
	- facerecog: indicating Fclient as the executable, check face_recognition documentation to know how to send commands to this nodes
	- ar_track_alvar: using grovi_ar_localization launch file
	- pocketsphinx: using 	grovi_voice_commands launch file
	- guide_ar_localization
	- guide_face_recog
	- guide_navigation_feedback
	- guide_voice_commands
	
##IMPORTANT NOTES

	- Parameters marker_size, max_new_marker_error and max_track_error included in file grovi_ar_localization.launch should be configured depending on the size of the physical AR markers that are being used.
	- Parameters lm and dict included in file grovi_voice_commands.launch should be configured to match the language model and dictionary files that will be used for the robot.
	- In the guide_voice_commands directory there is a directory called speech. The file speech_kb found in this directory was used to generate the language model and dictionary files used for GROVI. If the language needs to be modified this file should be modified and a new language model and dictionary file should be generated using CMU's online tool found here: http://www.speech.cs.cmu.edu/tools/lmtool-new.html
	- Directories ar_track_alvar, pocketsphinx, turtlebot_navigation and usb_cam include files that were used for this project and should be placed in the correct directory of each one of those packages to make the robot work properly.
	- The path found in the first line of the kingsMap.yaml file found in turtlebot_navigation/maps should match the directory of the kingsMap.pgm file.
	- The move_base_params.yaml file found in turtlebot_navigation/param deactives the turtlebot rotation recovery method and enables the clear costmap method instead.
	
##TO DO

The following are enhancements that could be do to this project
	- Create a node that handles the messages sent to sound_play_node to avoid repeated messages and stop messages from interrupting each other.
	- guide_ar_localization:
		- Map from ar tag ids to location names using a configuration file
		- Set timeout using a configuration file
	- guide_face_recog:
		- Set timeout using a configuration file
		- Change the feedback given by the robot to make it more understandable
	- guide_navigation_feedback:
		- Set timeout using a configuration file
		- Enhance the algorithm to increase reliability and responsiveness
		- Detect when the robot will rotate in order to give feedback to the user
		- Detect when there is a change in slope to give feedback to the user
		- Give the user feedback regarding the distance or percentage left to reach the goal
	- grovi_voice_commands:
		- Set timeout using a configuration file
		- Set location names and coordinates using a configuration file
	- pocketsphinx:
		- Try a more advanced voice to increase the quality of the feedback given by the robot
