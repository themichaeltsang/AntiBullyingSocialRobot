ROS workspace for autonomous operation of an antibullying robot

#requirements:
* a ubuntu 14.04 machine with ROS Indigo installed
* a windows 8 or 10 machine. cannot be windows in a virtual machine, must be dual boot - preferably not on bootcamp
* both machines connected to the same router, a "strong" network connection is needed
* a kinect2 attached to the windows machine. the machine must support all the requirements of running a kinect2. the kinect2 sdk must be installed. note that windows 10 is currently supported without speech recognition from the kinect sdk or access to the kinect's microphones
* kinect_bridge2 installed on the windows machine
* bandit robot - only available at the Interaction Lab at USC - connected to the ubuntu computer
* a webcam attached to the ubuntu machine

#dependencies:
* install ROS-indigo, rosjava indigo, and OpenCV >=3.0 (https://github.com/TadasBaltrusaitis/CLM-framework/blob/master/readme-ubuntu.txt)
* sudo apt-get install libsndfile1-dev
* sudo apt-get install libsfml-dev

#compilation:
* in .bashrc, add: export ROS_MAVEN_DEPLOYMENT_REPOSITORY=~/catkin_indigo_ws/devel/share/maven
* in .bashrc, add: export ROS_MAVEN_PATH=~/catkin_indigo_ws/devel/share/maven
* in catkin_workspace folder, delete build and devel folders
* in the same folder, run "genjava_message_artifacts --verbose" to generate all java messages, before catkin_make
* then run "catkin_make -j8" 3 times
* go to java_classifiers folder, run ./gradlew deployApp
* copy dependencies of clm to devel/lib/clm_bridge
* "roslaunch perceptual_filter perceptual_filter.launch server_ip:=10.0.1.2"
* "source devel/setup.sh"

#special thanks:
* Vadim Korolik


instructions for installing kinect_bridge2 on windows/ubuntu, clm_bridge on ubuntu, and for setting up bandit's serial usb connection to be added later


