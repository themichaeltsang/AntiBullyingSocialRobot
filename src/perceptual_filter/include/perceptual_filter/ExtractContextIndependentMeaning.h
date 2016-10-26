#include <ros/ros.h>
#include <string>
#include <chrono>
#include <deque>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <map>
#include <queue>
#include "std_msgs/String.h"
#include <boost/thread/mutex.hpp>


//#include <multimodal/MultimodalState.h>

//#include <kinect_bridge2/KinectBodies.h>
//#include <clm_bridge/ClmHeads.h>

class ExtractContextIndependentMeaning
{
  public:

    ExtractContextIndependentMeaning();
    ~ExtractContextIndependentMeaning();

	void publishSampleMessage(ros::Publisher *pub_message);


	std::map<uint32_t, std::map<std::string, float>> classifyInappropriateBehavior(std::map<uint32_t, std::map<std::string, std::map<std::string, float>>> &absolute_signals, std::map<uint32_t, std::map<std::string, std::map<std::string, float>>> &relative_signals, std::map<std::string, int32_t> &labels, bool new_sequence, uint64_t sequence_number, std::string sequence_boundary, ros::Publisher *pub_message);

  std::map<uint32_t, std::map<std::string, float>> classifyInappropriateBehaviorManually(std::map<uint32_t, std::map<std::string, std::map<std::string, float>>> &absolute_signals, std::map<uint32_t, std::map<std::string, std::map<std::string, float>>> &relative_signals); //inputs are absolute and relative signals, output is the type of inappriate behavior and its severity <key, value>
  
  void wekaClassifyCallback(const std_msgs::String::ConstPtr& msg); 

  std::map<uint32_t, std::map<std::string, float>> getInappropriateBehaviorClassification();


  private:
  	std::vector<std::string> joint_names_;
    std::vector<std::string> joint_names_pos_;
    std::vector<std::string> joint_names_vel_;

  	bool new_sequence_;
  	int count_;
    std::queue<std::string> classification_results_queue_;

    boost::mutex weka_mutex_;


  
};
