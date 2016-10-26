#include <ros/ros.h>
#include <string>
#include <chrono>
#include <deque>
#include <queue>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"
#include "multimodal/MultimodalState.h"

// #include "perceptual_filter/FeaturesArr.h"
// #include "perceptual_filter/Feature.h"

#include "perceptual_filter/PoseRecognizer.h"


#include <boost/thread/mutex.hpp>

//#include <multimodal/MultimodalState.h>

//#include <kinect_bridge2/KinectBodies.h>
//#include <clm_bridge/ClmHeads.h>

typedef std::map<uint32_t, std::map<std::string, std::map<std::string, float>>> _signals;
typedef std::map<uint32_t, std::map<std::string, float>> _audience;


class FilterPerception
{
  public:

    struct FilteredBehaviorSignals { 
        FilteredBehaviorSignals( _signals r = std::map<uint32_t, std::map<std::string, std::map<std::string, float>>>() ,
        _signals a = std::map<uint32_t, std::map<std::string, std::map<std::string, float>>>() ,
        _audience p = std::map<uint32_t, std::map<std::string, float>>(), std::map<std::string, int32_t> l = std::map<std::string, int32_t>(), bool ns = false ,
        uint64_t sn = 0, std::string sb = "no sequence")
        : relative_behavior_signals( r ), absolute_behavior_signals( a ), audience_properties ( p ), labels(l), new_sequence(ns), sequence_number(sn), sequence_boundary(sb) {}
        
        _signals relative_behavior_signals;
        _signals absolute_behavior_signals;
        _audience audience_properties;

        std::map<std::string, int32_t> labels;
        bool new_sequence;
        uint64_t sequence_number;
        std::string sequence_boundary;

    };   

    FilterPerception();
    ~FilterPerception();

    void multimodalStateCallback(const multimodal::MultimodalState &msg);

    FilteredBehaviorSignals getFilteredBehaviorSignals();
    FilteredBehaviorSignals getQueuedFilteredBehaviorSignals();

  
  private:
    // float test_running_average;  
    // float test_limit_modality;
    // float test_running_avg_weight;

    // float test_margin; // this is stddev==sqrt(variance) variance = (diff^2 + diff^2 + ... + diff^2)/ count
    // float test_diff_squared;
    // float test_init_average;
    // float test_sum;
    // float test_prev_frown;
    // float test_count;

    int32_t timeFrameMinusOne_;
    int32_t timeFrameMinusTwo_ = -1;
    int32_t timeFrameMinusThree_ = -1;

    std::map<std::string, tf::Vector3> jointPositionsFrameMinusOne_;
    std::map<std::string, tf::Vector3> jointPositionsFrameMinusTwo_;

    std::map<std::string, tf::Vector3> jointPositionsNormalizedFrameMinusOne_;
    std::map<std::string, tf::Vector3> jointPositionsNormalizedFrameMinusTwo_;

    std::map<std::string, tf::Vector3> jointVelocitiesFrameMinusOne_;
    std::map<std::string, tf::Vector3> jointVelocitiesFrameMinusTwo_;


    std::vector<bool> heads_lastFrame_detectionFailed_;
    std::vector<bool> bodies_lastFrame_detectionFailed_;

    //int count_frames_;

    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> timers_start_clm_;
    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> timers_start_kinect_;
    // std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> timers_start_clm;
    std::vector<int> frame_counters_clm_;
    std::vector<int> frame_counters_kinect_;
    std::map<int, std::vector<std::string>> aus_feature_representation_;

	int data_collection_trial_;
	std::string model_;
	bool compare_learning_rates_;
	
	std::set<int> aus_r_toexamine_;
    
    std::map<std::string, std::map<std::string,float>> personalization_baseline_stats_;
    bool timer_started_clm_;
    bool timer_started_kinect_;

    bool michael_comp_;

    int num_rolling_avg_points_;
    std::map<std::string, std::deque<double>> threshold_values_;

    float personalizationBaseline(const std::string tracker, const int person_idx, const std::string modality_name, const float modality_intensity, const std::string message); //return failure cue intensity after personalization
    void outputPersonalizationPerModality(const std::string tracker, const int person_idx, const std::string modality, const float intensity, const float average, const float stddev, const float running_avg, const float rolling_avg, const float learning_rate, const bool signalling, const bool personalizing, const bool normalizing, float running_avg2 = 3, float running_avg3 = 3, float learning_rate2 = 0.025, float learning_rate3 = 0.0125);
    double getRotationAngle(double x1, double x2, double z1, double z2);



    std::map<uint8_t, std::string> buildJointNamesMap();


    PoseRecognition::PoseRecognizer pose_recognizer_;

    boost::mutex filtered_behaviors_mutex_;  
    FilteredBehaviorSignals filtered_behavior_signals_;


    std::queue<FilteredBehaviorSignals*> filtered_behavior_signals_queue_;

   // std::list<FilteredBehaviorSignals*> filtered_behavior_signals_queue_;

};
