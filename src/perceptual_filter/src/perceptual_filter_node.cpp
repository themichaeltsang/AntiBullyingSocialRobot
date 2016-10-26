#include "perceptual_filter/FilterPerception.h"
#include "perceptual_filter/ExtractContextIndependentMeaning.h"
#include <perceptual_filter/AudienceBehavior.h>
#include <limits>
#include <chrono>
#include <thread>


#include "perceptual_filter/FeaturesArr.h"
#include "perceptual_filter/Feature.h"

typedef perceptual_filter::AudienceBehavior _AudienceBehaviorMsg;
typedef perceptual_filter::IndividualBehavior _IndividualBehaviorMsg;


typedef std::map<uint32_t, std::map<std::string, std::map<std::string, float>>> _signals;
typedef std::map<uint32_t, std::map<std::string, float>> _classifications;
typedef std::map<uint32_t, std::map<std::string, float>> _audience;

const bool filter_audience_ = false;
clock_t start_time_;


static double rad_to_deg(const double & rad)
{
  return rad * 180. / M_PI;
}

// void weka_callback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }


void publishContextIndependentBehaviorMessage(
	ros::Publisher *pub_message, 
    _signals &absolute_signals,
  	_classifications &inappropriate_behaviors, bool manualClassification )
{
	_AudienceBehaviorMsg audience_behavior_msg; 

  if (manualClassification)
  {
    audience_behavior_msg.classificationType = "manual";
  }

	for (auto const& entry : inappropriate_behaviors) // if there is an entry, this behavior was tracked
	{
    uint32_t person_index = entry.first;
		std::map<std::string, float> inappropriate_behavior = entry.second;

		_IndividualBehaviorMsg individual_behavior_msg;
		individual_behavior_msg.person_index = person_index;
		individual_behavior_msg.inappropriate_behavior.pointing_taunt = inappropriate_behavior["pointing_taunt"];
    individual_behavior_msg.inappropriate_behavior.pointing = inappropriate_behavior["pointing"];
    individual_behavior_msg.inappropriate_behavior.fist_threat = inappropriate_behavior["fist_threat"];
    individual_behavior_msg.inappropriate_behavior.hitting_prep = inappropriate_behavior["hitting_prep"];
    individual_behavior_msg.inappropriate_behavior.punching_prep = inappropriate_behavior["punching_prep"];
    individual_behavior_msg.inappropriate_behavior.kicking_prep = inappropriate_behavior["kicking_prep"];
    individual_behavior_msg.inappropriate_behavior.shoving_prep = inappropriate_behavior["shoving_prep"];
    individual_behavior_msg.inappropriate_behavior.punching = inappropriate_behavior["punching"];
    individual_behavior_msg.inappropriate_behavior.kicking = inappropriate_behavior["kicking"];
    individual_behavior_msg.inappropriate_behavior.shoving = inappropriate_behavior["shoving"];
    individual_behavior_msg.inappropriate_behavior.hitting = inappropriate_behavior["hitting"];

    individual_behavior_msg.inappropriate_behavior.making_faces_tongue_sticking_out_hands_at_ears = inappropriate_behavior["making_faces_tongue_sticking_out_hands_at_ears"];
    individual_behavior_msg.inappropriate_behavior.making_faces_tongue_sticking_out = inappropriate_behavior["making_faces_tongue_sticking_out"];

    individual_behavior_msg.head_position.x = absolute_signals[person_index]["kinect_body"]["head_position_x"];
    individual_behavior_msg.head_position.y = absolute_signals[person_index]["kinect_body"]["head_position_y"];
    individual_behavior_msg.head_position.z = absolute_signals[person_index]["kinect_body"]["head_position_z"];
    // if(individual_behavior_msg.inappropriate_behavior.right_arm_pointing_taunt == 1){
    //   std::cout << "TEASING!!!" << std::endl;
    // }

		audience_behavior_msg.audience.emplace_back( std::move( individual_behavior_msg ) );
	}
  
  //std::cout << "audience size: " << audience_behavior_msg.audience.size() << std::endl;
	// add other types of behaviors here

	pub_message->publish(audience_behavior_msg); //who receives this, how would this be processed??
}


void filterAudience(_signals& abs_signals)
{
  double threshold_field_of_view_angle = 32; // field of view angle to seek out closest body
  uint32_t min_std_body_idx = 1;
  float min_proxemics = std::numeric_limits<float>::max();

  for (auto const& idv_abs_signals : abs_signals)
  {
    int32_t std_body_idx = idv_abs_signals.first;  //key      
    std::map<std::string, std::map<std::string, float>> idv_abs_signals_value = idv_abs_signals.second; //value
    double field_of_view_angle_from_center = fabs( rad_to_deg( static_cast<double>(atan(idv_abs_signals_value["kinect_body"]["spine_mid_position_x"] / idv_abs_signals_value["kinect_body"]["spine_mid_position_z"])) ));
    //std::cout << "Angle: " << field_of_view_angle_from_center << std::endl;
    if (field_of_view_angle_from_center < threshold_field_of_view_angle / 2)
    {
      float proxemics = idv_abs_signals_value["kinect_body"]["spine_mid_position_z"];
      if (proxemics < min_proxemics)
      {
        min_proxemics = proxemics;
        min_std_body_idx = std_body_idx;
      }
    }
  }
  _signals new_abs_signals; //size 0
  if (min_proxemics != std::numeric_limits<float>::max())
  {
    new_abs_signals[min_std_body_idx] = abs_signals[min_std_body_idx];
  }
  abs_signals = new_abs_signals;

}





int main(int argc, char **argv)
{

  // Set up ROS.
  //string is name of node

  ros::init(argc, argv, "perceptual_filter_node");
  ros::NodeHandle nh;

  int rate = 400; //30

  FilterPerception *filter = new FilterPerception();
  ExtractContextIndependentMeaning *meaning_extractor = new ExtractContextIndependentMeaning();

  ros::Subscriber sub_msg = nh.subscribe("/multimodal/state", 1000, &FilterPerception::multimodalStateCallback, filter);
  //ros::Subscriber sub_msg2 = nh.subscribe("weka_publish", 1000, weka_callback);
  ros::Subscriber sub_msg2 = nh.subscribe("weka_publish", 1000, &ExtractContextIndependentMeaning::wekaClassifyCallback, meaning_extractor);
  ros::Publisher pub_message = nh.advertise<perceptual_filter::AudienceBehavior>("/perceptual_filter/behavior", 10);
  ros::Publisher pub_message2 = nh.advertise<perceptual_filter::FeaturesArr>("/perceptual_filter/features", 1000);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));


  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  //create a extractMeaning object

  start_time_ = clock();

  while (nh.ok())
  {
    ros::spinOnce();

    //FilterPerception::FilteredBehaviorSignals filtered_signals = filter->getFilteredBehaviorSignals(); //still need to filter relative signals of everyone in view
    FilterPerception::FilteredBehaviorSignals
    filtered_signals = filter->getQueuedFilteredBehaviorSignals(); //still need to filter relative signals of everyone in view

    _signals abs_signals = filtered_signals.absolute_behavior_signals;
    _signals rel_signals = filtered_signals.relative_behavior_signals; //currently empty
    _audience audience_properties = filtered_signals.audience_properties; //currently empty
    std::map<std::string,int32_t> labels = filtered_signals.labels;
    bool new_sequence = filtered_signals.new_sequence;
    uint64_t sequence_number = filtered_signals. sequence_number;
    std::string sequence_boundary = filtered_signals.sequence_boundary;

    // for (auto const& person_dict : abs_signals)
    // {
    //   for (auto const& modality_dict : person_dict.second)
    //   {
    //     std::cout << "keys: " + modality_dict.first << std::endl;

    //   }
    // }


  	if (filter_audience_) //assumes standard_idxs on all bodies, Find the closest person to the camera within a specified range near the center, only working on abs signals now
    { 
      filterAudience(abs_signals);
  	}

    meaning_extractor->classifyInappropriateBehavior(abs_signals, rel_signals, labels, new_sequence, sequence_number, sequence_boundary, &pub_message2 ); //need to create this method still. For now dont add audience properties t
    _classifications inappropriate_behavior = meaning_extractor->getInappropriateBehaviorClassification();

    //_classifications inappropriate_behavior = meaning_extractor->classifyInappropriateBehaviorManually(abs_signals, rel_signals); //need to create this method still. For now dont add audience properties t


    publishContextIndependentBehaviorMessage(&pub_message, abs_signals, inappropriate_behavior, false /*add anymore results here for context-specific decision-making*/);

    //publishContextIndependentBehaviorMessage(&pub_message, abs_signals, inappropriate_behavior /*add anymore results here for context-specific decision-making*/);

    //publishSampleMessage(&pub_message2);
    r.sleep();
  }

  return 0;
}
