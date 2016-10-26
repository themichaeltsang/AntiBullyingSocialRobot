#include "perceptual_filter/FilterPerception.h"


#define MAX_PEOPLE_TRACKED 10


using namespace std;

FilterPerception::FilterPerception()
{


  data_collection_trial_ = 20;  // identifier for what trial of data_collection -> better to change this into a datatime stamp
  model_ = "baseline"; //there is only one model right now: baseline
  compare_learning_rates_ = false; //only output info for one learning rate
  num_rolling_avg_points_ = 200; //might need to play with this number

  timer_started_clm_ = false;
  timer_started_kinect_ = false;
  aus_r_toexamine_ = {15, 4};

  //output msgs
  aus_feature_representation_[4] = {"brow-furrow","why are you furrowing your brow >:|"};
  aus_feature_representation_[15] = {"frown","why are you frowning :("};




  char * uName = new char[100];
  getlogin_r(uName, 100);
  michael_comp_ = (strstr(uName, "michael") != 0) ? true : false;
  delete uName;



  //create vector with all modality names
  //instatiate map with the names as keys and maps as values, where the maps all have the same key-value pairs. 
  
  // FOR THE BELOW, THE ONLY PARAMETERS THAT ARE CUSTOM ARE: learning rate and running avg
  for (auto au_type : aus_r_toexamine_)
  {
    string name = aus_feature_representation_[au_type][0];
    map<string,float> baseline_modality_stats;
    //baseline_modality_stats["running_avg"] = 3.0; // modality dependent set_to_equal limit
    baseline_modality_stats["diff_squared"] = 0;
    baseline_modality_stats["margin"] = 0; // stddev
    baseline_modality_stats["sum"] = 0;
    baseline_modality_stats["count"] = 0;
    baseline_modality_stats["average"] = 0;
    baseline_modality_stats["prev_intensity"] = 0; // no need to set
    baseline_modality_stats["count_cue"] = 0;
    baseline_modality_stats["sum_cue"] = 0;
    baseline_modality_stats["average_cue"] = 0;
    baseline_modality_stats["diff_squared_cue"] = 0;
    baseline_modality_stats["margin_cue"] = 0;
    //baseline_modality_stats["learning_rate"] = 0.05; // modality dependent, usually ~0.05
    
    baseline_modality_stats["learning_rate"] = 0.05;//0.05; 
    baseline_modality_stats["running_avg"] = 3.0;

    //if (compare_learning_rates_){
    baseline_modality_stats["learning_rate2"] = 0.025;
    baseline_modality_stats["running_avg2"] = 3.0;
    baseline_modality_stats["learning_rate3"] = 0.0125;
    baseline_modality_stats["running_avg3"] = 3.0;
    //}


    personalization_baseline_stats_[name] = baseline_modality_stats;
  }




  string name = "lean";
  map<string,float> baseline_modality_stats;
  baseline_modality_stats["diff_squared"] = 0;
  baseline_modality_stats["margin"] = 0; // stddev
  baseline_modality_stats["sum"] = 0;
  baseline_modality_stats["count"] = 0;
  baseline_modality_stats["average"] = 0;
  baseline_modality_stats["count_cue"] = 0;
  baseline_modality_stats["sum_cue"] = 0;
  baseline_modality_stats["average_cue"] = 0;
  baseline_modality_stats["diff_squared_cue"] = 0;
  baseline_modality_stats["margin_cue"] = 0;
  baseline_modality_stats["prev_intensity"] = 0; // no need to set
  baseline_modality_stats["learning_rate"] = 0.05;//0.05;
  baseline_modality_stats["running_avg"] = 1.0;
  baseline_modality_stats["learning_rate2"] = 0.025;
  baseline_modality_stats["running_avg2"] = 1.0;
  baseline_modality_stats["learning_rate3"] = 0.0125;
  baseline_modality_stats["running_avg3"] = 1.0;
  personalization_baseline_stats_[name] = baseline_modality_stats;

  string name2 = "crossed-arms";
  map<string,float> baseline_modality_stats2;
  baseline_modality_stats2["diff_squared"] = 0;
  baseline_modality_stats2["margin"] = 0; // stddev
  baseline_modality_stats2["sum"] = 0;
  baseline_modality_stats2["count"] = 0;
  baseline_modality_stats2["average"] = 0;
  baseline_modality_stats2["count_cue"] = 0;
  baseline_modality_stats2["sum_cue"] = 0;
  baseline_modality_stats2["average_cue"] = 0;
  baseline_modality_stats2["diff_squared_cue"] = 0;
  baseline_modality_stats2["margin_cue"] = 0;
  baseline_modality_stats2["prev_intensity"] = 0; // no need to set
  baseline_modality_stats2["learning_rate"] = 0.05;//0.05;
  baseline_modality_stats2["running_avg"] = 0.0;
  baseline_modality_stats2["learning_rate2"] = 0.025;
  baseline_modality_stats2["running_avg2"] = 0.0;
  baseline_modality_stats2["learning_rate3"] = 0.0125;
  baseline_modality_stats2["running_avg3"] = 0.0;
  personalization_baseline_stats_[name2] = baseline_modality_stats2;

  string name3 = "shove";
  map<string,float> baseline_modality_stats3;
  baseline_modality_stats3["diff_squared"] = 0;
  baseline_modality_stats3["margin"] = 0; // stddev
  baseline_modality_stats3["sum"] = 0;
  baseline_modality_stats3["count"] = 0;
  baseline_modality_stats3["average"] = 0;
  baseline_modality_stats3["count_cue"] = 0;
  baseline_modality_stats3["sum_cue"] = 0;
  baseline_modality_stats3["average_cue"] = 0;
  baseline_modality_stats3["diff_squared_cue"] = 0;
  baseline_modality_stats3["margin_cue"] = 0;
  baseline_modality_stats3["prev_intensity"] = 0; // no need to set
  baseline_modality_stats3["learning_rate"] = 0.05;//0.05;
  baseline_modality_stats3["running_avg"] = 0.0;
  baseline_modality_stats3["learning_rate2"] = 0.025;
  baseline_modality_stats3["running_avg2"] = 0.0;
  baseline_modality_stats3["learning_rate3"] = 0.0125;
  baseline_modality_stats3["running_avg3"] = 0.0;
  personalization_baseline_stats_[name3] = baseline_modality_stats3;



  // personalization_baseline_stats_["frown"]["learning_rate"] = 0.05;
  // personalization_baseline_stats_["frown"]["running_avg"] = 3.0;
  // personalization_baseline_stats_["brow-furrow"]["learning_rate"] = 0.05;
  // personalization_baseline_stats_["brow-furrow"]["running_avg"] = 3.0;

  heads_lastFrame_detectionFailed_.resize(MAX_PEOPLE_TRACKED,true);
  bodies_lastFrame_detectionFailed_.resize(MAX_PEOPLE_TRACKED,true);
  frame_counters_clm_.resize(MAX_PEOPLE_TRACKED,0);
  timers_start_clm_.resize(MAX_PEOPLE_TRACKED,chrono::time_point<chrono::high_resolution_clock>());

  frame_counters_kinect_.resize(MAX_PEOPLE_TRACKED,0);
  timers_start_kinect_.resize(MAX_PEOPLE_TRACKED,chrono::time_point<chrono::high_resolution_clock>());





  bool train = false;
  clock_t timer_start = clock();
  std::vector<std::string> testing_pose_names = {"crossed_arms","hands_at_ears","left_arm_pointing","right_arm_pointing","left_hand_covering_mouth","right_hand_covering_mouth","shove_preparation","left_hit_preparation","right_hit_preparation","left_kick_preparation","right_kick_preparation","left_arm_swing_preparation","right_arm_swing_preparation","showing_left_fist","showing_right_fist"}; //right hand hit doesnt exist...
  //{}

  std::string training_pose_name = "hands_at_ears";




  pose_recognizer_ = PoseRecognition::PoseRecognizer(train, timer_start, testing_pose_names, training_pose_name);



}

FilterPerception::~FilterPerception()
{

}




// a personalization filtering method 
float FilterPerception::personalizationBaseline(const string tracker, const int person_idx, const string modality_name, const float modality_intensity, const string message)
{
  float & count = personalization_baseline_stats_[modality_name]["count"];
  float & sum =  personalization_baseline_stats_[modality_name]["sum"];
  float & average = personalization_baseline_stats_[modality_name]["average"];
  float & diff_squared = personalization_baseline_stats_[modality_name]["diff_squared"];
  float & margin = personalization_baseline_stats_[modality_name]["margin"];

  float & count_cue = personalization_baseline_stats_[modality_name]["count_cue"];
  float & sum_cue =  personalization_baseline_stats_[modality_name]["sum_cue"];
  float & average_cue = personalization_baseline_stats_[modality_name]["average_cue"];
  float & diff_squared_cue = personalization_baseline_stats_[modality_name]["diff_squared_cue"];
  float & margin_cue = personalization_baseline_stats_[modality_name]["margin_cue"];

  float & prev_intensity = personalization_baseline_stats_[modality_name]["prev_intensity"];
  float & learning_rate = personalization_baseline_stats_[modality_name]["learning_rate"];
  float & running_avg = personalization_baseline_stats_[modality_name]["running_avg"];

  float & learning_rate2 = personalization_baseline_stats_[modality_name]["learning_rate2"];
  float & running_avg2 = personalization_baseline_stats_[modality_name]["running_avg2"];  
  float & learning_rate3 = personalization_baseline_stats_[modality_name]["learning_rate3"];
  float & running_avg3 = personalization_baseline_stats_[modality_name]["running_avg3"];
  //float learning_rate2, running_avg2, learning_rate3, running_avg3;
  // float running_avg2;
  // float learning_rate3;
  // float running_avg3;

  count ++;
  sum += modality_intensity; //add up modality intensity for averaging
  average = sum / count; //calculate average intensity
  diff_squared += pow((average - modality_intensity), 2); //sum the square of the difference between the average and the current intensity 
  margin = sqrt(diff_squared / count); //divide the sum of the difference squared by the count to get variance, and then take the sqrt to get stddev


  float intensity_after_personalization = 0.0;
  bool normalizing = (count <= 10) ? true : false;

  //the concept being the three conditions for personalizing are as follows: (count > 10 to signal normalizing is done, intensity >= previous intensity to avoid fitting to "smiles", intensity >= avg - margin to signal a lower bound for the intensity for negative personalization, and intensity < running avg to signal not to adapt when crossing a intensity peak)
  bool personalizing = (count > 10) && (modality_intensity >= prev_intensity) && (modality_intensity >= average_cue - margin); //&& (modality_intensity <= (running_avg/* + 2 * margin*/))? true : false;
  //bool personalizing2 = (count > 10) && (modality_intensity >= prev_intensity) && (modality_intensity >= average - margin) && (modality_intensity <= running_avg2)? true : false;
  //bool personalizing3 = (count > 10) && (modality_intensity >= prev_intensity) && (modality_intensity >= average - margin) && (modality_intensity <= running_avg3)? true : false;
  //bool personalizing = (count > 10) && (modality_intensity >= prev_intensity) && (modality_intensity >= average - margin) ? true : false;
  bool signalling = false;

  //TODO!!! consider eliminating this condition altogether for stddev calculation -> but need to think about how the average affects above condition: the idea is that if you are smiling alot, it is strange to all of a sudden frown! OR maybe dont think too hard, keep it simple and only calculate stddev when personalizing, so need to change??
  //if (normalizing || personalizing || ((count > 10) && (modality_intensity >= prev_intensity)) )// && (modality_intensity >= average - margin) ? true : false)) //train initial margin
  if (normalizing || personalizing )// && (modality_intensity >= average - margin) ? true : false)) //train initial margin
  {
    count_cue ++;
    sum_cue += modality_intensity;
    average_cue = sum_cue / count_cue;
    diff_squared_cue += pow((average_cue - modality_intensity), 2); //sum the square of the difference between the average and the current intensity 
    margin_cue = sqrt(diff_squared_cue / count_cue); //divide the sum of the difference squared by the count to get variance, and then take the sqrt to get stddev
  }

  if (personalizing) //recalculate margin if these conditions are met
  {
    // count ++;
    // sum += modality_intensity; //add up modality intensity for averaging
    // average = sum / count; //calculate average intensity
    // diff_squared += pow((average - modality_intensity), 2); //sum the square of the difference between the average and the current intensity 
    // margin = sqrt(diff_squared / count); //divide the sum of the difference squared by the count to get variance, and then take the sqrt to get stddev

    float fit_to_this;
    fit_to_this = margin_cue + modality_intensity;

    running_avg = learning_rate*fit_to_this + (1-learning_rate)*running_avg;
    running_avg2 = learning_rate2*fit_to_this + (1-learning_rate2)*running_avg2;
    running_avg3 = learning_rate3*fit_to_this + (1-learning_rate3)*running_avg3;
  }

  threshold_values_[modality_name].push_back(static_cast<double>(running_avg));
  //threshold_values_.push_back(static_cast<double>(1.5));
  if (threshold_values_[modality_name].size() > num_rolling_avg_points_)
    threshold_values_[modality_name].pop_front();

  double rolling_avg = 0.;
  for (int i = 0; i < threshold_values_[modality_name].size(); i++)
  {
    rolling_avg += threshold_values_[modality_name][i];
  }
  rolling_avg /= threshold_values_[modality_name].size();   

  if (modality_intensity > rolling_avg)
  {
    cout << message << endl;
    //cout << message << "  " << (modality_intensity-running_avg)/margin << " modality_intensity: " << modality_intensity << " running_avg: "<< running_avg << " modailty_stddev: " << margin << endl;
    //cout << message << "  " << (modality_intensity-running_avg)/margin << " modality_intensity: " << modality_intensity << " running_avg: "<< running_avg << " modailty_stddev: " << margin << " average: " << average << " sum: " << sum << " count: " << count << " diff_squared: " << diff_squared << endl;
    signalling = true;
    //intensity_after_personalization //TODO figure out how to set this
  }

  outputPersonalizationPerModality(tracker, person_idx, modality_name, modality_intensity, average, margin, running_avg, rolling_avg, learning_rate, signalling, personalizing, normalizing, running_avg2, running_avg3, learning_rate2, learning_rate3);

  prev_intensity = modality_intensity;

  return intensity_after_personalization;
}





// callback for multimodal message with all every modality combined, how do I access them? need to split between subjective and objective cues (relative vs absolute)
void FilterPerception::multimodalStateCallback(const multimodal::MultimodalState &msg)
{
  string sequence_boundary = msg.sequence_boundary;

  _signals absolute_behavior_signals;
  _signals relative_behavior_signals;
  _audience audience_properties;

  int32_t timeStampSingleBody;


  if (msg.kinect_bodies_msg_isvalid) {
    const auto & bodies = msg.kinect_bodies_msg.bodies;


    for (size_t body_idx = 0; body_idx < bodies.size(); ++body_idx)
    {
      const auto & body = bodies[body_idx];

      //std::cout << "std body index " << body.std_body_idx << std::endl;

      // START Send joint information into the pose recognizer
      auto const & joints_msg = body.joints;


      std::map<std::string, tf::Transform> joint_transforms_map;
      std::map<std::string, std::pair<double, double>> body_twists_features;
      std::map<std::string, tf::Vector3> joint_positions;


      for( size_t joint_idx = 0; joint_idx < joints_msg.size(); ++joint_idx )
      {
        auto const & joint_msg = joints_msg[joint_idx];

        tf::Transform joint_transform
        (
            joint_msg.tracking_state == kinect_bridge2::KinectJoint::TRACKING_STATE_TRACKED ? tf::Quaternion( joint_msg.orientation.x, joint_msg.orientation.y, joint_msg.orientation.z, joint_msg.orientation.w ).normalized() : tf::Quaternion( 0, 0, 0, 1 ),
            tf::Vector3( joint_msg.position.x, joint_msg.position.y, joint_msg.position.z )
        );

        if( std::isnan( joint_transform.getRotation().getAngle() ) ) joint_transform.setRotation( tf::Quaternion( 0, 0, 0, 1 ) );

        static tf::Transform const trunk_norm_rotation_tf( tf::Quaternion( -M_PI_2, -M_PI_2, 0 ).normalized() );

        switch( joint_msg.joint_type )
        {
        case kinect_bridge2::KinectJoint::JOINT_TYPE_SPINE_BASE:
        case kinect_bridge2::KinectJoint::JOINT_TYPE_SPINE_MID:
        case kinect_bridge2::KinectJoint::JOINT_TYPE_NECK:
        case kinect_bridge2::KinectJoint::JOINT_TYPE_HEAD:
        case kinect_bridge2::KinectJoint::JOINT_TYPE_SPINE_SHOULDER:
            joint_transform *= trunk_norm_rotation_tf;
            break;
        default:
            break;
        }

        auto const & joint_names_map = buildJointNamesMap();
        std::string joint_name = joint_names_map.find(joint_msg.joint_type)->second;

        joint_transforms_map[joint_name] = joint_transform;


        if (joint_name == "hip_left" || joint_name == "hip_right" || joint_name == "foot_left" || joint_name == "foot_right" || joint_name == "spine_mid")
        {
            body_twists_features[joint_name] = std::make_pair(joint_msg.position.x, joint_msg.position.z);
        }

        joint_positions[joint_name] = tf::Vector3(joint_msg.position.x, joint_msg.position.y, joint_msg.position.z);
      }

      std::map<std::string, tf::Vector3> normalized_joint_positions = pose_recognizer_.GetNormalizedJointPositions(joint_positions, "head");

      // std::map<std::string, double> pose_predictions;
      // pose_predictions = pose_recognizer_.PredictCurrentPoses(joint_transforms_map);
      //std::map<std::string, tf::Vector3> normalized_joint_positions = pose_recognizer_.GetNormalizedJointPositions(joint_transforms_map, "full_body"); //Deprecated
      //std::map<std::string, double> normalized_joint_angles = pose_recognizer_.GetNormalizedJointAngles(joint_positions, "aggressive_behaviors");
      std::map<std::string, double> normalized_joint_angles = pose_recognizer_.GetNormalizedJointAngles(joint_positions, "aggressive_behaviors");



      double hip_twist_angle =  getRotationAngle(body_twists_features["hip_left"].first, body_twists_features["hip_right"].first, body_twists_features["hip_left"].second, body_twists_features["hip_right"].second);
      //double shoulder_stance_twist_angle = getRotationAngle(body_twists_features["shoulder_left"].first, body_twists_features["shoulder_right"].first, body_twists_features["shoulder_left"].second, body_twists_features["shoulder_right"].second);
      double foot_stance_twist_angle =  getRotationAngle(body_twists_features["foot_left"].first, body_twists_features["foot_right"].first, body_twists_features["foot_left"].second, body_twists_features["foot_right"].second);
      double proxemics = body_twists_features["spine_mid"].second;



      // END

      if (body.is_tracked)
      {

        //std::cout << "received body message" << std::endl;

        //THIS IS WHERE I SHOULD CALCULATE POSE!


        //absolute_behavior_signals[body.std_body_idx]["kinect_body"]["is_tracked"] = body.is_tracked; //this should be true
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["time_stamp"] = body.time_stamp;
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["frame"] = body.frame;
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["hand_state_left"] = body.hand_state_left;
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["hand_state_right"] = body.hand_state_right;
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["lean_x"] = body.lean.x;
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["lean_y"] = body.lean.y;
        //std::cout << "body lean y: " << body.lean.y << std::endl;

        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["hip_rotation"] = static_cast<float>(hip_twist_angle);
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["foot_stance_rotation"] = static_cast<float>(foot_stance_twist_angle);
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["proxemics"] = static_cast<float>(proxemics);
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["head_position_x"] = joint_positions["head"].x();
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["head_position_y"] = joint_positions["head"].y();
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["head_position_z"] = joint_positions["head"].z();
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["spine_mid_position_x"] = joint_positions["spine_mid"].x();
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["spine_mid_position_y"] = joint_positions["spine_mid"].y();
        absolute_behavior_signals[body.std_body_idx]["kinect_body"]["spine_mid_position_z"] = joint_positions["spine_mid"].z();

        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["hip_rotation"] = body.orientation.hip_twist;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["proxemics"] = body.proxemics;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["foot_stance_rotation"] = body.orientation.foot_stance_twist;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["head_position_x"] = body.head_position.x;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["head_position_y"] = body.head_position.y;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["head_position_z"] = body.head_position.z;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["spine_mid_position_x"] = body.spine_mid_position.x;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["spine_mid_position_y"] = body.spine_mid_position.y;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["spine_mid_position_z"] = body.spine_mid_position.z;





        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["hands_at_ears"] = pose_predictions["hands_at_ears"];
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_hand_covering_mouth"] = pose_predictions["right_hand_covering_mouth"];
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_hand_covering_mouth"] = pose_predictions["left_hand_covering_mouth"];
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_arm_pointing"] = pose_predictions["right_arm_pointing"];
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_arm_pointing"] = pose_predictions["left_arm_pointing"];

        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["crossed_arms"] = pose_predictions["crossed_arms"];
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["showing_right_fist"] = pose_predictions["showing_right_fist"];
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["showing_left_fist"] = pose_predictions["showing_left_fist"];

        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["shove_preparation"] = pose_predictions["shove_preparation"];
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_hit_preparation"] = pose_predictions["right_hit_preparation"]; 
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_hit_preparation"] = pose_predictions["left_hit_preparation"]; 
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_kick_preparation"] = pose_predictions["right_kick_preparation"]; 
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_kick_preparation"] = pose_predictions["left_kick_preparation"]; 
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_arm_swing_preparation"] = pose_predictions["right_arm_swing_preparation"]; 
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_arm_swing_preparation"] = pose_predictions["left_arm_swing_preparation"]; 



        for (auto const & joint_position_entry : joint_positions)
        {
          tf::Vector3 joint_position = joint_position_entry.second;
          std::string joint_name = joint_position_entry.first;
          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_x"] = joint_position.x();
          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_y"] = joint_position.y();
          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_z"] = joint_position.z();
        }

        for (auto const & joint_position_entry : normalized_joint_positions)
        {
          tf::Vector3 joint_position = joint_position_entry.second;
          std::string joint_name = joint_position_entry.first;
          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_x_norm"] = joint_position.x();
          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_y_norm"] = joint_position.y();
          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_z_norm"] = joint_position.z();
        }


  //      	float currentSingleBodyRightWristX;

		// //normalized_joint_positions
  //       for (auto const & joint_position_entry : joint_positions)
  //       {
  //         std::string joint_name = joint_position_entry.first;

  //         if (joint_name == "wrist_right")
  //         {	
  //         	currentSingleBodyRightWristX = joint_position.z();

  //         }

  //       }


        //The following code determine the velocity of skeletal actions 

        // compare a joint of interest (right hand for example)
        if (sequence_boundary == "start")
        {
          timeFrameMinusTwo_ = -1; // reset this number to -1 at start of each sequence
          timeFrameMinusThree_ = -1;
          timeFrameMinusOne_ = body.time_stamp;
          jointPositionsFrameMinusOne_ = joint_positions;
          jointPositionsNormalizedFrameMinusOne_ = normalized_joint_positions;
        }
        else
        {
          if (timeFrameMinusTwo_!= -1) 
          {
            if (timeFrameMinusThree_ == -1)
            {
              sequence_boundary = "start";
            }
            int32_t dT = body.time_stamp - timeFrameMinusTwo_;


            if (dT != 0) // this is when msgs should be published
            {
              for (auto const & joint_position_entry : joint_positions)
              {
                tf::Vector3 joint_position = joint_position_entry.second;
                std::string joint_name = joint_position_entry.first;
                absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_x_vel"] = 1000* (joint_position.x() - jointPositionsFrameMinusTwo_[joint_name].x()) / dT;
                absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_y_vel"] = 1000* (joint_position.y() - jointPositionsFrameMinusTwo_[joint_name].y()) / dT;
                absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_z_vel"] = 1000* (joint_position.z() - jointPositionsFrameMinusTwo_[joint_name].z()) / dT;

                absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_x_avg"] = (normalized_joint_positions[joint_name].x() + jointPositionsNormalizedFrameMinusOne_[joint_name].x() + jointPositionsNormalizedFrameMinusTwo_[joint_name].x()) / 3.0;
                absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_y_avg"] = (normalized_joint_positions[joint_name].y() + jointPositionsNormalizedFrameMinusOne_[joint_name].y() + jointPositionsNormalizedFrameMinusTwo_[joint_name].y()) / 3.0;
                absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_z_avg"] = (normalized_joint_positions[joint_name].z() + jointPositionsNormalizedFrameMinusOne_[joint_name].z() + jointPositionsNormalizedFrameMinusTwo_[joint_name].z()) / 3.0;
                
                
                //cout << "v dX/dT " << (dX/dT)*1000 << endl;
                //wrist_right_z_vel
              }
              assert(jointPositionsFrameMinusTwo_!=joint_positions);
              assert(timeFrameMinusTwo_!=body.time_stamp);

              timeFrameMinusThree_ = timeFrameMinusTwo_;

              timeFrameMinusTwo_ = timeFrameMinusOne_;
              jointPositionsNormalizedFrameMinusTwo_ = jointPositionsNormalizedFrameMinusOne_;
              jointPositionsFrameMinusTwo_ = jointPositionsFrameMinusOne_;

              timeFrameMinusOne_ = body.time_stamp;
              jointPositionsNormalizedFrameMinusOne_ = normalized_joint_positions;
              jointPositionsFrameMinusOne_ = joint_positions;

            }
            // else
            // {
            //   exit(EXIT_FAILURE);
            // }
          }
          else
          {
            timeFrameMinusTwo_ = timeFrameMinusOne_;
            jointPositionsNormalizedFrameMinusTwo_ = jointPositionsNormalizedFrameMinusOne_;
            jointPositionsFrameMinusTwo_ = jointPositionsFrameMinusOne_;

            timeFrameMinusOne_ = body.time_stamp;
            jointPositionsNormalizedFrameMinusOne_ = normalized_joint_positions;
            jointPositionsFrameMinusOne_ = joint_positions;
          }
        }


       //  // compare a joint of interest (right hand for example)
       //  if (sequence_boundary == "start")
       //  {
    			// timeFrameMinusTwo_ = -1; // reset this number to -1 at start of each sequence
    			// timeFrameMinusThree_ = -1;
    			// timeFrameMinusOne_ = body.time_stamp;
       //    jointPositionsFrameMinusOne_.clear();
       //    jointPositionsFrameMinusOne_.insert(joint_positions.begin(), joint_positions.end());

       //    jointPositionsNormalizedFrameMinusOne_.clear();
       //    jointPositionsNormalizedFrameMinusOne_.insert(normalized_joint_positions.begin(), normalized_joint_positions.end());
       //  }
       //  else
       //  {
       //  	if (timeFrameMinusTwo_!= -1) 
       //  	{
       //  		if (timeFrameMinusThree_ == -1)
       //  		{
       //  			sequence_boundary = "start";
       //  		}
       //  		int32_t dT = body.time_stamp - timeFrameMinusTwo_;

       //  		if (dT != 0) // this is when msgs should be published
       //  		{
    			// 		for (auto const & joint_position_entry : joint_positions)
    			// 		{
    			// 			tf::Vector3 joint_position = joint_position_entry.second;
    			// 			std::string joint_name = joint_position_entry.first;
    			// 			absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_x_vel"] = 1000* (joint_position.x() - jointPositionsFrameMinusTwo_[joint_name].x()) / dT;
    			// 			absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_y_vel"] = 1000* (joint_position.y() - jointPositionsFrameMinusTwo_[joint_name].y()) / dT;
    			// 			absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_z_vel"] = 1000* (joint_position.z() - jointPositionsFrameMinusTwo_[joint_name].z()) / dT;

       //          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_x_avg"] = (joint_position.x() + jointPositionsNormalizedFrameMinusOne_.x() + jointPositionsNormalizedFrameMinusTwo_.x()) / 3.0;
       //          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_y_avg"] = (joint_position.y() + jointPositionsNormalizedFrameMinusOne_.y() + jointPositionsNormalizedFrameMinusTwo_.y()) / 3.0;
       //          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_z_avg"] = (joint_position.z() + jointPositionsNormalizedFrameMinusOne_.z() + jointPositionsNormalizedFrameMinusTwo_.z()) / 3.0;
                
                
    			// 			//cout << "v dX/dT " << (dX/dT)*1000 << endl;
    			// 			//wrist_right_z_vel
    			// 		}
       //        assert(jointPositionsFrameMinusTwo_!=joint_positions);
       //        assert(timeFrameMinusTwo_!=body.time_stamp);

       //        timeFrameMinusThree_ = timeFrameMinusTwo_;

       //        timeFrameMinusTwo_ = timeFrameMinusOne_;
       //        jointPositionsNormalizedFrameMinusTwo_.clear();
       //        jointPositionsNormalizedFrameMinusTwo_.insert(jointPositionsNormalizedFrameMinusOne_.begin(), jointPositionsNormalizedFrameMinusOne_.end());
       //        jointPositionsFrameMinusTwo_.clear();
       //        jointPositionsFrameMinusTwo_.insert(jointPositionsFrameMinusOne_.begin(), jointPositionsFrameMinusOne_.end());

    			// 		timeFrameMinusOne_ = body.time_stamp;
       //        jointPositionsNormalizedFrameMinusOne_.clear();
       //        jointPositionsNormalizedFrameMinusOne_.insert(normalized_joint_positions.begin(), normalized_joint_positions.end());
       //        jointPositionsFrameMinusOne_.clear();
       //        jointPositionsFrameMinusOne_.insert(joint_positions.begin(), joint_positions.end());

    			// 	}
       //  	}
       //  	else
       //  	{
       //      timeFrameMinusTwo_ = timeFrameMinusOne_;
       //      jointPositionsNormalizedFrameMinusTwo_.clear();
       //      jointPositionsNormalizedFrameMinusTwo_.insert(jointPositionsNormalizedFrameMinusOne_.begin(), jointPositionsNormalizedFrameMinusOne_.end());
       //      jointPositionsFrameMinusTwo_.clear();
       //      jointPositionsFrameMinusTwo_.insert(jointPositionsFrameMinusOne_.begin(), jointPositionsFrameMinusOne_.end());

       //      timeFrameMinusOne_ = body.time_stamp;
       //      jointPositionsNormalizedFrameMinusOne_.clear();
       //      jointPositionsNormalizedFrameMinusOne_.insert(normalized_joint_positions.begin(), normalized_joint_positions.end());
       //      jointPositionsFrameMinusOne_.clear();
       //      jointPositionsFrameMinusOne_.insert(joint_positions.begin(), joint_positions.end());
      	// 	}
       //  }


        for (auto const & joint_angle_entry : normalized_joint_angles)
        {
          double joint_angle = joint_angle_entry.second;
          std::string joint_name = joint_angle_entry.first;
          //cout << joint_name + "_angle" << joint_angle << endl;
          absolute_behavior_signals[body.std_body_idx]["kinect_body"][joint_name + "_angle"] = joint_angle;
        }





        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["hands_at_ears"] = body.pose.hands_at_ears;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_hand_covering_mouth"] = body.pose.right_hand_covering_mouth;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_hand_covering_mouth"] = body.pose.left_hand_covering_mouth;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_arm_pointing"] = body.pose.right_arm_pointing;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_arm_pointing"] = body.pose.left_arm_pointing;

        // //absolute_behavior_signals[body.std_body_idx]["kinect_body"]["crossed_arms"] = body.pose.crossed_arms;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["showing_right_fist"] = body.pose.showing_right_fist;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["showing_left_fist"] = body.pose.showing_left_fist;

        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["shove_preparation"] = body.pose.shove_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_hit_preparation"] = body.pose.right_hit_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_hit_preparation"] = body.pose.left_hit_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_kick_preparation"] = body.pose.right_kick_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_kick_preparation"] = body.pose.left_kick_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_arm_swing_preparation"] = body.pose.right_swing_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_arm_swing_preparation"] = body.pose.left_swing_preparation;


        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["showing_right_fist_elbow_up"] = body.pose.showing_right_fist_elbow_up;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["showing_left_fist_elbow_up"] = body.pose.showing_left_fist_elbow_up;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["showing_right_fist_elbow_down"] = body.pose.showing_right_fist_elbow_down;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["showing_left_fist_elbow_down"] = body.pose.showing_left_fist_elbow_down;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_arm_vertical_swing_preparation"] = body.pose.right_arm_vertical_swing_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_arm_vertical_swing_preparation"] = body.pose.left_arm_vertical_swing_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_arm_horizontal_swing_preparation"] = body.pose.right_arm_horizontal_swing_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_arm_horizontal_swing_preparation"] = body.pose.left_arm_horizontal_swing_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["right_arm_diagonal_swing_preparation"] = body.pose.right_arm_diagonal_swing_preparation;
        // absolute_behavior_signals[body.std_body_idx]["kinect_body"]["left_arm_diagonal_swing_preparation"] = body.pose.left_arm_diagonal_swing_preparation;




      //   // cout << "Body: " << body_idx << " Lean X: " << body.lean.x;
       //   cout << " Lean Y: " << body.lean.y << endl;

      //   cout << "Crossed Arms: " << body.pose.crossed_arms << endl;

      //   if (bodies_lastFrame_detectionFailed_[body_idx] && !timer_started_kinect_)
      //   {
      //     timer_started_kinect_ = true;
      //     frame_counters_kinect_[body_idx] = 0;
      //     timers_start_kinect_[body_idx] = chrono::high_resolution_clock::now();
      //   }
      //   frame_counters_kinect_[body_idx] ++;
      //   if(model_ == "baseline") personalizationBaseline("kinect", body_idx, "lean", (-1)*body.lean.y, "why are you leaning back?");

      //   frame_counters_kinect_[body_idx] ++;
      //   if(model_ == "baseline") personalizationBaseline("kinect", body_idx, "crossed-arms", (-1)*body.pose.crossed_arms, "why are you crossing your arms?");

      //   frame_counters_kinect_[body_idx] ++;
      //   if(model_ == "baseline") personalizationBaseline("kinect", body_idx, "shove", (-1)*body.pose.shove, "why are you shoving me?");

      //   bodies_lastFrame_detectionFailed_[body_idx] = body.is_tracked ? false : true;

      }
    }
  }



  if (msg.kinect_faces_msg_isvalid) {
    const auto & faces = msg.kinect_faces_msg.faces;

    for (size_t face_idx = 0; face_idx < faces.size(); ++face_idx)
    {
      const auto & face = faces[face_idx];
      if (face.is_tracked)
      {
        //absolute_behavior_signals[face.std_body_idx]["kinect_face"]["is_tracked"] = face.is_tracked; //this should be true
        absolute_behavior_signals[face.std_body_idx]["kinect_face"]["time_stamp"] = face.time_stamp;
        absolute_behavior_signals[face.std_body_idx]["kinect_face"]["frame"] = face.frame;
        absolute_behavior_signals[face.std_body_idx]["kinect_face"]["yaw"] = face.pose.yaw;
        absolute_behavior_signals[face.std_body_idx]["kinect_face"]["pitch"] = face.pose.pitch;
        absolute_behavior_signals[face.std_body_idx]["kinect_face"]["roll"] = face.pose.roll;
      }
    }
  }

  if (msg.kinect_hdfaces_msg_isvalid) {
    const auto & hdfaces = msg.kinect_hdfaces_msg.hdfaces;

    for (size_t hdface_idx = 0; hdface_idx < hdfaces.size(); ++hdface_idx)
    {
      const auto & hdface = hdfaces[hdface_idx];
      if (hdface.is_tracked)
      {
        //absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["is_tracked"] = hdface.is_tracked; //this should be true
        absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["time_stamp"] = hdface.time_stamp;
        absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["frame"] = hdface.frame;
        absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["yaw"] = hdface.pose.yaw;
        absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["pitch"] = hdface.pose.pitch;
        absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["roll"] = hdface.pose.roll;
        absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["tongue_sticking_out"] = hdface.tongue_sticking_out;
        absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["tongue_depth"] = hdface.tongue_depth;
        absolute_behavior_signals[hdface.std_body_idx]["kinect_hdface"]["upper_lip_depth"] = hdface.upper_lip_depth;

      }
    }
  }

  if (msg.kinect_clmheads_msg_isvalid) {
    const auto & clmheads = msg.kinect_clmheads_msg.heads;

    for (size_t clmhead_idx = 0; clmhead_idx < clmheads.size(); ++clmhead_idx)
    {
      const auto & clmhead = clmheads[clmhead_idx];
      if (clmhead.detection_success)
      {
        //absolute_behavior_signals[clmhead_idx]["kinect_clmhead"]["detection_success"] = clmhead.detection_success; //this should be true
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["time_stamp"] = clmhead.time_stamp;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["frame"] = clmhead.frame;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["yaw"] = clmhead.headpose.yaw;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["pitch"] = clmhead.headpose.pitch;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["roll"] = clmhead.headpose.roll;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["position_x"] = clmhead.headpose.x;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["position_y"] = clmhead.headpose.y;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["position_z"] = clmhead.headpose.z;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["left_eyegaze_cameraref_x"] = clmhead.left_eyegaze_cameraref.x;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["left_eyegaze_cameraref_y"] = clmhead.left_eyegaze_cameraref.y;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["left_eyegaze_cameraref_z"] = clmhead.left_eyegaze_cameraref.z;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["right_eyegaze_cameraref_x"] = clmhead.right_eyegaze_cameraref.x;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["right_eyegaze_cameraref_y"] = clmhead.right_eyegaze_cameraref.y;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["right_eyegaze_cameraref_z"] = clmhead.right_eyegaze_cameraref.z;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["left_eyegaze_headref_x"] = clmhead.left_eyegaze_headref.x;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["left_eyegaze_headref_y"] = clmhead.left_eyegaze_headref.y;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["left_eyegaze_headref_z"] = clmhead.left_eyegaze_headref.z;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["right_eyegaze_headref_x"] = clmhead.right_eyegaze_headref.x;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["right_eyegaze_headref_y"] = clmhead.right_eyegaze_headref.y;
        absolute_behavior_signals[clmhead.std_body_idx]["kinect_clmhead"]["right_eyegaze_headref_z"] = clmhead.right_eyegaze_headref.z;
      }
    }
  }


  FilterPerception::FilteredBehaviorSignals *filtered_behavior_signals = new FilterPerception::FilteredBehaviorSignals( relative_behavior_signals, absolute_behavior_signals, audience_properties );

  if (msg.labels_arevalid)
  {
    const auto & labels = msg.labels_msg;

    std::map<std::string, int32_t> labelsMap;

    for (size_t label_idx = 0; label_idx < labels.size(); ++label_idx)
    {
      const auto & label = labels[label_idx];
      labelsMap[label.labelName] = label.labelValue;
    }

    filtered_behavior_signals->labels = labelsMap;

    filtered_behavior_signals->new_sequence = msg.new_sequence;

    if (msg.sequence_number >= 1)
    {

      filtered_behavior_signals->sequence_number = msg.sequence_number;
      filtered_behavior_signals->sequence_boundary = sequence_boundary;
    }

  }


  {
    boost::mutex::scoped_lock lock(filtered_behaviors_mutex_); //proabably a good idea to make a queue of filtered_behavior_signals...
    filtered_behavior_signals_queue_.push(filtered_behavior_signals);
    //filtered_behavior_signals_ = filtered_behavior_signals;
  }



}

FilterPerception::FilteredBehaviorSignals FilterPerception::getFilteredBehaviorSignals()
{
  boost::mutex::scoped_lock lock(filtered_behaviors_mutex_);
  return filtered_behavior_signals_;
}


FilterPerception::FilteredBehaviorSignals FilterPerception::getQueuedFilteredBehaviorSignals()
{
  boost::mutex::scoped_lock lock(filtered_behaviors_mutex_);
  if (!filtered_behavior_signals_queue_.empty())
  {
    auto signals_ptr = filtered_behavior_signals_queue_.front();

    FilterPerception::FilteredBehaviorSignals signals = *signals_ptr;

    // for (auto const& person_dict : signals.absolute_behavior_signals)
    // {
    //   std::cout << "keys1: " + person_dict.first << std::endl;

    //   for (auto const& modality_dict : person_dict.second)
    //   {
    //     std::cout << "keys2: " + modality_dict.first << std::endl;
    //   }
    // }

    filtered_behavior_signals_queue_.pop();
    delete signals_ptr;
    return signals;
  }

  FilterPerception::FilteredBehaviorSignals filtered_behavior_signals;
  return filtered_behavior_signals; // add return statement, check queue size while running
}



void FilterPerception::outputPersonalizationPerModality(const string tracker, const int person_idx, const string modality, const float intensity, const float average, const float stddev, const float running_avg, const float rolling_avg, const float learning_rate, const bool signalling, const bool personalizing, const bool normalizing, float running_avg2, float running_avg3, float learning_rate2, float learning_rate3)
{
  int output_signalling = signalling ? 1 : 0;
  int output_personalizing = personalizing ? 1 : 0;
  int output_normalizing = normalizing ? 1 : 0;

  double time_stamp;
  auto now = chrono::high_resolution_clock::now();

 // ROS_INFO("reached method for outputting values");

  if (tracker == "clm") 
  {
    string filename;
    if (compare_learning_rates_) filename = "person" + to_string(person_idx) + "_clm_" + modality + "_personalization_" + model_ + "_trial" + to_string(data_collection_trial_) + "_compareLearningRates.csv";
    else filename = "person" + to_string(person_idx) + "_clm_" + modality + "_personalization_" + model_ + "_trial" + to_string(data_collection_trial_) + ".csv";

    if (michael_comp_)
      filename = "/home/michael/Dropbox/bridge_files/" + filename;
    else
      filename = "/home/vadim/Dropbox/bridge/" + filename;

    ofstream file;
    ifstream infile(filename);

    time_stamp = chrono::duration_cast<chrono::milliseconds>(now - timers_start_clm_[person_idx]).count();
    //if past frame didnt detect my head... then I need to intialize the output file 
    if (heads_lastFrame_detectionFailed_[person_idx])
    {
      if (!infile.good()) 
      {     //file doesn't exist:
        file.open(filename);
        file << "time,frame_number,intensity,average,stddev,running_avg(" << learning_rate << "),signalling,personalizing,normalizing,rolling_avg";
        if (compare_learning_rates_)
        {
          file << ",running_avg2(" << learning_rate2 << "),running_avg3(" << learning_rate3 << ")";
        }
        file << endl;
        file.close();
      }
    }
    //Append training data to file
    if (infile.good()) //file must exist, I want to append to it
    {  
      file.open(filename, ios_base::app);
      file << time_stamp << "," << frame_counters_clm_[person_idx] << "," << intensity << "," << average << "," << stddev << "," << running_avg << "," << output_signalling << "," << output_personalizing << "," << output_normalizing << "," << rolling_avg;
      if (compare_learning_rates_)
      {
        file << "," << running_avg2 << "," << running_avg3;
      }
      file << endl;
      file.close();
    }
  }
  else if (tracker == "kinect")
  {
    string filename;
    if (compare_learning_rates_) filename = "person" + to_string(person_idx) + "_kinect_" + modality + "_personalization_" + model_ + "_trial" + to_string(data_collection_trial_) + "_compareLearningRates.csv";
    else filename = "person" + to_string(person_idx) + "_kinect_" + modality + "_personalization_" + model_ + "_trial" + to_string(data_collection_trial_) + ".csv";
    
    if (michael_comp_)
      filename = "/home/michael/Dropbox/bridge_files/" + filename;
    else
      filename = "/home/vadim/Dropbox/bridge/" + filename;

    ofstream file;
    ifstream infile(filename);

    time_stamp = chrono::duration_cast<chrono::milliseconds>(now - timers_start_kinect_[person_idx]).count();
    //if past frame didnt detect my head... then I need to intialize the output file 
    if (heads_lastFrame_detectionFailed_[person_idx])
    {
      if (!infile.good()) 
      {     //file doesn't exist:
        file.open(filename);
        file << "time,frame_number,intensity,average,stddev,running_avg(" << learning_rate << "),signalling,personalizing,normalizing,rolling_avg";
        if (compare_learning_rates_)
        {
          file << ",running_avg2(" << learning_rate2 << "),running_avg3(" << learning_rate3 << ")";
        }
        file << endl;
        file.close();
      }
    }
    //Append training data to file
    if (infile.good()) //file must exist, I want to append to it
    {  
      file.open(filename, ios_base::app);
      file << time_stamp << "," << frame_counters_kinect_[person_idx] << "," << intensity << "," << average << "," << stddev << "," << running_avg << "," << output_signalling << "," << output_personalizing << "," << output_normalizing << "," << rolling_avg;
      if (compare_learning_rates_)
      {
        file << "," << running_avg2 << "," << running_avg3;
      }
      file << endl;
      file.close();
    }
  }

  //else {time_stamp = chrono::duration_cast<chrono::milliseconds>(now).count();} DOESNT WORK
}


std::map<uint8_t, std::string> FilterPerception::buildJointNamesMap()
{
    std::map<uint8_t, std::string> joint_names_map;
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_SPINE_BASE] = "spine_base";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_SPINE_MID] = "spine_mid";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_NECK] = "neck";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_HEAD] = "head";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_SHOULDER_LEFT] = "shoulder_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_ELBOW_LEFT] = "elbow_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_WRIST_LEFT] = "wrist_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_HAND_LEFT] = "hand_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_SHOULDER_RIGHT] = "shoulder_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_ELBOW_RIGHT] = "elbow_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_WRIST_RIGHT] = "wrist_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_HAND_RIGHT] = "hand_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_HIP_LEFT] = "hip_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_KNEE_LEFT] = "knee_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_ANKLE_LEFT] = "ankle_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_FOOT_LEFT] = "foot_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_HIP_RIGHT] = "hip_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_KNEE_RIGHT] = "knee_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_ANKLE_RIGHT] = "ankle_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_FOOT_RIGHT] = "foot_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_SPINE_SHOULDER] = "spine_shoulder";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_HANDTIP_LEFT] = "handtip_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_THUMB_LEFT] = "thumb_left";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_HANDTIP_RIGHT] = "handtip_right";
    joint_names_map[kinect_bridge2::KinectJoint::JOINT_TYPE_THUMB_RIGHT] = "thumb_right";
    return joint_names_map;
}


double FilterPerception::getRotationAngle(double x1, double x2, double z1, double z2)
{
  return double( std::atan2((z2 - z1),(x2 - x1)) * (180/M_PI) );
}
