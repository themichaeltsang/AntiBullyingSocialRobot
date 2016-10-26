#include "perceptual_filter/ExtractContextIndependentMeaning.h"
#include "perceptual_filter/FeaturesArr.h"
#include "perceptual_filter/Feature.h"
#include "perceptual_filter/Label.h"



using namespace std;
// clock_t start_time_;
//  ros::NodeHandle nh_;
//  ros::Rate r_(10);


ExtractContextIndependentMeaning::ExtractContextIndependentMeaning()
{
	new_sequence_ = false; 
	count_ = 0;
	joint_names_vel_ = {"head",
	"neck",
	"shoulder_left",
	"shoulder_right",
	"spine_shoulder",
	"elbow_left",
	"elbow_right",
	"wrist_left",
	"wrist_right",
	"hand_left",
	"hand_right",
	"handtip_left",
	"handtip_right",
	"thumb_left",
	"thumb_right",
	"spine_mid",
	"spine_base",
	"hip_left",
	"hip_right",
	"knee_left",
	"knee_right",
	"ankle_left",
	"ankle_right",
	"foot_left",
	"foot_right"};


	joint_names_pos_ = {
	"elbow_left",
	//"elbow_right",
	"wrist_left",
	//"wrist_right",
	//"knee_left",
	//"knee_right",
	//"ankle_left",
	//"ankle_right"
	};


	// joint_names_ = {
	// "elbow_left",
	// "elbow_right",
	// "wrist_left",
	// "wrist_right",
	// "hand_left",
	// "hand_right",
	// "handtip_left",
	// "handtip_right",
	// "hip_left",
	// "hip_right",
	// "knee_left",
	// "knee_right",
	// "ankle_left",
	// "ankle_right",
	// };


 // start_time_ = clock();
  //int rate = 1;
  //r_ = ros::Rate(rate);


}

ExtractContextIndependentMeaning::~ExtractContextIndependentMeaning()
{

}

void ExtractContextIndependentMeaning::publishSampleMessage(ros::Publisher *pub_message)
{
  perceptual_filter::FeaturesArr fArrMsg;
  perceptual_filter::Feature f;
  fArrMsg.features.push_back(f);

  // fArrMsg.features[0].values.push_back(12.3456789);
  // fArrMsg.features[0].values.push_back(3.14159);

  //fArrMsg.timeID = clock() - start_time_;//currently use placeholder for time...

  uint32_t dur = clock() ;//- start_time_;
  //std::cout << "duration " << dur << std::endl;

  //fArrMsg.features[0].values.push_back(dur);
  fArrMsg.features[0].values.push_back(3.14159);

  cout << "poo!" << endl;

  pub_message->publish(fArrMsg);
  // const unsigned int numElements = 5;
  // const unsigned int byteSize = 8;

  // m.layout.dim.push_back(std_msgs::MultiArrayDimension());
  // m.layout.dim[0].size = numElements;
  // m.layout.dim[0].stride = m.layout.dim[0].size * byteSize;
  // m.layout.dim[0].label = "dim0Label";

  // m.layout.dim.push_back(std_msgs::MultiArrayDimension());
  // m.layout.dim[1].size = numElements;
  // m.layout.dim[1].stride = m.layout.dim[1].size;
  // m.layout.dim[1].label = "dim1Label";

  //m.data[0] = 3.14159;
  //m.data.push_back(12.3456789);

  //m.data.resize(numElements);

  // m.data[0].push_back(0.1);
  // m.data.push_back({0.1, 0.2, 0.3});

  //element [3][3]
  //m.data[m.layout.dim[0].stride*0 + m.layout.dim[1].stride*0] = 12.3456789;
  //m.data[m.layout.dim[0].stride*3 + m.layout.dim[1].stride*3] = 3.14159;

  // m.data.push_back(0.1);
  // m.data.push_back(0.2);
  // m.data.push_back(0.3);
  // m.data.push_back(0.4);
  // m.data.push_back(0.5);

  // pub_message->publish(m);
}



void ExtractContextIndependentMeaning::wekaClassifyCallback(const std_msgs::String::ConstPtr& msg)
{    
	boost::mutex::scoped_lock lock(weka_mutex_);
	string callbackMessage = msg->data.c_str();
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	classification_results_queue_.push(callbackMessage);
}

 
std::map<uint32_t, std::map<std::string, float>> ExtractContextIndependentMeaning::getInappropriateBehaviorClassification()
{
	boost::mutex::scoped_lock lock(weka_mutex_);
	map<uint32_t, map<string, float>> recognized_inappropriate_behavior =  map<uint32_t, map<string, float>>();

	if (!classification_results_queue_.empty())
	{
		string classificationResult = classification_results_queue_.front();
		classification_results_queue_.pop();


		uint32_t std_body_idx = 0;
		auto & individuals_behavior_dict = recognized_inappropriate_behavior[std_body_idx];

		individuals_behavior_dict["pointing_taunt"] = 0;
		individuals_behavior_dict["pointing"] = 0;
		individuals_behavior_dict["fist_threat"] = 0;
		individuals_behavior_dict["hitting_prep"] = 0;
		individuals_behavior_dict["punching_prep"] = 0;
		individuals_behavior_dict["kicking_prep"] = 0;
		individuals_behavior_dict["shoving_prep"] = 0;
		individuals_behavior_dict["punching"] = 0;
		individuals_behavior_dict["kicking"] = 0;
		individuals_behavior_dict["shoving"] = 0;
		individuals_behavior_dict["hitting"] = 0;
		individuals_behavior_dict["making_faces_tongue_sticking_out_hands_at_ears"] = 0;
		individuals_behavior_dict["making_faces_tongue_sticking_out"] = 0;

  		int result = std::stoi(classificationResult);

		// switch(result)
		// {
		// 	case 8: 
		// 	{
		// 		individuals_behavior_dict["pointing"] = 1;
		// 		break;
		// 	}
		// 	case 4:
		// 	{
		// 		individuals_behavior_dict["shoving"] = 1;
		// 		break;
		// 	}
		// 	case 2:
		// 	{
		// 		individuals_behavior_dict["kicking"] = 1;
		// 		break;
		// 	}
		// 	case 0:
		// 	{
		// 		individuals_behavior_dict["punching"] = 1;
		// 		break;
		// 	}
		// 	case 6:
		// 	{
		// 		individuals_behavior_dict["hitting"] = 1;
		// 		break;
		// 	}
		// 	case 9:
		// 	{
		// 		individuals_behavior_dict["pointing_taunt"] = 1;
		// 		break;
		// 	}
		// 	case 5:
		// 	{
		// 		individuals_behavior_dict["shoving_prep"] = 1;
		// 		break;
		// 	}
		// 	case 3:
		// 	{
		// 		individuals_behavior_dict["kicking_prep"] = 1;
		// 		break;
		// 	}
		// 	case 1:
		// 	{
		// 		individuals_behavior_dict["punching_prep"] = 1;
		// 		break;
		// 	}
		// 	case 7:
		// 	{
		// 		individuals_behavior_dict["hitting_prep"] = 1;
		// 		break;
		// 	}
		// 	default:
		// 	{
		// 		break;
		// 	}

		// }
	}		

	return recognized_inappropriate_behavior;
}



map<uint32_t, map<string, float>> ExtractContextIndependentMeaning::classifyInappropriateBehavior(map<uint32_t, map<string, map<string, float>>> &absolute_signals, map<uint32_t, map<string, map<string, float>>> &relative_signals, map<string, int32_t> &labels, bool new_sequence, uint64_t sequence_number, std::string sequence_boundary, ros::Publisher *pub_message)
{
	map<uint32_t, map<string, float>> recognized_inappropriate_behavior =  map<uint32_t, map<string, float>>();

	//cout << "e1" << endl;

	if (new_sequence)
	{
		new_sequence_ = true;
	}



	for (auto const& individuals_signals_dict : absolute_signals)
	{
    	uint32_t std_body_idx = individuals_signals_dict.first;
		map<string, map<string, float>> individuals_signals = individuals_signals_dict.second;
		auto & individuals_behavior_dict = recognized_inappropriate_behavior[std_body_idx];

			//std::cout << "count " << count_ << std::endl;
		//cout << "sequence number " << sequence_number << ", boundary " << sequence_boundary << endl;



		if (individuals_signals.count("kinect_body") == 1) // && individuals_signals.count("kinect_face") == 1) //both exist in dict -> meaning both are tracked
		{
			//std::cout << "count " << count_ << std::endl;
			//count_ ++;



			// perceptual_filter::FeaturesArr fArrMsg;
			// perceptual_filter::Feature f;
			// fArrMsg.features.push_back(f);

			// for (std::string const& joint_name : joint_names_vel_)
			// {	
			// 	fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_x"]);
			// 	fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_y"]);
			// 	fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_z"]);
			// }

			// auto & fLabels = fArrMsg.labels;
			// for (auto const & entry : labels )
			// {
			// 	perceptual_filter::Label fLabel;
			// 	fLabel.labelName = entry.first;
			// 	fLabel.labelValue = entry.second;
			// 	fLabels.emplace_back( std::move( fLabel ) );
			// }

			// // cout << "new_sequence: " << new_sequence_ << endl;
			// // exit (EXIT_FAILURE);

			// fArrMsg.new_sequence = new_sequence_;
			// fArrMsg.sequence_boundary = sequence_boundary;
			// fArrMsg.sequence_number = sequence_number;
			// new_sequence_ = false;
			// pub_message->publish(fArrMsg);



			if ( individuals_signals["kinect_body"].count( "wrist_right_z_vel" ) == 1 ) // TODO: add switch/bool for different type of features. I added this to prevent publishing of certain feature arrays, namely ones that did not have velocity features
			{

				perceptual_filter::FeaturesArr fArrMsg;
				perceptual_filter::Feature f;
				fArrMsg.features.push_back(f);



				//fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["lean_y"]);

				//cout << "lean_y: " << fArrMsg.features[0].values[0] << endl;
				// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["proxemics"]);
				// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["hand_state_left"]);
				// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["hand_state_right"]);
				// fArrMsg.features[0].values.push_back(fabs(individuals_signals["kinect_body"]["hip_rotation"]));
				//fArrMsg.features[0].values.push_back(fabs(individuals_signals["kinect_face"]["yaw"]));

				// for (std::string const& joint_name : joint_names_pos_)
				// {	
				// 	fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_x_avg"]);
				// 	fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_y_avg"]);
				// 	fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_z_avg"]);
				// }

				for (std::string const& joint_name : joint_names_vel_)
				{	
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_x"]);
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_y"]);
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_z"]);
				}

				for (std::string const& joint_name : joint_names_vel_)
				{	
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_x_norm"]);
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_y_norm"]);
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_z_norm"]);
				}

				for (std::string const& joint_name : joint_names_vel_)
				{	
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_x_vel"]);
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_y_vel"]);
					fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_z_vel"]);
				}


				auto & fLabels = fArrMsg.labels;
				for (auto const & entry : labels )
				{
					perceptual_filter::Label fLabel;
					fLabel.labelName = entry.first;
					fLabel.labelValue = entry.second;
					fLabels.emplace_back( std::move( fLabel ) );
				}

				// cout << "new_sequence: " << new_sequence_ << endl;
				// exit (EXIT_FAILURE);

				fArrMsg.new_sequence = new_sequence_;
				fArrMsg.sequence_boundary = sequence_boundary;
				fArrMsg.sequence_number = sequence_number;
				new_sequence_ = false;
				pub_message->publish(fArrMsg);
				//

			}








			// std::vector<string> angle_joint_names = {"shoulder", "elbow", "wrist", "hand", "hip", "knee", "ankle"};
			// for (std::string const& joint_name : angle_joint_names)
			// {

			// 	//cout << individuals_signals["kinect_body"][joint_name + "_left_angle"] << endl;
			// 	fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_left_angle"]);
			// 	fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"][joint_name + "_right_angle"]);
			// }

			
			// for 
			// fArrMsg.features[0].values.push_back

			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["right_arm_pointing"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["left_hand_covering_mouth"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["left_arm_pointing"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["right_hand_covering_mouth"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["shove_preparation"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["left_hit_preparation"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["right_hit_preparation"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["left_kick_preparation"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["right_kick_preparation"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["showing_left_fist"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["showing_right_fist"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["left_arm_swing_preparation"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["right_arm_swing_preparation"]);


		}

		if (individuals_signals.count("kinect_body") == 1 && individuals_signals.count("kinect_hdface") == 1) //both exist in dict -> meaning both are tracked
		{

			// perceptual_filter::FeaturesArr fArrMsg;
			// perceptual_filter::Feature f;
			// fArrMsg.features.push_back(f);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_body"]["hands_at_ears"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_hdface"]["tongue_depth"]);
			// fArrMsg.features[0].values.push_back(individuals_signals["kinect_hdface"]["upper_lip_depth"]);
			// fArrMsg.features[0].values.push_back(fabs(individuals_signals["kinect_body"]["hip_rotation"]));
			// fArrMsg.features[0].values.push_back(fabs(individuals_signals["kinect_hdface"]["yaw"]));
			// fArrMsg.features[0].values.push_back(fabs(individuals_signals["kinect_hdface"]["pitch"]));

			// pub_message->publish(fArrMsg);

		}

	}

	return recognized_inappropriate_behavior;

}


map<uint32_t, map<string, float>> ExtractContextIndependentMeaning::classifyInappropriateBehaviorManually(map<uint32_t, map<string, map<string, float>>> &absolute_signals, map<uint32_t, map<string, map<string, float>>> &relative_signals)
{
	map<uint32_t, map<string, float>> recognized_inappropriate_behavior;


	for (auto const& individuals_signals_dict : absolute_signals)
	{
    	uint32_t std_body_idx = individuals_signals_dict.first;
		map<string, map<string, float>> individuals_signals = individuals_signals_dict.second;
		auto & individuals_behavior_dict = recognized_inappropriate_behavior[std_body_idx];

		if (individuals_signals.count("kinect_body") == 1 && individuals_signals.count("kinect_face") == 1) //both exist in dict -> meaning both are tracked
		{



			bool lean_in = individuals_signals["kinect_body"]["lean_y"] > 10; //must check
        	bool one_meter_distance = individuals_signals["kinect_body"]["proxemics"] > 1; 
        	bool one_part_meter_distance = individuals_signals["kinect_body"]["proxemics"] > 1.25; 
        	float proxemics_in_meters =  individuals_signals["kinect_body"]["proxemics"];
        	bool closer_than_two_meters = individuals_signals["kinect_body"]["proxemics"] < 2; 

        	bool two_meter_distance = individuals_signals["kinect_body"]["proxemics"] > 2; 

        	bool left_hand_open = individuals_signals["kinect_body"]["hand_state_left"] == 2; 
        	bool left_hand_closed = individuals_signals["kinect_body"]["hand_state_left"] == 3; 
        	bool left_hand_lasso = individuals_signals["kinect_body"]["hand_state_left"] == 4; 
        	bool right_hand_open = individuals_signals["kinect_body"]["hand_state_right"] == 2; 
        	bool right_hand_closed = individuals_signals["kinect_body"]["hand_state_right"] == 3; 
        	bool right_hand_lasso = individuals_signals["kinect_body"]["hand_state_right"] == 4; 


			bool body_facing_forward = abs(individuals_signals["kinect_body"]["hip_rotation"]) < 30;

			// if (body_facing_forward)
			// {
			// 	std::cout << "body facing forward" << std::endl;
			// 	std::cout << "" << std::endl;
			// }

			bool head_facing_forward = abs(individuals_signals["kinect_face"]["yaw"]) < 30;







			//right_hand_pointing, left hand covering mouth, forward facing hip , head direction forward (yaw -20 - 20, pitch -20 - 20, roll -20 -20)

			// cout << "Right arm point: " << individuals_signals["kinect_body"]["right_arm_pointing"] << endl;
			// cout << "Left arm point: " << individuals_signals["kinect_body"]["left_arm_pointing"] << endl;

			bool right_arm_pointing = individuals_signals["kinect_body"]["right_arm_pointing"] < 1.7;

			// if (right_arm_pointing){
			// 	std::cout << "right_arm_pointing" << std::endl;
			// }

			bool left_hand_covering_mouth = individuals_signals["kinect_body"]["left_hand_covering_mouth"] < 4.0;//3.0;// < 2.5;//1.4;

			// if (left_hand_covering_mouth){
			 //	std::cout << "left_hand_covering_mouth " << individuals_signals["kinect_body"]["left_hand_covering_mouth"] << std::endl;
			// }


			bool left_arm_pointing = individuals_signals["kinect_body"]["left_arm_pointing"] < 2.5;  //1.5
			//needs retraining because score is lower for right point than left


        	cout << "LM: " << (double)individuals_signals["kinect_body"]["left_hand_covering_mouth"] << endl;
        	cout << "RM: " << (double)individuals_signals["kinect_body"]["right_hand_covering_mouth"] << endl;

			bool right_hand_covering_mouth = individuals_signals["kinect_body"]["right_hand_covering_mouth"] < 1.5;
			//std::cout << "right_hand_covering_mouth " << individuals_signals["kinect_body"]["right_hand_covering_mouth"] << std::endl;



        	bool two_hand_shove_prep = individuals_signals["kinect_body"]["shove_preparation"] < 2;
        	//std::cout << "two_hand_shove_prep " << individuals_signals["kinect_body"]["shove_preparation"] << std::endl;
        	bool left_hit_prep = individuals_signals["kinect_body"]["left_hit_preparation"] < 1.3; // 1
        	//std::cout << "left_hit_prep " << individuals_signals["kinect_body"]["left_hit_preparation"] << std::endl;
        	bool right_hit_prep = individuals_signals["kinect_body"]["right_hit_preparation"] < 2; // 1.8
        	//std::cout << "right_hit_preparation " << individuals_signals["kinect_body"]["right_hit_preparation"] << std::endl;

        	bool left_kick_prep = individuals_signals["kinect_body"]["left_kick_preparation"] > 42; //15
			//std::cout << "left_kick_prep " << individuals_signals["kinect_body"]["left_kick_preparation"] << std::endl;
        	bool right_kick_prep = individuals_signals["kinect_body"]["right_kick_preparation"] > 39; //15
			//std::cout << "right_kick_prep " << individuals_signals["kinect_body"]["right_kick_preparation"] << std::endl;

        	bool showing_left_fist = individuals_signals["kinect_body"]["showing_left_fist"] < 0.7;//0.9; 
        	bool showing_right_fist = individuals_signals["kinect_body"]["showing_right_fist"] < 0.7;//0.9; 

        	cout << "LF: " << (double)individuals_signals["kinect_body"]["showing_left_fist"] << endl;
        	cout << "RF: " << (double)individuals_signals["kinect_body"]["showing_right_fist"] << endl;

        	bool left_swing_prep = individuals_signals["kinect_body"]["left_arm_swing_preparation"] < 1.8; 
        	bool right_swing_prep = individuals_signals["kinect_body"]["right_arm_swing_preparation"] < 1.8; 


        	//need bullying preparation values as well 

			bool right_arm_pointing_taunt = ( right_arm_pointing && left_hand_covering_mouth && body_facing_forward && head_facing_forward ); // add proxemics beyond a threshold?
			bool left_arm_pointing_taunt = ( left_arm_pointing && right_hand_covering_mouth && body_facing_forward && head_facing_forward ); // add proxemics beyond a threshold?
			individuals_behavior_dict["pointing_taunt"] = ((right_arm_pointing_taunt || left_arm_pointing_taunt) && one_part_meter_distance ) ? 1 : 0;

			// ADD CHECK FOR JUST POINTING
			individuals_behavior_dict["pointing"] = ( (right_arm_pointing || left_arm_pointing) && head_facing_forward && proxemics_in_meters > 1) ? 1 : 0;


			bool showing_left_fist_threat = ( showing_left_fist && left_hand_closed &&  head_facing_forward ); // add proxemics beyond a threshold?
			bool showing_right_fist_threat = ( showing_right_fist && right_hand_closed &&  head_facing_forward ); // add proxemics beyond a threshold?
			individuals_behavior_dict["fist_threat"] = (showing_left_fist_threat || showing_right_fist_threat) ? 1 : 0;

			bool multimodal_left_hitting_prep = left_swing_prep && left_hand_open;
			bool multimdoal_right_hitting_prep = right_swing_prep && right_hand_open; 
			individuals_behavior_dict["hitting_prep"] = ( (multimodal_left_hitting_prep || multimdoal_right_hitting_prep) && head_facing_forward && proxemics_in_meters < 1.5) ? 1 : 0; // add proxemics beyond a threshold?
			
			bool multimodal_left_punching_prep = (left_swing_prep || left_hit_prep) && left_hand_closed;
			bool multimodal_right_punching_prep = (right_swing_prep || right_hit_prep) && right_hand_closed; 
			individuals_behavior_dict["punching_prep"] = ( (multimodal_left_punching_prep || multimodal_right_punching_prep) && head_facing_forward && proxemics_in_meters < 1.5) ? 1 : 0;
			
			individuals_behavior_dict["kicking_prep"] = ( (left_kick_prep || right_kick_prep) && head_facing_forward && body_facing_forward && proxemics_in_meters > 1.25 && proxemics_in_meters < 2) ? 1 : 0; // add proxemics beyond a threshold?

			bool multimodal_left_shove_prep = left_hit_prep && left_hand_open;
			bool multimodal_right_shove_prep = right_hit_prep && right_hand_open;
			bool multimodal_two_hand_shove_prep = two_hand_shove_prep && left_hand_open && right_hand_open;
			individuals_behavior_dict["shoving_prep"] = ( (multimodal_left_shove_prep || multimodal_right_shove_prep || multimodal_two_hand_shove_prep) && head_facing_forward && proxemics_in_meters < 1.5) ? 1 : 0; // add proxemics beyond a threshold?

			// add more dict entries below...

		}

		if (individuals_signals.count("kinect_body") == 1 && individuals_signals.count("kinect_hdface") == 1) //both exist in dict -> meaning both are tracked
		{ 			
			bool hands_at_ears = individuals_signals["kinect_body"]["hands_at_ears"] > 42;

			float tongue_depth = individuals_signals["kinect_hdface"]["tongue_depth"];
			float upper_lip_depth = individuals_signals["kinect_hdface"]["upper_lip_depth"];

			//bool tongue_sticking_out = individuals_signals["kinect_hdface"]["tongue_sticking_out"] == 1;

			cout << "T: " << (static_cast<double>(tongue_depth) - upper_lip_depth) << endl;

			bool tongue_sticking_out = ((tongue_depth - upper_lip_depth) < -4);//-5Michael -3before


			bool body_facing_forward = abs(individuals_signals["kinect_body"]["hip_rotation"]) < 30;

			bool face_forward = (fabs(individuals_signals["kinect_hdface"]["yaw"]) < 11. ) && fabs(individuals_signals["kinect_hdface"]["pitch"]) < 30.;
			cout << "Yaw: " << individuals_signals["kinect_hdface"]["yaw"] << endl;	
			cout << "Pitch: " << individuals_signals["kinect_hdface"]["pitch"] << endl;

			individuals_behavior_dict["making_faces_tongue_sticking_out_hands_at_ears"] = ( hands_at_ears && tongue_sticking_out && body_facing_forward && face_forward) ? 1 : 0; // add proxemics beyond a threshold?

			//std::cout << "hands_at_ears " << individuals_signals["kinect_body"]["hands_at_ears"] << std::endl;

			individuals_behavior_dict["making_faces_tongue_sticking_out"] = ( tongue_sticking_out && body_facing_forward && face_forward ) ? 1 : 0; // add proxemics beyond a threshold?

			// ADD CHECK FOR JUST TONGUE STICKING OUT IF THE ABOVE IS NOT TRUE

		}
	}

	return recognized_inappropriate_behavior;
}
