#include "decision_making/openHouse2016ActionClient.h"

using namespace std;

Openhouse2016ActionClient::Openhouse2016ActionClient(){

	past_head_position_x_ = 0;
	past_head_position_y_ = 0;
	past_head_position_z_ = 0;
}

Openhouse2016ActionClient::~Openhouse2016ActionClient(){}


void Openhouse2016ActionClient::runActionClient()//int argc, char **argv)
{

}


void waveFeedbackCB(const _WaveFeedbackConstPtr & feedback)
{
	ROS_INFO("Wave Feedback State: %d", feedback->feedbackState);
}


//void Openhouse2016ActionClient::runActionClient(ros::NodeHandle nh)//int argc, char **argv)
void Openhouse2016ActionClient::runActionClientDev()//int argc, char **argv)
{
	ros::NodeHandle nh;

	int rate = 100;
	ros::Rate r(rate);

  	ReceiveMsg* behavior_msg_wrapper = new ReceiveMsg();
  	ros::Subscriber sub_msg = nh.subscribe("/perceptual_filter/behavior", 1000, &ReceiveMsg::messageCallbackContextIndependentBehavior, behavior_msg_wrapper);//&Openhouse2016ActionClient::messageCallbackContextIndependentBehavior, behavior_msg_wrapper);


	actionlib::SimpleActionClient<_WaveAction> ac("WaveActionControl", true);
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("wave action server started");
	_WaveGoal waveGoal; //change to something else
	waveGoal.arm_id = waveGoal.ARM_ID_RIGHT; //right arm
	waveGoal.gesture_duration = ros::Duration(5);

	actionlib::SimpleActionClient<_LeftArmAction> lac("LeftArmActionControl", true);
	ROS_INFO("Waiting for left arm action server to start.");
	lac.waitForServer();
	ROS_INFO("left arm action server started");
	_LeftArmGoal leftArmGoal; //change to something else
	leftArmGoal.armMovementType = leftArmGoal.POINT_FORWARD; //right arm
	leftArmGoal.gesture_duration = ros::Duration(5);

	actionlib::SimpleActionClient<_RightArmAction> rac("RightArmActionControl", true);
	ROS_INFO("Waiting for right arm action server to start.");
	rac.waitForServer();
	ROS_INFO("right arm action server started");
	_RightArmGoal rightArmGoal; //change to something else
	rightArmGoal.armMovementType = rightArmGoal.WAVE_ARM; //right arm
	rightArmGoal.gesture_duration = ros::Duration(15);

	actionlib::SimpleActionClient<_MouthAndSpeechAction> msac("MouthAndSpeechActionControl", true);
	ROS_INFO("Waiting for mouth and speech action server to start.");
	msac.waitForServer();
	ROS_INFO("mouth speech action server started");
	_MouthAndSpeechGoal mouthAndSpeechGoal; //change to something else
	mouthAndSpeechGoal.actionType = mouthAndSpeechGoal.SYNCHRONIZED_SPEECH; 
	mouthAndSpeechGoal.speechType = mouthAndSpeechGoal.PLAY_SOUND_FILE;
	//mouthAndSpeechGoal.speech_filename = "";
	//mouthAndSpeechGoal.mouth_expression_duration = ros::Duration(10); //?

	actionlib::SimpleActionClient<_HeadAction> hac("HeadActionControl", true);
	ROS_INFO("Waiting for head action server to start.");
	hac.waitForServer();
	ROS_INFO("head action server started");
	_HeadGoal headGoal; //change to something else
	//headGoal.speechType = headGoal.PLAY_SOUND_FILE;

	actionlib::SimpleActionClient<_EyebrowsAction> eac("EyebrowsActionControl", true);
	ROS_INFO("Waiting for eyebrows action server to start.");
	eac.waitForServer();
	ROS_INFO("eyebrows action server started");
	_EyebrowsGoal eyebrowsGoal; //change to something else
	eyebrowsGoal.browMovementType = eyebrowsGoal.BROW_FURROW; 
	eyebrowsGoal.degree_of_movement = eyebrowsGoal.DEGREE_LOW;
	eyebrowsGoal.gesture_duration = ros::Duration(2); //?


	bool goalSentLeft = false;
	bool goalSentRight = false;
	bool waveGoalSent = false;
	bool goalSentMouthAndSpeech = false;
	bool goalSentHead = false;
	bool goalSentEyebrows = false;

	map<string, int> countOccurances; 

	while (ros::ok())
	{
		// grabs audience features of interest
		std::pair<perceptual_filter::AudienceBehavior, bool> behavior_msg_pair = behavior_msg_wrapper->getBehaviorMsg();
		perceptual_filter::AudienceBehavior behavior_msg = behavior_msg_pair.first;
		bool behavior_msg_valid = behavior_msg_pair.second;

		if (behavior_msg_valid)
		{		
			uint8_t pointing_taunt = 0;
			uint8_t pointing = 0;
			uint8_t fist_threat = 0;
			uint8_t hitting_prep = 0;
			uint8_t punching_prep = 0;
			uint8_t kicking_prep = 0;
			uint8_t shoving_prep = 0;
			uint8_t making_faces_tongue_sticking_out_hands_at_ears = 0; //tongue_sticking_out + hands_at_ears
			uint8_t making_faces_tongue_sticking_out = 0;


			float head_position_x = 0;
			float head_position_y = 0;
			float head_position_z = 0;


	 		const auto & audience_behaviors = behavior_msg.audience;
			//std::cout << "audience size!: " << audience_behaviors.size() << std::endl;

			for (size_t person_idx = 0; person_idx < audience_behaviors.size(); ++person_idx)
			{
				const auto & individual_behavior = audience_behaviors[person_idx];
				pointing_taunt = individual_behavior.inappropriate_behavior.pointing_taunt;
				pointing = individual_behavior.inappropriate_behavior.pointing;
				fist_threat = individual_behavior.inappropriate_behavior.fist_threat;
				hitting_prep = individual_behavior.inappropriate_behavior.hitting_prep;
				punching_prep = individual_behavior.inappropriate_behavior.punching_prep;
				kicking_prep = individual_behavior.inappropriate_behavior.kicking_prep;
				shoving_prep = individual_behavior.inappropriate_behavior.shoving_prep;
				making_faces_tongue_sticking_out_hands_at_ears = individual_behavior.inappropriate_behavior.making_faces_tongue_sticking_out_hands_at_ears;
				making_faces_tongue_sticking_out = individual_behavior.inappropriate_behavior.making_faces_tongue_sticking_out;

				if (pointing_taunt) countOccurances["pointing_taunt"]++;
				else countOccurances["pointing_taunt"] = 0;
				if (pointing) countOccurances["pointing"]++;
				else countOccurances["pointing"] = 0;
				if (fist_threat) countOccurances["fist_threat"]++;
				else countOccurances["fist_threat"] = 0;
				if (hitting_prep) countOccurances["hitting_prep"]++;
				else countOccurances["hitting_prep"] = 0;
				if (punching_prep) countOccurances["punching_prep"]++;
				else countOccurances["punching_prep"] = 0;
				if (kicking_prep) countOccurances["kicking_prep"]++;
				else countOccurances["kicking_prep"] = 0;
				if (shoving_prep) countOccurances["shoving_prep"]++;
				else countOccurances["shoving_prep"] = 0;
				if (making_faces_tongue_sticking_out_hands_at_ears) countOccurances["making_faces_tongue_sticking_out_hands_at_ears"]++;
				else countOccurances["making_faces_tongue_sticking_out_hands_at_ears"] = 0;
				if (making_faces_tongue_sticking_out) countOccurances["making_faces_tongue_sticking_out"]++;
				else countOccurances["making_faces_tongue_sticking_out"] = 0;

				// for (auto it = countOccurances.begin(); it != countOccurances.end(); ++it)
				// 	cout << it->first << ": " << it->second << endl;

				head_position_x = individual_behavior.head_position.x;
				head_position_y = individual_behavior.head_position.y;
				head_position_z = individual_behavior.head_position.z;

				//must assume only one person tracked for this demo 
			}

			if (audience_behaviors.size() == 1) //there is only one person tracked
			{

				// if (!goalSentLeft)
				// {
				// 	lac.sendGoal(leftArmGoal);
				// 	goalSentLeft = true;
				// }
				// else if (lac.getState().toString() == "SUCCEEDED")
				// {
				// 	ROS_INFO("Left Arm Action finished: %s", ac.getState().toString().c_str());
				// 	goalSentLeft = false;
				// }

				// if (!goalSentRight)
				// {
				// 	rac.sendGoal(rightArmGoal);
				// 	goalSentRight = true;
				// }
				// else if (rac.getState().toString() == "SUCCEEDED")
				// {
				// 	ROS_INFO("Right Arm Action finished: %s", ac.getState().toString().c_str());
				// 	goalSentRight = false;
				// }

				// if (!waveGoalSent)
				// {
				// 	ac.sendGoal(waveGoal);
				// 	waveGoalSent = true;
				// }
				// else if (ac.getState().toString() == "SUCCEEDED")
				// {
				// 	ROS_INFO("Wave Action finished: %s", ac.getState().toString().c_str());
				// 	waveGoalSent = false;
				// }

				// if (ac.waitForResult(ros::Duration(30.0)))
				// 	ROS_INFO("Action finished: %s", ac.getState().toString().c_str());
				// else
				// 	ROS_INFO("Action did not finish before the time out.");


				// WANT HEAD

				if (!goalSentHead)
				{

					// if( (past_head_position_x_ == 0 && past_head_position_y_ == 0 && past_head_position_z_ == 0) && (head_position_x != 0 || head_position_y != 0 || head_position_z != 0) )
					// {
					// 	headGoal.gesture_duration = ros::Duration(2); 
					// }
					// else if ( (head_position_x == 0 && head_position_y == 0 && head_position_z == 0) && (past_head_position_x_ != 0 || past_head_position_y_ != 0 || past_head_position_z_ != 0) )
					// {
					// 	headGoal.gesture_duration = ros::Duration(2); 
					// }
					// else {
					// 	headGoal.gesture_duration = ros::Duration(0.3); 
					// }

					headGoal.gesture_duration = ros::Duration(0.05); 
					headGoal.headMovementType = headGoal.FOLLOW_YOU; 
					headGoal.your_head_position.x = head_position_x;
					headGoal.your_head_position.y = head_position_y;
					headGoal.your_head_position.z = head_position_z;

					//std::cout << "head_position: " << head_position_x  << ", " << head_position_y << ", " << head_position_z << std::endl;

					hac.sendGoal(headGoal);
					goalSentHead = true;

					//goalSentHead = false;
				}
				else if (hac.getState().toString() == "SUCCEEDED")
				{
					//ROS_INFO("Head Action finished: %s", hac.getState().toString().c_str());
					goalSentHead = false;
				}



				// WANT EYEBROWS

				// if (!goalSentEyebrows)
				// {
				// 	eac.sendGoal(eyebrowsGoal);
				// 	goalSentEyebrows = true;
				// }
				// else if (eac.getState().toString() == "SUCCEEDED")
				// {
				// 	ROS_INFO("Eyebrows Action finished: %s", eac.getState().toString().c_str());
				// 	goalSentEyebrows = false;
				// }



				// takes action if certain behaviors seen
				if (countOccurances["pointing_taunt"] >= 2)
				{
					countOccurances["pointing_taunt"] = 0;

					// // WANT MOUTH AND SPEECH

					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_pointing_taunt.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}
					// else if (msac.getState().toString() == "SUCCEEDED")
					// {
					// 	ROS_INFO("Mouth Speech Action finished: %s", msac.getState().toString().c_str());
					// 	goalSentMouthAndSpeech = false;
					// }



				}
				else if (countOccurances["fist_threat"] >= 10)
				{
					countOccurances["fist_threat"] = 0;
					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_fist_threat.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}
				}

				else if (countOccurances["hitting_prep"] >= 5)
				{
					countOccurances["hitting_prep"] = 0;
					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_hitting_prep.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}
				}


				else if (countOccurances["punching_prep"] >= 5)
				{
					countOccurances["punching_prep"] = 0;
					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_punching_prep.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}

				}

				else if (countOccurances["kicking_prep"] >= 15)
				{
					countOccurances["kicking_prep"] = 0;
					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_kicking_prep.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}

				}

				else if (countOccurances["shoving_prep"] >= 10)
				{
					countOccurances["shoving_prep"] = 0;
					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_shoving_prep.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}

				}

				else if (countOccurances["making_faces_tongue_sticking_out_hands_at_ears"] >= 3) //4
				{
					countOccurances["making_faces_tongue_sticking_out_hands_at_ears"] = 0;
					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_sticking_tongue_out_and_hands_at_ears.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}

				}

				else if (countOccurances["making_faces_tongue_sticking_out"] >= 6) //4
				{
					countOccurances["making_faces_tongue_sticking_out"] = 0;
					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_tongue_sticking_out.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}
				}
				else if (countOccurances["pointing"] >= 15)
				{
					countOccurances["pointing"] = 0;

					if (!goalSentMouthAndSpeech)
					{
						mouthAndSpeechGoal.speech_filename = "response_to_pointing.ogg";
						msac.sendGoal(mouthAndSpeechGoal);
						eac.sendGoal(eyebrowsGoal);
						goalSentMouthAndSpeech = true;
					}


				}


				if (goalSentMouthAndSpeech && msac.getState().toString() == "SUCCEEDED")
				{
					ROS_INFO("Mouth Speech Action finished: %s", msac.getState().toString().c_str());
					goalSentMouthAndSpeech = false;
					for (auto it = countOccurances.begin(); it != countOccurances.end(); ++it)
						it->second = 0;
				}

			}
		}
		// else
		// {

		// 	// reset head position
		// 	// feedback would be nice about the current head position, so I dont need to set it again...

		// 	if (!goalSentHead)
		// 	{
		// 		headGoal.gesture_duration = ros::Duration(0.05); 
		// 		headGoal.headMovementType = headGoal.SET_RESTING_POSITION; 
		// 		hac.sendGoal(headGoal);
		// 		goalSentHead = true;

		// 		//goalSentHead = false;
		// 	}
		// 	else if (hac.getState().toString() == "SUCCEEDED")
		// 	{
		// 		//ROS_INFO("Head Action finished: %s", hac.getState().toString().c_str());
		// 		goalSentHead = false;
		// 	}


		// }

		ros::spinOnce();
		r.sleep();

	}
}

