#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <decision_making/WaveAction.h>
#include <decision_making/LeftArmAction.h>
#include <decision_making/RightArmAction.h>
#include <decision_making/MouthAndSpeechAction.h>
#include <decision_making/HeadAction.h>
#include <decision_making/EyebrowsAction.h>

#include <bandit_driver/JointArray.h>
#include <libbandit/joint_name.h>
#include <SFML/Audio.hpp>

typedef decision_making::WaveAction _WaveAction;
typedef decision_making::WaveResult _WaveResult;
typedef decision_making::WaveFeedback _WaveFeedback;
typedef decision_making::WaveGoalConstPtr _WaveGoalConstPtr;

typedef decision_making::LeftArmAction _LeftArmAction;
typedef decision_making::LeftArmResult _LeftArmResult;
typedef decision_making::LeftArmFeedback _LeftArmFeedback;
typedef decision_making::LeftArmGoalConstPtr _LeftArmGoalConstPtr;

typedef decision_making::RightArmAction _RightArmAction;
typedef decision_making::RightArmResult _RightArmResult;
typedef decision_making::RightArmFeedback _RightArmFeedback;
typedef decision_making::RightArmGoalConstPtr _RightArmGoalConstPtr;

typedef decision_making::MouthAndSpeechAction _MouthAndSpeechAction;
typedef decision_making::MouthAndSpeechResult _MouthAndSpeechResult;
typedef decision_making::MouthAndSpeechFeedback _MouthAndSpeechFeedback;
typedef decision_making::MouthAndSpeechGoalConstPtr _MouthAndSpeechGoalConstPtr;

typedef decision_making::HeadAction _HeadAction;
typedef decision_making::HeadResult _HeadResult;
typedef decision_making::HeadFeedback _HeadFeedback;
typedef decision_making::HeadGoalConstPtr _HeadGoalConstPtr;

typedef decision_making::EyebrowsAction _EyebrowsAction;
typedef decision_making::EyebrowsResult _EyebrowsResult;
typedef decision_making::EyebrowsFeedback _EyebrowsFeedback;
typedef decision_making::EyebrowsGoalConstPtr _EyebrowsGoalConstPtr;


typedef bandit_driver::JointArray _JointArray;

static double deg_to_rad(const double & deg)
{
	return deg * M_PI / 180.;
}

static double rad_to_deg(const double & rad)
{
	return rad * 180. / M_PI;
}


class BanditActionControl 
{
	protected:
		ros::NodeHandle nh_;
		std::string action_name_;
		ros::Publisher & joints_publisher;
		_JointArray & desired_joint_pos_arr;

		actionlib::SimpleActionServer<_EyebrowsAction> eas_;
		actionlib::SimpleActionServer<_HeadAction> has_;
		actionlib::SimpleActionServer<_MouthAndSpeechAction> msas_;
		actionlib::SimpleActionServer<_LeftArmAction> las_;
		actionlib::SimpleActionServer<_RightArmAction> ras_;

		actionlib::SimpleActionServer<_WaveAction> as_;

		void publish_joints()
		{
			joints_publisher.publish(desired_joint_pos_arr);
		}

		void publishSingleJoint(const int & id, double angle)
		{
			desired_joint_pos_arr.joints.clear();
			bandit_driver::Joint desired_joint_pos;	
			desired_joint_pos_arr.joints.emplace_back( std::move( desired_joint_pos ) );
			desired_joint_pos_arr.joints[0].id = id;
			desired_joint_pos_arr.joints[0].angle = angle;
			publish_joints();
		}

		void publishJointMap(std::map<std::string, double> & joints)
		{
			desired_joint_pos_arr.joints.clear();

			const std::map<std::string, unsigned short> & jointMap = bandit::getJointIdsMap();

			size_t counter = 0;
			for (auto it = joints.begin(); it != joints.end(); ++it)
			{
				bandit_driver::Joint desired_joint_pos;	
				desired_joint_pos_arr.joints.emplace_back( std::move( desired_joint_pos ) );
				desired_joint_pos_arr.joints[counter].id = jointMap.find(it->first)->second;
				desired_joint_pos_arr.joints[counter].angle = it->second;
				counter ++;
				//publishSingleJoint(jointMap.find(it->first)->second, it->second);
			}
			publish_joints();
		}

		bool wave(const _WaveGoalConstPtr & goal);
		bool leftArmPerformAction(const _LeftArmGoalConstPtr & goal);
		bool rightArmPerformAction(const _RightArmGoalConstPtr & goal);
		bool mouthAndSpeechPerformAction(const _MouthAndSpeechGoalConstPtr & goal);
		bool headPerformAction(const _HeadGoalConstPtr & goal);
		bool eyebrowsPerformAction(const _EyebrowsGoalConstPtr & goal);

		void resetRobot();

		//bool reset = false;

	public:
		BanditActionControl(std::string name, ros::Publisher & jPub, _JointArray & jArr):
			as_(nh_, "WaveActionControl", boost::bind(&BanditActionControl::waveExecuteCB, this, _1), false),
			las_(nh_, "LeftArmActionControl", boost::bind(&BanditActionControl::leftArmExecuteCB, this, _1), false),
			ras_(nh_, "RightArmActionControl", boost::bind(&BanditActionControl::rightArmExecuteCB, this, _1), false),
			msas_(nh_, "MouthAndSpeechActionControl", boost::bind(&BanditActionControl::mouthAndSpeechExecuteCB, this, _1), false),
			has_(nh_, "HeadActionControl", boost::bind(&BanditActionControl::headExecuteCB, this, _1), false),
			eas_(nh_, "EyebrowsActionControl", boost::bind(&BanditActionControl::eyebrowsExecuteCB, this, _1), false),
			action_name_(name), 
			joints_publisher(jPub), 
			desired_joint_pos_arr(jArr)
		{
			ROS_INFO("ActionControl initialized with name: %s", action_name_.c_str());
			as_.start();
			las_.start();
			ras_.start();
			msas_.start();
			has_.start();
			eas_.start();	
		}

		~BanditActionControl() {}


		void eyebrowsExecuteCB(const _EyebrowsGoalConstPtr & goal)
		{
			ROS_INFO("SERVER RECEIVED EYEBROWS GOAL");
			_EyebrowsResult result_;
			if (eyebrowsPerformAction(goal))
				eas_.setSucceeded(result_);
		}

		void headExecuteCB(const _HeadGoalConstPtr & goal)
		{
			//ROS_INFO("SERVER RECEIVED HEAD GOAL");
			_HeadResult result_;
			if (headPerformAction(goal))
				has_.setSucceeded(result_);
		}

		void mouthAndSpeechExecuteCB(const _MouthAndSpeechGoalConstPtr & goal)
		{
			ROS_INFO("SERVER RECIEVED MOUTH SPEECH GOAL");
			_MouthAndSpeechResult result_;
			if (mouthAndSpeechPerformAction(goal))
				msas_.setSucceeded(result_);
		}

		void leftArmExecuteCB(const _LeftArmGoalConstPtr & goal)
		{
			ROS_INFO("SERVER RECIEVED LEFT ARM GOAL");
			_LeftArmResult result_;
			if (leftArmPerformAction(goal))
				las_.setSucceeded(result_);
		}

		void rightArmExecuteCB(const _RightArmGoalConstPtr & goal)
		{
			ROS_INFO("SERVER RECIEVED RIGHT ARM GOAL");
			_RightArmResult result_;
			if (rightArmPerformAction(goal))
				ras_.setSucceeded(result_);
		}

		void waveExecuteCB(const _WaveGoalConstPtr & goal)
		{
			ROS_INFO("SERVER RECIEVED WAVE GOAL");
			_WaveResult result_;
			if (wave(goal))
				as_.setSucceeded(result_);
		}
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "BanditActionControlServer");

	ros::NodeHandle n;
	ros::Publisher joints_publisher = n.advertise<bandit_driver::JointArray>("/bandit_driver/bandit_joint_array_cmd",10); //joint_ind

	//ros::Publisher joints_publisher = n.advertise<_JointArray>("/bandit_driver_node/bandit_joint_array_cmd", 100); //joint_ind
	_JointArray * joints_arr = new _JointArray();

	BanditActionControl banditControl(ros::this_node::getName(), joints_publisher, *joints_arr);
	ros::spin();

	return 0;
}

bool BanditActionControl::eyebrowsPerformAction(const _EyebrowsGoalConstPtr & goal)
{
	ROS_INFO("PERFORMING EYEBROWS ACTION");

	const std::map<std::string, unsigned short> & jointMap = bandit::getJointIdsMap();
	
	ros::Duration(0.5).sleep(); //delay for speech

	if ( goal->browMovementType == goal->BROW_FURROW)
	{
		if ( goal->degree_of_movement == goal->DEGREE_LOW )
		{
			publishSingleJoint(jointMap.find("eyebrows_joint")->second, deg_to_rad( 10 ));
		}
		else if ( goal->degree_of_movement == goal->DEGREE_MEDIUM )
		{
			publishSingleJoint(jointMap.find("eyebrows_joint")->second, deg_to_rad( 5 ));
		}
		else if ( goal->degree_of_movement == goal->DEGREE_HIGH )
		{
			publishSingleJoint(jointMap.find("eyebrows_joint")->second, deg_to_rad( 0 ));
		}

		(goal->gesture_duration).sleep();
		//ros::Duration(goal->gesture_duration.toSec()).sleep();
		publishSingleJoint(jointMap.find("eyebrows_joint")->second, deg_to_rad( 18 ));
	}

	//ros::Duration(5).sleep();
	ROS_INFO("DONE WITH EYEBROWS ACTION");
	return true;
}

bool BanditActionControl::headPerformAction(const _HeadGoalConstPtr & goal) // need to test FOLLOW_YOU!!!
{
	//ROS_INFO("PERFORMING HEAD ACTION");

	const std::map<std::string, unsigned short> & jointMap = bandit::getJointIdsMap();

	// if ( goal->headMovementType == goal->SET_RESTING_POSITION )
	// {
	// 	std::map<std::string, double> defaultPos = 
	// 	{
	// 		{"head_tilt_joint", deg_to_rad(0)},
	// 		{"head_pan_joint", deg_to_rad(0)}
	// 	};

	// 	publishJointMap(defaultPos);
	// 	(goal->gesture_duration).sleep();

	// }
	if ( goal->headMovementType == goal->FOLLOW_YOU )
	{

		if (goal->your_head_position.z != 0) {

			float offset_tilt = deg_to_rad(5);
			float offset_pan = deg_to_rad(8);

			float tilt_angle = (-1*atan(goal->your_head_position.y / goal->your_head_position.z)) + offset_tilt; //angles expressed in radians
			float pan_angle = (-1*atan(goal->your_head_position.x / goal->your_head_position.z)) + offset_pan; //angles expressed in radians

			if (tilt_angle < deg_to_rad(-5))
			{
				tilt_angle = deg_to_rad(-5);
			}

			// std::cout << "tilt angle: " << tilt_angle << ", pan_angle: " << pan_angle << std::endl; 

			// std::map<std::string, double> defaultPos = 
			// {
			// 	{"head_tilt_joint", tilt_angle},
			// 	{"head_pan_joint", pan_angle}
			// };

			// publishJointMap(defaultPos);

			publishSingleJoint(jointMap.find("head_pan_joint")->second, pan_angle );
			//publishSingleJoint(jointMap.find("head_tilt_joint")->second, tilt_angle );

			//(goal->gesture_duration).sleep();
			//resetRobot();
		}
	}

	// if (!reset)
	// {
	// 	resetRobot();
	// 	reset = true;
	// }

	//ros::Duration(5).sleep();
	//ROS_INFO("DONE WITH HEAD ACTION");
	return true;
}

bool BanditActionControl::mouthAndSpeechPerformAction(const _MouthAndSpeechGoalConstPtr & goal)
{
	ROS_INFO("PERFORMING MOUTH AND SPEECH ACTION");

	
	const std::map<std::string, unsigned short> & jointMap = bandit::getJointIdsMap();

	// from action file selector

	// if mouth movement, run common mouth movements - like :), :(, :O

	//if synchronized speech, open mouth, play soundfile, and close mouth

	if ( goal->actionType == goal->SYNCHRONIZED_SPEECH )
	{
		if ( goal->speechType == goal->PLAY_SOUND_FILE )
		{
			ros::Duration(0.1).sleep(); //delay for speech
			// open mouth

			sf::Music speech;
			if (!speech.openFromFile(static_cast<std::string>(goal->speech_filename))) // the filename should be sent over from actionmsg
				return -1; // error
			speech.play();

			ros::Duration(0.5).sleep(); //delay for speech

			publishSingleJoint(jointMap.find("mouth_top_joint")->second, deg_to_rad( 35 ));
			publishSingleJoint(jointMap.find("mouth_bottom_joint")->second, deg_to_rad( 45 ));
			//ros::Duration(0.25).sleep();

			while (true)
			{
				if (speech.getStatus() == sf::SoundSource::Status::Stopped){
					break;
				}
				//ros::Duration(0.5).sleep();
			}
			// close mouth

			resetRobot();

			//publishSingleJoint(jointMap.find("mouth_top_joint")->second, deg_to_rad( -90 ));
			//publishSingleJoint(jointMap.find("mouth_bottom_joint")->second, deg_to_rad( -90 ));

			ros::Duration(0.25).sleep();
		}
	}

	ROS_INFO("DONE WITH MOUTH AND SPEECH ACTION");
	return true;
}

bool BanditActionControl::leftArmPerformAction(const _LeftArmGoalConstPtr & goal)
{
	ROS_INFO("PERFORMING LEFT ARM ACTION");
	//resetRobot();
	ros::Duration(5).sleep();
	ROS_INFO("DONE WITH LEFT ARM ACTION");
	return true;
}

bool BanditActionControl::rightArmPerformAction(const _RightArmGoalConstPtr & goal)
{
	ROS_INFO("PERFORMING RIGHT ARM ACTION");
	//resetRobot();
	ros::Duration(1).sleep();
	ROS_INFO("DONE WITH RIGHT ARM ACTION");
	return true;
}

bool BanditActionControl::wave(const _WaveGoalConstPtr & goal) //TODO use goal info for waving
{
	double dur = 0.;
	if (goal->gesture_duration.toSec() <= 0.5)
		return false;


	_WaveFeedback feedback_;
	//publish some feedback
	feedback_.feedbackState = feedback_.MOVING_TO_TARGET;
	as_.publishFeedback(feedback_);


	std::string pref = "";
	if (goal->arm_id == goal->ARM_ID_LEFT)
		pref = "left";
	else if (goal->arm_id == goal->ARM_ID_RIGHT)
		pref = "right";

	const std::map<std::string, unsigned short> & jointMap = bandit::getJointIdsMap();

	std::map<std::string, double> pos = 
	{
		{pref + "_torso_shoulder_mounting_joint", deg_to_rad(10)},
		{pref + "_shoulder_mounting_shoulder_joint", deg_to_rad(-5)},
		{pref + "_bicep_forearm_joint", deg_to_rad(45)},
		{pref + "_shoulder_bicep_joint", deg_to_rad(0)},
		{pref + "_forearm_wrist_joint", deg_to_rad(0)},
		{pref + "_wrist_hand_joint", deg_to_rad(45)},
		{pref + "_hand_thumb_joint", deg_to_rad(0)}
	};

	for (auto it = pos.begin(); it != pos.end(); ++it)
	{
		if (pref == "left")
			it->second = -it->second;
		publishSingleJoint(jointMap.find(it->first)->second, it->second);
	}
	
	ros::Duration(0.5).sleep();
	dur += 0.5;

	while (dur < goal->gesture_duration.toSec())
	{
		if (as_.isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("Preempted");
			// set the action state to preempted
			as_.setPreempted();
			return false;
		}

		double wave_deg = 0;
		for (int i_wave = 0; i_wave < 3; i_wave++){
			ROS_INFO("JOINT 12 Wave");
			if (i_wave % 2 == 0){
				wave_deg = -10;
			}
			else{
				wave_deg = 70;
			}

			publishSingleJoint(jointMap.find(pref + "_bicep_forearm_joint")->second, deg_to_rad( wave_deg ));
			ros::Rate(2).sleep();
			dur += (double)1.0/2.0; //rate const
		}
		ROS_INFO("Dur %f", dur);
	}
	resetRobot();
	feedback_.feedbackState = feedback_.AT_TARGET;
	as_.publishFeedback(feedback_);
	return true;
}

void BanditActionControl::resetRobot()
{
	std::map<std::string, double> defaultPos = 
	{
		//{"head_tilt_joint", deg_to_rad(0)},
		//{"head_pan_joint", deg_to_rad(0)},
		{"left_torso_shoulder_mounting_joint", deg_to_rad(-65)},
		{"left_shoulder_mounting_shoulder_joint", deg_to_rad(-65)},
		{"left_shoulder_bicep_joint", deg_to_rad(-45)},
		{"left_bicep_forearm_joint", deg_to_rad(45)},
		{"left_forearm_wrist_joint", deg_to_rad(0)},
		{"left_wrist_hand_joint", deg_to_rad(0)},
		{"left_hand_thumb_joint", deg_to_rad(0)},
		{"right_torso_shoulder_mounting_joint", deg_to_rad(65)},
		{"right_shoulder_mounting_shoulder_joint", deg_to_rad(65)},
		{"right_shoulder_bicep_joint", deg_to_rad(45)},
		{"right_bicep_forearm_joint", deg_to_rad(-45)},
		{"right_forearm_wrist_joint", deg_to_rad(0)},
		{"right_wrist_hand_joint", deg_to_rad(45)},
		{"right_hand_thumb_joint", deg_to_rad(0)},
		{"eyebrows_joint", deg_to_rad(18)},
		{"mouth_top_joint", deg_to_rad(0)},
		{"mouth_bottom_joint", deg_to_rad(10)}
	};

	publishJointMap(defaultPos);
}

//old waveExecuteCB
// void waveExecuteCB(const _WaveGoalConstPtr & goal)
// {
// 	ROS_INFO("SERVER RECIEVED WAVE GOAL");

// 	_WaveResult result_;
// 	if (wave(goal))
// 		as_.setSucceeded(result_);

// 	// bool success = false;

// 	// //_WaveFeedback feedback_;
// 	// //do action!!
// 	// wave(goal);
// 	// //publish some feedback
// 	// //as_.publishFeedback(feedback_);

// 	// success = true;

// 	// if (success)
// 	// {
// 	// 	_WaveResult result_;
// 	// 	//result_.someResultStateVar = feedback_.someResultStateVar;
// 	// 	ROS_INFO("Succeeded!");
// 	// 	as_.setSucceeded(result_);
// 	// }
// }