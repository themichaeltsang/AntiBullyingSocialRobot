#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "decision_making/WaveAction.h"
#include "decision_making/LeftArmAction.h"
#include "decision_making/RightArmAction.h"
#include "decision_making/MouthAndSpeechAction.h"
#include "decision_making/EyebrowsAction.h"
#include "decision_making/HeadAction.h"
#include "perceptual_filter/AudienceBehavior.h"
#include "decision_making/receiveMsg.h"
#include <boost/thread/mutex.hpp>

typedef decision_making::WaveAction _WaveAction;
typedef decision_making::WaveGoal _WaveGoal;
typedef decision_making::WaveFeedbackConstPtr _WaveFeedbackConstPtr;

typedef decision_making::LeftArmAction _LeftArmAction;
typedef decision_making::LeftArmGoal _LeftArmGoal;
typedef decision_making::LeftArmFeedbackConstPtr _LeftArmFeedbackConstPtr;

typedef decision_making::RightArmAction _RightArmAction;
typedef decision_making::RightArmGoal _RightArmGoal;
typedef decision_making::RightArmFeedbackConstPtr _RightArmFeedbackConstPtr;

typedef decision_making::MouthAndSpeechAction _MouthAndSpeechAction;
typedef decision_making::MouthAndSpeechGoal _MouthAndSpeechGoal;
typedef decision_making::MouthAndSpeechFeedbackConstPtr _MouthAndSpeechFeedbackConstPtr;

typedef decision_making::HeadAction _HeadAction;
typedef decision_making::HeadGoal _HeadGoal;
typedef decision_making::HeadFeedbackConstPtr _HeadFeedbackConstPtr;

typedef decision_making::EyebrowsAction _EyebrowsAction;
typedef decision_making::EyebrowsGoal _EyebrowsGoal;
typedef decision_making::EyebrowsFeedbackConstPtr _EyebrowsFeedbackConstPtr;


class AntiBullyStudy2016ActionClient
{
	public:
	    AntiBullyStudy2016ActionClient();
	    ~AntiBullyStudy2016ActionClient();

	   // void runActionClient(ros::NodeHandle nh);
	    void runActionClient();
	    void runActionClientDev();


	private:

		//void AntiBullyStudy2016ActionClient::contextIndependentBehaviorCallback(const perceptual_filter::AudienceBehavior &msg);


		float past_head_position_x_;
		float past_head_position_y_;
		float past_head_position_z_;


		//perceptual_filter::AudienceBehavior audience_behavior_msg_;
		//actionlib::SimpleActionClient<_WaveAction> ac_;
		// _WaveGoal waveGoal_;
};
