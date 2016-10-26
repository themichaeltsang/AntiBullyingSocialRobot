#include "decision_making/receiveMsg.h"


ReceiveMsg::ReceiveMsg()
{

}

ReceiveMsg::~ReceiveMsg()
{

}

void ReceiveMsg::messageCallbackContextIndependentBehavior(const perceptual_filter::AudienceBehavior &msg)
{
    boost::mutex::scoped_lock lock(behavior_mutex_);
    //behavior_msg_deque_.push_back(msg);
	behavior_msg_ = msg;
	behavior_msg_empty_ = false;

}

std::pair<perceptual_filter::AudienceBehavior, bool> ReceiveMsg::getBehaviorMsg()
{
	boost::mutex::scoped_lock lock(behavior_mutex_);

	if (!behavior_msg_empty_)
	{
		behavior_msg_empty_ = true;
		return std::make_pair(behavior_msg_,true);
	}
	else
	{
		return std::make_pair(behavior_msg_,false);
	}
}


