#ifndef RECEIVEMSG_H
#define RECEIVEMSG_H

#include <boost/thread/mutex.hpp>
#include "perceptual_filter/AudienceBehavior.h"


class ReceiveMsg
{
	public:
	    ReceiveMsg();
	    ~ReceiveMsg();
	    void messageCallbackContextIndependentBehavior(const perceptual_filter::AudienceBehavior &msg);
	   	std::pair<perceptual_filter::AudienceBehavior,bool> getBehaviorMsg(); //need boolean??

	private:

		bool behavior_msg_empty_ = true; 
		perceptual_filter::AudienceBehavior behavior_msg_;
		boost::mutex behavior_mutex_;
};


#endif /* RECEIVEMSG_H */