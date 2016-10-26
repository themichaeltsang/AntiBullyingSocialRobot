#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <decision_making/WaveAction.h>

typedef decision_making::WaveAction _WaveAction;
typedef decision_making::WaveGoal _WaveGoal;

int main (int argc, char ** argv)
{

	ros::init (argc, argv, "BanditActionControlClient");

	actionlib::SimpleActionClient<_WaveAction> ac("BanditActionControlServer", true);

	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("Action server started, sending goal");

	_WaveGoal waveGoal;
	waveGoal.arm_id = waveGoal.ARM_ID_RIGHT;
	waveGoal.gesture_duration = ros::Duration(5);

	while (true)
	{
		ac.sendGoal(waveGoal);

		if (ac.waitForResult(ros::Duration(30.0)))
			ROS_INFO("Action finished: %s", ac.getState().toString().c_str());
		else
			ROS_INFO("Action did not finish before the time out.");
	}

	return 0;
}

// WaveGoal waveGoal;
// waveGoal. ...
// actionClient.sendGoal(waveGoal);
