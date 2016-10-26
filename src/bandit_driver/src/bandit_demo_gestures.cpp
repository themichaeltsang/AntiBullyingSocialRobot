#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <bandit_driver/Joint.h>
#include <bandit_driver/JointArray.h>
#include <bandit_driver/Params.h>
#include <math.h>
//#include <bandit/bandit.h>

using namespace std;
//bandit::Bandit c_bandit;
bandit_driver::JointArray * desired_joint_pos_arr;
ros::Publisher * joints_publisher;
ros::Subscriber * joints_subscriber;
ros::AsyncSpinner * spinner;

std::vector<double> * joint_positions;

double deg_to_rad(double deg)
{
	return deg * M_PI / 180;
}

double rad_to_deg(double rad)
{
	return rad * 180 / M_PI;
}

void publish_joints()
{
	joints_publisher->publish(*desired_joint_pos_arr);
}

void jointStatesCallBack(const sensor_msgs::JointStateConstPtr &js)
{
	*joint_positions = js->position;//pulls out all the positions for all joints and stores it to joint_positions
}

void publishSingleJoint(int id, double angle)
{
	desired_joint_pos_arr->joints.clear();
	bandit_driver::Joint desired_joint_pos;	
	desired_joint_pos_arr->joints.emplace_back( std::move( desired_joint_pos ) );
	desired_joint_pos_arr->joints[0].id = id;
	desired_joint_pos_arr->joints[0].angle = angle;
	publish_joints();
}

void bandit_nod()
{	
	ROS_INFO("nodding");
	double nod_deg = 20;

	for (int i_nod = 0; i_nod < 8; i_nod++){
		if (i_nod % 2 == 0){
			nod_deg = - nod_deg;
		}
		publishSingleJoint(0, deg_to_rad( nod_deg ));
		ros::Rate(5).sleep();
	}
	publishSingleJoint(0, deg_to_rad(0));
	publishSingleJoint(16, deg_to_rad(90)); //eyebrow?
	ros::Rate(1).sleep();
	publishSingleJoint(16, deg_to_rad(-90)); //eyebrow?
	//publishSingleJoint(17, deg_to_rad(45)); mouth
	//publishSingleJoint(18, deg_to_rad(45)); mouth
	ros::Rate(1).sleep();
	publishSingleJoint(16, deg_to_rad(0));

	for(int i = 0; i < 10; i++)
	{

		publishSingleJoint(17, deg_to_rad(90));
		publishSingleJoint(18, deg_to_rad(90));

		ros::Duration(0.25).sleep();

		publishSingleJoint(17, deg_to_rad(-90));
		publishSingleJoint(18, deg_to_rad(-90));

		ros::Duration(0.25).sleep();
	}

	publishSingleJoint(17, deg_to_rad(0));
	publishSingleJoint(18, deg_to_rad(0));
}

// void bandit_shake()
// {
// 	double shake_deg = 30;
// 	desired_joint_pos->id = 1;
// 	for (int i_shake = 0; i_shake < 8; i_shake++){
// 		if (i_shake % 2 == 0){
// 			shake_deg = - shake_deg;
// 		}
// 		desired_joint_pos->angle = deg_to_rad( shake_deg );
// 		publish_joint_ind();
// 		ros::Rate(5).sleep();
// 	}
// 	desired_joint_pos->angle = deg_to_rad(0);
// 	publish_joint_ind();
// 	ros::Rate(1).sleep();
// }

void bandit_wave()
{
	ROS_INFO("waving");

	publishSingleJoint(9, deg_to_rad(10));
	ros::Rate(10).sleep();
	
	publishSingleJoint(10, deg_to_rad(-5));
	ros::Rate(10).sleep();

	publishSingleJoint(12, deg_to_rad(45));
	ros::Rate(10).sleep();
	
	ros::Duration(0.5).sleep();

	
	double wave_deg = 0;
	for (int i_wave = 0; i_wave < 3; i_wave++){
		ROS_INFO("JOINT 12 Wave");
		if (i_wave % 2 == 0){
			wave_deg = -30;
		}
		else
			wave_deg = 30;

		publishSingleJoint(12, deg_to_rad( wave_deg ));
		ros::Rate(2).sleep();
	}
	
	for (int ii = 0; ii < 16; ii++){
		if (ii == 5){
			publishSingleJoint(ii, deg_to_rad(10));
		}
		else if (ii == 7 || ii == 8 || ii == 14 || ii == 15){
			continue;
		}
		else
			publishSingleJoint(ii, deg_to_rad(0));
		ros::Rate(10).sleep();
	}
}

// void bandit_comehere()
// {
// 	desired_joint_pos->id = 9;
// 	desired_joint_pos->angle = deg_to_rad(60);
// 	publish_joint_ind();
// 	ros::Rate(10).sleep();
	
// 	desired_joint_pos->id = 12;
// 	desired_joint_pos->angle = deg_to_rad(115);
// 	publish_joint_ind();
// 	ros::Rate(10).sleep();
	
// 	desired_joint_pos->id = 13;
// 	desired_joint_pos->angle = deg_to_rad(90);
// 	publish_joint_ind();
// 	ros::Rate(10).sleep();
	
// 	ros::Duration(0.5).sleep();

// 	double comehere_deg = 0;
// 	desired_joint_pos->id = 12;
// 	for (int i_comehere = 0; i_comehere < 8; i_comehere++){
// 		if (i_comehere % 2 == 0){
// 			comehere_deg = 90;
// 		}
// 		else
// 			comehere_deg = 115;
// 		desired_joint_pos->angle = deg_to_rad( comehere_deg );
// 		publish_joint_ind();
// 		ros::Rate(2).sleep();
// 	}
	
// 		for (int i_i = 0; i_i < 16; i_i++){
// 		desired_joint_pos->id = i_i;
// 		if (i_i == 5){
// 			desired_joint_pos->angle = deg_to_rad(10);
// 		}
// 		else if (i_i == 7 || i_i == 8 || i_i == 14 || i_i == 15){
// 			continue;
// 		}
// 		else
// 			desired_joint_pos->angle = deg_to_rad( 0 );
// 		publish_joint_ind();
// 		ros::Rate(10).sleep();
// 	}
// }

int main( int argc, char* argv[] )
{
	ROS_INFO( "started demo gestures" );

	ros::init(argc,argv,"bandit_demo_gestures");
	
	ros::NodeHandle n;
	
	joints_publisher = new ros::Publisher;
	*joints_publisher = n.advertise<bandit_driver::JointArray>("/bandit_driver/bandit_joint_array_cmd",10); //joint_ind
	
	// joint_subscriber = new ros::Subscriber;
	// *joint_subscriber = n.subscribe("joint_states", 10, jointStatesCallBack); //creates subscriber and subscribes to topic "joint_states"

	spinner = new ros::AsyncSpinner(2);//Use 2 threads
	ros::Rate(1).sleep();
	
	spinner->start();
	
	desired_joint_pos_arr = new bandit_driver::JointArray;
	
	//joint_positions = new std::vector<double>;

	//initialize the desired_joint_pos_arr
	// int NUM_JOINTS = 19;

	// desired_joint_pos_arr->joints.clear();
	// for (int i = 0; i < NUM_JOINTS; i++)
	// {
	// 	bandit_driver::Joint desired_joint_pos;	
	// 	desired_joint_pos_arr->joints.emplace_back( std::move( desired_joint_pos ) );
	// 	desired_joint_pos_arr->joints[i].id = i;
	// 	//desired_joint_pos_arr->joints[i].angle = 0;
	// }
	
	while(ros::ok()){
		bandit_wave();
		ros::Duration(1).sleep();
		bandit_nod();
		//ros::Duration(1).sleep();
		//bandit_comehere();
		//ros::Duration(1).sleep();
		//bandit_shake();
		//ros::Duration(1).sleep();
		ROS_INFO( "loop" );

	}
	
	return 0;
}
