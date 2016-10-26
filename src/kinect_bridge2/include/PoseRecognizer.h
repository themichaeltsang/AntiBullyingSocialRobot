#ifndef KINECT_BRIDGE2_POSERECOGNIZER_H_
#define KINECT_BRIDGE2_POSERECOGNIZER_H_

#include <ctime>
//#include <ros/ros.h>
#include <sstream>
#include <tf/LinearMath/Transform.h>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <numeric>
#include <algorithm>
//#include <linestream>
//include <tf/transform_broadcaster.h>


namespace PoseRecognition
{

class PoseRecognizer{

	public:
		struct Joint { 
			Joint( std::string n = std::string(), std::string p = std::string() ) : name( n ), parent_name( p ) {}
			std::string name;
			std::string parent_name;
		};   

		struct JointNormalizedTransform {
		    JointNormalizedTransform( tf::Transform t = tf::Transform(), std::string n = std::string(), std::string p = std::string() ) : this_to_parent_tf( t.getRotation(), t.getOrigin().normalized() ), name( n ), parent_name( p ) {}
			tf::Transform this_to_parent_tf;
			std::string name;
			std::string parent_name;
		};
		
		PoseRecognizer(bool train = false, clock_t timer_start = clock(), std::vector<std::string> test_names = {"crossed_arms"}, std::string train_name = "");
		//void getLegPosePredictions();


		std::map<std::string, double> PredictCurrentPoses(std::map<std::string, tf::Transform> joint_transforms_map);
		void TrainReferencePose(std::map<std::string, tf::Transform> joint_transforms_map); //how about mirroring? disabling one side of the body?, specifying what joints to look at 


		std::map<std::string, double> PredictCurrentArmPoses(std::map<std::string, tf::Transform> joint_transforms_map);
		void TrainReferenceArmPoses(std::map<std::string, tf::Transform> joint_transforms_map); //how about mirroring? disabling one side of the body?, specifying what joints to look at 



	private:
		std::map<std::string, Joint> joint_map_; //this should be comprehensive
		std::vector<std::string> armpose_target_joint_names_; 
		std::map<std::string, std::map<std::string, std::map<std::string, std::pair<double, double> > > > poses_ref_tables_;
		std::vector<std::string> testing_pose_names_;
		std::string training_pose_name_;
		//std::map<std::string, std::vector<std::string>> root_joints_of_pose_;
		std::map<std::string, std::vector<std::string>> target_joints_of_pose_; 


		// struct Joint {
		//     Joint() : name(), parent_name() {}
		//     Joint( std::string n, std::string p ) : name( n ), parent_name( p ) {}
		// };

		//std::map<std::string, tf::Transform> transforms_map;

		PoseRecognizer::Joint const & findTopmostParentJoint( std::string const & joint_name, std::map<std::string,  PoseRecognizer::Joint> const & joint_map );
		tf::Transform lookupTranslationTransform( std::map<std::string,  PoseRecognizer::JointNormalizedTransform> const & armpose_transforms_map, std::string const & start, std::string & pose_name );

	};
}

#endif
