#include <perceptual_filter/PoseRecognizer.h>
#include <string>
//#include <tf/LinearMath/Transform.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <exception>
#include <queue>

using namespace PoseRecognition;

using namespace std;

#define PI 3.1415926535897


PoseRecognizer::PoseRecognizer(bool train, clock_t timer_start, vector<string> test_names, string train_name)
{
	//cout << "reached PoseRecognizer Constructor" << endl;

	testing_pose_names_ = test_names; //copy over pose names passed in as argument to local copy for access from other functions
	training_pose_name_ = train_name;

	armpose_target_joint_names_ = {"elbow_left","elbow_right","wrist_left","wrist_right"};


	// construct parent-child joint relations
	// joints can include other info as well, like joint confidence?
	PoseRecognizer::Joint headjoint("head", "");
	PoseRecognizer::Joint neckjoint("neck", "head");
	PoseRecognizer::Joint spineshoulderjoint("spine_shoulder", "neck");
	PoseRecognizer::Joint shoulderjoint_l("shoulder_left", "spine_shoulder");
	PoseRecognizer::Joint shoulderjoint_r("shoulder_right", "spine_shoulder");
	PoseRecognizer::Joint elbowjoint_l("elbow_left", "shoulder_left");
	PoseRecognizer::Joint elbowjoint_r("elbow_right", "shoulder_right");
	PoseRecognizer::Joint wristjoint_l("wrist_left", "elbow_left");
	PoseRecognizer::Joint wristjoint_r("wrist_right", "elbow_right");
	PoseRecognizer::Joint handjoint_l("hand_left", "wrist_left");
	PoseRecognizer::Joint handjoint_r("hand_right", "wrist_right");
	PoseRecognizer::Joint handtipjoint_l("handtip_left", "hand_left");
	PoseRecognizer::Joint handtipjoint_r("handtip_right", "hand_right");
	PoseRecognizer::Joint thumbjoint_l("thumb_left", "hand_left");
	PoseRecognizer::Joint thumbjoint_r("thumb_right", "hand_right");
	PoseRecognizer::Joint spinemidjoint("spine_mid", "spine_shoulder");
	PoseRecognizer::Joint spinebasejoint("spine_base", "spine_mid");
	PoseRecognizer::Joint hipjoint_l("hip_left", "spine_base");
	PoseRecognizer::Joint hipjoint_r("hip_right", "spine_base");
	PoseRecognizer::Joint kneejoint_l("knee_left", "hip_left");
	PoseRecognizer::Joint kneejoint_r("knee_right", "hip_right");
	PoseRecognizer::Joint anklejoint_l("ankle_left", "knee_left");
	PoseRecognizer::Joint anklejoint_r("ankle_right", "knee_right");
	PoseRecognizer::Joint footjoint_l("foot_left", "ankle_left");
	PoseRecognizer::Joint footjoint_r("foot_right", "ankle_right");


	//currently some redundancy here, oh well, 
	joint_map_["head"] = headjoint;
	joint_map_["neck"] = neckjoint;
	joint_map_["shoulder_left"] = shoulderjoint_l;
	joint_map_["shoulder_right"] = shoulderjoint_r;
	joint_map_["spine_shoulder"] = spineshoulderjoint;
	joint_map_["elbow_left"] = elbowjoint_l;
	joint_map_["elbow_right"] = elbowjoint_r;
	joint_map_["wrist_left"] = wristjoint_l;
	joint_map_["wrist_right"] = wristjoint_r;
	joint_map_["hand_left"] = handjoint_l;
	joint_map_["hand_right"] = handjoint_r;
	joint_map_["handtip_left"] = handtipjoint_l;
	joint_map_["handtip_right"] = handtipjoint_r;
	joint_map_["thumb_left"] = thumbjoint_l;
	joint_map_["thumb_right"] = thumbjoint_r;
	joint_map_["spine_mid"] = spinemidjoint;
	joint_map_["spine_base"] = spinebasejoint;
	joint_map_["hip_left"] = hipjoint_l;
	joint_map_["hip_right"] = hipjoint_r;
	joint_map_["knee_left"] = kneejoint_l;
	joint_map_["knee_right"] = kneejoint_r;
	joint_map_["ankle_left"] = anklejoint_l;
	joint_map_["ankle_right"] = anklejoint_r;
	joint_map_["foot_left"] = footjoint_l;
	joint_map_["foot_right"] = footjoint_r;


	//input pose -> root nodes

	// root_joints_of_pose_["shove"] = {"shoulder_left", "shoulder_right"};
	// root_joints_of_pose_["crossed_arms"] = {"shoulder_left", "shoulder_right"};
	// root_joints_of_pose_["showing_left_fist_elbow_up"] = {"spine_shoulder"};
	// root_joints_of_pose_["showing_right_fist_elbow_up"] = {"spine_shoulder"};


	target_joints_of_pose_["crossed_arms"] = {"elbow_left","elbow_right","wrist_left","wrist_right"};
	
	target_joints_of_pose_["showing_left_fist_elbow_up"] = {"shoulder_left","elbow_left","wrist_left"}; //hand closed
	target_joints_of_pose_["showing_right_fist_elbow_up"] = {"shoulder_right","elbow_right","wrist_right"}; //hand closed
	target_joints_of_pose_["showing_left_fist_elbow_down"] = {"shoulder_left","elbow_left","wrist_left"}; //hand closed
	target_joints_of_pose_["showing_right_fist_elbow_down"] = {"shoulder_right","elbow_right","wrist_right"}; //hand closed
	
	target_joints_of_pose_["showing_left_fist"] = {"elbow_left","wrist_left"}; //hand closed
	target_joints_of_pose_["showing_right_fist"] = {"elbow_right","wrist_right"}; //hand closed
	

	target_joints_of_pose_["shoulders_up_closed_stance"] = {"shoulder_left","elbow_left","wrist_left","knee_left","ankle_left","shoulder_right","elbow_right","wrist_right","knee_right","ankle_right"};
	
	target_joints_of_pose_["left_arm_pointing"] = {"elbow_left","wrist_left"};//,"hand_left","handtip_left"};
	target_joints_of_pose_["right_arm_pointing"] = {"elbow_right","wrist_right"};//,"hand_right","handtip_right"};
	target_joints_of_pose_["left_hand_covering_mouth"] = {"elbow_left","wrist_left"};//,"hand_left","handtip_left"};
	target_joints_of_pose_["right_hand_covering_mouth"] = {"elbow_right","wrist_right"};//,"hand_right","handtip_right"};
	target_joints_of_pose_["hands_at_ears"] = {"elbow_left","wrist_left","elbow_right","wrist_right"}; //hands open


	target_joints_of_pose_["shove"] = {"elbow_left","elbow_right","wrist_left","wrist_right"}; //hand open
	target_joints_of_pose_["left_hit"] = {"elbow_left","wrist_left"}; //hand open or closed
	target_joints_of_pose_["right_hit"] = {"elbow_right","wrist_right"}; //hand open or closed
	target_joints_of_pose_["left_kick"] = {"knee_left","ankle_left"}; 
	target_joints_of_pose_["right_kick"] = {"knee_right","ankle_right"}; 
	target_joints_of_pose_["left_arm_vertical_swing"] = {"shoulder_left","elbow_left","wrist_left"}; //hand open or closed
	target_joints_of_pose_["right_arm_vertical_swing"] = {"shoulder_right","elbow_right","wrist_right"}; //hand open or closed
	target_joints_of_pose_["left_arm_horizontal_swing"] = {"shoulder_left","elbow_left","wrist_left"}; //hand open or closed
	target_joints_of_pose_["right_arm_horizontal_swing"] = {"shoulder_right","elbow_right","wrist_right"}; //hand open or closed

	target_joints_of_pose_["shove_preparation"] = {"elbow_left","elbow_right","wrist_left","wrist_right"}; //hand open
	target_joints_of_pose_["left_hit_preparation"] = {"elbow_left","wrist_left"}; //hand open or closed
	target_joints_of_pose_["right_hit_preparation"] = {"elbow_right","wrist_right"}; //hand open or closed
	target_joints_of_pose_["left_kick_preparation"] = {"knee_left","ankle_left"}; 
	target_joints_of_pose_["right_kick_preparation"] = {"knee_right","ankle_right"}; 
	target_joints_of_pose_["left_arm_swing_preparation"] = {"elbow_left","wrist_left"}; //hand open or closed
	target_joints_of_pose_["right_arm_swing_preparation"] = {"elbow_right","wrist_right"}; //hand open or closed

	target_joints_of_pose_["left_arm_vertical_swing_preparation"] = {"shoulder_left","elbow_left","wrist_left"}; //hand open or closed
	target_joints_of_pose_["right_arm_vertical_swing_preparation"] = {"shoulder_right","elbow_right","wrist_right"}; //hand open or closed
	target_joints_of_pose_["left_arm_horizontal_swing_preparation"] = {"shoulder_left","elbow_left","wrist_left"}; //hand open or closed
	target_joints_of_pose_["right_arm_horizontal_swing_preparation"] = {"shoulder_right","elbow_right","wrist_right"}; //hand open or closed
	target_joints_of_pose_["left_arm_diagonal_swing_preparation"] = {"shoulder_left","elbow_left","wrist_left"}; //hand open or closed
	target_joints_of_pose_["right_arm_diagonal_swing_preparation"] = {"shoulder_right","elbow_right","wrist_right"}; //hand open or closed

	std::vector<std::string> full_body_joints;
	for (auto const &joint : joint_map_)
	{
		full_body_joints.push_back(joint.first);

	}
	target_joints_of_pose_["full_body"] = full_body_joints;
	target_joints_of_pose_["aggressive_behaviors"] = {"ankle_left", "ankle_right", "knee_left", "knee_right", "hip_left", "hip_right", "handtip_left", "handtip_right", "hand_left", "hand_right", "wrist_left", "wrist_right", "elbow_left", "elbow_right", "shoulder_left", "shoulder_right"};
 
	//train = false; //TODO FIX
	// = true;
	if (train)
	{
		string filename = "pose_" + training_pose_name_ + ".train";
		ifstream infile(filename);
		if (!infile.good()) {//file doesn't exist:
			ofstream pose_file;
			pose_file.open(filename);

			auto target_joints = target_joints_of_pose_[training_pose_name_];

			int idx = 0;
			for (auto target_joint : target_joints){
				pose_file << target_joint + "_x,";
				pose_file << target_joint + "_y,";
				pose_file << target_joint + "_z";
				idx ++;
				if (idx != target_joints.size()) pose_file << ",";
			}
			pose_file << endl;
			pose_file.close();
		}
		else
			throw invalid_argument("pose training file already exists");

		//cout<< "timer - initializing training file: " << double(clock() - timer_start) / CLOCKS_PER_SEC << endl;
	}

	else
	{

		//calculate average and variance from the file 
		//put the values in poses_ref_tables_

		//load mean and stddev pose data from "armpose_crossed_arms.train"
		try{

			for (string pose_name: testing_pose_names_)
			{

				string filename = "pose_" + pose_name + ".train";
				ifstream data(filename);

				if (data.good())
				{

					string line;

					vector<vector<double>> pos_data;

					int linenum = 0;
					while(getline(data,line))
					{

						stringstream lineStream(line);
						//cout<< "attempting to load file 2" << endl;
						string cell;
						int cellnum = 0;
						while(getline(lineStream,cell,','))
						{
							if (linenum == 0)
							{
								vector<double> datavec;
								pos_data.push_back(datavec);
								//cout<< "attempting to load file 3" << endl;

							}
							else
							{
								//cout<< "attempting to load file 4" << endl;
								double cellval = atof(cell.c_str());
								//cout << cellval << endl;
								pos_data[cellnum].push_back(cellval);
							}
							cellnum++;

						}
						linenum++;
			    	}
					//cout<< "attempting to load file 5" << endl;

			    	int j = 0;
			    	for (int i = 0; i < pos_data.size(); i++)
			    	{

			    		vector<double> v = pos_data[i];
						//calculate mean
						double sum = accumulate(v.begin(), v.end(), 0.0);
						double mean = sum / v.size();
						//calculate stdev
						vector<double> diff(v.size());
						transform(v.begin(), v.end(), diff.begin(), bind2nd(minus<double>(), mean));
						double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
						double stdev = sqrt(sq_sum / v.size());

						pair<double, double> stats;
						stats = make_pair(mean,stdev);

			    		if (i % 3 == 0){
							map<string, pair<double, double> > pos_dimension;

							if (j == target_joints_of_pose_[pose_name].size())
							{
								cout << "ERROR in PoseRecognizer, j went out of bounds in joint array" << endl;
							}
							poses_ref_tables_[pose_name][ target_joints_of_pose_[pose_name][j] ] = pos_dimension;
							poses_ref_tables_[pose_name][ target_joints_of_pose_[pose_name][j] ]["x"] = stats;
						}
						if (i % 3 == 1){
							poses_ref_tables_[pose_name][ target_joints_of_pose_[pose_name][j] ]["y"] = stats;
						}
						if (i % 3 == 2){
							poses_ref_tables_[pose_name][ target_joints_of_pose_[pose_name][j] ]["z"] = stats;
							j++;
						}
					}
				}
				else
				{
					cout << "Error! Filename, " + filename + " does not exist" << endl;
				}

			}
		}
		catch (exception& e)
		{
			cout << "Error in pose recognizer reading test files" << endl;

		}

	   	//cout<< "timer - loading training file: " << double(clock() - timer_start) / CLOCKS_PER_SEC << endl;

	}

	
	//cout<< "timer - loading joint_map: " << double(clock() - timer_start) / CLOCKS_PER_SEC << endl;

	// map<string, joint> root_joint_map;
	// for( auto joint_it = joint_map_.cbegin(); joint_it != joint_map_.cend(); ++joint_it )
	// {
	// 	root_joint_map[joint_it->first] = findTopmostParentJoint( joint_it->first, joint_map_ );
	// }

}



double AngleBetweenTwoVectors(tf::Vector3 vectorA, tf::Vector3 vectorB)
{

	//double x1 = vectorA.x();
	double dotProduct;
	vectorA.normalize();
	vectorB.normalize();

	// cout << vectorA.x() << endl;
	// double x2 = vectorA.x();
	// assert(x1 != x2);

	dotProduct = vectorA.dot(vectorB);

	//cout << dotProduct << endl;
	if (dotProduct != dotProduct)
	{
		cout << "NAN error" << endl;
		cout << vectorA.x() << endl;
		cout << vectorA.y() << endl;
		cout << vectorA.z() << endl;
		cout << "" << endl;
		cout << vectorB.x() << endl;
		cout << vectorB.y() << endl;
		cout << vectorB.z() << endl;
		exit(0);
	}
	//cout << acos(dotProduct)/PI*180 << endl; //why -nan??
	return acos(dotProduct)/PI*180;
}


map<string, double> PoseRecognizer::GetNormalizedJointAngles(map<string, tf::Vector3> joint_positions_map, string context)
{

	map<string, double> joint_angles = map<string, double>();

	if (context == "aggressive_behaviors")
	{
		joint_angles["shoulder_right"] = AngleBetweenTwoVectors( joint_positions_map["shoulder_right"] -  joint_positions_map["spine_shoulder"],  joint_positions_map["shoulder_right"] -  joint_positions_map["elbow_right"]);
		joint_angles["elbow_right"] = AngleBetweenTwoVectors( joint_positions_map["elbow_right"] -  joint_positions_map["shoulder_right"],  joint_positions_map["elbow_right"] -  joint_positions_map["wrist_right"]);
		joint_angles["wrist_right"] = AngleBetweenTwoVectors( joint_positions_map["wrist_right"] -  joint_positions_map["elbow_right"],  joint_positions_map["wrist_right"] -  joint_positions_map["hand_right"]);
		joint_angles["hand_right"] = AngleBetweenTwoVectors( joint_positions_map["hand_right"] -  joint_positions_map["wrist_right"],  joint_positions_map["hand_right"] -  joint_positions_map["handtip_right"]);

		joint_angles["shoulder_left"] = AngleBetweenTwoVectors( joint_positions_map["shoulder_left"] -  joint_positions_map["spine_shoulder"],  joint_positions_map["shoulder_left"] -  joint_positions_map["elbow_left"]);
		joint_angles["elbow_left"] = AngleBetweenTwoVectors( joint_positions_map["elbow_left"] -  joint_positions_map["shoulder_left"],  joint_positions_map["elbow_left"] -  joint_positions_map["wrist_left"]);
		joint_angles["wrist_left"] = AngleBetweenTwoVectors( joint_positions_map["wrist_left"] -  joint_positions_map["elbow_left"],  joint_positions_map["wrist_left"] -  joint_positions_map["hand_left"]);
		joint_angles["hand_left"] = AngleBetweenTwoVectors( joint_positions_map["hand_left"] -  joint_positions_map["wrist_left"],  joint_positions_map["hand_left"] -  joint_positions_map["handtip_left"]);

		joint_angles["hip_right"] = AngleBetweenTwoVectors( joint_positions_map["hip_right"] -  joint_positions_map["spine_base"],  joint_positions_map["hip_right"] -  joint_positions_map["knee_right"]);
		joint_angles["knee_right"] = AngleBetweenTwoVectors( joint_positions_map["knee_right"] -  joint_positions_map["hip_right"],  joint_positions_map["knee_right"] -  joint_positions_map["ankle_right"]);
		joint_angles["ankle_right"] = AngleBetweenTwoVectors( joint_positions_map["ankle_right"] -  joint_positions_map["knee_right"],  joint_positions_map["ankle_right"] -  joint_positions_map["foot_right"]);

		joint_angles["hip_left"] = AngleBetweenTwoVectors( joint_positions_map["hip_left"] -  joint_positions_map["spine_base"],  joint_positions_map["hip_left"] -  joint_positions_map["knee_left"]);
		joint_angles["knee_left"] = AngleBetweenTwoVectors( joint_positions_map["knee_left"] -  joint_positions_map["hip_left"],  joint_positions_map["knee_left"] -  joint_positions_map["ankle_left"]);
		joint_angles["ankle_left"] = AngleBetweenTwoVectors( joint_positions_map["ankle_left"] -  joint_positions_map["knee_left"],  joint_positions_map["ankle_left"] -  joint_positions_map["foot_left"]);


		//wrist hand handtiphoulder_right
		//hand wrist elbow
		//wrist elbow shoulder
		//elbow shoulder spine 

		//ankle knee hip
		// knee hip gut
		//foot ankle knee

	}
	return joint_angles;
}


// TODO: Fix transforms implementation
// map<string, tf::Vector3> PoseRecognizer::GetNormalizedJointPositions(map<string, tf::Transform> joint_transforms_map, string context)
// {
// 	map<string, tf::Vector3> joint_positions; 

// 	map<string, PoseRecognizer::JointNormalizedTransform> observed_transforms_map;

// 	for( auto joint_it = joint_map_.cbegin(); joint_it != joint_map_.cend(); ++joint_it )
// 	{
// 		auto const & joint_msg = joint_it->second;
// 		auto const parent_joint_it = joint_map_.find( joint_msg.parent_name );
// 		auto parent_tf = tf::Transform( tf::Quaternion( 0, 0, 0, 1 ) );
// 		if( parent_joint_it != joint_map_.cend() )
// 		{
// 			auto const & parent_joint_msg = parent_joint_it->second;
// 			parent_tf = joint_transforms_map[parent_joint_msg.name].inverse() * joint_transforms_map[joint_msg.name]; //lookup transform... how? build a map 

//         }
//         PoseRecognizer::JointNormalizedTransform observed_joint( parent_tf, joint_msg.name, joint_msg.parent_name );
//         observed_transforms_map[joint_msg.name] = observed_joint;
//     }

//     std::string pose_name = context; //"full_body";
// 	auto const & target_joint_names = target_joints_of_pose_[pose_name];
// 	for(auto target_joint_name_it = target_joint_names.cbegin(); target_joint_name_it != target_joint_names.cend(); ++target_joint_name_it )
// 	{
// 		auto const & target_joint_name = *target_joint_name_it;
// 		tf::Transform observed_tf = lookupTranslationTransform( observed_transforms_map, target_joint_name, pose_name ); //need to specify end which is pose dependent 

// 		tf::Vector3 observed_origin = observed_tf.getOrigin(); 
		
// 		joint_positions[target_joint_name] = observed_origin;
// 	}
// 	cout <<  (joint_positions["head"] - joint_positions["spine_shoulder"]).length() << endl;
// 	return joint_positions;
// }


map<string, tf::Vector3> PoseRecognizer::GetNormalizedJointPositions(map<string, tf::Vector3> joint_positions_map, string root) // what is context
{
	map<string, vector<string>> jointTree;

	if (root == "head")
	{

		jointTree[ "head" ] = { "neck" };
		jointTree[ "neck" ] = { "spine_shoulder" };
		jointTree[ "spine_shoulder" ] = { "shoulder_left", "shoulder_right", "spine_mid" };
		jointTree[ "shoulder_left" ] = { "elbow_left" };
		jointTree[ "shoulder_right" ] = { "elbow_right" };
		jointTree[ "elbow_left" ] = { "wrist_left" };
		jointTree[ "elbow_right" ] = { "wrist_right" };
		jointTree[ "wrist_left" ] = { "hand_left" };
		jointTree[ "wrist_right" ] = { "hand_right" };
		jointTree[ "hand_left" ] = { "thumb_left", "handtip_left" };
		jointTree[ "hand_right" ] = { "thumb_right", "handtip_right" };
		jointTree[ "spine_mid" ] = { "spine_base" };
		jointTree[ "spine_base" ] = { "hip_left", "hip_right" };
		jointTree[ "hip_left" ] = { "knee_left" };
		jointTree[ "hip_right" ] = { "knee_right" };
		jointTree[ "knee_left" ] = { "ankle_left" };
		jointTree[ "knee_right" ] = { "ankle_right" };
		jointTree[ "ankle_left" ] = { "foot_left" };
		jointTree[ "ankle_right" ] = { "foot_right" };

	}

	queue<string> q;
	q.push("head");
	map<string, tf::Vector3> normalized_positions;

	while (!q.empty())
	{		
		string vertex = q.front();
		q.pop();
		if (joint_map_[vertex].parent_name != "")
		{
			tf::Vector3 normalized = (joint_positions_map[vertex] - joint_positions_map[joint_map_[vertex].parent_name]).normalized();
			normalized_positions[vertex] = normalized_positions[joint_map_[vertex].parent_name] + normalized;
		}
		else
		{
			normalized_positions[vertex] = tf::Vector3(0,0,0);
		} 

		if (jointTree.find(vertex)!=jointTree.end())
		{
			for (auto const& child : jointTree[vertex])
			{
				q.push(child);
			}
		}
	}

	return normalized_positions;
}




/*
	reference position @ 0


	given tree:

	class joint{
		joint.parent
		joint.children
		joint.position

	}



	def normalize_joint_position(joint, normalized_positions_map):
		if (not joint.children):
			#there are no children for this joint, determine it's position, then exot
			

		if (joint == root):
			norm_position = (0,0,0) #need to propogate this
		else:
			norm_position = normalized_positions_map[joint.parent] + (joint.position - joint.parent.position).norm.length

		normalized_positions_map[joint] = norm_position

		for child in joint.children:
			normalize_joint_position(child, normalized_positions_map)
			







*/




map<string, double> PoseRecognizer::PredictCurrentPoses(map<string, tf::Transform> joint_transforms_map)
{   
    map<string, double> predictions;

	map<string, PoseRecognizer::JointNormalizedTransform> observed_transforms_map;

	for( auto joint_it = joint_map_.cbegin(); joint_it != joint_map_.cend(); ++joint_it )
	{
		auto const & joint_msg = joint_it->second;
		auto const parent_joint_it = joint_map_.find( joint_msg.parent_name );
		auto parent_tf = tf::Transform( tf::Quaternion( 0, 0, 0, 1 ) );
		if( parent_joint_it != joint_map_.cend() )
		{
			auto const & parent_joint_msg = parent_joint_it->second;
			parent_tf = joint_transforms_map[parent_joint_msg.name].inverse() * joint_transforms_map[joint_msg.name]; //lookup transform... how? build a map 

        }
        PoseRecognizer::JointNormalizedTransform observed_joint( parent_tf, joint_msg.name, joint_msg.parent_name );
        observed_transforms_map[joint_msg.name] = observed_joint;
    }

    #pragma omp parallel for
	for (string pose_name: testing_pose_names_)
	{
		double sum = 0.0;
		double total = 0.0;

		//here is where I get a list of the pose 

		auto const & target_joint_names = target_joints_of_pose_[pose_name];
		for(auto target_joint_name_it = target_joint_names.cbegin(); target_joint_name_it != target_joint_names.cend(); ++target_joint_name_it )
		{
			auto const & target_joint_name = *target_joint_name_it;
			tf::Transform observed_tf = lookupTranslationTransform( observed_transforms_map, target_joint_name, pose_name ); //need to specify end which is pose dependent 

			tf::Vector3 observed_origin = observed_tf.getOrigin(); 

            double w1 = 1 / poses_ref_tables_[pose_name][target_joint_name]["x"].second; //stddev
            double w2 = 1 / poses_ref_tables_[pose_name][target_joint_name]["y"].second;
            double w3 = 1 / poses_ref_tables_[pose_name][target_joint_name]["z"].second;

			double fx = abs(observed_origin.x() - poses_ref_tables_[pose_name][target_joint_name]["x"].first); //avg
			double fy = abs(observed_origin.y() - poses_ref_tables_[pose_name][target_joint_name]["y"].first);
			double fz = abs(observed_origin.z() - poses_ref_tables_[pose_name][target_joint_name]["z"].first);
            
            fx *= w1;
            fy *= w2;
            fz *= w3;

			sum += fx;
			sum += fy;
			sum += fz;

			total += 3;
		}

    	#pragma omp critical
		predictions[pose_name] = sum / total;
	}

	return predictions;
}


void PoseRecognizer::TrainReferencePose(map<string, tf::Transform> joint_transforms_map)
{

	map<string, PoseRecognizer::JointNormalizedTransform> reference_transforms_map;
	for( auto joint_it = joint_map_.cbegin(); joint_it != joint_map_.cend(); ++joint_it )
	{
		auto const & joint_msg = joint_it->second;
		auto const parent_joint_it = joint_map_.find( joint_msg.parent_name );
		auto parent_tf = tf::Transform( tf::Quaternion( 0, 0, 0, 1 ) );
		if( parent_joint_it != joint_map_.cend() )
		{
			auto const & parent_joint_msg = parent_joint_it->second;
			parent_tf = joint_transforms_map[parent_joint_msg.name].inverse() * joint_transforms_map[joint_msg.name]; //lookup transform... how? build a map 
        }
		PoseRecognizer::JointNormalizedTransform reference_joint( parent_tf, joint_msg.name, joint_msg.parent_name );
        reference_transforms_map[joint_msg.name] = reference_joint;
    }

	map<string, tf::Vector3> reference_joint_positions;
	auto const & target_joint_names = target_joints_of_pose_[training_pose_name_];
	for(auto target_joint_name_it = target_joint_names.cbegin(); target_joint_name_it != target_joint_names.cend(); ++target_joint_name_it )
	{
		auto const & target_joint_name = *target_joint_name_it;
		tf::Transform reference_tf = lookupTranslationTransform( reference_transforms_map, target_joint_name, training_pose_name_ );

		tf::Vector3 reference_origin = reference_tf.getOrigin();
		reference_joint_positions[target_joint_name] = reference_origin;
	}


	//Append training data to file
	ofstream out_file;
	out_file.open("pose_" + training_pose_name_ + ".train", ios_base::app);
	int i = 0;
	for (auto ref_joint_pos_it = reference_joint_positions.cbegin(); ref_joint_pos_it != reference_joint_positions.cend(); ++ref_joint_pos_it)
	{
		auto const & ref_joint_pos = ref_joint_pos_it->second;
		//auto const & ref_joint_name = ref_joint_pos_it->first;
		if (i == 0)
		{
			out_file << ref_joint_pos.x() << "," << ref_joint_pos.y() << "," << ref_joint_pos.z();

		}
		else
		{
			out_file << "," << ref_joint_pos.x() << "," << ref_joint_pos.y() << "," << ref_joint_pos.z();
		}
		i++;
	}
	out_file << endl;
	out_file.close();

}

// climb our unit-humanoid "tree" from the start to the end (or topmost parent) joint, calculating the cumulative transform as we go
tf::Transform PoseRecognizer::lookupTranslationTransform( map<string,  PoseRecognizer::JointNormalizedTransform> const & transforms_map, string const & start, string & pose_name ) //, string const & end )
{
	// start with a zero transform
	tf::Transform cumulative_translation_tf( tf::Quaternion( 0, 0, 0, 1 ) );
	// initialize our search with the starting joint
	auto transform_it = transforms_map.find( start );
	// while the joint we're looking for exists
	while( transform_it != transforms_map.cend() ) //cend it at the head
	{
		// get a ref to the joint
		auto const & joint = transform_it->second;
// 		// if we've reached the given end joint, we're done traversing the tree
		if( joint.parent_name == "" )
		{
			//cout << "REACHED END OF CHAIN" << endl;
			break;
		}		
		// split up the current tf (which will necessarily be normalized) into its translation and rotation components
//            tf::Transform const & current_tf = joint.internal_transform_;
//            tf::Transform const current_rotation_tf = tf::Transform( current_tf.getRotation() );
//            tf::Transform const current_translation_tf = tf::Transform( tf::Quaternion( 0, 0, 0, 1 ), current_tf.getTranslation() );

		// transform the current joint's translation into the parent frame's coordinate frame using the inverse of the cumulative rotation transform
		// additionally, since the
//            cumulative_translation_tf = current_translation_tf * ( current_rotation_tf * cumulative_translation_tf );
		auto const & cumulative_translation = cumulative_translation_tf.getOrigin();
		auto const & current_translation = joint.this_to_parent_tf.getOrigin();

        // otherwise, update the cumulative translation tf
		cumulative_translation_tf *= joint.this_to_parent_tf;
		auto const & target_joints = target_joints_of_pose_[pose_name];

		if(find(target_joints.begin(), target_joints.end(), joint.parent_name) == target_joints.end()) {
    		break;
		}
        // and grab the parent joint
        transform_it = transforms_map.find( joint.parent_name );
	}
	return cumulative_translation_tf;
}



























map<string, double> PoseRecognizer::PredictCurrentArmPoses(map<string, tf::Transform> joint_transforms_map)
{   
    map<string, double> predictions;
	for (string pose_name: testing_pose_names_)
	{
		map<string, PoseRecognizer::JointNormalizedTransform> observed_armpose_transforms_map;

		// build our observed_armpose_transforms map from provided transforms
		for( auto joint_it = joint_map_.cbegin(); joint_it != joint_map_.cend(); ++joint_it )
		{
			auto const & joint_msg = joint_it->second;

			// the iterator for the parent joint
			auto const parent_joint_it = joint_map_.find( joint_msg.parent_name );

			auto parent_tf = tf::Transform( tf::Quaternion( 0, 0, 0, 1 ) );

	        // does the parent exist?
			if( parent_joint_it != joint_map_.cend() )
			{
				auto const & parent_joint_msg = parent_joint_it->second;

				// the "parent transform" is the transform from the child's start frame to the parent's end frame (ie it points back up to the parent,
				// whereas the "internal transform" points in the opposite direction)

				//f::Transform 

				parent_tf = joint_transforms_map[parent_joint_msg.name].inverse() * joint_transforms_map[joint_msg.name]; //lookup transform... how? build a map 

				// parent_tf = transforms_map[parent_joint_msg.end_frame_name].inverse() * transforms_map[joint_msg.start_frame_name];
				// parent_tf = _TfTranceiverPolicy::lookupTransform( joint_msg.start_frame_name, parent_joint_msg.end_frame_name );
	        }


	        //PRINT_INFO( "Adding entry to unit-humanoid: [ %s ] -> [ %s ]", joint_msg.name.c_str(), joint_msg.parent_name.c_str() );
	        PoseRecognizer::JointNormalizedTransform observed_joint( parent_tf, joint_msg.name, joint_msg.parent_name );
	        observed_armpose_transforms_map[joint_msg.name] = observed_joint;
	        //Joint( parent_tf, joint_msg.name, joint_msg.parent_name );
	    }

		//tf::Vector3 const variance_vec = tf::Vector3( 0.2, 0.2, 0.2 );
		

		//vector<string> armpose_target_joint_names_ = {"elbow_left","elbow_right","hand_left","hand_right"};
		
		//auto desired_joint_name_it = desired_joint_names.cbegin();
		//auto observed_joint_name_it = observed_joint_names.cbegin();

		// map<string, vector<tf::Vector3>> armpose_match_qualities;
		// vector<tf::Vector3> match_qualities;
		// armpose_match_qualities["crossed_arms"] = match_qualities;

		double sum = 0.0;
		//double total = 0.0;

		for(auto armpose_target_joint_name_it = armpose_target_joint_names_.cbegin(); armpose_target_joint_name_it != armpose_target_joint_names_.cend(); ++armpose_target_joint_name_it )
		{
			auto const & armpose_target_joint_name = *armpose_target_joint_name_it;
			tf::Transform observed_tf = lookupTranslationTransform( observed_armpose_transforms_map, armpose_target_joint_name, pose_name ); //need to specify end which is pose dependent 

			//tf::Transform reference_tf = lookupArmposeTranslationTransform( reference_armpose_transforms_map, armpose_target_joint_name );
			//Need this reference_tf

			//quality here is the number of standard deviations from the mean (along each axis)
			//armpose_match_qualities["crossed_arms"].push_back( ( observed_tf.getOrigin() - reference_tf.getOrigin() ) * tf::Vector3( 1/variance_vec.x(), 1/variance_vec.y(), 1/variance_vec.z() ) );
			

			//tf::Vector3 diff = observed_tf.getOrigin() - reference_tf.getOrigin();

			tf::Vector3 observed_origin = observed_tf.getOrigin(); 

            double w1 = 1 / poses_ref_tables_[pose_name][armpose_target_joint_name]["x"].second;
            double w2 = 1 / poses_ref_tables_[pose_name][armpose_target_joint_name]["y"].second;
            double w3 = 1 / poses_ref_tables_[pose_name][armpose_target_joint_name]["z"].second;

			double fx = abs(observed_origin.x() - poses_ref_tables_[pose_name][armpose_target_joint_name]["x"].first);
			double fy = abs(observed_origin.y() - poses_ref_tables_[pose_name][armpose_target_joint_name]["y"].first);
			double fz = abs(observed_origin.z() - poses_ref_tables_[pose_name][armpose_target_joint_name]["z"].first);
            
            fx *= w1;
            fy *= w2;
            fz *= w3;

			sum += fx;
			sum += fy;
			sum += fz;
		}


		predictions[pose_name] = sum;// /total;
	}

	return predictions;
}

// IMPORTANT: ONLY FOR TRAINING ARMPOSES
void PoseRecognizer::TrainReferenceArmPoses(map<string, tf::Transform> joint_transforms_map)
{

	map<string, PoseRecognizer::JointNormalizedTransform> reference_armpose_transforms_map;
	for( auto joint_it = joint_map_.cbegin(); joint_it != joint_map_.cend(); ++joint_it )
	{
		auto const & joint_msg = joint_it->second;
		auto const parent_joint_it = joint_map_.find( joint_msg.parent_name );
		auto parent_tf = tf::Transform( tf::Quaternion( 0, 0, 0, 1 ) );
		if( parent_joint_it != joint_map_.cend() )
		{
			auto const & parent_joint_msg = parent_joint_it->second;
			parent_tf = joint_transforms_map[parent_joint_msg.name].inverse() * joint_transforms_map[joint_msg.name]; //lookup transform... how? build a map 
        }
        //JointNormalizedTransform( parent_tf, joint_msg.name, joint_msg.parent_name );
		PoseRecognizer::JointNormalizedTransform reference_joint( parent_tf, joint_msg.name, joint_msg.parent_name );
        reference_armpose_transforms_map[joint_msg.name] = reference_joint;
    }
	//vector<string> armpose_target_joint_names_ = {"elbow_left","elbow_right","hand_left","hand_right"};
	
	//Change this to include other arm poses
	map<string, tf::Vector3> reference_joint_positions;

	for(auto armpose_target_joint_name_it = armpose_target_joint_names_.cbegin(); armpose_target_joint_name_it != armpose_target_joint_names_.cend(); ++armpose_target_joint_name_it )
	{
		auto const & armpose_target_joint_name = *armpose_target_joint_name_it;
		tf::Transform reference_tf = lookupTranslationTransform( reference_armpose_transforms_map, armpose_target_joint_name, training_pose_name_ );

		tf::Vector3 reference_origin = reference_tf.getOrigin();
		reference_joint_positions[armpose_target_joint_name] = reference_origin;

		//cout << reference_origin << endl;

		//cout << "r_x: " << reference_origin.x() << ", r_y: " << reference_origin.y() << ", r_z: " << reference_origin.z() << endl;
		// double r_x = reference_origin.x();
		// double r_y = reference_origin.y();
		// double r_z = reference_origin.z();
	}

	//Append training data to file
	ofstream armpose_out_file;
	armpose_out_file.open("armpose_" + training_pose_name_ + ".train", ios_base::app);
	int i = 0;
	for (auto ref_joint_pos_it = reference_joint_positions.cbegin(); ref_joint_pos_it != reference_joint_positions.cend(); ++ref_joint_pos_it)
	{
		auto const & ref_joint_pos = ref_joint_pos_it->second;
		//auto const & ref_joint_name = ref_joint_pos_it->first;
		if (i == 0)
		{
			armpose_out_file << ref_joint_pos.x() << "," << ref_joint_pos.y() << "," << ref_joint_pos.z();

		}
		else
		{
			armpose_out_file << "," << ref_joint_pos.x() << "," << ref_joint_pos.y() << "," << ref_joint_pos.z();
		}
		i++;
	}
	armpose_out_file << endl;
	armpose_out_file.close();

}



// this isn't used
PoseRecognizer::Joint const & PoseRecognizer::findTopmostParentJoint( string const & joint_name, map<string, PoseRecognizer::Joint> const & joint_map )
{
    //PRINT_INFO( "Looking for topmost parent joint of [ %s ]", joint_name.c_str() );
    auto current_joint_it = joint_map.find( joint_name );
    while( true )
    {
        auto const & joint = current_joint_it->second;
        auto new_joint_it = joint_map.find( joint.parent_name );

        if( new_joint_it == joint_map.cend() )
        {
            break;
        }

        current_joint_it = new_joint_it;
    }

    auto const & joint = current_joint_it->second;

   // PRINT_INFO( "Found [ %s ]", joint.name.c_str() );

    return joint;
}


// // is this restricted to armposes? 
// // climb our unit-humanoid "tree" from the start to the end (or topmost parent) joint, calculating the cumulative transform as we go
// tf::Transform PoseRecognizer::lookupTranslationTransform( map<string,  PoseRecognizer::JointNormalizedTransform> const & transforms_map, string const & start, string & pose_name ) //, string const & end )
// {
// 	//PRINT_INFO( "Looking up unit-humanoid transform [ %s ] -> [ %s ]", start.c_str(), end.c_str() );

// 	// start with a zero transform
// 	tf::Transform cumulative_translation_tf( tf::Quaternion( 0, 0, 0, 1 ) );

// 	// initialize our search with the starting joint
// 	auto transform_it = transforms_map.find( start );

// 	// while the joint we're looking for exists
// 	while( transform_it != transforms_map.cend() )
// 	{
// 		// get a ref to the joint
// 		auto const & joint = transform_it->second;

// 		// split up the current tf (which will necessarily be normalized) into its translation and rotation components
// //            tf::Transform const & current_tf = joint.internal_transform_;
// //            tf::Transform const current_rotation_tf = tf::Transform( current_tf.getRotation() );
// //            tf::Transform const current_translation_tf = tf::Transform( tf::Quaternion( 0, 0, 0, 1 ), current_tf.getTranslation() );

// 		// transform the current joint's translation into the parent frame's coordinate frame using the inverse of the cumulative rotation transform
// 		// additionally, since the
// //            cumulative_translation_tf = current_translation_tf * ( current_rotation_tf * cumulative_translation_tf );

// 		// if we've reached the given end joint, we're done traversing the tree
// 		// if( joint.parent_name == end )
// 		// {
// 		// 	//cout << "REACHED END OF CHAIN" << endl;
// 		// 	break;
// 		// }

// 		auto const & cumulative_translation = cumulative_translation_tf.getOrigin();
// 		auto const & current_translation = joint.this_to_parent_tf.getOrigin();
// 		//PRINT_INFO( "[ %s ] -> [ %s ] ( %f, %f, %f ) * ( %f, %f, %f )", joint.name_.c_str(), joint.parent_name_.c_str(), cumulative_translation.x(), cumulative_translation.y(), cumulative_translation.z(), current_translation.x(), current_translation.y(), current_translation.z() );

//         // otherwise, update the cumulative translation tf
// 		cumulative_translation_tf *= joint.this_to_parent_tf;
//         // and grab the parent joint
//         transform_it = transforms_map.find( joint.parent_name );
// 	}

// 	//auto const & cumulative_translation = cumulative_translation_tf.getOrigin();
// 	//PRINT_INFO( "Cumulative translation: ( %f, %f, %f )", cumulative_translation.x(), cumulative_translation.y(), cumulative_translation.z() );

// 	return cumulative_translation_tf;
// }

